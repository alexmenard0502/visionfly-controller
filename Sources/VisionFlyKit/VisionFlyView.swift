import SwiftUI
import Network
import Foundation
import UIKit

// Reassemble JPEG frames from consecutive UDP payloads. Resync on each SOI.
// The related WIFI_8K firmware may prepend an eight-byte TZH header.
struct JPEGAssembler {
    private var frame = Data()
    private var active = false
    private var previousFF = false
    private let limit = 2 * 1024 * 1024

    mutating func push(_ packet: Data) -> [Data] {
        let prefix = Array(packet.prefix(8))
        let hasHeader = prefix.count == 8 && prefix[4] == 0x54 && prefix[5] == 0x5A && prefix[6] == 0x48
        let payload = hasHeader ? packet.dropFirst(8) : packet.dropFirst(0)
        var completed: [Data] = []
        for byte in payload {
            if previousFF && byte == 0xD8 {
                frame = Data([0xFF, 0xD8])
                active = true
            } else if active {
                frame.append(byte)
                if previousFF && byte == 0xD9 {
                    completed.append(frame)
                    frame = Data()
                    active = false
                } else if frame.count > limit {
                    frame = Data()
                    active = false
                }
            }
            previousFF = byte == 0xFF
        }
        return completed
    }
}

@MainActor
final class DroneDiagnostics: ObservableObject {
    @Published var cameraImage: UIImage?
    @Published var frameCount = 0
    @Published var decodeFailures = 0
    @Published var lastFrame: Date?
    private var assembler = JPEGAssembler()
    @Published var videoState = "Not opened"
    @Published var controlState = "Not opened"
    @Published var packetCount = 0
    @Published var lastPacket: Date?
    @Published var videoSend = "No request sent"
    @Published var videoReady = false
    @Published var appState = "Active"
    @Published var wifiPath = "Not monitored"
    @Published var controlReady = false
    @Published var heartbeatRunning = false
    @Published var heartbeatCount = 0
    @Published var packetPrefix = "None"
    @Published var events: [String] = []
    private var monitor: NWPathMonitor?
    private var heartbeatTask: Task<Void, Never>?
    @Published private(set) var benchRunning = false
    @Published private(set) var benchStatus = "No motor command tested"
    private var benchTask: Task<Void, Never>?
    private var benchID = UUID()
    private var stopTask: Task<Void, Never>?
    @Published private(set) var stopRunning = false
    @Published private(set) var benchSent = 0
    @Published var includeCommandInChecksum = true
    private var activeChecksumIncludesCommand = true
    @Published private(set) var manualRunning = false
    private var leftStick = CGPoint.zero
    private var rightStick = CGPoint.zero

    // Full byte endpoints with exact center, or the original conservative ranges.
    // Actual aircraft response to endpoints is not yet verified.
    @Published var fullRangeBench = false
    @Published private(set) var manualChannels = "T 128 | Y 128 | P 128 | R 128"
    private var channelMin = [128, 128, 128, 128]
    private var channelMax = [128, 128, 128, 128]
    @Published private(set) var manualExtrema = "No manual test yet"
    private var activeFullRange = false
    @Published var controllerBenchMode = false
    @Published var steeringSensitivity = 40
    private var activeCenteredHold = false
    private var activeSteeringScale: CGFloat = 1
    @Published private(set) var preparing = false
    private var preparationTask: Task<Void, Never>?

    func cancelPreparation() {
        preparationTask?.cancel()
        preparationTask = nil
        preparing = false
    }

    // Connect once, request the existing video stream, then start the known
    // idle heartbeat. Never start manual motor control or auto-rearm.
    func prepareController() {
        guard !preparing, !benchRunning, !stopRunning, appState == "Active" else { return }
        connect()
        let id = session
        preparing = true
        preparationTask = Task { @MainActor [weak self] in
            let clock = ContinuousClock()
            let deadline = clock.now.advanced(by: .seconds(10))
            var requested = false
            while !Task.isCancelled && clock.now < deadline {
                guard let self = self, self.session == id, self.appState == "Active" else { return }
                if self.videoReady && self.controlReady {
                    if !requested {
                        requested = true
                        self.requestVideo()
                    }
                    if let last = self.lastPacket, Date().timeIntervalSince(last) < 1 {
                        self.startHeartbeat()
                        self.preparing = false
                        self.preparationTask = nil
                        self.log("Controller ready for explicit bench input")
                        return
                    }
                }
                try? await Task.sleep(nanoseconds: 100_000_000)
            }
            guard let self = self, !Task.isCancelled, self.session == id else { return }
            self.preparing = false
            self.preparationTask = nil
            self.log("Preparation timed out; check drone Wi-Fi and retry")
            self.benchStatus = "Connection not ready — check Wi-Fi and retry"
        }
    }

    private func axis(_ input: CGFloat, range: CGFloat) -> UInt8 {
        let x = max(-1, min(1, input))
        let span: CGFloat = activeFullRange ? (x < 0 ? 128 : 127) : range
        return UInt8(max(0, min(255, Int((128 + x * span).rounded()))))
    }

    private func recordManual(throttle: UInt8, yaw: UInt8, pitch: UInt8, roll: UInt8) {
        let values = [Int(throttle), Int(yaw), Int(pitch), Int(roll)]
        let labels = ["T", "Y", "P", "R"]
        let current = zip(labels, values).map { "\($0.0) \($0.1)" }.joined(separator: " | ")
        if manualChannels != current { manualChannels = current }
        for i in 0..<4 {
            channelMin[i] = min(channelMin[i], values[i])
            channelMax[i] = max(channelMax[i], values[i])
        }
        let extrema = (0..<4).map { "\(labels[$0]) \(channelMin[$0])–\(channelMax[$0])" }.joined(separator: " | ")
        if manualExtrema != extrema { manualExtrema = extrema }
    }

    func moveLeft(_ point: CGPoint) {
        if manualRunning {
            leftStick = point
            return
        }
        guard point.y < -0.15, !benchRunning, !stopRunning,
              heartbeatRunning, videoReady, controlReady, includeCommandInChecksum,
              appState == "Active", let c = control,
              let last = lastPacket, Date().timeIntervalSince(last) < 1 else { return }
        stopHeartbeat()
        activeChecksumIncludesCommand = true
        activeCenteredHold = controllerBenchMode
        activeFullRange = controllerBenchMode || fullRangeBench
        activeSteeringScale = controllerBenchMode ? CGFloat(steeringSensitivity) / 100 : 1
        channelMin = [128, 128, 128, 128]
        channelMax = [128, 128, 128, 128]
        manualChannels = "T 128 | Y 128 | P 128 | R 128"
        manualExtrema = "T 128–128 | Y 128–128 | P 128–128 | R 128–128"
        benchSent = 0
        leftStick = point
        rightStick = .zero
        let id = session
        let token = UUID()
        benchID = token
        benchRunning = true
        manualRunning = true
        benchStatus = activeCenteredHold
            ? "Centered hold bench — STOP to end"
            : "Manual bench — hold left stick; release to stop"
        log("Manual joystick bench started")
        benchTask = Task { @MainActor [weak self] in
            let clock = ContinuousClock()
            // Continuous portrait bench session; only legacy diagnostic mode
            // retains its 15-second elapsed-time limit. Link and lifecycle
            // checks remain active. This is not a verified flight failsafe.
            let deadline: ContinuousClock.Instant? = self?.activeCenteredHold == true
                ? nil : clock.now.advanced(by: .seconds(15))
            var preludeRemaining = 13
            while !Task.isCancelled {
                guard let self = self, !Task.isCancelled, self.session == id,
                      self.benchID == token, self.manualRunning else { return }
                guard self.videoReady, self.controlReady, self.appState == "Active",
                      (deadline.map { clock.now < $0 } ?? true),
                      let last = self.lastPacket,
                      Date().timeIntervalSince(last) < 1 else {
                    self.emergencyStop()
                    return
                }
                if preludeRemaining > 0 {
                    preludeRemaining -= 1
                    self.sendBench(0, throttle: 128, connection: c)
                } else {
                    let t = self.axis(-self.leftStick.y, range: 64)
                    let r = self.axis(self.rightStick.x * self.activeSteeringScale, range: 32)
                    let p = self.axis(-self.rightStick.y * self.activeSteeringScale, range: 32)
                    let y = self.axis(self.leftStick.x * self.activeSteeringScale, range: 32)
                    self.sendBench(0, throttle: t, roll: r, pitch: p, yaw: y, connection: c)
                    self.recordManual(throttle: t, yaw: y, pitch: p, roll: r)
                }
                try? await Task.sleep(nanoseconds: 40_000_000)
            }
            guard let self = self, !Task.isCancelled, self.session == id,
                  self.benchID == token else { return }
            self.emergencyStop()
        }
    }

    func releaseLeft() {
        if manualRunning {
            if activeCenteredHold { leftStick = .zero }
            else { emergencyStop() }
        }
    }

    func moveRight(_ point: CGPoint) {
        if manualRunning { rightStick = point }
    }

    func releaseRight() {
        rightStick = .zero
    }

    // E58-style packet, independently implemented from the documented wire format.
    // https://github.com/popolony2k/maritaca-e88-controller
    // Propeller-free bench diagnostics only; startup flags may spin motors.
    private func sendBench(_ flag: UInt8, throttle t: UInt8 = 0,
                           roll r: UInt8 = 0x80, pitch p: UInt8 = 0x80,
                           yaw y: UInt8 = 0x80, connection c: NWConnection) {
        let id = session
        let checksum = r ^ p ^ t ^ y ^ (activeChecksumIncludesCommand ? flag : 0)
        let packet = Data([0x66, r, p, t, y, flag, checksum, 0x99])
        c.send(content: packet,
               completion: .contentProcessed { [weak self] error in
            Task { @MainActor in
                guard let self = self, self.session == id else { return }
                if let error = error {
                    self.log("Bench send error: \(error)")
                    if self.benchRunning { self.emergencyStop() }
                    self.benchStatus = "Send failed — power off drone"
                    self.cancelBench()
                    self.stopHeartbeat()
                } else {
                    self.benchSent += 1
                }
            }
        })
    }

    private func cancelBench() {
        benchID = UUID()
        benchTask?.cancel()
        benchTask = nil
        benchRunning = false
        manualRunning = false
        activeCenteredHold = false
        manualChannels = "Idle — no manual packets"
        leftStick = .zero
        rightStick = .zero
    }

    func emergencyStop() {
        cancelPreparation()
        cancelBench()
        stopHeartbeat()
        guard controlReady, let c = control else {
            benchStatus = "No control path — power off drone"
            return
        }
        stopTask?.cancel()
        // WiFi_CAM documents emergency stop, not a separate lock flag.
        activeChecksumIncludesCommand = true
        sendBench(0x00, throttle: 0x00, connection: c)
        sendBench(0x04, connection: c)
        let id = session
        stopRunning = true
        benchStatus = "Stop requested — verify motors stopped; power off"
        log("Emergency stop requested; no aircraft acknowledgement")
        stopTask = Task { @MainActor [weak self] in
            for _ in 0..<24 {
                try? await Task.sleep(nanoseconds: 40_000_000)
                guard let self = self, !Task.isCancelled, self.session == id,
                      self.controlReady else { break }
                self.sendBench(0x00, throttle: 0x00, connection: c)
                self.sendBench(0x04, connection: c)
            }
            guard let self = self, !Task.isCancelled, self.session == id else { return }
            self.stopRunning = false
        }
    }

    func runBench(calibrate: Bool) {
        guard !benchRunning, videoReady, controlReady, heartbeatRunning,
              let c = control, let last = lastPacket,
              Date().timeIntervalSince(last) < 2 else { return }
        guard !stopRunning, appState == "Active",
              calibrate || includeCommandInChecksum else { return }
        stopHeartbeat()
        activeChecksumIncludesCommand = includeCommandInChecksum
        benchSent = 0
        let id = session
        let token = UUID()
        benchID = token
        benchRunning = true
        benchStatus = calibrate ? "Calibration pulse in progress" : "Joystick reproduction in progress"
        log(benchStatus + (activeChecksumIncludesCommand ? " / WiFi_CAM checksum" : " / axes-only checksum"))
        benchTask = Task { @MainActor [weak self] in
            // Hypothesis from HQ-8K observed behavior, not a capture of HQ-8K.
            // No explicit unlock/takeoff flags; bounded propeller-free test.
            let stages: [(UInt8, UInt8, Int, String)] = calibrate
                ? [(0x80, 0x00, 38, "Calibration")]
                : [(0x00, 0x80, 13, "Centered stick"), (0x00, 0xC0, 15, "Stick up"), (0x00, 0x00, 38, "Stick fully down")]
            let clock = ContinuousClock()
            let deadline = clock.now.advanced(by: .seconds(4))
            for (flag, throttle, count, label) in stages {
                guard let self = self, !Task.isCancelled, self.session == id,
                      self.benchID == token else { return }
                self.benchStatus = label + " — bench test"
                self.log(self.benchStatus)
                for _ in 0..<count {
                    guard !Task.isCancelled, self.session == id, self.benchID == token else { return }
                    guard self.controlReady, self.videoReady, self.appState == "Active",
                          clock.now < deadline,
                          let last = self.lastPacket, Date().timeIntervalSince(last) < 2 else {
                        self.emergencyStop()
                        return
                    }
                    self.sendBench(flag, throttle: throttle, connection: c)
                    try? await Task.sleep(nanoseconds: 40_000_000)
                }
            }
            guard let self = self, !Task.isCancelled, self.session == id,
                  self.benchID == token else { return }
            if calibrate {
                self.cancelBench()
                self.benchStatus = "Calibration pulse complete — observe LEDs"
            } else {
                self.emergencyStop()
            }
            // Require explicit user restart after every pulse; never auto-resume.
        }
    }
    private func log(_ message: String) {
        events.insert("\(Date().formatted(date: .omitted, time: .standard))  \(message)", at: 0)
        if events.count > 18 { events.removeLast() }
    }
    private var video: NWConnection?
    private var control: NWConnection?
    private var session = UUID()
    private var total = 0
    private var lastUpdate = Date.distantPast
    private let host = NWEndpoint.Host("192.168.4.153")

    func connect() {
        disconnect()
        let id = session
        events = []
        let m = NWPathMonitor(requiredInterfaceType: .wifi)
        monitor = m
        m.pathUpdateHandler = { [weak self] path in
            let summary = "\(path.status); Wi-Fi: \(path.usesInterfaceType(.wifi)); IPv4: \(path.supportsIPv4)"
            Task { @MainActor in
                guard let self = self, self.session == id else { return }
                self.wifiPath = summary
                self.log("Wi-Fi path: " + summary)
            }
        }
        m.start(queue: .main)
        videoState = "Opening…"
        controlState = "Opening…"
        let vp = NWParameters.udp
        vp.requiredInterfaceType = .wifi
        let cp = NWParameters.udp
        cp.requiredInterfaceType = .wifi
        let v = NWConnection(host: host, port: 8080, using: vp)
        let c = NWConnection(host: host, port: 8090, using: cp)
        video = v
        control = c
        v.stateUpdateHandler = { [weak self] state in
            Task { @MainActor in self?.update(state, videoPort: true, id: id) }
        }
        c.stateUpdateHandler = { [weak self] state in
            Task { @MainActor in self?.update(state, videoPort: false, id: id) }
        }
        v.start(queue: .main)
        c.start(queue: .main)
    }

    private func update(_ state: NWConnection.State, videoPort: Bool, id: UUID) {
        guard id == session else { return }
        let description: String
        switch state {
        case .setup: description = "Setup"
        case .preparing: description = "Preparing"
        case .ready:
            description = "Ready (local UDP path)"
            if videoPort && !videoReady {
                videoReady = true
                receive(id)
            }
        case .waiting(let error):
            description = "Waiting: \(error)"
            if videoPort { videoReady = false }
        case .failed(let error):
            description = "Failed: \(error)"
            if videoPort { videoReady = false }
        case .cancelled:
            description = "Cancelled"
            if videoPort { videoReady = false }
        @unknown default: description = "Unknown state"
        }
        if videoPort { videoState = description } else {
            controlState = description
            if case .ready = state { controlReady = true } else { controlReady = false }
        }
        if !videoReady || !controlReady {
            if benchRunning { emergencyStop() }
            stopHeartbeat()
        }
        log("UDP \(videoPort ? 8080 : 8090): " + description)
    }

    private var receiving = false
    private func receive(_ id: UUID) {
        guard id == session, !receiving, let v = video else { return }
        receiving = true
        v.receiveMessage { [weak self] data, _, _, error in
            Task { @MainActor in
                guard let self = self, self.session == id else { return }
                self.receiving = false
                if let data = data, !data.isEmpty {
                    self.total += 1
                    for jpeg in self.assembler.push(data) {
                        if let image = UIImage(data: jpeg) {
                            self.cameraImage = image
                            self.frameCount += 1
                            self.lastFrame = Date()
                        } else {
                            self.decodeFailures += 1
                        }
                    }
                    if self.total == 1 {
                        self.packetPrefix = data.prefix(16).map { String(format: "%02X", Int($0)) }.joined(separator: " ")
                        self.log("First video packet received")
                    }
                    let now = Date()
                    if now.timeIntervalSince(self.lastUpdate) >= 0.25 || self.total == 1 {
                        self.packetCount = self.total
                        self.lastPacket = now
                        self.lastUpdate = now
                    }
                }
                if let error = error {
                    self.videoState = "Receive error: \(error)"
                    self.videoReady = false
                    if self.benchRunning { self.emergencyStop() }
                    self.stopHeartbeat()
                    self.log("Video receive error: \(error)")
                } else {
                    self.receive(id)
                }
            }
        }
    }

    func requestVideo() {
        guard videoReady, let v = video else { return }
        let id = session
        videoSend = "Sending video request…"
        v.send(content: Data([0x42, 0x76]), completion: .contentProcessed { [weak self] error in
            Task { @MainActor in
                guard let self = self, self.session == id else { return }
                if let error = error { self.videoSend = "Send error: \(error)" }
                else { self.videoSend = "Request sent locally; check received packets"; self.log("Video request sent") }
            }
        })
    }

    // Idle heartbeat documented for the similar WIFI_8K_ hardware.
    // This is unverified on this aircraft: props must be removed.
    func startHeartbeat() {
        guard videoReady, controlReady, !heartbeatRunning, !benchRunning, !stopRunning, let c = control else { return }
        let id = session
        heartbeatRunning = true
        heartbeatCount = 0
        log("Idle heartbeat test started (60 sec maximum)")
        heartbeatTask = Task { @MainActor [weak self] in
            for _ in 0..<75 {
                guard let self = self, !Task.isCancelled, self.session == id,
                      self.videoReady, self.controlReady, self.heartbeatRunning else { return }
                c.send(content: Data([0xAA, 0x80, 0x80, 0x00, 0x80, 0x00, 0x80, 0x55]), completion: .contentProcessed { [weak self] error in
                    Task { @MainActor in
                        guard let self = self, self.session == id, self.heartbeatRunning else { return }
                        if let error = error {
                            self.log("Heartbeat send error: \(error)")
                            self.stopHeartbeat()
                        } else { self.heartbeatCount += 1 }
                    }
                })
                try? await Task.sleep(nanoseconds: 790_000_000)
            }
            guard let self = self, self.session == id, !Task.isCancelled else { return }
            self.stopHeartbeat()
        }
    }

    func stopHeartbeat() {
        if heartbeatRunning { log("Idle heartbeat stopped") }
        heartbeatRunning = false
        heartbeatTask?.cancel()
        heartbeatTask = nil
    }

    func disconnect() {
        cancelPreparation()
        if benchRunning { emergencyStop() }
        cancelBench()
        stopTask?.cancel()
        stopTask = nil
        stopRunning = false
        session = UUID()
        stopHeartbeat()
        monitor?.cancel()
        monitor = nil
        wifiPath = "Not monitored"
        controlReady = false
        heartbeatCount = 0
        benchStatus = "No motor command tested"
        packetPrefix = "None"
        assembler = JPEGAssembler()
        cameraImage = nil
        frameCount = 0
        decodeFailures = 0
        lastFrame = nil
        video?.cancel()
        control?.cancel()
        video = nil
        control = nil
        videoReady = false
        receiving = false
        total = 0
        packetCount = 0
        lastPacket = nil
        lastUpdate = .distantPast
        videoState = "Not opened"
        controlState = "Not opened"
        videoSend = "No request sent"
    }
}

public struct VisionFlyView: View {
    public init() {}
    @StateObject private var drone = DroneDiagnostics()
    @State private var propsRemoved = false
    @State private var confirmMotor = false
    @State private var joystickEnabled = false
    @State private var showController = false
    @AppStorage("visionfly.cameraQuarterTurns") private var cameraQuarterTurns = 1
    @Environment(\.scenePhase) private var scenePhase

    private var leftAvailable: Bool {
        propsRemoved && joystickEnabled && drone.includeCommandInChecksum &&
        (drone.manualRunning || (drone.heartbeatRunning && !drone.benchRunning && !drone.stopRunning))
    }

    private func exitController() {
        drone.cancelPreparation()
        if drone.benchRunning || drone.stopRunning { drone.emergencyStop() }
        else { drone.stopHeartbeat() }
        drone.controllerBenchMode = false
        joystickEnabled = false
        showController = false
    }

    private var controllerScreen: some View {
        GeometryReader { geo in
            let stickSize = min(geo.size.width * 0.43, geo.size.height * 0.27, 290)
            VStack(spacing: 8) {
                HStack {
                    Button("Diagnostics") { exitController() }
                    Spacer()
                    Text("VisionFly V13").font(.headline)
                    Spacer()
                    Button {
                        cameraQuarterTurns = (cameraQuarterTurns + 1) % 4
                    } label: {
                        Image(systemName: "rotate.right").frame(minWidth: 44, minHeight: 44)
                    }
                    .accessibilityLabel("Rotate camera right")
                }
                .frame(height: 44)
                HStack(spacing: 10) {
                    Button(drone.preparing ? "Connecting…" : "Connect & prepare") {
                        if propsRemoved { joystickEnabled = true; drone.prepareController() }
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(!propsRemoved || drone.preparing || drone.benchRunning || drone.stopRunning)
                    Picker("Steering", selection: $drone.steeringSensitivity) {
                        Text("Low").tag(40)
                        Text("Med").tag(70)
                        Text("High").tag(100)
                    }
                    .pickerStyle(.segmented)
                    .disabled(drone.benchRunning || drone.stopRunning)
                }
                .frame(height: 44)
                TimelineView(.periodic(from: .now, by: 0.5)) { context in
                    let recent = drone.lastPacket.map { context.date.timeIntervalSince($0) < 1 } ?? false
                    let ready = drone.videoReady && drone.controlReady && recent
                    Text(ready ? (drone.manualRunning ? "Commands active • no aircraft ACK" : (drone.heartbeatRunning ? "Ready — push left stick up" : "Connected — prepare again to start")) : "Link not ready / stale video")
                        .font(.caption.bold())
                        .foregroundStyle(ready ? Color.green : Color.orange)
                        .lineLimit(1)
                        .minimumScaleFactor(0.75)
                        .frame(maxWidth: .infinity)
                }
                .frame(height: 24)
                // Flexible camera viewport: the fixed controls and stop button
                // reserve their own space; video uses only the remainder.
                GeometryReader { camera in
                    TimelineView(.periodic(from: .now, by: 0.5)) { context in
                        let fresh = drone.lastFrame.map { context.date.timeIntervalSince($0) < 2 } ?? false
                        ZStack {
                            Color.black
                            if fresh, let image = drone.cameraImage {
                                let sideways = cameraQuarterTurns % 2 != 0
                                Image(uiImage: image)
                                    .resizable()
                                    .scaledToFit()
                                    .frame(width: sideways ? camera.size.height : camera.size.width,
                                           height: sideways ? camera.size.width : camera.size.height)
                                    .rotationEffect(.degrees(Double(cameraQuarterTurns) * 90))
                                    .position(x: camera.size.width / 2, y: camera.size.height / 2)
                            } else {
                                Text("No fresh video").foregroundStyle(.white)
                            }
                        }
                    }
                }
                .frame(maxWidth: .infinity, maxHeight: .infinity)
                .clipShape(RoundedRectangle(cornerRadius: 10))
                .overlay {
                    RoundedRectangle(cornerRadius: 10)
                        .strokeBorder(Color.white.opacity(0.18))
                }
                .allowsHitTesting(false)
                HStack(alignment: .center, spacing: 8) {
                    BenchStick(title: "Throttle / Yaw", top: "Climb", bottom: "Descend",
                               enabled: leftAvailable, moved: { drone.moveLeft($0) },
                               released: { drone.releaseLeft() })
                        .frame(width: stickSize)
                    Spacer(minLength: 0)
                    BenchStick(title: "Pitch / Roll", top: "Forward", bottom: "Back",
                               enabled: propsRemoved && joystickEnabled && drone.manualRunning,
                               moved: { drone.moveRight($0) }, released: { drone.releaseRight() })
                        .frame(width: stickSize)
                }
                .frame(height: stickSize + 54)
                .padding(.horizontal, 4)
                Button(role: .destructive) { drone.emergencyStop() } label: {
                    Text("STOP MOTORS")
                        .font(.headline)
                        .frame(maxWidth: .infinity, minHeight: 44)
                }
                .buttonStyle(.borderedProminent)
                .tint(.red)
                .disabled(!drone.controlReady)
                VStack(spacing: 2) {
                    Text("BENCH ONLY • NO VERIFIED FAILSAFE")
                        .font(.caption2.bold())
                        .foregroundStyle(.yellow)
                    Text(drone.manualRunning ? "Continuous bench • " + drone.manualChannels : drone.benchStatus)
                        .font(.caption2)
                        .lineLimit(1)
                        .minimumScaleFactor(0.7)
                }
                .frame(height: 34)
            }
            .padding(12)
            .foregroundStyle(.white)
            .background(Color(red: 0.07, green: 0.09, blue: 0.12))
            .environment(\.colorScheme, .dark)
        }
    }

    public var body: some View {
        NavigationStack {
            if showController {
                controllerScreen
                    .toolbar(.hidden, for: .navigationBar)
            } else {
            Form {
                Section("Safety — bench only") {
                    Text("Remove all propellers and secure the drone. The video request may change control mode. The optional idle heartbeat is unverified on this aircraft. Keep hands clear. V13 removes the elapsed-time limit in the centered-hold controller screen; link and lifecycle stop requests still apply. In diagnostics mode, release the left stick to request shutdown; in controller mode, press STOP MOTORS. NEVER fit propellers for this test. Secure the drone and keep hands clear; be ready to cut power.")
                        .foregroundStyle(.red)
                    Toggle("All propellers removed", isOn: $propsRemoved)
                        .onChange(of: propsRemoved) { _, value in
                            if !value {
                                joystickEnabled = false
                                if drone.benchRunning || drone.stopRunning { drone.emergencyStop() } else { drone.disconnect() }
                            }
                        }
                }
                Section("Controller layout") {
                    Button("Open portrait controller") {
                        guard propsRemoved else { return }
                        drone.controllerBenchMode = true
                        joystickEnabled = true
                        showController = true
                    }
                    .disabled(!propsRemoved || drone.benchRunning || drone.stopRunning)
                    Text("Hold the iPad vertically. Video and controls share one screen without scrolling. Centered sticks keep sending neutral commands without an elapsed-time limit after startup; press STOP MOTORS to end. A stale video stream or app interruption can still request immediate motor shutdown. No verified aircraft-side failsafe. Propellers must remain removed.")
                        .font(.footnote)
                }
                Section("Live camera — bench only") {
                    TimelineView(.periodic(from: .now, by: 1)) { context in
                        let fresh = drone.lastFrame.map { context.date.timeIntervalSince($0) < 2 } ?? false
                        ZStack {
                            Color.black
                            if fresh, let image = drone.cameraImage {
                                GeometryReader { geo in
                                    let sideways = cameraQuarterTurns % 2 != 0
                                    Image(uiImage: image)
                                        .resizable()
                                        .scaledToFit()
                                        .frame(width: sideways ? geo.size.height : geo.size.width,
                                               height: sideways ? geo.size.width : geo.size.height)
                                        .rotationEffect(.degrees(Double(cameraQuarterTurns) * 90))
                                        .position(x: geo.size.width / 2, y: geo.size.height / 2)
                                }
                            } else {
                                Text(drone.frameCount == 0 ? "Waiting for JPEG frames" : "Video stalled")
                                    .foregroundStyle(.white)
                            }
                        }
                        .aspectRatio(4.0 / 3.0, contentMode: .fit)
                        .clipShape(RoundedRectangle(cornerRadius: 8))
                    }
                    HStack {
                        Button {
                            cameraQuarterTurns = (cameraQuarterTurns + 3) % 4
                        } label: {
                            Label("Rotate left", systemImage: "rotate.left")
                        }
                        .buttonStyle(.borderless)
                        Spacer()
                        Button {
                            cameraQuarterTurns = (cameraQuarterTurns + 1) % 4
                        } label: {
                            Label("Rotate right", systemImage: "rotate.right")
                        }
                        .buttonStyle(.borderless)
                    }
                    LabeledContent("Camera rotation", value: "\(cameraQuarterTurns * 90)°")
                    LabeledContent("Decoded frames", value: "\(drone.frameCount)")
                    LabeledContent("JPEG decode failures", value: "\(drone.decodeFailures)")
                    if let image = drone.cameraImage, let cg = image.cgImage {
                        LabeledContent("Image pixels", value: "\(cg.width) × \(cg.height)")
                    }
                }
                Section("Manual joysticks — bench only") {
                    Toggle("Enable manual bench controls", isOn: $joystickEnabled)
                        .disabled(!propsRemoved)
                        .onChange(of: joystickEnabled) { _, enabled in
                            if !enabled && drone.manualRunning { drone.emergencyStop() }
                        }
                    Text("Start the heartbeat below, then hold the left stick up to start. Keep it held while testing the right stick. Release the left stick to stop. Each attempt has a 15-second maximum; restart the heartbeat before another attempt. Directions are unverified. Use full range only for brief endpoint checks with propellers removed.")
                        .font(.footnote)
                    Toggle("Enable full 0–255 command range", isOn: $drone.fullRangeBench)
                        .disabled(!propsRemoved || !joystickEnabled || drone.benchRunning || drone.stopRunning)
                    LabeledContent("Manual command bytes", value: drone.manualChannels)
                        .font(.caption.monospaced())
                    LabeledContent("Last run min–max", value: drone.manualExtrema)
                        .font(.caption.monospaced())
                    Text("Values are generated command bytes, not aircraft telemetry or measured motor RPM.")
                        .font(.caption)
                    HStack(alignment: .top, spacing: 16) {
                        BenchStick(title: "Throttle / Yaw", top: "Climb", bottom: "Descend",
                                   enabled: leftAvailable, moved: { drone.moveLeft($0) },
                                   released: { drone.releaseLeft() })
                        BenchStick(title: "Pitch / Roll", top: "Forward", bottom: "Back",
                                   enabled: propsRemoved && joystickEnabled && drone.manualRunning,
                                   moved: { drone.moveRight($0) }, released: { drone.releaseRight() })
                    }
                    Button("EMERGENCY STOP", role: .destructive) { drone.emergencyStop() }
                        .disabled(!drone.controlReady)
                    Text(drone.benchStatus).font(.footnote)
                }
                Section("Network — 192.168.4.153") {
                    LabeledContent("Wi-Fi path", value: drone.wifiPath)
                    LabeledContent("Video UDP 8080", value: drone.videoState)
                    LabeledContent("Control UDP 8090", value: drone.controlState)
                    Button("Open UDP sockets") { drone.connect() }
                        .disabled(!propsRemoved || drone.benchRunning || drone.stopRunning)
                    Button("Request video / Wi-Fi mode") { drone.requestVideo() }
                        .disabled(!propsRemoved || !drone.videoReady || drone.benchRunning || drone.stopRunning)
                    Text(drone.videoSend).font(.footnote)
                    Button("Start 60-second idle heartbeat") { drone.startHeartbeat() }
                        .disabled(!propsRemoved || !drone.videoReady || !drone.controlReady || drone.heartbeatRunning || drone.benchRunning || drone.stopRunning || drone.lastPacket == nil)
                    Button("Stop heartbeat") { drone.stopHeartbeat() }
                        .disabled(!drone.heartbeatRunning)
                    LabeledContent("Heartbeats sent locally", value: "\(drone.heartbeatCount)")
                }
                Section("Motor protocol — bench only") {
                    Text("Secure the aircraft, remove every propeller, and keep the power switch accessible. Restart the heartbeat manually before each test. The joystick test starts centered, briefly pushes up, then holds down and requests stop. Leave WiFi_CAM checksum enabled. Power off after the motor test.")
                        .font(.footnote)
                    Toggle("WiFi_CAM checksum (include command)", isOn: $drone.includeCommandInChecksum)
                        .disabled(drone.benchRunning)
                    LabeledContent("Bench packets sent locally", value: "\(drone.benchSent)")
                    Button("Calibrate gyro (1.5 s)") { drone.runBench(calibrate: true) }
                        .disabled(!propsRemoved || !drone.heartbeatRunning || drone.benchRunning)
                    Button("Joystick up/down bench test (2.6 s)", role: .destructive) { confirmMotor = true }
                        .disabled(!propsRemoved || !drone.heartbeatRunning || drone.benchRunning || drone.stopRunning || !drone.includeCommandInChecksum)
                        .confirmationDialog("All propellers removed, drone secured, power switch accessible?", isPresented: $confirmMotor, titleVisibility: .visible) {
                            Button("Run joystick up/down bench test", role: .destructive) {
                                if propsRemoved { drone.runBench(calibrate: false) }
                            }
                            Button("Cancel", role: .cancel) {}
                        }
                    Button("EMERGENCY STOP", role: .destructive) { drone.emergencyStop() }
                        .disabled(!drone.controlReady)
                    Text(drone.benchStatus).font(.footnote)
                    Text("Stop delivery is not guaranteed. If motors keep turning, disconnect drone power safely. No response does not prove the protocol is wrong.")
                        .font(.footnote)
                }
                Section("Incoming data") {
                    LabeledContent("Video packets", value: "\(drone.packetCount)")
                    LabeledContent("First packet prefix", value: drone.packetPrefix).font(.caption.monospaced())
                    if let last = drone.lastPacket {
                        TimelineView(.periodic(from: .now, by: 1)) { context in
                            LabeledContent("Last video packet", value: "\(max(0, Int(context.date.timeIntervalSince(last)))) sec ago")
                        }
                    } else {
                        LabeledContent("Last video packet", value: "None")
                    }
                    LabeledContent("App state", value: drone.appState)
                    Button("Disconnect") { drone.disconnect() }
                        .disabled(drone.benchRunning || drone.stopRunning)
                }
                Section("Network event log (newest first)") {
                    ForEach(Array(drone.events.enumerated()), id: \.offset) { item in
                        Text(item.element).font(.caption.monospaced())
                    }
                }
                Section("Limitations") {
                    Text("The portrait bench session has no fixed time limit, but one second without incoming video, a network error or an app interruption can trigger a motor stop request. Do not fly with propellers fitted. UDP readiness does not prove the aircraft receives control commands. Heartbeat send counts are not acknowledgements. JPEG decoding is experimental; missing or reordered packets may corrupt frames. No verified flight controls or failsafe. Motor-start and stop are experimental. Do not fly this test.")
                        .font(.footnote)
                }
            }
            .navigationTitle("VisionFly Bench V13")
            }
        }
        .onDisappear {
            drone.cancelPreparation()
            if drone.benchRunning { drone.emergencyStop() }
            drone.stopHeartbeat()
        }
        .onChange(of: scenePhase) { _, phase in
            switch phase {
            case .active: drone.appState = "Active"
            case .inactive: drone.cancelPreparation(); joystickEnabled = false; drone.appState = "Inactive"; if drone.benchRunning { drone.emergencyStop() }; drone.stopHeartbeat()
            case .background: drone.cancelPreparation(); joystickEnabled = false; drone.appState = "Background"; if drone.benchRunning { drone.emergencyStop() }; drone.stopHeartbeat()
            @unknown default: drone.appState = "Unknown"
            }
        }
    }
}


// A spring-centered touch control. The left release also ends the entire bench run.
private struct BenchStick: View {
    let title: String
    let top: String
    let bottom: String
    let enabled: Bool
    let moved: (CGPoint) -> Void
    let released: () -> Void
    @State private var offset = CGSize.zero
    @GestureState private var touching = false

    var body: some View {
        VStack(spacing: 4) {
            Text(title).font(.caption.bold())
            Text(top).font(.caption2)
            GeometryReader { geo in
                let side = min(geo.size.width, geo.size.height)
                let limit = max(1, side * 0.33)
                ZStack {
                    Circle().fill(Color.secondary.opacity(0.12))
                    Circle().strokeBorder(Color.secondary.opacity(0.5), lineWidth: 1)
                    Path { p in
                        p.move(to: CGPoint(x: geo.size.width / 2 - limit, y: geo.size.height / 2))
                        p.addLine(to: CGPoint(x: geo.size.width / 2 + limit, y: geo.size.height / 2))
                        p.move(to: CGPoint(x: geo.size.width / 2, y: geo.size.height / 2 - limit))
                        p.addLine(to: CGPoint(x: geo.size.width / 2, y: geo.size.height / 2 + limit))
                    }.stroke(Color.secondary.opacity(0.4), style: StrokeStyle(lineWidth: 1, dash: [4, 4]))
                    Circle().fill(enabled ? Color.accentColor : Color.secondary)
                        .frame(width: side * 0.30, height: side * 0.30)
                        .offset(offset)
                }
                .contentShape(Rectangle())
                .highPriorityGesture(
                    DragGesture(minimumDistance: 0)
                        .updating($touching) { _, state, _ in state = true }
                        .onChanged { value in
                            guard enabled else { return }
                            let dx = value.location.x - geo.size.width / 2
                            let dy = value.location.y - geo.size.height / 2
                            let distance = max(limit, (dx * dx + dy * dy).squareRoot())
                            let point = CGPoint(x: dx / distance, y: dy / distance)
                            offset = CGSize(width: point.x * limit, height: point.y * limit)
                            moved(point)
                        }
                        .onEnded { _ in offset = .zero; released() }
                )
            }
            .aspectRatio(1, contentMode: .fit)
            Text(bottom).font(.caption2)
        }
        .opacity(enabled ? 1 : 0.45)
        .onChange(of: touching) { _, active in
            if !active { offset = .zero; released() }
        }
        .onChange(of: enabled) { _, active in
            if !active { offset = .zero; released() }
        }
        .accessibilityLabel(title + " bench joystick")
    }
}
