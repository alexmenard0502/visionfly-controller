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

    // E58-style packet, independently implemented from the documented wire format.
    // https://github.com/popolony2k/maritaca-e88-controller
    // No throttle increase or automatic takeoff in this bench version.
    private func sendBench(_ flag: UInt8, connection c: NWConnection) {
        let r: UInt8 = 0x80, p: UInt8 = 0x80, t: UInt8 = 0, y: UInt8 = 0x80
        let id = session
        c.send(content: Data([0x66, r, p, t, y, flag, r ^ p ^ t ^ y, 0x99]),
               completion: .contentProcessed { [weak self] error in
            Task { @MainActor in
                guard let self = self, self.session == id else { return }
                if let error = error {
                    self.log("Bench send error: \(error)")
                    self.benchStatus = "Send failed — power off drone"
                    self.cancelBench()
                    self.stopHeartbeat()
                }
            }
        })
    }

    private func cancelBench() {
        benchID = UUID()
        benchTask?.cancel()
        benchTask = nil
        benchRunning = false
    }

    func emergencyStop() {
        cancelBench()
        stopHeartbeat()
        guard controlReady, let c = control else {
            benchStatus = "No control path — power off drone"
            return
        }
        sendBench(0x04, connection: c)
        sendBench(0x20, connection: c)
        benchStatus = "Stop/lock sent locally; verify motors stopped"
        log("Stop/lock requested; no aircraft acknowledgement")
    }

    func runBench(calibrate: Bool) {
        guard !benchRunning, videoReady, controlReady, heartbeatRunning,
              let c = control, let last = lastPacket,
              Date().timeIntervalSince(last) < 2 else { return }
        stopHeartbeat()
        let id = session
        let token = UUID()
        benchID = token
        benchRunning = true
        benchStatus = calibrate ? "Calibration pulse in progress" : "Unlock pulse in progress"
        log(benchStatus)
        benchTask = Task { @MainActor [weak self] in
            let count = calibrate ? 38 : 12
            for _ in 0..<count {
                guard let self = self, !Task.isCancelled, self.session == id,
                      self.benchID == token else { return }
                guard self.controlReady, self.videoReady,
                      let last = self.lastPacket, Date().timeIntervalSince(last) < 2 else {
                    self.emergencyStop()
                    return
                }
                self.sendBench(calibrate ? 0x80 : 0x40, connection: c)
                try? await Task.sleep(nanoseconds: 40_000_000)
            }
            guard let self = self, !Task.isCancelled, self.session == id,
                  self.benchID == token else { return }
            self.emergencyStop()
            self.benchStatus = calibrate
                ? "Calibration pulse complete; stop/lock sent — observe LEDs"
                : "Unlock pulse complete; stop/lock sent — verify motors stopped"
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
        guard videoReady, controlReady, !heartbeatRunning, !benchRunning, let c = control else { return }
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
        if benchRunning { emergencyStop() }
        cancelBench()
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
    @State private var confirmUnlock = false
    @Environment(\.scenePhase) private var scenePhase

    public var body: some View {
        NavigationStack {
            Form {
                Section("Safety — bench only") {
                    Text("Remove all propellers and secure the drone. The video request may change control mode. The optional idle heartbeat is unverified on this aircraft. Keep hands clear. V5 can briefly unlock motors at zero throttle. Keep clear and be ready to cut power. No takeoff or throttle increase is provided.")
                        .foregroundStyle(.red)
                    Toggle("All propellers removed", isOn: $propsRemoved)
                        .onChange(of: propsRemoved) { _, value in
                            if !value { drone.disconnect() }
                        }
                }
                Section("Live camera — bench only") {
                    TimelineView(.periodic(from: .now, by: 1)) { context in
                        let fresh = drone.lastFrame.map { context.date.timeIntervalSince($0) < 2 } ?? false
                        ZStack {
                            Color.black
                            if fresh, let image = drone.cameraImage {
                                Image(uiImage: image)
                                    .resizable()
                                    .scaledToFit()
                            } else {
                                Text(drone.frameCount == 0 ? "Waiting for JPEG frames" : "Video stalled")
                                    .foregroundStyle(.white)
                            }
                        }
                        .aspectRatio(4.0 / 3.0, contentMode: .fit)
                        .clipShape(RoundedRectangle(cornerRadius: 8))
                    }
                    LabeledContent("Decoded frames", value: "\(drone.frameCount)")
                    LabeledContent("JPEG decode failures", value: "\(drone.decodeFailures)")
                    if let image = drone.cameraImage, let cg = image.cgImage {
                        LabeledContent("Image pixels", value: "\(cg.width) × \(cg.height)")
                    }
                }
                Section("Network — 192.168.4.153") {
                    LabeledContent("Wi-Fi path", value: drone.wifiPath)
                    LabeledContent("Video UDP 8080", value: drone.videoState)
                    LabeledContent("Control UDP 8090", value: drone.controlState)
                    Button("Open UDP sockets") { drone.connect() }
                        .disabled(!propsRemoved || drone.benchRunning)
                    Button("Request video / Wi-Fi mode") { drone.requestVideo() }
                        .disabled(!propsRemoved || !drone.videoReady || drone.benchRunning)
                    Text(drone.videoSend).font(.footnote)
                    Button("Start 60-second idle heartbeat") { drone.startHeartbeat() }
                        .disabled(!propsRemoved || !drone.videoReady || !drone.controlReady || drone.heartbeatRunning || drone.benchRunning || drone.lastPacket == nil)
                    Button("Stop heartbeat") { drone.stopHeartbeat() }
                        .disabled(!drone.heartbeatRunning)
                    LabeledContent("Heartbeats sent locally", value: "\(drone.heartbeatCount)")
                }
                Section("Motor protocol — bench only") {
                    Text("Secure the aircraft, remove every propeller, and keep the power switch accessible. Restart the heartbeat manually before each test. Power-cycle the drone after the unlock test.")
                        .font(.footnote)
                    Button("Calibrate gyro (1.5 s)") { drone.runBench(calibrate: true) }
                        .disabled(!propsRemoved || !drone.heartbeatRunning || drone.benchRunning)
                    Button("Brief unlock (0.48 s)") { confirmUnlock = true }
                        .disabled(!propsRemoved || !drone.heartbeatRunning || drone.benchRunning)
                        .confirmationDialog("All propellers removed and drone secured?", isPresented: $confirmUnlock, titleVisibility: .visible) {
                            Button("Run unlock bench test", role: .destructive) {
                                if propsRemoved { drone.runBench(calibrate: false) }
                            }
                            Button("Cancel", role: .cancel) {}
                        }
                    Button("STOP / LOCK", role: .destructive) { drone.emergencyStop() }
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
                }
                Section("Network event log (newest first)") {
                    ForEach(Array(drone.events.enumerated()), id: \.offset) { item in
                        Text(item.element).font(.caption.monospaced())
                    }
                }
                Section("Limitations") {
                    Text("UDP readiness does not prove the aircraft receives control commands. Heartbeat send counts are not acknowledgements. JPEG decoding is experimental; missing or reordered packets may corrupt frames. No verified flight controls or failsafe. Do not fly this test.")
                        .font(.footnote)
                }
            }
            .navigationTitle("VisionFly Bench V5")
        }
        .onChange(of: scenePhase) { _, phase in
            switch phase {
            case .active: drone.appState = "Active"
            case .inactive: drone.appState = "Inactive"; if drone.benchRunning { drone.emergencyStop() }; drone.stopHeartbeat()
            case .background: drone.appState = "Background"; if drone.benchRunning { drone.emergencyStop() }; drone.stopHeartbeat()
            @unknown default: drone.appState = "Unknown"
            }
        }
    }
}
