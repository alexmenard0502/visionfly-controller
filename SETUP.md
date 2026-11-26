# VisionFlyKit — V4 camera diagnostics

Experimental Swift package for iPadOS 17 and later. Remove all propellers and secure the drone before testing. No unlock, takeoff, landing or motor commands are included. The video request can change control mode; the idle heartbeat and camera decoding are not verified for flight. Do not fly this prototype.

## iPad setup

1. Connect the iPad to internet-capable Wi-Fi.
2. Open the app playground and its sidebar. Choose + > Swift Package.
3. Enter https://github.com/alexmenard0502/visionfly-controller.git
4. If a branch requirement is offered, select main. If a semantic version is required, publish a release tagged 0.4.0 on main first.
5. Select the VisionFlyKit product and add it to the project.
6. Replace ContentView.swift with:

```swift
import SwiftUI
import VisionFlyKit

struct ContentView: View {
    var body: some View {
        VisionFlyView()
    }
}
```

Run the app, grant local network permission if prompted, and connect to drone Wi-Fi for bench testing. Open UDP sockets, request video, then start the idle heartbeat. Do not install propellers.

For updates, reconnect to internet-capable Wi-Fi and use the package update option in Swift Playground if available. Return to drone Wi-Fi for testing. The package is not a standalone installable iOS app. Full iPad compilation and hardware validation are still required.

[Apple package instructions](https://developer.apple.com/documentation/swift-playgrounds/add-a-swift-package)
