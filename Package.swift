// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "VisionFlyKit",
    platforms: [.iOS(.v17)],
    products: [.library(name: "VisionFlyKit", targets: ["VisionFlyKit"])],
    targets: [.target(name: "VisionFlyKit")]
)
