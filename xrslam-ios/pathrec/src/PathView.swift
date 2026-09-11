import UIKit

// A 3D point in the SLAM world frame.
// xrslam world is z-up: gravity = (0, 0, -9.80665), see
// xrslam/src/xrslam/core/initializer.cpp and estimation/preintegrator.cpp.
struct Vec3 {
    var x: Double
    var y: Double
    var z: Double
}

// Interactive 3D view of the recorded walk path.
//
// - Drag to rotate (horizontal = azimuth around z, vertical = elevation).
// - Pinch to zoom, double-tap to reset the view.
// - Ground grid lies on the x-y plane at the height of the first point.
// - Current position has a dashed drop-line down to the ground plane;
//   the axes triad at the start is R=x, G=y, B=z (B is up).
final class PathView: UIView {

    var points: [Vec3] = [] {
        didSet { setNeedsDisplay() }
    }

    private let defaultAzimuth: Double = 0.9
    private let defaultElevation: Double = 0.6
    private var azimuth: Double = 0.9
    private var elevation: Double = 0.6
    private var zoom: Double = 1.0

    // MARK: - Init

    override init(frame: CGRect) {
        super.init(frame: frame)
        commonInit()
    }

    required init?(coder: NSCoder) {
        super.init(coder: coder)
        commonInit()
    }

    private func commonInit() {
        isUserInteractionEnabled = true

        let pan = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        addGestureRecognizer(pan)

        let pinch = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        addGestureRecognizer(pinch)

        let doubleTap = UITapGestureRecognizer(target: self, action: #selector(handleDoubleTap(_:)))
        doubleTap.numberOfTapsRequired = 2
        addGestureRecognizer(doubleTap)
    }

    // MARK: - Gestures

    @objc private func handlePan(_ g: UIPanGestureRecognizer) {
        let t = g.translation(in: self)
        azimuth -= Double(t.x) * 0.01
        elevation += Double(t.y) * 0.01
        elevation = min(max(elevation, 0.05), 1.52)
        g.setTranslation(.zero, in: self)
        setNeedsDisplay()
    }

    @objc private func handlePinch(_ g: UIPinchGestureRecognizer) {
        zoom = min(max(zoom * Double(g.scale), 0.2), 5.0)
        g.scale = 1.0
        setNeedsDisplay()
    }

    @objc private func handleDoubleTap(_ g: UITapGestureRecognizer) {
        azimuth = defaultAzimuth
        elevation = defaultElevation
        zoom = 1.0
        setNeedsDisplay()
    }

    // MARK: - Projection

    private struct Projection {
        let azimuth: Double
        let elevation: Double
        let scale: Double
        let cx: Double
        let cy: Double
        let center: Vec3

        func viewSpace(_ p: Vec3) -> (x: Double, y: Double) {
            let dx = p.x - center.x
            let dy = p.y - center.y
            let dz = p.z - center.z
            let xr = cos(azimuth) * dx + sin(azimuth) * dy
            let yr = -sin(azimuth) * dx + cos(azimuth) * dy
            return (xr, dz * cos(elevation) - yr * sin(elevation))
        }

        func project(_ p: Vec3) -> CGPoint {
            let v = viewSpace(p)
            return CGPoint(x: CGFloat(cx + v.x * scale), y: CGFloat(cy - v.y * scale))
        }
    }

    private struct Scene {
        let projection: Projection
        let gridMinX: Double
        let gridMaxX: Double
        let gridMinY: Double
        let gridMaxY: Double
        let gridZ: Double
        let spacing: Double
    }

    private func makeScene() -> Scene? {
        guard !points.isEmpty else { return nil }

        var minX = points[0].x
        var maxX = points[0].x
        var minY = points[0].y
        var maxY = points[0].y
        var minZ = points[0].z
        var maxZ = points[0].z
        for p in points {
            minX = min(minX, p.x)
            maxX = max(maxX, p.x)
            minY = min(minY, p.y)
            maxY = max(maxY, p.y)
            minZ = min(minZ, p.z)
            maxZ = max(maxZ, p.z)
        }

        let margin = max(1.0, 0.15 * max(maxX - minX, maxY - minY))
        let gridZ = points[0].z
        let gMinX = minX - margin
        let gMaxX = maxX + margin
        let gMinY = minY - margin
        let gMaxY = maxY + margin

        let center = Vec3(x: (minX + maxX) / 2.0,
                          y: (minY + maxY) / 2.0,
                          z: (minZ + maxZ) / 2.0)

        // Fit the trajectory plus the grid corners into the view.
        var fitPoints = points
        fitPoints.append(Vec3(x: gMinX, y: gMinY, z: gridZ))
        fitPoints.append(Vec3(x: gMaxX, y: gMaxY, z: gridZ))
        fitPoints.append(Vec3(x: gMinX, y: gMaxY, z: gridZ))
        fitPoints.append(Vec3(x: gMaxX, y: gMinY, z: gridZ))

        let probe = Projection(azimuth: azimuth, elevation: elevation,
                               scale: 1.0, cx: 0.0, cy: 0.0, center: center)
        var minSX = Double.greatestFiniteMagnitude
        var maxSX = -Double.greatestFiniteMagnitude
        var minSY = Double.greatestFiniteMagnitude
        var maxSY = -Double.greatestFiniteMagnitude
        for p in fitPoints {
            let v = probe.viewSpace(p)
            minSX = min(minSX, v.x)
            maxSX = max(maxSX, v.x)
            minSY = min(minSY, v.y)
            maxSY = max(maxSY, v.y)
        }
        var spanX = maxSX - minSX
        var spanY = maxSY - minSY
        if spanX < 1.0 { spanX = 1.0 }
        if spanY < 1.0 { spanY = 1.0 }

        let pad = 0.82
        let scale = min(Double(bounds.width) * pad / spanX,
                        Double(bounds.height) * pad / spanY) * zoom
        let midSX = (minSX + maxSX) / 2.0
        let midSY = (minSY + maxSY) / 2.0
        let proj = Projection(azimuth: azimuth, elevation: elevation,
                              scale: scale,
                              cx: Double(bounds.midX) - midSX * scale,
                              cy: Double(bounds.midY) + midSY * scale,
                              center: center)

        // Grid spacing: nice numbers, at most ~8 lines per direction.
        let extent = max(gMaxX - gMinX, gMaxY - gMinY)
        let candidates: [Double] = [1, 2, 5, 10, 20, 50, 100]
        var spacing = candidates[candidates.count - 1]
        for c in candidates {
            if extent / c <= 8.0 {
                spacing = c
                break
            }
        }

        return Scene(projection: proj,
                     gridMinX: gMinX, gridMaxX: gMaxX,
                     gridMinY: gMinY, gridMaxY: gMaxY,
                     gridZ: gridZ, spacing: spacing)
    }

    // MARK: - Drawing

    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext() else { return }

        guard let scene = makeScene() else {
            let text = "no path yet" as NSString
            let attrs: [NSAttributedString.Key: Any] = [
                .font: UIFont.systemFont(ofSize: 13),
                .foregroundColor: UIColor.gray,
            ]
            let size = text.size(withAttributes: attrs)
            text.draw(at: CGPoint(x: bounds.midX - size.width / 2.0,
                                  y: bounds.midY - size.height / 2.0),
                      withAttributes: attrs)
            return
        }

        let proj = scene.projection
        drawGrid(ctx, scene: scene, proj: proj)
        drawAxes(ctx, proj: proj, origin: points[0],
                 length: max(scene.spacing * 0.6, 0.3))
        drawPath(ctx, proj: proj)
        drawCurrentMarker(ctx, scene: scene, proj: proj)
    }

    private func drawGrid(_ ctx: CGContext, scene: Scene, proj: Projection) {
        ctx.setStrokeColor(UIColor(white: 1.0, alpha: 0.15).cgColor)
        ctx.setLineWidth(1.0)
        ctx.beginPath()
        var x = (scene.gridMinX / scene.spacing).rounded(.up) * scene.spacing
        while x <= scene.gridMaxX {
            ctx.move(to: proj.project(Vec3(x: x, y: scene.gridMinY, z: scene.gridZ)))
            ctx.addLine(to: proj.project(Vec3(x: x, y: scene.gridMaxY, z: scene.gridZ)))
            x += scene.spacing
        }
        var y = (scene.gridMinY / scene.spacing).rounded(.up) * scene.spacing
        while y <= scene.gridMaxY {
            ctx.move(to: proj.project(Vec3(x: scene.gridMinX, y: y, z: scene.gridZ)))
            ctx.addLine(to: proj.project(Vec3(x: scene.gridMaxX, y: y, z: scene.gridZ)))
            y += scene.spacing
        }
        ctx.strokePath()
    }

    private func drawAxes(_ ctx: CGContext, proj: Projection, origin: Vec3, length: Double) {
        let axes: [(Vec3, UIColor)] = [
            (Vec3(x: origin.x + length, y: origin.y, z: origin.z), .systemRed),
            (Vec3(x: origin.x, y: origin.y + length, z: origin.z), .systemGreen),
            (Vec3(x: origin.x, y: origin.y, z: origin.z + length), .systemBlue),
        ]
        for (end, color) in axes {
            ctx.setStrokeColor(color.cgColor)
            ctx.setLineWidth(2.0)
            ctx.beginPath()
            ctx.move(to: proj.project(origin))
            ctx.addLine(to: proj.project(end))
            ctx.strokePath()
        }
    }

    private func drawPath(_ ctx: CGContext, proj: Projection) {
        guard points.count > 1 else { return }
        ctx.setStrokeColor(UIColor(red: 1.0, green: 0.6, blue: 0.1, alpha: 1.0).cgColor)
        ctx.setLineWidth(2.0)
        ctx.setLineJoin(.round)
        ctx.setLineCap(.round)
        ctx.beginPath()
        ctx.move(to: proj.project(points[0]))
        for p in points.dropFirst() {
            ctx.addLine(to: proj.project(p))
        }
        ctx.strokePath()
    }

    private func drawCurrentMarker(_ ctx: CGContext, scene: Scene, proj: Projection) {
        let current = points[points.count - 1]

        // Dashed drop-line to the ground plane.
        ctx.setLineDash(phase: 0, lengths: [4, 4])
        ctx.setStrokeColor(UIColor(white: 0.7, alpha: 0.7).cgColor)
        ctx.setLineWidth(1.0)
        ctx.beginPath()
        ctx.move(to: proj.project(current))
        ctx.addLine(to: proj.project(Vec3(x: current.x, y: current.y, z: scene.gridZ)))
        ctx.strokePath()
        ctx.setLineDash(phase: 0, lengths: [])

        drawDot(ctx, at: proj.project(points[0]), color: .systemGreen)
        if points.count > 1 {
            drawDot(ctx, at: proj.project(current), color: .systemRed)
        }
    }

    private func drawDot(_ ctx: CGContext, at p: CGPoint, color: UIColor) {
        ctx.setFillColor(color.cgColor)
        ctx.fillEllipse(in: CGRect(x: p.x - 5.0, y: p.y - 5.0, width: 10.0, height: 10.0))
    }
}
