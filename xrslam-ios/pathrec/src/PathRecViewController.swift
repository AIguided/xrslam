import UIKit
import AVFoundation

// Yellow feature dots drawn on top of the camera preview.
final class FeaturesOverlayView: UIView {
    var features: [CGPoint] = [] { didSet { setNeedsDisplay() } }
    var imageSize: CGSize = .zero { didSet { setNeedsDisplay() } }

    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext(),
              imageSize.width > 1, imageSize.height > 1 else { return }
        let sx = bounds.width / imageSize.width
        let sy = bounds.height / imageSize.height
        ctx.setStrokeColor(UIColor(red: 1.0, green: 1.0, blue: 0.2, alpha: 0.9).cgColor)
        ctx.setLineWidth(1.5)
        for p in features {
            let x = p.x * sx
            let y = p.y * sy
            ctx.strokeEllipse(in: CGRect(x: x - 3.0, y: y - 3.0, width: 6.0, height: 6.0))
        }
    }
}

// Minimal RD-VIO (XRSLAM) walk-path recorder.
//
// Drift-debugging notes (v0.2, 2026-09):
// - Engine is fed only while a session is running, and every Start creates a
//   fresh engine (matches the upstream visualizer app's Start/stopFlag flow).
// - Capture settings match upstream: fixed 30 fps + locked focus (0.835).
//   The device configs are calibrated for fixed focus; AF hunting hurts tracking.
// - Ground truth instruction from official docs: initialise by moving in a
//   curved trajectory slowly.
// - XRSLAM is odometry-only (no loop closure / relocalisation), so some drift
//   is expected; the rec_*.log sidecar records state changes and pose jumps
//   to separate "normal drift" from tracking failures.
final class PathRecViewController: UIViewController, CameraDelegate, MotionDelegate {

    // MARK: - Engine and sensors
    private var camera: Camera?
    private var motion: Motion?
    private var xrslam: XRSLAM?

    // MARK: - UI
    private let stateLabel = UILabel()
    private let imageView = UIImageView()
    private let overlay = FeaturesOverlayView()
    private let pathView = PathView()
    private let recordButton = UIButton(type: .system)
    private let resetButton = UIButton(type: .system)
    private let infoLabel = UILabel()
    private let saveLabel = UILabel()

    // MARK: - Session state
    private var sessionActive = false
    private var sampleLines: [String] = []
    private var logLines: [String] = []
    private var pathPoints: [Vec3] = []
    private var lastSampleTime: Double = 0
    private var lastStatLog: Double = 0
    private var lastFeatWarn: Double = 0
    private var sessionStartUptime: Double = 0
    private var loopPassIndex = 0
    private var distance: Double = 0
    private var recordStart: Date?
    private var lastPosition: (x: Double, y: Double, z: Double)?
    private var lastState: SysState = .SYS_UNKNOWN

    // Keyframes (for offline loop detection): 1 Hz raw JPEG + pose.
    private var sessionBase: String?
    private var keyframeFolder: String?
    private var keyframeSeq = 0
    private var keyframeCount = 0
    private var lastKeyframeTime: Double = 0
    private let keyframeQueue = DispatchQueue(label: "pathrec.keyframes", qos: .utility)

    // On-device loop closure (v0.5).
    private var rawTimes: [Double] = []
    private var rawXYZ: [Double] = []
    private var rawQuat: [Double] = []
    private var correctedPoints: [Vec3]?
    private var keyframesAtLastPass = 0
    private var closurePassInFlight = false
    private var loopKept = 0
    private var loopTotal = 0

    private var displayLink: CADisplayLink?
    private var lastInfoUpdate: Double = 0

    // MARK: - Lifecycle

    override func viewDidLoad() {
        super.viewDidLoad()
        setupUI()

        UIDevice.current.beginGeneratingDeviceOrientationNotifications()

        xrslam = XRSLAM(UIDevice().type.rawValue)
        camera = Camera(position: .back, preset: .vga640x480)
        camera?.delegate = self
        // Match upstream capture settings: fixed rate + locked focus.
        camera?.setFps(30)
        camera?.setFocus(0.835)
        motion = Motion(updateInterval: 0.01)
        motion?.delegate = self

        NotificationCenter.default.addObserver(self,
                                               selector: #selector(appDidEnterBackground(_:)),
                                               name: UIApplication.didEnterBackgroundNotification,
                                               object: nil)

        displayLink = CADisplayLink(target: self, selector: #selector(tick))
        displayLink?.add(to: .main, forMode: .common)

        print("[PathRec] started; device=\(UIDevice().type.rawValue)")
    }

    deinit {
        displayLink?.invalidate()
    }

    // MARK: - UI setup

    private func setupUI() {
        view.backgroundColor = .black

        stateLabel.font = UIFont.monospacedSystemFont(ofSize: 13, weight: .semibold)
        stateLabel.textColor = .lightGray
        stateLabel.text = "STATE: IDLE (press Start)"

        imageView.contentMode = .scaleAspectFill
        imageView.clipsToBounds = true
        imageView.backgroundColor = UIColor(white: 0.1, alpha: 1.0)
        overlay.backgroundColor = .clear
        overlay.isOpaque = false
        overlay.translatesAutoresizingMaskIntoConstraints = false
        imageView.addSubview(overlay)

        pathView.backgroundColor = UIColor(white: 0.05, alpha: 1.0)
        pathView.layer.borderColor = UIColor.darkGray.cgColor
        pathView.layer.borderWidth = 1.0

        recordButton.titleLabel?.font = UIFont.systemFont(ofSize: 20, weight: .semibold)
        recordButton.setTitle("Start Recording", for: .normal)
        recordButton.setTitleColor(.white, for: .normal)
        recordButton.backgroundColor = .systemGreen
        recordButton.layer.cornerRadius = 12
        recordButton.addTarget(self, action: #selector(toggleRecording), for: .touchUpInside)

        resetButton.titleLabel?.font = UIFont.systemFont(ofSize: 15, weight: .medium)
        resetButton.setTitle("Reset SLAM", for: .normal)
        resetButton.setTitleColor(.white, for: .normal)
        resetButton.backgroundColor = .systemGray
        resetButton.layer.cornerRadius = 12
        resetButton.addTarget(self, action: #selector(resetTapped), for: .touchUpInside)

        infoLabel.font = UIFont.monospacedSystemFont(ofSize: 12, weight: .regular)
        infoLabel.textColor = .white
        infoLabel.numberOfLines = 2
        infoLabel.text = "DIST: -    PTS: -\nPOS: -"

        saveLabel.font = UIFont.monospacedSystemFont(ofSize: 11, weight: .regular)
        saveLabel.textColor = .lightGray
        saveLabel.numberOfLines = 1
        saveLabel.text = "SAVED: -"

        for v in [stateLabel, imageView, pathView, recordButton, resetButton, infoLabel, saveLabel] {
            v.translatesAutoresizingMaskIntoConstraints = false
            view.addSubview(v)
        }

        let m = view.safeAreaLayoutGuide
        NSLayoutConstraint.activate([
            stateLabel.topAnchor.constraint(equalTo: m.topAnchor, constant: 8),
            stateLabel.leadingAnchor.constraint(equalTo: m.leadingAnchor, constant: 16),
            stateLabel.trailingAnchor.constraint(equalTo: m.trailingAnchor, constant: -16),

            imageView.topAnchor.constraint(equalTo: stateLabel.bottomAnchor, constant: 6),
            imageView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            imageView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            imageView.heightAnchor.constraint(equalTo: view.heightAnchor, multiplier: 0.30),

            overlay.topAnchor.constraint(equalTo: imageView.topAnchor),
            overlay.bottomAnchor.constraint(equalTo: imageView.bottomAnchor),
            overlay.leadingAnchor.constraint(equalTo: imageView.leadingAnchor),
            overlay.trailingAnchor.constraint(equalTo: imageView.trailingAnchor),

            pathView.topAnchor.constraint(equalTo: imageView.bottomAnchor, constant: 6),
            pathView.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 8),
            pathView.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -8),

            infoLabel.topAnchor.constraint(equalTo: pathView.bottomAnchor, constant: 8),
            infoLabel.leadingAnchor.constraint(equalTo: m.leadingAnchor, constant: 16),
            infoLabel.trailingAnchor.constraint(equalTo: m.trailingAnchor, constant: -16),

            saveLabel.topAnchor.constraint(equalTo: infoLabel.bottomAnchor, constant: 4),
            saveLabel.leadingAnchor.constraint(equalTo: m.leadingAnchor, constant: 16),
            saveLabel.trailingAnchor.constraint(equalTo: m.trailingAnchor, constant: -16),

            recordButton.topAnchor.constraint(equalTo: saveLabel.bottomAnchor, constant: 10),
            recordButton.leadingAnchor.constraint(equalTo: m.leadingAnchor, constant: 16),
            recordButton.heightAnchor.constraint(equalToConstant: 56),
            recordButton.bottomAnchor.constraint(equalTo: m.bottomAnchor, constant: -10),

            resetButton.centerYAnchor.constraint(equalTo: recordButton.centerYAnchor),
            resetButton.leadingAnchor.constraint(equalTo: recordButton.trailingAnchor, constant: 10),
            resetButton.trailingAnchor.constraint(equalTo: m.trailingAnchor, constant: -16),
            resetButton.widthAnchor.constraint(equalToConstant: 110),
            resetButton.heightAnchor.constraint(equalToConstant: 56),
        ])
    }

    // MARK: - Session control

    @objc private func toggleRecording() {
        if sessionActive {
            stopSession(reason: "stopped")
        } else {
            startSession()
        }
    }

    private func startSession() {
        // Fresh engine per session, like the upstream app pressing Start.
        xrslam?.resetSystem()
        sampleLines.removeAll()
        logLines.removeAll()
        pathPoints.removeAll()
        distance = 0
        lastPosition = nil
        lastSampleTime = 0
        lastStatLog = 0
        lastFeatWarn = 0
        loopPassIndex = 0
        sessionStartUptime = ProcessInfo.processInfo.systemUptime
        lastStatLog = sessionStartUptime
        lastState = .SYS_UNKNOWN
        let startDate = Date()
        recordStart = startDate
        let df = DateFormatter()
        df.locale = Locale(identifier: "en_US_POSIX")
        df.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        sessionBase = "rec_" + df.string(from: startDate)
        keyframeFolder = (sessionBase ?? "rec") + ".kf"
        keyframeSeq = 0
        keyframeCount = 0
        lastKeyframeTime = 0
        rawTimes.removeAll()
        rawXYZ.removeAll()
        rawQuat.removeAll()
        correctedPoints = nil
        keyframesAtLastPass = 0
        closurePassInFlight = false
        loopKept = 0
        loopTotal = 0
        xrslam?.lcReset()
        sessionActive = true
        UIApplication.shared.isIdleTimerDisabled = true
        recordButton.setTitle("Stop & Save", for: .normal)
        recordButton.backgroundColor = .systemRed
        pathView.points = []
        saveLabel.text = "SAVED: -"
        appendLog("SESS", String(format: "start t0=%.3f device=%@ engine=fresh",
                                 sessionStartUptime, UIDevice().type.rawValue))
    }

    private func stopSession(reason: String) {
        guard sessionActive else { return }
        sessionActive = false
        UIApplication.shared.isIdleTimerDisabled = false
        recordButton.setTitle("Start Recording", for: .normal)
        recordButton.backgroundColor = .systemGreen
        appendLog("SESS", String(format: "stop reason=%@ samples=%d kfs=%d dist=%.1f m",
                                 reason, sampleLines.count, keyframeCount, distance))
        saveRecording()
        runFinalLoopClosurePass()
    }

    // Final detection + pose-graph solve after the walk. The raw recording
    // is already saved; if corrections come back, write the corrected file.
    private func runFinalLoopClosurePass() {
        guard let xrslam = xrslam, !rawTimes.isEmpty, keyframeCount >= 5 else { return }
        let task = UIApplication.shared.beginBackgroundTask(withName: "pathrec.finalLoopPass")
        xrslam.lcRunPass { [weak self] result in
            guard let self = self else { return }
            self.loopKept = result?["kept"] as? Int ?? 0
            self.loopTotal = result?["loopsTotal"] as? Int ?? 0
            let correctedFlag = ((result?["corrected"] as? NSNumber)?.boolValue ?? false)
            let corrBytes = (result?["corrections"] as? Data)?.count ?? -1
            self.appendLog("LOOP", String(format: "final pass pool=%d kept=%d rejected=%d corrected=%@ corrBytes=%d",
                                          self.loopTotal, self.loopKept,
                                          result?["rejected"] as? Int ?? 0,
                                          correctedFlag ? "yes" : "no", corrBytes))
            if correctedFlag && corrBytes > 0 {
                self.applyCorrectionsToDisplay { data in
                    if let data = data {
                        self.writeCorrectedFile(data)
                    } else {
                        self.appendLog("WARN", "corrections apply returned nil; no corrected file")
                        self.saveLogFile()
                    }
                    UIApplication.shared.endBackgroundTask(task)
                }
            } else {
                self.saveLabel.text = (self.saveLabel.text ?? "") + " | LOOPS \(self.loopKept)"
                if self.loopKept > 0 {
                    self.appendLog("WARN", "kept=\(self.loopKept) but corrected flag/bytes missing")
                }
                self.saveLogFile()
                UIApplication.shared.endBackgroundTask(task)
            }
        }
    }

    @objc private func resetTapped() {
        if sessionActive {
            stopSession(reason: "reset")
        }
        xrslam?.resetSystem()
        pathPoints.removeAll()
        correctedPoints = nil
        distance = 0
        lastPosition = nil
        pathView.points = []
        print("[PathRec] SLAM reset")
    }

    @objc private func appDidEnterBackground(_ note: Notification) {
        if sessionActive {
            stopSession(reason: "backgrounded")
        }
    }

    // MARK: - CameraDelegate / MotionDelegate

    func cameraDidOutput(timestamp: CMTime, sampleBuffer: CMSampleBuffer) {
        if sessionActive {
            xrslam?.trackCamera(timestamp.seconds, buffer: sampleBuffer)
        } else {
            xrslam?.processBuffer(sampleBuffer)
        }
    }

    func motionDidGyroscopeUpdate(timestamp: Double, rotationRateX: Double, rotationRateY: Double, rotationRateZ: Double) {
        guard sessionActive else { return }
        xrslam?.trackGyroscope(timestamp, x: rotationRateX, y: rotationRateY, z: rotationRateZ)
    }

    func motionDidAccelerometerUpdate(timestamp: Double, accelerationX: Double, accelerationY: Double, accelerationZ: Double) {
        guard sessionActive else { return }
        xrslam?.trackAccelerometer(timestamp, x: accelerationX, y: accelerationY, z: accelerationZ)
    }

    // MARK: - Periodic update

    @objc private func tick() {
        guard let xrslam = xrslam else { return }

        xrslam.setDisplayRotation(currentDisplayRotation())

        if let img = xrslam.getCurrentImage() {
            imageView.image = img
            overlay.imageSize = img.size
        }

        let featureValues = xrslam.getFeaturePoints()
        overlay.features = featureValues.map { $0.cgPointValue }

        let state = xrslam.get_system_state()
        updateStateLabel(state: state)

        let now = ProcessInfo.processInfo.systemUptime

        if sessionActive {
            if state != lastState {
                appendLog("STATE", "\(stateName(lastState)) -> \(stateName(state)) feat=\(featureValues.count)")
                lastState = state
            }
            if now - lastStatLog >= 10.0 {
                lastStatLog = now
                let p = lastPosition ?? (x: 0.0, y: 0.0, z: 0.0)
                appendLog("STAT", String(format: "pos=(%.1f, %.1f, %.1f) dist=%.1f m kf=%d feat=%d",
                                         p.x, p.y, p.z, distance, keyframeCount, featureValues.count))
            }
            if state == .SYS_TRACKING {
                if lastSampleTime == 0 || now - lastSampleTime >= 0.05 {
                    lastSampleTime = now
                    appendSample(at: now)
                }
                if lastKeyframeTime == 0 || now - lastKeyframeTime >= 1.0 {
                    lastKeyframeTime = now
                    captureKeyframe(at: now)
                }
                if keyframeCount - keyframesAtLastPass >= 10 {
                    keyframesAtLastPass = keyframeCount
                    runLoopClosurePass()
                }
            }
        }

        if now - lastInfoUpdate >= 0.2 {
            lastInfoUpdate = now
            updateInfoLabel()
        }
    }

    private func stateName(_ s: SysState) -> String {
        switch s {
        case .SYS_INITIALIZING: return "INIT"
        case .SYS_TRACKING: return "TRACK"
        case .SYS_CRASH: return "FAIL"
        default: return "UNK"
        }
    }

    private func appendSample(at t: Double) {
        guard let pose = xrslam?.getBodyPose(), pose.count == 8 else { return }
        let tx = pose[1].doubleValue
        let ty = pose[2].doubleValue
        let tz = pose[3].doubleValue
        let qx = pose[4].doubleValue
        let qy = pose[5].doubleValue
        let qz = pose[6].doubleValue
        let qw = pose[7].doubleValue
        sampleLines.append(String(format: "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f",
                                  t, tx, ty, tz, qx, qy, qz, qw))
        let p = Vec3(x: tx, y: ty, z: tz)
        if let last = pathPoints.last {
            let dx = p.x - last.x
            let dy = p.y - last.y
            let dz = p.z - last.z
            let d = sqrt(dx * dx + dy * dy + dz * dz)
            if d > 0.5 {
                appendLog("JUMP", String(format: "%.2f m (pos %.1f, %.1f, %.1f)", d, tx, ty, tz))
            } else if d < 1.0 {
                distance += d
            }
        }
        pathPoints.append(p)
        rawTimes.append(t)
        rawXYZ.append(contentsOf: [tx, ty, tz])
        rawQuat.append(contentsOf: [qx, qy, qz, qw])
        if correctedPoints != nil {
            correctedPoints?.append(p)  // tail point; refreshed next pass
        }
        pathView.points = correctedPoints ?? pathPoints
        lastPosition = (tx, ty, tz)
    }

    private func captureKeyframe(at t: Double) {
        guard let xrslam = xrslam,
              let jpeg = xrslam.getCurrentImageJPEG(0.7),
              let folder = keyframeFolder else { return }
        let pose = xrslam.getBodyPose()
        guard pose.count == 8 else { return }
        let qnorm = sqrt(pose[4].doubleValue * pose[4].doubleValue
            + pose[5].doubleValue * pose[5].doubleValue
            + pose[6].doubleValue * pose[6].doubleValue
            + pose[7].doubleValue * pose[7].doubleValue)
        guard qnorm > 0.5 else { return } // skip frames with an invalid (zero) pose
        xrslam.lcAddKeyframe(withTime: t,
                             x: pose[1].doubleValue, y: pose[2].doubleValue, z: pose[3].doubleValue,
                             qx: pose[4].doubleValue, qy: pose[5].doubleValue,
                             qz: pose[6].doubleValue, qw: pose[7].doubleValue)
        keyframeSeq += 1
        keyframeCount = keyframeSeq
        let name = String(format: "kf_%05d.jpg", keyframeSeq)
        let line = String(format: "%d %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %@\n",
                          keyframeSeq, t,
                          pose[1].doubleValue, pose[2].doubleValue, pose[3].doubleValue,
                          pose[4].doubleValue, pose[5].doubleValue,
                          pose[6].doubleValue, pose[7].doubleValue,
                          name)
        keyframeQueue.async {
            let fm = FileManager.default
            guard let docs = fm.urls(for: .documentDirectory, in: .userDomainMask).first else { return }
            let dir = docs.appendingPathComponent(folder, isDirectory: true)
            try? fm.createDirectory(at: dir, withIntermediateDirectories: true)
            try? jpeg.write(to: dir.appendingPathComponent(name))
            let indexURL = dir.appendingPathComponent("index.txt")
            if let handle = FileHandle(forWritingAtPath: indexURL.path) {
                handle.seekToEndOfFile()
                handle.write(line.data(using: .utf8) ?? Data())
                handle.closeFile()
            } else {
                try? line.write(to: indexURL, atomically: true, encoding: .utf8)
            }
        }
    }

    // MARK: - On-device loop closure

    private func runLoopClosurePass() {
        guard let xrslam = xrslam, !closurePassInFlight, keyframeCount >= 5 else { return }
        closurePassInFlight = true
        xrslam.lcRunPass { [weak self] result in
            guard let self = self else { return }
            self.closurePassInFlight = false
            guard let r = result else { return }
            self.loopPassIndex += 1
            let kept = r["kept"] as? Int ?? 0
            let total = r["loopsTotal"] as? Int ?? 0
            self.loopKept = kept
            self.loopTotal = total
            self.appendLog("LOOP", String(format: "pass#%d cand=%d new=%d pool=%d kept=%d rejected=%d corrMax=%.2f m",
                                          self.loopPassIndex,
                                          r["candidates"] as? Int ?? 0,
                                          r["newLoops"] as? Int ?? 0,
                                          total, kept,
                                          r["rejected"] as? Int ?? 0,
                                          r["corrMax"] as? Double ?? 0.0))
            if (r["corrected"] as? Bool ?? false) {
                self.applyCorrectionsToDisplay { _ in }
            }
            self.updateInfoLabel()
        }
    }

    // Recompute the displayed trajectory with the latest pose-graph
    // corrections; completion gets the packed 8N doubles (or nil).
    private func applyCorrectionsToDisplay(completion: @escaping (Data?) -> Void) {
        guard let xrslam = xrslam, !rawTimes.isEmpty else {
            appendLog("WARN", "corrections apply skipped (no samples)")
            completion(nil)
            return
        }
        let tsData = rawTimes.withUnsafeBytes { Data($0) }
        let posData = rawXYZ.withUnsafeBytes { Data($0) }
        let quatData = rawQuat.withUnsafeBytes { Data($0) }
        xrslam.lcApply(toTrajectoryTimes: tsData,
                       positions: posData,
                       quaternions: quatData) { [weak self] corrected in
            guard let self = self else {
                completion(nil)
                return
            }
            guard let data = corrected,
                  data.count % (8 * MemoryLayout<Double>.size) == 0 else {
                self.appendLog("WARN", "corrections apply returned no data")
                completion(nil)
                return
            }
            let n = data.count / (8 * MemoryLayout<Double>.size)
            var pts: [Vec3] = []
            pts.reserveCapacity(n)
            data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
                let d = raw.bindMemory(to: Double.self)
                for i in 0..<n {
                    pts.append(Vec3(x: d[8 * i + 1], y: d[8 * i + 2], z: d[8 * i + 3]))
                }
            }
            self.correctedPoints = pts
            self.pathView.points = pts
            completion(data)
        }
    }

    private func writeCorrectedFile(_ data: Data) {
        guard let base = sessionBase else { return }
        let n = data.count / (8 * MemoryLayout<Double>.size)
        var body = ""
        body.reserveCapacity(n * 100)
        data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
            let d = raw.bindMemory(to: Double.self)
            for i in 0..<n {
                body += String(format: "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                               d[8 * i], d[8 * i + 1], d[8 * i + 2], d[8 * i + 3],
                               d[8 * i + 4], d[8 * i + 5], d[8 * i + 6], d[8 * i + 7])
            }
        }
        let fm = FileManager.default
        let dir = fm.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let url = dir.appendingPathComponent(base + ".corrected.txt")
        do {
            try body.write(to: url, atomically: true, encoding: .utf8)
            appendLog("SAVE", "corrected \(base).corrected.txt (\(n) pts, \(loopKept) loops)")
            saveLabel.text = "SAVED: \(base).txt + corrected (\(n) pts, loops \(loopKept))"
        } catch {
            appendLog("ERR", "corrected write failed: \(error.localizedDescription)")
            saveLabel.text = "CORRECTED SAVE FAILED"
        }
        saveLogFile()
    }

    private func saveLogFile() {
        guard !logLines.isEmpty else { return }
        let fm = FileManager.default
        let dir = fm.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let base = sessionBase ?? "rec_log"
        let logBody = logLines.joined(separator: "\n") + "\n"
        try? logBody.write(to: dir.appendingPathComponent(base + ".log"),
                           atomically: true, encoding: .utf8)
    }

    private func appendLog(_ tag: String, _ message: String) {
        let rel = ProcessInfo.processInfo.systemUptime - sessionStartUptime
        let t = tag.padding(toLength: 5, withPad: " ", startingAt: 0)
        let line = String(format: "%8.1f [%@] %@", rel, t, message)
        logLines.append(line)
        print("[PathRec] " + line)
    }

    private func updateStateLabel(state: SysState) {
        if !sessionActive {
            stateLabel.text = "STATE: IDLE (press Start)"
            stateLabel.textColor = .lightGray
            return
        }
        switch state {
        case .SYS_INITIALIZING:
            stateLabel.text = "STATE: INITIALISING - walk a slow curve"
            stateLabel.textColor = .systemYellow
        case .SYS_TRACKING:
            stateLabel.text = "STATE: TRACKING [REC]"
            stateLabel.textColor = .systemGreen
        case .SYS_CRASH:
            stateLabel.text = "STATE: TRACK FAIL (tap Reset SLAM)"
            stateLabel.textColor = .systemRed
        default:
            stateLabel.text = "STATE: UNKNOWN"
            stateLabel.textColor = .lightGray
        }
    }

    private func updateInfoLabel() {
        var text = String(format: "DIST: %.2f m   PTS: %d   KF: %d   LOOPS: %d/%d",
                          distance, sampleLines.count, keyframeCount, loopKept, loopTotal)
        if let p = lastPosition {
            text += String(format: "\nPOS: x %.2f  y %.2f  z %.2f m", p.x, p.y, p.z)
        } else {
            text += "\nPOS: -"
        }
        infoLabel.text = text
    }

    private func currentDisplayRotation() -> Int {
        switch UIDevice.current.orientation {
        case .landscapeLeft: return 3
        case .landscapeRight: return 1
        case .portraitUpsideDown: return 3
        default: return 1
        }
    }

    // MARK: - Saving

    private func saveRecording() {
        let fm = FileManager.default
        let dir = fm.urls(for: .documentDirectory, in: .userDomainMask)[0]

        let df = DateFormatter()
        df.locale = Locale(identifier: "en_US_POSIX")
        df.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        let base = sessionBase ?? ("rec_" + df.string(from: Date()))

        saveLogFile()

        if sampleLines.isEmpty {
            saveLabel.text = "SAVED: - (no samples)"
            appendLog("WARN", "no samples; nothing to save")
            saveLogFile()
            return
        }

        let txtURL = dir.appendingPathComponent(base + ".txt")
        let body = sampleLines.joined(separator: "\n") + "\n"
        do {
            try body.write(to: txtURL, atomically: true, encoding: .utf8)

            let duration = recordStart.map { Date().timeIntervalSince($0) } ?? 0
            let meta: [String: Any] = [
                "device": UIDevice().type.rawValue,
                "points": sampleLines.count,
                "keyframes": keyframeCount,
                "distance_m": distance,
                "duration_s": duration,
                "events": logLines.count,
                "format": "TUM: t tx ty tz qx qy qz qw",
                "timestamp_note": "t is ProcessInfo.systemUptime in seconds (monotonic)",
                "engine": "xrslam (RD-VIO)",
                "session_note": "fresh engine per Start; on-device loop closure (v0.5)",
                "loops_kept": loopKept,
                "loops_total": loopTotal,
            ]
            if let metaData = try? JSONSerialization.data(withJSONObject: meta,
                                                          options: [.prettyPrinted, .sortedKeys]) {
                try? metaData.write(to: dir.appendingPathComponent(base + ".json"))
            }

            saveLabel.text = "SAVED: \(base).txt (\(sampleLines.count) pts)"
            appendLog("SAVE", String(format: "%@.txt (%d pts, %d kfs, %.1f m)",
                                     base, sampleLines.count, keyframeCount, distance))
            saveLogFile()
        } catch {
            saveLabel.text = "SAVE FAILED: \(error.localizedDescription)"
            appendLog("ERR", "save failed: \(error.localizedDescription)")
            saveLogFile()
        }
    }
}
