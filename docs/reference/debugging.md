# Debugging

Logging, the command registry, and the diagnostic instruments used to investigate a tracking problem: the on-demand state dump (F9) and deterministic record/replay (F10). Build and toolchain details are in [build.md](./build.md), the full command-line flag list in [commands.md](./commands.md), the app's architecture in [architecture.md](./architecture.md), coordinate and unit conventions in [conventions.md](./conventions.md), and OSC/VMC wire details in [wire-protocol.md](./wire-protocol.md). Hand and body tracking specifics live in [hand-tracking.md](./hand-tracking.md) and [body-pose.md](./body-pose.md), camera calibration in [calibration.md](./calibration.md), and IMU mounting/axis calibration in [imu.md](./imu.md). Past investigations and the reasoning behind non-obvious design choices are in [LEARNINGS.md](../../LEARNINGS.md).

---

## Logging

The logger lives in `src/Utility/Logger.h` / `src/Utility/Logger.cpp`. Levels are `LogSeverityLevel` trace/debug/info/warning/error/fatal. Log with the `MIKAN_LOG_<LEVEL>("Scope::function") << ...` stream macros on the main thread, and the `MIKAN_MT_LOG_*` variants off it (`ThreadSafeLoggerStream` takes a mutex). Each process calls `log_init(LoggerSettings)` once; the settings choose the minimum level, an optional log file (`log_filename`, opened as a relative path), whether a Win32 GUI process allocates a console (`enable_console`), and an optional `log_callback`.

Every log line always reaches stdout (stderr at error and above) and the log file, if one was configured, through `log_default_callback`. A caller-supplied `log_callback` is invoked in addition to that, never instead of it: `LoggerStreamImpl::write_line` calls `log_default_callback` first and then calls the configured callback only when it differs from the default. A custom sink (the in-app log panel) can add a destination; it cannot silently swallow the file or stdout output for the code paths that forgot to also write there.

`MikanTrack.exe` is a `WIN32`-subsystem executable (`add_executable(MikanTrack WIN32 ...)` in the top-level `CMakeLists.txt`), so it has no attached console and printing to stdout goes nowhere. `App::startup` (`src/App/App.cpp`) calls `log_init` with `enable_console= false` and `log_filename= "MikanTrack.log"`, so the only way to read a normal run's log is the file, written next to the process's working directory. It also sets `log_callback= &LogPanel::logCallback` (`src/UI/LogPanel.*`), so the same lines populate the in-app Log panel.

The command-line self-tests and tools (below) each get their own logger via `TestRegistry::tryRun`: minimum level info, console enabled, and a log filename built from the flag by stripping leading dashes and appending `.log`. `--replay-verify` writes `replay-verify.log`, `--test-fusion` writes `test-fusion.log`, and so on for every registered command.

---

## Test registry

`src/Tests/TestRegistry.h` / `TestRegistry.cpp`. Each self-test, hardware test, and headless tool lives in its own file under `src/Tests` and registers itself with `MIKAN_REGISTER_TEST(flag, description, category, function)`, placed directly below the command's function so the flag, description, and implementation stay together.

`eTestCategory` has three values:
- `SelfTest`: deterministic, no hardware, no input files. Safe to run as a batch.

- `Hardware`: needs a camera or controller physically connected to mean anything (for example `--test-imuaxes`, `--test-joycon`).

- `Tool`: a headless diagnostic that operates on a file the user names (for example `--replay-verify`, `--calibrate-bones`).

Registrations land in a function-local static (`getMutableRegistry()` inside `TestRegistry.cpp`), not a file-scope global, because registration runs from static initializers scattered across translation units whose construction order relative to a file-scope container is undefined.

`src/Tests` has one command per file: `TestFusion.cpp` (`--test-fusion`), `TestBodyPose.cpp` (`--test-bodypose`), `TestReplay.cpp` (`--test-replay`), `TestDump.cpp` (`--test-dump`), `TestFrameRecorder.cpp` (`--test-framerecorder`), `ToolReplayVerify.cpp` (`--replay-verify`), `ToolReplayDump.cpp` (`--replay-dump`), `ToolReplayBodyPose.cpp` (`--replay-bodypose`), `ToolReplayHands.cpp` (`--replay-hands`), `ToolReplayExtrinsics.cpp` (`--replay-extrinsics`), `ToolCalibrateBones.cpp` (`--calibrate-bones`), and more than twenty others. All of them share `src/Tests/TestCommon.h`, one deliberately generous include block covering most of the app, so each file stays a function plus a registration line. The full flag list is in [commands.md](./commands.md).

What each registered command verifies, and how to run every self-test in one pass, is in [testing.md](./testing.md).

`main.cpp` checks `--list-tests` first (prints every registered command grouped by category via `TestRegistry::printAvailable()`) and otherwise falls through to `TestRegistry::tryRun`, which owns the per-command logger setup/teardown described above and returns the process exit code.

Adding a test means adding one file under `src/Tests` plus a CMake re-run: the target's sources come from `file(GLOB TESTS_SRC src/Tests/*.h src/Tests/*.cpp)` in the top-level `CMakeLists.txt`, and a plain (non `CONFIGURE_DEPENDS`) glob does not pick up a new file until CMake reconfigures.

---

## The diagnostic dump (F9)

Trigger: `F9` anywhere in the app (`MainWindow::handleHotkeys`, `src/UI/MainWindow.cpp`), or the "Dump tracking state (F9)" button in the Tracking panel's Diagnostics section (`SettingsPanels.cpp`). `Shift+F9` and the "Dump in 3 s (Shift+F9)" button beside it run a 3 second countdown first (`Countdown`, `src/UI/Countdown.h`, drawn in the panel like the hold-still test's), the time to get into the pose the dump captures. Both land in `MainWindow::performDiagnosticDump` after the frame's avatar update, which renders the avatar from the tracking camera when the scene has a camera match (below) and hands the render to `visionThread->requestDiagnosticDump(dir, render)`, so the vision thread's state and the avatar's picture describe the same pose.

Output directory: `AppConfig::makeDumpDirectoryPath()` (`src/App/AppConfig.cpp`) returns `<project folder>/dumps/<yyyy-mm-dd_hh-mm-ss>/` under the active project. `DiagnosticDump::write` (`src/App/DiagnosticDump.cpp`) fills that directory with `dump.json`, `dump.jsonl` and per-camera PNGs:

- `config`: the live `AppConfig` snapshot, which may differ from the saved `project.json`.
- `cameras[]`: per camera, `trackingEnabled`, `activeEp`, `deviceFps`, `droppedFrames`, cumulative `seedStats` (cross-camera search-hint accounting), `hasExtrinsics`, a full landmark `snapshot` of that camera's last result, and `images` (`camN_raw.png` plus `camN_annotated.png`, the latter drawn by `annotateFrame` with landmarks, bones, ROI boxes, and per-hand labels overlaid).
- `fusedSnapshot`: a full landmark snapshot of the latest fused output, same schema as a per-camera snapshot.
- `history[]`: the rolling ring, oldest first. `DiagnosticDump::record` appends one compact entry every fusion iteration on the vision thread (`kMaxHistory= 240`, roughly 4-8s of history depending on the fusion rate). Each entry carries per-camera compact hand/body state, the fused `left`/`right` state, `dominantCamera`, `autoScaleFactor`, `fusion` (that iteration's `FusionDiagnostics`), and per-side `imu` state.
- `imuRaw`: raw accelerometer/gyro samples per side, deliberately uncorrected, so any candidate sensor axis mapping can be replayed offline and scored against the palm quaternions already in `history`.
- `imuLastCapture`: the mounting calibration currently in force (`forearmToSensor` plus the capture-quality numbers it was measured from), written once per dump rather than per frame since it only changes on a recalibration.
- `avatar`: in direct mode with a pose solved, the posed avatar and the solve behind it, else `null`: `modelPath`, `cameraIndex`, `cameraToWorld` (the camera's pose in the avatar's world, column-major, OpenCV camera axes, from `DirectRetarget::makeCameraToWorld`), `pose` (`AvatarJson::avatarPoseToJson`: the root and every present bone by humanoid name with its world delta and its posed world position and rotation) and `solveState` (`directSolveStateToJson`: the camera model and tilt, the torso's shoulders, midpoint and frame, the spine and head deltas, and per arm the chain, the seen flags, the reachable depth interval, the model's depth prior, the lift's distance, the chosen depth with its cost and elbow miss, the filtered depth, the wrist rotation, the twist and the measured and filtered finger angles). With it the direct camera's `camN_annotated.png` carries the posed avatar's skeleton (arms, hands, fingers and head, white) projected over the landmarks through `AvatarOverlay` (`src/App/AvatarOverlay.h`), and `camN_avatar.png` is the avatar rendered from that camera at the frame's size with `camN_compare.png` the annotated frame and the render side by side, both listed under the camera's `images`.

`dump.jsonl` is the frame as a one-frame recording in the recorder's own line format (the header with the live config, one frame holding every camera's last staged inputs with the output snapshot and its checksum, the footer), so `TrackingReplay`, `--gate-direct` and `--pose-report` read a dump like any take. `CameraContext` stages the record inputs every frame for it, recording or not.

The ring is recorded every fusion iteration on the vision thread and is serviced even when cameras are stopped: `VisionThread::threadLoop`'s idle path (`!bAnyNewResult`) still checks the dump request flag, so `F9` writes something useful with no camera producing frames.

The scene's camera match: the 3D scene can look through the tracking camera's own lens (the point of view choice in the Stage View's header while it shows a scene, `Scene3dPanel::setPovCamera`, one entry per camera), the frame's aspect letterboxed in the panel and the orbit camera untouched for when it is off. `makeCameraMatchedView` (`src/UI/Scene3dPanel.h`) builds the view and the off-center pinhole projection from a `DirectCameraModel` and the camera's pose so a world point lands on the pixel the intrinsics put it on. In direct mode the pose is the solve's (`MainWindow::updateSceneCameraMatch`: the camera at the measured shoulders, through `DirectRetarget::makeCameraToWorld`), and the solve's chains draw beside the avatar; otherwise camera 1's calibration. The dump's avatar render is that view at the frame's size (`captureFromCamera`), which is what makes `camN_compare.png` a like-for-like pair. The static-pose loop: Shift+F9 with the pose held, open the dump's compare image and the annotated frame, point at the joint that is wrong, then `--pose-report` on the dump's `dump.jsonl` after a solver change re-solves the same frame and reports the reprojection without a recapture (testing.md).

How to read a dump:

- `history` is oldest-first: the frames leading up to a failure sit near the end of the array.

- `fusion.clusters[].affinity` (keys `left`/`right`, each with `vote`/`temporal`/`spatial`/`total`) explains why a cluster was assigned the side it got. `assignedSide == -1` means the cluster was dropped (more clusters than hands) or the assignment was refused.

- The per-camera `snapshot`/annotated-PNG labels and the top-level `fusedSnapshot` are different failure domains. A camera can mislabel or lose a hand in its own preview while fusion still recovers the correct fused output from another camera, and the reverse can happen too. A wrong overlay does not by itself mean the fused output is wrong: check `fusedSnapshot` (or `history[].fused`) before concluding fusion broke.

- Per-pose fields worth reading first: `fkReprojectionPx` (how well the fitted parametric hand model reprojects onto the observed 2D landmarks, the angles-vs-reality fidelity metric, 0 when not evaluated, such as on a fused pose with no single owning camera), `stereoTriangulated`, and for a triangulated cluster the `fusion.clusters[].triResidualRmsPx` / `triResidualMaxPx` (cross-camera reprojection residual; a pairing above `triangulationMaxResidualPx` is vetoed as a wrong cross-camera match rather than a real hand).

---

## Record/replay (F10)

Trigger: `F10` anywhere, or the Timeline panel's "Start/Stop Recording (F10)" button (`src/UI/TimelinePanel.cpp`), calling `VisionThread::requestRecordingStart(config->makeRecordingFilePath())` / `requestRecordingStop()`.

Format: JSONL at `<project folder>/recordings/<yyyy-mm-dd_hh-mm-ss>.jsonl` (`AppConfig::makeRecordingFilePath`): one header line, one line per frame, one footer line (`src/App/TrackingRecording.h`). A missing or torn footer is tolerated as a crash-truncated recording; `TrackingReplay::load` stops cleanly at the first unparseable line.

What gets recorded (`RecordedFrame`): the per-hand fields `LandmarkTo3D::process` consumes (`imagePoints`, `modelPoints`, presence/handedness scores, `imageQuality`, and `sideCorrected` on a hand the direct camera moved to its side) for every camera that popped a fresh frame that iteration, post hand-model, pre-3D-projection and pre-fusion. It also carries the effective `refLengthMeters` (the auto-scale EMA recorded as a plain input rather than re-derived), the opt-in body-pose observation when that stage ran, and the fused output taken immediately after `fuse()` plus an FNV-1a-64 `checksum`. Both the snapshot and checksum are computed PRE-IMU and PRE-body-pose-solver (`TrackingRecording::computeFusedChecksum`), matching the point at which the live vision thread taps them.

Overflow behavior differs between the two recorders, deliberately:

- `TrackingRecorder` (`src/App/TrackingRecorder.*`), the JSONL landmark recorder: a full write queue (`k_maxQueuedFrames= 2048`, roughly 20s at 120 iterations/s) ABORTS the recording instead of dropping a frame. A silent gap would break replay determinism for every later frame, which is strictly worse than an honest truncated file.

- `FrameRecorder` (`src/App/FrameRecorder.*`), the opt-in raw-frame recorder: a full queue (`k_maxQueuedFrames= 64`) DROPS the frame and counts it. Frames are evidence for offline model comparison, not pipeline inputs, so losing one costs coverage of a moment, never replay correctness.

Starting a recording resets transient tracking state (`VisionThread::handleRecordingStartOnThread`): `HandFusion::resetTransientState`, `BodyPoseSolver::reset`, each camera's `LandmarkTo3D::resetTransientState`, and every camera's `lastResult`, so replay's freshly constructed instances start from the same zero state live did. `m_autoScaleFactor` is deliberately left untouched. A config refresh while recording finalizes the in-flight recording first, with reason `"config changed"`: a hard discontinuity the recording's single config-header snapshot cannot represent.

Opt-in raw frame recording: `AppConfig::recording.recordRawFrames`, off by default. The Timeline panel's checkbox carries an explicit privacy warning, because raw frames are video of the room, whereas a landmark recording is abstract enough to share. Frames land in `<recording>_frames/cam<N>_<frameIndex>.jpg` (`TrackingRecording::makeFrameDirectoryPath`), keyed by the same `frameIndex` the JSONL records per camera so a frame and its landmarks join up. This is what makes an offline model A/B possible: the checksummed replay re-runs every stage after inference, so it cannot judge whether a different pose model would have done better, because its input is gone. With frames on disk, `--replay-bodypose <recording.jsonl> [cameraIndex] [rtmpose|rtmw3d]` (`ToolReplayBodyPose.cpp`) re-runs the body-pose stage over them, on the recorded backend or the named one, and reports per-joint step/score statistics (plus depth and its step on the whole-body backend). `--replay-hands <recording.jsonl> [cameraIndex] [out.csv]` (`ToolReplayHands.cpp`) is the hand landmark model's counterpart, the A/B for a change to its crop or model. It re-estimates each recorded hand from the ROI the live pipeline used, the previous fresh frame's landmarks of the same slot, so the comparison stays per frame instead of compounding through the slot's tracking. It reports the estimate time and the landmark and presence differences against the recorded output, overall and by palm size, and the CSV carries each hand's numbers and re-estimated landmarks. The stored frames are JPEG, so the unchanged model differs from its own recording by about 0.5 px at the median, the floor a change is read against. `--gate-direct` (`ToolGateDirectMode.cpp`) runs the whole-body backend the same way and measures its arm directions against the stereo replay, see testing.md.

Audio sidecar: a recording made while the microphone mouth stage is on writes the captured microphone to `<recording>.wav` beside the JSONL (`AudioRecorder`, `src/Audio/AudioRecorder.*`), its header carries where in the capture stream the file begins, and each frame carries the stage's newest viseme weights with their stream index. The weights sit outside the checksum. `--replay-audio <recording.jsonl>` re-runs the stage over the sidecar on the live hop grid and reports the per-viseme difference against the recorded weights, which is the A/B for a profile, gain, or gate change against an old voice ([audio.md](./audio.md)). Like raw frames, the sidecar is a recording of the room.

Face sidecar: every recording opens `<recording>.face.jsonl` beside the JSONL (`FaceRecorder`, `src/App/FaceRecorder.*`) and writes the phone's samples to it, one line per sample at the phone's rate, so a phone that connects mid-recording is captured. The header names the file, and a sidecar that received no sample is deleted when the recording stops. The timeline replays a take through the current face profile with the phone from the sidecar and the mouth from the records, so a profile edit is judged on a recorded face ([face.md](./face.md)). The face sits outside the checksum.

Replay (`TrackingReplay`, `src/App/TrackingReplay.*`) loads the file, reconstructs the recorded `AppConfig` from the header, then re-runs `LandmarkTo3D` -> `HandFusion` bit-exactly on fresh instances per pass, matching the checksum the live tap recorded (PRE-IMU, PRE-body-solver). The recorded IMU forearm output is overlaid rather than re-run (the EKF is deliberately not replayed), and `BodyPoseSolver` then runs fresh over the replayed candidates. Re-running the solver, instead of replaying its recorded output, is what makes body-pose-solver changes A/B-able against old recordings, at the cost of the solver's output sitting outside the checksum contract.

THE RULES:

- `--replay-verify <recording.jsonl>` (`ToolReplayVerify.cpp`) exits 0 only when every frame's replayed checksum matches the recorded one. Always read `replay-verify.log` rather than trusting a shell pipeline's exit status alone: the log lists every divergent frame with both checksums in hex.

- A checksum divergence between an old recording and the current binary, after a change that touches `LandmarkTo3D` or `HandFusion`, is the diff of that change, not corruption. It is expected, and it is exactly what makes such a change visible.

- Checksums are a same-binary guarantee only. Nothing about a match or mismatch survives a rebuild that changes those stages' output; a divergent recording after an unrelated rebuild is not itself evidence of a bug.

- `--replay-dump <recording.jsonl> <firstFrame> <lastFrame> [outPath]` (`ToolReplayDump.cpp`) always replays from frame 0 for determinism, but only regenerates full `FusionDiagnostics` (the same schema the F9 dump's `history[].fusion` uses) inside the requested range, alongside the replayed fused poses, body-pose output, and per-frame checksum comparison. Default output path is `<recording>.frames_<first>_<last>.json`.

What-if replay: the recording's header line embeds the full config used to build the fusion settings (`RecordingHeader::appConfigJsonText`). Header-patching a copy of the recording, or using the Timeline panel's What-if controls (which seed a `TrackingReplay::WhatIfParams` from that same config), re-runs the identical recorded inputs under different fusion, hand-scale, or extrinsics parameters: an A/B of a config change against identical inputs with zero code changes. `--replay-extrinsics <recording.jsonl> <config.json>` (`ToolReplayExtrinsics.cpp`) specializes this to camera extrinsics: it replays once with the recording's own extrinsics and once with a candidate `config.json`'s, and compares triangulation success and fused confidence on the same recorded hands. `WhatIfParams::cameraEnabled` masks cameras out of a pass: a masked camera is as if it never popped a frame, so fusion, the body solver and the no-world decision see only the rest, and `bApplyRecordedImu` off leaves the forearms to the body solver. `--replay-mono-baseline <recording.jsonl>... [cameraIndex] [frames.csv]` (`ToolReplayMonoBaseline.cpp`) uses both to replay a multi-camera recording as one camera alone and measure that output against the stereo-triangulated one on the same frames, which is how single-camera tracking is judged without a single-camera rig. Header-patching `fusion.estimatorMonoDepthAnchorSigmaM` or `cameras[i].liftDepthScale` into copies of a recording is the zero-code A/B of the single-view depth anchor and the lift depth scale. Header-patching an arm's `mapping` or `ikWeight` under `retarget.chains` in a copy of a direct take or a dump's `dump.jsonl`, then running `--pose-report` on the copy, is the same A/B for an arm chain: the report solves under the header's chains and prints each arm's mapping, weight and wrist miss.

---

## The hitch watchdog

`VisionThread::reportHitchIfSlow` (`src/App/VisionThread.cpp`) runs at the end of every loop iteration, including the idle (no fresh camera frame) path. Threshold: `k_hitchThresholdMs= 50.0`. Every camera shares this one thread, so any phase that overruns the frame budget starves all of them at once: frames keep arriving from the driver, find no free block, and are dropped, which looks downstream like a synchronized multi-camera tracking gap or a USB fault rather than what it is.

Named phases (`VisionThread::eVisionPhase`): `ConfigRefresh` (config refresh / recording start-stop), `Capture` (frame pop, undistort, ML inference, 3D projection), `Imu` (wrist IMU sample drain, discovery, forearm publish), `Fusion` (cross-camera fusion, smoothing, auto-scale bookkeeping), `Osc`, `Diagnostics` (dump history record, recording enqueue, dump writes).

Over threshold, `MIKAN_MT_LOG_WARNING` logs the total time, the worst phase and its time, and the full per-phase breakdown; `m_hitchCount` increments and `m_lastHitchMs` / `m_lastHitchPhase` are published for the Performance panel's hitch readout. The panel's chart marks each hitch on the last 600 iterations' per-phase history. A stall now names its phase instead of leaving a generic "tracking froze" symptom to diagnose from scratch.

Rule this watchdog enforces: anything that can block for hundreds of milliseconds must not run on the vision thread. HID enumeration and the Bluetooth open handshake are the concrete case already built this way: `ImuService::update()` runs on every vision-thread iteration and must never block on them, so device discovery runs on a separate worker (`ImuService::discoveryLoop`) instead.

---

## Capture timing

The hitch watchdog only speaks when an iteration overruns; the steady-state cost of each capture step is measured on every frame. `CameraContext::process` (`src/App/CameraContext.cpp`) laps the steady clock between its steps and publishes them on the per-camera result as `TrackingFrameResult::captureTimings`:

- `queueAgeMs`: the capture callback's arrival stamp to the pop on the vision thread, which is how long the frame waited for its turn
- `convertMs`: the raw driver format to BGR
- `undistortMs`: the calibrated lens correction over the full frame (zero without intrinsics)
- `flickerMs`: the whole-frame luminance tracker
- `bodyPoseMs`: the opt-in body-pose stage
- `roiQualityMs`: the per-hand image-quality statistics
- `liftMs`: the 2D landmarks into camera space and world space
- `publishMs`: the preview frame copy for the main thread
- `totalMs`: the whole call, pop to publish

The hand pipeline's own `inferenceMs` sits alongside. The capture side adds two counters per camera on `VideoCaptureSystem`: `getCallbackCopyMs` (an EMA of the time the driver's callback spends copying a frame out of the driver buffer, during which the driver thread is blocked) and `getQueuedFrameCount` (frames waiting for the vision thread right now), next to the existing dropped-frame count.

The Capture stage's Capture Timing table, under its Diagnostics, shows all of it per camera as a ~1 s EMA, with the two counters shown raw. The dump records the raw per-frame series as `captureTimings` in every history record and in each camera's snapshot, plus `callbackCopyMs` and `queuedFrames` in the camera's dump-time entry. Everything before ONNX Runtime runs on the CPU, so this table is the evidence any GPU pre-processing decision waits on.

---

## Image-quality metrics (`HandRoiQuality`)

`HandRoiQuality::analyzeHand` (`src/Vision/HandRoiQuality.*`) runs on the vision thread against the exact frame the model consumed (after undistortion), for every tracked hand, every frame, at about 0.2 ms per camera.

Metrics fill `HandImageQuality` (`src/Vision/TrackingTypes.h`): `meanLuma`, `shadowClipRatio` / `highlightClipRatio`, `contrast` (gray-level standard deviation inside the ROI), `backgroundSeparation` (ROI mean versus the surrounding ring mean), `sharpness` (variance of the Laplacian after a 3x3 median filter), and `noise` (mean `|gray - 3x3 median|` residual). A per-camera `LumaFlickerTracker` separately tracks whole-frame flicker (`lumaInstability`, `lumaFlickerHz`).

All heavy statistics run on a nearest-neighbor decimated crop, capped at `kAnalysisMaxDim= 160`, which is what makes the values resolution-independent and comparable across cameras of different native resolution. The ROI is the landmark bounding box expanded by `kRoiExpand= 1.5`; the background ring is a further `kRingExpand= 1.4` band around that.

Sharpness and noise are split deliberately: raw Laplacian variance alone scores a noisy frame as sharp, because sensor noise reads as edges. Noise is measured separately (the median-filtered residual), and sharpness is measured on the median-filtered image so it is not rewarding noise. Sharpness only means something when it DROPS: a well-lit, low-jitter camera reads in the low thousands, and a higher value than that is just texture or scale, not "better."

These metrics show up in two places: the Capture stage's per-camera table under its Diagnostics (`SettingsPanels::drawImageQualitySection`, worst tracked hand per camera, roughly a 1s EMA, each row's tooltip naming the camera-setting or lighting change to make), and the F9 dump / recording's per-hand `imageQuality` field, which carries the exact raw per-frame series behind that live readout.

---

## The hold-still jitter test

Lives in the Fusion stage's Diagnostics in the Inspector, and in the Direct solver stage's on a direct project (`SettingsPanels::drawHoldStillSection`, "Hold-Still Jitter Test" button), with its transient state in `TrackingPanelState::holdStill*`.

Sequence: a `k_holdStillCountdownSeconds= 3` countdown (time to get the hands back into position), then a `k_holdStillSampleSeconds= 3` sampling window that accumulates every NEW fused result (deduplicated on `fusedResult.timestampMs`, since fused results arrive slower than UI frames and re-adding a held UI frame would fake stability). For each tracked, world-posed hand it rebuilds all 21 world-space joints via `HandPoseModel::buildFingerJoints` (the same forward kinematics a client rebuilds) and accumulates a sum and sum-of-squares per joint.

Result: per side, the mean per-landmark standard deviation over the window, in millimeters, requiring at least `k_holdStillMinSamples= 30` fused samples to be meaningful. With the hands genuinely still, that deviation IS tracking noise, which makes it the one repeatable A/B number for a camera-settings or lighting change: run it once before, once after, compare.

The rule this test exists to satisfy: a free-form (moving) capture cannot A/B jitter. `HandFusion`'s own jitter estimate is a constant-velocity residual (`HandFusionConfig::jitterReferenceM`, surfaced per observation as `FusionDiagnostics::Observation::jitterMm`), and real fast motion produces a large residual by the same arithmetic that flags actual noise, so the two are indistinguishable from that signal alone. Only a genuinely still hand removes the ambiguity, which is what this test buys that a normal recording cannot.

Division of labor with the image-quality table: `HandRoiQuality` A/Bs what a single frame looks like (exposure, noise, sharpness). The hold-still test A/Bs what tracking does with a held-still hand over time (jitter). They diagnose different failure classes and are read together, not as substitutes for each other.

---

## Automation server

For a question about the running app rather than a replayed stage, drive it instead of reasoning from impressions: launch `MikanTrack.exe --automation-server` and use `tools/automate.py` to open a project, inject input, read the log tail, and capture the window. The channel, its namespaces, and the drive loop are in [automation.md](./automation.md).

---

## UDP/OSC

OSC and VMC output go over UDP (`src/Osc/OscStreamer.*`, `src/Osc/UdpSocket.*`). The wire-level traps worth knowing before debugging a receiver, port exclusivity and receive-buffer sizing among them, are documented in [wire-protocol.md](./wire-protocol.md).
