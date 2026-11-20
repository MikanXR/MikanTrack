# Proposal: regroup the Direct-mode and body-model UI

Accepted 2026-11-19, after four review passes and three rounds of questions. Nothing here is built. The design reflects the answers (Decisions below) and all four reviews, and no design question is open. This document proposes clearing and hiding the extrinsics in a direct project, letting the project mode decide each camera's body pose and model, keeping the direct camera index valid, moving the camera tilt to the Direct solver's inputs, giving the Direct solver's body measurements their own tooltips, and making the hold-still test work in a direct project.

## Findings

An audit of the Inspector traced each of these fields to the code that reads it.

- Capture neutral pose: Direct mode only (the Direct solver stage's Inputs and the guided setup's neutral step). It stores the torso frame and shoulder midpoint, the head frame, the face neutral when the whole-body model saw the face, and the phone's face anchor while it streams. It is the zero for the spine lean and the head (`src/Avatar/DirectRetarget.cpp:666-744`).

- Camera tilt: Direct mode only. It carries the camera rays into the level world, which sets the lean pivot, the vertical depth planes, and the arm and palm directions (`DirectRetarget.cpp:564-570`). Nothing measures it. Its only writers are the two sliders (`SettingsPanels.cpp:1383`, `SetupFlow.cpp:908`), and the stored neutral is carried to the current tilt at solve time (`DirectRetarget.cpp:567-569`), so the order of the two inputs does not matter.

- Upper arm from shoulder width: both modes. It is the upper arm length (`AppConfig.cpp:506`). The body solver's elbow solve reads it, and in Direct mode it sets the arm's reach, which places the depth planes and limits how far the wrist goes along its ray (`DirectRetarget.cpp:764-765`).

- Head width and nose forward: both modes. The ear rays at the depth the head width implies, and the nose on a sphere of the nose-forward radius about the ear midpoint, give the head's position and its yaw and pitch (`BodyPoseSolver.cpp:669, 685`, `DirectRetarget.cpp:694, 703`). In Direct mode the phone head replaces this while it streams.

- Shoulder width in Direct mode: the torso depth uses the avatar's own shoulder width (`DirectRetarget.cpp:634`). The user's value only scales head width, nose forward and the arm lengths into avatar units (`DirectRetarget.cpp:576-584, 764-765`), so in this mode only those values' ratios to it matter. The shared tooltip ("Sets the shoulders' distance from the camera") does not hold there.

- Extrinsics in Direct mode: the Direct solver does not read them. It reads the intrinsics, the tilt and the camera-space palm (`DirectRetarget.cpp:566, 1263`). Stored extrinsics still have readers in a direct project, because `CameraContext::process` applies the world transform to the direct camera's result whenever they exist (`CameraContext.cpp:484-485`) and `runDirectStage` passes that result through as the output (`VisionThread.cpp:1119`):
	- The VMC stream without an avatar. `solveDirect` fails on a null skeleton (`RetargetStage.cpp:166-167`), and `runOutputStage` falls back to `solvePosition` (`VisionThread.cpp:1343-1344`), which fills sides by `hasWorldPose` (`AvatarRetarget.cpp:140-141`). Arms stream only when extrinsics are stored.

	- The recording. `palmPosW` is written and the fused checksum hashes `hasWorldPose` and the world palm (`TrackingRecording.cpp:430-433, 479-482`), and replay re-applies the transform (`TrackingReplay.cpp:288-290`).

	- The 3D view. The camera frustum draws from the extrinsics and the fused hand skeleton only with a world pose (`Scene3dPanel.cpp:443, 819, 870-871`), in marker space beside an avatar posed in the solve's level world.

	- The hold-still jitter test in the Direct solver's Diagnostics. It accumulates only poses with a world pose (`SettingsPanels.cpp:470`), so it collects nothing in a direct project made by the guided setup, which has no extrinsics.

	- The fused palm readout (`SettingsPanels.cpp:704`), which falls back to camera space with a suffix.

	- The diagnostic dump: `hasWorldPose` and the world palm (`DiagnosticDump.cpp:107-109, 355`), and each camera's `hasExtrinsics` (`:484`).

	The guided setup already skips the extrinsics step for the direct variant (`SetupFlow.cpp:140`). A direct project already hides the IMU stage and its controls, so hiding mode-irrelevant controls has precedent (architecture.md).

- The body model: RTMPose-m (17 planar keypoints) and RTMW3D-X (133 keypoints with a depth each, 12 to 14 ms a frame on DirectML on the dev machine, LEARNINGS.md). No solver reads the depth. What RTMW3D adds is its face and hand blocks, and every consumer of those is Direct only: the hand hints and side correction (switched on for the direct camera only, `VisionThread.cpp:1034-1042`), the webcam face and blinks (which return early outside direct mode, `VisionThread.cpp:1386`), and the slotless-wrist hand size gate inside `DirectRetarget`. The only other users are the Show body model face and Show body model hands debug overlays. In a multi-camera project RTMW3D costs GPU time and adds nothing. In a direct project RTMPose loses all of the above.

- Body pose on the direct camera is required. With it off the direct solve has no input: the torso is never measured (it comes from the body model's shoulders), no arm is taken up (an arm at rest enters only on a body wrist carrying a hand slot), the head has no ears or nose unless the phone streams, and the webcam face has no face block. The finger angles remain, with no arm to hang on. The Direct solver stage then reads "No person" (`PipelineModel.cpp:342-343`) rather than a configuration problem. In a multi-camera project the checkbox is meaningful: the body solver takes the first camera with it on (`PipelineModel.cpp:294-302`), and the body wizard reads it (`BodyCalibrationWizard.cpp:65`).

- Project mode has no switch after creation. `direct.enabled` is written only by the guided setup (`SetupFlow.cpp:548`), which runs only on a new default project (`App.cpp:317-325`), and by config load (`AppConfig.cpp:841`). The direct camera combo (`SettingsPanels.cpp:1356-1371`) is the one live control that moves the direct role between cameras. It commits through `commitTrackingConfigChange`, and the vision thread runs `configure` on every camera (`VisionThread.cpp:711-715`). The vision thread reads `m_config->direct.cameraIndex` live (`VisionThread.cpp:1034, 1107, 1119, 1138, 1389`), so the role moves before that refresh runs. Today the change leaves the new direct camera on its stored backend, and keeps state from the old camera's view: the neutral, the retarget stage's holds and filters (`VisionThread.cpp:743-752`), the webcam neutral averager (`VisionThread.cpp:1395`) and the blink filter (`VisionThread.h:459`, reset only when its neutral seed changes, `VisionThread.cpp:1400-1401`).

- The Inspector draws the stage tree from the shown frame's config (`MainWindow.cpp:1277, 1296`), which is the recorded config in a replay (`MainWindow.cpp:754`), while the stage sections edit the live config (`MainWindow.cpp:546`). A multi-camera live project replaying a direct take therefore draws the Direct solver's controls over its live, non-direct config.

- Removing a camera leaves `direct.cameraIndex` as it was. The Remove button (`DevicePanel.cpp:329-336`, never camera 0) erases the camera while the vision thread runs and then restarts the thread (`App::applyCameraCountChange`, `App.cpp:765`). Removing a camera below the direct one makes the index name a different physical camera with no notice. Removing the direct camera when it is last leaves the index out of range: live, `isDirectModeActive` is false (`VisionThread.cpp:1105-1109`) and the fusion path runs while the UI shows the direct shape. The combo clamps the index only while the Direct solver's Inputs are drawn (`SettingsPanels.cpp:1349-1355`). The only other writers of the camera list grow it: Add Camera (`DevicePanel.cpp:363`) and the setup flow (`SetupFlow.cpp:186`).

- The hold-still test dedupes samples on the fused result's timestamp (`SettingsPanels.cpp:463-464`), and a repeated hand pose can carry a new timestamp in two ways:
	- Face-only iterations republish the last output with the face sample's timestamp (`VisionThread.cpp:1493-1494`), whenever the phone or the microphone drives face-only frames.

	- Fusion re-fuses a camera's unchanged result whenever another camera delivers a frame (`HandFusion.cpp:113-118`), so a multi-camera project repeats poses with no face stream at all.

	Repeats are counted as new samples, which weights the figure toward held poses and lets the 30-sample floor (`SettingsPanels.cpp:490, 501`) be met by repeats. Nothing shows or logs the sample count. The UI polls `m_latestFused` (`MainWindow.cpp:520, 553`), before the loss stage, and a face republish past the hold goes untracked (`VisionThread.cpp:1505-1509`), so no frozen or held pose from the loss stage reaches the test.

- Four replay tools choose their default camera by `bodyPose.enabled`, each taking the first such camera: `--replay-bodypose` (`ToolReplayBodyPose.cpp:105-117`), `--render-direct` (`ToolRenderDirect.cpp:107-111`), the mono baseline replay (`ToolReplayMonoBaseline.cpp:438-446`), and `--gate-wholebody`, which tries the direct camera first (`ToolGateWholeBody.cpp:2484-2490`).

## Decisions

- The project mode picks the body model. The dropdown is removed: every body camera of a multi-camera project runs RTMPose, and the direct camera of a direct project runs RTMW3D. An old config's `backend` key is ignored.

- The project mode also decides body pose in a direct project: always on for the direct camera, off for every other camera. The checkbox shows only in a multi-camera project.

- A direct project has no extrinsics. They are cleared on load, not set to identity: an identity transform would still mark the camera calibrated and would read OpenCV camera space (Y down, Z forward) as the Z-up marker world. Prior project files and recordings are not preserved, so the checksum change this brings to old direct takes is accepted, and new takes replace them.

- Camera tilt moves from the Direct solver's Settings to its Inputs and calibration, above the neutral capture.

- The Direct solver's body section keeps the shared sliders and gets tooltips that say what each value does in this mode.

- The neighboring defects the reviews found (the direct camera index on camera removal, per-camera state across a direct camera change, the hold-still test's frame and sample count) are fixed in this proposal rather than deferred.

- The hold-still test measures camera-space poses in a direct project, rather than being hidden there or deferred. Its color bands stay until the runs in Verification measure the single-camera figure and the skip's effect on the multi-camera one.

- The hold-still repeat skip applies in both modes, so multi-camera figures move toward the figure over distinct frames, with a before and after run recorded.

- The Timeline's Recorded view drops the recorded world pose of old direct takes, so both view modes agree.

- A direct camera change and the removal of the direct camera clear the neutral and reset the retarget stage, the webcam neutral averager and the blink filter.

- Removing the direct camera moves the role to camera 0, which can never be removed.

- The vision thread reads its own last-applied direct camera index, so the role and the body models switch together at the refresh.

- The Body model stage in a replay names the take's model, decided once per take and camera from its recorded observations.

- The whole-body face and hands overlay toggles show only on an RTMW3D camera.

- The reload-on-backend-change block in `CameraContext::configure` and the combo's clamp branch are removed as unreachable.

The dropdown was also the only in-app way to compare the two models. The A/B stays available on recordings through the body-pose replay tool's explicit backend argument, which is the instrument the repo prefers.

## Design

### What a direct project is

`bool isDirectProject(const AppConfig& config)` in `src/App/AppConfig.h/.cpp` is `direct.enabled`. The clear, `getBodyPoseBackend`, the pipeline shape (`MainWindow.cpp:1296`) and the UI gates all read this one predicate.

The invariant is that `direct.cameraIndex` names a camera the config has, whatever `direct.enabled` is. A parse never leaves the camera list empty (`AppConfig.cpp:700-701`), and two places keep the index in it:

- The config parse clamps the index into the list. When the clamp moves the index it also clears the stored neutral, since the neutral belongs to the camera it was captured through, and logs one line naming the old and new index.

- Camera removal adjusts the index with the camera, through one config-level function, `removeCamera(AppConfig&, int cameraIndex)`, that erases the camera and returns whether the direct role moved. A removed camera below the direct one decrements the index, so the same physical camera keeps the role. Removing the direct camera itself moves the role to camera 0 and clears the neutral.

The direct camera combo (`SettingsPanels.cpp:1356-1371`) is disabled unless the live project is direct, as the Capture neutral pose button already is (`SettingsPanels.cpp:1409`), so a non-direct project replaying a direct take cannot move a role it does not use. With the invariant held at both places, the combo's own clamp branch (`SettingsPanels.cpp:1349-1355`) is unreachable and is removed. `VisionThread::isDirectModeActive` (`VisionThread.cpp:1105-1109`) keeps its check against the opened cameras, which is a runtime question (a camera not yet open) rather than the project's shape.

### A direct camera change

The role moves at runtime in two places: the direct camera combo (`SettingsPanels.cpp:1365-1369`) and camera removal. Both run the same main-thread step, and only in a live direct project:

- Clear the stored neutral (`direct.neutralPresent` and its fields, as the Clear button does at `SettingsPanels.cpp:1418-1427`). The Direct solver stage then reads "No neutral" until the user captures again.

- Cancel a neutral capture counting down (`NeutralCaptureControl::reset`, `NeutralCaptureControl.h:20`). The countdown lives in `TrackingPanelState::directNeutral` (`SettingsPanels.h:68`), owned by `MainWindow` (`MainWindow.cpp:551`). The combo reaches it through its section context. `DevicePanel` does not, so the Remove button reports a moved role in its draw result, as `CalibrationPanel` reports its wizard launches, and `MainWindow` cancels the countdown on it.

- Drop a capture already requested and not yet landed (`NeutralCaptureControl.cpp:22-31`). `requestNeutralCapture` is a bare atomic flag today (`VisionThread.h:184, 528`). It takes the direct camera index the request was made for, which the landed `NeutralCapture` (`VisionThread.h:180`) carries back, and a landed capture for another index is discarded. The guided setup's own `NeutralCaptureControl` (`SetupFlow.h:151`, `SetupFlow.cpp:918`) passes its index too. It runs on a one-camera project at index 0 and is not otherwise affected.

- The phone's face anchor (`face.headAnchor`) is camera independent and stays, as the Clear button leaves it.

The Remove button stops the vision thread before it calls `removeCamera`, rather than after, as `App::applyCameraCountChange` does today (`App.cpp:765`). That closes the window in which the running thread sees the camera list and the index change under it.

On the vision thread, the direct solve, the direct camera's hand hints and side correction, and the webcam face read a member, the last-applied direct camera index, in place of `m_config->direct.cameraIndex` (`VisionThread.cpp:1034, 1107, 1119, 1138, 1389`). The config refresh updates it, so the role switches in one step at the refresh, together with the `configure` that swaps the body models. The member survives a thread restart, so the camera removal's restart still sees the change. When the refresh finds a new index in a direct project it resets:

- the retarget stage (`RetargetStage::reset`, `RetargetStage.h:171`, `RetargetStage.cpp:142-148`)
- the webcam neutral averager (`m_webcamNeutralAverager.clear()`, `WebcamFace.h:59`)
- the blink filter (`m_blinkFilter`, `VisionThread.h:459`), to the current face neutral seed

so no hold, filter or face sample from the old camera's view survives. A removal below the direct camera changes the index without changing the physical camera, so this reset fires there too. It costs a moment of rest pose and is accepted. The general config commit and project load do not run the main-thread step: the setup flow writes index 0 onto a default 0 with no neutral, and replay uses the recorded config.

### The mode decides body pose and its model

- Add `std::optional<eBodyPoseBackend> getBodyPoseBackend(const AppConfig& config, int cameraIndex)` beside `makeBodyDimensions`, the camera's body pose with its model, or none when it runs no body pose:
	- direct project, the direct camera: `Rtmw3dWholeBody`
	- direct project, any other camera: none
	- multi-camera project: `RtmPoseBody` when the camera's `bodyPose.enabled` is set, else none
	- a camera index outside the list: none

	The rule lives in this one function.

- `CameraContext::configure` (`src/App/CameraContext.cpp:124-151`) calls it in place of reading `profile.bodyPose.enabled` and parsing `profile.bodyPose.backend`, with the vision thread's last-applied direct index (above). Under the rule a camera only moves between none and one fixed model, so the swap runs through the existing paths: a camera that leaves the direct role gets none and the `else` branch drops its tracker (`:148-151`), and one that takes it has no tracker and builds one on RTMW3D (`:138`). The reload-on-backend-change block (`:131-136`) has no path left to it and is removed. `CameraContext` is constructed only by the vision thread (`VisionThread.cpp:290`), and replay and the tools drive `BodyPoseTracker` directly.

- The pipeline status inputs (`MainWindow.cpp:1308, 1334`) read `bBodyEnabled` and the model-loaded flag through it, so the Body model stages agree with what runs. A non-direct camera of a direct project reads a new reason, that a direct project runs body pose only on its direct camera, in place of `whyBodyDisabled` ("Body pose is not enabled on this camera", `en.json:1199`), which no control could act on there.

- The take's model per camera. `TimelinePanel` computes it when a take loads, beside `getRecordedConfig` (`TimelinePanel.h:48`): for each camera, the first valid recorded body observation's `hasDepth` names RTMW3D or RTMPose, and a camera with no valid observation takes `getBodyPoseBackend` over the recorded config. `TrackingReplay` holds every frame after load (`TrackingReplay.h:104-108, 152`), so the scan is cheap. `ShownFrame` (`MainWindow.h:154`, `MainWindow.cpp:750-754`) carries it in a replay, and live it is `getBodyPoseBackend` over the live config. The per-frame `bBodyHasDepth` (`MainWindow.cpp:1344`) is not used for this, since an invalid observation is written as `{"valid": false}` alone (`TrackingJson.cpp:45-46`) and loads with `hasDepth` false (`:245`), which would name RTMPose on every no-person frame of an RTMW3D take.

- Remove `BodyPoseCameraConfig::backend` (`AppConfig.h:196`), its load (`AppConfig.cpp:312`) and its save (`:364`). `bodyPose.enabled` stays, as the multi-camera choice. A direct project's stored value is ignored.

- Remove the setup flow's body pose writes for the direct variant and the backend write (`SetupFlow.cpp:546-547`), and correct the comment above them (`SetupFlow.cpp:533-535`), which says the solver reads the whole-body model's depth. The tri-camera and single-overhead writes (`:540, :543`) stay.

- The four replay tools that choose a camera by `bodyPose.enabled` take the first camera `getBodyPoseBackend` gives a model to over the recorded config. Their default changes only on a direct take whose direct camera has `bodyPose.enabled` false, every new direct take among them, where it becomes the direct camera. `tools/elbow_root_probe.py:15` reads the raw header the same way and is a multi-camera tool, so it stays.

- `--replay-bodypose` (`ToolReplayBodyPose.cpp:125-127`): an explicit backend argument still wins. The default backend comes from the take's recorded body observations by the same rule as the take's model above. That is exact for takes recorded before and after this change. It falls back to RTMPose when the rule gives none.

- `SettingsPanels::drawBodyPoseCameraControls` (`SettingsPanels.cpp:1201-1237`) takes the shown config and the shown model per camera beside the live config it edits today. Its early return on a camera index outside the live config (`:1204-1205`) gates only the checkbox:
	- The checkbox draws when the live project is not direct and the camera is in the live config, and edits the live config.

	- The text lines follow the shown config. In a direct shown project the direct camera shows a line saying the direct solve runs body pose on it, and any other camera a line saying a direct project runs body pose only on its direct camera.

	- The model line follows the shown model, so a replay names the take's model on every camera it has. The model combo becomes this disabled text line, reusing `bodyBackendRtmPose` and `bodyBackendRtmw3d`. `bodyBackendTooltip` is rewritten to say the project mode picks it and why. `bodyBackendLabel` has no other user (`SettingsPanels.cpp:1220`) and leaves both tables.

- `SettingsPanels::drawBodyOverlayControls` (`SettingsPanels.cpp:1767-1790`) draws four toggles and takes no camera index today (called at `StageContent.cpp:172`). It gains the camera index. The body landmarks and low-visibility landmarks toggles are unchanged. The whole-body face and whole-body hands toggles draw only on a camera whose shown model is RTMW3D, the one camera with those blocks to draw: the direct camera of a direct project, or of a direct take viewed from any project.

### Clear and hide the extrinsics in a direct project

- `applyConfigJson` (`AppConfig.cpp:678`), the parse behind both `AppConfig::load` (`:649`) and `AppConfig::loadFromJsonString` (`:667`), resets every camera's `extrinsics` to its default once the `direct` section is read with `enabled` set (`:841`), after the cameras (`:678-701`, including the v1 fold into the first camera) and `extrinsicsQuality` (`:726-735`) are parsed. The reset covers every field: `present` false, an identity `markerFromCamera`, and the marker id, marker length, pattern error and corner count at their defaults. `extrinsicsQuality` is reset with them. When a camera's `extrinsics.present` or `extrinsicsQuality.present` was true, the parse logs one line naming the cameras cleared, so a project load and a replay of an old take both say why. The parse runs at the startup load (`ProjectManager.cpp:77`), at a project open (`ProjectManager.cpp:134`), once per take load (`TrackingReplay.cpp:48`) and in tools and tests, not per refresh or seek, and a saved project holds `present` false afterward, so the line appears once per old file.

- A project file and a recording's header take the same rule. The project file holds `"present": false` with the identity matrix after its next save, since `cameraProfileToJson` always writes the object (`AppConfig.cpp:369-376`). A new take's header is the live config (`VisionThread.cpp:452`), already cleared, so live and replay agree.

- With the extrinsics gone, the readers in Findings get no world pose in a direct project, live, on new takes and in the Timeline's Replayed view:
	- an avatar-less direct project streams the phone head and no arms
	- recordings carry no world palm, and the checksum hashes the camera palm
	- the 3D view draws the grid, the avatar, the source skeleton and the direct solve's layer, with no frustum and no fused hand
	- the dump carries no world palm and `hasExtrinsics` false
	- the fused palm readout shows camera space

	`applyWorldTransform` and its gate (`CameraContext.cpp:484`) are unchanged, since the stored flag is never set in this mode.

- No way back in. The only writer of extrinsics outside the parse is `ExtrinsicsWizard` (`ExtrinsicsWizard.cpp:458-460`), and it writes the live config. Its launches are gated on the live config, not the shown one: the Calibration menu item (`MainWindow.cpp:386-388`) and the Capture stage's Calibrate Extrinsics button (`StageContent.cpp:141`) are hidden while the live project is direct, and `MainWindow::launchWizard` (`MainWindow.cpp:223-229`, the one caller of `WizardHost::launch`) refuses `eWizardKind::Extrinsics` on a live direct project as the one point that closes every path. Without that, a live direct project viewing a multi-camera take would offer the wizard, and its next takes would record a world palm that their replay parse clears. The setup flow skips the step for the direct variant (`SetupFlow.cpp:140`), and a new project starts from a default config (`App.cpp:317-320`).

- The Timeline's Recorded view builds its frame from the file's recorded poses (`TimelinePanel.cpp:484-500`), not from the config. `buildRecordedResult` drops `hasWorldPose` when the recorded config is direct, so an old take shows the same in both view modes. The What-if summary reads the recorded poses directly (`TimelinePanel.cpp:416, 452`), not through `buildRecordedResult`. On an old direct take its what-if side has no world pose after the clear, so the palm delta is skipped (`:422, 453-454`) and the summary compares tracked flags only. The Timeline also shows its CHECKSUM MISMATCH line (`TimelinePanel.cpp:359-360`) on every tracked frame of such a take, for the reason the load's log line gives.

- The extrinsics status readouts follow the shown frame's config, the one the stage tree reads (`MainWindow.cpp:1296`). The Lift section then always agrees with its status line. Pass it to `StageContent` beside the live config the sections edit (`MainWindow.cpp:546`).

- `CalibrationPanel::drawCameraCalibration` takes `bool bWithExtrinsics` after the existing `bWithHeader` (`CalibrationPanel.h:25`). The one call site (`StageContent.cpp:175`) passes both explicitly, the second as "the shown project is not direct", which drops the Extrinsics separator and its status.

### The hold-still test in a direct project

- The test (`SettingsPanels.cpp:466-515`) picks its frame once per window, by the live project: world poses in a multi-camera project, camera-space poses (`hasCameraPose`, `palmPositionCamera`, `palmOrientationCamera`) in a direct one. A pose without the chosen kind is skipped, so one accumulator never mixes the two frames, and a tracked pose with neither (no intrinsics) adds nothing. The result is the mean over the 21 points of the square root of each point's summed per-axis variance (`:507-515`), which a rigid change of frame does not alter, so the figure keeps its meaning and units.

- A sample is skipped when its palm position and orientation equal that side's previous accepted sample, in addition to the timestamp dedupe. Both repeat sources in Findings carry the hand pose unchanged and are no longer counted, while a live pose never repeats exactly. This applies in both modes, so it changes multi-camera figures too, toward the figure over distinct frames.

- The result shows each side's accepted sample count beside the deviation, and the test logs both when a window ends, so the effect of the skip can be read.

- In a direct project the number is the raw single-camera lift before the retarget, not what clients receive. The tooltip `trackingPanel.holdStillJitterTestTooltip` (`en.json:628`), which says the test uses "the fused world-space output (what clients actually receive)", is reworded for both modes, in `en.json` and `ja.json`. The 2 mm and 5 mm color bands (`SettingsPanels.cpp:565-567`) were set for fused output with repeats counted. They stay until the runs in Verification measure where the single-camera lift sits and how far the skip moves the multi-camera figure.

### Camera tilt moves to Inputs and calibration

`StageContent.cpp:230-244`: the Direct solver's Inputs become the camera, the tilt, then the neutral capture. Settings keeps the depth planes and the body section. `SetupFlow` already shows the tilt with the neutral step and does not change.

### Direct tooltips on the body section

`SettingsPanels::drawBodyDimensionControls` (`SettingsPanels.cpp:1254-1285`) draws six tooltips, and every one says something specific to the multi-camera body solver. The function takes them as one small struct of keys from the host, as `drawForearmLengthControl(context, tooltipKey)` already takes its key. The Body solver stage (`StageContent.cpp:222`) passes today's keys, and the Direct solver stage (`:242`) passes six new `directStage.*` keys:

- Upper arm from shoulder width (today `deriveUpperArmTooltip`, about the elbow bending the wrong way): takes the upper arm as a multiple of the shoulder width, which is easier to set than the upper arm itself
- Upper arm ratio (today `upperArmRatioTooltip`, the 0.74 biacromial figure measured on the RTMPose shoulder points): the multiple, which with the forearm sets the arm's reach
- Upper arm (today `upperArmTooltip`, "Decides which of the two elbow solutions is real"): sets the arm's reach, which places the depth planes and limits how far the wrist goes along its ray
- Shoulder width: converts the other measurements into the avatar's size, so in this mode only their ratios to it matter
- Head width: with nose forward, sets how the head's turn is read while the phone is not streaming
- Nose forward: sets head yaw and pitch while the phone is not streaming

The keys land in `en.json` and `ja.json` in the same change, since `--loc-test` fails on parity. The new Japanese strings join the human-review list in `docs/plan.md`.

### Docs

- `docs/reference/architecture.md`, the Inspector paragraphs (`:179` names the tilt under Settings): Capture, Lift and the Calibration menu without extrinsics in a direct project, the tilt in the Direct solver's inputs, the direct camera combo live only in a direct project, the Body model stage naming its model rather than choosing it, its checkbox only in a multi-camera project, and the whole-body overlay toggles only on an RTMW3D camera.

- `docs/reference/body-pose.md:27-30` and `:124`: the backend list, the `bodyPose.backend` config line and `bodyPose.enabled` become the mode rule.

- `docs/reference/calibration.md`, where results persist: a direct project holds no extrinsics, loading one clears them, and hand-editing a multi-camera project to direct deletes its extrinsics on the next save.

- `docs/reference/avatar.md`: the direct retarget's wiring paragraph says the direct camera always runs the whole-body model, and what a direct camera change and a camera removal reset. The paragraph on switching a project between shapes (`:149`) says the switch to direct drops the extrinsics.

- `docs/reference/commands.md`: the `--replay-bodypose` entry (`:128`) states its default camera and backend, and the `--render-direct` entry (`:136`) its default camera on a direct take.

- `docs/reference/debugging.md`: the record/replay and dump sections say direct takes carry no world palm and `hasExtrinsics` false, the load logs a cleared camera, and old direct takes diverge under `--replay-verify` and in the Timeline for that reason. The hold-still section (`:184-190`, which says "world-posed hand" and "world-space joints") gives the frame by project mode, both repeat sources and the repeat skip, and the sample count readout.

- `docs/reference/testing.md`: the new self-test cases and the changed `TestDirectRetarget` case. `--gate-wholebody labels` (`:183, :209`) reads the recorded blocks against each kind's fused side, and no live configuration now records both the whole-body blocks and extrinsics. Its raw-frame model pass (`ToolGateWholeBody.cpp:196-208`) still covers it, which the doc should say.

- `LEARNINGS.md`: the selectable backend went in as an experiment (`:245`). Its removal and the verdict (no solver reads the depth, RTMW3D adds nothing outside a direct project) join that entry. The hold-still repeat skip and the before and after figures from Verification join it as a measured change to an instrument.

- `README.md:171`: check, and update if it names the model choice.

- `docs/plan.md`: an item pointing at this proposal once it is accepted. The item at `:36` (a direct project falling back to the position pipeline streams its rest arms) names an out-of-range camera index as one cause and assumes world-placed arms. Both premises go with this change, so it is reworded or removed in the same pass.

## Verification

`--replay-verify` diverges on direct takes whose header carried extrinsics: clearing them on load drops the world palm the checksum hashes. That divergence is this change's diff, not corruption, the load's log line names it, and those takes are replaced by new recordings. Multi-camera takes and direct takes without extrinsics verify as before. No replayed stage (`LandmarkTo3D`, `HandFusion`, `BodyPoseSolver`) changes.

- Build RelWithDebInfo.

- The existing direct config load case in `TestDirectRetarget.cpp` (`:1799-1845`) sets `direct.cameraIndex` 2 on a one-camera default config and checks it loads as 2 (`:1814`), which the clamp breaks. It gains three cameras so the round trip still checks 2, and a second case keeps one camera and checks the clamp to 0 with the neutral cleared. A third case clamps a non-direct config's index.

- A self-test case for `getBodyPoseBackend`, added to an existing test:
	- the direct camera
	- another camera of a direct project
	- a multi-camera camera with body pose on and off
	- a queried index outside the list

- A self-test case for the load rule beside the direct config load case: a direct config with extrinsics on two cameras loads with every field at its default and `extrinsicsQuality` reset, and a multi-camera config keeps them.

- A self-test case for `removeCamera`: removing a camera below the direct one decrements the index, keeps the neutral and reports the role unmoved, removing the direct camera moves the role to camera 0, clears the neutral and reports it moved, and a non-direct config keeps its index in range.

- `MikanTrack.exe --loc-test` and the full self-test batch from testing.md, reading the logs.

- With no camera or backend argument, `--replay-bodypose` picks the body camera and names `rtmpose` on a tri-camera take, and picks the direct camera and names `rtmw3d` on a direct take, including a new one whose cameras all have `bodyPose.enabled` false. `--render-direct` and the mono baseline replay pick the direct camera of such a take.

- Live, through `tools/automate.py`, on two projects, since nothing switches a project's mode:
	- A multi-camera project shows Calibrate Extrinsics under Capture and in the Calibration menu, the extrinsics status under Lift, RTMPose on each Body model stage with its checkbox, and no whole-body overlay toggles.

	- A direct project hides all three extrinsics surfaces, shows no body pose checkbox, shows the whole-body overlay toggles only on the direct camera, its direct camera's Body model stage reads RTMW3D, its other cameras read the direct-only reason, the log shows `rtmw3d_wholebody.onnx` loading, and the Webcam face stage goes live.

	- The direct project replaying a multi-camera take: the Calibration menu and Capture stage offer no extrinsics wizard, the Lift stages show the take's extrinsics status, and every camera of the take shows RTMPose with no direct-only text.

	- A one-camera multi-camera project replaying a direct take with more cameras: the Direct solver's camera combo is disabled, and each of the take's cameras shows its Body model text and model, RTMW3D on its direct camera, with no checkbox for the cameras the live project lacks.

	- On a direct project with two cameras, changing the direct camera moves body pose with it (the new direct camera loads RTMW3D and the old one runs none, in the log and the Body model stages), clears the neutral, and cancels a neutral countdown started before the change.

	- On a direct project with three cameras and the direct role on camera 2, removing camera 1 keeps the same physical camera direct with its neutral. Removing the direct camera moves the role to camera 0, clears the neutral, and cancels a running neutral countdown.

	- The Direct solver's Inputs show the tilt above Capture neutral pose, and its body section shows the six Direct tooltips.

	- The hold-still test in a direct project collects samples with the phone streaming, and its shown and logged sample count matches the camera frames in the window rather than the face frames. Its figure is recorded as the single-camera baseline against the 2 mm and 5 mm bands.

	- The hold-still test in a multi-camera project, before and after the change on the same rig and pose: the logged sample count drops by the repeats, and both figures are recorded against the bands.

- A direct project file with extrinsics on its cameras, `"backend": "rtmpose"` and `"enabled": false` on the direct camera: it loads with the log line naming the cleared cameras, the direct camera runs RTMW3D, and the Lift stages read camera space. After a save each camera's extrinsics hold `"present": false` with the identity matrix and default fields, and no camera holds a `backend` key.

- An old direct take with extrinsics in the Timeline: the Recorded and Replayed views both show no fused hand in the 3D view, and its Body model stage names its model on every frame, including frames with no person.

## Resolved in review

- Extrinsics in a direct project are cleared on load, not kept hidden or set to identity (Decisions).

- Body pose on the direct camera is no longer switchable, and other cameras in a direct project run none (Decisions). Nothing in the direct pipeline reads another camera's body result (`VisionThread.cpp:1136-1152, 1380-1402` read only the direct camera), which reached only the preview overlay, the dump and the recording. The body and hand calibration wizards are reachable only from the Body solver and Fusion stages (`StageContent.cpp:180, 218`), which a direct project does not have.

- Readers of `bodyPose.backend`: the setup flow, `CameraContext`, the Body model stage and `ToolReplayBodyPose`. Readers of `bodyPose.enabled`: `CameraContext`, the pipeline status inputs, the body wizard, the Body model stage, the four replay tools named in Findings, and `tools/elbow_root_probe.py`. No automation command, test or recording consumer reads either.

- No other path brings extrinsics into a direct project. `ToolRenderDirect` sets `direct.enabled` on a copy of a multi-camera take's config after the parse (`ToolRenderDirect.cpp:105-118`), so the extrinsics survive there, and it reads only the intrinsics and the camera-space result. `ToolReplayExtrinsics` compares nothing on a direct take, as before. The tests that set extrinsics on an in-memory direct config (`TestSceneView.cpp:331-341`, `TestDump.cpp:212-215`) do not pass them through the parse. `WizardHost::launch` has one caller, the setup flow drives the extrinsics step only for non-direct variants, and no automation command launches a wizard.

- The parse's log lines appear once per old file, not per refresh or seek. An empty camera list cannot follow a parse.

- No doc, script or test relies on a `--render-direct` default other than the first body camera (`tools/kalidokit_ab/README.md:5` passes an explicit index).
