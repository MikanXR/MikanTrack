# Proposal: direct-mode proportion calibration (palms at head)

Not built. Written 2026-11-14 from a design conversation and one dump of the pose. This document proposes a calibration stage for a direct (single front camera) project that measures the user's hand size and arm bone lengths in shoulder widths, from one held pose, with no printed target and no ruler.

## Goal

A direct project runs on the default body lengths and an uncalibrated hand. The body wizard takes its scale from the fused stereo wrist and the hand wizard from the desk marker plane (calibration.md), and a direct project has neither. The `MonoCameraTest` project holds the stock `body` block and no `handScale`.

The direct solve reads those lengths in two places (avatar.md):

- The arm chain. The elbow sits on the circle the upper arm and forearm lengths allow, its miss from the elbow landmark is the dominant term in the wrist's depth, and the two lengths bound the depths searched.
- The hand lift. The lift's palm distance, which scales with the hand's size and the camera's `liftDepthScale`, is scored against each candidate wrist depth.

Both feed the wrist's distance along its ray and the elbow's pole. Under the IK mapping they no longer decide where the avatar's hand lands in the image, only its depth and the bend.

## Why no ruler

`DirectRetarget` multiplies every user length by the avatar's shoulder width over `body.shoulderWidthMeters`, and places the shoulders at the depth where their rays sit the avatar's shoulder width apart. Arm lengths, head lengths, the reach interval and the motion sigmas therefore enter only as ratios to the shoulder width. Scaling every length by one factor scales every depth and leaves every direction alone.

Metric scale would only buy placing the avatar at the user's true distance from the camera, and the VMC root is the identity today. A metric target was considered and set aside for that reason:

- A chessboard on a phone, made metric by matching a bank card outline (ISO ID-1, 85.60 mm) on the screen or by a typed screen diagonal. A web page cannot read the screen's physical size.
- An ArUco marker on a phone. A charuco board on a phone was found whole in 0 to 7 percent of frames (LEARNINGS.md), and a chessboard with a known square size gives the same plane depth without marker bits.

What depends on absolute meters in the direct solve, for the record:

- the hand lift's distance, which is consistent only when the hand scale and the configured shoulder width are wrong by the same factor
- the body model's relative depth prior
- the 0.15 to 2 m depth accept window, compared against depths in the avatar's units

## The pose

Facing the camera, open palms out, fingers touching the sides of the head. The hands, the forearms and the upper arms then lie near the shoulders' plane, and touching the head is a contact the image can check.

Pixel lengths between points at one camera depth scale alike, so ratios of pixel lengths in that plane are ratios of true lengths, with the focal length cancelled.

A dump of the pose on `MonoCameraTest` (2026-11-14, shoulder separation 408 px), against that project's defaults and the multi-camera `Default` project, in shoulder widths:

| Ratio | Dump, left / right | `MonoCameraTest` | `Default` |
|---|---|---|---|
| Hand (wrist to middle knuckle), from pixels | 0.255 / 0.268 | 0.267 (default) | 0.273 (stereo bones) |
| Hand, from the lift's depth against the shoulders' | 0.30 / 0.29 | | |
| Forearm | 0.70 / 0.64 | 0.83 (default) | 0.65 (IMU curl) |
| Upper arm | 0.73 / 0.65 | 0.67 (default) | 1.05 (derived multiple) |
| Head width | 0.43 | 0.60 (default) | 0.47 |

- The default forearm is about 25 percent long.

- Left and right differ by about 10 percent on both arm bones, so one frame is not a measurement.

- The two hand estimates disagree by about 12 percent. The pixel ratio shortens with a tilted palm. The lift's depth does not, and it is the quantity the solver scores.

- The ears are under the fingers in this pose, so the head width is not read from it.

- The `Default` project's shoulder width is exactly 0.30, which looks unmeasured, so its ratios are a soft reference. Nothing on the rig confirms an upper arm.

## What is measured and where it is stored

With `W` the configured `body.shoulderWidthMeters`, used as the unit and not re-measured:

- Hand, per side: `cameras[direct.cameraIndex].liftDepthScale[side]`. Per frame the sample is `scale = sOld * zShoulder / zPalm`, where `zShoulder` is the camera Z of the shoulders placed by `BodyPoseSolver::depthFromKnownSeparation` with `W`, and `zPalm` is `poses[side].palmPositionCamera.z`. The result is the median and the spread is the MAD over the median, as `DeskHandCalibrator::finish` reports them.

- Forearm: `body.forearmLengthMeters = ratio * W`, the ratio being elbow to wrist in pixels over the shoulder separation.

- Upper arm: `body.upperArmPerShoulderWidth = ratio` with `deriveUpperArmFromShoulderWidth` true, so it stays a ratio under a later shoulder width edit.

- Arm aggregation: the 90th percentile per side over the samples, since foreshortening only shortens, then the mean of the two sides. The review warns when the sides differ by more than 10 percent.

- The review shows the implied hand ratio `refLength * scale / W` as a readout.

Why `liftDepthScale` and not `handScale.refLengthMeters`:

- One view measures only the product of hand size, lift bias and depth scale, so writing both would count it twice.

- The depth scale applies after PnP on both object-model paths (`LandmarkTo3D.cpp`). The reference length is ignored for a side with a calibrated skeleton.

- The depth scale is per side and per camera, and replay reads it from the recorded config. Replay takes the reference length from each recorded frame.

The solver already keeps a running median of the same correction (`SolveState::Arm::pnpScale`, the geometry's depth over the raw lift distance), which the score does not read. `pnpScale` over the length scale is the factor the stored scale would need.

## Pose gates

Per frame, from the direct camera's preview result, with the constants on the calibrator. The values are starting points to tune against the dump:

- Body valid, both shoulders at visibility 0.5 or more, the shoulder line within 8 degrees of level, the nose within 0.15 shoulder separations of the shoulder midpoint in x
- Each hand tracked with camera space and its side decided
- Palm facing the camera: the normal from the lifted wrist, index knuckle and pinky knuckle within 25 degrees of the view axis
- Fingers open: the largest proximal bend under 25 degrees, the curl rule of `DeskHandCalibrator::assess`
- Hand at the head: the middle fingertip within 0.6 ear separations of the same side's ear landmark
- Hand not held forward: `zPalm / zShoulder` between 0.7 and 1.4 at the current scale
- Arm bones only, per side: the elbow visible and 16 px inside the frame

A frame samples when every gate but the last passes for both hands. A side that fails the elbow gate still samples its hand. Capture completes at 60 samples. The arm bones need 30 samples on at least one side, and are otherwise left as they are with the review saying so.

Capture is gated and automatic: a live checklist of the gates beside the camera view, sampling while they pass, as the intrinsics wizard samples a still board.

## Pieces

1. `DirectProportionCalibrator` (`src/Calibration/`): a `SampleStatus` with a flag and a value per gate, `assess`, `addSample`, and `finish` into a `Result`. No thread affinity. Inputs are a `TrackingFrameResult`, `MikanMonoIntrinsics`, `W` and the current `liftDepthScale`. Rays are camera-frame, `((px - cx)/fx, (py - cy)/fy, 1)`, with no extrinsics. The pattern is `DeskHandCalibrator`.

2. `--test-directproportion` (`src/Tests/TestDirectProportion.cpp`) on `SyntheticDirectUser.h`: known ratios recovered, each gate rejecting its violation, elbows out of frame leaving the arm bones unset, a foreshortened side flagged.

3. `--direct-proportions <recording.jsonl>` (`src/Tests/ToolDirectProportions.cpp`) over `TrackingReplay`: the gate status and the ratios per frame, so a dump's `dump.jsonl` and recorded takes are read without the window.

4. `DirectProportionWizard` (`src/UI/`): an intro with the live checklist, sampling with progress, a review of measured against current, and accept, which writes the config, marks it dirty and requests a config refresh. Hosted by `WizardHost` under a new `eWizardKind`. The pattern is `BodyCalibrationWizard`, on a camera frame built from intrinsics alone. It runs on the main thread from the preview, so the vision thread is not changed.

5. The launch: in a direct project `SettingsPanels::drawHandCalibrationSection` shows a button beside the direct hands note, through a `TrackingPanelState` launch flag that `MainWindow` forwards.

6. The guided setup: a skippable step after `NeutralCapture` in the single-front chain (`SetupFlow::nextStep`, `update`, `transitionTo`, `WizardSet`), with `TestSetupFlow.cpp` updated for the new chain.

7. Strings: a `directProportionWizard` section and `windows.directProportionWizard` in `en.json` and `ja.json`.

8. Docs: a section in calibration.md, the source of the direct solve's lengths in avatar.md, testing.md, commands.md, and the dump's numbers in LEARNINGS.md.

## Risks and checks

- The shoulder landmarks may move when the arms are raised, which would change the unit. Compare the shoulder pixel separation in the pose with a hands-down frame from the same seat before the wizard writes arm ratios. A shift over 5 percent stops the arm half of the stage.

- The stored scale and forearm are tied to the shoulder width they were measured against. A later edit of the shoulder width leaves the forearm and the depth scale stale. The upper arm, stored as a ratio, follows.

- The elbows sat 179 and 133 px inside a 1280 px frame on the dump. A narrower lens or a closer seat loses them, and the stage then measures the hand alone.

- The lift term compares the palm center's distance with a wrist distance along the wrist ray, a bias of up to half a palm that this stage does not remove.

- The desk hand wizard writes `camera(0).liftDepthScale`, so running it later on the same camera overwrites this value.

## Verification

- `--test-directproportion`, `--test-setupflow` and `--loc-test`, then the full self-test run.
- `--direct-proportions` on the dump: the gates pass and the ratios match the table above.
- Live on `MonoCameraTest` through the automation server: launch, capture, accept, the project holding the new `liftDepthScale`, forearm and upper arm ratio, and `--pose-report` on the hands-up dump with the elbow's reprojection no worse.

## Not in this proposal

- The head width over the shoulder width, which wants the neutral capture, where the ears are uncovered and the hands are down.

- Demoting the elbow term so the hand lift leads the wrist depth. It is a solver change that needs the takes scored again through `--gate-direct`, and the live check of the IK mapping at weight 1 with reach scale 0 comes first (plan.md).

- The "Measure my body" button in a direct project, which opens a wizard that needs extrinsics.

- A learned 2D-to-3D lifter such as MotionBERT as a depth prior. It takes a 243-frame window of 17 Human3.6M joints rooted at the pelvis, which a seated user's framing does not show, and its output has no metric scale.
