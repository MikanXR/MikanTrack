# Proposal: the whole-body model's face and hand landmarks

Draft for review, 2026-11-11. Nothing here is built. Two rounds of questions are answered (Decisions below), and the design reflects the answers. This document proposes carrying the RTMW3D whole-body model's 68 face landmarks and 42 hand landmarks past the tracker, and three uses of them in the single-camera direct mode: a `webcam.*` face source with blinks, hand slot seeding from the body model, and a hand side check. It lists what is in place, the decisions, the design, the measurements that gate each use, and the questions still open.

## Goal

The `rtmw3d` backend (body-pose.md) runs RTMW3D-X, which emits 133 COCO-WholeBody keypoints per model frame:

- 0 to 16: the 17 COCO body keypoints
- 17 to 22: six foot points
- 23 to 90: the 68 iBUG face landmarks
- 91 to 111: the left hand, 21 points in the MediaPipe hand order
- 112 to 132: the right hand, likewise

The tracker keeps 29 of them. `BodyPoseTracker::fillObservation` (`src/Vision/BodyPoseTracker.cpp:182`) copies the 17 COCO keypoints and the 12 `k_wholeBodyExtraSlots` entries (`BodyPoseTracker.cpp:50`) into the 33-slot BlazePose layout of `BodyPoseObservation` (`src/Vision/BodyPoseTypes.h`). Kept from the face block: the inner and outer eye corners and the mouth corners, 6 of 68. Kept from each hand block: the thumb, index and pinky MCPs, 3 of 21. The rest stays in the tracker's `RtmPoseResult` and is overwritten by the next model run, so no overlay, dump, recording or solver sees it, and a replay cannot recover it without the raw frames.

The model already computes these points. Carrying them costs no inference. What they could buy, in the direct mode a monitor webcam runs:

- **Face expressions without the phone.** face.md's sources are the phone and the microphone. plan.md's Later list holds "A `webcam.*` face source". The 68 points give eye openness, mouth opening and width, and brow height, from the camera the direct mode already runs.

- **A better head.** The direct head is solved from three points, the ears and the nose (avatar.md, the direct pipeline's Head rule). Hair and headsets hide the ears. The face block gives about 50 rigid points (the eyes, the nose, the jaw line) for a fit against a generic face.

- **Hand reacquisition.** plan.md's direct-mode item "Direct mode's hand pipeline runs blind" says the hand slots need seeding from the body model, because the cross-camera search hints need extrinsics the direct setup lacks. The whole-body hand block gives what a `HandSearchHint` needs: a center, a direction and a size.

- **Hand side evidence.** The direct solver reports swapped hand labels from wrist distances alone, and plan.md's open question asks whether `tracking.flipHandedness` defaults right. The hand block is 21 points per hand labelled by the arm it ends, so the hand slots can be matched by hand shape and not only by one wrist pixel.

## Decisions

- Blinks are a requirement of the `webcam.*` source. Whether they need a second, face-only model or the whole-body face points are enough is decided by measurement (piece 4b). Both face model candidates are run, and they are measured beside the whole-body points on the same take.

- The blink evaluation measures every candidate at two cadences, every camera frame and the live `poseFrameDivider`. At the default divider of 2 a 30 Hz webcam samples the eyes at 15 Hz, where a fast blink is one or two samples. The cadence a face model runs at is decided at the 4b review stop from those numbers.

- The Auto profile falls back to `webcam.*` when the phone is not tracked (piece 5). On the mouth, Auto combines the webcam's shape and the microphone's with max, in the manner of `max(webcam.jawOpen, viseme.aa)`. That is a default: how the sources combine is the user's choice through their own profile.

- `webcam.*` declares all 52 ARKit names, as `phone.*` does. A column the active method does not measure reads zero. The Auto profile is generated from the method's measured set, so an unmeasured column keeps its current fallback. A profile therefore compiles whichever method piece 4b picks.

- `webcam.*` values are raw, as the phone's are. The dead zone and the smoothing live in the Auto profile's functions, sized from piece 4's rest jitter, and a duplicated profile retunes them.

- The face neutrals live on `DirectNeutralPose`, behind their own present flag. A neutral captured before they existed reads generic defaults, and the Webcam row prompts a recapture.

- The head has two references. The neutral capture stores one head frame per method, the face fit and the ear and nose head. The solver uses the face fit while its scores clear the gate and the ear and nose head otherwise, each against its own neutral.

- The block lives on `BodyPoseObservation`. A sibling on `CameraFrameResult` was considered. Nothing found argues for it: the block comes from the same model run, shares the observation's `modelFrameIndex`, box source and hold behavior, and rides the same mirrors and recording key.

- Recordings write the block on every frame. Trimming it is a later optimization.

- Hand seeding is for direct mode only. Doing both is not easier: `CameraContext::setSearchHints` replaces the hint queue, so a multi-camera camera would need its body hints merged with the cross-camera ones, while a direct camera has no extrinsics and so no cross-camera hints to merge with.

- A body hint can evict a slot. `applySearchHints` seeds free slots only, so a hint arriving while the palm detector's scene false positives hold the slots would be skipped. On the direct camera a body hint with no free slot replaces a slot that sits on neither whole-body hand.

- The hand side check stays a report, and the solver's entry gate keeps the wrist distance. The shape match feeds the `bHandednessSwapped` report and the rejected-frame count only. The direct solver rejects a hand slot that sits on the opposite wrist rather than posing the wrong arm with it, so a wrong label costs a hand dropout, not a wrong pose. Per-frame correction would turn the dropouts while the hands cross or clasp into kept hands. That is the one gain it offers, and it is deferred until the report shows how often those dropouts happen.

- The model's per-point depth is kept on the face and hand points.

- The measurement take shares a session with the take plan.md's direct stage 1 item owes. Same setup and calibration, two takes back to back: the IMU and depth-sweep take for the forearm gates, then this proposal's face and hands script.

- The take includes a head sweep with the ears hidden by over-ear headphones, since hidden ears are the reason for piece 6. The face fit can pass its gate on that segment alone.

## What is in place

- `RtmPoseResult` (`src/Vision/RtmPoseModel.h`) holds all 133 points with their x and y in undistorted full-frame pixels, a depth relative to the hip midpoint, a SimCC score and a depth score. The block offsets are named there (`WHOLEBODY_FACE_START`, `WHOLEBODY_LEFT_HAND_START`, `WHOLEBODY_RIGHT_HAND_START`).

- `BodyPoseObservation` rides `TrackingFrameResult::body` through the fusion candidate mirrors, recordings, replay and dumps. `bodyPoseToJson` (`src/App/TrackingJson.cpp:43`) writes its 33 slots, and the reader takes a missing key at its default. The recording format check is strict (`TrackingRecording.cpp:624`, `k_formatVersion` 1), so a new field goes in as an optional key, the way `hasDepth` and `providedMask` went in, and the version does not move.

- `CameraContext::process` (`src/App/CameraContext.cpp:269`) runs the hand pipeline (line 348) before the body tracker (line 365) on each frame, and the body tracker runs on its `poseFrameDivider` cadence and re-emits the held observation in between. A body-derived hint therefore comes from the previous model frame, one or more camera frames old, as the cross-camera hints do.

- `CameraContext::seedSearchHints` (`CameraContext.cpp:186`) builds `HandSearchHint`s (center, direction, palm size in pixels) from the last fused result projected through the extrinsics. Hint-seeded slots are speculative: a first landmark pass that finds no confident hand drops them. `HandSeedStats` counts each hint's fate.

- The direct solver admits a hand slot only near its body wrist (`DirectRetarget.cpp:756`, within `kHandAgreementShoulderWidths` shoulder widths for the entry frames) and reports `bHandednessSwapped` after a run of frames with a slot on the opposite wrist and none on its own (`DirectRetarget.cpp:1161`). The report reaches the Direct solver stage and the guided setup, which flips `tracking.flipHandedness` once. Nothing in the solve reads it.

- The face side: `FaceSources` (`src/Face/FaceSources.h`) holds the phone and the viseme sources and the profile's outputs. `FaceExprInputs` is the one place a namespace is added (face.md). Profiles map sources to VMC blend names, so a new source changes no wire address.

- The rig has measurement truth for all of this: the phone (iFacialMocap) gives 52 ARKit blendshapes and a head rotation, the tri-camera rig gives stereo hands with fused side votes, and raw-frame recording lets the model be re-run on a take.

## Design

### 1. The observation carries the whole-body extras

A `WholeBodyExtras` block on `BodyPoseObservation`, filled only when the backend emits 133 keypoints:

- `present`: true on a whole-body model frame
- `face[68]`: x, y in undistorted full-frame pixels, z the model's depth relative to the hip midpoint, as the slots carry it
- `faceScore[68]`: the SimCC score. The model's separate depth score is not carried, as the slots do not carry it
- `hands[2][21]` and `handScores[2][21]`, indexed by the body side (0 left, 1 right, the person's own), the side being the arm the model attached the hand to

The 33-slot layout and its `providedMask` stay as they are, so every current consumer is untouched and both backends still read alike. The six face points and six hand points already copied into slots are duplicated in the block. That keeps the block self-contained in iBUG and MediaPipe hand order, which is what the face and hand math index by.

The foot points stay dropped. Nothing names a use for them.

### 2. Recordings and dumps

`bodyPoseToJson` writes the block under an optional `wholeBody` key and the reader leaves it absent when the key is missing, so every existing recording loads unchanged. Checksums do not move: `computeFusedChecksum` (`TrackingRecording.cpp:463`) hashes the two hand poses and nothing of the body observation, and the direct mode's passthrough record uses the same function.

Size: 110 points of 4 numbers each is about 3.5 KB of JSON per frame per whole-body camera, about 6 MB a minute at 30 Hz. The block is written on every frame, a held frame repeating it as the slots repeat. Writing it only on a fresh `modelFrameIndex` is a later optimization.

The F9 dump writes the block into `dump.json` and draws it on the camera image.

### 3. The overlay

`HandOverlay.cpp` draws the 68 face points with the iBUG contour connections (jaw, brows, nose, eyes, lips) and each whole-body hand with `HAND_CONNECTIONS`, in a color distinct from the hand pipeline's own hand. A toggle on the camera's Body model stage shows or hides each group. The point that started this proposal, that the debug view shows far fewer face points than the model emits, is answered by this piece alone.

### 4. The measurement take and the gate tool

Before any consumer is built, measure what the points are worth from the webcam position, where the face covers perhaps 100 pixels of the model's 384 by 288 input:

- **The take:** raw frames on, the webcam at the monitor, the phone streaming with its face sidecar, and the tri-camera rig recording. The script: a neutral hold, slow and fast blinks, each eye alone, the mouth through the vowels and a wide smile, brows up and down, the head through yaw, pitch and roll, the same head sweep again wearing over-ear headphones, the hands crossing, clasping and passing near the face, and each hand dropping out of frame and returning. It is recorded in the same session as the IMU and depth-sweep take that plan.md's direct stage 1 item owes, as a second take on the same setup and calibration.

- **The tool:** `--gate-wholebody` (or a mode of `--gate-direct`) re-runs RTMW3D over the webcam's raw frames and reports, per group:
	- rest jitter of each face point, in pixels and in face-widths
	- the eye openness ratio against the phone's `eyeBlinkLeft` and `eyeBlinkRight`, the mouth opening against `jawOpen`, the mouth width against `mouthSmileLeft` and `mouthSmileRight`, the brow height against `browInnerUp`: correlation and the separability of a blink
	- the head from a face fit against the phone head, after the anchor alignment, beside the current ear and nose head against the same truth, reported apart for the ears-visible sweep and the headphones sweep
	- each whole-body hand's wrist and MCPs against the stereo hand projected into the webcam, and the hand box's overlap with the hand pipeline's box
	- the side each whole-body hand assigns to each hand slot against the stereo fused side
	- the hand pipeline re-run over the raw frames with body hints off and on: the frames from a dropout to the slot's return, the hints' funnel by source, and the count of hints that met no free slot

The hint comparison has to re-run the hand models on the raw frames. A recording's replay starts after inference, so it cannot show what a hint would have changed.

The gates come out of these numbers. Each consumer below lands only when its gate passes, and a gate that fails is recorded in LEARNINGS.md as a measured verdict.

### 4b. The blink evaluation: whole-body points against a face model

Blinks are required, and they are the hardest shape for the whole-body crop: an open eye is a few pixels tall in the model's input at the webcam's distance. The tool measures both candidates on the same take's raw frames, against the phone's `eyeBlinkLeft` and `eyeBlinkRight` as truth:

- **Whole-body points:** the eye aspect ratio from the six iBUG points per eye.

- **A face model on a face crop:** the crop taken from the face block's extent each model frame, so no face detector is added. Both candidates are run. Each is checked for its license, its ONNX export and DirectML support first, and a candidate that fails the check is recorded with the reason and not run:
	- MediaPipe Face Landmarker's mesh model, with its iris points, and its blendshape model, which emits the 52 ARKit columns directly
	- an RTMPose face model (the 106-point face6 family), which shares the `RtmPoseModel` SimCC decode

For the whole-body points and each face model the tool reports the blink detection rate and false blinks at a threshold, the open-eye level against the phone's, the latency of a blink edge, and the inference milliseconds on DirectML. Each is reported at two cadences: the model run on every camera frame, and on the frames the live `poseFrameDivider` leaves. The whole-body points run live at the divider, so they are enough only if they pass the blink gate there. If they do not, the best passing face model becomes a third ONNX model on the direct camera, and its output feeds piece 5 in place of the ratios for the shapes it measures better. Its cadence comes from the same numbers: the body model's if it passes there, otherwise every camera frame with the crop held from the last body model frame. Its output is then recorded per frame on the observation beside the block, so the timeline's replay still re-evaluates the source from the record. The decision and its numbers are a review stop, and the options that lose are recorded in LEARNINGS.md.

### 5. The `webcam.*` face source

A third member of `FaceSources`:

- `webcam`:
	- `present`: the direct camera's last whole-body frame is fresh
	- `tracked`: `present`, and the face block's scores clear a gate set by piece 4
	- `blendshapes`: the 52 ARKit columns in ARKit order, raw, all zero unless `tracked`
	- `measuredMask`: a bit per column the active method measures. A column outside it reads zero

The whole-body ratios measure nine columns: `eyeBlinkLeft`, `eyeBlinkRight`, `jawOpen`, `mouthSmileLeft`, `mouthSmileRight`, `mouthPucker`, `browInnerUp`, `browOuterUpLeft`, `browOuterUpRight`. A face model from piece 4b widens the mask to what it measures, up to all 52 for a blendshape model, and no name is added or removed either way.

Each ratio shape is a ratio of face-landmark distances normalized by a face size (the outer eye corner span or the nose length), so it does not depend on the user's distance, mapped from a neutral value to a full value. The neutral values are captured with the direct mode's neutral: `DirectNeutralPose` gains the face ratios behind their own present flag, and `captureNeutral` fills them when the face block's scores clear the gate. A neutral without them, one captured before this piece or with the face unseen, reads generic defaults, and the Webcam row prompts a recapture. The full values are defaults, so a profile function tunes them with `remap`. The values are raw: the source applies no dead zone and no smoothing, as the phone's columns have none. The ratios are computed from the recorded observation, so the timeline's replay re-evaluates them from the record and no sidecar is needed. They live outside the replayed stages, but the computation is a pure function of the frame, as face.md requires of a source.

`FaceExprInputs` gains the 52 `webcam.*` column names with `webcam.present` and `webcam.tracked`, the Expression Editor lists them, and the Phone stage gains a sibling Webcam row under Face and mouth with the live values. The wire does not change.

The Auto profile falls back to the webcam when the phone is not tracked. It is generated from the measured mask, so a column outside the mask keeps the rule it has now. Each webcam term below stands for the raw column inside Auto's dead zone and smoothing functions, with the dead zone sized from piece 4's rest jitter so that landmark noise at rest does not open a silent mouth through the max. Each ARKit column the webcam measures reads `select(phone.tracked, phone.X, select(webcam.tracked, webcam.X, <the current viseme fallback or 0>))`, and the VRM presets follow the same order: the blinks from the webcam's blinks, the gaze directions left at rest since the source has no gaze. On a mouth column both sources apply at once: the webcam's shape and the column's viseme fallback combine with max, so `jawOpen` reads `select(phone.tracked, phone.jawOpen, max(webcam.jawOpen, <viseme fallback>))` and a vowel preset reads `select(phone.tracked, phone.jawOpen, max(webcam.jawOpen, viseme.aa))`. Either source alone opens the mouth, and a source that is not tracked reads zero and drops out of the max. A user who wants another rule duplicates Auto and edits the functions.

### 6. The head from the face fit (gated)

A PnP fit of the face block's rigid points (the eye corners, the nose bridge and tip, the jaw line) against a generic 3D face in the direct camera's intrinsics gives the head rotation. It lands only if piece 4 shows it closer to the phone head than the ear and nose head, on the ears-visible sweep or on the headphones sweep alone.

The ear and nose head stays as the fallback. The solver uses the face fit while the face block is present and its scores clear the gate, and the ear and nose head otherwise: a hand over the face, a far turn, a backend without the block, a recording made before the block existed. The two heads have different rest frames, so `DirectNeutralPose` stores a head reference per method and each is measured against its own. A neutral that lacks the face fit's reference keeps the solver on the ear and nose head until a recapture. The existing head filter smooths the handover between them.

Either head goes through the existing filters and the 70 degree limit, and the phone head still wins while it streams. The fit's output rotation enters the head frame at one conversion point beside the current one (conventions.md).

### 7. Hand slot seeding from the whole-body hands

On a camera running the whole-body backend, a whole-body hand whose wrist and MCPs clear a score gate, with no tracked hand slot near it, becomes a `HandSearchHint`:

- center: the hand block's points' center
- direction: the wrist to middle MCP, as `HandRoi::fromLandmarks` takes it
- palm size: the wrist to middle MCP length scaled as the landmark ROI does

The hint takes the existing path: speculative, dropped on a first pass with no confident hand, rejected by `isSeedRedundant` on a tracked hand. `HandSeedStats` gains a source field so the funnel separates body hints from cross-camera ones.

A body hint can also evict. `applySearchHints` (`HandTrackingPipeline.cpp:245`) seeds free slots only and counts the rest as `skippedNoFreeSlot`, which is the case the plan.md item describes: the palm detector's scene false positives hold the slots and the real hand has to win one back. A body hint that finds no free slot replaces a slot that sits on neither whole-body hand, measured as the slot's box against each scored whole-body hand's box with a margin. A slot on a whole-body hand is never evicted, and a cross-camera hint never evicts. `HandSeedStats` counts the evictions.

This answers plan.md's "Direct mode's hand pipeline runs blind" item. Body hints run on the direct camera only. They need the direct mode's camera index to reach `CameraContext`, which today knows nothing of the mode. Multi-camera projects are out of scope (Decisions).

The gate from piece 4: the whole-body hand's box contains the stereo hand on most frames where the hand is in view, and in the tool's raw-frame re-run of the hand pipeline with hints on, slots return sooner after a dropout and the full-frame palm detector's scene false positives no longer hold a slot against a hint.

### 8. The hand side check from hand shape

The direct solver's agreement test compares one pixel, the slot's wrist against the body wrist. With both hands near each other (clasped, crossed, near the face) both wrists are within the agreement distance and the test cannot tell the sides apart. The hand block gives a shape match: the mean distance between the slot's 21 points and each whole-body hand's 21 points, in units of the hand's size. The side whose hand matches better, by a margin, is the slot's side.

The shape match decides the slot's side for the `bHandednessSwapped` report where the wrist distance does now, and nothing in the solve reads it. The entry gate's agreement (`bHandNear`) keeps the wrist distance, so the solve's output does not move. The wrist distance also stays as the report's fallback on a frame where the block is absent or a hand's scores are below the gate, which covers the other backend and recordings made before the block existed. The solve state also counts the frames a slot was rejected for sitting on the opposite wrist, so the report shows how often a label costs a hand. Correcting labels per frame is deferred on that count (Decisions).

The gate from piece 4: on the hands-crossing and clasping segments, the shape match agrees with the stereo fused side on more frames than the wrist test.

## Order and review stops

1. Pieces 1 to 3: the block, the recording key, the overlay and the dump. Self-contained, behavior-neutral, and the overlay alone shows the points on the rig.
2. Pieces 4 and 4b: the take, the tool and the blink evaluation. Review stop: the numbers decide which of pieces 5 to 8 go ahead, with what gates, and whether a face model is added.
3. Pieces 7 and 8, the hand pair. They are independent of the face and answer two existing plan.md items.
4. Piece 5, with the face model if 4b chose one, then piece 6 if its gate passed.

Each piece ends with its self-test and a replay check (`--replay-verify` on recordings made before the change, which must still load and verify).

## Tests

- `--test-replay`: a recording with the `wholeBody` key round-trips the block, and one without it loads with the block absent.
- A self-test feeding a synthetic `RtmPoseResult` of 133 points through `fillObservation` checks the block's indexing per side.
- `--test-faceexpr`: the 52 `webcam.*` names resolve, read zero unless tracked, and a column outside the measured mask reads zero while tracked.
- The Auto profile generated for a nine-column mask leaves every other column's function as it is today.
- The shape ratios on synthetic faces at two distances read the same.
- A neutral without the face ratios loads with them absent and the source reads the defaults.
- The direct solver on a synthetic user whose face block drops out hands the head to the ear and nose head against its own neutral, with no jump at the handover beyond the filter's band.
- The hint builder on a synthetic hand produces the ROI `HandRoi::fromLandmarks` would.
- `--test-seeding`: with both slots active, a body hint evicts the slot on neither whole-body hand, leaves a slot on one alone, and a cross-camera hint evicts nothing.
- The shape match on two synthetic hands at one wrist position assigns the sides, and falls back to the wrist distance when a hand's scores are below the gate.
- The direct solver's output on the recorded takes is unchanged by piece 8.

## Open questions for the reviewer

None open. The questions raised in both rounds are answered under Decisions.

## Out of scope

- The six foot points.
- Feeding the whole-body hand's finger points into the finger angles. The hand pipeline's own 21 points come from a dedicated crop at a higher resolution.
- Any change to the VMC wire.
- The multi-camera position pipeline's head, which the body solver and the phone already cover.
- Body hints in multi-camera projects.
- Per-frame hand label correction from the shape match.
- The shape match in the solver's entry gate.
