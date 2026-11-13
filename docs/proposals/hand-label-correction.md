# Proposal: per-frame hand label correction in direct mode

Design decided, 2026-11-12. Nothing here is built. This document proposes giving a hand slot on the direct camera the side the whole-body model's hands name for it, on each frame, before the hand is lifted and before the direct solver sees it. It starts from a measurement of when the solver turns a slot away on the other arm's wrist, which is piece 1 below and changes what the problem is. The reviewer's questions are answered (Decisions below), and the design reflects the answers.

## Goal

The direct solver takes a hand slot as the hand of an arm only when the slot's wrist is within half a shoulder width of that arm's body wrist (`bHandNear`, `src/Avatar/DirectRetarget.cpp:768`). A slot published under the wrong side sits on the other arm's wrist, so the solver turns it away. The arm stays placed from the body wrist, the palm and the fingers hold their last values, and the wrist depth scorer loses the hand lift's distance, for as long as the label is wrong. `SolveState::oppositeWristRejections` counts those frames. Live sessions of 2026-11-12 counted 468 and 1010 (left, right) over several minutes, 84 and 95 in 46 seconds, and 14 and 10 in 49 seconds.

`matchHandSideByShape` (`src/Vision/HandSideMatch.h`) already names a slot's side from the whole-body model's two hands, and was right on every one of the 2158 slot frames it decided on the rig take of 2026-11-12. It feeds the swapped-hands report alone. The goal is that a real hand under a wrong label keeps driving its own arm, and that nothing which is not a hand is taken as one because of it.

## Decisions

Carried from `docs/proposals/wholebody-landmarks.md`, piece 8 and its Decisions:

- The body model is the reference for a hand's side. It labels a hand by the arm it ends, where the hand model's score is view-dependent.

- The solver's entry gate keeps the wrist distance. The shape match does not become the gate.

- Correction is for the direct camera only. Multi-camera projects settle sides in fusion, from `rightProb` and never from the label.

- The correction was deferred until the rejection count existed. It exists, and piece 1 below reads it.

Decided at the review of this document:

- The correction is applied upstream, in the direct camera before the lift, and not inside the solver. New takes record the corrected hands with a flag per hand.

- The one-hand rule is built with the main rule, as part of `correctHandSides`.

- The correction is always on. There is no setting for it.

- The swapped-hands warning is reworded: the labels are reversed and the body model is carrying them.

- The slots that are not hands (a slot on the face or the wall beside a raised wrist) get their own plan item, with the shape bar as an entry gate as its candidate. This proposal does not change the gate.

- The correction is enough. The pipeline's stickiness and collision rule stay as they are.

## What is in place

- The hand pipeline publishes each slot under a side (`HandTrackingPipeline::resolveHandedness` and `publishHands`, `src/Vision/HandTrackingPipeline.cpp:514` and `:573`). The side is the classifier's `rightProb` made sticky: it flips only after `handednessSwitchFrames` (15) consecutive contradictions. Two slots claiming one side are ordered by decisiveness, and the loser is displaced to the other side. `TrackedHand::rightProb` carries the classifier's own opinion beside the published `side` (hand-tracking.md).

- `CameraContext::process` runs the hand pipeline (`src/App/CameraContext.cpp:379`), then the body tracker (`:396`), then the recording tap of the hands (`:412`), then the lift (`:443`). So this frame's body observation, fresh or held, is in hand before the hands are recorded or lifted.

- The lift keys its state by the published side: `LandmarkTo3D` keeps a palmar side memory and a PnP warm start per side (`src/Tracking/LandmarkTo3D.cpp:98`, `:260`), and `HandPoseModel::computePalmFrame` falls back to the label for the palmar sign when the geometry says nothing (`src/Tracking/HandPoseModel.cpp:151`). A memory is reset when its side loses its hand, not when the hand under that side changes.

- The direct solver reads `camera.hands[side]` and `camera.poses[side]` per arm. Its per-arm state (the hand entry gate, the palm filters and the palm hold, the palmar memory of `measureMonoFingers`, the finger filters, the held twist, wrist rotation and fingers) lives in `ArmState`, indexed by the arm's side (`src/Avatar/DirectRetarget.h:315`).

- The solver's label report takes each slot's side from `matchHandSideByShape` where both whole-body hands score and from the wrists otherwise (`DirectRetarget.cpp:783`). A second of frames with a slot on the other side and none on its own sets `SolveState::bHandednessSwapped` (`:1178`). The pipeline view degrades the direct solver on it, the Filters readout shows the Flip handedness setting beside a warning, and the guided setup's handedness step flips `tracking.flipHandedness` once on it (`SetupFlow::updateHandednessCheckPrompt`, `src/UI/SetupFlow.cpp:628`).

- `matchHandSideByShape` decides by a margin alone: the nearer whole-body hand wins when its mean point distance is at most 0.8 of the other's. It has no bar on the distance itself.

- A direct take records the hands as the lift consumes them and the body observation with its whole-body block, so the label, `rightProb`, the slot's 21 points and both whole-body hands are in the recording on every frame. `--gate-wholebody` (`src/Tests/ToolGateWholeBody.cpp`) re-runs the body model over a take's raw frames for a take recorded without the block.

- The checksum of a direct take hashes the lifted hand poses. The solver is not a checksummed stage. `--retarget-golden` compares the live solver against its frozen copy bit for bit.

## Design

### 1. When the rejections happen

Measured on the four direct takes of 2026-11-12 (4605 frames, 153.5 s, one camera, `flipHandedness` off, the body model at a divider of 2) from the recorded hands and whole-body blocks, and on the front camera of the rig take of 2026-11-12 (2760 frames, 92.1 s) with the body model re-run over its raw frames. The measurement re-applies the solver's rule to each recorded frame: a slot is turned away when it is tracked, not within half a shoulder width of its own side's usable body wrist, and within that distance of the other side's. On the direct take that matches the 84 and 95 session it counts 84 and 88 over the 42 s recorded. It was made with scratch scripts outside the repository. Piece 2 puts it in the tool.

What the hands were doing is read from the two body wrists: clasped (within half a shoulder width of each other), crossed (the left wrist past the right along the shoulder line by 0.15 shoulder widths), close (half to one shoulder width apart), free and apart, or one body wrist unseen. What the slot was is read from the whole-body hands: the mean distance over the 21 points to the hand of the wrist it sits on, in the slot's palm lengths, with half a palm length as the bar for being on that hand.

The 207 slot frames turned away on the direct takes (101 left, 106 right):

| The hands | Frames in that state | Turned away | On the other arm's hand, both whole-body hands scoring | On the other arm's hand, the label's own hand unscored | On no hand |
|---|---|---|---|---|---|
| Clasped | 64 | 0 | | | |
| Crossed | 0 | 0 | | | |
| Close | 72 | 35 | 35 | 0 | 0 |
| Free and apart | 1830 | 95 | 82 | 0 | 13 |
| One body wrist unseen | 1315 | 77 | 4 | 14 | 59 |
| Total | | 207 | 121 | 14 | 72 |

The remaining 1324 frames have neither wrist usable, and nothing is turned away on them. On the rig take 38 slot frames were turned away (19 and 19): 28 apart, 4 close, 6 with a wrist unseen, none on its 24 clasped and 10 crossed frames. 4 were on the other arm's hand and 34 on no hand.

What that says:

- **No slot is turned away while the hands are clasped or crossed.** Clasped wrists are beside both slots, so both pass the gate. Crossed arms keep each hand on its own wrist. The gain the earlier proposal expected from correction, kept hands while the hands cross or clasp, is not where the rejections are.

- **A clasp costs a wrong hand, not a dropout.** On the one direct take with a clasp, 34 slot frames were accepted under the wrong side (30 clasped, 4 close): the slot's shape names the other arm's hand while its wrist is within the gate of its label's arm. So avatar.md's statement that a wrong label costs a dropout and never a wrong arm does not hold with the wrists together.

- **135 of the 207 are a real hand under the wrong side, and the classifier is not the cause.** On 128 of the 135 the slot's own `rightProb` names the true side and the published label contradicts it. The label is held by the pipeline's 15-frame stickiness after the slots change hands in a clasp or a pass, or by a side collision. On 7 the classifier names the wrong side. On 92 of the 121 with both hands scoring the other slot was turned away on the same frame: both labels crossed.

- **The runs are short and end at the stickiness.** The 135 fall in runs of at most 15 frames, the pipeline's switch count. The typical episode is the hands parting after a clasp: both slots turned away for 15 frames, half a second, then both labels flip back.

- **72 of the 207 are not a hand of that arm at all, and turning them away is correct.** They are a slot on the face beside a wrist raised to the head (one run of 36 frames) and a slot on a picture frame beside a wrist behind the head. The solver turns them away only because their label happens to be the other side. Under the matching label the wrist gate would have taken them.

- **The margin alone names a side for things that are not hands.** On 19 of those frames `matchHandSideByShape` decides the other side for a slot more than half a palm length from both whole-body hands, 17 of them more than two palm lengths, because one distance is still under 0.8 of the other. A correction on the match as it stands would hand the solver a wall object under the label that passes its gate.

- **A side collision can cost the real hand.** On 9 of the 14 frames in the fourth column the slot on the face claimed the right label on its decisiveness and displaced the real right hand to the left, with the left hand out of view and its whole-body hand unscored.

The distance separates the two kinds cleanly on the take with the most rejections: of its 172 turned-away slot frames 102 are within 0.5 palm lengths of the nearest whole-body hand, 1 is between 0.5 and 1, and 69 are beyond 1. Slots the solver accepts sit at 0.05 to 0.08 palm lengths p50 and 0.08 to 0.29 p90 across the four takes.

### 2. The measurement in the tool

`--gate-wholebody` gains a label report over the live-cadence pass, the numbers of piece 1 from the tool and not from a scratch script:

- the slot frames turned away per side, split by what the hands were doing and by what the slot was, as the table above
- for the slots on the other arm's hand, whether `rightProb` names the true side, and the other slot's state
- the run lengths
- the slot frames accepted under the wrong side
- the same counts with piece 3's rule applied to the recorded slots, beside the counts without it, with the slot frames it moved and the side changes of a tracked slot before and after

It reads the recorded whole-body block when the take has one and the re-run model's otherwise, so a direct take needs no raw frames for it. With extrinsics it adds the slot's fused side as truth, as the existing hands report does. Nothing is gated. Review stop: the tool's numbers against this document's.

### 3. The rule

Two changes to `src/Vision/HandSideMatch.h`, both pure functions of one frame.

`matchHandSideByShape` gains a bar: the nearer whole-body hand is the slot's only when its distance is also at most `k_handShapeMaxDistance`, 0.5 palm lengths. A slot nearer one hand by the margin and farther than the bar from both has no side. The report reads the same function, so it stops counting a wall object as a mislabelled hand.

`correctHandSides` takes the frame's two published hands and the whole-body block and returns which side each goes to:

- A slot the match decides goes to its matched side.

- A slot the match does not decide because its own label's whole-body hand does not score, and which is within the bar of the other whole-body hand, goes to that hand's side. This is the one hand in view: a real hand displaced to the wrong side with the other hand out of frame. On the four direct takes and the rig take it applies to the 14 slot frames of piece 1's fourth column and to no other, and to none where the slot is beside its own label's wrist.

- Any other slot the match does not decide keeps its label.

- With two slots, a decided slot takes its side and the other takes the side left over. Two slots decided for the same side are two slots on one hand: the nearer takes the side and the other is displaced, as the pipeline's own collision rule displaces one.

- A block that is absent, both hands under the score gate or a slot with no size decides nothing, so the planar backend and a take from before the block behave as today.

The rule holds no state between frames. A hold (a slot keeps its last matched side for 500 ms while the match is undecided) was tried on the takes and bought little without the one-hand rule: 90 slot frames turned away against 95, and 49 side changes against 55. It stays out.

The rule on the recorded slots, with the solver's gate applied after it:

| | Direct takes, today | Direct takes, with the rule | Rig take, today | Rig take, with the rule |
|---|---|---|---|---|
| Slot frames turned away | 207 | 82 | 38 | 35 |
| Of those, on the other arm's hand | 135 | 10 | 4 | 0 |
| Of those, on no hand | 72 | 72 | 34 | 35 |
| Frames with a left hand | 1983 | 2045 | 2304 | 2304 |
| Frames with a right hand | 1888 | 1951 | 2060 | 2063 |
| Slot frames moved to the other side | | 190 | | 8 |
| Side changes of a tracked slot | 61 | 46 | 8 | 2 |

The 10 left on the direct takes are frames where two slots sit on one hand and the displaced one is turned away, which is right. Without the one-hand rule the direct takes read 95 turned away, 23 of them on the other arm's hand. No slot on no hand is taken on any take. The 34 slot frames accepted under the wrong side are all moved to their own arm.

### 4. Where it applies: in the direct camera, before the lift

`CameraContext::process` applies `correctHandSides` on the direct camera after the body tracker and before the recording tap and the lift, swapping the two entries of `result.hands` where the rule says so. The vision thread already tells each `CameraContext` whether it is the direct camera, for the body hints. `TrackedHand` gains `bSideCorrected`: the entry arrived under the other label. The pipeline's slots are not told: the slot's own sticky side stays as it is, and only what is published moves.

What this changes downstream, and what it does not:

- **The lift.** The hand is lifted under its true side. Its palmar side memory and PnP warm start stay with the physical hand through a label flip, where today both labels crossing hands each side's memory the other hand's palm normal. The label fallback for the palmar sign reads the true side.

- **The solver's entry gate.** Unchanged. The slot arrives in the entry of its true side and is tested against that side's body wrist. An arm already placed takes the hand back the frame it is near, as today. An arm from rest still needs its 3 frames.

- **The earned hold and the per-side measurements.** Unchanged, and that is the point of correcting upstream: `ArmState` is indexed by the arm, and the hand that reaches it is now that arm's hand on every frame the rule decides. The palm filters, the palm hold, the finger filters and the solver's palmar memory no longer reset at each label flip, and the held twist, wrist and fingers are held only when the hand is really gone.

- **The wrist depth.** The lift's distance stays in the scorer through the episode. plan.md's lift item records that the scorer is bistable without it.

- **Recordings.** A new take records the corrected hands, with `bSideCorrected` as an optional key per hand that reads false when missing. `k_formatVersion` does not move. The recorded label can be recovered from the flag, so a take stays reversible. Existing takes are untouched: `--replay-verify` passes on them with no divergence, since nothing in a replayed stage moves.

- **Everything else that reads the direct camera's hands.** The camera view's overlay, the dump, the passthrough result and the no-avatar stream all show the true side.

The alternative considered, and turned down at the review, is to correct inside `DirectRetarget::measure`: each arm picks the slot whose matched side is the arm's. It keeps recordings as the pipeline published them, so every existing take replays through it. It fits less well. The slot was already lifted under the wrong label, so its palm comes from a memory keyed to the other side, which after a clasp holds the other hand's palm normal: the palm can come back mirrored on exactly the frames the correction is for. The solver would also need the side passed around the label in `measureMonoFingers`, and the overlay, the dump and the stream would still show the wrong side. Correcting the pipeline's own label was also considered: the pipeline runs before the body tracker, so it would act on the previous model frame, and its stickiness guards multi-camera views this proposal does not touch.

The correction has no setting. The A/B is the tool's: `--gate-wholebody` and `--gate-direct` run a take with and without the rule.

### 5. The report, the counts and the Flip handedness prompt

The report keeps answering one question: are the labels the wrong way round as a standing condition, which is what `tracking.flipHandedness` fixes. It is read from the labels as the pipeline published them, so the correction does not hide it:

- A corrected slot is evidence of a swap. A slot the match decides for its own label is evidence against. An undecided slot falls back to the wrists, as today.

- `bHandednessSwapped` still needs a second of the first with none of the second. A clasp episode is half a second, so it does not raise the report.

- With the flip setting wrong, every decided frame is corrected, the hands keep working while both are in view, and the report still comes up after a second. The pipeline view and the Filters readout still show it, and the guided setup's step still flips the setting once. The prompt is still worth acting on: the correction needs a whole-body hand that scores, and the lift's and the pipeline's own uses of the label stay wrong.

- `oppositeWristRejections` keeps counting the slots the gate turns away on the other arm's wrist. With the correction on it becomes the count of what the correction did not catch. `SolveState::sideCorrections` counts, per side, the frames an arm took a corrected slot, and the Direct solver stage shows both.

- The guided setup's match test (both hands seen and no swap) is unchanged.

- The warning's text changes, since with the correction a swapped label no longer costs the hands. The pipeline view's reason, the Filters readout's warning and the guided setup's swapped line say that the hand labels are reversed, that the body model is correcting them, and that Flip handedness fixes it. The Japanese strings change with them.

## Order and review stops

1. Piece 2, the label report in `--gate-wholebody`. Review stop: its numbers on the four direct takes and the rig take against piece 1's.
2. Piece 3, the bar and `correctHandSides` with the one-hand rule and their self-tests, not wired. The tool's with-the-rule columns come from the real function here.
3. Pieces 4 and 5: the direct camera applies the rule, the flag rides the hand and the recording, the solver's report and counts read it, the readout and the reworded warning. Review stop: the replay checks below.
4. The live check, then the reference docs (avatar.md's entry gate and report paragraphs, hand-tracking.md's handedness paragraph, testing.md, debugging.md's recorded fields) and LEARNINGS.md for the measured split of piece 1.

## Tests

Self-tests:

- `--test-seeding`, beside the existing shape cases: a slot nearer one hand by the margin and farther than the bar from both has no side, and a slot inside the bar still does.
- `correctHandSides` on synthetic hands:
	- two slots under crossed labels are swapped and both flagged
	- a single slot under the wrong label moves to the other entry
	- two slots matching one hand: the nearer keeps the side and the other is displaced
	- one decided slot and one undecided: the undecided takes the side left over
	- a slot whose own label's hand is unscored, within the bar of the other hand, moves to it, and beyond the bar it stays
	- an absent block, both hands under the score gate and a slot with no size change nothing
	- a slot far from both hands under either label is not moved
- `--test-direct-retarget`, cases (k2) and (k3) extended: with corrected slots in the frame both hands are seen and nothing is counted as turned away, the report still comes up after a second of corrected slots and not before, one uncorrected slot the match decides for its label ends it, `sideCorrections` counts, and with the block absent the cases read as they do today.
- `--test-replay`: a recording with `bSideCorrected` round-trips it, and one without it loads with the flag false.
- `--test-retarget-golden`: every case passes unchanged. With no slot flagged the solver's output is the frozen copy's bit for bit.

Replay checks on the recorded takes:

- `--replay-verify` on every existing take, direct and rig: verifies with no divergence.
- `--gate-wholebody` on the four direct takes of 2026-11-12 and the rig take, with the rule applied to the recorded slots. The gate for wiring it in:
	- slot frames turned away on the other arm's hand fall from 135 to 10 on the direct takes
	- no slot on no hand is taken on any take
	- neither side's hand frames fall on any take
	- the side changes of a tracked slot do not rise
- `--gate-direct` on the direct takes with the rule applied ahead of the lift: its twist jumps and mirrored-palm frames per hand do not rise. This is the check on the palm through a corrected episode.
- A take recorded on the new build verifies in replay and carries the flag.

Live checks, direct mode on the webcam with an avatar loaded:

- Clasp the hands and part them, ten times. Both hands follow from the first frames after parting, where today they freeze for half a second. The Direct solver stage's turned-away counts stay near zero and the corrected counts rise.
- Pass one hand over the other and cross the arms. Each hand stays on its own arm.
- Raise one hand to the head and behind it. The slot on the face is not taken as the hand.
- Switch Flip handedness to the wrong value. The hands keep working with both in view, the report and the warning come up after a second, and the guided setup's handedness step flips the setting once and passes.
- Drop one hand below the frame under the wrong flip. The other hand stays on its own arm while its whole-body hand scores, which is the one-hand rule.
- With the flip wrong, read the reworded warning on the pipeline view, the Filters readout and the guided setup, in English and Japanese.
- A VMC receiver shows the same hands.
- Read the two counts after a session of the length of the 2026-11-12 ones (468 and 1010 over several minutes).

## Open questions for the reviewer

None open. The six questions raised with the draft are answered under Decisions.

## Out of scope

- The shape match as the solver's entry gate. The slots that are not hands are their own plan.md item: 72 of the 207 turned-away slot frames on the direct takes are a slot on the face or the wall, turned away only because the label is the other side, and on the rig take 133 of 4364 accepted slot frames sit beyond the bar of both whole-body hands, so the bar as a gate has a cost to measure first.
- Any change to the hand pipeline's own labels, its stickiness or its collision rule.
- A setting for the correction.
- Multi-camera projects.
- The default of `tracking.flipHandedness`, which stays plan.md's open question.
- The slots turned away because their own body wrist is unseen or out of frame. The arm needs that wrist.
- Any change to the VMC wire.
