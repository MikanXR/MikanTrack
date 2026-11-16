# Experiments and Learnings

MikanTrack went from empty repo to a working multi-camera hand tracker between late July and mid August 2026, developed almost entirely in AI pair-programming sessions (Claude Fable 5 and Opus 5, see `TOKEN_STATS.md`). Most major experiments ran on their own branch and were merged when they won or parked when they lost. This document records what was tried and what each experiment taught us. It is a companion to `README.md`, which describes what the app IS. This describes how it got that way, including the dead ends.

## Timeline at a glance

| Branch | When | Experiment | Outcome |
|---|---|---|---|
| (main) | Jul 29-30 | ONNX Runtime + DirectML instead of the MediaPipe framework | Foundation, kept |
| (main) | Jul 30 | BlazePose for elbow tracking | Removed, never fired on overhead views |
| multi_camera | Jul 30-31 | Multi-camera fusion of landmark positions | Kept, then superseded by pose-level fusion |
| multi_camera_body | Jul 30 | Hand-seeded body-pose ROI fallback | Failed live, removed |
| (main) | Jul 31 | Parametric hand model (palm transform + finger angles) | Kept, became the core representation |
| (main) | Aug 1 | Diagnostic dump system + measured-jitter confidence | Kept, changed how every later bug was fixed |
| multi_camera_3d | Aug 1 | Stereo triangulation as the primary 3D lift | Kept |
| rtm-pose | Aug 1 | RTMPose hand landmarks as 2D refinement | Won its A/B, parked anyway (cost/benefit) |
| stereo_camera | Aug 2 | RealSense D455 hardware depth | Kept as an option; deleted Aug 16 once the estimator's mono prior covered its niche |
| imu | Aug 2-9 | Wrist IMUs (Joy-Cons) for forearm orientation | Kept |
| image_metrics | Aug 9-10 | Image-quality diagnostics + record/replay | Kept |
| extrinsic_calibration | Aug 10-11 | Charuco-board extrinsics, all cameras in one session | Kept |
| (main) | Aug 11 | Depth A/B tool, marker-scale discovery, bone calibration | Kept |
| seed_fix | Aug 12 | Cross-camera seeding instrumentation + fixes | Kept, seeding now unconditional |
| body_pose | Aug 12-14 | Opt-in per-camera body pose (elbows, shoulders, head) | Kept; landmark model swapped BlazePose -> RTMPose |
| biomech | Aug 15-16 | Angle-space multi-view state estimator + biomechanical priors | Live-verified, made permanent; superseded holds/hysteresis/one-euro deleted |
| mono-camera | Oct 30 | Single-camera baseline: tri-camera recordings replayed as the front camera alone | Measured; gates failed on a front-camera left-hand depth bias and mono palmar flips, which open the single-camera work |

## 1. ONNX Runtime + DirectML instead of the MediaPipe framework

The obvious path was building Google's MediaPipe C++ framework. It was rejected before writing code: on native Windows it is CPU-only and requires a Bazel build. Instead we took the OpenCV Zoo ONNX conversions of the MediaPipe models and ran them through ONNX Runtime with the DirectML execution provider, reimplementing the two-stage detector-to-landmark graph (SSD anchors, weighted NMS, rotated crops, ROI tracking) in C++ by porting the opencv_zoo Python demos.

The port was validated numerically against the Python reference to 0.0002 px agreement before any live testing. That upfront rigor paid off repeatedly: every later tracking bug could be assumed to be OUR logic or OUR calibration, never the inference stack.

Learned: porting a model graph is far cheaper than binding a framework, if and only if you cross-validate numerically against the reference implementation first. One trap worth recording: `cv::copyMakeBorder` on a ROI view silently reads pixels outside the ROI unless `BORDER_ISOLATED` is set.

## 2. BlazePose elbows: a model outside its training distribution

We wanted elbow positions for client-side arm IK, so we ran BlazePose alongside the hand models. Its person detector simply never fires on a top-down desk view of a torso-less human. A fallback that seeded the pose ROI from the tracked hands (branch multi_camera_body) also failed live. BlazePose was removed entirely; purely geometric elbow estimates (forearm direction from hand orientation, length from hand scale, table-plane clamp) did better, and eventually even those were dropped in favor of streaming just the palm transform and letting the client solve arms with Two-Bone IK. The wrist IMU work later restored a real forearm measurement.

Learned: a model trained on upright people contributes nothing to an overhead rig, no matter how it is coaxed. When the sensor geometry is outside a model's training distribution, simple geometry beats the model. Also: not estimating something is a valid design. Deleting the elbow estimate made the OSC contract more useful, not less.

Postscript (branch `body_pose`, Aug 12-13): a third, front-facing camera was added to the rig, which puts an upright person squarely back inside the model's training distribution. BlazePose returned as an OPT-IN PER-CAMERA stage - the overhead cameras leave it off and pay nothing - now feeding measured elbows, shoulders and head pose. The removal was still correct: the rejection was of the model on that geometry, not of the model. Three lessons landed from the first two live sessions.

First, the elbow could not be re-solved on every fused frame: the pose models run near 10 Hz while wrists update at 30 Hz, so re-intersecting a held observation against a moved wrist slid the elbow along a ray cast up to 200 ms earlier. A repeated observation now holds its forearm direction and lets the elbow ride the fresh wrist.

Second, confidence-from-measured-jitter has to be normalized by the sampling interval: the hands' plain-distance residual assumes a fixed rate, and applied to a variable ~10 Hz signal it mostly measured how long the gap was, scoring every elbow at 0.01. Dividing by dt squared turns it into an acceleration, which means the same thing at any cadence.

Third, and the one that was actually causing the visible popping: anchoring the elbow to a forearm-length sphere around the wrist leaves a camera ray intersecting that sphere TWICE, roughly 400 mm apart, and we were choosing between them with the model's own elbow-versus-wrist depth. That sign flips on 28-42% of model frames because its magnitude (24-46 mm median) sits inside its own noise, so the elbow teleported at random. Raising a threshold on it does not help - even the top 5-19% of samples by magnitude still flip 10-17% of the time. Continuity decides it instead (the forearm cannot swing 400 mm between model results, giving 0-1% flips), with the model depth demoted to seeding the first solve and an independent shoulder-length check as a sustained-evidence escape hatch. The general lesson: when a geometric solve has a discrete ambiguity, resolve it with the strongest signal available, and prefer temporal continuity over a per-frame measurement whose noise is comparable to the quantity being measured.

Then the model itself lost. Even with the elbow solve fixed, the landmarks under it were poor, and the reason was structural rather than tunable: BlazePose is holistic, deriving its own crop from a hip center and a "full body" point. A person truncated at a desk has no correct crop, so it fabricated a lower body and dragged the arms with it, while scoring the shoulders 0.9998 as they jittered 30-40 px. The landmark model was swapped for RTMPose-m (body7), which is top-down: this app supplies the person box, and each of the 17 COCO keypoints is scored independently, so joints outside the frame read as low confidence instead of being invented. Same frames, same session, the two measured: 99% of frames usable against 70%, elbow 2D step medians of 6.7 and 7.4 px against 53 and 74 px, nose 3.9 px against 23 px. BlazePose was then removed outright. Two things made that decision cheap. Raw frame recording (opt-in, off by default) preserves the model's actual input, so "would a different model have done better" became a replay measurement rather than an argument. And the box handed to the top-down model has to be built from the person detector's KEYPOINTS, not its own box, which is a face box - feeding that to RTMPose cropped the arms away.

Learned: a confidence score is a statement about the model's own crop, not about the answer. When a model owns its region of interest, its assumptions about the scene are non-negotiable, and a scene that violates them cannot be fixed downstream.

## 3. Multi-camera fusion and the hand-identity wars

Adding a second camera was the single most consequential feature, and hand identity (which physical hand is left vs right) was its hardest problem. It took roughly five distinct diagnoses, each driven by a diagnostic dump of a live failure, to get stable:

- Per-camera left/right labels are unreliable whenever a camera sees only one hand. Sides must be assigned at the fusion level by clustering observations in world space. Trusting per-camera labels collapses both physical hands into one slot.
- After a clap, both of one camera's tracking slots converge on the same physical hand and confidently track it, leaving the separated hand untracked with no free slot. Duplicate-slot detection (hand-box IoU over consecutive frames) is required.
- Monocular depth error lies along the camera's view ray and reaches 15-25 cm on this rig, comparable to the separation between hands. Position alone therefore CANNOT solve cross-camera correspondence when hands are near each other. Clustering must be ray-aware, and pairing must be a joint assignment that weighs label votes, not greedy nearest-neighbor.
- Slot labels get physically swapped by ROI hijacks. Labels are bookkeeping, not evidence. The classifier's raw score on the hands a camera sees well IS evidence, even while its labels are wrong.
- MediaPipe's presence score measures "is a hand here", never "is this the left or right hand" and never "is this pose good". Resolving a side collision by presence displaced a decisively-classified hand on what amounted to noise. Decisiveness of the handedness score is the right tiebreak.

Learned, beyond the mechanics: every one of these was diagnosed from a recorded dump, not from staring at live behavior, and two early hypotheses (a mirrored-camera theory, a "fusion is broken" report that was actually a preview-overlay artifact) were disproven by dump data. The general rule that emerged: per-camera overlays show per-camera state; always check the fused output before believing a failure report.

## 4. The parametric hand model

The first fusion approach blended 21 3D landmark positions per hand across cameras. When cameras disagreed about articulation, blending positions distorted bone lengths and produced hands no skeleton could make. The replacement, modeled on Ultraleap's representation, is a palm transform plus per-finger bend angles over a fixed skeleton, with fusion operating at the pose level (position blend, hemisphere-aligned quaternion blend, angle blend, later angle SELECTION). Forward kinematics from the streamed values is what clients rebuild, and the app's 3D view renders exactly that reconstruction.

Learned: choose a representation in which every reachable value is a valid hand. Blending is then safe by construction. The sign conventions (which way is positive curl, what zero means) were the genuinely hard part and only became stable once round-trip tests (extract angles, FK back, compare: 2e-7 rad) and an end-to-end FK reprojection metric pinned them down. That reprojection metric, kept as a live per-frame value, later became the key diagnostic separating "the angles are wrong" from "the hand really is bent".

## 5. Confidence is measured, not reported

A recurring, load-bearing negative result, found independently three times:

- MediaPipe presence does not correlate with pose quality. Binning frame-to-frame jitter by presence showed 12 mm median jitter at presence 0.85-0.95 and 26 mm at presence above 0.95. An edge-on, dim hand keeps presence near 0.9 while its depth solve is ill-conditioned.
- RTMPose's own confidence scores ran around 0.3 on landmarks that were WINNING the measured reprojection contest against MediaPipe's.
- Calibration reprojection error looked clean (0.42 px) on an extrinsics solve carrying a 1-degree orientation error that warped reconstructed geometry by 2.2 mm of corner spacing. Reconstruction-scale metrics catch what reprojection metrics cannot.

The fix in all cases was to measure quality directly: per-camera jitter (constant-velocity residual) drives confidence, triangulation residual drives stereo trust and can VETO a pairing, and FK reprojection measures articulation fidelity. Model confidence outputs answer "did the model find something", never "is the answer good".

## 6. The models are view-dependent and the hand is not canonical

Two related discoveries about MediaPipe's metric hand output:

- For the same physical hand at the same instant, two cameras' "world landmarks" disagree about articulation by 25-41 degrees. Rest-pose calibration therefore has to be captured per camera, and the bias was later confirmed to be pose-dependent, not a constant offset.
- MediaPipe's canonical hand model does not match a real hand's proportions. Measured against stereo triangulation of the user's actual hand, the model's proximal phalanges were short by a factor of about 1.8, and the model has an index distal phalanx longer than its proximal, which no human hand has. Since solvePnP rescales the model on a single bone and then fits all 21 points, a proportionally wrong model biases every monocular depth estimate. Per-bone skeleton calibration from stereo data fixed it, and immediately lifted the cross-camera seeding success rate from 32% to 88%.

Learned: a "metric" model output is an opinion, not a measurement. Calibrate anything personal (rest pose, bone lengths, hand scale) from your own sensors. Related human fact: a hand resting on a keyboard genuinely holds 20-50 degrees of MCP flexion, so a rest-calibrated zero can never be a flat hand, by anatomy rather than by bug.

## 7. Stereo triangulation as the primary 3D lift

A study of the SystemAnimatorOnline project prompted a rework of the 3D pipeline: triangulate the 21 2D image points across cameras as the PRIMARY path, and never let network-estimated depth into the 3D solve when two views are available. The triangulation reprojection residual doubles as a correspondence test (high residual vetoes the pairing as two different hands) and as a confidence input. Monocular PnP became strictly a fallback, and articulation from multiple cameras is selected (with hysteresis) rather than blended.

Learned: when you have two calibrated views, classical geometry beats learned depth in accuracy, and its residual is a trustworthy self-diagnostic, which learned depth never provides. Splitting smoothing into separate palm and finger-angle filters (different cutoffs) came from the same study and removed a long-standing tuning tension.

## 8. RTMPose: winning the A/B and losing the cost-benefit

RTMPose-m hand landmarks were integrated as an optional 2D refinement over MediaPipe's, with MediaPipe keeping detection, ROI tracking and handedness. The A/B methodology matured over three rounds:

- Round 1 mixed RTMPose and MediaPipe points within a single triangulation pair. The two models place joint centers by different conventions, and the mismatch read as correspondence error. Comparisons must keep sources pure per solve and select per frame by measured residual.
- Round 2 appeared to show MediaPipe winning, but the confidence gate (0.35) sat mid-distribution for RTMPose and censored most of its frames: gate-survivorship bias. One camera-hand combination never passed the gate at all.
- Round 3, with the gate floored, showed RTMPose decisively better: it won 84-96% of per-frame residual contests, with the winning-side median residual at 1.84 px.

RTMPose was parked anyway. It roughly tripled per-camera inference cost (6.5 to 23 ms), and the user judged the tracking improvement too modest to pay for. The branch survives if wanted later.

Learned: an experiment can succeed technically and still lose on cost. Also two transferable A/B rules: audit what a quality gate censors before trusting a comparison, and expect different models to disagree on landmark conventions even when both are "the same" landmarks.

## 9. Calibration: where the silent errors live

Camera calibration produced more subtle bugs than any other subsystem:

- OpenCV's `CALIB_RATIONAL_MODEL` with k3/k4/k5 fixed leaves k6 free: a lone r^6 denominator term that fits sub-pixel while producing a degenerate undistortion (reported FOV of 180 or 8.6 degrees). The same latent bug existed in MikanXR. Plausibility gates on recovered FOV are now part of the wizard.
- Focal length is unobservable from fronto-parallel board views. The solver pins fx at exactly width/2 (a suspiciously round 90-degree FOV) instead of failing. The wizard now requires tilted captures with measurable keystone.
- Recalibrating at the same resolution kept stale undistortion maps, so a good calibration LOOKED broken. Rebuild derived state unconditionally on recalibration.
- Extrinsics captured per camera in separate sessions, with the marker nudged between, produce a silent world-frame disagreement that masquerades as tracking noise. The wizard was rewritten to calibrate all cameras in one session against one board placement.
- The biggest one: world scale was 30% short for most of the project, because extrinsics had been solved against a printed aruco marker ASSUMED to be 100 mm that actually measured 130 mm. Every internal quality metric validated the wrong scale, because they all compared the reconstruction against the same assumed size that produced it. The error was caught only when hardware depth (factory-calibrated, board-independent) disagreed with the stereo world by a consistent factor, and settled by the user measuring a hand bone with a physical ruler: ruler 85 mm, depth 82 mm, stereo 68 mm. The two board-free measurements agreed; the board-derived one was wrong. Fixing the marker size snapped triangulated bone lengths to within 1.5% of the ruler.

Learned: calibration quality metrics that share an assumption with the calibration are self-referential and will happily validate a wrong world. Keep at least one measurement path that does not depend on the calibration (hardware depth, a physical ruler), and when two internal numbers disagree, reach for it immediately. Also, a diagnosis arc from this incident: the depth camera was first accused of reading 24% long, then the desk was accused of contaminating depth samples, and both theories fit the histograms. Only the independent measurement discriminated. Two wrong-but-plausible readings preceded the right one, and the dump-everything habit is what kept each theory cheaply testable.

## 10. Hardware depth: real value, initially masked

A RealSense D455 was added as a backend (dynamically loaded SDK, depth-resolved palm anchors replacing PnP on its camera). Three results:

- A single overhead D455 replacing the two-webcam rig produced mushier fingers. Depth helps the palm; articulation still needs two angular viewpoints. The two-camera rig stayed.
- The first depth on/off A/B (a replay tool that re-runs a recorded session both ways over identical motion) showed depth buying almost nothing. That result was an artifact of the 30% world-scale error above, which was punishing the depth path for being right.
- On the corrected rig, the same A/B showed depth's real contribution: about 17% more tracked frames, 5 points of stereo rate, 15% confidence, and 17-19% lower p90 jitter, concentrated in the 10-15% of frames where fusion falls back to one camera.

The rig nevertheless moved to dual wide-FOV webcams. The user's live observation that a 90-degree FOV camera "surprisingly" improved tracking held up: field of view keeps both hands in both frusta, which raises the stereo rate, and stereo frames never read depth anyway. After bone calibration and the seeding fixes, the dual wide-FOV rig hit 93.3% stereo share against the RealSense rig's ~95%, without the depth hardware.

Learned: quantify what a sensor buys before paying for it, on a correctly calibrated rig, with identical-motion replays. And FOV is upstream of almost everything in a stereo hand tracker: a frame that keeps both hands in both views avoids the entire monocular fallback problem that depth existed to soften.

Postscript (Aug 16): with the state estimator permanent - whose anisotropic mono prior softens the same monocular-depth problem the sensor targeted - the RealSense backend, the depth-sampling path, and their recording/replay plumbing were deleted entirely under the settled-experiment rule. The measured verdicts above are the record of what the hardware was worth.

## 11. Wrist IMUs: model the joint you actually measure

Joy-Cons on wrist straps feed a 6-axis orientation ESKF per wrist. Design decisions that mattered:

- A wrist-strap IMU rides the FOREARM, not the palm. Modeling it as forearm orientation, with the wrist joint recovered as palm-relative-to-forearm, restored the forearm information vision measures poorly and enabled elbow visualization again.
- Orientation only. Accelerometer double-integration diverges, and vision already delivers millimeter-level position.
- Without a magnetometer, yaw is unobservable, and (same physics) so is gyro bias about the gravity axis. The filter test suite documents this observability structure explicitly. Vision anchors yaw through an anisotropic measurement update that corrects ONLY yaw, because vision measures the palm and a full-orientation correction would fight real wrist motion through the unknown joint between the sensors.
- When the filter owns orientation, the downstream one-euro smoothing for that quantity is disabled. Cascading filters double-smooths and adds lag.
- Mounting calibration evolved from "hold a pose" to solving the strap orientation from a twist-and-curl motion, scored against real forearm twist. Held poses under-constrain the solve.

Measured along the way: Joy-Cons stream at exactly 200 Hz over Bluetooth HID, resting gyro bias on one unit implied 114 degrees/minute of yaw drift (stable, hence estimable), and the left and right units mount their IMUs with opposite orientations, which is why mounting calibration cannot be hardcoded.

## 12. Look at the image: quality metrics for the WHY layer

By August the project had good outcome metrics (jitter, residuals, reprojection) but nothing explaining WHY a camera was bad. Per-hand image statistics (luminance, clipping, contrast, background separation, sharpness, sensor noise, flicker detection), computed on the exact pixels the model consumed, filled that layer. First live use attributed one camera's 4x worse jitter to dim exposure and 2x sensor noise; turning up a ring light halved that camera's jitter p90 in a controlled before/after.

Methodology lessons: raw Laplacian variance scores a noisy frame as "sharp", so sharpness (after median filtering) and noise (the median residual) must be split into separate metrics. And free-form hand motion cannot A/B jitter, because the jitter metric reads real acceleration as noise; a scripted hold-still test exists for exactly that comparison, while the image metrics are motion-independent.

## 13. Record/replay: the highest-leverage tool in the project

Deterministic recording of all post-inference inputs, with bit-exact offline re-simulation (checksummed, verified per frame), turned live-only tracking bugs into desk work:

- The first replay-driven fix: a stereo "pop" the user reported as frame 348 of a recording was diagnosed in minutes (a frame stutter pushed one camera past the staleness window, dropping fusion to the wrong monocular camera) and the fix was validated by replaying the SAME recording: a 212 mm pop became 5 mm.
- The stutter itself traced to something nobody suspected: IMU device rescans running Windows HID enumeration (measured 237 ms) on the vision thread every 151 iterations. It was never USB. The periodicity was visible only because recordings made the loop timing inspectable. Discovery moved to a worker thread, and a per-phase hitch watchdog now names any future stall.
- What-if replay (same recording, edited config, different extrinsics, depth on/off) became the standard A/B harness. Both the depth verdict and the marker-scale discovery ran on it.

Learned: in a real-time perception system, determinism is a feature you build early and lean on constantly. The prerequisite discipline (no wall clock or randomness in the replayed stages, explicit transient-state reset) is cheap compared to what it buys. One expectation to set: after a legitimate fusion change, mass checksum divergence against old recordings IS the diff of the fix, not a bug.

## 14. Cross-camera seeding: instrument the funnel before tuning it

Seeding (projecting a hand one camera lost into the other camera's image to skip the palm detector) existed for weeks and appeared to work. Reacquisition latency histograms said otherwise: a cluster of tails sat exactly at the palm-detector period, meaning seeding had silently not fired. Rather than tuning, we added counters at every gate a seed can die at (candidates, offered, redundant, no-free-slot, applied, model-rejected). One live session of counters exposed two independent defects: most hints were overwritten before the pipeline ever evaluated them (a producer/consumer rate mismatch), and the survivors lost the free slot to the palm detector that ran first. Fixing both took reacquisition from 160 ms median to 32 ms (one frame). The counters then caught a third issue for free: seed projections missing the hand, which turned out to be the bone-calibration error, not the seeding code.

Learned: for any multi-stage pipeline with drop points, instrument the funnel first. Each fix here was obvious once the counters localized the loss, and unmeasurable before. Postscript: once seeding was verified, its config toggle and UI were deleted. Settled experiments do not keep their switches, a cleanup rule applied across the project (depth estimators, blend-vs-select, smoothing variants all had their losing arms removed).

## 15. The waving hand: a solver degeneracy hiding behind a plausible theory

A third camera joined the rig and the right hand would not stay tracked while waving (53.7% fused tracking, dropouts up to 1.7s). The replay harness localized the loss precisely: the camera with the BEST right-hand view (presence 0.99, decisive handedness) was contributing nothing to fusion, because its monocular PnP solve collapsed to a 2-4 cm or negative depth on 412 frames and the sanity gate rejected every one. The first theory (a poisoned palmar-side memory mirroring the object model) was plausible, mechanistic, and testable, and a flip-and-retry experiment refuted it: 6 rescues out of 412. The real cause was geometry: a flat waving hand posed by FK is a NEAR-planar PnP object model, and ITERATIVE's cold-start DLT is ill-conditioned near planarity. Exactly planar routes to homography initialization and is fine; near-planar is the worst case. The failure self-sustained because each rejected frame cleared the warm start, so every solve was a cold solve. Switching the cold path to SQPnP removed 406 of 412 collapses. Synthetic flat hands with noise would not reproduce the collapse, so the regression test embeds a real collapsed frame from the recording, which fails under ITERATIVE and solves to a plausible depth under SQPnP.

The second, compounding cause was configuration: fusion.minCameraConfidence had been raised to 0.2 (default 0), and a fast-waving hand's constant-velocity jitter residual crushes confidence exactly when tracking it matters most, demoting high-presence observations to the rescue pool. Replay what-ifs attributed the two cleanly: config alone 54% to 88%, SQPnP on top 95.3%, dropouts 20 to 7. A live session with both fixes confirmed the replay's prediction: 92.8% right-hand tracking, and every remaining gap showed the hand at frame edges in all three views at once.

Learned: when a solve fails persistently, suspect the solver's degenerate configurations before the inputs. And a hard confidence cutoff turns a soft-weighting system into a gate that censors precisely the fast-motion frames the jitter metric mismeasures (lesson 5's censoring bias, in config form).

## 16. The state estimator: replacing selection seams with one continuous fit

The remaining pops all shared one shape: a discrete per-frame selection changing between frames (tri-vs-mono path, stereo pair choice, articulation source, palmar sign), each patched by its own hold or hysteresis. Branch `biomech` replaced the selections with one temporally continuous 26-DoF state per hand (palm transform + 20 raw finger angles) fit every fuse to ALL cameras' 2D landmarks by FK reprojection, damped Gauss-Newton with a constant-position prior, the classic path demoted to seed and fallback. Measured over a 5-recording corpus with a purpose-built pop-metrics replay tool: palmar flips 8 to 0, palm pops over 100 mm 5 to 0, worst single pop 288 to 90 mm, and the FK reprojection residual (the anti-cheat metric, since a frozen hand has zero pops) HALVED at the same time. The estimator both explains the pixels better and pops less, which is what distinguishes a structural fix from a smoothing knob.

Three designs died on measurement inside one session. Fitting only the freshest camera's rows ping-ponged the state between the cameras' systematically disagreeing solutions at fuse cadence (9-10 mm median step on a quiet recording); the fix was fitting every observation inside the staleness window every fuse, age-weighted, which is sound because a per-fuse MAP re-fit of unchanged rows from a converged state stays converged. An instant divergence drop turned every marginal frame into a drop/classic-snap/reseed pop cycle; one bad fit now holds the previous state and only a sustained streak reseeds. And a mono stretch drifted with the single camera's scale-implied depth then snapped 106 mm when a second camera returned; the position prior now goes anisotropic under a single-camera fit, tight along that camera's view ray - the continuous generalization of the old tri position hold, and the anisotropic-covariance step the EKF roadmap note predicted.

The first out-of-sample live recording added a fourth: a solo wrong-side observation (a mislabeled cluster force-assigned by elimination while the real hand was unobserved for one fuse) teleported the palm 0.7 m at a 15 px fit residual - perfectly consistent pixels of the wrong hand, invisible to every quality gate. The fix is physical rather than statistical: a dt-scaled innovation gate (4 m/s ceiling) routes impossible steps into the bad-fit hold path, so a genuine relocation still lands via the reseed within a few fuses and the gate can delay truth but never censor it (the failure mode that killed the hard confidence cutoff in lesson 15).

The deeper issue was upstream: side assignment handing a cluster to a side by ELIMINATION even when its own affinity said no. A survey across three recordings made the rule obvious - healthy assignments never scored below +2.05 total affinity, while all 14 negative-affinity assignments were mislabeled or phantom clusters that stepped the fused hand 256-782 mm, every one a transient in the middle of a well-tracked stretch, none a reacquisition. Fusion now refuses an assignment whose raw affinity is negative WHILE THE SIDE WAS RECENTLY TRACKED (250 ms window): the side goes untracked for a fuse instead of adopting the wrong cluster. The window is the load-bearing half - after a real dropout the temporal prior is stale testimony and the same cluster is a legitimate reacquisition, so refusal expires and a deadlock is structurally impossible. The refusal fixed the CLASSIC path's worst pops too (288 mm max step to 149 on the hardest recording), for a tracked-percentage cost of 0.1-0.7. Learned: when a forced choice has no good option, refusing to choose for a frame is cheaper than either error - but only with an expiry that keeps the refusal from becoming a censor.

Biomechanical priors then landed as residuals on the state, cheapest layer first. Anatomical joint limits (one-sided soft penalties, zero cost in range) collapsed the worst implausible excursions from 34-94 degrees beyond range to 13-27 at zero FK-residual cost. A DIP-PIP coupling fallback holds a garbaged fingertip to anatomical coherence (46 to 20 degrees of deviation in the synthetic test). A per-user angle prior (mean + shrunk precision over the 20 raw angles, fit by `--fit-angle-prior` from the user's own stereo frames) cut single-DoF angle jumps a further 20% on the busiest recording by making coordinated pose changes cheap and uncorrelated snaps expensive; sweeping its weight 0.3 to 1.0 changed little, so it ships weak. A musculoskeletal layer (muscle dynamics) was surveyed and deliberately skipped: it buys torque plausibility, which a kinematic tracking prior at 30 Hz never consumes.

Learned: when every mitigation is a hold or a hysteresis on a discrete switch, the switch itself is the bug - estimate one continuous state and the seams disappear along with their patches. And a prior's test scenario must match its failure mode: the fitted angle prior looked useless against a persistent stereo-consistent snap (correctly - that could be a real unusual pose) and earned its keep exactly on view-inconsistent corruption. A synthetic training set with independent per-finger motion taught the prior that lone-finger curls are normal, and it then correctly refused to fix them; the test had to encode real hands' shared-synergy structure before it could measure anything.

A hard pose - both index fingers pointing and touching - then exposed two things the estimator had inherited wrong. On that rig cam0 and cam1 see the two index tips 1-2 px apart (cam2, the front camera, separates them by 33+ px), so their landmark sets mix the hands; cam0's right-hand palm jitter measured 85-126 mm against cam2's 2.8 mm, and the fusion confidence signal knew it (0.12 vs 0.98). The estimator did not: it weighted measurement rows by the model's reported PRESENCE, which is high on all three. The project's most-repeated lesson, violated in new code. Symptom chain: garbage rows held the fit residual at 25-30 px (healthy is 5-8), which tripped the divergence guard, which after five held frames dropped the state, which fell back to the classic path - and the classic path has no joint limits, so a -148 degree middle-finger angle reached the output. The limits were never violated by the estimator; they were bypassed by its fallback.

Weighting rows by measured confidence fixed the fit, but half the bug survived: the guard judged an UNWEIGHTED mean residual, so a camera the fit had correctly discounted could still trip it. Weighting the residual by the same row weights closed it. Measured on that recording: right-hand angle jumps 125 to 23, bad-fit holds 110 to 36, classic fallbacks to zero, the -148 degree excursion gone (worst violation now 9.9 degrees). One measurement caveat worth keeping: the FK-reprojection anti-cheat metric got WORSE (17 to 21 px) because it averages agreement over all cameras, and agreeing with a camera whose landmarks belong to the other hand is not a virtue - a metric that assumes every camera is valid ground truth stops being one when a camera is provably wrong.

The same pass caught a self-inflicted regression: the estimator had been overwriting the fused `worldPoints` with its own forward kinematics, and bone calibration samples that field - so calibration had quietly become circular, re-measuring the skeleton it was handed instead of the hand. Nothing rendered from the field (the 3D view builds FK from the streamed pose), so the fix was simply to stop overwriting it and let bone calibration ask for the triangulated landmarks explicitly. Learned: when a new estimator starts writing into a field, check who was READING it - a fitted quantity silently replacing a measured one is invisible until a calibration stops converging.

After live verification the estimator became the permanent output path, and the settled-experiment cleanup rule deleted its losing arms wholesale: the experimental toggle, the tri angle and palm-depth holds, the articulation-source hysteresis, and the post-fusion one-euro cascade for hands (dead by construction once estimator poses bypassed it) - along with their config keys, UI, and tests, whose guarantees moved to the estimator's own test suite. The classic extraction survives only as the estimator's seed, its fallback, and the output for uncalibrated cameras.

## 17. A missing joint became a broken joint one bone down

Elbows cut out whenever one hand crosses in front of the other elbow, and in VSeeFace the hand appeared to spin about 180 degrees around the wrist each time - while the app's own 3D view stayed perfectly stable. Two facts explain the whole thing. The solver takes forearm ROLL from the palm and only DIRECTION from the elbow, so a missing elbow cannot produce a twist. And the VMC retarget gated both arm bones on having an elbow while streaming the hand unconditionally: a receiver leaves an unstreamed bone at the avatar's REST pose, which for an arm is the T-pose, so the arm snapped sideways while the hand kept its correct world orientation, and the arm's entire rotation surfaced at the only joint left to absorb it. The app's 3D view was unaffected because it draws the hand from the pose and never walks the arm chain - the same output looking fine in one consumer and broken in another was the clue that the bug was in the retarget, not the tracking.

Two fixes, and the second matters more than the first. The arm chain is now always streamed: with no elbow the upper arm aims straight at the wrist and the forearm takes the hand's own frame, so the wrist reads neutral instead of carrying the arm. And the elbow is no longer dropped when its landmark is occluded - it is re-solved on the same bone circle the measured path uses (shoulder sphere meeting wrist sphere), with the point picked by continuity instead of by the missing landmark ray. Holding the last elbow POSITION was the obvious alternative and is wrong for a measurable reason: the wrist keeps moving, so a held position silently changes the forearm's length (37 mm in the test's own numbers). Re-solving on the circle holds both bones exactly by construction - measured live at 0.8 mm upper-arm error across 395 dead-reckoned frames - because the circle IS the two length constraints. Confidence decays over a one-second budget and deliberately excludes the jitter tracker, since a dead-reckoned elbow is smooth by construction and scoring its smoothness would report high confidence exactly when the value is inferred. Result on the recording: left-arm dropouts 7 to 0, forearm validity 88.6% to 98.2%, with the only remaining gap before the first elbow was ever seen.

Learned: when a consumer composes a hierarchy, a missing estimate is not a local absence - it lands on whichever child is still being streamed. And when the thing you cannot see is constrained by lengths you know, re-solve it under those constraints rather than holding its last value; holding is only safe for quantities nothing else moves.

## 18. A smoothing filter inside a discrete choice

The left elbow spent the last 2.5 seconds of a session parked at shoulder height, 25 cm from where the recorded frames put it, and it stayed there while the 2D landmark returned to exactly the pixel it had started at. Starting a new recording always cured it, which is the signature of state rather than input. The elbow is picked from the circle where the two bone spheres meet, and both points on that circle are anatomically valid arms, so something has to choose: continuity does, by predicting the elbow from the direction held since the last model frame. That held direction was the SMOOTHED one, which is also the output. Through a fast swing the smoothed direction still points at the pose the arm has already left, so the root matching the old pose keeps winning, and once the choice lands on the wrong point the filter converges onto it and endorses it harder. Separating the two - predict from the unsmoothed direction, publish the smoothed one - restored the arm: median elbow height went from 31 mm below the shoulder to 282 mm, against the other arm's 307 mm, and every other recording in the corpus replayed bit-identical.

The second half of the fix was tried, measured, and thrown away. If continuity can take a wrong root, it seems obvious that the image should be able to take it back: let the elbow ray overrule the held choice when the other circle point is decisively nearer it for several consecutive frames. It fires correctly on the recording that motivated it, where the correct point sat 10-19 mm from the ray against the held point's 24-32 mm for 35 straight frames. On an earlier recording, with both elbows plainly down in the frames, the same ray preferred the shoulder-height root for 7 consecutive frames by 15-36 mm - and there is nothing to threshold on, because the two cases' margins and their absolute fits overlap. Shipping it would have cost that arm 32 jumps over 100 mm to buy nothing anywhere else in the corpus.

Learned: never put a smoothing filter in the feedback path of a discrete decision - smoothing is for what you publish, not for what you decide from, and a lagging predictor does not merely delay a decision, it can latch the wrong one permanently. And an arbiter has to be shown to arbitrate: the elbow ray is the only new evidence each frame, which made it look like the natural tiebreaker, but measured against ground truth it is right often enough to convince and wrong often enough to be unusable.

## 19. The single-camera baseline: measure before building

Single-camera support (one front webcam at the desk, avatar output) started as a measurement rather than a feature. The replay engine grew a camera mask, so a tri-camera recording replays as its front camera alone with no IMU overlay, and `--replay-mono-baseline` scores that pass against the stereo-triangulated output of the same frames. Three recordings, about 30 seconds in total, with calibrated hand skeletons:

- Fingers passed their gate (angle error p50 8 degrees). Lateral palm error was under 1 cm everywhere, and shoulders and head matched to zero, since they hang off their own rays and widths, not the wrist.
- The right palm sat 1.6 cm deep of truth at the median. The left palm sat 16 cm deep, consistently, with lateral error of half a centimeter: a pure along-ray scale bias, and only from the front camera. Replayed as either overhead camera alone, the same left hand lands within 4 cm. The front camera's view of the left hand is what the hand model shrinks, not the left skeleton.
- One recording's right hand ran palm-flipped for most of its length (orientation error p50 85 degrees, finger error p90 65 degrees). With one view the palmar sign has no second camera to overrule it.
- Elbow root flips (elbow more than 15 cm from the truth elbow on a model frame) ran 8 percent. On the right arm the median elbow error was 1.7 cm, so the flips follow the wrist's depth excursions rather than the bone-circle choice itself.

Learned: the monocular path's weak point is not resolution or coverage but two signs and a scale, each of which stereo had been quietly fixing. The gates (palm p50 3 cm, p90 8 cm, flips 2 percent, fingers 10 degrees) failed as written, and the number that matters most is the view-dependent depth bias, because it is systematic: a bias can be calibrated out where noise cannot. Coverage in this measurement is optimistic by one mechanism, the masked camera's recorded landmarks include hands it found through cross-camera search hints.

The per-frame view (the tool's CSV) then placed the bias. The front camera labeled the left hand correctly (right-hand probability 0.04), its own PnP lift of the same pixels sat only 4 cm deep, and the estimator's fit reached a 2.6 px residual against those pixels where the stereo truth itself only reached 10.6 px: the camera's landmarks are inconsistent with the true hand, and a fit with 20 free angles can satisfy them over a range of depths, so it settles wherever the valley bottoms out. With no stereo depth to hold, the along-ray prior was leashing the state to its own drift. The fix is a depth anchor: a side that has never been fitted from two views pulls its along-ray coordinate toward the view's own lift, the landmark model's self-consistent depth. Sigma was chosen by header-patching the three recordings:

| Anchor sigma | Palm p50 / p90 cm | Elbow root flips | Left palm along-ray p50 | Right palm rot p90 |
|---|---|---|---|---|
| off | 8.0 / 20.7 | 8.2 % | +15.7 cm | 110 deg |
| 5 mm | 4.0 / 9.8 | 1.1 % | +4.6 cm | 44 deg |
| 10 mm | 3.8 / 8.2 | 1.5 % | +6.7 cm | 21 deg |
| 20 mm | 4.5 / 11.4 | 2.3 % | +10.5 cm | 21 deg |
| 40 mm | 7.6 / 16.2 | 4.4 % | +13.6 cm | 21 deg |

10 mm shipped: the flips clear their gate, the mono palm steps match the truth's, and the palmar flip disappears with the depth freedom that fed it. Tighter follows the lift's frame-to-frame noise (6 cm spread on a still hand), looser lets the pixels drift the palm deep again. What remains is the lift's own bias in that view, about 4 to 7 cm on the left hand, which keeps palm p50 and p90 just over their gates.

Learned: a prior that leashes a state to its own prediction holds nothing when the state was never right, and in a single view the pixels are indifferent to the depth-versus-articulation trade. The absolute reference has to come from somewhere, and the model's self-consistent 2.5D is a better one than the free fit, even though it is a view-dependent opinion.

The lift's own bias then got its calibration: a per-camera, per-side depth scale on the PnP result (`liftDepthScale`), measured offline as the median of stereo distance over lift distance on the same frames. The three recordings share one camera, one user and one calibrated skeleton, and still disagree: left 0.92, 0.92, 0.96 and right 0.96, 0.91, 1.03. Within a recording the one-second rolling median wanders between 0.79 and 1.05 while the hands move, and holds within 0.02 on the one recording where they rest, so the bias is a function of pose rather than of the camera. Cross-validated by giving each recording the median of the other two, all four gates pass (palm p50 2.8 cm, p90 8.0 cm, flips 1.3 percent, fingers 5.9 degrees), the per-recording optimum does no better (2.9 / 8.0), and the sample-weighted pooled value fails p90 because one recording's 400 resting frames outvote the other two. A scale from another session is as good as the session's own, and the p90 that remains is the pose-dependent spread no constant can remove.

Learned: a systematic bias calibrates out, but measure first whether it is a constant of the device or a function of the input. This one is both, a device-sized mean with a pose-sized swing around it, so the wizard's single-pose measurement buys the mean and nothing more, and the swing is the next model's problem, not another calibration's.

## 20. The whole-body model's depth: a sign at best on the desk view

The direct retarget plan for a monitor webcam rests on a monocular 3D body model, RTMW3D-X (133 COCO-WholeBody keypoints, each with a depth relative to the hip midpoint, the same SimCC family as the body model already in use). Before any solver was written it went in as a selectable backend with its depth carried through observations and recordings, and `--gate-direct` measured it against the stereo rig on the three raw-frame tri-camera takes, re-running the model over the front camera's frames and comparing its arm directions with the faithful replay's arms in that camera's frame. The lift from pixels plus relative depth needs a root depth, so each arm was lifted at the root depth that best fit the truth (the model's own error with the scale solved away), at the depth the pixel shoulder width implies (what the live mode would have), and with every relative depth zeroed (a planar model).

- The planar side held up: shoulders within 3 to 11 px of the rig's, wrists within 13 to 15 px, the live root depth within 3 cm of the fitted one at the median, inference 12 to 14 ms per frame on DirectML.
- The depth axis did not. On these takes the forearms point at the camera: the known forearm length over the pixel span requires about 0.22 m of depth from elbow to wrist, and the rig's replay carries 0.22 to 0.26 m. The model reported 0.05 to 0.07 m on the left arm and 0.01 to 0.02 m with the wrong sign on the right. Upper arms read 0.00 to 0.03 m against 0.07 to 0.17 m. Direction error came out at 20 degrees for the upper arm and 60 degrees for the forearm at the median, within a few degrees of the planar lift's.
- The takes are a hard case for the model: the camera looks down steeply from above the desk, the head is cut off at the top of the frame, and the subject fills the frame at 0.8 m, far from the standing, distant, full-body views the depth head was trained on. The monitor-webcam view the mode is for keeps the head in frame and looks down less, so the number is a first measurement, not the gate.

Learned: a model's depth is as out-of-distribution as its view, and on a desk view the one thing it gave reliably was the planar landmark. The webcam vtuber tools never needed the depth: a known segment length against its pixel span fixes the depth magnitude of every segment, and all that is left to decide is the sign, which is where a weak depth estimate can still earn its keep. The direct mode's solver takes that shape.

The take from the webcam position followed (head in frame, a shallow tilt, hands raised toward the camera at about 0.45 m with the shoulders at 0.7 m): shoulders within 9 px, wrists within 58 px at the median with the hands large and the fingers spread, the live root depth within 7 cm, and the depth sign right on 80 percent of frames at 0.6 of the true wrist-to-shoulder difference. That view is what the model was trained on and it shows. The elbows told a different story: they sat below the bottom of the frame for most of the take, the model placed them on the torso, and the rig's own elbow was no better a witness, since the IMUs were not worn and the body solver's elbow comes off the bone circle through that same 2D landmark. The gates were set on what the rig measures on its own, the stereo wrist and the ray-and-width shoulder, and the forearm gates wait for a take with the IMUs on.

Learned: a gate is only as good as the independence of its truth. When the reference and the candidate share an input, agreement measures the sharing, and the honest move is to gate the quantities with an independent witness and name the ones without.

## 21. The A/B against the webcam vtuber pattern: the posed avatar does not yet keep the 2D match

The direct retarget rests on a premise: the mono camera's job is the 2D look, the avatar's joints reprojecting onto the model's landmarks, with the depth along each ray the only thing left to decide. Before changing how that depth is chosen, the gate tool gained what the premise needs to be measured. A Kalidokit-style baseline (`src/Tests/KalidokitStyleRetarget`, the webcam vtuber tools' pattern: the model's own 3D landmarks as directions, the relative depth stretched by the measured 0.6, each bone swung onto its direction, the hand from the hand model's palm plane, every bone slerped 0.3 toward its target per frame, no bone lengths, no holds) runs beside the direct retarget through the same comparison, and two metrics need no stereo truth: the posed avatar's shoulder, elbow and wrist projected into the camera against the model's landmarks (the wrist against the hand model's wrist where the solver admits it), and the rotation of each arm bone between consecutive frames. A direct-mode take has no extrinsics and so no truth, and the tool now runs such a take for the two metrics alone.

Measured on the tri-camera webcam-position take (camera 2, 1532 frames) and the two mono takes of 2026-10-31 and 2026-11-01 (2016 and 1322 frames), both sides pooled:

- Against the stereo reference on the tri-camera take the direct retarget beats the baseline on every bone: upper arm 32 against 38 degrees p50, forearm 33 against 35, wrist 57 against 73, fingers 43 against 49.
- The posed reprojection: shoulders within 5 to 7 px for both, the same placement. Elbows 36 px against 60 on the tri-camera take, 25 against 30 and 27 against 52 on the mono takes, the direct p90 past 300 px on the mono takes where the model invents an elbow on the torso for an arm whose elbow is below the frame, inside the solver's 16 px margin. Wrists 43 px against 123 on the tri-camera take, 31 against 79 and 15 against 112 on the mono takes: a direction transfer without bone lengths lands the hand a hand's width or more from where the camera saw it.
- The step: the direct retarget's upper arm 1.4, forearm 2.9 and hand 3.0 degrees p50 on the tri-camera take (p90 5.6, 32 and 31), 1.0, 2.2 and 2.3 on the first mono take, 0.4, 0.7 and 0.6 on the second. The baseline's hand steps 12, 5 and 7 degrees p50 with p90 30 to 39. Its forearm step p50 reads 0 on two takes because an unseen elbow keeps the last bone, so where the elbows are below the frame the number flatters it.

Learned: the direct retarget wins the A/B against the pattern it was modelled on, on every number, so what remains of that pattern are priors to fold in (the network's depth magnitude, the heavy filter on the quantity that carries depth), not a replacement. And the premise does not hold as built: a wrist the solver places on its own ray reprojects 15 to 43 px off at the median and 44 to 100 px at p90, because the chain is filtered as 3D points after the ray is read and the holds carry stale wrists. Those are the numbers the depth scorer with the filtering split by axis has to beat, with the baseline's as the floor.

The scorer followed: the wrist's distance along its ray as the one scored variable over the reach the arm allows (the elbow's reprojection, the model's decompressed depth as a wide prior, the last frame's chain), the elbow on the bone circle for each candidate, and the filtering split by axis (pixels at 2.5 Hz, depths at 0.6 Hz). Measured on the same three takes, first commit against second:

- The 2D match the mode is for: the wrist's reprojection fell from 43 to 16 px p50 (101 to 35 p90) on the tri-camera take and from 31 to 14 and 15 to 11 on the mono takes. The elbow's fell from 36 to 31 on the tri-camera take and rose from 25 to 29 and 27 to 39 on the mono takes, where the forearm points at the camera and the elbow's miss barely depends on the depth, so the model's prior decides it and leaves the elbow a few px off its ray.
- Against the stereo truth: forearm 33 to 31 degrees p50 (p90 88 to 63), wrist 57 to 52, upper arm 32 to 35. The wrist along its ray lands 11 cm from the truth at p50 and 18 at p90, 7 cm too near at the median.
- The step: on the tri-camera take upper arm 1.4 to 1.2, forearm 2.9 to 2.5 (p90 32 to 12) and hand 3.0 to 2.4 (p90 31 to 13) degrees p50. On the mono takes the hand's fell and the forearm's rose by a quarter degree. The four gates pass.
- The hand lift's depth, the one metric cue for a hand whose elbow is out of frame, was scored at first with an online scale against the geometry. The scale came out 1.1 and 0.6 of the length ratio on the two hands of the calibrated webcam take and at its 2x bound on the uncalibrated mono project, and the term moved the forearm from 34 to 44 degrees against the truth and the mono elbows 10 px further off. The lift is now measured and reported per take, not scored.
- "The circle point nearest the elbow ray" is ill-conditioned exactly in the seated pose: with the forearm at the camera the ray pierces the circle's plane near its center, every point is about as near, and the elbow slid around the circle on every small change of depth. On the truth frames of the tri-camera take the upper arm's step rose from 1.5 to 2.3 degrees p50 on frames that had been still, and the lighter pixel band was not the cause (1.2 Hz changed nothing). Regularizing the elbow's place on the circle by its distance from the previous elbow, at the same joint-speed sigma the wrist uses, brought it to 1.2.

Learned: the wrist's 2D match comes from the filter split, not the scorer: a wrist on a filtered ray at a filtered depth reprojects where the camera saw it, and the old 3D filters were what moved it off. The scorer's value is in the 3D (the forearm against the truth, the step), and each of its terms has a regime where it is blind, so a term is kept only where the truth shows it earns its weight: the lift did not. And a parameterization is judged by its conditioning in the pose the mode is for, not by its elegance: wrist-depth-first put the elbow where the seated pose cannot see it, and the fix was continuity, not a sharper term.

The fingers followed, from the first live test: hands held up open, palms to the camera, and the avatar's fingers folded back toward the wrist on one hand and clawed on the other, with the landmarks clean (fingers straight in 2D, the FK rebuild 5 to 7 px off). The direct mode had inherited the stereo path's finger zero, the fused rest angles subtracted from every frame, and the neutral capture had taken that zero from one hand-model frame with the hands resting on the desk pointing at the camera. In that pose the palm's finger axis runs along the view ray, so every angle measured against it is the model's depth noise: the stored zero held laterals of 55 to 154 degrees, and the subtraction swung every finger across or behind the palm. Kalidokit, the pattern this A/B is against, has no such capture: each bend is the angle between adjacent bones in the model's own 3D, with a fixed per-joint offset and clamp tuned once for the hand model, and it does not read splay. The direct mode now does the same with the splay kept: each finger is measured against its own metacarpal (the hand's fan is the zero, so a relaxed open hand reads zero and the avatar's rest fan is what zero reproduces), the three bends lose the bend the model reads on a straight finger facing the camera (10, 25 and 16 degrees at the three joints of the four fingers, 21, 10 and 0.5 for the thumb, the pooled medians over 390 held frames of both hands), and are floored at zero. The desk setup keeps its rest capture: hands flat on the table beside the marker are a pose the cameras measure well.

Learned: a per-user zero is only as good as the pose it is captured in and the view that measures it, and a single webcam at desk height cannot measure a resting hand. Where the model's bias is a property of the model, not the user, it is a constant measured once and re-measured per model, which is what the fixed offsets in the webcam tools are. The parameterization decided it: the stereo path's angles are frame-relative so the fan and the knuckle offsets need a zero, while an adjacent-bone bend is zero on a straight finger whatever the frame, and the one frame-relative angle worth keeping, the splay, has a zero the hand carries with it in its metacarpals.

The arm followed, from the first held pose: hands up open, palms to the camera, elbows at the frame's bottom, and the avatar's hands rendered giant with the wrists bent 50 degrees, the wrists 0.2 m nearer than the hand lift's 0.67 and 0.70 m. The static-pose loop (the delayed dump with the avatar rendered from the camera, `--pose-report` on it) and a one-off print of the scorer's whole cost landscape over the wrist's distance showed the chain was solved with the avatar's bone lengths: a forearm of 0.34 m in the user's units against the body config's landmark-space 0.25 m, and an upper arm of 0.30 against 0.42. The surplus and deficit were consumed by depth, so the elbow reprojected within 2 px only at the near crossing of its bone circle (0.51 m) and the far one (1.1 m), and scoring the lift's distance against that moved the choice 0.02 m, the elbow term holding the wrist at the near crossing against the lift's 35 px miss between the two. With the chain in the user's landmark-space lengths through the length ratio the same frame solved to the pose (elbows bent to 36 and 47 degrees, wrists at 0.81 m, the elbow within 5 px), but only with the lift scored: without it the model's decompressed prior (0.41 avatar units of shoulder-to-wrist depth where the pose has about 0.2) and the elbow took the near end of the reach, and on the seated webcam take the left forearm flipped 124 degrees; a 0.25 sigma on the lift did the same on both. At the 0.15 sigma the seated take's forearm went from 30.6 to 27.9 degrees against the stereo truth and the upper arm from 34.7 to 42.2, the wrist along its ray from 10.9 to 13.7 cm p50, with the lift's measured scale 1.4 and 0.7 of the ratio between the two hands of that take.

Learned: the lengths that make a landmark chain consistent are the landmark-space ones the body solver already carries, and an avatar's lengths in their place are not a harmless scale but a depth error the elbow's reprojection cannot see. The price is the premise: with the chain in the user's proportions the avatar's elbow and wrist leave the landmarks by 60 to 170 px, so the 2D match is now the shoulders and the directions, and a receiver that overlays the avatar on the video will need the wrist put back on its ray by an IK over the avatar's lengths (plan.md).

## 22. The palm mirror: a tiebreak decided with no evidence

A webcam take of palms held to the camera and turned 180 degrees about the wrist and back, three times, showed the avatar's palms popping as each hand passed edge-on. The hand model's own palm was smooth there. Re-solving single frames with `--pose-report` put the pops in the direct solver: the forearm twist stepped about 170 degrees between adjacent frames while the lifted palm moved under 15.

The solver chose between the lifted palm and its mirror along the view ray by which finger axis better continued the forearm, with zero as the bar. The difference between the two is twice the finger axis's lean along the view ray times the forearm's. With the fingers upright that lean is near zero, so the choice followed its sign: it changed 22 and 21 times on the two hands, on exactly the frames where the twist jumped. The mirror leaves a palm that faces the camera as it is and turns an edge-on palm's normal by up to 176 degrees, so the choice was least informed where it cost most. The 60 degree hold adopted any flip that lasted two frames.

`--gate-direct` gained a count of twist steps over 90 degrees between frames that read the hand, and the per-frame gain in its CSV. On the take the count was 8 of 406 frame pairs on the left hand and 6 of 407 on the right. With the mirror taken above a gain of 0.25 and kept above 0.15 the count went to 0, but the right hand still took the mirror for six frames at a gain of 0.29, a 68 degree step under the count's bar. The gain over the whole take, which needs no mirror anywhere, has a p99 of 0.21 and a maximum of 0.29. At 0.5 and 0.3 no frame takes the mirror and no twist step exceeds 45 degrees, and the twist runs from +22 to -114 degrees on the left hand and -35 to +139 on the right through each turn. Under the old rule the first part of each turn was solved on the mirror, at +110 to +137 degrees on the left.

Learned: a rule that picks between two hypotheses needs a bar set by what the evidence can tell apart, not zero, and the place to look for the failure is where the rule's evidence is weakest and its effect largest. What remains on the take is the hand model's own palm jumping 40 to 63 degrees a frame on the left hand's second pass through edge-on (plan.md).

## 23. The hand crop's cost followed the hand's size

A single-webcam take showed hand inference drifting between 6 and 28 ms with both hands tracked, smooth over seconds rather than spiking. It did not follow the body-pose cadence or the palm detector's interval. It followed hand size: against the pre-crop's pixel count it fit 5 ms plus 8.4 ms per megapixel at a correlation of 0.73. The `mp_handpose.py` port built its crop in stages at source resolution. It took the palm box enlarged 4x and padded to its diagonal, converted it to RGB, rotated it with a full-size `warpAffine`, cropped again and shrank the result to 224. A hand near the camera made that pre-crop most of the frame's height, up to a megapixel per hand.

The crop is now one warp straight from the frame into the model input, at twice the model resolution and box filtered down for a large hand. On the take's raw frames through `--replay-hands`, the estimate went from 3.0 ms per hand at the median (1.9 to 4.1 ms by size) to 1.3 to 1.6 ms flat. Against the recorded landmarks the median difference rose from 0.5 px (JPEG alone) to 1.4 px, but the 90th percentile rose to 13 px. Those frames were hands low in the frame. There the old crop blacked out image content outside its axis-aligned pre-crop and re-centered a final box it had clipped. Overlays of the worst of them put the new landmarks on the hand and the old ones off it, and in one the old crop scored a plainly visible hand at 0.00 presence where the new one scored 0.97.

Learned: a staged image pipeline ported from a demo carries the demo's intermediate buffers, and their cost and their clipping are both invisible until a hand fills the frame. Compose the geometry and resample once.

## 24. A head from the face landmarks: three estimators, none closer to the phone

The whole-body model emits 68 face landmarks, and the direct mode's head comes from three body keypoints, the ears and the nose. The proposal (`docs/proposals/wholebody-landmarks.md`, piece 6) was to fit the head from the face instead, on the expectation that headphones and hair hide the ears. The measurement take was made on 2026-11-12: the webcam-position camera, the phone's head rotation in the face sidecar as truth, a head sweep with the ears visible and a second one under over-ear headphones. `--gate-wholebody` compares each head as a turn from the rest pose, after the one rotation that best carries the phone's turns onto the method's.

Over the two sweeps, p50 error against the phone:

- the six-point face fit (a PnP of the nose tip, chin, eye corners and mouth corners against a generic face): 15.8 and 15.3 degrees
- the ear and nose head: 18.7 and 15.8 degrees
- a rigid fit of the brows, nose and eyes lifted by the model's own depth: 43.9 and 41.0 degrees, its turn 2.3 times the phone's. At a quarter of the depth it reads 15.5 and 15.4, where the depth axis adds almost nothing.

Every method read on every frame of both sweeps. The model placed the ears through the headphones, so the failure the piece was built for did not occur. The six-point fit and the ear and nose head sat 14 degrees from each other with no phone in it. A mirrored alignment fitted neither better, which rules out a phone axis read with the wrong sign, and shifting the phone by 200 ms either way moved the numbers by under 2 degrees.

Two earlier readings of the same take were wrong and are worth keeping as the lesson. Over the whole take the ear and nose head read 8.4 degrees against the fit's 12.4, because the take is mostly a still head, where a filtered estimate sits quiet and a raw one jitters. And the ear head appeared to read on 54 percent of frames, because the tool took a head frame that repeated bit for bit as a held one, and a still head through a filter repeats.

Dropped: no face head is closer to the phone than the ear and nose head by more than the comparison resolves. Not settled: whether 15 degrees is the true error of all three during a turn or a floor in the comparison, which has not been run against a known rotation.

Learned: judge an estimator on the segment that exercises it. A whole-take median over a mostly still take measures the filter, and a freshness test built on "the value changed" fails on exactly the frames where nothing moves.

## 25. Blinks from the webcam: a face model beat the whole-body points and was not worth its cost

The webcam face source needs blinks, and an open eye is a few pixels tall in the whole-body model's input. The proposal (`docs/proposals/wholebody-landmarks.md`, piece 4b) was to measure the whole-body eye points against a face-only model on a crop the face block places. Three candidates ran on the 2026-11-12 take against the phone's blink columns: RTMPose face6-m and face6-t, and MediaPipe Face Landmarker's mesh with its blendshape model. The take's blink window is 22 s with 10 phone blinks and one wink, the user in thick-rimmed glasses at a 98 px face width.

The open and closed levels' separation in pooled deviations, and the crop and inference time:

- the whole-body points: 2.2 to 2.9, at no added cost
- RTMPose face6-m: 3.8, its closed level a third of its open level where the points' is two thirds, at 6.6 ms a frame
- RTMPose face6-t: 2.5 to 3.0, at 4.2 ms
- the MediaPipe mesh: 2.3 to 2.5, at 5.2 ms with the blendshape model
- the MediaPipe blendshape model: 1.9 to 2.5, its blink output reading 0.25 to 0.37 where the phone reads over 0.6

At the live divider the points find 8 of 8 and 9 of 10 of the phone's blinks with 3 and 0 false events. face6-m finds all of them with 3 false events per eye, the same at every cadence.

Two earlier readings were wrong. The points first showed 8 false blinks in the 22 s, which read as a failure. Most were the truth: the phone's blink column peaks at 0.45 to 0.59 on five closures that every camera signal, and the frames, show as full. And the points appeared unable to tell a wink's eye, because each eye's openness correlated equally with both phone columns. Nearly every blink in the take closes both eyes, so that correlation says nothing about sides. On the 12 frames of the one wink every signal tells the eye, with the camera's sides crossed against the phone's columns.

Decided: the whole-body points carry the blinks and no face model is added. face6-m is better, and 6.6 ms a frame for a third model is not worth the margin while the points find the blinks at the cadence they run at. The other three candidates do not beat the points. Not settled: which of the phone's stream and the camera's sides is mirrored, and how the points hold on a second person or without glasses.

A third reading fell when the source was built on the points. The evaluation scored blink events at each pass's best threshold, against an open level taken from the frames the phone called open. With a neutral fixed at the capture, as a live source has, the points read the eyes over half shut on 26 to 81 percent of the take's frames outside its rest hold. The openness is the same with the lids lowered as with the eyes shut (0.201 both, against 0.306 open), and it sags as the head turns and the gaze drops. The decision above stands as the cost call it was, and what it buys is narrower than this section first said: the points give the mouth and the brows, and the built-in profile does not take its blinks from them. The blinks stay the phone's. face6-m tells a lowered lid from a shut eye by 1.5 deviations, which is better and still thin.

Learned: when unrelated estimators agree on the same false events, suspect the truth before the estimators, and look at the frames. An event score at a tuned threshold says a signal can be thresholded on that window, not that a fixed threshold works across a session: test the rule the consumer will run, over the whole take. A correlation cannot separate two columns that move together in the data.

## 26. Hand label correction: the dropouts were not where the design expected, and the rule traded entries with itself

The direct solver takes a hand slot for an arm only when the slot's wrist is beside that arm's body wrist, so a slot under the wrong side is turned away and the hand freezes. The first design expected the loss while the hands clasp or cross. The label report in `--gate-wholebody` on four direct takes put none of the 207 turned-away slot frames there. Clasped wrists are beside both slots, so both pass, and 34 slot frames were accepted on the wrong arm instead. 135 of the 207 were a real hand under the wrong side after the hands parted, in runs of at most 15 frames: the slot's own `rightProb` named the true side on 128 of them, and the published label was held by the pipeline's 15-frame stickiness. The other 72 were a slot on the face or a picture frame beside a raised wrist, turned away only because its label happened to be the other side.

The whole-body model's hands name a slot's side by shape. The margin between the two distances alone gave a side to slots more than two palm lengths from both hands, so the match took a bar of 0.5 palm lengths, set from the gap in the data: turned-away real hands sat within 0.5, non-hands beyond 1, one frame between. With the rule ahead of the lift the four takes went from 207 turned away to 82 and from 135 on the other arm's hand to 10, with no slot on no hand taken.

The first live take found what the recorded ones had not. With one hand out of view its whole-body hand does not score, the match decides nothing, and the one-hand rule decides for a slot under the unscored hand's label. Two slots on the visible hand then read as one decided challenger and one undecided holder, and the challenger took the entry on 42 frames. Applying the rule to the take's recorded slots moved 68 of them again, which is how it showed: a correct rule moves nothing on its own outcome. Letting the nearer of the two take the entry stopped that and raised the side changes of a tracked slot from 38 to 85, because two slots on one hand differ in distance by jitter. The slot already under the side now keeps it while it is within the bar of that side's hand, and the side changes stay at 38 on that take and fall from 61 to 27 on the four.

Learned: measure where the loss is before designing for where it is expected, since the fix for a sticky label is not the fix for a crossing. A stateless rule that reorders its inputs should be checked by applying it to its own output, and that check belongs in the self-tests. A contest between two near-equal candidates needs a holder, or the measurement noise decides it every frame.

## 27. The palm turned right round: two fixes from inference, then a count

With the hand labels corrected, direct mode still showed the palm jumping half a turn, 44 twist steps over 90 degrees on seven takes. Eleven of the jumps on two takes fell 5 to 8 frames after an entry took a hand, which matches the lift's palmar memory overturning a wrong seed after five contradicting frames plus the solver's two-frame hold. Two fixes were built on that reading. A solver wait of eight frames on a new hand's palm removed the jumps it covered and froze the palm on a quarter of the read frames, half on two takes, because a hand entry drops for a single frame constantly: 169 of 253 tracked runs on one take are one frame long. Keeping the lift's palmar memory across a short loss of the same slot changed nothing, 44 jumps to 46, and broke replay on every take.

`--gate-direct` then gained the count that should have come first: each jump put down to what changed in the camera's hand over the three frames before it. 24 were the lift's palmar side flipping, 15 the solver's mirror toggling, 5 neither, and none a slot change. The knuckles' winding in the image changed on 3, so the hand was not turning over. The palmar side disagreed with the one the hand's side names on 1397 of 12694 lifted frames, and 21 of the 24 flips were into agreement: a jump was the lift correcting itself, and the stretch before it was the palm shown the wrong way round. The lift had kept clear of the side because the hand model's label is a view-dependent guess. In direct mode the side is now the body model's, so the lift takes the palmar side of a hand whose side the whole-body hands decided from that side. Jumps went from 44 to 26 and the jumps on a palmar flip from 24 to 1.

Learned: a timing coincidence is a hypothesis, and a fix built on it tests the hypothesis at the price of the fix. Count the failures by cause in the tool before changing the pipeline. And a rule written to distrust an input should be revisited when the input's source changes.

## 28. A hand half out of the frame is laid out as the other hand

The first take recorded with `sideDecided` verified in replay and its palmar side matched the hand's side on 98% of lifted frames, against 89% before. It had three twist jumps in 1782 frame pairs, none on a palmar flip or a mirror toggle, all on the right hand as it left the frame's left edge or came back in. With the wrist 40 pixels from the edge and 12 to 17 of the 21 landmarks within 16 pixels of it, the hand model put the thumb off the edge and ran the pinky up the visible thumb. The whole-body hand sat on the same spot, so the side stayed decided and the lift's sign stayed right over landmarks that had turned. The palm turned about 130 degrees and stayed there until the hand was lost.

The direct solver now holds the palm and fingers of a hand with three or more landmarks at the edge, as through a dropout. On that take the jumps went from 3 to 0 with 79 hand frames held. On the seven takes of 2026-11-12 the 26 jumps are unchanged, none of them being on a cut hand, with 99 frames held. `--gate-direct` counted the step off the held palm when a cut hand cleared the edge as a jump until a held frame stopped counting as read.

Learned: the first reading of the three jumps took the tool's CSV, which interleaves the two sides' rows, as one series, and called each jump a single frame on a hand read on alternate frames. Per side each was a turn that persisted. Read a per-side series per side.

## 29. The palm mirror's gain does not say whether the mirror is right

17 of the 26 twist jumps left on the takes of 2026-11-12 follow the direct solver's palm mirror toggling. On those hands the mirror and the lifted palm are about 110 degrees of twist apart, the gain sits between 0.3 and 0.7 for long stretches, and it crosses the bars (0.5 to enter, 0.3 to leave) on small changes. Seven of the jumps are excursions of one or two frames into the mirror and back, which the palm hold's two frames let through.

A sweep of the bars through `--gate-direct ... mirror=<enter>,<leave>` on the eight takes:

| Enter, leave | Twist jumps | Mirrored frames |
|---|---|---|
| 0.5, 0.3 | 26 | 265 |
| 0.7, 0.5 | 18 | 93 |
| 0.7, 0.3 | 17 | 114 |
| 0.9, 0.6 | 13 | 29 |
| 0.9, 0.3 | 11 | 45 |
| never | 10 | 0 |
| 0, 0 | 94 | 2277 |

Higher bars cut jumps by mirroring less. Against the other hand's twist as a reference, which holds only while the hands are posed alike, one mirrored run of 113 frames at a gain of 0.43 to 0.70 was nearer the other hand on every frame, and a run of 27 frames at 0.35 to 0.73 on one. The same gain gets opposite verdicts, so no pair of bars keeps the first and drops the second.

Learned: a jump count falls when a correction is switched off, and that says nothing about whether the correction was right. Check a threshold sweep against a reference before reading it.

## 30. Blinks from the webcam over time: the timing finds the blinks and does not clear the false ones

Section 25 left the webcam's blinks out of Auto: the eye openness reads as low with the lids lowered or the gaze dropped as with the eyes shut, so a fixed neutral holds the blink over a half on 26 to 81 percent of the frames outside the rest hold. XRAnimator's blink path, read for reference, gets its clean blinks from post-processing over MediaPipe's blendshape: an adaptive baseline per eye, a running floor, an S-curve, and both eyes pulled toward the less closed one unless they differ by a wink. It has no rate test. The Blink stage (face.md) takes those parts and adds one: a closure starts only on a fast drop under the baseline, and a slow sag moves the baseline instead.

`--gate-wholebody` ran the stage over the whole-body points under each take's neutral, with the default settings. The bar for Auto's fallback was at least 9 of 10 phone blinks found at the live divider, over a half on no more than the phone's share of frames plus 5 points, and no held look down reading shut.

- On the 92 s rig take (`2026-11-12_00-18-34`, the webcam position camera, head sweeps and a hands segment): at the divider the stage finds 14 of 15 and 18 of 19 of the phone's blinks, where the raw points find 14 of 15 and 19 of 19. It reads over a half on 24 percent of the frames outside the rest window for each eye, where the phone's columns are on 5 percent. It counts 69 and 61 false closures in the 92 s against the raw points' 80 and 85 at their best threshold, 49 and 31 of them on one model frame.

- On the 33 s wink take (`2026-11-12_13-39-53`): 4 of 4 and 4 of 5 found at the divider, over a half on 5 and 4 percent against the phone's 14, and the stage tells the winking eye on 96 percent of the one-eye frames where the raw points do not tell the eyes apart (36 percent). It counts 4 and 5 false closures in the 33 s against the raw points' 2 and 0.

- On a 56 s take with no phone blinks (`2026-11-12_14-45-00`): over a half on 1 to 2 percent against the phone's 0.

The long closure rule was the expected risk: a closure held in the closed band does not end, and lowered lids read at the closed level. A run on the rig take with every closure released after 600 ms moved the share over a half from 24 to 21 percent, so the held closures are not the cause. The false closures are short: point jitter and head motion make drops as fast as a blink.

Decided: the stage lands for custom profiles as `blink.*`, and Auto stays on the phone's blinks. The new take with deliberate looks down and a long eyes-closed hold was not recorded, so the held look down part of the bar is unmeasured. Not tried: confirming a closure over two model frames, which the one-frame share suggests would cut half the false ones, and holding off closure starts while the face block moves.

Learned: a timing rule finds blinks on these points as well as the best threshold does, and the false events it does not remove are the points' own noise, which no rule on one ratio over time tells from a blink at the same speed.

## 31. Intrinsics without a board for the front camera: what a wrong focal length costs and what estimates one

The single-front chain's only print is the charuco board, and the plan was to let the user skip it. Two things were unmeasured: how far the direct retarget's output moves under a wrong focal length, and whether anything short of a board estimates one. The scripts are in `tools/intrinsics_spike`. `--gate-direct` gained the posed upper arm, forearm, hand and head rotations in its CSV, so two replays of one take under different headers compare frame by frame.

What a wrong calibration costs, on three direct takes of the C920 (9056 hand frames), as the rotation of each posed bone against the unperturbed replay in degrees. Each row is the p50 averaged over the two directions of the change, with the larger p90 in brackets:

| Change | Upper arm | Forearm | Hand | Head |
|---|---|---|---|---|
| fx 2 percent off | 1.9 (2.7) | 0.7 (2.1) | 0.3 (2.0) | 0.1 |
| fx 5 percent off | 4.7 (6.8) | 1.6 (5.2) | 0.8 (5.1) | 0.2 |
| fx 10 percent off | 9.3 (13.6) | 3.1 (10.7) | 1.7 (10.4) | 0.4 |
| fx 15 percent off | 13.8 (20.3) | 5.0 (16.3) | 2.6 (15.9) | 0.6 |
| tilt 5 degrees off | 5.8 (12.1) | 5.0 (5.5) | 5.0 (5.1) | 1.0 |
| tilt 10 degrees off | 11.6 (24.5) | 10.0 (11.3) | 10.0 (10.3) | 2.1 |

- The upper arm turns about one degree per percent of focal length error, the same in both directions, and the forearm a third of that. The head does not move.

- A tilt error passes straight through to the forearm and the hand, degree for degree.

- The cost of no undistortion could not be measured this way. Replaying the takes on raw-equivalent frames with a plain pinhole moved the arm bones 3.3 degrees p50 and 61 to 74 at p90. The control, one take's frames re-encoded as JPEG with the calibration untouched, moved them 8 degrees p50 and over 100 at p90. The solver's discrete choices (the wrist's near or far basin, the palm mirror) flip on pixel-level changes to its input, and that floor is above what the C920's distortion (k1 0.12, k2 -0.24) does.

For scale, on the tri-camera webcam-position take the direct retarget's upper arm sits 37 to 48 degrees p50 from the position retarget of the stereo output.

What estimates a focal length, as the error against the charuco value on raw-equivalent frames of seven camera views (the C920 from two positions, the PW315 and the C960s over the desk and facing the user), 24 frames each:

| Source | fx error |
|---|---|
| A nominal 70 degree horizontal field of view | -1 percent on the C920, +23 to +44 percent on the wider cameras |
| A nominal 78 degrees | -14 percent on the C920, +6 to +25 percent on the wider cameras |
| MoGe-2 (`moge-2-vitl-normal`, its own focal recovery) | -12 to +12 percent |
| GeoCalib (pinhole and simple radial weights) | -24 to +21 percent |
| A generic six-point face over a head turn | not observable |

- The C920's two charuco calibrations, made in different projects, differ by 0.5 percent, which is the reference's own noise.

- MoGe-2 is the steadier of the two models. It reads the same C920 at -12 and -11 percent from two positions, and the widest camera at +12 from both of its positions.

- GeoCalib reads the same C920 at -23 percent in one room view and +13 in another, both full of picture frames and door edges. Its radial term does not follow the charuco k1 on any camera (-0.03 where the board gives 0.12). Solving the 24 frames with shared intrinsics narrows the spread and leaves the median where it was, since a fixed camera's frames are one scene.

- GeoCalib's pitch against the camera's tilt from the rig's extrinsics, the desk taken as level: 10 degrees short on one camera facing the user and 3 on the other, and the wrong sign on the cameras over the desk. On the one-camera project it reads 1 degree where the tilt slider is set to 21. That project has no extrinsics to say which is right.

- The face solve (`cv2.calibrateCamera` over 40 views spread across the head's yaw, the principal point fixed, no distortion) returns the focal length it was started from, -31 to +48 percent over five starts on one take, or diverges, at 15 to 30 px rms. The whole-body face points against a generic face do not carry the perspective a focal length is read from.

Decided: the webcam table stays the first source, since a listed camera is exact and a nominal field of view is only right for the camera it was chosen for. GeoCalib is not taken up for the focal length, the distortion or the tilt. Open, for the unlisted camera: MoGe-2's 12 percent is about 11 degrees of upper arm where a nominal 78 degrees is 6 to 25 percent off on the cameras here, and it costs a 1.3 GB model the app does not otherwise load.

Learned: a single-image camera model was 10 to 24 percent off on webcam frames of a seated person, and the replay instrument cannot see an effect smaller than the solver's own response to a re-encoded frame.

The board on a phone screen instead of on paper, measured afterwards on the C920 at 20 to 30 cm, as the share of recorded frames each pattern was found whole in:

| Pattern on the phone | Found whole |
|---|---|
| Charuco, the detector as shipped | 0 to 7 percent |
| Charuco, the detector tuned for bloom | 48 to 49 percent |
| Circle grid | 62 percent |
| Chessboard, sector based detector | 90 percent |
| Chessboard, classic detector | 97 percent |

- The charuco board fails on the marker bits, not the corners: the screen's white cells bloom into the black ones. Ignoring a wider margin of each cell or allowing more bit errors recovers single frames, and dimming the screen does little, since the camera's exposure follows it.

- A hand-held phone moves a median of 5.6 px between frames, and motion blur biases the corners. The chessboard solved over every frame was 2.1 percent off the printed calibration, with sets of 12 views from -1.8 to +8.5. Keeping only frames within 3 px of the one before brought it to +0.6 (-1.0 to +2.0), and the sector based detector on those frames to a 0.3 px reprojection error.

- A hold of 0.3 seconds under 1.5 px captured nothing on the same recording. A hand holds still for a frame or two, so the gate is one comparison against the detection 0.09 seconds earlier.

- Live, one run in five landed 2.7 percent low. Its saved views showed why: two of the twelve had corners in the wrong places (10.9 and 2.5 px of reprojection error where the rest sat at 0.25), the worst a full grid returned by the sector based detector on a 53 degree tilt. Dropping the worst view while it is over 1 px, one at a time, brought the run to -0.1 percent and left the other four untouched, all five between -0.1 and +0.8. Dropping only the single worst view left it at -3.3, since the second then dominates.

- Nothing about the set as a whole gave the bad run away beforehand on recorded footage: the spread with one view left out, a half split and the tilt counts all had no correlation with a 12-view set's error. The per-view error is what separates a bad view, by a factor of ten or more.

Decided: the one-camera lens calibration shows a plain chessboard on a phone, found per frame with the classic detector and re-detected with the sector based one on the frames it keeps. The solve drops outlier views and the wizard asks for their replacements. The printed charuco board stays for a rig.

Learned: a pattern that needs no decoding survives an emissive screen, and which frames are kept moved the focal length more than which detector found them.

## 32. An arm held up by a wrist the body model could not see

With both hands below the frame the direct avatar kept its right arm raised. The hand slots were not the cause. The palm detector did fire on the chair every other frame, but the solver's wrist rule turned every one of those slots away. The arm was placed from the body model's wrist alone: with the hand out of view the model parked the right wrist 29 to 51 px above the bottom edge and scored it 0.52 to 0.69, over the 0.5 bar for runs of up to 38 frames, where a wrist in view scored 0.80 to 0.85. The hand model's presence at that wrist was 0.00 throughout. The three-frame entry gate was built for a one-frame flicker and did nothing for a steady score. Five of twelve direct takes had it, one for over 700 frames.

The first rule tried: no arm without a hand slot on the wrist, at entry or after, with the wrist alone running out the hold. It removed every phantom run. It also rested a real arm in nine runs, about 110 frames, where the wrist was in view and no slot was on it for longer than the hold: clasped hands, crossed hands, palms held out at the camera. Six checks and two golden cases that assert the carry through a hand dropout failed. The rule kept: an arm at rest needs a slot on the wrist to enter, and a placed arm follows a wrist with no slot only while the wrist is farther than a tenth of the frame's height from every edge. The clasp and crossed-hand runs came back, the phantom runs stayed gone, and the golden compare is bit-identical to the frozen solver again. A hand pointing forward at the bottom edge is lost under either rule.

The slots that flickered were detector-spawned slots kept through a failed first landmark pass: published once at presence 0.00 to 0.20, dropped on the next frame, detected again, each drop re-arming the relaxed detector window. Dropping any slot on a failed first pass, as a seeded slot already was, took the slot frames on neither whole-body hand from 2350 to 999 over the twelve takes through `--gate-wholebody`, with the hands' coverage down 54 frames of about 17,300, 37 of them one hand on the crossed-hands take. The slots left are ones that pass their first landmark pass.

One take's right arm entered five frames later under the new entry rule and settled on a different wrist distance for the rest of the take, 93 px against 72 px in wrist reprojection. The wrist depth depends on the frame the arm enters on, which is open in plan.md.

Learned: the first count of frames with the arm up took the most common upper arm rotation as the rest pose and missed the take where the phantom pose was the most common one. The posed wrist's reprojection against the landmark, which is several hundred pixels for an arm at rest, showed it.

## 33. A wrong lens against the stereo truth: the calibration does not bring the direct arm nearer the rig's

Section 31 measured how far a wrong focal length moves the direct retarget's output, against its own unperturbed replay. Two things were left unmeasured: whether that movement is toward the truth, and whether placing the joints on calibrated rays beats the learned depth the webcam tools use. `--gate-direct` gained `candidates=`, which runs the candidates under a copy of the take with another lens while the truth keeps the take's own calibration. Patching the take itself moves the truth too, since the replay triangulates through the header. The CSV gained each candidate's shoulder-to-wrist reach in the camera's frame, compared here with the truth's stereo wrist and ray-and-width shoulder, which do not depend on the elbow. `tools/intrinsics_spike/sweep_truth.py` runs the sweep.

Two takes, the tilt slider set to each camera's pitch from its extrinsics: the tri-camera webcam-position take (the C920, 18.7 degrees down, no IMUs, 1042 arm frames) and the rig take of 2026-11-12 (a C960, 39.4 degrees down, the IMUs on, mostly head sweeps, 4879 arm frames). Each count is the arm frames with a truth wrist that every lens posed. The table is the angle between the reach and the truth's at p50 in degrees, for the direct solver's chain and the Kalidokit-style baseline's posed avatar, with the chain's depth error on the first take in cm:

| Focal length scale | Direct, first take | Depth error, first take | Baseline, first take | Direct, second take | Baseline, second take |
|---|---|---|---|---|---|
| 0.85 | 30.4 | 13.5 | 26.6 | 14.3 | 14.6 |
| 0.90 | 30.5 | 12.4 | 26.9 | 14.9 | 14.6 |
| 0.95 | 26.5 | 13.8 | 27.3 | 15.0 | 14.6 |
| 1.00 | 26.4 | 15.9 | 27.3 | 15.8 | 14.7 |
| 1.05 | 21.2 | 14.9 | 27.5 | 15.6 | 14.8 |
| 1.10 | 20.6 | 16.5 | 27.8 | 15.4 | 14.8 |
| 1.15 | 17.2 | 19.4 | 28.0 | 15.3 | 14.8 |
| 1.25 | 15.8 | 19.3 | 28.4 | 15.3 | 14.8 |
| 1.50 | 13.8 | 20.2 | 28.9 | 15.5 | 14.9 |

- The calibration is not where the error is least. On the first take the direction error falls as the assumed lens gets longer, from 30 degrees at 0.85 to 14 at 1.50, and the depth error rises with it, from 13 to 20 cm. On the second take neither moves: 14.3 to 15.8 degrees, the wrist 10 cm from the truth's at every lens.

- The baseline matches the direct chain at the calibrated lens on both takes, 27.3 against 26.4 and 14.7 against 15.8.

- The error is the depth. On the first take the truth's wrist is 24 cm nearer the camera than the shoulder (15 to 29 from p10 to p90). The chain's is 15 cm nearer at the median and 7 to 46 from p10 to p90.

- The longer lens is not a gain. From 1.15 up the first take's chain sits at full reach toward the camera, 0.44 m against the truth's 0.26. On a take of hands raised at the camera that points nearer the truth while the wrist lands farther from it.

- Against the position retarget the first take's direct upper arm is 39 to 42 degrees and its forearm 30 to 33 over these lenses, where the baseline's are 38 and 34 to 38.

- On the second take the direct retarget's right upper arm is 136 degrees p50 from the position retarget where the baseline's is 16, and its posed reach is 30 degrees from the truth where its own chain's is 16. The take's recorded tilt gives the same. Not explained (plan.md).

- A lens far from the calibration changes which frames are posed. At 3.00 the first take posed 527 of the 1042 arm frames, and a table over the frames every lens shares then read 23.0 degrees at the calibration and no order across the lenses. The sweep prints each lens's own frame count beside the shared one for this reason.

Both of those cameras look down, the first take's truth elbow is not independent of the model, and the second take has few frames with the hands up. A third take answers the three: the same C920 at the webcam position, 4.7 degrees above level, both IMUs worn and the hands up for most of it, 3072 arm frames posed under every lens.

| Focal length scale | Direct, degrees p50 | Direct, depth error cm | Baseline, degrees p50 |
|---|---|---|---|
| 0.85 | 16.6 | 5.9 | 45.0 |
| 0.90 | 16.5 | 6.1 | 45.4 |
| 0.95 | 16.9 | 6.4 | 45.9 |
| 1.00 | 19.9 | 7.5 | 46.0 |
| 1.05 | 20.2 | 8.1 | 46.3 |
| 1.10 | 19.6 | 8.5 | 46.5 |
| 1.15 | 19.7 | 9.0 | 46.6 |
| 1.25 | 19.2 | 9.7 | 46.8 |
| 1.50 | 17.7 | 11.8 | 47.5 |

- On the level camera the direct chain beats the baseline at every lens, 19.9 degrees against 46.0 at the calibration, where the tilted takes had the two tied.

- The calibration is again no optimum. The direction stays between 16.5 and 20.2 degrees, and the depth error grows with the assumed focal length from 5.9 to 11.8 cm, so a lens 15 percent short reads slightly better than the calibrated one.

- The direct error is two populations. On 17 percent of the left arm's frames and 32 percent of the right's the chain is over 60 degrees off: the truth's wrist is about 20 cm nearer the camera than the shoulder, and the chain's is out to the side at the shoulder's depth. The rest read 11.4 and 14.3 degrees at p50 with the depth 4.5 and 6.4 cm off. The bad frames come in runs. Section 34 finds the cause.

- Against the position retarget the direct forearm is 78 degrees p50 on the left and 58 on the right where the baseline's is 44 and 45, and it moves with the lens, from 96 at 0.85 to 53 at 1.50 as the mean of the sides. The reference was not verified on this take, whose right mounting was captured minutes before it with a 36 degree pose spread (plan.md).

Learned: a sensitivity is not an accuracy. The output moving a degree per percent of focal length said nothing about which way, and against the truth the calibrated lens is no optimum: the wrist's depth sets the error, and the lens only trades direction against distance. A what-if made by patching a shared header moves the truth with the candidate, so the candidate needs its own copy. And a comparison over the frames every variant shares is only as wide as its narrowest variant. And a tie on a tilted camera did not carry to a level one, where the rays beat the learned depth by 26 degrees at every lens tried.

## 34. An elbow below the frame, placed at its edge and scored as seen, pushed the direct wrist away from the camera

Section 33 left the level take's direct error in two populations. The bad frames are one pose: both hands reached at the camera, palms down, the forearms foreshortened behind them. `--gate-direct` gained the solver's wrist distance state in its CSV to read them: the reach along the ray, the model's depth prior, the lift, the chosen and filtered distance, the elbow's miss and the cost.

- The hand lift was right. It read within 6 cm of the stereo wrist's distance at p50 on the good and the bad frames alike. The solver chose 0.76 to 0.80 m where the truth and the lift read 0.46 to 0.55.

- The elbow outvoted it. With the arm reached at the camera the elbow is below the frame, and the body model puts one on the forearm where it leaves the image, over the visibility bar: 87 percent of the accepted elbows sat in the bottom tenth of the frame. At the true wrist distance the chain's elbow misses that pixel by about 200 px against a sigma of 20, and the distance that fits it best is 0.9 to 1.1 m.

- The frames over 60 degrees off were 47 percent of the 673 with an accepted elbow in the bottom tenth, 7 percent of the 73 with one above it, and 19 percent of the 1697 with none.

Two changes followed: an elbow in the edge band the wrist already had (0.1 frame heights) is unseen, and the elbow's reprojection counts up to 3 sigmas and no further. The reach against the stereo truth at the calibrated lens, p50:

| Take | Direct before, degrees | Direct after, degrees | Depth error before, cm | Depth error after, cm | Baseline, degrees |
|---|---|---|---|---|---|
| Level camera | 19.9 | 13.8 | 7.5 | 5.9 | 46.0 |
| Webcam position, 18.7 degrees down | 26.4 | 12.8 | 15.9 | 5.6 | 27.3 |

- The tie with the baseline on the tilted take in section 33 was this fault, not the camera's tilt. After the change the direct chain reads 12.8 degrees against the baseline's 27.3.

- On the level take the right arm's frames over 60 degrees fall from 32 percent to 14. The left arm's stay at 17 to 18.

- What holds the left arm is the entry (plan.md). On its reach the model's depth prior reads the wrist 1.35 m behind the shoulder, more than the arm is long. The arm starts at the far end of its ray before the hand lift arrives, and the motion terms hold it there against the lift for some 200 frames.

- With the palms beside the head the truth's wrist is 9 to 11 cm from the shoulder, nearer than the 13.6 cm the chain allows on the project's 0.169 m forearm. The mounting wizard had written the curl's elbow-to-controller radius over the forearm length (plan.md). At the user's 0.26 m the frames over 60 degrees among those with the hands near the shoulders fall from 43 percent to 33, and the take's p90 from 83.6 degrees to 67.6.

Not measured: the two changes apart, the lens sweep after them, and the rig take of 2026-11-12 (plan.md).

Learned: a landmark's visibility score is not evidence that the joint is in the frame, and a tight sigma on such a landmark outvotes a metric cue that is right. And a tie between two candidates can be a fault in one of them: section 33 read it as the method's.

## 35. The real Kalidokit on the level take: half the direct chain's accuracy on a typical frame, the same share of bad ones

Sections 33 and 34 scored a Kalidokit-style baseline, Kalidokit's arm math on MikanTrack's body model. This runs Kalidokit itself: MediaPipe Holistic over the front camera's raw frames of the level take with the settings of Kalidokit's 3d sample, a local Kalidokit 1.1.5 build on those landmarks, and its rotations posed on a T-pose arm with the sample's dampeners and slerp (`tools/kalidokit_ab`). The reach against the stereo truth on the 3072 arm frames every candidate has, after section 34's elbow change:

| Candidate | p50, degrees | p90, degrees | Over 60 degrees |
|---|---|---|---|
| Direct chain | 13.8 | 83.6 | 16% |
| Kalidokit rig on the avatar | 28.4 | 78.3 | 18% |
| Holistic world landmarks, raw | 26.2 | 57.9 | 9% |
| Kalidokit-style baseline on the body model | 46.0 | 85.6 | 27% |

- On a typical frame the direct chain is twice as near the truth as Kalidokit. On the share of frames badly wrong it has no lead, and Holistic's raw landmarks have half as many.

- Kalidokit rests the arm on 15 percent of the frames by its own off-screen rule. On the frames it poses it reads 25.5 degrees with 9 percent over 60.

- With the hands near the shoulders (510 frames, the truth's wrist less than 10 cm nearer the camera than the shoulder) every candidate is poor: 45.0 degrees for the direct chain, 45.4 for Kalidokit, 35.4 for Holistic's landmarks.

- The direct chain is the steadiest. Its reach turns 1.3 degrees between model frames at p50 and 4.1 at p90, where the truth turns 1.5 and 4.2, Kalidokit's 1.8 and 8.3, and Holistic's landmarks 2.8 and 11.3.

- The Kalidokit-style baseline is not a stand-in for Kalidokit: on the body model's depth it reads 46.0 degrees where Kalidokit on Holistic's reads 28.4.

A what-if with the wrist on the direct chain's own ray at a fixed distance ahead of the shoulder, no depth solved:

| Wrist | p50, degrees | p90, degrees | Over 60 degrees |
|---|---|---|---|
| The chain's | 13.8 | 83.6 | 16% |
| 0.20 m ahead | 13.8 | 46.4 | 8% |
| 0.25 m ahead | 12.4 | 49.2 | 8% |

The plane matches the chain's median and halves its tail. It is fitted to this take, where 83 percent of the arm frames are a reach at the camera, and on the frames with the hands near the shoulders it is worse, 54.9 degrees at 0.20 m against the chain's 45.0 (plan.md).

One take and the arms only: the face, the blinks and the fingers are not compared, and Python's Holistic stands in for the browser's.

Learned: a baseline that borrows a method's math and not its inputs measures neither. And a median hides what a viewer sees: the direct chain leads by half on the typical frame and not at all on the frames that look wrong.

## 36. The wrist distance scorer retired for two depth planes

The side-by-side video settled what the numbers had not. At desk distance a webcam's field of view leaves the hands a band about 10 cm deep, and an estimate inside that band bought less expression than XY and the fingers already gave. The scorer's cost was visible: a distance that jumped, and an elbow that followed it. Kalidokit's arm read as more natural on screen with a worse chain, because its joints stay inside limits.

The direct solver now puts the wrist where its ray crosses a plane ahead of the shoulder, at 0.667 of the reach, or at 0.25 while the palm is at the head. The near test is a ratio of pixels against the ear separation, so no distance enters it. The first version compared the hand lift's distance with the head's and never fired: the two were 0.2 m apart on a hand touching the head. The elbow follows the image direction from the wrist to the body model's elbow, which holds even where the model has placed that elbow by guess.

The scorer, its depth prior, its lift scale, the stereo-truth gate tool and the frozen copy the golden compared it with were deleted in the same change. The planes were judged by eye and not scored against the truth.

## 37. The body tracker's periodic re-detect lost a seated user for seconds

A direct take showed both arms dropping to rest for 1.9 s with the hands in view and both hand slots tracked at 0.99. The body model had lost the person: its mean keypoint score fell from 0.53 to about 0.15 and it put both wrists on one pixel.

The cause was the drift guard. Every 20 model frames the tracker took the person detector's box in place of its tracked one. On 9 of that take's 12 re-detects the detector's crop scored 0.09 to 0.36. A frame with fewer than three confident keypoints then dropped the tracked box, so the next frame ran the detector again, and the tracker stayed on detector crops until one worked: 42 of 294 model frames lost, 29 of them in a row.

The detector's box is now scored against the tracked one before it is used, on the divided-out frame after the model frame, where the landmark model is idle. Replayed at the live cadence the same take loses the person on no model frame, and no frame's stage time passes 26 ms. Scoring it on the model frame itself would have cost about 41 ms against a 32 ms camera period.

## 38. The body model on every frame: it fits a 30 fps period, and both models run faster for it

On the one-camera project the avatar's elbows, torso and head read as choppy beside a wrist that moved smoothly. The wrist comes from the hand model at 30 Hz, and the rest from the whole-body model, which ran every other frame. Two Tracy captures of the same project, one at each divider:

| Zone | Divider 2, p50 | Divider 1, p50 | Divider 1, max |
|---|---|---|---|
| `RtmPoseModel::estimate` | 15.8 ms | 10.7 ms | 13.3 ms |
| `HandLandmarkModel::estimate` | 1.8 ms | 1.2 ms | 2.3 ms |
| Camera process, one model run | 21 ms | 15.6 ms | 19 ms |
| Camera process, challenge frame | | 28 ms | 31.4 ms |

- At a divider of 1 the loop held 30.0 fps over 1216 frames, and no frame passed the 32 ms camera period. The choppiness went.

- The models ran a third faster with the GPU busy on every frame. GPU clocks were not logged, so power management is the likely cause and not a measured one.

- The challenge frame, one in 20, runs the landmark model twice and peaked 0.7 ms inside the period. At the divider 2 capture's model speed it would not have fit.

- In both captures about every 12th camera frame arrived 46 ms after the one before, behind frames of ordinary length. The capture callback itself fires on a 16 ms grid: eleven intervals of 32 ms and one of 48, every 400 ms, which is twelve frames at 33.3 ms. Recordings show the same pattern on the C920, the PW315 and the C960. What sets the grid was not identified. A frame's timestamp is its arrival time, so every stage that reads an interval reads 32 or 48 ms for 33.3 ms of motion.

- Stamping a frame with its capture time, from Media Foundation's sample time plus the smallest recent difference between arrival and sample time, was built and removed. A probe through OpenCV's reader had read sample times 33.3 ms apart on every frame. Through the app's own reader the sample times sit on a 16 ms grid of their own: the difference from arrival read 16 ms for nine frames and 0 for three, and the mapped intervals came out at 32, 48 and 16 ms, worse than the arrivals'. Why the two readers differ was not looked at.

- What evenly spaced timestamps would buy was measured offline and is small. A tri-camera take (4570 frames) was copied with each camera's times moved to a line fitted under its arrivals, and both copies were replayed. The fused palms moved 0.4 to 0.5 mm at p50 and 2 to 3 mm at p90, and their frame-to-frame second difference stayed within 3 percent, as did the elbows', shoulders' and head's. Fusion takes each camera's newest landmarks whatever their times, so a truer timestamp changes the filters' intervals and not which observations are fused.

The challenge now takes the frame's one landmark run at a divider of 1. Where the detector's box wins, its landmarks are the frame's observation. Where it loses, the frame re-emits the held observation. Replayed over two one-camera takes (588 and 645 frames) the person is lost on no model frame, 19 and 11 frames are held, the detector's box is taken on 7 and 21, and the longest stage time is 20.4 and 19.6 ms against a median of 11.0. Not yet captured live.

Learned: the divider was set when the model was measured at 18 to 24 ms, and nobody had measured the frame it was protecting. A cadence chosen to save time can cost the speed it was saving.

## Cross-cutting lessons

1. Measured quality beats reported confidence. Presence scores, model confidences and self-consistent calibration metrics all failed as quality signals. Reprojection residuals, measured jitter and reconstruction-geometry checks did not.
2. Keep one measurement path independent of your calibration. The 30% world-scale error survived every internal metric and fell to a ruler.
3. Record everything, replay deterministically. The dump system (built week one) and the replay system (built week two) drove nearly every diagnosis afterward. The pattern was always: user cites a timestamp or frame, read the data, form a theory, test it against the same data.
4. Labels are bookkeeping, evidence is measurement. Slot labels, preview overlays and classifier argmaxes all lied at some point; raw scores and geometry did not.
5. A/Bs have their own failure modes: confidence gates censor (survivorship bias), free motion confounds jitter, mixed conventions read as error, and a miscalibrated rig can invert a verdict. Identical-motion replays and scripted tests are the antidotes.
6. Models carry their training distribution with them: overhead views broke the person detector and degraded the handedness classifier, and the canonical hand matched nobody's hand. Geometry plus per-user calibration filled every one of those gaps.
7. Winning the benchmark is not winning the argument. RTMPose won its A/B and was still parked on cost. The two-camera rig beat a depth camera on articulation. Field of view beat both.
8. Delete the losing arm. Every settled A/B ended with the toggle and the dead code removed, which is why the codebase still reads like a design rather than an archaeology site.
