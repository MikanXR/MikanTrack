# Pipeline panel

A proposal to replace most of the app's settings panels with a Pipeline strip, a stage inspector and a central view that follows the selected stage. The interactive mockup is [pipeline-panel-mockup.html](./pipeline-panel-mockup.html), a self-contained file that opens by double-click. This document records the decisions the mockup is built on and the checklist that maps every control in today's panels to its new home.

Status: the mockup's three passes are built (the shell, every inspector populated, then the Retarget chain editor, the Source pose tables, pinned windows and the Performance chart). The implementation follows the order below. M0 through M16 are in, the UI track and the first step of the retarget track: the strip, the Inspector and the Performance panel are in the app, every panel section is a function an inspector can host, a recording carries the stage state a replay shows, and the Capture, Fusion, IMU, Body solver and Face and mouth stages' controls have moved into the Inspector, with the Device, Calibration and Audio panels and the Avatar panel's Face tab gone. The Face and mouth stage itself holds no controls and lists its sub-stages as rows that select them. The Retarget stage holds the avatar's load controls and summary and a tab bar of Placement, Mapping, Retarget and the interim Loss group, with the Avatar panel gone and Load Avatar and Unload Avatar added to the File menu. Output holds the stream's enable and its send rate and frame size, Delivery rules the max rate, the confidence gate and each hand's confidence against it, VMC stream the target and the head offset, and the Source pose the finger angle tables, with the OSC Output panel gone. The Direct solver holds the camera, the neutral capture, the hand calibration note and Measure My Body as inputs, the switch, the tilt and the body measurements with the forearm length as settings, and the hold-still test with a summary of the shown solve as diagnostics. Its Filters and gates, Torso and head, Arm chain, Palm and fingers and Hold and rest sub-stages show the solve state that reached only the dump before, and Hold and rest holds the rest blend as its interim setting. A direct project shows no wrist IMU controls, since the direct pipeline does not run the IMU. The Body solver's elbow source is not shown, because the fused result does not say whether a forearm came from the wrist IMU or the body camera. The Tracking panel is the only old panel left. It holds the dump buttons, and the direct mode switch while the mode is off, since a multi-camera project has no Direct solver stage to hold it. The Stage View has replaced the 3D Scene and Video Preview tabs: it shows the cameras for Capture and the Direct solver and the scene for everything else, with the scene's layers chosen by the stage, and its header holds a mode that overrides the selection (follow, cameras, 3D scene) and the view from camera toggle. The mockup's face value bars, the Mapping figure in the view and the per-sub-stage camera overlays are not built. A double-click on a stage's box pins it into its own dockable window, its view on the left and its inspector on the right, with a window id per stage so a stage pinned again returns to its place. The Performance panel charts the last 600 iterations' time per phase with the hitch threshold and a marker on each hitch. Its phases are the vision thread's, not the mockup's per-stage split. The expression language compiles against an input table the caller passes, and a value-or-function setting type with its runner exists for the chain parameters, with no setting using it yet. The source pose exists as the hand-off both pipelines fill, with the direct solver split into measure and place and the position solve reading it, and every output bit unchanged. A retarget stage (source fill, loss stage, target solve) runs on the vision thread every output frame, the stream only encodes its result, and the window shows the same pose. The loss stage holds the stream's gate, hold and freeze and the direct solver's ease as data policies with today's values. The Source pose stage shows the source pose after the loss stage: which pipeline filled it, its finger zero and clock, a row per group with its state, confidence and what it carries, and the finger angles the target solve takes. The table is per group, not the mockup's per-bone one. The retarget's clock holds across the small backward steps that interleaved face-only frames cause, so a hold lasts its duration and the root does not snap on them.

---

## Why

The Tracking panel holds twelve sections from different pipeline stages in one scroll: settings, live state and actions together. Reading it means knowing which stage each row belongs to. The vision thread already runs a fixed sequence of named stages, and most stage state is already published. The proposal shows that structure directly.

## Decisions

### Main window

- A Pipeline strip across the top, a central view, an inspector docked on the right, and a bottom dock.

- The bottom dock has three tabs:
	- Timeline: recording, replay, the live or replay indicator, the diagnostic dump buttons
	- Log
	- Performance: per-stage milliseconds and the hitch history

- The Tracking, OSC Output, Avatar, Audio, Calibration and Device panels fold into stage inspectors. The Video Preview and 3D Scene panels become the central view.

- With nothing selected, the central view is the whole 3D scene and the inspector shows a project summary with a list of the stages that need attention.

### Pipeline strip

- Two levels. The top row holds the coarse stages and never reflows. Selecting a stage that has sub-stages fills a second row beneath it.

- Each box shows a status dot and one key number. The four states are running, degraded, inactive and not configured. Hover text gives the stage's description and the reason for a dot that is not green.

- A box's width is fixed by its name and a value slot per box style, so a changing number never resizes it. A value that does not fit ends in an ellipsis and reads in full in the hover text and the Inspector.

- Nobody to track is not a fault. With the hands out of view a stage runs and its value says so ("no hands", "arms: 0"). The mockup marked those stages degraded, which would leave the strip amber whenever the user sits back.

- One data item is selectable: the Source pose, drawn as a pill between the solvers and Retarget. Every other hand-off is a plain arrow.

- The strip shows the active project's shape. A stage the project does not use draws greyed in place. In direct mode one Direct solver box stands where Fusion, IMU and Body solver are.

### Stages

- Multi-camera top row: Capture, Fusion, IMU, Body solver, Source pose, Retarget, Output. Face and mouth joins at Output.

- Direct top row: Capture, Direct solver, Source pose, Retarget, Output. Face and mouth joins at Output.

- Sub-stages:
	- Capture: one lane per camera, each with Video, Hand models, Body model, Lift. The lane's camera label is itself selectable.
	- Direct solver: Filters and gates, Torso and head, Arm chain, Palm and fingers, Hold and rest
	- Face and mouth: Phone, Microphone, Profile
	- Output: Delivery rules, VMC stream

### Selection and windows

- A single click selects a stage. The central view becomes that stage's visualization and the inspector shows its properties.

- A double-click pins the stage into its own window, with the visualization on the left and the properties on the right. Several can be open, and in the application they dock like any other panel.

- Every stage view follows the shown frame, live or the Timeline's replayed frame. A stage whose state is not recorded says so in replay (the IMU filter is the present case).

### Inspector

- One template in one order: Status, Inputs and calibration, Settings, Diagnostics. An empty section is hidden. Diagnostics is collapsed by default.

- Status says what the stage is doing on the shown frame and, when it is not running, why.

- Inputs and calibration lists what the stage depends on, each with its state and the wizard that produces it.

- A view option (an overlay toggle in today's Tracking panel) lives in the Settings of the stage whose visualization it affects, under a View sub-heading.

- Loading and unloading the avatar is in both the File menu and the Retarget stage's inputs.

### Retarget and the source pose

These come from the retargeting discussion that preceded the panel proposal. The mockup's Retarget stage and Source pose pill show them.

- Both pipelines fill one source pose: a skeleton indexed by the VRM humanoid bone table in the user's proportions, with parallel tables for confidence, for which of position, direction and roll were measured, and for each group's state (measured, held, lost). Fingers stay as the finger angle table, with an enum naming its zero convention (the captured rest pose, or the metacarpal).

- Retarget maps the source pose onto the avatar chain by chain: torso, head, each arm, each hand. An arm chain runs the angle mapping first and then pulls its end toward a goal by an IK weight. Multi-camera projects default to IK weight 1 and direct projects to 0.

- Loss handling splits in two. Whether a group is measured this frame is decided by the pipeline that fills the source pose. What happens after a loss is one selectable policy per chain: a hold duration, then freeze, ease to rest, or release. The defaults keep today's behavior: 250 ms then freeze for multi-camera, the earned hold then ease to rest for direct.

- A chain's numeric parameters can each be a plain value or a function over the source pose's tables, written in the face expression language. In the mockup an fx button beside the slider swaps between the two, and the expression text is inert.

- The Retarget view draws the source skeleton over the avatar, or side by side. Measured bones are solid, inferred ones dashed, lost ones red, and the color runs green to amber with confidence. A click on a bone selects its chain. Each arm's IK goal is marked on the avatar.

## Checklist: where each control goes

Every section of today's panels, and the stage and inspector section that receives it. Rows marked new are readouts that exist today only in the diagnostic dump or not at all.

### Tracking panel

| Today | Stage | Section |
|---|---|---|
| Flip handedness | Capture | Settings |
| Fusion: dominant camera, per-camera confidence table | Fusion | Diagnostics |
| Hand Scale: measured length | Fusion | Diagnostics |
| Image Quality: the eight metrics per camera | Capture, as one table across cameras | Diagnostics |
| Wrist IMU: enable, swap wrists, forearm length | IMU | Settings |
| Wrist IMU: device, rate, yaw drift, roll error table | IMU | Diagnostics |
| Wrist IMU: Calibrate Mounting | IMU | Inputs and calibration |
| Face: enable, port, phone address, align and reset alignment | Face and mouth, Phone | Settings |
| Face: stream status, unknown names, parse failures, head rotation, alignment state | Face and mouth, Phone | Diagnostics |
| Body Pose: per-camera enable and model | Capture, camera, Body model | Settings |
| Body Pose: Measure My Body | Body solver | Inputs and calibration |
| Body Pose: upper arm from shoulder width, ratio, upper arm, shoulder width, head width, nose forward | Body solver | Settings |
| Direct Retarget: enable, camera tilt | Direct solver | Settings |
| Direct Retarget: camera, capture and clear neutral, neutral state | Direct solver | Inputs and calibration |
| Hand Calibration: bones and rest pose state, Calibrate Hands, Clear Hand Calibration | Fusion | Inputs and calibration |
| Hand Calibration: the direct mode note | Direct solver | Inputs and calibration |
| Overlay: show skeleton overlay, show detection boxes, show every hand slot | Capture, camera, Hand models | Settings, View |
| Overlay: show body landmarks | Capture, camera, Body model | Settings, View |
| Overlay: 3D per-camera skeletons | Fusion | Settings, View |
| Overlay: 3D view from camera | Central view title bar | |
| Inference: execution provider per camera, inference time | Capture | Diagnostics |
| Inference: frame loop hitches | Bottom dock, Performance | |
| Capture Timing: the thirteen rows per camera | Capture, as one table across cameras | Diagnostics |
| Diagnostics: dump now, dump in 3 s, last dump path | Bottom dock, Timeline | |
| Diagnostics: hold-still jitter test and its result | Fusion | Diagnostics |

### OSC Output panel

| Today | Stage | Section |
|---|---|---|
| Enabled | Output | Settings |
| Target IP, port | Output, VMC stream | Settings |
| Max rate, min confidence | Output, Delivery rules | Settings |
| Dropout hold, freeze on loss | Retarget, each chain's loss policy | Settings |
| Head offset, the streamed bones notes | Output, VMC stream | Settings |
| Avatar mouth expressions | Face and mouth, Profile | Diagnostics |
| Per hand: confidence, withheld state | Output, Delivery rules | Diagnostics |
| Per hand: palm position | Fusion | Diagnostics |
| Per hand: wrist bend, forearm and wrist quaternions | IMU | Diagnostics |
| Per hand: elbow | Body solver | Diagnostics |
| Per hand: finger angle table | Source pose | Diagnostics |

### Avatar panel

| Today | Stage | Section |
|---|---|---|
| Load VRM, Unload, path, load error | Retarget, and the File menu | Inputs and calibration |
| Model tab: name, author, license, counts, bones mapped, arm and shoulder comparison, warnings, rig sidecar | Retarget | Inputs and calibration |
| Model tab: show in 3D scene, follow shoulders, root position, root yaw, reset | Retarget | Settings, Placement |
| Mapping tab: view buttons, figure, reset all, filter, bone table with node pickers | Retarget | Settings, Mapping. The figure draws in the central view |
| Mapping tab: head, hand and forearm roll trims | Retarget | Settings, the head, hand and arm chains |
| Retarget tab: preview pose | Retarget | Settings |
| Retarget tab: rest blend | Retarget | Settings, each chain's loss policy |
| Retarget tab, per arm: elbow hint, measured elbow confidence, rest elbow, wrist, palm and finger curl, gizmo, reset | Retarget | Settings, the arm chain |
| Retarget tab, per hand: thumb pronation, curl and splay gain, fingers driven, reset | Retarget | Settings, the hand chain |
| Face tab: active profile, new, duplicate, delete, assign, unassign, name | Face and mouth, Profile | Inputs and calibration |
| Face tab: variables and VRM expression bindings tables | Face and mouth, Profile | Settings |

### Audio panel

| Today | Stage | Section |
|---|---|---|
| Microphone, refresh | Face and mouth, Microphone | Inputs and calibration |
| Profile path, Calibrate, Import, Clear profile | Face and mouth, Microphone | Inputs and calibration |
| Enable, gate, gain, model, smoothing, engine smoothing | Face and mouth, Microphone | Settings |
| Level meter, device, model, mouth owner, dropped blocks, the fifteen viseme weights | Face and mouth, Microphone | Diagnostics |

### Calibration and Device panels

| Today | Stage | Section |
|---|---|---|
| Per camera: intrinsics state and numbers, Calibrate Intrinsics | Capture, camera, Lift | Inputs and calibration |
| Per camera: extrinsics state, camera height | Capture, camera, Lift | Inputs and calibration |
| Extrinsics (all cameras): note, Calibrate Extrinsics | Capture | Inputs and calibration |
| Hand scale seed | Fusion | Inputs and calibration |
| Rescan Devices, Add Camera | Capture | Inputs and calibration |
| Per camera: device, resolution, frame rate, format, start and stop stream, mode, delivery rate, dropped frames, Remove Camera | Capture, camera | Inputs and calibration |
| Per camera: Camera Settings sliders, Reset Defaults, Re-read | Capture, camera, Video | Settings |

### What stays

- The Timeline panel keeps every control it has and gains the dump buttons. It becomes the bottom dock's first tab.

- The Log panel becomes the bottom dock's second tab.

- The Settings panel (the language choice) stays a panel.

- The File and Calibration menus stay. The View menu shrinks to Pipeline, Inspector, Timeline, Log, Performance and Settings.

- The wizards and the guided setup are unchanged. A stage's Inputs and calibration section launches the same wizard the Tracking or Calibration panel launches today.

### New readouts the stages make room for

- Direct solver and its sub-stages: the solve state that today reaches only the F9 dump (landmark visibility and the entry gate, shoulder depth, lean, head source, the reach interval, the chosen depth, the elbow miss, the depth prior, the lift distance and scale, each group's hold and rest state).
- Body solver: the elbow's source (measured or dead reckoned) and the shoulder confidence.
- Body model: the person box source and the mean keypoint score.
- Output: the send rate, the bundle sizes and the avatar announcement.

## Implementation order

A golden test harness first, then the UI track, then the retarget track. Each milestone ships on its own and ends at a review.

- M0: the retarget golden harness, the live retarget path against a frozen copy of it (testing.md)
- M1: the stage model and its status rules, the read-only Pipeline strip, an inspector with Status only, the dock layout rebuilt once with View > Reset Layout
- M2: the Tracking and OSC panels split into per-section functions that commit on their own, with their per-frame logic run whether drawn or not
- M3: recordings carry the stage state that is not recorded today, as an optional block at the same format version, and the strip reads it in a replay
- M4 to M11: one stage per milestone moves out of the old panels (Capture, Fusion, IMU, Body solver, Face and mouth, Retarget, Output, Direct solver), and a panel is deleted when it empties The Fusion milestone also has the Timeline ask the replay for fusion diagnostics, which nothing reads before then.
- M12: the Stage View replaces the 3D Scene and Video Preview tabs
- M13: pinned stage windows and the Performance tab
- M14: the expression language's input table becomes a parameter
- M15: the source pose, with both solvers' output bit-identical to M0
- M16: one shared target solve and loss stage, run every output frame on the vision thread and published, so the display shows what is streamed
- M17: chain settings in the project file and the sidecar, the Retarget inspector regrouped into chains, the Retarget view
- M18: value-or-function parameters edited inline
- M19: the new behaviors (IK weight in both pipelines, selectable loss policies, reach scale and goal offset, rotation limits)

Three things hold through the retarget track:

- Rig corrections stay in the avatar's sidecar. The mapping mode, IK weight, reach scale, goal offset and loss policy are the project's and go in `project.json`.

- The direct source pose stays in the avatar's units and carries the forearm twist as measured against the avatar's rest palm, so splitting the solver changes no output.

- The elbow parameter becomes a direct weight toward the measured elbow. Its default is the function form of today's rule, `clamp(source.<side>LowerArm.confidence / threshold, 0, 1)`, so output is unchanged until someone switches it to a plain weight.

Until M17, two pieces of the UI are interim: a flat Loss group under Retarget (dropout hold, freeze on loss, rest blend) and the Direct solver's Hold and rest sub-stage.

## Open points

- The Retarget inspector shows one chain at a time below a six-row chain list, which is what keeps it readable. Whether one chain at a time is enough when tuning both arms together is untested. The arm chain has a "Copy to the other arm" button for that case.

- Several chain parameters are new and have no solver behind them yet: the IK weight, the reach scale, the goal offset, the release policy, and the limits on a multi-camera project.

- The second row of the strip keeps a fixed height so the layout below never moves. It leaves a blank band under a stage with one lane or none.

- The mockup's stage visualizations are 2D canvas sketches. They test where a view goes and what it shows, not 3D interaction.
