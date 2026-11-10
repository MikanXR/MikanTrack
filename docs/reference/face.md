# Face

MikanTrack receives ARKit face data from the iFacialMocap iPhone app. The app streams 52 blendshape weights plus head and eye rotation as UDP text. `FaceService` (`src/Face/FaceService.h/.cpp`) owns the socket, the handshake, and the parser. It publishes the newest sample as a `FaceSample` (`src/Face/FaceTypes.h`). The phone and the microphone's visemes reach the avatar through a face profile: user-authored expressions over the raw sources, described below.

## Protocol

The app streams to a machine that asks it to. The ask is a datagram carrying a fixed ASCII text starting with `iFacialMocap_` and no terminator, sent to the phone address on the stream port. The service sends it as a burst of five copies when its socket binds or the phone address changes, and again every 3 seconds while no stream datagram has arrived for 3 seconds. The default phone address is the broadcast address `255.255.255.255`, which reaches a phone anywhere on the LAN.

The socket is bound to the stream port with `SO_BROADCAST`. A broadcast handshake loops back to the same port on this machine, so any datagram starting with `iFacialMocap_` is dropped as our own.

A stream datagram is one frame of text, well under the 4096-byte receive capacity:

- Blendshape section before the first `=`: `|`-separated entries `name-value` (v1) or `name&value` (v2, allows negatives). Values are percent and are stored as value/100.
- Bone section after the `=`: `|`-separated entries `name#numbers`. `head#rx,ry,rz,px,py,pz` needs all six values. `rightEye#rx,ry,rz` and `leftEye#rx,ry,rz` need three.

Two status fields share the blendshape section. `trackingStatus` is the phone's face-tracked flag (1 tracked, 0 lost) and becomes `FaceSample::faceTracked`. `hapihapi` is an app field with nothing a face needs and is skipped. While the phone reports the face lost its values are stale: the phone source reads zero and the camera head stands. Blendshape names come from the ARKit table in `FaceTypes.cpp`. The phone's `_L` and `_R` suffixes fold to `Left` and `Right`. Unknown names are counted and skipped. A datagram where no entry parsed counts as a parse failure and changes nothing. Values a datagram omits keep their previous state.

Samples stay in the phone's convention: rotations in degrees, positions in the app's units. The head rotation enters MikanTrack's head frame at one conversion point, `faceHeadDeltaFromPhoneEuler` (`src/Face/FaceHead.h`, see [conventions.md](./conventions.md)).

## Service and threading

`App` owns the service like `ImuService`: created in `App::startup`, `startup()` on project activation, `shutdown()` wherever the IMU service shuts down. `App::applyFaceConfig()` maps `AppConfig::face` to `FaceServiceConfig` and calls `setConfig`, which binds or releases the socket on a port or enable change. `App::getFaceService()` gives the UI its handle.

The service has no thread of its own. The vision thread calls `update(steadyNowMs())` once per iteration, in its own `Face` hitch-watchdog phase right after the capture stage. `update` drains every waiting datagram and the last one wins, then runs the handshake schedule. One mutex guards all state, so `setConfig`, `getStatus`, and `getLatestSample` are safe from any thread. A sample counts as streaming while its datagram is younger than 0.5 seconds. `timestampMs` is the `update` time on the shared `steadyNowMs()` clock, the same clock camera frames use.

## Sources

`TrackingFrameResult::face` is a `FaceSources` (`src/Face/FaceSources.h`): the raw face sources side by side, and the face profile evaluated over them.

- `phone`:
	- `present`: the stream is live (`FaceService::isStreaming`)
	- `tracked`: `present`, and the phone sees a face
	- `blendshapes[52]`: raw, in ARKit order
- `viseme`:
	- `present`: the microphone's newest `MouthSample` ([audio.md](./audio.md)) is no more than a second old
	- `weights[15]`: after the emission filter
- `outputs`: the active face profile's values for this frame, with the generation of the binding that produced them

A source's values read zero unless it is tracked: the phone's blendshapes while the stream is quiet or the phone reports the face lost, the visemes past the stale window. An expression without a guard therefore relaxes instead of freezing on the last raw values. The face is present while either source is.

`VisionThread::applyFaceToOutput` fills both sources on every output frame and decides nothing about who drives which shape. That choice belongs to the profile. It also keeps the head duties: the phone's head rotation replaces the camera head's orientation while the stream is live and tracked, with the head marked valid at full confidence. The phone measures the head far better than the body-pose stage. The position stays the camera head's, since the phone's is relative to the phone.

The phone runs faster than the cameras, the microphone's mouth frames arrive at 30 Hz on their own thread, and both keep flowing when the cameras stop. A new sample of either that no camera frame carried goes out on its own (`runFaceOnlyOutput`). That frame repeats the last output with the new sources and the sample's own timestamp. Once the last camera frame is older than the dropout window, the repeated hands go out untracked, so the streamer's hold and freeze rules see a loss rather than a pose that never ages.

The phone reports its head rotation away from looking straight at the phone. The anchor is the world orientation of that straight ahead. Unaligned it is identity, so looking at the phone faces world +X, the avatar's rest facing. "Align head to camera" captures the anchor at the next camera frame where both the body-pose head and the phone head are measured (`faceHeadAnchorFromWorld`), so a phone mounted off to one side still turns the head about the right axes.

## Profiles

A face profile (`FaceProfile`, `src/FaceExpr/FaceProfile.h`) maps the sources to the avatar's face: variables, which are named intermediates, and outputs, the VRM expression bindings, that each drive a target with a function.

- `id`: a UUID minted when the profile is created or duplicated, never changed.

- `name`: the display name, free to change.

- `variables`: `{name, expr}`. A file that carries them under the key `lets` loads the same.

- `outputs`: `{targetKind, target, mesh, vrmExpressionFunction}`. A binding that carries its function under the key `expr` loads the same. `targetKind` is `expression` (a VRM expression, by its name in the file) or `morph` (a morph target, by name). `mesh` narrows a morph target to one mesh. Without it the target is that morph on every mesh carrying the name.

The library (`FaceProfileLibrary`) is one JSON file per profile, `<id>.json`, in `Documents/MikanTrack/FaceProfiles/` beside the project folders. The project list is not a scan of that root, so the folder costs nothing but the name. A rename rewrites the name inside the file and nothing else. Edits save after a three-second settle, the way rig edits do.

An avatar's rig sidecar (`<stem>.mikanrig.json`, [avatar.md](./avatar.md)) assigns profiles by id (`faceProfiles`) and names the active one (`activeFaceProfile`). One avatar can have several profiles and one profile can serve several avatars. The assignment follows the VRM file across projects, as the rig does.

### Auto

The built-in Auto profile (`makeAutoFaceProfile`, `src/FaceExpr/FaceAutoProfile.h`) is generated per avatar, read-only, and never stored. It runs whenever the avatar has no active profile, when the active id is not in the library (a sidecar that traveled to another machine, a deleted profile), and for a profile that has never compiled. "Duplicate to edit" copies it into the library under a new id. Its rules:

- Expressions named after ARKit blendshapes (perfect sync): those expressions alone, one column each, in the avatar's own spelling.

- Otherwise the VRM presets the avatar carries, each from the ARKit columns that shape it: the blinks from the blinks, the vowels from the jaw and mouth, the gaze directions from the eye looks. The two-eyed blink only when the avatar cannot blink each eye on its own. The emotion presets get no output. Plus the ARKit names the avatar carries as morph targets, as morph outputs.

- No avatar: the 52 ARKit names as morph outputs.

An ARKit column reads the phone while it sees a face and otherwise the visemes the viseme table spreads over it: `select(phone.tracked, phone.jawOpen, 0.85 * viseme.aa + 0.35 * viseme.E + ...)`. A vowel preset reads its viseme directly: `select(phone.tracked, phone.jawOpen, viseme.aa)`. The phone therefore wins while it streams a tracked face, and the microphone drives the mouth otherwise, with the eyes and brows at rest.

### Binding and evaluation

`FaceProfileBinding` is a compiled profile resolved against one avatar: the program, the targets with the outputs grouped per target, and the morph binds. It is immutable and carries a generation unique in the process. A profile edit, a profile switch or an avatar change builds a new one. The combine rules:

- An output's value is clamped to 0..1.
- Outputs sharing a target combine with max.
- An expression target drives its morph binds times the bind weight, a morph target its morphs at weight 1, and a morph driven by several targets takes their max.
- A target the avatar lacks is reported and skipped, never fatal, so one profile stays portable across similar faces.

The vision thread evaluates the binding once per output frame in `runOutputStage`, before the OSC rate gate, through its `FaceProfileRunner`, so the stateful functions advance exactly once per frame. The values go into `frame.face.outputs` with the binding's generation. The streamer sends one `/VMC/Ext/Blend/Val` per target name ([wire-protocol.md](./wire-protocol.md)). The 3D scene, the Profile stage's mouth readout and its bindings rows read the values off the frame, matched to the binding of the same generation, and the 3D scene resolves morphs through that binding. A frame from an older generation shows the rest face until the next arrives. The 3D scene applies the values only while the binding's morph count per mesh matches the loaded model's.

The binding reaches the vision thread through `VisionThread::setAvatarIdentity`, applied between frames. The evaluator state carries over by call site, so editing one output does not restart the smoothing of the others. An edit that does not compile leaves the last good binding running and shows its errors.

## Expression language

Expressions are parsed by a hand-written Pratt parser (`src/FaceExpr/FaceExprParser.h`) and compiled to a flat register program (`FaceExprProgram`, `FaceExprCompiler`). A profile compiles whole or not at all.

Every error carries the variable or output it belongs to, an error code with its arguments, and where it points. A variable-name error (an invalid name, a function's name, a name defined twice) points at the variable's name field. Every other error carries the character span of the expression text it is about, and a cycle error's span is the reference that closes the cycle. The Face tab formats the code and its arguments as localized text in English and Japanese, such as "variable 'x' is defined twice", "'x' is not a valid variable name" and "variable 'x' has an error". An unresolved target's reason is a code with its arguments too, formatted the same way.

Names compare without case, the input namespaces and the variable names alike:

- `phone.<arkitName>`: a phone blendshape, ARKit spelling or the phone's `_L`/`_R` suffix (`phone.jawOpen`, `phone.eyeBlink_L`)
- `phone.present`, `phone.tracked`
- `viseme.<name>`: a viseme weight (`viseme.aa`, `viseme.PP`, `viseme.sil`)
- `viseme.present`
- the profile's variable names

An unknown name is a compile error. The face input table (`FaceExprInputs`) is the one place a new face namespace is added.

The language itself is not tied to that table. The compiler takes an `ExprInputTable` (`src/FaceExpr/ExprInputTable.h`: a slot count, a slot by name without case, a name by slot), and `compileFaceProfile(profile)` is the form that passes the face table, `FaceExprInputs::table()`. A program records the input count it was compiled against, and `FaceExprEvaluator::evaluate` over a caller-filled array of that many floats is the general entry, the `FaceSources` form filling the face table's values and calling it. A count that does not match runs nothing, writes zeros and returns false. `ExprNameTable` is a table built from a list of names.

`ExprParameter` (`src/FaceExpr/ExprParameter.h`) is a setting that is a plain value or a function over some table: the value, the expression text and which of the two is in force. It saves as a bare JSON number while it is a plain value with no text, so a numeric setting that becomes one reads and writes existing files unchanged, and otherwise as an object of `value`, `expression` and `useExpression`. `ExprParameterRunner` evaluates one such setting at one site: it compiles the text as a one-output profile, again only when the text or the table changes, and returns the expression's value clamped to the parameter's range, or the plain value clamped when the setting is a plain value, the text did not compile, or the result is not finite. It keeps the compile error and the last raw value for an editor. No setting uses the type yet.

Operators, loosest first, all left associative:

- `||`
- `&&`
- `==` `!=`
- `<` `<=` `>` `>=`
- `+` `-`
- `*` `/`
- unary `-` `!`

A value counts as true above 0.5. Comparisons and logic yield 0 or 1. Division by zero yields 0, and any non-finite intermediate becomes 0.

Functions:

- `min(a, b, ...)`, `max(a, b, ...)`
- `clamp(x, lo, hi)`
- `abs(x)`, `pow(x, y)`
- `lerp(a, b, t)`
- `remap(x, inLo, inHi)` and `remap(x, inLo, inHi, outLo, outHi)`: clamped to the input range, output 0..1 by default
- `smoothstep(edge0, edge1, x)`
- `select(c, a, b)`
- `smooth(x, tauSeconds)`: exponential smoothing, stateful
- `hysteresis(x, on, off)`: 1 once `x` reaches `on`, 0 once it falls to `off`, stateful

The function table is `FaceExprFunctions` (`src/FaceExpr/FaceExprFunctions.h`): each function's name, opcode, arity, statefulness and the argument names an editor inserts as placeholders. The compiler resolves calls through it and the expression editor's palette enumerates it, so a function added there appears in both.

Each stateful call site has its own slot in a `FaceExprState` kept apart from the program, so one program can drive several evaluators (the live stream, the timeline's replay). Every call site advances every frame, including in the branch a `select` does not take. A slot's key is its variable or output, that expression's text, and the call site's ordinal in it. A recompile carries over the slots whose key still exists.

Time: the evaluator steps on the frame's own timestamp. The step a stateful call sees is the timestamp's advance clamped to zero and one second. Camera and face-only frames interleave a few milliseconds out of order, so a small regression steps nothing and keeps the clock where it was. Only a regression of more than a second, or an explicit reset (the timeline's seek), reinitializes the state. A program swap does not reset: it carries the state over by slot key.

Variables: a variable has a name and an expression. The name is a plain identifier with no dot, is not a function's name, and is unique without case. Variables are evaluated once per frame in dependency order, before the outputs. A cycle is a compile error.

## Face sidecar

Every recording opens `<stem>.face.jsonl` beside it when it starts, named after the recording as the audio sidecar is, so a phone that connects mid-recording is captured. One line per phone sample as the vision thread saw it: `{"seq", "t", "tracked", "w"[52]}`, with `t` the sample's receipt time on the recording's clock. A sample is written when its sequence changes, on camera and face-only iterations alike, so the sidecar runs at the phone's rate while the frame stream keeps one record per fusion iteration. `FaceRecorder` (`src/App/FaceRecorder.h`) writes it on its own thread and is finalized with the recording. A sidecar that received no sample is deleted when the recording stops.

The header carries a `face` block `{"file", "startMs"}` beside `audio`: the file name and the start time. The format version is 1. A header without the block loads as no face sidecar, and a recording whose sidecar file is absent replays with no phone face.

The timeline's replay (`TimelinePanel`) keeps its own `FaceProfileRunner` and evaluates the live binding over each record's sources: the mouth from the record's `mouth` block, the phone through `FaceSidecar::samplePhone`. That one shared function takes the newest sample at or before the record's timestamp, present while it is younger than the streaming window, with the blendshapes zero unless that sample was tracked. Playing forward evaluates frame by frame, and a seek resets the state. An edited profile can therefore be judged on a recorded take.

## Config and UI

`FaceConfig` on `AppConfig::face`, persisted under `face` in the project JSON:

- `bool enabled` (default false)
- `int port` (default 49983)
- `std::string phoneAddress` (default `255.255.255.255`)
- `glm::quat headAnchor` with `headAnchorPresent`, written as `headAnchor` `[x, y, z, w]` only once captured

The Phone stage in the Inspector, under Face and mouth, holds the enable checkbox, the port, the phone address, and a status readout: bound or not, streaming or quiet, datagrams per second, unknown-name and parse-failure counts, the last unknown name, and the last head rotation. Below it sit the head alignment button, its reset, and whether the head is aligned.

The Profile stage (`AvatarFaceTab`) edits the profiles:

- The profile bar: the assigned profiles with a missing id shown as such, the active picker, assignment from the library, new, duplicate, rename and delete. Every assigned profile, a missing id included, has its own unassign control. Auto shows read-only with "Duplicate to edit".

- The variables table: per variable its name, its function, and its value. A name error is underlined on the name field and spelled out beneath it.

- The VRM Expression Bindings table: per binding the target picker, the function, a value bar, and the unresolved-target warning with its reason. The picker lists the avatar's expressions, then each morph name once, meaning that morph on every mesh carrying it. A morph name that several meshes carry is also listed once per mesh as "name (mesh)", which narrows the binding to that mesh. Picking an expression clears the mesh.

- A compile error in a function is underlined at its span and spelled out beneath the row.

- An Edit button on every variable and binding row opens the row in the Expression Editor window. The row the editor is on draws highlighted.

The value bars and the variable values read the frame the 3D scene shows, live or the timeline's replay, and are live only for a frame of the current binding's generation, drawing empty otherwise.

The Expression Editor window (`FaceExprEditorWindow`) edits one row of the active profile at a time. It is not in the default dock layout: it floats at first use and docks or tears out like every other window. The design record is the interactive mockup at `docs/proposals/face-expr-editor-mockup.html`.

- One editor: Edit on another row moves it to that row, and so does its own header's row picker. It closes itself when its row goes away: the active profile changes, the row is deleted, or Auto becomes active.

- Edits apply live through the same path as the tab's (the last good binding keeps running while the text does not compile). Revert restores the text the editor opened with on this row, and is disabled while the text matches it.

- The header carries the row's kind, the row picker, and the live value bar. The row picker lists the profile's variables and then its bindings under a filter box, and picking a row is the Edit button on that row: the edits to the row being left stay applied, and Revert then covers the picked row. A binding's target and a variable's name are edited in the tab, not here. Under the picker the header shows the binding's unresolved-target warning or the variable's name error.

- The inputs column lists every input the language knows, enumerated from `FaceExprInputs` and grouped by namespace with the flags (`present`, `tracked`) first, then the profile's variables, each with its live value. A filter box narrows the list. The rows the compiled expression reads directly draw highlighted, and clicking a row inserts its name at the caret. The variable being edited is disabled.

- The function field is multi-line in the monospace font (M PLUS 1 Code, which carries its own Japanese glyphs) and grows with its text between two and ten lines. A compile error is underlined at its span, line by line, and spelled out beneath the field.

- The trace plots the last six seconds of the row's value (the clamped output value for a binding, the raw value for a variable) over the inputs and variables it reads directly, one sample per shown frame, so a replayed take traces the same as a live one. It is an ImPlot plot with locked axes: time in seconds before the shown frame, and a value range of 0..1 that expands to fit the shown lines. The result draws heaviest, the inputs lighter, and the variables thinnest. The legend below the plot carries each line's name and live value, toggles a line on a click, and draws it heavier on hover. The editor draws it itself as a grid that wraps at the plot's width, since an ImPlot legend is a single row or a single column. Every cell is as wide as the longest name plus a fixed value slot, so the grid does not reflow as the digits change. Hovering the plot shows a cursor at the nearest sample and a tooltip of every shown line's value there. A seek backwards or an edit that changes what the row reads starts the trace over.

- The palette's function buttons insert the signature with the first placeholder selected, so typing replaces it. A selection becomes the first argument instead. Operators and punctuation insert literally. Stateful functions draw tinted, and each function's tooltip carries its signature and a note.

- The preview is a head close-up of the avatar at rest, framed on the head bone, orbited by dragging and zoomed with the wheel, with Reset view restoring the framing. Solo on shows the rest face plus this row's own target alone, a variable reaching every binding that reads it. Solo off shows the whole profile as the 3D scene does. Beneath it the driven targets list by value. The preview draws through its own `AvatarMeshView` (a second GPU copy of the model, released when the editor closes) and only while the binding fits the loaded model and the frame is live, as the scene does.

- The `face` automation namespace drives all of this without the mouse ([automation.md](./automation.md)).

The Profile stage's Diagnostics is a filtered view of the same values: the mouth targets of the active binding, the targets whose outputs read a mouth input. Its header counts them, and its source line says what they read this frame: the phone, the visemes, or both.

## Test

`--test-face` covers the name table, both blendshape formats, bone parsing and rejection, the handshake filter, garbage input, a loopback round trip through the service, and the socket primitives. `--test-faceexpr` covers the language, the compiler's errors, every function, the time rule, the state slots across a recompile, the input sets, the profile JSON and the library. `--test-vmc` covers the head conversion, the anchor round trip, the Auto profile and the binding's combine rules, and the blendshape and avatar messages on the wire. `--test-replay` covers the face sidecar and `FaceSidecar::samplePhone`.
