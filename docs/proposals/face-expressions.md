# Proposal: face expression profiles

Built on the `face-expression` branch. The live check is still owed (plan.md). This document is the record of the design: the decisions, the pieces in order, and what verifies each one. The reference is [face.md](../reference/face.md).

## Goal

Today every face source is first merged into one 52-wide ARKit array (`TrackingFrameResult::face`, `src/Vision/TrackingTypes.h:354`). `VisionThread::applyFaceToOutput` (`src/App/VisionThread.cpp:1054`) chooses all-or-nothing between the phone (iFacialMocap) and the microphone. On the microphone path, `applyVisemeWeights` (`src/Audio/VisemeTable.cpp:94`) spreads the 15 visemes into ARKit mouth columns. `AvatarFaceMap` (`src/Avatar/AvatarFaceMap.*`) then maps that array to VMC blend names with fixed linear terms. The preset vowels special-case the raw visemes.

The goal is a user-authored mapping, in the spirit of VBridger:
- Each output is an expression over raw source namespaces (`phone.*`, `viseme.*`, a future `webcam.*`).
- An output targets a VRM expression or a morph target directly.
- Expressions can use stateful functions (smoothing, hysteresis).
- Expressions live in named profiles, stored in a global library and assigned per avatar.
- The UI shows each binding's live value.

The collapsed ARKit array and the viseme-to-ARKit detour go away. Recordings gain the raw phone face so an edited profile can be re-run on a replayed take.

Settled decisions:
- Expressions read raw namespaces. There is no collapsed array.
- A source's values are zero unless it is tracked: `phone.*` blendshapes read zero while the stream is quiet or the phone reports the face lost, and `viseme.*` weights read zero past the stale window. An expression without a guard therefore relaxes instead of freezing on the last raw values.
- Stateful functions are in v1.
- Profiles carry a stable id and a display name, are kept in a global library, and are assigned per avatar by id. One avatar can have several profiles, and one profile can serve several avatars.
- Outputs can target expressions or morphs directly. Several outputs on one target combine with max, before the wire and before the morph binds.
- The raw phone face is recorded as a sidecar stream beside the recording, at the phone's own rate, like the audio sidecar. The frame stream keeps its invariant that one record is one fusion iteration.
- The tab holds the variables and the VRM Expression Bindings with live value bars. An inspector and a per-morph weight view were built and removed, since the per-binding value is what a VRM author reasons about.

## Design

### 1. Sources: `FaceSources` replaces the merged `FacePose`

`TrackingFrameResult::face` becomes `FaceSources`:
- `phone`:
	- `present`: the stream is live (`FaceService::isStreaming`)
	- `tracked`: `present` and the phone sees a face
	- `blendshapes[52]`: raw, in ARKit order, all zero unless `tracked`
- `viseme`:
	- `present`: the sample is no more than `k_mouthStaleMs` old
	- `weights[15]`: post-`MouthFilter`, all zero unless `present`
- `outputs`: the evaluated profile values for this frame (section 4), with the generation of the binding that produced them

`applyFaceToOutput` and `applyMouthToOutput` fill both sources side by side and no longer decide which one owns the mouth. That choice moves into the expressions as `select(phone.tracked, ..., ...)`. Phone eyes with a microphone mouth then becomes possible. `applyFaceToOutput` keeps its head duties unchanged: the phone's head rotation replaces the camera head while the stream is live and tracked, and the anchor capture still runs there on a camera frame.

`runFaceOnlyOutput` sends an extra frame whenever either source has a new sample. The "only while the phone does not own the mouth" gate goes away.

Presence for the wire and the preview is the union: the face is present while either source is. The relaxing zero frame goes out once when both have gone away.

### 2. The expression language and IR (new `src/FaceExpr/`)

The parser is hand-written: a Pratt parser that compiles to a flat register IR. No dependency is added. Owning the IR gives the UI each output's input list, and leaves room for a graph editor later that bakes to the same IR.

**Identifiers**, all compared without case, including variable names:
- `phone.<arkitName>`: through `arkitBlendshapeFromName` (`src/Face/`)
- `phone.present`, `phone.tracked`
- `viseme.<name>`: through `visemeName` (`src/Audio/VisemeTypes.h`)
- `viseme.present`
- The names of the profile's variables
- Unknown names are compile errors. The namespace table is the single extension point for `webcam.*`.

**Operators:**
- `+ - * /`, unary `-`
- Comparisons
- `&& || !`, where booleans are 0 and 1
- Parentheses

**Pure functions:**
- `min`, `max`, `clamp`, `abs`, `pow`
- `lerp`
- `remap(x, inLo, inHi[, outLo, outHi])`
- `smoothstep`
- `select(c, a, b)`

**Stateful functions:** each call site gets its own state slot in a `FaceExprState` that is separate from the program, so one compiled program can drive several evaluators.
- `smooth(x, tauSeconds)`: exponential smoothing, using frame dt
- `hysteresis(x, on, off)`

**Time.** The evaluator takes the frame's own timestamp. Face-only frames carry their sample's timestamp, so interleaved camera and face frames can arrive a few milliseconds out of order. The dt a stateful function sees is the clamp of the timestamp step to zero and a cap of one second. The state is reinitialized only on an explicit reset (a seek, a program swap) or a backwards jump of more than the cap, never on the small regressions of the interleave.

**State slots across edits.** A slot is keyed by the output or variable it belongs to and the call-site ordinal inside that expression. Recompiling after an edit carries over the slots whose key still exists, so editing one output does not restart the smoothing of every other.

**Profile variables:** named intermediates. A variable's name is a plain identifier with no dot, not a function's name, and unique without case. Variables are evaluated once per frame in dependency order, before the outputs. Cycles are compile errors.

**Pieces:**
- `FaceExprProgram`: an immutable compiled program. It holds instructions, constants, input-slot bindings, the state slot table with its keys, and the input set of each output and each variable.
- `FaceExprCompiler`:
	- Turns text into a program, or into errors carrying the variable or output, an error code with its arguments, and where the error points: the variable's name field for a variable-name error, otherwise the character span in the expression. A cycle error's span is the reference that closes the cycle.
	- The Face tab formats the code and its arguments as localized text (en and ja), for example "variable 'x' is defined twice", "'x' is not a valid variable name", "variable 'x' has an error".
	- It is all-or-nothing per profile. The last good program stays live while an edit is invalid.
- `FaceExprEvaluator`: `evaluate(const FaceSources&, double timeSeconds, FaceExprState&, float* outValues)`, applying the time rule above.

### 3. Profiles, the library and assignment

**`FaceProfile`:**
- `{version, id, name, variables[{name, expr}], outputs[{targetKind: expression|morph, target, mesh?, vrmExpressionFunction}]}`
- A file that carries the variables under the key `lets` loads the same, as does a binding that carries its function under the key `expr`.
- `id` is a UUID string minted when the profile is created or duplicated and never changed. `name` is the display name and is free to change.
- `mesh` is optional. When it is missing, a morph target means every mesh's morph with that name, which matches today's behavior.
- JSON goes through nlohmann, in the `j.value(key, default)` style of `AppConfig.cpp`.

**`FaceProfileLibrary`:**
- One JSON file per profile, `<id>.json`, in `Documents/MikanTrack/FaceProfiles/`, beside the project folders under the same root. The project list is not a scan of that root, so the folder only ever costs a new project named `FaceProfiles` its suffix.
- Supports list, load, save, rename and duplicate. Rename rewrites `name` inside the file and nothing else.
- Saves through the same settle-then-save pattern as `App::updateAvatarRigAutoSave` (`src/App/App.cpp:380-416`).

**Assignment:**
- The rig sidecar `<stem>.mikanrig.json` (`src/Avatar/AvatarRig.*`, version bump) gains `faceProfiles: [ids]` and `activeFaceProfile`.
- The assignment follows the VRM file across projects, the way the rig already does.
- An assigned id the library does not hold (a sidecar that traveled to another machine, a deleted profile) is listed as missing in the Face tab, and the avatar runs Auto while that id is active.

**Built-in "Auto" profile:**
- Generated per avatar from today's rules: perfect-sync, the presets in `makePresetRules`, and ARKit-named morph targets.
- The viseme spreading in `VisemeTable` `k_rows` becomes ordinary expressions, `select(phone.tracked, phone.jawOpen, <viseme terms>)`, so the default output stays close to today's.
- With no avatar loaded, Auto is the 52 ARKit names as morph targets, each `select(phone.tracked, phone.<name>, <viseme terms>)`, which is today's raw map with the microphone through the viseme rows.
- Auto is read-only. "Duplicate to edit" copies it into the library under a new id.
- An avatar with no assignment uses Auto, so existing setups behave the same with no migration.

**Resolution against an avatar:**
- A target missing from the avatar is reported in the UI and skipped, never fatal. This keeps one profile portable across similar faces.

### 4. One evaluation site and one binding

**`FaceProfileBinding`**, which replaces `AvatarFaceMap` and `AvatarFaceMorphs`:
- It is built from (profile, avatar) and holds:
	- the compiled program
	- the resolved targets, with the outputs grouped per target
	- the morph binds
	- a generation counter, so a frame's `outputs` can be matched to the binding that produced them
- An output's value is clamped to 0..1. Outputs sharing a target combine with max into one target value. An expression target expands to its `morphBinds` times the bind weight. A morph target becomes a direct bind at weight 1.
- Morph weights combine with max across targets, as `AvatarFaceMorphs::evaluate` does today.
- The binding is immutable once built. Which output won a target and which target won a morph are per-frame facts, computed by whoever evaluates (`combineTargets` and `evaluateMorphs` report them on request), never stored on the binding.
- The preview ignores `isBinary` on an expression, as `AvatarFaceMorphs` does today. A receiver applies it.

**Live evaluation** happens once per output frame on the vision thread, in `runOutputStage`, before the OSC rate gate. The stateful slots therefore advance exactly once per frame. Results go into `frame.face.outputs` with the binding's generation.
- `OscStreamer::appendVmcBlendMessages` (`src/Osc/OscStreamer.cpp:299`) sends one `target name -> value` per target from the combined values, so a target never appears twice in a bundle. A morph-target output goes on the wire under its morph name, as ARKit morph outputs already do. The relaxing zero frame is kept, keyed on the union presence of section 1.
- `Scene3dPanel` and the readout read the `outputs` carried on the frame and resolve morph weights through the binding of the matching generation. A frame from an older generation shows the rest face until the next frame arrives. Their private `AvatarFaceMap` copies go away.

**Hot-swap:**
- A profile edit, a profile switch or an avatar change builds a new `shared_ptr<const FaceProfileBinding>` with the next generation.
- It is handed over through the existing `VisionThread::setAvatarIdentity`/`servicePendingRequests` slot (`VisionThread.cpp:678-726`) and the `OscStreamerConfig` copy.
- The evaluator state carries over by slot key (section 2) and resets for the slots whose key is new.

**Timeline replay:** `TimelinePanel` keeps its own evaluator and state. The state resets on seek, and replay rebuilds `FaceSources` for each record: the mouth from the record's `mouth` block, the phone through one shared function, `FaceSidecar::samplePhone`: the face sidecar's newest sample at or before the record's timestamp, present while that sample is younger than the streaming window, with the blendshapes zero unless that sample was tracked. That replaces the fake face built from visemes at `TimelinePanel.cpp:517-529`.

### 5. The face sidecar (`src/App/TrackingRecording.*`)

- Every recording opens `<stem>.face.jsonl` beside it when it starts, named after the recording the way the audio sidecar is, so a phone that connects mid-recording is captured. A sidecar that received no sample is deleted when the recording stops. One line per phone sample as the vision thread saw it: `{seq, t, tracked, w[52]}`, with `t` on the recording's clock. A sample is written when its sequence changes, on camera iterations and face-only iterations alike, so the sidecar runs at the phone's rate without putting face-only records in the frame stream.
- The header gains a `face` block `{file, startMs}` beside `audio`, the file name and the start time. The format version stays at 1, as the audio sidecar left it: a header without the block loads as no face sidecar, so v1 files replay with no phone face, and a recording whose sidecar file is absent replays the same way.
- The sidecar is written by its own writer thread, like `AudioRecorder`, and finalized with the recording.
- The per-record `mouth` block is unchanged.

### 6. UI: a "Face" tab in `AvatarPanel`

The tab sits in the existing tab bar (`src/UI/AvatarPanel.cpp:82-92`). It holds:
- **Profile bar:**
	- assigned profiles, a missing id shown as such
	- the active profile picker
	- assign/unassign from the library
	- new, duplicate, rename
	- Auto shown read-only
- **Variables table**, one row per variable:
	- the name, with a variable-name error underlined on it
	- the function `InputText`, any other compile error underlined at its span and spelled out beneath the row
	- the live value
- **VRM Expression Bindings table**, one row per binding:
	- the target picker: the avatar's expressions, then each morph name once (that morph on every mesh carrying it), and for a morph name several meshes carry, one "name (mesh)" entry per mesh that narrows the binding to that mesh, while picking an expression clears the mesh
	- the function `InputText`, a compile error underlined at its span and spelled out beneath the row
	- a live value bar
	- an unresolved-target warning with its reason
- The value bars are live only for a frame of the current binding's generation. Otherwise they draw empty.
- Every assigned profile, a missing id included, has its own unassign control.
- `AvatarMouthReadout` becomes a filtered view of the same data, or is removed in favor of the tab.
- Strings go through `locLabel` with `en.json` and `ja.json` entries (`--loc-test`).
- The per-row Expression Editor window that followed is designed in the interactive mockup `face-expr-editor-mockup.html` beside this file.

### 7. Removals

- The collapsed-array writes in `applyVisemeWeights`.
- `AvatarFaceMap::Output::viseme` and the vowel special case.
- `buildRaw`, `build`, and `AvatarFaceMorphs`, replaced by the binding plus the Auto generator.
- `VisemeTable` `k_rows` stays only as input data for the Auto generator. `ToolRenderMouth` moves to evaluating the Auto profile. `ToolVrmInfo` prints the Auto profile's targets.

## Critical files

**New:**
- `src/FaceExpr/`: Lexer and Parser, `FaceExprProgram`, `FaceExprCompiler`, `FaceExprEvaluator`, `FaceProfile`, `FaceProfileLibrary`, `FaceProfileBinding`, `FaceAutoProfile`
- `src/App/FaceRecorder.*`: the face sidecar writer and reader
- `src/Tests/TestFaceExpr.cpp`
- The `CMakeLists.txt` glob and `source_group` for the new directory

**Modified:**
- `src/Vision/TrackingTypes.h`
- `src/App/VisionThread.{h,cpp}`
- `src/Osc/OscStreamer.{h,cpp}`
- `src/App/App.cpp` (`loadAvatar` and the binding builds)
- `src/Avatar/AvatarRig.*`
- `src/App/TrackingRecording.*`
- `src/App/TrackingReplay.*` (loading the face sidecar)
- `src/UI/TimelinePanel.cpp`
- `src/UI/Scene3dPanel.cpp`
- `src/UI/AvatarPanel.*`
- `src/UI/AvatarMouthReadout.*`
- `src/Audio/VisemeTable.*`
- The tools that use `AvatarFaceMap`

**Removed:** `src/Avatar/AvatarFaceMap.*`

**Docs:**
- `docs/reference/face.md`: sources, profiles, the language reference, the sidecar
- `audio.md`: visemes no longer feed ARKit
- `avatar.md`: the binding and the scene preview
- `wire-protocol.md`: the blend names now come from the profile, one message per target
- `debugging.md` or `testing.md`: the new test and the sidecar
- `docs/plan.md`: remove the two Later items this subsumes (face samples in tracking recordings, the VBridger-style expression editor), add the deferred items (the graph editor, the `webcam.*` source, Lua-defined inputs, curve-preview widgets), and add the machine-translated Japanese review item for the Face tab's strings

## Implementation order

1. [x] `FaceExpr` core (parser, IR, evaluator, the time rule, keyed state slots), plus `--test-faceexpr`
2. [x] `FaceSources` split in the vision thread, the binding, the Auto generator, OSC and Scene3d switched over, `AvatarFaceMap` removed
3. [x] Profile library, the rig sidecar assignment by id, hot-swap
4. [x] The face sidecar and timeline replay through the evaluator
5. [x] Face tab UI (profile bar, variables, VRM Expression Bindings)
6. [x] Docs pass

## Verification

**`--test-faceexpr`, new.** It covers:
- precedence and associativity
- every function
- identifier matching without case in every namespace
- variable ordering, plus cycle and unknown-identifier errors with spans
- `smooth` converging at the given tau under varying dt
- the time rule: a small backwards step advances nothing, a jump past the cap reinitializes, an explicit reset reinitializes
- `hysteresis` latching
- separate state per call site
- state carried across a recompile for an unchanged output and reset for an edited one
- input-set extraction
- a profile JSON round trip keeping the id

**`--test-vmc` sections (j) and (k), rewritten for the binding:**
- The Auto profile on the synthetic preset avatar sends the same names as today and the same values within tolerance.
- The perfect-sync Auto profile is checked the same way.
- A direct morph target output reaches its morph.
- Two outputs on one target send one message carrying the max, and the winner is reported.
- Max combine across targets reports the winning output per morph.
- An unresolved target is skipped.
- The relaxing zero frame on the union presence, and the no-avatar Auto sending the 52 names, still hold.
- A source that is present but not tracked reads zero.

**`--test-replay`:** the face sidecar round-trips float-exact and is sampled by time onto the records, a recording without one loads with no phone face, and a v1 header loads unchanged.

**`--test-avatar-rig`:** the sidecar round-trips the profile ids and the active id.

**`--test-audio`:** the viseme table checks move to the Auto generator's expressions.

**Batch run:** every self-test, using the `--list-tests` loop in `docs/reference/testing.md`.

**Live check:**
- Run `build/RelWithDebInfo/MikanTrack.exe` with Seed-san.vrm.
- Microphone only: the Auto profile drives the vowels.
- Phone streaming: the phone drives the face.
- Both at once, with a binding function of `select(phone.tracked, phone.jawOpen, viseme.aa)` edited to `max(phone.jawOpen, viseme.aa)`: the phone's eyes and brows hold while the voice opens the mouth.
- Edit a binding's function, for example to `smooth(viseme.aa, 0.05)`, and watch the value bar and the VMC values. Confirm the values in VNyan, or with `send-vmc`/`ue_automate` reading the curves on the UE side.
- Rename a profile and confirm the avatar keeps it. Delete one that is assigned and confirm the tab lists it as missing and the avatar runs Auto.
- Record a phone take, edit the profile, and replay it in the timeline.
