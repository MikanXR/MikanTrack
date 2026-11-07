# Proposal: face expression profiles

Design decided, not yet built. This document is the record of the design: the decisions, the pieces in order, and what verifies each one.

## Goal

Today every face source is first merged into one 52-wide ARKit array (`TrackingFrameResult::face`, `src/Vision/TrackingTypes.h:354`). `VisionThread::applyFaceToOutput` (`src/App/VisionThread.cpp:1054`) chooses all-or-nothing between the phone (iFacialMocap) and the microphone. On the microphone path, `applyVisemeWeights` (`src/Audio/VisemeTable.cpp:94`) spreads the 15 visemes into ARKit mouth columns. `AvatarFaceMap` (`src/Avatar/AvatarFaceMap.*`) then maps that array to VMC blend names with fixed linear terms. The preset vowels special-case the raw visemes.

The goal is a user-authored mapping, in the spirit of VBridger:
- Each output is an expression over raw source namespaces (`phone.*`, `viseme.*`, a future `webcam.*`).
- An output targets a VRM expression or a morph target directly.
- Expressions can use stateful functions (smoothing, hysteresis).
- Expressions live in named profiles, stored in a global library and assigned per avatar.
- The UI shows how each input reaches each final morph weight.

The collapsed ARKit array and the viseme-to-ARKit detour go away. Recordings gain the raw phone face so an edited profile can be re-run on a replayed take.

Settled decisions:
- Expressions read raw namespaces. There is no collapsed array.
- Stateful functions are in v1.
- Profiles are named, kept in a global library, and assigned per avatar. One avatar can have several presets, and one profile can serve several avatars.
- Outputs can target morphs directly. Several writes to the same morph combine with max, and the UI shows which output won.
- Recording format v2 stores the raw phone face.

## Design

### 1. Sources: `FaceSources` replaces the merged `FacePose`

`TrackingFrameResult::face` becomes `FaceSources`:
- `phone`:
	- `present`: the stream is live
	- `tracked`: the phone sees a face
	- `blendshapes[52]`: raw, in ARKit order
- `viseme`:
	- `present`: the sample is no more than `k_mouthStaleMs` old
	- `weights[15]`: post-`MouthFilter`
- `outputs`: the evaluated profile values for this frame (section 4)

`applyFaceToOutput` and `applyMouthToOutput` fill both sources side by side and no longer decide which one owns the mouth. That choice moves into the expressions as `select(phone.tracked, ..., ...)`. Phone eyes with a microphone mouth then becomes possible.

`runFaceOnlyOutput` sends an extra frame whenever either source has a new sample. The "only while the phone does not own the mouth" gate goes away.

### 2. The expression language and IR (new `src/FaceExpr/`)

The parser is hand-written: a Pratt parser that compiles to a flat register IR. No dependency is added. Owning the IR gives the UI each output's input list, and leaves room for a graph editor later that bakes to the same IR.

**Identifiers:**
- `phone.<arkitName>`: case-insensitive, through `arkitBlendshapeFromName` (`src/Face/`)
- `phone.present`, `phone.tracked`
- `viseme.<name>`: through `visemeName` (`src/Audio/VisemeTypes.h`)
- `viseme.present`
- The names of the profile's `let` values
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

**Profile-level `let`:** named intermediates, evaluated once in dependency order. Cycles are compile errors.

**Pieces:**
- `FaceExprProgram`: an immutable compiled program. It holds instructions, constants, input-slot bindings, the number of state slots, and the input set of each output and each `let`.
- `FaceExprCompiler`:
	- Turns text into a program, or into errors carrying the output index and character span.
	- It is all-or-nothing per profile. The last good program stays live while an edit is invalid.
- `FaceExprEvaluator`: `evaluate(const FaceSources&, double timeSeconds, FaceExprState&, float* outValues)`. On a time reset it reinitializes the stateful slots.

### 3. Profiles, the library and assignment

**`FaceProfile`:**
- `{version, name, lets[{name, expr}], outputs[{targetKind: expression|morph, target, mesh?, expr}]}`
- `mesh` is optional. When it is missing, a morph target means every mesh's morph with that name, which matches today's behavior.
- JSON goes through nlohmann, in the `j.value(key, default)` style of `AppConfig.cpp`.

**`FaceProfileLibrary`:**
- One JSON file per profile in `Documents/MikanTrack/FaceProfiles/`, beside the existing recordings root.
- Supports list, load, save, rename and duplicate.
- Saves through the same settle-then-save pattern as `App::updateAvatarRigAutoSave` (`src/App/App.cpp:380-416`).

**Assignment:**
- The rig sidecar `<stem>.mikanrig.json` (`src/Avatar/AvatarRig.*`, version bump) gains `faceProfiles: [names]` and `activeFaceProfile`.
- The assignment follows the VRM file across projects, the way the rig already does.

**Built-in "Auto" profile:**
- Generated per avatar from today's rules: perfect-sync, the presets in `makePresetRules`, and ARKit-named morph targets.
- The viseme spreading in `VisemeTable` `k_rows` becomes ordinary expressions, `select(phone.tracked, phone.jawOpen, <viseme terms>)`, so the default output stays close to today's.
- Auto is read-only. "Duplicate to edit" copies it into the library.
- An avatar with no assignment uses Auto, so existing setups behave the same with no migration.

**Resolution against an avatar:**
- A target missing from the avatar is reported in the UI and skipped, never fatal. This keeps one profile portable across similar faces.

### 4. One evaluation site and one binding

**`FaceProfileBinding`**, which replaces `AvatarFaceMap` and `AvatarFaceMorphs`:
- It is built from (profile, avatar) and holds:
	- the compiled program
	- the resolved targets
	- the morph binds
- An expression target expands to its `morphBinds` times the bind weight. A morph target becomes a direct bind at weight 1.
- Morph weights combine with max, as `AvatarFaceMorphs::evaluate` does today.
- For each morph it records which output won, for the UI.

**Live evaluation** happens once per output frame on the vision thread, in `runOutputStage`, before the OSC rate gate. The stateful slots therefore advance exactly once per frame. Results go into `frame.face.outputs`.
- `OscStreamer::appendVmcBlendMessages` (`src/Osc/OscStreamer.cpp:299`) sends `target name -> value` from `outputs`. A morph-target output goes on the wire under its morph name, as ARKit morph outputs already do. The relaxing zero frame is kept.
- `Scene3dPanel` and the readout read the `outputs` carried on the frame and resolve morph weights through the shared binding. Their private `AvatarFaceMap` copies go away.

**Hot-swap:**
- A profile edit, a profile switch or an avatar change builds a new `shared_ptr<const FaceProfileBinding>`.
- It is handed over through the existing `VisionThread::setAvatarIdentity`/`servicePendingRequests` slot (`VisionThread.cpp:678-726`) and the `OscStreamerConfig` copy.
- The evaluator state resets when the program's identity changes.

**Timeline replay:** `TimelinePanel` keeps its own evaluator and state. The state resets on seek, and replay rebuilds `FaceSources` from the recorded blocks. That replaces the fake face built from visemes at `TimelinePanel.cpp:517-529`.

### 5. Recording format v2 (`src/App/TrackingRecording.*`)

- `k_formatVersion` becomes 2.
- New per-record `phoneFace` block: `{tracked, seq, t, w[52]}`, next to the existing `mouth` block.
- Face-only output frames, from a new phone or viseme sample with no camera iteration, are also recorded. They are marked face-only so hand and body replay skips them.
- v1 files still load with no phone face.

### 6. UI: a "Face" tab in `AvatarPanel`

The tab sits in the existing tab bar (`src/UI/AvatarPanel.cpp:82-92`). It holds:
- **Profile bar:**
	- assigned profiles
	- the active profile picker
	- assign/unassign from the library
	- new, duplicate, rename
	- Auto shown read-only
- **Lets and outputs table**, one row per output:
	- the target picker (the avatar's expressions and morphs, grouped by mesh)
	- the expression `InputText`, with an inline red error and the span marked under it
	- a live value bar
	- an unresolved-target warning
- **Inspector for the selected output:**
	- each input it reads, with live values, taken from the program's input set
	- the `let` chain it depends on
	- the morphs it drives, with bind weights
- **Morph view:**
	- per mesh, each morph's final weight
	- every output contributing to it, with the max winner highlighted
	- This is the "how every input maps to the final weight" view.
- Weight bars reuse `drawWeightTable` (`src/UI/WeightBars.cpp`).
- `AvatarMouthReadout` becomes a filtered view of the same data, or is removed in favor of the tab.
- Strings go through `locLabel` with `en.json` and `ja.json` entries (`--loc-test`).

### 7. Removals

- The collapsed-array writes in `applyVisemeWeights`.
- `AvatarFaceMap::Output::viseme` and the vowel special case.
- `buildRaw`, `build`, and `AvatarFaceMorphs`, replaced by the binding plus the Auto generator.
- `VisemeTable` `k_rows` stays only as input data for the Auto generator. `ToolRenderMouth` moves to evaluating the Auto profile. `ToolVrmInfo` prints the Auto profile's targets.

## Critical files

**New:**
- `src/FaceExpr/`: Lexer and Parser, `FaceExprProgram`, `FaceExprCompiler`, `FaceExprEvaluator`, `FaceProfile`, `FaceProfileLibrary`, `FaceProfileBinding`, `FaceAutoProfile`
- `src/Tests/TestFaceExpr.cpp`
- The `CMakeLists.txt` glob and `source_group` for the new directory

**Modified:**
- `src/Vision/TrackingTypes.h`
- `src/App/VisionThread.{h,cpp}`
- `src/Osc/OscStreamer.{h,cpp}`
- `src/App/App.cpp` (`loadAvatar` and the binding builds)
- `src/Avatar/AvatarRig.*`
- `src/App/TrackingRecording.*`
- `src/UI/TimelinePanel.cpp`
- `src/UI/Scene3dPanel.cpp`
- `src/UI/AvatarPanel.*`
- `src/UI/AvatarMouthReadout.*`
- `src/Audio/VisemeTable.*`
- The tools that use `AvatarFaceMap`

**Removed:** `src/Avatar/AvatarFaceMap.*`

**Docs:**
- `docs/reference/face.md`: sources, profiles, the language reference
- `audio.md`: visemes no longer feed ARKit
- `avatar.md`: binding and morph view
- `wire-protocol.md`: the blend names now come from the profile
- `debugging.md` or `testing.md`: the new test
- `docs/plan.md`: deferred items (the graph editor, the `webcam.*` source, Lua-defined inputs, curve-preview widgets)

## Implementation order

1. [ ] `FaceExpr` core (parser, IR, evaluator, stateful slots), plus `--test-faceexpr`
2. [ ] `FaceSources` split in the vision thread, the binding, the Auto generator, OSC and Scene3d switched over, `AvatarFaceMap` removed
3. [ ] Profile library, the rig sidecar assignment, hot-swap
4. [ ] Recording v2 and timeline replay through the evaluator
5. [ ] Face tab UI (profile bar, outputs, inspector, morph view)
6. [ ] Docs pass

## Verification

**`--test-faceexpr`, new.** It covers:
- precedence and associativity
- every function
- `let` ordering, plus cycle and unknown-identifier errors with spans
- `smooth` converging at the given tau under varying dt
- `hysteresis` latching
- separate state per call site
- state reset on a time reset
- input-set extraction
- a profile JSON round trip

**`--test-vmc` sections (j) and (k), rewritten for the binding:**
- The Auto profile on the synthetic preset avatar sends the same names as today and the same values within tolerance.
- The perfect-sync Auto profile is checked the same way.
- A direct morph target output reaches its morph.
- Max combine reports the winning output.
- An unresolved target is skipped.
- The relaxing zero frame and the raw-names case with no avatar still hold.

**`--test-replay`:** the v2 `phoneFace` block round-trips float-exact, and a v1 file loads with no phone face.

**`--test-audio`:** the viseme table checks move to the Auto generator's expressions.

**Batch run:** every self-test, using the `--list-tests` loop in `docs/reference/testing.md`.

**Live check:**
- Run `build/RelWithDebInfo/MikanTrack.exe` with Seed-san.vrm.
- Microphone only: the Auto profile drives the vowels.
- Phone streaming: the phone drives the face.
- Edit an output, for example `smooth(viseme.aa, 0.05)`, and watch the value bar, the morph view and the VMC values. Confirm the values in VNyan, or with `send-vmc`/`ue_automate` reading the curves on the UE side.
- Record a phone take, edit the profile, and replay it in the timeline.
