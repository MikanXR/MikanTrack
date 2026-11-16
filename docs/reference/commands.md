# Command Cheat Sheet

Handy commands for working in the MikanTrack repo, all run from the repo root unless noted. Background on the build system, dependency versions, and the model set is in [build.md](./build.md). How to read test/log output and the replay tooling workflows are in [debugging.md](./debugging.md), and what each test asserts, plus the command that runs the whole self-test batch, is in [testing.md](./testing.md).

---

## One-time setup

```
InitialSetup_x64.bat
```

Populates git submodules, wipes and re-downloads `deps/` (roughly 1.3 GB), and downloads the ONNX models into `models/`. See [build.md](./build.md) for exactly what it fetches.

---

## Generate project files

```
GenerateProjectFiles_X64_VS2026.bat
GenerateProjectFiles_X64_VS2022.bat
```

Configures `build/` with the `Visual Studio 18 2026` generator (producing `build/MikanTrack.slnx`) or the `Visual Studio 17 2022` generator (producing `build/MikanTrack.sln`). A `build/` tree is tied to the generator that first configured it, so switching versions means deleting `build/` first. Rerun after `InitialSetup_x64.bat`, and rerun any time a `.cpp` file is added to an existing `src/<Dir>` (the source list is a configure-time glob).

---

## Build

```
cmake --build build --target MikanTrack --config Release --parallel
```

Or open `build\MikanTrack.sln` in Visual Studio 2022 and build there. There is only the one target, `MikanTrack`.

The build the generation scripts configure carries the Tracy profiler client. To profile, start the viewer and run the app:

```
RunProfiler.bat
```

It starts `deps\tracy\tracy-profiler.exe` connected to this machine on port 8286, fetching the viewer first where `deps\` has none, and the viewer waits for the app. `deps\tracy\tracy-capture.exe -p 8286 -o <file.tracy>` records a run without the viewer, and `tracy-csvexport.exe <file.tracy>` lists its zones.

To let an AI assistant query captures, start Tracy's MCP server:

```
RunTracyMcp.bat
```

It serves `http://127.0.0.1:47380/mcp` from its own window, building its venv and Tracy's Python server bindings into `deps\tracy-mcp` on first use. Claude Code registers it once with `claude mcp add --transport http --scope user tracy http://127.0.0.1:47380/mcp`.

---

## Automation server

```
cd build/Release && ./MikanTrack.exe --automation-server
python tools/automate.py "app info" "screenshot window"
```

`--automation-server` opens the loopback command channel for that launch (off by default), and `--automation-port=<n>` picks a port other than 21140 and implies it. The client script sends each argument as one command and prints the replies. The namespaces and the drive loop are in [automation.md](./automation.md).

---

## Run the app and its self-tests

The app exe doubles as the test runner: every self-test, hardware check, and headless tool is a flag handled before `App::exec` ever starts, and `MikanTrack.exe` must be run with its own directory as the working directory (see [build.md](./build.md)).

```
MikanTrack.exe --list-tests
```

Prints every registered command grouped by category (self-tests, hardware required, tools). Each command logs to `<flag-without-dashes>.log` next to the exe in addition to console output (`--test-fusion` writes `test-fusion.log`), and returns exit code 0 on pass.

### Self-tests (deterministic, no hardware or input files needed)

- `--loc-test`: localization tables: key parity, printf specifiers, window IDs, glyph coverage
- `--selftest`: OSC writer packet round-trip
- `--test-angleprior`: finger-angle prior fitting (mean, correlations, floors)
- `--test-audio`: audio stage: the mouth columns and the viseme frame, window policy, wav reader, filter, the MFCC chain against a uLipSync reference, the profile JSON round trip, the viseme table, the backend, the loudness backend and the model names, the level gate, the audio config round trip, the wav writer, the sidecar recorder, the pacer, the replay alignment and the shared chain with a backend that runs while gated, the calibration frames and the gate suggestion, and the Oculus LipSync backend in a build carrying its SDK (skipped when `OVRLipSync.dll` is absent)
- `--test-automation`: automation server: the command tokenizer and reply framing, and a loopback session on an ephemeral port
- `--test-bodycalib`: body dimension calibration recovers known landmark separations
- `--test-bodypose`: body-pose solver: rays + known lengths, root choice, hold, gates
- `--test-bonecalib`: hand bone calibration from triangulated landmarks
- `--test-charuco`: Charuco board finder + synthetic intrinsics recovery
- `--test-deskhand`: desk-plane hand calibration: bones from one camera, flatness gates, lift depth scale
- `--test-dump`: diagnostic dump writer + schema
- `--test-extrinsics`: Aruco marker pose + table-plane raycast
- `--test-framerecorder`: raw frame recording: round trip, naming, overflow drops
- `--test-face`: iFacialMocap receiver: datagram parser, receive filter, loopback socket
- `--test-faceexpr`: face expressions, the webcam face source and the Blink stage: precedence, functions, names, variables, smooth, the time rule, hysteresis, state slots across a recompile, input sets, profile JSON, the library
- `--test-fusion`: multi-camera fusion, clustering, triangulation, holds
- `--test-handestimator`: angle-space multi-view hand state estimator
- `--test-handpose`: palm frame, finger angles, FK round-trip, conventions
- `--test-imudiscovery`: IMU device discovery never blocks the caller
- `--test-imufilter`: IMU orientation EKF, observability, gating
- `--test-pnp`: monocular PnP recovers a synthetic hand pose
- `--test-replay`: record/replay determinism, checksums, what-if, the audio and face sidecars
- `--test-roiquality`: hand ROI image-quality metrics + flicker tracker
- `--test-scene-view`: camera-matched scene view and overlay projection
- `--test-seeding`: the seed redundancy gate, the body model's hand hints, the eviction rule and the hand side by shape
- `--test-vmc`: VMC retarget: axis conversion, rest identity, chain round trip, the face profile binding and blend messages
- `--test-avatar-retarget`: avatar retarget: rest identity, reach scaling, two-bone IK, hands, fingers, root, VMC
- `--test-avatar-rig`: avatar rig: sidecar round trip, mapping overrides, trims, finger tweaks, elbow hint
- `--test-direct-retarget`: direct retarget: camera world, neutral capture, the depth planes, unseen elbow, hold and rest, determinism, the body-wrist gate on hand slots
- `--test-retarget-golden`: retarget golden: the live position retarget path against its frozen copy, bit for bit, over synthetic sequences, and the direct source pose's clock and group states on the live solver alone
- `--test-setup-flow`: guided setup: the step chain of every tracking variant
- `--test-stage-view`: Stage View: the content of every view mode and the mode names
- `--test-pipeline-model`: pipeline model: layout per project shape, stage ids, status rules, string table coverage
- `--test-vrm`: VRM loader and rest skeleton on synthetic in-memory files, both VRM generations
- `--test-webcam-table`: webcam table: USB ids, table lookup and scaling, the nominal pinhole, the intrinsics source

### Hardware (need a camera or controller physically connected to mean anything)

- `--audio-probe [seconds] [--device name] [--gate dbfs] [--gain db]`: live microphone mouth stage: devices, level, gate, the backend's info line, frames per second with their dominant visemes, and the calibration wizard's capture tap
- `--test-imuaxes`: live Joy-Con axis convention measurement
- `--test-joycon`: Joy-Con sample decode + live HID streaming
- `--test-posemodel`: body pose ONNX models load + match the reference decode
- `--test-vrm-samples`: the shipped VRoid sample avatars load with full humanoid maps (needs `models/avatars/`)

### Tools (headless diagnostics that operate on a file the user names)

- `--calibrate-bones`: measure hand bone lengths from a recording's stereo frames
- `--export-board`: writes + opens the printable Charuco board PNG
- `--export-marker`: writes + opens the printable origin Aruco marker PNG
- `--fit-angle-prior`: fit the finger-angle prior from recordings' stereo frames
- `--gate-wholebody`: the whole-body model's face and hand blocks on one camera's raw frames, at every frame and at the live divider: face point jitter, the face shape ratios and blinks against the phone's face sidecar, the blink evaluation's face model candidates found in `models/`, the body model's hands and the hand slot's side against the stereo rig's triangulated hands, and the hand slots the direct solver turns away on the other arm's wrist, split by what the hands were doing and what the slot was (`labels` runs that report alone)
- `--replay-bodypose <recording.jsonl> [cameraIndex] [rtmpose|rtmw3d] [divider=<n>]`: re-run body pose over a recording's raw frames, on the recorded backend or a named one. `divider` runs the models every nth frame as the live stage does, and the report counts the model frames the person was lost on and the longest stage time
- `--replay-hands <recording.jsonl> [cameraIndex] [out.csv]`: re-run the hand landmark model over a recording's raw frames from the recorded ROIs, reporting estimate time and the landmark difference against the recording
- `--replay-dump`: emits a recording frame range with regenerated fusion diagnostics
- `--replay-extrinsics`: A/B a candidate extrinsics config against a recording
- `--replay-intrinsics <frames_dir> chessboard|charuco [cols rows] [capture_set.json]`: runs the intrinsics wizard's pattern finder, hold-still gate and lens solve over a folder of frames recorded by `tools/intrinsics_spike/pattern_probe.py`, reporting the samples the gate took and the solved focal length, distortion and reprojection error, the check for a change to the gate or a finder. A trailing `.json` path receives the captured views, the file the wizard leaves in the project's `dumps/` after each run
- `--replay-mono-baseline`: single-camera output of a recording measured against its multi-camera output, with the single-camera plan's acceptance gates
- `--replay-popmetrics`: pop statistics of a recording: baseline vs hand estimator
- `--render-avatar`: renders a VRM at rest (or a canned retargeted pose with a trailing `demo`) to a PNG through the scene renderer from a hidden window, with the model's rig sidecar applied
- `--render-direct <recording.jsonl> <outDir> [cameraIndex] [vrm=<file.vrm>] [tilt=<degrees>] [forearm=<meters>] [view=front] [near=<fraction>] [far=<fraction>]`: renders the avatar a one-camera direct project would show for every raw frame of a take, from the camera at the frame's size, one `frame_<frameIndex>.jpg` per raw frame. The whole-body model runs over the camera's raw frames, the hands are the recorded ones, and the retarget stage is the app's for a direct project on that camera (the first with body pose by default), so the take need not be a direct-mode recording. `vrm` names the avatar in place of the recorded one, and `tilt` and `forearm` stand in for the recorded direct tilt and forearm length. `view=front` draws the avatar from a 35 degree lens 1.4 m ahead of its hips in place of the tracking camera, so the arms stay in the picture when the user's hands leave the frame. `near` and `far` stand in for the depth planes' fractions of the reach. The face is at rest
- `--direct-trace <recording.jsonl> [out.csv]`: solves every frame of a direct-mode take through the retarget stage and writes the solve state per frame as CSV, with the shoulder jitter, inboard elbow and elbow pop counts in the log, the before and after of a direct solver or filter change
- `--pose-report <recording.jsonl> [frameIndex] [outDir]`: re-solves one frame of a direct-mode take or a dump's `dump.jsonl` through the retarget stage under the take's own chain settings and reports the posed avatar against the landmarks (reprojection per joint, each arm's mapping and IK weight with its wrist goal, arm angles and depths, finger angles) with a compare image, the loop after a solver change or a chain edit
- `--replay-verify`: re-runs a recording and verifies every frame checksum
- `--retarget-golden <recording.jsonl>`: the live position retarget path against its frozen copy over a recording, bit for bit (a direct-mode take is not compared and exits 1)
- `--replay-audio <recording.jsonl> [out.json] [--model loudness|mfcc|ovrlipsync] [--profile path] [--gain db] [--gate dbfs]`: re-runs the microphone mouth stage over a recording's audio sidecar on the live hop grid (under the recording's model unless `--model` names another) and reports the per-viseme difference against the recorded frames, the A/B for a model, profile, gain, or gate change; the JSON carries the `recorded` and `replayed` series, `model`, and `recordedModel` for `--render-mouth`
- `--viseme-bench <clip.wav> [out.json] [--model loudness|mfcc|ovrlipsync] [--hop ms] [--window ms] [--lookahead frames] [--profile path] [--cutoff hz] [--beta b]`: runs a mouth model over a wav clip under the live rolling-window policy (defaults: `mfcc`, volume-only without a profile, the live hop, window, and lookahead) and reports the share of frames each viseme dominated, the chunked frames' drift from one whole-clip run, the per-run inference p50 and p95 against the hop, and the viseme weights as JSON (the `offline`, `chunked`, and `filtered` series, with `model`)
- `--render-mouth <file.vrm> <bench.json> [out.png] [tiles] [chunked|offline|filtered]`: renders a contact sheet of the avatar's face under a bench JSON's viseme frames through the scene renderer and the avatar's Auto face profile from a hidden window, tiles spread evenly over the clip and labelled with the frame index and the dominant viseme
- `--vrm-info`: prints what the loader reads from a VRM file and the derived skeleton
- `--test-imupair`: solves the transform between two rigidly coupled IMUs from a dump (categorized as a Tool despite the `--test-` name, since it takes a dump file argument rather than running standalone)

Python, no build needed:

- `python tools/finger_bias.py <dump.json> [cameraIndex]`: the hand model's bend on a straight finger (the direct retarget's open-hand bias) from a dump taken with the hands held up open, palms to the camera: per-joint medians per hand and pooled
- `tools/intrinsics_spike/`: the lens what-if scripts over recordings (header patching, raw-equivalent frames, the focal length estimators, the phone pattern probes), run from their own venv as its README sets out
- `tools/kalidokit_ab/`: Kalidokit's 3d sample page fed a take's front-camera frames, rendered beside `--render-direct` into a side-by-side video, run as its README sets out

---

## Token stats

```
python tools/token_stats.py
```

Regenerates `TOKEN_STATS.md` and the AI-usage badge line in `README.md` from local Claude Code session transcripts.
