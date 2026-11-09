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
GenerateProjectFiles_X64_VS2022.bat
```

Configures `build/` with the `Visual Studio 17 2022` generator and produces `build/MikanTrack.sln`. Rerun after `InitialSetup_x64.bat`, and rerun any time a `.cpp` file is added to an existing `src/<Dir>` (the source list is a configure-time glob).

---

## Build

```
cmake --build build --target MikanTrack --config Release --parallel
```

Or open `build\MikanTrack.sln` in Visual Studio 2022 and build there. There is only the one target, `MikanTrack`.

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
- `--test-faceexpr`: face expressions: precedence, functions, names, variables, smooth, the time rule, hysteresis, state slots across a recompile, input sets, profile JSON, the library
- `--test-fusion`: multi-camera fusion, clustering, triangulation, holds
- `--test-handestimator`: angle-space multi-view hand state estimator
- `--test-handpose`: palm frame, finger angles, FK round-trip, conventions
- `--test-imudiscovery`: IMU device discovery never blocks the caller
- `--test-imufilter`: IMU orientation EKF, observability, gating
- `--test-pnp`: monocular PnP recovers a synthetic hand pose
- `--test-replay`: record/replay determinism, checksums, what-if, the audio and face sidecars
- `--test-roiquality`: hand ROI image-quality metrics + flicker tracker
- `--test-scene-view`: camera-matched scene view and overlay projection
- `--test-seeding`: cross-camera seed redundancy gate
- `--test-vmc`: VMC retarget: axis conversion, rest identity, chain round trip, the face profile binding and blend messages
- `--test-avatar-retarget`: avatar retarget: rest identity, reach scaling, two-bone IK, hands, fingers, root, VMC
- `--test-avatar-rig`: avatar rig: sidecar round trip, mapping overrides, trims, finger tweaks, elbow hint
- `--test-direct-retarget`: direct retarget: camera world, neutral capture, depth sign, unseen elbow, hold and rest, determinism, the body-wrist gate on hand slots
- `--test-setup-flow`: guided setup: the step chain of every tracking variant
- `--test-vrm`: VRM loader and rest skeleton on synthetic in-memory files, both VRM generations

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
- `--gate-direct`: whole-body 3D arm directions on one camera's raw frames against the stereo rig's output, the single-camera direct mode's gate, with the direct retarget and a Kalidokit-style baseline measured side by side (a direct-mode take runs their truth-free metrics alone)
- `--replay-bodypose`: re-run body pose over a recording's raw frames, on the recorded backend or a named one (`rtmpose` or `rtmw3d`)
- `--replay-dump`: emits a recording frame range with regenerated fusion diagnostics
- `--replay-extrinsics`: A/B a candidate extrinsics config against a recording
- `--replay-mono-baseline`: single-camera output of a recording measured against its multi-camera output, with the single-camera plan's acceptance gates
- `--replay-popmetrics`: pop statistics of a recording: baseline vs hand estimator
- `--render-avatar`: renders a VRM at rest (or a canned retargeted pose with a trailing `demo`) to a PNG through the scene renderer from a hidden window, with the model's rig sidecar applied
- `--pose-report <recording.jsonl> [frameIndex] [outDir]`: re-solves one frame of a direct-mode take or a dump's `dump.jsonl` through the direct retarget and reports the posed avatar against the landmarks (reprojection per joint, arm angles and depths, finger angles) with a compare image, the loop after a solver change
- `--replay-verify`: re-runs a recording and verifies every frame checksum
- `--replay-audio <recording.jsonl> [out.json] [--model loudness|mfcc|ovrlipsync] [--profile path] [--gain db] [--gate dbfs]`: re-runs the microphone mouth stage over a recording's audio sidecar on the live hop grid (under the recording's model unless `--model` names another) and reports the per-viseme difference against the recorded frames, the A/B for a model, profile, gain, or gate change; the JSON carries the `recorded` and `replayed` series, `model`, and `recordedModel` for `--render-mouth`
- `--viseme-bench <clip.wav> [out.json] [--model loudness|mfcc|ovrlipsync] [--hop ms] [--window ms] [--lookahead frames] [--profile path] [--cutoff hz] [--beta b]`: runs a mouth model over a wav clip under the live rolling-window policy (defaults: `mfcc`, volume-only without a profile, the live hop, window, and lookahead) and reports the share of frames each viseme dominated, the chunked frames' drift from one whole-clip run, the per-run inference p50 and p95 against the hop, and the viseme weights as JSON (the `offline`, `chunked`, and `filtered` series, with `model`)
- `--render-mouth <file.vrm> <bench.json> [out.png] [tiles] [chunked|offline|filtered]`: renders a contact sheet of the avatar's face under a bench JSON's viseme frames through the scene renderer and the avatar's Auto face profile from a hidden window, tiles spread evenly over the clip and labelled with the frame index and the dominant viseme
- `--vrm-info`: prints what the loader reads from a VRM file and the derived skeleton
- `--test-imupair`: solves the transform between two rigidly coupled IMUs from a dump (categorized as a Tool despite the `--test-` name, since it takes a dump file argument rather than running standalone)

Python, no build needed:

- `python tools/finger_bias.py <dump.json> [cameraIndex]`: the hand model's bend on a straight finger (the direct retarget's open-hand bias) from a dump taken with the hands held up open, palms to the camera: per-joint medians per hand and pooled

---

## Token stats

```
python tools/token_stats.py
```

Regenerates `TOKEN_STATS.md` and the AI-usage badge line in `README.md` from local Claude Code session transcripts.
