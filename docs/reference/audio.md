# Audio stage

The microphone mouth stage: one of three mouth models turns the voice into the 15 Oculus visemes (its level alone, uLipSync's MFCC scoring against the user's calibrated vowels, or the Oculus LipSync engine), and the viseme weights reach the avatar's mouth as the `viseme.*` inputs of the face profile, beside the phone face (see [face.md](./face.md)). This doc covers what exists: the frame and the viseme table, the backend and its three models, the rolling-window policy, the emission filter, the wav reader, the live stage and its thread, the config and the Audio panel, the profile, the calibration wizard, the recording sidecar and its replay, the bench, and what the bench measured. The design record, including the backends that were evaluated and dropped, is `docs/proposals/viseme.md`.

## Mouth frames and the viseme table (`src/Audio/AudioTypes.h`, `VisemeTable`)

A `MouthFrame` is one 30 fps frame of the 15 viseme weights in 0..1, in `eViseme` order (`VisemeTypes.h`, which is not the Oculus SDK's order: `aa`, `E`, `I`, `O`, `U`, `PP`, `SS`, `TH`, `DD`, `FF`, `kk`, `nn`, `RR`, `CH`, `sil`). The frame is what the backend emits, the filter smooths, the recording stores, and the vision thread publishes, and it reaches the avatar two ways at once:

- `TrackingFrameResult::face` carries the frame as its viseme source (`FaceSources::viseme`), present while the sample is no more than a second old. A face profile reads them as `viseme.*` inputs.
- The built-in Auto profile reads a VRM preset vowel (`aa`, `ih`, `ou`, `ee`, `oh`) straight from its viseme (`aa`, `I`, `U`, `E`, `O`), since the viseme is the vowel, and gives each ARKit-named expression or morph target the clamped sum of the table's terms for its column while the phone sees no face.

The viseme table (`VisemeTable.cpp`) is the fixed map from each viseme to the ARKit mouth columns it shapes: the jaw and lip shape of each sound, the vowels opening the jaw and shaping the lips (each leading with its defining column at 0.8 to 0.85, the jaw for `aa`, the smile for `I`, the stretch for `E`, the funnel for `O`, the pucker for `U`), the consonants the lip contact (`PP`), the lip under the teeth (`FF`), the tongue between them (`TH`), and the small openings of the rest. Silence drives nothing. The table only ever names the 29 columns the voice can drive (the four jaw columns, the twenty-three mouth columns, `cheekPuff`, and `tongueOut`), which `isMouthArkitIndex` names and a self-test holds it to, so the Auto profile never drives the eyes and brows from audio. `dominantViseme` names the heaviest viseme of a frame for the tools' labels.

The stage's constants: 16 kHz mono float audio in, 30 fps frames out, `mouthFramesForSamples` giving the frame count the backend emits for a span of samples, `ceil(30 * samples / 16000)`, and `mouthFrameEndSample` the sample frame f ends at, `round((f + 1) * 16000 / 30)`.

## The backend (`IMouthBackend`)

`IMouthBackend` (`src/Audio/MouthBackend.h`) is what the chain runs over each window, one implementation per mouth model. A run takes the window's 16 kHz samples and fills one `MouthFrame` per 30 fps frame of it, first to last. The interface:

- `isReady`
- `getInfo`: one line for the panel on what runs
- `getReason`: why the backend is not ready or runs degraded, empty when fine
- `runsWhileGated`: whether the backend must see the audio while the gate is closed
- `setVolumeWindow`: the level that leaves the mouth closed and the level that opens it fully, for the models that open the mouth by level
- `reset`: drops whatever carries between runs
- `infer(samples, count, newFrameCount, outFrames)`: `newFrameCount` is how many tail frames the chain emits from the run
- `getLastInferMs`

`createMouthBackend(model, profilePath, outReason)` (`MouthBackend.cpp`) builds the backend for an `eMouthModel`. It returns null with the reason for a model the build does not carry.

The gate decides what a run emits, and the backend decides what it sees. The chain runs the backend when the hop's gate is open, or on every hop when the backend `runsWhileGated`, and it emits the neutral mouth for a gated hop either way. The MFCC and loudness models are pure over the window and skip gated hops. The Oculus engine is stateful and sees every hop, so it never misses audio. `replayMouthChain` resets the backend before a replay, and the live service resets it when it creates one.

The three models are below. `MouthFilter` smooths all three alike after emission.

### Simple Loudness (`LoudnessBackend`)

The level alone, with no calibration. Per 30 fps frame it takes the RMS level of the same 1024-sample block the MFCC model analyzes (ending at `mouthFrameEndSample` of the frame, zero-padded at the front while the window is shorter than a block) through the gate-relative volume window below: `aa` at the volume and `sil` the remainder. It is pure over the window, like the MFCC model.

### MFCC (`MfccBackend`)

uLipSync's method (MIT, hecomi, https://github.com/hecomi/uLipSync, version 3.1.5, the engine behind VTube Studio's microphone lip sync), ported stage for stage so a profile calibrated there reads the same coefficients here. `MfccFeatures` turns one 1024-sample block at 16 kHz into 12 coefficients:

- pre-emphasis at 0.97, the first sample untouched
- a Hamming window
- a peak normalization to 1
- a radix-2 FFT to the magnitude spectrum
- a mel filterbank of `melFilterBankChannels` triangles (30 by default) on the 1127 ln mel scale from 0 to 8 kHz, with uLipSync's bin indexing
- 10 log10 of each band
- a direct, unnormalized DCT-II, keeping coefficients 1 to 12

uLipSync's windowed-sinc low-pass and its decimation have no counterpart: they anti-alias the host's audio before downsampling to 16 kHz, and the capture is already 16 kHz. The block's RMS level in dBFS comes out beside the coefficients, read before the chain touches the samples.

Scoring (`LipSyncProfile::score`) compares the coefficients against each phoneme's template, the mean of its calibration frames. With `useStandardization` on, both sides are first standardized by the profile's pooled mean and population standard deviation per coefficient (identity otherwise). The distance d per coefficient gives the phoneme's score by the profile's `compareMethod`:

- L1: `10^-(mean |d|)`
- L2, the default: `10^-sqrt(mean d^2)`
- cosine: `max(cos, 0)^100`

The scores are normalized to sum to 1, folded onto the visemes by phoneme name (the profile below), and the best phoneme's viseme is the frame's main viseme.

The frame rule: the backend turns a window of 16 kHz samples into the frames covering it, first to last, one block per 30 fps frame ending at `mouthFrameEndSample` of the frame, zero-padded at the front while the window is shorter than a block. The block's level is mapped linearly through the volume window to a volume in 0..1, closed at `gateDbfs` and fully open 20 dB above it (`MouthChainConfig::k_volumeSpanDb`), which is uLipSync's log10 RMS window of -2.5..-1.5 in the same shape at -50..-30 dBFS. A vowel's weight is its viseme share times the volume, `sil` is 1 minus the sum of the vowel weights, and the nine consonant columns of the 15-wide `VisemeWeights` stay zero. Without a profile the backend is volume-only by the Simple Loudness rule: `aa` at the volume and `sil` the remainder, so the jaw tracks speech before any calibration. A profile file that fails to load leaves the backend volume-only and ready, with the reason (`getReason`) shown in the panel. The info line reads "uLipSync profile, N phonemes" or "no profile, volume only".

Nothing carries between frames, so a run is pure over its window: the same samples give the same frames, and every frame of the window is scored whatever the chain emits. Which frames are new is the window policy's business, and smoothing happens after emission (the one-euro `MouthFilter`), so the bench can compare chunked and whole-clip output with nothing in between. `getLastInferMs` is the per-run cost, and `getLastFrameScores` holds each frame's level, volume, and scores for the tools.

### Oculus LipSync (`OvrLipSyncBackend`)

The Oculus Lipsync native SDK (OVRLipSync 1.61.0, Meta Platforms, under the Oculus SDK License), compiled in only when the SDK is present at build time (`MIKANTRACK_OVRLIPSYNC`, see [build.md](./build.md)). The SDK is a manual download unpacked to `deps/OVRLipSyncNative`. A build without it compiles `OvrLipSyncBackend.cpp` to nothing, and the factory reports "this build has no Oculus LipSync SDK" for a config naming the model. A build with it links the SDK's shim, a static loader that loads `OVRLipSync.dll` beside the exe at run time. When the DLL is missing the backend reports "OVRLipSync.dll is not beside MikanTrack.exe" and the mouth stays neutral. The DLL prints Caffe2 CPU-feature lines to stderr when it loads, which are harmless.

The engine is initialized at 16 kHz, the capture rate, with the `Enhanced` provider (not the one with laughter detection) and acceleration off. It accepts 16 kHz input directly: a voiced test buzz scores speech visemes and silence scores `sil` at 16 kHz as at 48 kHz, so no resampling is done. `ovrLipSync_Initialize` is process-wide, so the engine is held while any backend lives, and each backend owns one context. The info line is the SDK version, "Oculus LipSync 1.61.0".

The engine is stateful, so the backend feeds it each new frame's audio exactly once, in stream order. `runsWhileGated` is true, so it sees every hop, gated or not. A run feeds only the `newFrameCount` frames the chain emits, each frame's own slice of the window (from the previous frame's `mouthFrameEndSample` to its own, 533 or 534 samples) as one `ovrLipSync_ProcessFrameEx` call, and leaves the window's earlier frames neutral. The engine's fifteen scores come in the SDK's order (`sil`, `PP`, `FF`, `TH`, `DD`, `kk`, `CH`, `SS`, `nn`, `RR`, `aa`, `E`, `ih`, `oh`, `ou`) and are remapped into `eViseme` order and emitted raw, clamped to 0..1:

- no volume window (`setVolumeWindow` does nothing)
- the engine's own `VisemeSmoothing` signal set from the panel's Engine smoothing slider (`AudioConfig::ovrSmoothing`, 1 the raw prediction to 100 the smoothest, default 70, `setEngineSmoothing`), sent to each new context, with `MouthFilter` smoothing after emission as for every model
- the gain slider still scales its input

`reset` destroys and recreates the context: the SDK's own `ovrLipSync_ResetContext` keeps state behind (two runs after it differ by their history), and a fresh context is what makes a reused backend deterministic. The first frames after a reset sum below 1 while the engine warms up. The engine is deterministic for a given sample sequence, so a replay, which resets the backend and feeds the same samples in the same order, reproduces live.

## The window policy (`MouthWindow`)

Pure accounting with no backend and no clock. Captured samples are pushed in; once a hop's worth has arrived a run is due, `beginRun` hands out the window ending at that hop boundary (zero-padded at the front until enough audio has arrived) and returns how many of the backend's frames for it are new, and `takeTail` copies those frames off the end of the backend's output. Frames are due at `floor(30 * consumedSamples / 16000)`, so a 1600-sample hop is always three new frames and an odd hop size alternates counts without skipping or repeating a frame.

The config:

- `hopSamples`: the new audio between runs, 1600 samples (100 ms, three frames). A hop of a whole number of frames is what keeps a replay's frame grid exactly on live's.
- `windowSamples`: the span the backend sees per run, two hops (3200 samples). A whole number of frames, so the frame grid the backend lays over the window is the stream's own, with the block the first emitted frame reaches back over inside it.
- `lookaheadFrames`: frames held back from the tail of each run, 0. The backend scores a frame from the audio up to its end alone, so no frame needs audio after it, and each frame held would cost 33 ms of latency. The emitted count trails the frames due by the lookahead, clamped so the first runs never emit the padding before the clip's own first frame.

## The emission filter (`MouthFilter`)

A one-euro filter per viseme over the emitted 30 fps frames (`OneEuroFilter`, the hand filter). The filter smooths the frames across runs and eases the mouth shut when the gate closes instead of snapping it. Beta is 0.5 and the derivative cutoff 1 Hz. The minimum cutoff is the panel's Smoothing slider, a settling time in ms kept per model (`AudioConfig::smoothingMs`, defaults 20 ms for Simple Loudness and 50 ms for the other two, `MOUTH_SMOOTHING_DEFAULT_MS`), mapped by `smoothingCutoffHz` to `1000 / (2 pi ms)` Hz; 0 ms is no filter, the frames pass through as emitted. The service applies a change on the fly (`MouthChain::setSmoothingMs`, the filters keep their state). `--replay-audio` runs the recording's smoothing for the model it replays; `--viseme-bench` starts from the model's default unless `--cutoff` is given. The bench writes the filtered series beside the raw one, so the filter is judged on the avatar rather than assumed.

## The wav reader (`WavFile`)

RIFF wave reading for the bench clips and the recording sidecars: 8, 16, 24, and 32-bit PCM and 32-bit float, plain or `WAVE_FORMAT_EXTENSIBLE`, any channel count, chunks walked by size so a `LIST` chunk before the data is skipped. `toMono` averages the channels, `resample` goes through SDL's `SDL_AudioCVT` converter (no audio subsystem init needed), and `readMono16k` is the three in one.

## The live stage (`AudioService`)

`AudioService` (`src/Audio/AudioService.h`/`.cpp`) is the microphone in the app: SDL2 capture, an audio thread of its own, the configured model's backend, the chain from samples to frames, and a latest-wins `MouthSample` the vision thread reads. The chain is `MouthChain` (`src/Audio/MouthChain.h`/`.cpp`): the gain, the window policy above, the hop's level through the gate, the backend over the window, the tail frames through the filter, each emitted frame carrying its index in the capture stream. The live service and the offline replay below both run this one class, so a recording replays through exactly what live ran. `App` owns it like `FaceService`: created in `App::startup`, `startup()` on project activation (which initializes SDL's audio subsystem and spawns the thread), `shutdown()` wherever the face service shuts down. `App::applyAudioConfig()` maps `AppConfig::audio` to `AudioServiceConfig` and calls `setConfig`, which parks the config for the audio thread to adopt between blocks: an enable or device change reopens the device (which restarts the stream and its frame indices), a model or profile change recreates the backend under the running stream and resets it (the window keeps its samples and the indices keep counting, so a recording stays tied to its sidecar across the change), and the gain and gate apply at once.

The device is opened at 16 kHz mono float in 256-sample blocks with no allowed changes, so SDL converts whatever the hardware gives and the window only ever sees the backend's rate. The configured device name is used while a device still carries it and falls back to the system default with a warning, since a USB microphone comes and goes. SDL's callback thread copies each block into a 64-block SPSC queue (a second of audio) and the audio thread drains it. A block the queue cannot take is dropped and counted (`droppedBlocks` on the status), never waited for, so a stalled backend cannot back up the capture.

Per block the audio thread pushes the samples through the chain, which applies the gain (dB to linear), feeds `MouthWindow`, and runs the window policy's due runs. Each run measures the hop's RMS level in dBFS, feeds it to the gate, and runs the backend over the window when the gate is open or the backend `runsWhileGated`. A gate-open run emits the backend's tail frames, and a gated run emits the neutral mouth for its frames whether the backend ran or not. Both go through `MouthFilter`, so a gate closing eases the mouth shut rather than snapping it. The gate (`LevelGate`) opens at the configured threshold and closes 6 dB under it, which holds it through the dips inside a word. The backend's run time is tracked as an EMA for the panel.

The backend produces its frames in bursts, three per 100 ms hop, and a run holds the audio thread for longer than a frame, so the pacing lives with the readers rather than the thread: the bursts go into `MouthPacer` (a jitter buffer, six frames deep), and `getLatestSample` and `getStatus` draw it down one frame per 33.3 ms on the steady clock whoever calls them. A run that lands late only deepens the buffer by its delay, and a reader that polls late takes the newest due frame and drops the ones it supersedes. The vision thread therefore sees a frame per 30 Hz tick with the sequence stepping by one, and a mouth-only output frame goes out per frame rather than three at once. The latency from voice to published frame is the hop (100 ms) plus the backend's run plus up to one pacing interval.

`--audio-probe [seconds] [--device name] [--gate dbfs] [--gain db]` is the live stage without the UI: it lists the capture devices, runs the service on one for a few seconds, reports the level, the gate, the backend's run time, the frames per second, and the dominant viseme of each polled frame, and passes when the device opened, the backend is ready, and frames arrived. It prints the backend's info line, so the model and its profile state are in the log. It tells a microphone problem from a profile problem before the panel is involved, and a `--gate` under the room's floor makes it exercise the backend in a quiet room.

## Config and the panel

`AudioConfig` on `AppConfig::audio`, persisted under `audio` in the project JSON:

- `bool enabled` (default false)
- `std::string deviceName` (empty for the system default)
- `float gainDb` (default 0)
- `float gateDbfs` (default -45)
- `eMouthModel model` (default `Mfcc`), written as `"model"` with the values `loudness`, `mfcc`, and `ovrlipsync`, an unknown value reading as `mfcc`
- `std::string profilePath` (the MFCC model's profile: empty for none, the volume-only mouth, else a uLipSync profile JSON below)
- `std::array<float, MOUTH_MODEL_COUNT> smoothingMs` (the emission filter's settling time per model, written as `"smoothingMs": {"loudness": 20, "mfcc": 50, "ovrlipsync": 50}`, 0 for no filter, clamped to 0..200)
- `int ovrSmoothing` (the Oculus engine's own smoothing, 1..100, default 70)

The Microphone stage in the Inspector, under Face and mouth (its sections come from `src/UI/AudioPanel.h`/`.cpp`), holds the enable checkbox, the microphone combo (the device list is fetched on the first draw and on Refresh, since enumeration is not free), a level meter over the newest hop that turns green while the gate is open, the gate and gain sliders, the Model combo ("Simple Loudness", "MFCC (uLipSync)", "Oculus LipSync"), the Smoothing slider (the selected model's settling time), the Engine smoothing slider while Oculus LipSync is selected, and, while MFCC is selected, the profile block: the profile line (none and volume-only, or the path), the Calibrate button (the wizard below), the Import button that picks any uLipSync profile JSON, and, once a profile is set, the Clear profile button. The Oculus entry is listed only in a build carrying its SDK, though a config naming it still shows as the current choice. Every edit marks the config dirty and goes through `App::applyAudioConfig`. The status block below names the device and its rate, the model as `Model: <info>, N ms per run` while the backend is ready and `Model: unavailable (<reason>)` in red when a reason is set (an MFCC profile that failed to load shows both), which source owns the mouth under the Auto profile (the phone face, the microphone, gated, or off), and the dropped block count once any dropped. A custom profile decides for itself ([face.md](./face.md)). The block at the bottom is the 15 viseme weights as labelled bars, read from the frame the scene is showing: the live output while the microphone's sample is fresh, or the recorded frame while the timeline replay view is active. It is the readout that says whether the voice is producing clear vowel calls before the avatar is involved.

What those weights become on the loaded avatar is the Profile stage's Diagnostics, the avatar mouth expressions (`src/UI/AvatarMouthReadout.h`/`.cpp`): the mouth targets of the active binding, the targets with an output reading a mouth input (the phone's mouth columns or a viseme), with their values on the live frame. The header counts those targets, and the source line says what they read this frame: the phone, the visemes, or both. It is a filtered view of the Avatar panel's Face tab. It is where a shape that looks wrong on the avatar is told apart from a weight that never arrived.

## The profile (`LipSyncProfile`)

A profile is uLipSync's JSON, the file its `Export` writes and VTube Studio hands out, field for field, so a profile exported from uLipSync or VTube Studio loads as is and one written here loads there:

- `jsonPath`
- `mfccNum`: must be 12
- `mfccDataCount`: the frames kept per phoneme, 16 by default
- `melFilterBankChannels`: 30 by default
- `targetSampleRate`: must be 16000
- `sampleCount`: the block, must be 1024
- `useStandardization`: false by default
- `compareMethod`: 0 L1, 1 L2, 2 cosine, 1 by default
- `mfccs`: `[{name, mfccCalibrationDataList: [{array: [12 floats]}]}]`, the phonemes and their frames

On load each phoneme keeps only its last `mfccDataCount` frames, as uLipSync does when it enables a profile, so the templates come out the same there and here. A file with another coefficient count, block size, or sample rate is refused with the reason. `useStandardization` and `compareMethod` are honored from the file and have no UI.

A phoneme's name picks its viseme, case-insensitive (`visemeForPhonemeName`):

- `A`, `aa`: `aa`
- `I`, `ih`: `I`
- `U`, `ou`, `oo`: `U`
- `E`, `eh`: `E`
- `O`, `oh`: `O`
- `-`, `N`, `sil`, `silence`, `noise`: `sil`

A name outside the list still scores, and its share folds into `sil`.

## Calibration (`AudioCalibrationWizard`)

The microphone calibration wizard (`src/UI/AudioCalibrationWizard.h`/`.cpp`, `eWizardKind::AudioCalibration` on the `WizardHost`, the Calibrate button in the Microphone stage's profile block) builds a profile from the user's own voice and room. It needs no tracking, only the audio service, and the button is enabled while the stage has a device open and no other wizard runs.

Six prompts of three seconds each, every one behind the three-second countdown the other wizards use, the vowel held steadily after it: the visemes `aa`, `I`, `U`, `E`, `O` as the vowels of father, sit, boot, bed, boat (the Japanese walkthrough prompts the five Japanese vowels in the same order, whose shapes land on the same visemes), then the room's silence. A prompt collects the raw stream from the service (`AudioService::startCapture`/`stopCapture`, the audio thread appending each block to a vector reserved up front, dropped by a device reopen) and turns it into MFCC frames through `calibrationFrames` in `src/Audio/LipSyncProfile.h`/`.cpp`: one block per 30 fps frame of the capture, the backend's own extractor, kept when the block's level reaches the gate. A vowel prompt that keeps fewer than 15 frames (half a second) reports too quiet and is recorded again. The silence prompt keeps every frame as the noise phoneme `-`, and suggests the gate (`suggestGateDbfs` in `src/Audio/MouthChain.h`): its loudest hop in dBFS, after the configured gain, plus 6 dB, clamped to the slider's range.

The profile is built with the phonemes `A`, `I`, `U`, `E`, `O`, and `-`, uLipSync's defaults, and `mfccDataCount` set to the largest prompt's frame count, so no prompt is trimmed on load. The review page shows per vowel the share of its own frames whose best phoneme is itself, green from 80 percent: a lower share means two prompts overlapped (an `oo` drifting toward `oh` is the usual one), and Redo records the six again. Finish writes `<project>/audio/profile.json`, sets `AudioConfig::profilePath` to it, `gateDbfs` to the suggestion, and the model to MFCC, since a fresh calibration is meant to drive the mouth, marks the config dirty, and applies it, which recreates the backend on the new file. `--audio-probe` runs the capture tap for a second at the end of its run and reports the calibration frames that passed the gate and the gate the clip suggests, the data path a prompt takes without the UI.

## Recording and replay

A tracking recording made while the stage is enabled carries the microphone with it. `AudioService::startRecording` writes the captured stream, before the gain and the gate, as 16 kHz mono 16-bit PCM to `<recording>.wav` beside the JSONL (`TrackingRecording::makeAudioSidecarPath`), through `AudioRecorder` (`src/Audio/AudioRecorder.h`/`.cpp`): the audio thread hands each raw block to a bounded queue and a writer thread turns them into the file through `WavWriter`. A queue overflow neither aborts nor shifts the file: the dropped blocks are written as silence in their place when the queue has room again, and counted, so a sample's index in the file stays its index in the stream. The wav header's sizes are placeholders until the file closes, set to the maximum, so a file cut short by a crash still reads to its end.

The recording ties its frames to the audio by index rather than by clock. Every emitted frame carries its index in the capture stream (frame i scores the audio around i/30 s after the device opened), the sidecar's header block records the stream index of its first sample, and each recorded iteration carries the newest mouth frame with that index:

- the header's `audio` block: the file name (resolved beside the recording, so a moved folder still works), the sample rate, `startSample`, and the start time on the steady clock. The model and the profile the run used are in the header's config snapshot (`audio.model`, `audio.profilePath`), so a recording remembers its model.
- each frame's `mouth` block: the 15 viseme weights as the stage emitted them and `frameIndex`, which is -1 when the sidecar does not hold that frame's audio (the device reopened mid-recording, which ends the sidecar and restarts the indices). A `mouth` block of another width is from before the frame carried visemes and is skipped on load.

The vision thread starts the sidecar before it writes the JSONL header, since the header carries where the sidecar begins, and ends it with the recording. The Timeline panel's recording status shows the sidecar's length while it is written. During playback every timeline view (recorded, replayed, what-if) evaluates the current face profile over the frame's recorded visemes (and the face sidecar's phone samples, [face.md](./face.md)), since the audio stage is not re-run by the timeline, and the Audio panel's weights block reads the same frame. A recording made with the stage off has no sidecar and no mouth blocks, and an older recording without either loads as before. The mouth weights sit outside the checksum, like the IMU output, which is what lets a profile change be measured against old recordings.

`--replay-audio <recording.jsonl> [out.json] [--model loudness|mfcc|ovrlipsync] [--profile path] [--gain db] [--gate dbfs]` re-runs the chain over the sidecar and measures it against the recorded frames. The model is `--model`, else the recording's `audio.model`. The profile is `--profile`, else the recording's `audio.profilePath`, else none (volume-only). The chain runs at every hop boundary counted from the device's first sample, while a sidecar starts somewhere inside a hop, so the tool pads the sidecar with `startSample mod hop` samples of silence to put its hop boundaries where live had them, after which a replayed frame j is the live frame j plus the frames due at that boundary (`mouthReplayAlignment` in `src/Audio/MouthReplay.h`). The recorded model and profile under the recorded gain and gate then reproduce the live frames to float precision once the window has filled: the first window's frames replay from padding where live had the stream's history, so they are measured apart as the warm-up. The report is the per-viseme mean absolute difference against the recorded frames beside each viseme's recorded activity, the worst viseme, the run p50 and p95, and optionally a JSON in the bench's shape with the `recorded` and `replayed` series, the `model` and `recordedModel` names, and the `profile` and `recordedProfile` paths, which `--render-mouth` renders. Another model, profile, gain, or gate replays the same voice under the change, the A/B the sidecar exists for.

## The bench (`--viseme-bench`)

```bash
cd build/RelWithDebInfo && ./MikanTrack.exe --viseme-bench resources/audio/bench/bench_en.wav bench_en.json --profile <project>/audio/profile.json
```

Runs the backend of `--model loudness|mfcc|ovrlipsync` (`mfcc` by default, under `--profile`, volume-only without one) once over the whole clip (the reference), then feeds the clip in hops through the window policy the way capture will, and reports:

- the share of the clip's frames each viseme won, from the dominant viseme of the emitted frames
- the chunked frames' mean absolute drift from the whole-clip run, per viseme and overall, beside each viseme's mean activity so the drift reads relative to motion
- the per-run inference p50, p95, and max against the hop, as `LATENCY PASS` or `FAIL`, and the share of real time the runs took
- the `offline`, `chunked`, and `filtered` frame series as JSON, with the `model` name and the `profile` path, for `--render-mouth`

`--render-mouth <file.vrm> <bench.json> [out.png] [tiles] [series]` renders a contact sheet of an avatar's face under the frames through the scene renderer and the avatar's Auto face profile from a hidden window, the visemes applied as the live stream applies them with the microphone alone, tiles spread evenly over the clip and labelled with the frame index and the dominant viseme with its weight. It is the headless way to see what the weights do to a real mouth, on a preset avatar and on a perfect-sync one. `--test-audio` covers the pieces above on synthetic input.

The standing inputs are two Kokoro-synthesized clips, English (25 s, Harvard Sentences) and Japanese (21 s), fetched into `resources/audio/bench/` by the setup script.

## What the bench measured

On this development machine, the default two-hop window at a 100 ms hop, volume-only over the English clip (25 s, 742 frames):

| Run per window, p50 / p95 / max | 0.07 / 0.15 / 0.25 ms |
|---|---|
| Share of real time | 0.0007 |
| Chunked drift from the whole-clip run | 0 (the window's frame grid is the stream's, so a frame scores the same block either way) |

The stage costs well under a tenth of a percent of a core, and nothing of the hop is spent. The profile's separation was measured on a recording of "aa, ih, oo, eh, oh" held in turn: templates taken from the first 60 percent of each vowel plus a silent gap as the noise phoneme, replayed through `--replay-audio` over the whole recording, call `aa`, `I`, `U`, `E`, `O` in order, each as one contiguous run over its whole segment including the 40 percent the templates never saw, with a two-frame `U` at the onset of "oh" the only stray call. The same recording under the dropped classifier had called `ih` as `U`, `eh` as `PP`, and `oo` five ways.
