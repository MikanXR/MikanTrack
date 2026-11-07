# Audio stage

The microphone mouth stage: the voice drives the jaw, mouth, cheek puff, and tongue blendshapes through the same ARKit columns, face map, and VMC output the phone face uses (see [face.md](./face.md)). This doc covers what exists: the types, the backend contract and the two backends, the rolling-window policy, the emission filter, the wav reader, the live stage and its thread, the config and the Audio panel, the recording sidecar and its replay, the bench, and what the bench measured. The design and the piece still to land (the calibration wizard) are in `docs/proposals/viseme.md`.

## The mouth column group (`src/Audio/AudioTypes.h`)

Audio cannot see eyes or brows, so a backend's output is cut down to the 29 ARKit columns the voice drives: the four jaw columns, the twenty-three mouth columns, `cheekPuff`, and `tongueOut`. `mouthColumnArkitIndex`, `mouthColumnFromArkitIndex`, and `mouthColumnName` map between the mouth column order and the ARKit table in `FaceTypes.cpp`, where the jaw and mouth columns sit contiguously. A `MouthFrame` is one 30 fps frame of those 29 weights in 0..1. `applyMouthFrame` writes it onto a 52-wide face leaving the other 23 columns untouched, which is how the live merge keeps the phone's eyes and brows while audio owns the mouth. `MouthSample` is the latest-wins publication shape the live stage will use, a `FaceSample` reduced to its mouth.

The stage's constants: 16 kHz mono float audio in, 30 fps frames out, and `mouthFramesForSamples` giving the frame count a backend emits for a span of samples, `ceil(30 * samples / 16000)`, the wav2arkit model's own formula.

## Backends (`IMouthBackend`)

A backend turns a window of 16 kHz samples into the frames covering it, first to last, and is pure over the window: the same samples give the same frames. Which frames are new is the window policy's business, and smoothing happens after emission, so the bench can compare a backend's chunked and whole-clip output with nothing in between. A backend that cannot run (model file missing, profile unreadable) reports not ready with a reason string the panel will show. `getLastInferMs` is the per-run cost.

### wav2arkit (`Wav2ArkitBackend`)

The `wav2arkit_cpu` model (Apache 2.0, Myned AI), a fused ONNX export of the wav2vec2-base-960h encoder and the LAM Audio2Expression decoder: `audio_waveform` float32 `[1, samples]` in, `blendshapes` float32 `[1, frames, 52]` out, values in 0..1, frames by the ceil formula above. The model lists its 52 outputs in alphabetical order, not ARKit's, so the backend carries that order as a name table and maps each output to its mouth column at create time (`outputColumnArkitIndex`, covered by `--test-audio`). Weights are clamped to 0..1 on the way out. The encoder's convolution stack needs at least 400 samples, below which a run is refused.

The session is an `OnnxSession` on the CPU provider with a capped intra-op thread count (default 4, the bench sweeps it), so the DirectML queue stays with the vision models. `create` takes the provider string as its second argument (`"cpu"` or `"directml"`) and the cap as its third; `OnnxSession::create` grew the same cap, where 0 leaves the runtime's default, which is what the vision models want. The shipped graph does not load on DirectML (the measurements below), so `"directml"` is a bench option, not a product path. The backend holds the session through a pointer and must be created, run, and destroyed on one thread, the session's rule.

The model and its external weights (`models/wav2arkit_cpu.onnx`, `wav2arkit_cpu.onnx.data`, about 0.4 GB) are fetched by `InitialSetup_x64.bat` (see [build.md](./build.md)).

### mfcc (`MfccBackend`)

HeadAudio's method (MIT, Mika Suominen, https://github.com/met4citizen/HeadAudio), the reimplementation of uLipSync's, ported stage for stage so its shipped model reads the same vectors:

- `MfccFeatures`: pre-emphasis (0.97), 512-sample blocks at a 256-sample hop, a Hamming window, a radix-2 FFT, the power spectrum and its log energy, 40 triangular mel bands over 30..7800 Hz, a DCT keeping coefficients 1..12 under a sinusoidal lifter, then tanh compression. HeadAudio's polyphase downsampler has no counterpart, since the stage's input is already 16 kHz.
- `VadGate`: HeadAudio's energy gate, open after a hop above -40 dB and closed after one below -50 dB (the thresholds and the hop counts are its config). A gated hop is silence and feeds no vote.
- `VisemeModel`: the binary model, a flat run of 368-byte records (an 8-byte header with the phoneme packed as two 16-bit code points, the group, and the viseme id, then the 12-float mean and the 78-float packed lower triangle of the inverse covariance). `VisemeClassifier` takes the nearest prototype by squared Mahalanobis distance, silence prototypes at their distance over the sensitivity (1.2), and votes over a ring of the last six hops, the most frequent viseme winning and ties going to the highest id.
- the crossfade: the winning viseme's weight ramps toward one and every other toward zero over 100 ms, through HeadAudio's sigmoid easing, one step per hop.
- `VisemeTable`: the fixed map from the 15 Oculus visemes to the mouth columns (the jaw and lip shape of each sound), each column the clamped sum of its terms. Each vowel row leads with the column the VRM preset map derives that vowel from (the jaw for aa, the smile for I, the stretch for E, the funnel for O, the pucker for U) at 0.8 to 0.85, so a preset avatar gets a full shape at a full viseme, and carries the secondary columns a perfect-sync avatar reads. It is the only place the two backends differ in what they hand the merge.

A run is pure over its window: the gate, the vote, and the crossfade restart from silence at the window's start, so a window's first hundred milliseconds are warm-up and the emitted tail is not. Each 30 fps frame takes the weights of the latest hop centered at or before its center. The model is `resources/audio/model-en-mixed.bin`, HeadAudio's shipped one (39 prototypes, four Kokoro voices over Harvard Sentences), committed with the tree since it is 14 kB; its silence prototype is the zero vector under a very tight covariance, which only digital silence reaches, so the gate is what calls a pause silence. The viseme ids follow HeadAudio's order (`eViseme`), which is not the Oculus SDK's.

## The window policy (`MouthWindow`)

Pure accounting with no backend and no clock. Captured samples are pushed in; once a hop's worth has arrived a run is due, `beginRun` hands out the window ending at that hop boundary (zero-padded at the front until enough audio has arrived, the LAM decoder's own first-call behavior) and returns how many of the backend's frames for it are new, and `takeTail` copies those frames off the end of the backend's output. Frames are due at `floor(30 * consumedSamples / 16000)`, so a 1600-sample hop is always three new frames and an odd hop size alternates counts without skipping or repeating a frame.

The config:

- `windowSamples`: the span a backend sees per run, 1 s by default (the measurements below)
- `hopSamples`: the new audio between runs, 100 ms
- `lookaheadFrames`: frames held back from the tail of each run, so an emitted frame has that much audio after it, two by default (the measurements below). Each frame costs 33 ms of latency and buys the encoder a look past the frame it is scoring. The emitted count trails the frames due by the lookahead, clamped so the first runs never emit the padding before the clip's own first frame.

## The emission filter (`MouthFilter`)

A one-euro filter per mouth column over the emitted 30 fps frames (`OneEuroFilter`, the hand filter). The LAM post-processing that smooths the decoder's frames (a Savitzky-Golay pass, symmetrization) lives in Python and is not in the exported graph, so this stands in for it. Defaults: a 3 Hz minimum cutoff, beta 0.5, a 1 Hz derivative cutoff. The bench writes the filtered series beside the raw one, so the filter is judged on the avatar rather than assumed.

## The wav reader (`WavFile`)

RIFF wave reading for the bench clips and, later, the recording sidecars: 8, 16, 24, and 32-bit PCM and 32-bit float, plain or `WAVE_FORMAT_EXTENSIBLE`, any channel count, chunks walked by size so a `LIST` chunk before the data is skipped. `toMono` averages the channels, `resample` goes through SDL's `SDL_AudioCVT` converter (no audio subsystem init needed), and `readMono16k` is the three in one.

## The live stage (`AudioService`)

`AudioService` (`src/Audio/AudioService.h`/`.cpp`) is the microphone in the app: SDL2 capture, an audio thread of its own, one backend, the chain from samples to frames, and a latest-wins `MouthSample` the vision thread reads. The chain is `MouthChain` (`src/Audio/MouthChain.h`/`.cpp`): the gain, the window policy above, the hop's level through the gate, the backend over the window, the tail frames through the filter, each emitted frame carrying its index in the capture stream. The live service and the offline replay below both run this one class, so a recording replays through exactly what live ran. `App` owns it like `FaceService`: created in `App::startup`, `startup()` on project activation (which initializes SDL's audio subsystem and spawns the thread), `shutdown()` wherever the face service shuts down. `App::applyAudioConfig()` maps `AppConfig::audio` to `AudioServiceConfig` and calls `setConfig`, which parks the config for the audio thread to adopt between blocks: an enable or device change reopens the device (which restarts the stream and its frame indices), a backend or profile change recreates the backend under the running stream (the window keeps its samples and the indices keep counting, so a recording stays tied to its sidecar across the change), and the gain and gate apply at once.

The device is opened at 16 kHz mono float in 256-sample blocks with no allowed changes, so SDL converts whatever the hardware gives and the window only ever sees the backend's rate. The configured device name is used while a device still carries it and falls back to the system default with a warning, since a USB microphone comes and goes. SDL's callback thread copies each block into a 64-block SPSC queue (a second of audio) and the audio thread drains it. A block the queue cannot take is dropped and counted (`droppedBlocks` on the status), never waited for, so a stalled backend cannot back up the capture.

Per block the audio thread pushes the samples through the chain, which applies the gain (dB to linear), feeds `MouthWindow`, and runs the window policy's due runs. Each run measures the hop's RMS level in dBFS, feeds it to the gate, and either runs the backend over the window (gate open and backend ready) or emits the neutral mouth for the run's frames. Both go through `MouthFilter`, so a gate closing eases the mouth shut rather than snapping it. The gate (`LevelGate`) opens at the configured threshold and closes 6 dB under it, which holds it through the dips inside a word. The backend's run time is tracked as an EMA for the panel.

The backend produces its frames in bursts, three per 100 ms hop, and a run holds the audio thread for longer than a frame, so the pacing lives with the readers rather than the thread: the bursts go into `MouthPacer` (a jitter buffer, six frames deep), and `getLatestSample` and `getStatus` draw it down one frame per 33.3 ms on the steady clock whoever calls them. A run that lands late only deepens the buffer by its delay, and a reader that polls late takes the newest due frame and drops the ones it supersedes. The vision thread therefore sees a frame per 30 Hz tick with the sequence stepping by one, and a mouth-only output frame goes out per frame rather than three at once. The latency from voice to published frame is the hop (100 ms) plus the lookahead (two frames, 67 ms) plus the backend's run plus up to one pacing interval.

`--audio-probe [seconds] [--backend wav2arkit|mfcc] [--device name] [--gate dbfs] [--gain db]` is the live stage without the UI: it lists the capture devices, runs the service on one for a few seconds, reports the level, the gate, the backend's run time, and the frames per second, and passes when the device opened, the backend is ready, and frames arrived. It tells a microphone problem from a model problem before the panel is involved, and a `--gate` under the room's floor makes it exercise the backend in a quiet room.

## Config and the panel

`AudioConfig` on `AppConfig::audio`, persisted under `audio` in the project JSON:

- `bool enabled` (default false)
- `std::string deviceName` (empty for the system default)
- `std::string backend` (`wav2arkit` by default, or `mfcc`)
- `float gainDb` (default 0)
- `float gateDbfs` (default -45)
- `std::string profilePath` (empty for the shipped mfcc model, else the calibration wizard's profile below)

The Audio panel (`src/UI/AudioPanel.h`/`.cpp`, `View > Audio Panel`, docked in the left column) holds the enable checkbox, the backend and microphone combos (the device list is fetched on the first draw and on Refresh, since enumeration is not free), a level meter over the newest hop that turns green while the gate is open, the gate and gain sliders, and the mfcc profile line with the Calibrate button (the wizard below) and, once a profile is set, the button back to the shipped model. Every edit marks the config dirty and goes through `App::applyAudioConfig`. The status block below names the device and its rate, the backend with its run time or the reason it is unavailable, which source owns the mouth (the phone face, the microphone, gated, or off), and the dropped block count once any dropped. The merge rule that decides the owner is in [face.md](./face.md). The two blocks at the bottom read the face the scene is showing, the live output while tracking runs (the microphone or the phone, whichever owns the mouth) or the recorded frame while the timeline replay view is active: the 29 mouth columns as labelled bars, then the loaded avatar's mouth expressions as its face map derives them from that face (the vowel presets of a preset avatar, the mouth-named expressions of a perfect-sync one, where the bars repeat the columns). The second block is where a shape that looks wrong on the avatar is told apart from a weight that never arrived.

## Calibration (`AudioCalibrationWizard`)

The mfcc backend's shipped model was fit on synthesized voices, and a vowel prototype is where a voice and a microphone show most, so the microphone calibration wizard (`src/UI/AudioCalibrationWizard.h`/`.cpp`, `eWizardKind::AudioCalibration` on the `WizardHost`, the Calibrate button in the Audio panel's profile block) refits the five vowel visemes to the user and leaves the rest of the model alone. It needs no tracking, only the audio service, and the button is enabled while the stage has a device open and no other wizard runs.

Six prompts of three seconds each, every one behind the three-second countdown the other wizards use, the vowel held steadily after it: the visemes `aa`, `I`, `U`, `E`, `O` as the vowels of father, sit, boot, bed, boat (the Japanese walkthrough prompts the five Japanese vowels in the same order, whose shapes land on the same visemes), then the room's silence. A prompt collects the raw stream from the service (`AudioService::startCapture`/`stopCapture`, the audio thread appending each block to a vector reserved up front, dropped by a device reopen) and runs it through the fit's front end, `extractSpeechVectors` in `src/Audio/VisemeFit.h`/`.cpp`: the backend's own chain (pre-emphasis, the extractor, the tanh compression) with HeadAudio's energy gate restarted at the clip's start, so a fit reads exactly the vectors the classifier will see. A vowel prompt that leaves fewer than thirteen voiced hops (a fifth of a second, the rank the covariance needs) is refused with the reason and recorded again. The silence prompt suggests the gate: its loudest hop in dBFS, after the configured gain, plus 6 dB, clamped to the slider's range.

The fit (`fitVisemePrototype`) is one Gaussian per vowel: the mean of its vectors and the inverse of their sample covariance, with 0.25 added to every diagonal term before the inversion (about a third of the shipped prototypes' typical per-coefficient variance of 0.7). The floor is what keeps a vowel held too steadily from getting a prototype so tight that the same vowel in running speech misses it, and it is also what makes the covariance invert on a short capture. `overlayVisemePrototypes` builds the profile from the shipped model: every shipped prototype of a fitted viseme is dropped (the shipped model carries nineteen across the five vowels, several phonemes per viseme), the consonants and the silence prototype are kept, and the five fits are appended with group byte 1, which tells them from the shipped records in a dump. The profile is a complete model file in the shipped format (`VisemeModel::save`, the record encoder the inverse of the decoder, a phoneme past two code points cut to two), so the backend loads it through the one path it has.

The review classifies each vowel's own vectors against the new profile by nearest prototype and shows the share called its viseme, green from 80 percent: a lower share means two prompts overlapped (an `oo` drifting toward `oh` is the usual one), and Redo records the six again. Finish writes `<project>/audio/profile.bin`, sets `AudioConfig::profilePath` to it and `gateDbfs` to the suggestion, marks the config dirty, and applies it, which recreates the backend on the new file. The panel's "Use shipped model" button beside Calibrate clears the path again. `--audio-probe` runs the capture tap for a second at the end of its run and reports the samples it collected, the hops that passed the energy gate, and the gate the clip suggests, the data path a prompt takes without the UI.

## Recording and replay

A tracking recording made while the stage is enabled carries the microphone with it. `AudioService::startRecording` writes the captured stream, before the gain and the gate, as 16 kHz mono 16-bit PCM to `<recording>.wav` beside the JSONL (`TrackingRecording::makeAudioSidecarPath`), through `AudioRecorder` (`src/Audio/AudioRecorder.h`/`.cpp`): the audio thread hands each raw block to a bounded queue and a writer thread turns them into the file through `WavWriter`. A queue overflow neither aborts nor shifts the file: the dropped blocks are written as silence in their place when the queue has room again, and counted, so a sample's index in the file stays its index in the stream. The wav header's sizes are placeholders until the file closes, set to the maximum, so a file cut short by a crash still reads to its end.

The recording ties its frames to the audio by index rather than by clock. Every emitted frame carries its index in the capture stream (frame i scores the audio around i/30 s after the device opened), the sidecar's header block records the stream index of its first sample, and each recorded iteration carries the newest mouth frame with that index:

- the header's `audio` block: the file name (resolved beside the recording, so a moved folder still works), the sample rate, `startSample`, the start time on the steady clock, and the backend
- each frame's `mouth` block: the 29 mouth weights as the stage emitted them and `frameIndex`, which is -1 when the sidecar does not hold that frame's audio (the device reopened mid-recording, which ends the sidecar and restarts the indices)

The vision thread starts the sidecar before it writes the JSONL header, since the header carries where the sidecar begins, and ends it with the recording. The Timeline panel's recording status shows the sidecar's length while it is written. During playback every timeline view (recorded, replayed, what-if) shows the frame's recorded mouth over a neutral face, since the audio stage is not re-run by the timeline, and the Audio panel's weights block reads the same frame. A recording made with the stage off has no sidecar and no mouth blocks, and an older recording without either loads as before. The mouth weights sit outside the checksum, like the IMU output, which is what lets a backend change be measured against old recordings.

`--replay-audio <recording.jsonl> [out.json] [--backend wav2arkit|mfcc] [--model path] [--gain db] [--gate dbfs]` re-runs the chain over the sidecar and measures it against the recorded frames. The chain runs at every hop boundary counted from the device's first sample, while a sidecar starts somewhere inside a hop, so the tool pads the sidecar with `startSample mod hop` samples of silence to put its hop boundaries where live had them, after which a replayed frame j is the live frame j plus the frames due at that boundary (`mouthReplayAlignment` in `src/Audio/MouthReplay.h`). The same backend under the recorded gain and gate then reproduces the live frames to float precision once the window has filled: the first window's frames replay from padding where live had the stream's history, so they are measured apart as the warm-up. The report is the per-column mean absolute difference against the recorded frames beside each column's recorded activity, the worst column, the run p50 and p95, and optionally a JSON in the bench's shape with the `recorded` and `replayed` series, which `--render-mouth` renders. A different backend, model, gain, or gate replays the same voice under the change, the A/B the sidecar exists for.

## The bench (`--viseme-bench`)

```bash
cd build/RelWithDebInfo && ./MikanTrack.exe --viseme-bench resources/audio/bench/bench_en.wav bench_en.json --backend wav2arkit --window 1000 --threads 4 --lookahead 2 --ep cpu
```

Runs the chosen backend (`--backend wav2arkit|mfcc`, `--model` overriding its default model) once over the whole clip (the reference, skipped with a warning when the model cannot take a whole clip, as a fixed-shape export cannot), then feeds the clip in hops through the window policy the way capture will, and reports:

- the chunked frames' mean absolute drift from the whole-clip run, per mouth column and overall, beside each column's mean activity so the drift reads relative to motion
- the per-run inference p50, p95, and max against the hop, as `LATENCY PASS` or `FAIL`, and the share of real time the runs took
- the `offline`, `chunked`, and `filtered` frame series as JSON, for `--render-mouth`
- for the mfcc backend, the share of the clip's hops each viseme won, the readout that says whether the classifier calls anything but silence

`--render-mouth <file.vrm> <bench.json> [out.png] [tiles] [series]` renders a contact sheet of an avatar's face under the frames through the scene renderer and the face map from a hidden window, tiles spread evenly over the clip and labelled with the frame index and `jawOpen`. It is the headless way to see what the weights do to a real mouth. `--test-audio` covers the pieces above on synthetic input and, with the model present, the backend's shape and range.

The standing inputs are two Kokoro-synthesized clips, English (25 s, Harvard Sentences) and Japanese (21 s), fetched into `resources/audio/bench/` by the setup script.

## What the bench measured

### The wav2arkit backend

wav2arkit on this development machine (CPU provider), both clips, 100 ms hop:

| Window | Threads | p50 / p95 per run |
|---|---|---|
| 2 s | 2 | 162 / 191 ms |
| 2 s | 4 | 91 / 94 ms |
| 1.5 s | 4 | 72 / 94 ms |
| 1 s | 2 | 86 / 96 ms |
| 1 s | 4 | 50 / 53 ms |

The window length barely changes the drift (0.0151 at 2 s against 0.0160 at 1 s with no lookahead), so the cheapest window is the right one: 1 s on 4 threads passes the hop with room, 2 s on 2 threads cannot keep up at all. The run cost is linear in the window.

Drift of the chunked frames from the whole-clip run, mean absolute per column over a mean activity of 0.047, 1 s window:

| Lookahead | English | Japanese | Added latency |
|---|---|---|---|
| 0 frames | 0.0160 | 0.0162 | 0 ms |
| 2 frames | 0.0106 | 0.0115 | 67 ms |
| 4 frames | 0.0077 | 0.0093 | 133 ms |
| 6 frames | 0.0068 | 0.0085 | 200 ms |

The two languages behave the same, in drift and in which columns move. The worst column is `mouthLowerDownLeft` throughout (0.079 without lookahead, 0.036 at four frames, over an activity of 0.32). End-to-end the emitted frames trail the voice by the hop plus the lookahead plus the run, about 150 ms with no lookahead and 220 ms at two frames. The default is two frames: the first two buy a third of the drift for 67 ms, the next two buy a quarter more for another 67 ms, and the lookahead costs no CPU, only latency.

### The CPU cost, and where it comes from

The 1 s window on 4 threads runs in either 30 ms or 50 ms per hop, and which mode a process lands in is set at launch and holds for its life. The machine is a Ryzen 9 7950X (two eight-core dies, SMT): with the process pinned to four distinct physical cores of one die (`start /affinity 55`) every run lands in the 30 ms mode (p50 29 to 31, p95 30 to 36), pinned to one whole die it is either, free-running it is either. The runtime's four intra-op threads are being placed on SMT siblings or across the dies, so the real lever on the CPU cost is thread affinity, not the model. ONNX Runtime 1.24.4 (built against the NuGet package in a scratch tree) shows the same two modes at the same costs as the pinned 1.20.1, so a runtime upgrade buys nothing here.

Fewer threads cost less CPU but leave no room under the hop: 1 thread runs at 85 / 90 ms (p50 / p95), 2 threads at 70 / 94 ms. At 4 threads the share of real time is 0.3 to 0.5 of wall time with four threads busy, between one and two cores of the sixteen, for the audio stage alone.

### The GPU

The shipped graph fails to load on DirectML under both 1.20.1 and 1.24.4: its 49 `Reshape` nodes carry `allowzero=1`, which the DirectML provider rejects (`The parameter is incorrect`, from `MLOperatorAuthorImpl.cpp`). A re-export with the input fixed at 16000 samples and `allowzero` patched to 0 on every `Reshape` does load (`--model` points the bench at it, `--ep directml` selects the provider), at these per-run costs over the 1 s window:

| Runtime | Provider | English p50 / p95 | Japanese p50 / p95 |
|---|---|---|---|
| 1.20.1 | CPU, 4 threads | 30 to 50 / 53 ms | 30 to 50 / 53 ms |
| 1.20.1 | DirectML | 62 / 79 ms | 57 / 78 ms |
| 1.24.4 | CPU, 4 threads | 29 to 48 / 52 ms | 29 to 48 / 52 ms |
| 1.24.4 | DirectML | 64 / 71 ms | 64 / 74 ms |

The GPU run is slower than the CPU run on the shipped runtime, and it would share the single DirectML queue with the hand and body models, hold another 0.4 GB of VRAM, need the patched fixed-shape export as a setup step, and lose the whole-clip reference run (a fixed input cannot take a 25 s clip). The graph's cost is the wav2vec2 encoder's convolution stack over the raw samples, which DirectML does not run well at this size. The same python runtime (1.24.4, the `onnxruntime-directml` wheel) ran the patched graph in 6 ms on the GPU, so a future runtime or a graph exported for the GPU could change this picture, but nothing in the shipped runtime does. The CPU provider stays.

### Against the Oculus LipSync SDK

The Oculus LipSync native SDK (1.61.0, the Enhanced provider, closed binary under the Oculus SDK license) was run over the same two clips from a small timing harness outside the tree, feeding 10 ms and 1024-sample buffers through `ovrLipSync_ProcessFrame`:

| | wav2arkit, 1 s window, 4 threads | Oculus LipSync, Enhanced |
|---|---|---|
| CPU per second of audio | 300 to 500 ms across four threads | 9 ms on one thread |
| Output | 29 ARKit mouth columns at 30 fps | 15 visemes per buffer, a fixed table to ARKit |
| Added latency | 100 ms hop, 67 ms lookahead, 30 to 50 ms run | one buffer, frameDelay 0 |
| Lag against the audio envelope | about 100 ms | about 100 ms |
| License | Apache 2.0, redistributable | closed DLL, the Oculus SDK license |

The two systems' mouth openness agree at a correlation of 0.48 (English) and 0.54 (Japanese) between wav2arkit's `mouthLowerDown` and the SDK's `aa` viseme, and each lags the clip's energy envelope by about the same 100 ms. The SDK is the cheaper one by a factor of about fifty on CPU; the difference it cannot close is the output, since a viseme table drives a few columns at once where wav2arkit drives all 29, and its license, which the mfcc backend (open, the same 15-viseme output shape) answers in this design.

### The mfcc backend

The same bench on the mfcc backend, 1 s window, 100 ms hop:

| | English | Japanese |
|---|---|---|
| Run per 1 s window, p50 / p95 | 0.60 / 0.68 ms | 0.57 / 0.62 ms |
| Share of real time | 0.006 | 0.006 |
| Chunked drift from the whole-clip run, over activity | 0.0080 over 0.021 | 0.0073 over 0.023 |
| Worst column | `mouthPucker` 0.035 | `mouthPucker` 0.041 |
| Hops called silence | 0.16 | 0.16 |
| Most frequent visemes | I 0.25, U 0.11, RR 0.10 | I 0.33, RR 0.25, CH 0.06 |

The cost is a hundredth of wav2arkit's. The drift does not come from context (the lookahead changes it by nothing) but from the block grid: a 1600-sample hop shifts the 256-sample blocks by 64 samples per run against the whole-clip grid, and a vote that flips on a boundary moves a whole viseme. Against the other systems, the mfcc jaw agrees with wav2arkit's `mouthLowerDown` at a correlation of 0.31 (English) and 0.37 (Japanese) at a one-frame lag, with the Oculus SDK's `aa` at 0.25 and 0.06, and with the clip's energy envelope at 0.32 and 0.35 once shifted three frames, which is the 100 ms its gate and vote and crossfade add. Where the envelope is quiet the mfcc jaw sits at 0.05 against 0.11 overall, while wav2arkit's `mouthLowerDown` stays at 0.18 against 0.30: the gate shuts the mouth in pauses that wav2arkit leaves half open. `--render-mouth` on the English run shows closed lips in the pauses and the vowel and pucker shapes on the sentences.

What the model drives: `mouthLowerDownLeft` and `Right` carry the mouth (mean 0.32, p95 0.52, max 0.74), with `mouthUpperUp` (p95 0.24), `mouthShrugUpper` (p95 0.22), `mouthPucker` (p95 0.15), and `mouthStretch` (p95 0.11) behind them. `jawOpen` stays small: mean 0.065, p95 0.14, max 0.19 on both clips. The LAM decoder's blendshapes were adapted to the FLAME face, and the adaptation opens the mouth through the lips rather than the jaw. The face map derives a VRM avatar's `aa` preset from `jawOpen` alone, so on a preset avatar (the VRoid samples) the audio mouth barely opens, as the contact sheets show, while a perfect-sync avatar with its own `mouthLowerDown` shapes gets the model's full motion. Where to close that gap (a jaw gain in the stage, `mouthLowerDown` feeding `aa` in the face map, or both) is an open question in `docs/plan.md`.
