# Audio stage

The microphone mouth stage: the voice is classified into the 15 Oculus visemes, and the viseme weights drive the avatar's mouth through the same face slot, face map, and VMC output the phone face uses (see [face.md](./face.md)). This doc covers what exists: the frame and the viseme table, the backend, the rolling-window policy, the emission filter, the wav reader, the live stage and its thread, the config and the Audio panel, the calibration wizard, the recording sidecar and its replay, the bench, and what the bench measured. The design record, including the learned model that was evaluated and dropped, is `docs/proposals/viseme.md`.

## Mouth frames and the viseme table (`src/Audio/AudioTypes.h`, `VisemeTable`)

A `MouthFrame` is one 30 fps frame of the 15 viseme weights in 0..1, in `eViseme` order (`VisemeTypes.h`, HeadAudio's id order, which is not the Oculus SDK's: `aa`, `E`, `I`, `O`, `U`, `PP`, `SS`, `TH`, `DD`, `FF`, `kk`, `nn`, `RR`, `CH`, `sil`). The frame is what the backend emits, the filter smooths, the recording stores, and the vision thread publishes, and it reaches the avatar two ways at once:

- `TrackingFrameResult::FacePose` carries the frame as `visemes` with `visemesPresent` set, and the face map reads a VRM preset vowel (`aa`, `ih`, `ou`, `ee`, `oh`) straight from its viseme (`aa`, `I`, `U`, `E`, `O`), since the viseme is the vowel (see [avatar.md](./avatar.md)).
- `applyVisemeWeights` writes the frame through the viseme table onto the 52-wide ARKit face: each column the table drives is the clamped sum of its terms, and every other column is left as it was. That is what a perfect-sync avatar, an ARKit-named morph target, the raw face map, and a VMC receiver matching ARKit names read.

The viseme table (`VisemeTable.cpp`) is the fixed map from each viseme to the ARKit mouth columns it shapes: the jaw and lip shape of each sound, the vowels opening the jaw and shaping the lips (each leading with its defining column at 0.8 to 0.85, the jaw for `aa`, the smile for `I`, the stretch for `E`, the funnel for `O`, the pucker for `U`), the consonants the lip contact (`PP`), the lip under the teeth (`FF`), the tongue between them (`TH`), and the small openings of the rest. Silence drives nothing. The table only ever touches the 29 columns the voice can drive (the four jaw columns, the twenty-three mouth columns, `cheekPuff`, and `tongueOut`), which `isMouthArkitIndex` names and a self-test holds it to, so the phone's eyes and brows are never written by audio. `dominantViseme` names the heaviest viseme of a frame for the tools' labels.

The stage's constants: 16 kHz mono float audio in, 30 fps frames out, and `mouthFramesForSamples` giving the frame count the backend emits for a span of samples, `ceil(30 * samples / 16000)`.

## The backend (`MfccBackend`)

HeadAudio's method (MIT, Mika Suominen, https://github.com/met4citizen/HeadAudio), the reimplementation of uLipSync's, ported stage for stage so its shipped model reads the same vectors:

- `MfccFeatures`: pre-emphasis (0.97), 512-sample blocks at a 256-sample hop, a Hamming window, a radix-2 FFT, the power spectrum and its log energy, 40 triangular mel bands over 30..7800 Hz, a DCT keeping coefficients 1..12 under a sinusoidal lifter, then tanh compression. HeadAudio's polyphase downsampler has no counterpart, since the stage's input is already 16 kHz.
- `VadGate`: HeadAudio's energy gate, open after a hop above -40 dB and closed after one below -50 dB (the thresholds and the hop counts are its config). A gated hop is silence and feeds no vote.
- `VisemeModel`: the binary model, a flat run of 368-byte records (an 8-byte header with the phoneme packed as two 16-bit code points, the group, and the viseme id, then the 12-float mean and the 78-float packed lower triangle of the inverse covariance). `VisemeClassifier` takes the nearest prototype by squared Mahalanobis distance, silence prototypes at their distance over the sensitivity (1.2), and votes over a ring of the last six hops, the most frequent viseme winning and ties going to the highest id.
- the crossfade: the winning viseme's weight ramps toward one and every other toward zero over 100 ms, through HeadAudio's sigmoid easing, one step per hop.

The backend turns a window of 16 kHz samples into the frames covering it, first to last, each 30 fps frame taking the weights of the latest hop centered at or before its center. A run is pure over its window: the same samples give the same frames, and the gate, the vote, and the crossfade restart from silence at the window's start, so a window's first hundred milliseconds are warm-up and the emitted tail is not. Which frames are new is the window policy's business, and smoothing happens after emission, so the bench can compare chunked and whole-clip output with nothing in between. A backend that cannot run (model file missing, profile unreadable) reports not ready with a reason string the panel shows, and `getLastInferMs` is the per-run cost.

The shipped model is `resources/audio/model-en-mixed.bin`, HeadAudio's own (39 prototypes, four Kokoro voices over Harvard Sentences), committed with the tree since it is 14 kB. Its silence prototype is the zero vector under a very tight covariance, which only digital silence reaches, so the gate is what calls a pause silence. A calibration profile (below) is a model file in the same format and loads through the same path.

## The window policy (`MouthWindow`)

Pure accounting with no backend and no clock. Captured samples are pushed in; once a hop's worth has arrived a run is due, `beginRun` hands out the window ending at that hop boundary (zero-padded at the front until enough audio has arrived) and returns how many of the backend's frames for it are new, and `takeTail` copies those frames off the end of the backend's output. Frames are due at `floor(30 * consumedSamples / 16000)`, so a 1600-sample hop is always three new frames and an odd hop size alternates counts without skipping or repeating a frame.

The config:

- `windowSamples`: the span the backend sees per run, 1 s by default
- `hopSamples`: the new audio between runs, 100 ms
- `lookaheadFrames`: frames held back from the tail of each run, so an emitted frame has that much audio after it, two by default. Each frame costs 33 ms of latency. The emitted count trails the frames due by the lookahead, clamped so the first runs never emit the padding before the clip's own first frame.

## The emission filter (`MouthFilter`)

A one-euro filter per viseme over the emitted 30 fps frames (`OneEuroFilter`, the hand filter). The backend's crossfade already ramps between the hops' votes inside a run; the filter smooths the frames across runs and eases the mouth shut when the gate closes instead of snapping it. Defaults: a 3 Hz minimum cutoff, beta 0.5, a 1 Hz derivative cutoff. The bench writes the filtered series beside the raw one, so the filter is judged on the avatar rather than assumed.

## The wav reader (`WavFile`)

RIFF wave reading for the bench clips and the recording sidecars: 8, 16, 24, and 32-bit PCM and 32-bit float, plain or `WAVE_FORMAT_EXTENSIBLE`, any channel count, chunks walked by size so a `LIST` chunk before the data is skipped. `toMono` averages the channels, `resample` goes through SDL's `SDL_AudioCVT` converter (no audio subsystem init needed), and `readMono16k` is the three in one.

## The live stage (`AudioService`)

`AudioService` (`src/Audio/AudioService.h`/`.cpp`) is the microphone in the app: SDL2 capture, an audio thread of its own, the backend, the chain from samples to frames, and a latest-wins `MouthSample` the vision thread reads. The chain is `MouthChain` (`src/Audio/MouthChain.h`/`.cpp`): the gain, the window policy above, the hop's level through the gate, the backend over the window, the tail frames through the filter, each emitted frame carrying its index in the capture stream. The live service and the offline replay below both run this one class, so a recording replays through exactly what live ran. `App` owns it like `FaceService`: created in `App::startup`, `startup()` on project activation (which initializes SDL's audio subsystem and spawns the thread), `shutdown()` wherever the face service shuts down. `App::applyAudioConfig()` maps `AppConfig::audio` to `AudioServiceConfig` and calls `setConfig`, which parks the config for the audio thread to adopt between blocks: an enable or device change reopens the device (which restarts the stream and its frame indices), a profile change recreates the backend under the running stream (the window keeps its samples and the indices keep counting, so a recording stays tied to its sidecar across the change), and the gain and gate apply at once.

The device is opened at 16 kHz mono float in 256-sample blocks with no allowed changes, so SDL converts whatever the hardware gives and the window only ever sees the backend's rate. The configured device name is used while a device still carries it and falls back to the system default with a warning, since a USB microphone comes and goes. SDL's callback thread copies each block into a 64-block SPSC queue (a second of audio) and the audio thread drains it. A block the queue cannot take is dropped and counted (`droppedBlocks` on the status), never waited for, so a stalled backend cannot back up the capture.

Per block the audio thread pushes the samples through the chain, which applies the gain (dB to linear), feeds `MouthWindow`, and runs the window policy's due runs. Each run measures the hop's RMS level in dBFS, feeds it to the gate, and either runs the backend over the window (gate open and backend ready) or emits the neutral mouth for the run's frames. Both go through `MouthFilter`, so a gate closing eases the mouth shut rather than snapping it. The gate (`LevelGate`) opens at the configured threshold and closes 6 dB under it, which holds it through the dips inside a word. The backend's run time is tracked as an EMA for the panel.

The backend produces its frames in bursts, three per 100 ms hop, and a run holds the audio thread for longer than a frame, so the pacing lives with the readers rather than the thread: the bursts go into `MouthPacer` (a jitter buffer, six frames deep), and `getLatestSample` and `getStatus` draw it down one frame per 33.3 ms on the steady clock whoever calls them. A run that lands late only deepens the buffer by its delay, and a reader that polls late takes the newest due frame and drops the ones it supersedes. The vision thread therefore sees a frame per 30 Hz tick with the sequence stepping by one, and a mouth-only output frame goes out per frame rather than three at once. The latency from voice to published frame is the hop (100 ms) plus the lookahead (two frames, 67 ms) plus the backend's run (under a millisecond) plus up to one pacing interval.

`--audio-probe [seconds] [--device name] [--gate dbfs] [--gain db]` is the live stage without the UI: it lists the capture devices, runs the service on one for a few seconds, reports the level, the gate, the backend's run time, the frames per second, and the dominant viseme of each polled frame, and passes when the device opened, the backend is ready, and frames arrived. It tells a microphone problem from a model problem before the panel is involved, and a `--gate` under the room's floor makes it exercise the backend in a quiet room.

## Config and the panel

`AudioConfig` on `AppConfig::audio`, persisted under `audio` in the project JSON:

- `bool enabled` (default false)
- `std::string deviceName` (empty for the system default)
- `float gainDb` (default 0)
- `float gateDbfs` (default -45)
- `std::string profilePath` (empty for the shipped model, else the calibration wizard's profile below)

The Audio panel (`src/UI/AudioPanel.h`/`.cpp`, `View > Audio Panel`, docked in the left column) holds the enable checkbox, the microphone combo (the device list is fetched on the first draw and on Refresh, since enumeration is not free), a level meter over the newest hop that turns green while the gate is open, the gate and gain sliders, and the profile line with the Calibrate button (the wizard below) and, once a profile is set, the button back to the shipped model. Every edit marks the config dirty and goes through `App::applyAudioConfig`. The status block below names the device and its rate, the model with its run time or the reason it is unavailable, which source owns the mouth (the phone face, the microphone, gated, or off), and the dropped block count once any dropped. The merge rule that decides the owner is in [face.md](./face.md). The block at the bottom is the 15 viseme weights as labelled bars, read from the frame the scene is showing: the live output while the microphone owns the mouth, or the recorded frame while the timeline replay view is active. It is the readout that says whether the voice is producing clear vowel calls before the avatar is involved.

What those weights become on the loaded avatar is the OSC Output panel's "Avatar mouth expressions" section (`src/UI/AvatarMouthReadout.h`/`.cpp`, collapsed by default): the avatar's mouth expressions as its face map derives them from the live output face, the vowel presets of a preset avatar read from the visemes, the mouth-named expressions of a perfect-sync avatar from the ARKit columns, with a line saying which source the frame carried. It is where a shape that looks wrong on the avatar is told apart from a weight that never arrived.

## Calibration (`AudioCalibrationWizard`)

The shipped model was fit on synthesized voices, and a vowel prototype is where a voice and a microphone show most, so the microphone calibration wizard (`src/UI/AudioCalibrationWizard.h`/`.cpp`, `eWizardKind::AudioCalibration` on the `WizardHost`, the Calibrate button in the Audio panel's profile block) refits the five vowel visemes to the user and leaves the rest of the model alone. It needs no tracking, only the audio service, and the button is enabled while the stage has a device open and no other wizard runs.

Six prompts of three seconds each, every one behind the three-second countdown the other wizards use, the vowel held steadily after it: the visemes `aa`, `I`, `U`, `E`, `O` as the vowels of father, sit, boot, bed, boat (the Japanese walkthrough prompts the five Japanese vowels in the same order, whose shapes land on the same visemes), then the room's silence. A prompt collects the raw stream from the service (`AudioService::startCapture`/`stopCapture`, the audio thread appending each block to a vector reserved up front, dropped by a device reopen) and runs it through the fit's front end, `extractSpeechVectors` in `src/Audio/VisemeFit.h`/`.cpp`: the backend's own chain (pre-emphasis, the extractor, the tanh compression) with HeadAudio's energy gate restarted at the clip's start, so a fit reads exactly the vectors the classifier will see. A vowel prompt that leaves fewer than thirteen voiced hops (a fifth of a second, the rank the covariance needs) is refused with the reason and recorded again. The silence prompt suggests the gate: its loudest hop in dBFS, after the configured gain, plus 6 dB, clamped to the slider's range.

The fit (`fitVisemePrototype`) is one Gaussian per vowel: the mean of its vectors and the inverse of their sample covariance, with 0.25 added to every diagonal term before the inversion (about a third of the shipped prototypes' typical per-coefficient variance of 0.7). The floor is what keeps a vowel held too steadily from getting a prototype so tight that the same vowel in running speech misses it, and it is also what makes the covariance invert on a short capture. `overlayVisemePrototypes` builds the profile from the shipped model: every shipped prototype of a fitted viseme is dropped (the shipped model carries nineteen across the five vowels, several phonemes per viseme), the consonants and the silence prototype are kept, and the five fits are appended with group byte 1, which tells them from the shipped records in a dump. The profile is a complete model file in the shipped format (`VisemeModel::save`, the record encoder the inverse of the decoder, a phoneme past two code points cut to two), so the backend loads it through the one path it has.

The review classifies each vowel's own vectors against the new profile by nearest prototype and shows the share called its viseme, green from 80 percent: a lower share means two prompts overlapped (an `oo` drifting toward `oh` is the usual one), and Redo records the six again. Finish writes `<project>/audio/profile.bin`, sets `AudioConfig::profilePath` to it and `gateDbfs` to the suggestion, marks the config dirty, and applies it, which recreates the backend on the new file. The panel's "Use shipped model" button beside Calibrate clears the path again. `--audio-probe` runs the capture tap for a second at the end of its run and reports the samples it collected, the hops that passed the energy gate, and the gate the clip suggests, the data path a prompt takes without the UI.

## Recording and replay

A tracking recording made while the stage is enabled carries the microphone with it. `AudioService::startRecording` writes the captured stream, before the gain and the gate, as 16 kHz mono 16-bit PCM to `<recording>.wav` beside the JSONL (`TrackingRecording::makeAudioSidecarPath`), through `AudioRecorder` (`src/Audio/AudioRecorder.h`/`.cpp`): the audio thread hands each raw block to a bounded queue and a writer thread turns them into the file through `WavWriter`. A queue overflow neither aborts nor shifts the file: the dropped blocks are written as silence in their place when the queue has room again, and counted, so a sample's index in the file stays its index in the stream. The wav header's sizes are placeholders until the file closes, set to the maximum, so a file cut short by a crash still reads to its end.

The recording ties its frames to the audio by index rather than by clock. Every emitted frame carries its index in the capture stream (frame i scores the audio around i/30 s after the device opened), the sidecar's header block records the stream index of its first sample, and each recorded iteration carries the newest mouth frame with that index:

- the header's `audio` block: the file name (resolved beside the recording, so a moved folder still works), the sample rate, `startSample`, and the start time on the steady clock. The profile the run used is in the header's config snapshot (`audio.profilePath`).
- each frame's `mouth` block: the 15 viseme weights as the stage emitted them and `frameIndex`, which is -1 when the sidecar does not hold that frame's audio (the device reopened mid-recording, which ends the sidecar and restarts the indices). A `mouth` block of another width is from before the frame carried visemes and is skipped on load.

The vision thread starts the sidecar before it writes the JSONL header, since the header carries where the sidecar begins, and ends it with the recording. The Timeline panel's recording status shows the sidecar's length while it is written. During playback every timeline view (recorded, replayed, what-if) shows the frame's recorded visemes the way the live merge handed them on, over a neutral face through the viseme table and as the visemes themselves, since the audio stage is not re-run by the timeline, and the Audio panel's weights block reads the same frame. A recording made with the stage off has no sidecar and no mouth blocks, and an older recording without either loads as before. The mouth weights sit outside the checksum, like the IMU output, which is what lets a model change be measured against old recordings.

`--replay-audio <recording.jsonl> [out.json] [--model path] [--gain db] [--gate dbfs]` re-runs the chain over the sidecar and measures it against the recorded frames. The chain runs at every hop boundary counted from the device's first sample, while a sidecar starts somewhere inside a hop, so the tool pads the sidecar with `startSample mod hop` samples of silence to put its hop boundaries where live had them, after which a replayed frame j is the live frame j plus the frames due at that boundary (`mouthReplayAlignment` in `src/Audio/MouthReplay.h`). The recorded profile (or the shipped model) under the recorded gain and gate then reproduces the live frames to float precision once the window has filled: the first window's frames replay from padding where live had the stream's history, so they are measured apart as the warm-up. The report is the per-viseme mean absolute difference against the recorded frames beside each viseme's recorded activity, the worst viseme, the run p50 and p95, and optionally a JSON in the bench's shape with the `recorded` and `replayed` series, which `--render-mouth` renders. Another profile, gain, or gate replays the same voice under the change, the A/B the sidecar exists for.

## The bench (`--viseme-bench`)

```bash
cd build/RelWithDebInfo && ./MikanTrack.exe --viseme-bench resources/audio/bench/bench_en.wav bench_en.json --window 1000 --lookahead 2
```

Runs the backend (`--model` pointing it at a profile instead of the shipped model) once over the whole clip (the reference), then feeds the clip in hops through the window policy the way capture will, and reports:

- the share of the clip's hops each viseme won, the readout that says whether the classifier calls anything but silence
- the chunked frames' mean absolute drift from the whole-clip run, per viseme and overall, beside each viseme's mean activity so the drift reads relative to motion
- the per-run inference p50, p95, and max against the hop, as `LATENCY PASS` or `FAIL`, and the share of real time the runs took
- the `offline`, `chunked`, and `filtered` frame series as JSON, for `--render-mouth`

`--render-mouth <file.vrm> <bench.json> [out.png] [tiles] [series]` renders a contact sheet of an avatar's face under the frames through the scene renderer and the face map from a hidden window, the visemes applied the way the live merge applies them (through the viseme table onto a neutral face, and as the visemes to the face map), tiles spread evenly over the clip and labelled with the frame index and the dominant viseme with its weight. It is the headless way to see what the weights do to a real mouth, on a preset avatar and on a perfect-sync one. `--test-audio` covers the pieces above on synthetic input.

The standing inputs are two Kokoro-synthesized clips, English (25 s, Harvard Sentences) and Japanese (21 s), fetched into `resources/audio/bench/` by the setup script.

## What the bench measured

The shipped model on this development machine, 1 s window, 100 ms hop, two frames of lookahead:

| | English | Japanese |
|---|---|---|
| Run per 1 s window, p50 / p95 | 0.56 / 0.58 ms | 0.57 / 0.57 ms |
| Share of real time | 0.006 | 0.006 |
| Chunked drift from the whole-clip run, over activity | 0.020 over 0.043 | 0.018 over 0.047 |
| Worst viseme | `I` 0.072 (activity 0.23) | `I` 0.085 (activity 0.32) |
| Hops called silence | 0.16 | 0.16 |
| Most frequent visemes | `I` 0.25, `U` 0.11, `RR` 0.10 | `I` 0.33, `RR` 0.25, `CH` 0.06 |

The run cost leaves the hop almost entirely free, so the latency is the hop plus the lookahead plus the gate, vote, and crossfade inside the model (about 100 ms), and the stage costs well under a percent of a core. The drift does not come from context (the lookahead changes it by nothing) but from the block grid: a 1600-sample hop shifts the 256-sample blocks by 64 samples per run against the whole-clip grid, and a vote that flips on a boundary moves a whole viseme, which is why the worst viseme is the most frequent one. Against the clip's energy envelope the mouth opens with a lag of about 100 ms, the gate and the vote and the crossfade, and the gate shuts the mouth in the pauses. The contact sheets on the VRoid samples show closed lips in the pauses and the vowel shapes on the sentences, `O` and `aa` opening the preset mouth fully through the direct viseme path and `U` puckering it.

The shipped model calls `I` far more often than the other vowels on both clips, and the calibration wizard is the answer to that for a given voice: the review percentages after a fit say how well the five vowels separate, and `--replay-audio --model` replays an old recording under the new profile.
