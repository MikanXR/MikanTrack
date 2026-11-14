# Intrinsics spike

Offline scripts that measure what a one-camera direct project loses without a charuco calibration, and how close marker-free focal length estimates come to one. The measured verdicts are in [LEARNINGS.md](../../LEARNINGS.md). Nothing here runs in the app.

## Setup

The scripts run from this folder in a venv that sees a system Python 3.11 holding `torch`, `opencv-python`, `numpy` and `moge`:

```bash
python -m venv --system-site-packages .venv
```

```bash
.venv/Scripts/python -m pip install "git+https://github.com/cvg/GeoCalib@1dedb29f94fc11b896c5f572005978aeebccd67f"
```

GeoCalib (Apache-2.0) downloads its two checkpoints into the torch hub cache on first use. MoGe-2 loads `Ruicheng/moge-2-vitl-normal` from the Hugging Face cache.

## Scripts

- `patch_header.py <recording.jsonl> <out.jsonl> [--camera N] [--fx-scale S] [--tilt-delta DEG] [--raw-frames DIR]`: a copy of a recording with its header's calibration changed. The copy shares the original's frames through a directory junction.

- `redistort_frames.py <recording.jsonl> <out_dir> [--camera N] [--every K] [--limit M]`: the recording's undistorted frames warped back to raw-equivalent frames with the header's calibration. A camera with no calibration only has raw frames, so the estimators and the no-undistortion replay run on these.

- `sweep_sensitivity.py <MikanTrack.exe> <work_dir> <recording.jsonl>... [--camera N] [--fx-scales ...] [--tilt-deltas ...] [--no-undistort]`: replays each take through `--gate-direct` under each perturbed calibration and reports the rotation of the direct retarget's posed upper arm, forearm, hand and head against the unperturbed run, p50 and p90 in degrees. It reads the posed bone columns of the tool's CSV. A finished variant is kept under the work folder and not replayed again.

- `estimate_fx.py <recording.jsonl> <raw_frames_dir> [--camera N] [--samples M] [--methods nominal,moge,geocalib] [--label NAME]`: nominal fields of view, MoGe-2 and GeoCalib on the raw-equivalent frames against the header's charuco focal length, with GeoCalib's radial term and gravity.

- `face_fx.py <recording.jsonl>... [--camera N] [--views M]`: the focal length solved from the recorded face points over a head turn against a generic six-point face.

- `board_runs.py record <project.json> <runs.jsonl> <label> [--camera N]` and `board_runs.py report <runs.jsonl> --reference <project.json>[:camera] ...`: the intrinsics wizard overwrites a camera's calibration on accept, so `record` copies it out after each run and `report` tabulates the runs against reference calibrations. Runs whose labels differ only in a trailing number report as one group. This is the loop for comparing a board shown on a phone screen against a printed one.

- `phone_patterns.py <out_dir>`: the calibration patterns sized for a phone screen in landscape, one PNG each: the charuco board under two dictionaries, a plain chessboard, an asymmetric grid of white circles on black, and dimmed copies.

- `pattern_probe.py capture <out_dir> [--camera N] [--seconds S]` and `pattern_probe.py score <dir> --pattern charuco|chessboard|circles [--dictionary 6x6|4x4]`: `capture` records a webcam looking at a pattern, with the app closed so the camera is free. `score` runs each detector variant over the saved frames and reports the share of frames with the whole pattern found, the detection time, and the one second holds the intrinsics wizard's gate would have completed, as it ships and with a grace period for missed frames.

- `pattern_solve.py <dir> --pattern charuco|chessboard|circles --detector NAME --reference-fx FX ... [--views N] [--trials M] [--steady PX]`: the focal length solved from a probe recording under the app's solve model, from every distinct view and from random sets of the wizard's view count, against reference focal lengths. `--detector` takes a variant name from the `score` table.

## Checks

- A copy patched at `--fx-scale 1` replays to the same CSV as the original, byte for byte.
- Only one `--gate-direct` run at a time writes `gate-direct.log` beside the exe, so a second run started by hand during a sweep overwrites the sweep's log copy. The CSVs are unaffected.
