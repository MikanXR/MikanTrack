# Kalidokit A/B

Scores the real Kalidokit on a take's front-camera frames against the same stereo truth `--gate-direct` scores the direct retarget against. The measured result is in [LEARNINGS.md](../../LEARNINGS.md) section 35.

## Setup

MediaPipe's Holistic solution needs its own venv, on Python 3.11:

```bash
python -m venv venv
```

```bash
venv/Scripts/python -m pip install mediapipe==0.10.14 opencv-python numpy
```

Kalidokit is a separate checkout with its `dist` built. The scoring script runs on a Python with `numpy` and `scipy`.

## Run

- `venv/Scripts/python run_holistic.py <recording_frames_dir> <cameraIndex> <holistic.jsonl>`: Holistic over the camera's raw frames in order, with the settings of Kalidokit's `sample/3d/script.js`, writing the image and world pose landmarks and the hands per frame

- `node solve_kalidokit.js <kalidokit.umd.js> <holistic.jsonl> <rig.jsonl>`: `Kalidokit.Pose.solve` on each frame, writing the hips, spine and arm rotations

- `python score_ab.py <recording.jsonl> <frames.csv> <holistic.jsonl> <rig.jsonl> <cameraIndex>`: the shoulder-to-wrist reach of each candidate against the truth in the gate's CSV. Kalidokit's rotations are posed on a T-pose arm with the sample's dampeners and its 0.3 slerp per frame, and carried to the camera's frame through the sample's mirror. The script scores every sign flip and arm pairing of that mapping and prints the best four, as a check that the derived one leads
