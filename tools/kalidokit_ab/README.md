# Kalidokit side by side

A take rendered three ways in one row: the camera, MikanTrack's direct retarget, and Kalidokit, both on one VRM. Kalidokit is a separate checkout with its `dist` built, and the video script needs `opencv-python`, `numpy` and `ffmpeg` on the path.

- `MikanTrack.exe --render-direct <recording.jsonl> <mikan_dir> <cameraIndex> view=front`: MikanTrack's avatar per raw frame, from the same lens and distance as the Kalidokit page's camera

- `python render/serve_render.py <recording_frames_dir> <cameraIndex> <file.vrm> <kalidokit_dir> [port]`: serves `render/render.html`, which is Kalidokit's `sample/3d` page with the webcam replaced by the take's raw frames in order. Open `http://localhost:<port>/` and leave it until its status line reads done. It loads three.js, three-vrm, MediaPipe Holistic and Kalidokit from the CDNs the sample names, runs the browser's Holistic on every frame, poses the VRM with the sample's rigging (face, blink, arms, hands) and posts one image per frame. Its camera rides with the avatar's hips, which Kalidokit moves toward and away from the viewer

- `python render/make_video.py <recording_frames_dir> <cameraIndex> <out.mp4> <fps> <label>=<dir>[,mirror] ...`: tiles the camera and one panel per labelled folder with `ffmpeg`, up to three panels in a row and four in a square. Kalidokit's avatar mirrors the user, so the camera is flipped to read the same way round, and a `--render-direct` folder takes `,mirror` for the same reason. Several `--render-direct` folders (one per pair of plane fractions, say) compare solver settings on one take
