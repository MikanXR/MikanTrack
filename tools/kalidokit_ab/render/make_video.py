"""Tile a take's raw frames, the --render-direct images and the Kalidokit page's images into one video.

  python make_video.py <recording_frames_dir> <cameraIndex> <mikan_dir> <kalidokit_dir> <out.mp4> [fps]

One row of three panels per raw frame that all three have: the camera, MikanTrack's direct retarget, and
Kalidokit. Kalidokit's avatar is a mirror of the user, so the camera and the MikanTrack panels are flipped
to read the same way round. Needs ffmpeg on the path.
"""

import glob
import os
import re
import subprocess
import sys

import cv2
import numpy as np

frames_dir, camera, mikan_dir, kalidokit_dir, out_path = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4], sys.argv[5]
fps = float(sys.argv[6]) if len(sys.argv) > 6 else 30.0
PANEL = (854, 480)


def indices(pattern):
    return {int(re.search(r"_(\d+)\.jpg$", path).group(1)) for path in glob.glob(pattern)}


shared = sorted(indices(os.path.join(frames_dir, f"cam{camera}_*.jpg")) & indices(os.path.join(mikan_dir, "frame_*.jpg")) &
                indices(os.path.join(kalidokit_dir, "frame_*.jpg")))
if not shared:
    raise RuntimeError("no frame index is in all three folders")


def panel(path, label, mirror):
    image = cv2.resize(cv2.imread(path), PANEL, interpolation=cv2.INTER_AREA)
    if mirror:
        image = cv2.flip(image, 1)
    cv2.putText(image, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return image


width, height = PANEL[0] * 3, PANEL[1]
ffmpeg = subprocess.Popen(
    ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", str(fps),
     "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out_path], stdin=subprocess.PIPE)
for count, index in enumerate(shared):
    row = np.hstack([
        panel(os.path.join(frames_dir, f"cam{camera}_{index}.jpg"), f"Camera  frame {count}", True),
        panel(os.path.join(mikan_dir, f"frame_{index}.jpg"), "MikanTrack direct", True),
        panel(os.path.join(kalidokit_dir, f"frame_{index}.jpg"), "Kalidokit", False),
    ])
    ffmpeg.stdin.write(row.tobytes())
ffmpeg.stdin.close()
if ffmpeg.wait() != 0:
    raise RuntimeError("ffmpeg failed")
print(f"{len(shared)} frames at {fps:g} fps to {out_path}")
