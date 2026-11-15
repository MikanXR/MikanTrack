"""Tile a take's raw frames and rendered avatar sequences into one video.

  python make_video.py <recording_frames_dir> <cameraIndex> <out.mp4> <fps> <label>=<dir>[,mirror] ...

The camera is the first panel, then one panel per <label>=<dir> of frame_<frameIndex>.jpg images, as
--render-direct and the Kalidokit page write them. Up to three panels sit in one row and four in a square.
Kalidokit's avatar is a mirror of the user, so the camera is flipped to read the same way round, and so
is a panel given with ,mirror, which a --render-direct sequence needs. Only the frames every panel has
are written. Needs ffmpeg on the path.
"""

import glob
import os
import re
import subprocess
import sys

import cv2
import numpy as np

frames_dir, camera, out_path, fps = sys.argv[1], int(sys.argv[2]), sys.argv[3], float(sys.argv[4])
PANEL = (854, 480)


def indices(pattern):
    return {int(re.search(r"_(\d+)\.jpg$", path).group(1)) for path in glob.glob(pattern)}


panels = []
shared = indices(os.path.join(frames_dir, f"cam{camera}_*.jpg"))
for arg in sys.argv[5:]:
    label, _, rest = arg.partition("=")
    directory, _, flag = rest.partition(",")
    panels.append((label, directory, flag == "mirror"))
    shared &= indices(os.path.join(directory, "frame_*.jpg"))
shared = sorted(shared)
if not panels or not shared:
    raise RuntimeError("no panel given, or no frame index is in every folder")


def panel(path, label, mirror):
    image = cv2.resize(cv2.imread(path), PANEL, interpolation=cv2.INTER_AREA)
    if mirror:
        image = cv2.flip(image, 1)
    cv2.putText(image, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return image


columns = 2 if len(panels) == 3 else len(panels) + 1
rows = -(-(len(panels) + 1) // columns)
width, height = PANEL[0] * columns, PANEL[1] * rows
ffmpeg = subprocess.Popen(
    ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", str(fps),
     "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out_path], stdin=subprocess.PIPE)
for count, index in enumerate(shared):
    tiles = [panel(os.path.join(frames_dir, f"cam{camera}_{index}.jpg"), f"Camera  frame {count}", True)]
    tiles += [panel(os.path.join(directory, f"frame_{index}.jpg"), label, mirror) for label, directory, mirror in panels]
    while len(tiles) < columns * rows:
        tiles.append(np.zeros_like(tiles[0]))
    frame = np.vstack([np.hstack(tiles[row * columns:(row + 1) * columns]) for row in range(rows)])
    ffmpeg.stdin.write(frame.tobytes())
ffmpeg.stdin.close()
if ffmpeg.wait() != 0:
    raise RuntimeError("ffmpeg failed")
print(f"{len(shared)} frames at {fps:g} fps to {out_path}")
