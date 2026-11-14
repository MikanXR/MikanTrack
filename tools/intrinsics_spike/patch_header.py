"""Copy a recording with its header's calibration changed, for a replay what-if.

  python patch_header.py <recording.jsonl> <out.jsonl> [--camera N] [--fx-scale S]
      [--tilt-delta DEG] [--raw-frames DIR]

--fx-scale multiplies the camera's focal lengths. --tilt-delta adds to
direct.cameraTiltDegrees. --raw-frames names a folder of raw-equivalent frames
(redistort_frames.py): the copy then reads those frames, its header carries no
distortion and the distorted pinhole, and the recorded hand pixels are moved
into the raw image, which is what a camera with no undistortion would record.
Without --raw-frames the copy shares the original's frames through a junction.
"""

import argparse
import json
import os

import cv2
import numpy as np

from spike_common import (frames_directory, hfov_degrees, link_directory, matrix_from_config,
                          matrix_to_config)


def distort_pixels(points_xy, undistorted_matrix, distorted_matrix, distortion):
    """Undistorted-image pixels to the raw image's pixels."""
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    normalized = np.empty((len(points), 3))
    normalized[:, 0] = (points[:, 0] - undistorted_matrix[0, 2]) / undistorted_matrix[0, 0]
    normalized[:, 1] = (points[:, 1] - undistorted_matrix[1, 2]) / undistorted_matrix[1, 1]
    normalized[:, 2] = 1.0
    projected, _ = cv2.projectPoints(normalized, np.zeros(3), np.zeros(3), distorted_matrix, distortion)
    return projected.reshape(-1, 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("recording")
    parser.add_argument("out")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--fx-scale", type=float, default=1.0)
    parser.add_argument("--tilt-delta", type=float, default=0.0)
    parser.add_argument("--raw-frames")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.recording, encoding="utf-8") as source:
        lines = source.readlines()
    header = json.loads(lines[0])
    config = header["appConfig"]
    intrinsics = config["cameras"][args.camera]["intrinsics"]
    undistorted = matrix_from_config(intrinsics["undistortedCameraMatrix"])
    distorted = matrix_from_config(intrinsics["distortedCameraMatrix"])
    distortion = np.array(intrinsics["distortion"], dtype=np.float64)

    if args.raw_frames:
        for index in range(1, len(lines)):
            frame = json.loads(lines[index])
            if frame.get("type") != "frame":
                continue
            for fresh in frame.get("fresh", []):
                if fresh.get("cam") != args.camera:
                    continue
                for hand in fresh.get("hands", []):
                    points = hand.get("imagePoints")
                    if not points:
                        continue
                    moved = distort_pixels([p[:2] for p in points], undistorted, distorted, distortion)
                    for point, xy in zip(points, moved):
                        point[0], point[1] = float(xy[0]), float(xy[1])
            lines[index] = json.dumps(frame, separators=(",", ":")) + "\n"
        intrinsics["distortion"] = [0.0] * len(intrinsics["distortion"])
        undistorted = distorted.copy()

    for matrix in (undistorted, distorted):
        matrix[0, 0] *= args.fx_scale
        matrix[1, 1] *= args.fx_scale
    intrinsics["undistortedCameraMatrix"] = matrix_to_config(undistorted)
    intrinsics["distortedCameraMatrix"] = matrix_to_config(distorted)
    intrinsics["hfov"] = hfov_degrees(distorted[0, 0], intrinsics["width"])
    intrinsics["vfov"] = hfov_degrees(distorted[1, 1], intrinsics["height"])

    if args.tilt_delta != 0.0:
        direct = config.setdefault("direct", {})
        direct["cameraTiltDegrees"] = float(direct.get("cameraTiltDegrees", 10.0)) + args.tilt_delta

    lines[0] = json.dumps(header, separators=(",", ":")) + "\n"
    with open(args.out, "w", encoding="utf-8", newline="\n") as out:
        out.writelines(lines)
    link_directory(frames_directory(args.out), args.raw_frames or frames_directory(args.recording))


if __name__ == "__main__":
    main()
