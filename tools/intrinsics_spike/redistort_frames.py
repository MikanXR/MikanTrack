"""Turn a recording's undistorted frames back into raw-equivalent frames.

  python redistort_frames.py <recording.jsonl> <out_dir> [--camera N] [--every K] [--limit M]

Recorded frames are undistorted with the header's calibration. A camera with no
calibration only has raw frames, so the estimators and the no-undistortion
replay run on these instead. Each raw pixel samples the undistorted frame where
the calibration maps it. File names are kept, so the folder stands in for the
recording's frame directory.
"""

import argparse
import glob
import os

import cv2
import numpy as np

from spike_common import camera_intrinsics, frames_directory, matrix_from_config, read_header


def raw_from_undistorted_maps(intrinsics):
    width, height = int(intrinsics["width"]), int(intrinsics["height"])
    distorted = matrix_from_config(intrinsics["distortedCameraMatrix"])
    undistorted = matrix_from_config(intrinsics["undistortedCameraMatrix"])
    distortion = np.array(intrinsics["distortion"], dtype=np.float64)
    grid_x, grid_y = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    raw_pixels = np.stack([grid_x.ravel(), grid_y.ravel()], axis=1).reshape(-1, 1, 2)
    source = cv2.undistortPoints(raw_pixels, distorted, distortion, P=undistorted).reshape(height, width, 2)
    return source[..., 0].astype(np.float32), source[..., 1].astype(np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("recording")
    parser.add_argument("out_dir")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--every", type=int, default=1)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    intrinsics = camera_intrinsics(read_header(args.recording), args.camera)
    map_x, map_y = raw_from_undistorted_maps(intrinsics)
    os.makedirs(args.out_dir, exist_ok=True)
    paths = sorted(glob.glob(os.path.join(frames_directory(args.recording), f"cam{args.camera}_*.jpg")))
    paths = paths[:: args.every]
    if args.limit > 0:
        step = max(1, len(paths) // args.limit)
        paths = paths[::step][: args.limit]
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        raw = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        cv2.imwrite(os.path.join(args.out_dir, os.path.basename(path)), raw, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(f"wrote {len(paths)} frames to {args.out_dir}")


if __name__ == "__main__":
    main()
