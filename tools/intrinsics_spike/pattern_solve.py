"""Solves a camera's focal length from a probe recording of a pattern, the way the intrinsics wizard does.

    pattern_solve.py <dir> --pattern charuco|chessboard|circles --detector NAME --reference-fx FX ...
                     [--dictionary 6x6|4x4] [--views N] [--trials M] [--separation PX] [--steady PX]

Keeps the frames where the whole pattern was found and the pattern moved since the last kept frame, then
solves focal length and two radial terms from all of them and from random sets of the wizard's view count.
`--steady` also drops a frame whose pattern moved more than that many pixels since the frame before.
"""

import argparse
import random
import statistics

import cv2
import numpy as np

import pattern_probe

# The app's intrinsics solve: one focal length, principal point at the image centre, k1 and k2 only
SOLVE_FLAGS = (cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_ZERO_TANGENT_DIST
               | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6)


def solve(views, image_size):
    image_points = [view[0].astype(np.float32) for view in views]
    object_points = [view[1].astype(np.float32).reshape(-1, 3) for view in views]
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(object_points, image_points, image_size, None, None,
                                                        flags=SOLVE_FLAGS)
    return float(matrix[0, 0]), float(distortion.flatten()[0]), float(distortion.flatten()[1]), float(rms)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dir")
    parser.add_argument("--pattern", choices=["charuco", "chessboard", "circles"], required=True)
    parser.add_argument("--detector", required=True)
    parser.add_argument("--dictionary", default="6x6")
    parser.add_argument("--reference-fx", type=float, action="append", required=True)
    parser.add_argument("--views", type=int, default=12)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--separation", type=float, default=10.0)
    parser.add_argument("--steady", type=float, default=0.0)
    args = parser.parse_args()

    detect = pattern_probe.pattern_variants(args.pattern, args.dictionary)[args.detector]
    _, frames = pattern_probe.load_frames(args.dir)
    image_size = (frames[0].shape[1], frames[0].shape[0])

    views = []
    previous = None
    for gray in frames:
        detection = detect(gray)
        # A pattern that moved since the frame before is a blurred one
        moved = None
        if detection is not None and previous is not None:
            moved = float(np.mean(np.linalg.norm(detection[0] - previous[0], axis=1)))
        previous = detection
        if detection is None:
            continue
        if args.steady > 0.0 and (moved is None or moved > args.steady):
            continue
        if views and np.mean(np.linalg.norm(detection[0] - views[-1][0], axis=1)) < args.separation:
            continue
        views.append(detection)

    reference = statistics.mean(args.reference_fx)
    fx, k1, k2, rms = solve(views, image_size)
    print(f"{args.dir}  {args.detector}: {len(views)} distinct views of {len(frames)} frames at {image_size}")
    print(f"  all views: fx {fx:.1f} ({100.0 * (fx / reference - 1.0):+.1f}%)  k1 {k1:.3f}  k2 {k2:.3f}  rms {rms:.2f} px")

    if len(views) > args.views:
        generator = random.Random(0)
        results = [solve(generator.sample(views, args.views), image_size) for _ in range(args.trials)]
        errors = sorted(100.0 * (result[0] / reference - 1.0) for result in results)
        print(f"  {args.trials} sets of {args.views} views: fx error median {statistics.median(errors):+.1f}%  "
              f"range {errors[0]:+.1f} to {errors[-1]:+.1f}%  rms median {statistics.median(r[3] for r in results):.2f} px")


if __name__ == "__main__":
    main()
