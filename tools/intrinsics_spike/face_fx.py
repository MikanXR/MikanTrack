"""Focal length from the recorded face points: a generic rigid face seen over a
head turn, solved as a camera calibration with the face as the target.

  python face_fx.py <recording.jsonl>... [--camera N] [--views M]

The whole-body model's 68 face points are recorded every frame in undistorted
pixels, so the reference is the header's undistorted camera matrix. Six of them
are fitted against a generic face (arbitrary units, focal length does not
depend on scale) with the principal point at the image center and no
distortion. The views are picked to spread over the head's yaw, read from where
the nose sits between the eye corners. The solve is run from several starting
focal lengths, since a non-planar target needs one.
"""

import argparse
import json

import cv2
import numpy as np

from spike_common import camera_intrinsics, fx_from_hfov, matrix_from_config, read_header

# iBUG index and position: x to the image's right, y down, z away from the viewer
SIX_POINT_FACE = [
    (30, (0.0, 0.0, 0.0)),
    (8, (0.0, 330.0, 65.0)),
    (36, (-225.0, -170.0, 135.0)),
    (45, (225.0, -170.0, 135.0)),
    (48, (-150.0, 150.0, 125.0)),
    (54, (150.0, 150.0, 125.0)),
]
MIN_SCORE = 0.5
START_HFOV_DEGREES = [50.0, 60.0, 70.0, 80.0, 90.0]


def load_face_views(recording, camera):
    views = []
    with open(recording, encoding="utf-8") as f:
        f.readline()
        for line in f:
            frame = json.loads(line)
            if frame.get("type") != "frame":
                continue
            for fresh in frame.get("fresh", []):
                if fresh.get("cam") != camera:
                    continue
                face = (fresh.get("body") or {}).get("wholeBody", {}).get("face")
                if not face or len(face) < 68:
                    continue
                points = np.array([face[index] for index, _ in SIX_POINT_FACE], dtype=np.float64)
                if points[:, 3].min() < MIN_SCORE:
                    continue
                eye_span = points[3, 0] - points[2, 0]
                if abs(eye_span) < 1.0:
                    continue
                yaw_proxy = (points[0, 0] - 0.5 * (points[2, 0] + points[3, 0])) / eye_span
                views.append((yaw_proxy, points[:, :2].astype(np.float32)))
    return views


def spread_views(views, count):
    """One view per yaw bin, so a take that mostly faces the camera does not drown its turns."""
    views = sorted(views, key=lambda view: view[0])
    proxies = np.array([view[0] for view in views])
    edges = np.linspace(proxies[0], proxies[-1], count + 1)
    picked = []
    for low, high in zip(edges[:-1], edges[1:]):
        inside = np.where((proxies >= low) & (proxies <= high))[0]
        if len(inside):
            picked.append(views[inside[len(inside) // 2]])
    return picked


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--views", type=int, default=40)
    args = parser.parse_args()

    object_points = np.array([position for _, position in SIX_POINT_FACE], dtype=np.float32)
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO |
             cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    for index, recording in enumerate(args.recordings):
        intrinsics = camera_intrinsics(read_header(recording), args.camera)
        width, height = int(intrinsics["width"]), int(intrinsics["height"])
        truth_fx = matrix_from_config(intrinsics["undistortedCameraMatrix"])[0, 0]
        views = load_face_views(recording, args.camera)
        if len(views) < 10:
            print(f"take {index}: {len(views)} face frames, too few")
            continue
        picked = spread_views(views, args.views)
        proxies = [view[0] for view in picked]
        print(f"take {index}: {len(views)} face frames, {len(picked)} views, nose offset "
              f"{min(proxies):+.2f} to {max(proxies):+.2f} eye spans, charuco fx {truth_fx:.1f}")
        for hfov in START_HFOV_DEGREES:
            start = fx_from_hfov(hfov, width)
            matrix = np.array([[start, 0, 0.5 * (width - 1)], [0, start, 0.5 * (height - 1)], [0, 0, 1]])
            rms, matrix, _, _, _ = cv2.calibrateCamera([object_points] * len(picked), [view[1] for view in picked],
                                                       (width, height), matrix, np.zeros(5), flags=flags)
            error = 100.0 * (matrix[0, 0] - truth_fx) / truth_fx
            print(f"    start {start:7.1f} px -> fx {matrix[0, 0]:8.1f} px ({error:+6.1f} %), rms {rms:.2f} px")


if __name__ == "__main__":
    main()
