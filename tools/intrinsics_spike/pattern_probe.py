"""Records a webcam looking at a calibration pattern, then scores how reliably each detector finds it.

    pattern_probe.py capture <out_dir> [--camera N] [--seconds S] [--width W] [--height H]
    pattern_probe.py score <dir> --pattern charuco|chessboard|circles [--dictionary 6x6|4x4]

`capture` saves every frame with its time and shows a plain preview. It runs no detector, so the capture
rate stays the camera's. `score` runs each detector variant over the saved frames and reports the share
of frames with the whole pattern found, the detection time, and how many one second holds the intrinsics
wizard's gate would have completed, as it ships and with a grace period for missed frames.
"""

import argparse
import csv
import os
import statistics
import time

import cv2
import numpy as np

import phone_patterns

HOLD_SECONDS = 1.0
GRACE_SECONDS = 0.25


def capture(args):
    os.makedirs(args.out_dir, exist_ok=True)
    camera = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if not camera.isOpened():
        raise SystemExit(f"camera {args.camera} did not open (is another application holding it?)")
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print("Preview is up. Press space to start recording, Esc to quit.")
    recording_start = None
    rows = []
    while True:
        ok, frame = camera.read()
        if not ok:
            raise SystemExit("the camera stopped delivering frames")
        now = time.perf_counter()
        if recording_start is not None:
            elapsed = now - recording_start
            if elapsed >= args.seconds:
                break
            name = f"frame_{len(rows):05d}.jpg"
            cv2.imwrite(os.path.join(args.out_dir, name), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            rows.append((name, f"{elapsed:.4f}"))

        preview = frame.copy()
        text = "space: record" if recording_start is None else f"recording {args.seconds - (now - recording_start):.0f}s"
        cv2.putText(preview, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow("pattern probe", preview)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == 32 and recording_start is None:
            recording_start = time.perf_counter()

    camera.release()
    cv2.destroyAllWindows()
    with open(os.path.join(args.out_dir, "frames.csv"), "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows([("file", "seconds")] + rows)
    if rows:
        print(f"{len(rows)} frames at {frame.shape[1]}x{frame.shape[0]}, {len(rows) / float(rows[-1][1]):.1f} fps")


def charuco_variants(dictionary_name):
    board = phone_patterns.charuco_board(dictionary_name)
    corner_count = (phone_patterns.CHARUCO_SQUARES[0] - 1) * (phone_patterns.CHARUCO_SQUARES[1] - 1)

    def variant(**parameters):
        detector_parameters = cv2.aruco.DetectorParameters()
        for key, value in parameters.items():
            setattr(detector_parameters, key, value)
        detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), detector_parameters)

        def detect(gray):
            corners, ids, _, _ = detector.detectBoard(gray)
            if ids is None or len(ids) != corner_count:
                return None
            return corners.reshape(-1, 2), board.getChessboardCorners()[ids.flatten()]

        return detect

    return {
        "charuco as shipped": variant(),
        "charuco centre sampling": variant(perspectiveRemoveIgnoredMarginPerCell=0.25),
        "charuco centre + full correction": variant(perspectiveRemoveIgnoredMarginPerCell=0.25, errorCorrectionRate=1.0),
    }


def chessboard_variants():
    size = phone_patterns.CHESSBOARD_INNER_CORNERS
    object_points = np.array([(col, row, 0) for row in range(size[1]) for col in range(size[0])], np.float32)

    def sector_based(gray):
        found, corners = cv2.findChessboardCornersSB(gray, size, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        return (corners.reshape(-1, 2), object_points) if found else None

    def classic(gray):
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(gray, size, flags=flags)
        if not found:
            return None
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        return corners.reshape(-1, 2), object_points

    return {"chessboard sector based": sector_based, "chessboard classic": classic}


def circle_variants():
    blob_parameters = cv2.SimpleBlobDetector_Params()
    blob_parameters.filterByColor = True
    blob_parameters.blobColor = 255
    blob_parameters.filterByArea = True
    blob_parameters.minArea = 20
    blob_parameters.maxArea = 20000
    blob_parameters.filterByConvexity = False
    blob_parameters.filterByInertia = False
    blob_parameters.filterByCircularity = False
    blob_detector = cv2.SimpleBlobDetector_create(blob_parameters)

    per_row, rows = phone_patterns.CIRCLE_GRID
    object_points = np.array([(2 * col + row % 2, row, 0) for row in range(rows) for col in range(per_row)], np.float32)

    def variant(flags):
        def detect(gray):
            found, centers = cv2.findCirclesGrid(gray, phone_patterns.CIRCLE_GRID, flags=flags,
                                                 blobDetector=blob_detector)
            return (centers.reshape(-1, 2), object_points) if found else None

        return detect

    return {
        "circle grid": variant(cv2.CALIB_CB_ASYMMETRIC_GRID),
        "circle grid clustering": variant(cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING),
    }


def pattern_variants(pattern, dictionary_name):
    """Detector variants by name. Each returns the pattern's image points and matching board points, or None."""
    if pattern == "charuco":
        return charuco_variants(dictionary_name)
    if pattern == "chessboard":
        return chessboard_variants()
    return circle_variants()


def load_frames(directory):
    with open(os.path.join(directory, "frames.csv"), encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    times = [float(row["seconds"]) for row in rows]
    frames = [cv2.imread(os.path.join(directory, row["file"]), cv2.IMREAD_GRAYSCALE) for row in rows]
    return times, frames


def completed_holds(times, found, grace_seconds):
    """One second holds of an unbroken detection, as the wizard's gate counts them.

    The gate restarts on a frame without the whole pattern. A grace period lets the hold survive a gap of
    missed frames up to that long.
    """
    holds = 0
    held = 0.0
    gap = 0.0
    for index in range(1, len(times)):
        step = times[index] - times[index - 1]
        if found[index]:
            held += step
            gap = 0.0
        else:
            gap += step
            if gap > grace_seconds:
                held = 0.0
        if found[index] and held >= HOLD_SECONDS:
            holds += 1
            held = 0.0
    return holds


def longest_run_seconds(times, found):
    longest = 0.0
    start = None
    for index, hit in enumerate(found):
        if hit and start is None:
            start = times[index]
        if start is not None and (not hit or index == len(found) - 1):
            longest = max(longest, times[index] - start)
            start = None
    return longest


def score(args):
    times, frames = load_frames(args.dir)

    variants = pattern_variants(args.pattern, args.dictionary)
    duration = times[-1] - times[0] if len(times) > 1 else 0.0
    print(f"{args.dir}: {len(frames)} frames over {duration:.1f} s")
    print(f"{'detector':<34}{'found %':>9}{'ms p50':>8}{'longest s':>11}{'holds':>7}{'holds, grace':>14}")
    for name, detect in variants.items():
        found = []
        milliseconds = []
        for gray in frames:
            start = time.perf_counter()
            found.append(detect(gray) is not None)
            milliseconds.append(1000.0 * (time.perf_counter() - start))
        share = 100.0 * sum(found) / max(len(found), 1)
        print(f"{name:<34}{share:>9.1f}{statistics.median(milliseconds):>8.1f}{longest_run_seconds(times, found):>11.1f}"
              f"{completed_holds(times, found, 0.0):>7}{completed_holds(times, found, GRACE_SECONDS):>14}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)

    capture_parser = commands.add_parser("capture")
    capture_parser.add_argument("out_dir")
    capture_parser.add_argument("--camera", type=int, default=0)
    capture_parser.add_argument("--seconds", type=float, default=20.0)
    capture_parser.add_argument("--width", type=int, default=1280)
    capture_parser.add_argument("--height", type=int, default=720)
    capture_parser.set_defaults(run=capture)

    score_parser = commands.add_parser("score")
    score_parser.add_argument("dir")
    score_parser.add_argument("--pattern", choices=["charuco", "chessboard", "circles"], required=True)
    score_parser.add_argument("--dictionary", choices=sorted(phone_patterns.CHARUCO_DICTIONARIES), default="6x6")
    score_parser.set_defaults(run=score)

    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
