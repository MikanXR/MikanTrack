"""Keeps each intrinsics wizard run of a project and reports the runs against reference calibrations.

The wizard overwrites a camera's intrinsics on accept, so `record` copies them out after each run, with the
views the run solved from (the capture set the wizard leaves in the project's dumps folder).

    board_runs.py record <project.json> <runs.jsonl> <label> [--camera N]
    board_runs.py report <runs.jsonl> --reference <project.json>[:camera] ...
    board_runs.py views <runs.jsonl | capture_set.json> [--reference-fx FX]

`views` solves each run's capture set again and reports what the set looked like: the solver's own standard
deviation on the focal length, how far the focal length moves with one view left out, each view's tilt, and
how much of the frame the pattern filled.
"""

import argparse
import json
import os
import statistics

import cv2
import numpy as np

from pattern_solve import SOLVE_FLAGS
from spike_common import hfov_degrees, matrix_from_config


def read_intrinsics(project_path, camera_index):
    with open(project_path, encoding="utf-8") as f:
        project = json.load(f)
    intrinsics = project["cameras"][camera_index]["intrinsics"]
    if not intrinsics.get("present", True):
        raise SystemExit(f"{project_path}: camera {camera_index} has no intrinsics")
    matrix = matrix_from_config(intrinsics["distortedCameraMatrix"])
    return {
        "board": project.get("charucoBoard", {}),
        "width": intrinsics["width"],
        "height": intrinsics["height"],
        "fx": float(matrix[0, 0]),
        "k1": float(intrinsics["distortion"][0]),
        "k2": float(intrinsics["distortion"][1]),
        "reprojectionError": float(intrinsics.get("reprojectionError", 0.0)),
    }


def record(args):
    run = read_intrinsics(args.project, args.camera)
    run["label"] = args.label
    capture_path = os.path.join(os.path.dirname(os.path.abspath(args.project)), "dumps",
                                f"intrinsics_capture_camera{args.camera + 1}.json")
    if os.path.exists(capture_path):
        with open(capture_path, encoding="utf-8") as f:
            run["capture"] = json.load(f)
    else:
        print(f"no capture set at {capture_path}")
    with open(args.runs, "a", encoding="utf-8") as f:
        f.write(json.dumps(run) + "\n")
    board = run["board"]
    print(f"{args.label}: board {board.get('cols')}x{board.get('rows')}  fx {run['fx']:.1f}  "
          f"k1 {run['k1']:.3f}  k2 {run['k2']:.3f}  rms {run['reprojectionError']:.2f} px")


def parse_reference(text):
    # A drive letter's colon is not a camera separator
    head, separator, tail = text.rpartition(":")
    if separator and tail.isdigit() and len(head) > 1:
        return head, int(tail)
    return text, 0


def report(args):
    references = [read_intrinsics(*parse_reference(text)) for text in args.reference]
    reference_fx = statistics.mean(r["fx"] for r in references)
    print("reference fx: " + ", ".join(f"{r['fx']:.1f}" for r in references) + f"  (mean {reference_fx:.1f})")
    print("reference k1: " + ", ".join(f"{r['k1']:.3f}" for r in references)
          + "   k2: " + ", ".join(f"{r['k2']:.3f}" for r in references))

    with open(args.runs, encoding="utf-8") as f:
        runs = [json.loads(line) for line in f if line.strip()]

    print(f"\n{'label':<24}{'board':>7}{'fx':>9}{'err %':>8}{'hfov':>7}{'k1':>8}{'k2':>8}{'rms px':>8}")
    groups = {}
    for run in runs:
        board = f"{run['board'].get('cols')}x{run['board'].get('rows')}"
        error = 100.0 * (run["fx"] / reference_fx - 1.0)
        print(f"{run['label']:<24}{board:>7}{run['fx']:>9.1f}{error:>8.1f}{hfov_degrees(run['fx'], run['width']):>7.1f}"
              f"{run['k1']:>8.3f}{run['k2']:>8.3f}{run['reprojectionError']:>8.2f}")
        # Runs sharing a label up to its trailing run number form one group
        groups.setdefault(run["label"].rstrip("0123456789").rstrip("-_ "), []).append(run["fx"])

    print(f"\n{'group':<24}{'runs':>7}{'median fx':>11}{'err %':>8}{'spread %':>10}")
    for name, values in groups.items():
        median = statistics.median(values)
        spread = 100.0 * (max(values) - min(values)) / reference_fx
        print(f"{name:<24}{len(values):>7}{median:>11.1f}{100.0 * (median / reference_fx - 1.0):>8.1f}{spread:>10.1f}")


def solve_capture(capture, skip=None):
    board = np.array(capture["boardPoints"], dtype=np.float32)
    object_points = []
    image_points = []
    for index, view in enumerate(capture["views"]):
        if index == skip:
            continue
        object_points.append(board[view["ids"]])
        image_points.append(np.array(view["points"], dtype=np.float32))
    image_size = (int(capture["width"]), int(capture["height"]))
    rms, matrix, _, rvecs, _, deviations, _, _ = cv2.calibrateCameraExtended(
        object_points, image_points, image_size, None, None, flags=SOLVE_FLAGS)
    # One focal length is solved, reported under whichever axis the solver varied
    return float(matrix[0, 0]), float(max(deviations.flatten()[0:2])), rvecs, image_points, float(rms)


def views(args):
    with open(args.runs, encoding="utf-8") as f:
        text = f.read()
    if args.runs.endswith(".jsonl"):
        runs = [json.loads(line) for line in text.splitlines() if line.strip()]
    else:
        runs = [{"label": os.path.basename(args.runs), "capture": json.loads(text)}]

    for run in runs:
        capture = run.get("capture")
        if capture is None:
            print(f"{run['label']}: no capture set recorded")
            continue
        fx, deviation, rvecs, image_points, rms = solve_capture(capture)
        count = len(capture["views"])
        left_out = [solve_capture(capture, skip=index)[0] for index in range(count)] if count > 4 else [fx]
        tilts = []
        for rvec in rvecs:
            rotation, _ = cv2.Rodrigues(rvec)
            # The angle between the pattern's normal and the camera's optical axis
            tilts.append(float(np.degrees(np.arccos(min(1.0, abs(rotation[2, 2]))))))
        width_share = [float(np.ptp(points[:, 0])) / capture["width"] for points in image_points]
        centres = np.array([points.mean(axis=0) for points in image_points])
        error = f"  ({100.0 * (fx / args.reference_fx - 1.0):+.1f}%)" if args.reference_fx else ""
        print(f"{run['label']}: {count} views  fx {fx:.1f}{error}  rms {rms:.2f} px")
        dropped = capture.get("droppedViews", [])
        if dropped:
            print(f"  dropped as outliers   {' '.join(format(view['reprojectionError'], '.2f') for view in dropped)} px")
        print(f"  solver std on fx      {100.0 * deviation / fx:.2f}%")
        print(f"  one view left out     fx {min(left_out):.1f} to {max(left_out):.1f}  "
              f"(spread {100.0 * (max(left_out) - min(left_out)) / fx:.2f}%)")
        print(f"  tilt per view, deg    {' '.join(str(round(tilt)) for tilt in sorted(tilts))}")
        print(f"  pattern width share   {min(width_share):.2f} to {max(width_share):.2f} of the frame")
        print(f"  pattern centres       x {centres[:, 0].min():.0f} to {centres[:, 0].max():.0f}  "
              f"y {centres[:, 1].min():.0f} to {centres[:, 1].max():.0f}  of {capture['width']}x{capture['height']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)

    record_parser = commands.add_parser("record")
    record_parser.add_argument("project")
    record_parser.add_argument("runs")
    record_parser.add_argument("label")
    record_parser.add_argument("--camera", type=int, default=0)
    record_parser.set_defaults(run=record)

    report_parser = commands.add_parser("report")
    report_parser.add_argument("runs")
    report_parser.add_argument("--reference", action="append", required=True)
    report_parser.set_defaults(run=report)

    views_parser = commands.add_parser("views")
    views_parser.add_argument("runs")
    views_parser.add_argument("--reference-fx", type=float, default=0.0)
    views_parser.set_defaults(run=views)

    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
