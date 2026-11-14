"""Keeps each intrinsics wizard run of a project and reports the runs against reference calibrations.

The wizard overwrites a camera's intrinsics on accept, so `record` copies them out after each run.

    board_runs.py record <project.json> <runs.jsonl> <label> [--camera N]
    board_runs.py report <runs.jsonl> --reference <project.json>[:camera] ...
"""

import argparse
import json
import statistics

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

    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
