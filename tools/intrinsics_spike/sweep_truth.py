"""Replay a take with stereo truth through --gate-direct with the candidates
under scaled focal lengths and the truth on the take's own calibration, and
report the candidates' shoulder-to-wrist reach against the truth's.

  python sweep_truth.py <MikanTrack.exe> <work_dir> <recording.jsonl> --camera N
      [--tilt DEG] [--fx-scales 0.85,0.9,...]

Each scale is a header-patched copy under <work_dir>, passed to the tool as
candidates= beside the scale-1 copy, so only the candidates see the changed
lens. --tilt sets direct.cameraTiltDegrees in every copy, for a take whose
header does not carry the camera's own tilt. The reach is the wrist minus
the shoulder in the camera's frame: the truth's from the stereo wrist and the
ray-and-width shoulder, which do not depend on the elbow, the direct solver's
from its own chain in the user's meters, and each candidate's from its posed
avatar. The upper arm, forearm and wrist-along-ray columns are the tool's own
numbers against the position retarget, the mean of the two sides.
"""

import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

LOG_PATTERNS = [
    ("directUpper", r"direct retarget vs position retarget on \d+ frames: upper arm p50 ([\d.]+)"),
    ("directForearm",
     r"direct retarget vs position retarget on \d+ frames: upper arm p50 [\d.]+ p90 [\d.]+ deg, forearm p50 ([\d.]+)"),
    ("baselineUpper", r"kalidokit-style vs position retarget on \d+ frames: upper arm p50 ([\d.]+)"),
    ("baselineForearm",
     r"kalidokit-style vs position retarget on \d+ frames: upper arm p50 [\d.]+ p90 [\d.]+ deg, forearm p50 ([\d.]+)"),
    ("ray", r"wrist along its ray vs reference: error p50 ([\d.]+)"),
]


def angle_deg(a, b):
    norms = np.maximum(np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1), 1e-9)
    return np.degrees(np.arccos(np.clip(np.sum(a * b, axis=1) / norms, -1.0, 1.0)))


def vector(row, prefix):
    return np.array([float(row[prefix + axis]) for axis in "XYZ"])


def load_reaches(csv_path):
    """(frame, side) to the truth, chain, direct and baseline reach, on rows that carry all four."""
    reaches = {}
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if float(row["truthWristZ"]) <= 0.0:
                continue
            if row["directPosed"] != "1" or row["baselinePosed"] != "1" or row["directChain"] != "1":
                continue
            reaches[(int(row["frame"]), int(row["side"]))] = (
                vector(row, "truthWrist") - vector(row, "truthShoulder"), vector(row, "directChainReach"),
                vector(row, "directReach"), vector(row, "baselineReach"))
    return reaches


def load_log_numbers(log_path):
    with open(log_path, encoding="utf-8", errors="replace") as f:
        text = f.read()
    numbers = {}
    for name, pattern in LOG_PATTERNS:
        # The take's own block comes first and the pooled block repeats it
        found = [float(value) for value in re.findall(pattern, text)][:2]
        numbers[name] = float(np.mean(found)) if found else float("nan")
    return numbers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("exe")
    parser.add_argument("work_dir")
    parser.add_argument("recording")
    parser.add_argument("--camera", type=int, required=True)
    parser.add_argument("--tilt", type=float)
    parser.add_argument("--fx-scales", default="0.85,0.90,0.95,1.00,1.05,1.10,1.15,1.25,1.50")
    args = parser.parse_args()

    scales = sorted(set([float(s) for s in args.fx_scales.split(",")] + [1.0]))
    # The tool runs from its own folder, so a relative path to it would not resolve there
    args.exe = os.path.abspath(args.exe)
    exe_dir = os.path.dirname(args.exe)
    stem = os.path.splitext(os.path.basename(args.recording))[0]
    os.makedirs(args.work_dir, exist_ok=True)

    tilt_delta = 0.0
    if args.tilt is not None:
        with open(args.recording, encoding="utf-8") as f:
            header = json.loads(f.readline())
        tilt_delta = args.tilt - float(header["appConfig"].get("direct", {}).get("cameraTiltDegrees", 0.0))

    def variant(scale):
        out = os.path.join(args.work_dir, f"{stem}_fx{scale:.2f}.jsonl")
        if not os.path.exists(out):
            subprocess.run([sys.executable, os.path.join(HERE, "patch_header.py"), args.recording, out, "--camera",
                            str(args.camera), "--fx-scale", str(scale), "--tilt-delta", str(tilt_delta)], check=True)
        return out

    base = variant(1.0)
    runs = {}
    for scale in scales:
        csv_path = os.path.join(args.work_dir, f"{stem}_fx{scale:.2f}.csv")
        log_path = os.path.splitext(csv_path)[0] + ".log"
        if not os.path.exists(csv_path):
            command = [args.exe, "--gate-direct", base, str(args.camera)]
            if scale != 1.0:
                command.append("candidates=" + variant(scale))
            command.append(csv_path)
            # The tool's exit status is its gates', which a perturbed run may fail: the CSV is the result
            subprocess.run(command, cwd=exe_dir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if not os.path.exists(csv_path):
                raise RuntimeError(f"no CSV from --gate-direct at scale {scale}; read gate-direct.log beside the exe")
            shutil.copyfile(os.path.join(exe_dir, "gate-direct.log"), log_path)
        runs[scale] = (load_reaches(csv_path), load_log_numbers(log_path))

    keys = sorted(set.intersection(*[set(reaches) for reaches, _ in runs.values()]))
    if not keys:
        raise RuntimeError("no arm frame carries a truth wrist and a posed candidate under every lens")
    print(f"{stem}: {len(keys)} arm frames with a truth wrist, posed under every lens")
    # A lens far from the calibration poses fewer frames and narrows the shared set: its own count shows it
    print("  fx    own    | chain reach against truth    | posed reach deg   | against the position retarget")
    print("  scale frames | deg p50   p90 cm p50 depth cm | direct  baseline  | d.upper d.fore b.upper b.fore ray cm")
    for scale in scales:
        reaches, log = runs[scale]
        truth, chain, direct, baseline = (np.array([reaches[key][i] for key in keys]) for i in range(4))
        chain_deg = angle_deg(truth, chain)
        print(f"  {scale:5.2f} {len(reaches):6d} | {np.median(chain_deg):7.1f} {np.percentile(chain_deg, 90):5.1f}"
              f" {np.median(np.linalg.norm(chain - truth, axis=1)) * 100:6.1f}"
              f" {np.median(np.abs(chain[:, 2] - truth[:, 2])) * 100:8.1f} |"
              f" {np.median(angle_deg(truth, direct)):6.1f} {np.median(angle_deg(truth, baseline)):9.1f}  |"
              f" {log['directUpper']:7.1f} {log['directForearm']:6.1f} {log['baselineUpper']:7.1f}"
              f" {log['baselineForearm']:6.1f} {log['ray']:6.1f}")


if __name__ == "__main__":
    main()
