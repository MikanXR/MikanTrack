"""Replay a recording through --gate-direct under perturbed calibration and
report how far the direct retarget's posed bones move from the unperturbed run.

  python sweep_sensitivity.py <MikanTrack.exe> <work_dir> <recording.jsonl>...
      [--camera N] [--fx-scales 0.85,0.9,...] [--tilt-deltas -10,-5,...] [--no-undistort]

Each variant is a header-patched copy under <work_dir>, replayed with the exe's
directory as the working directory. The CSV's posed bone rotations are compared
frame by frame against the scale-1 run: the angle of the rotation between the
two, p50 and p90 in degrees per bone, over frames both runs posed.
--no-undistort adds a variant on raw-equivalent frames with no distortion
correction, at each fx scale given.
"""

import argparse
import csv
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BONES = ["Upper", "Forearm", "Hand", "Head"]


def run(command, cwd=None):
    subprocess.run(command, cwd=cwd, check=True, stdout=subprocess.DEVNULL)


def make_variant(recording, out_path, camera, fx_scale=1.0, tilt_delta=0.0, raw_frames=None):
    command = [sys.executable, os.path.join(HERE, "patch_header.py"), recording, out_path, "--camera", str(camera),
               "--fx-scale", str(fx_scale), "--tilt-delta", str(tilt_delta)]
    if raw_frames:
        command += ["--raw-frames", raw_frames]
    run(command)


def gate_direct(exe, recording, camera, csv_path):
    exe_dir = os.path.dirname(os.path.abspath(exe))
    # The tool's exit status is its gates', which a perturbed run may fail: the CSV is the result
    subprocess.run([exe, "--gate-direct", recording, str(camera), csv_path], cwd=exe_dir,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log = os.path.join(exe_dir, "gate-direct.log")
    if os.path.exists(log):
        shutil.copyfile(log, os.path.splitext(csv_path)[0] + ".log")
    if not os.path.exists(csv_path):
        raise RuntimeError(f"no CSV from --gate-direct for {recording}; read {log}")


def load_poses(csv_path):
    poses = {}
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if row.get("directPosed") != "1":
                continue
            quats = [np.array([float(row[f"direct{bone}Q{axis}"]) for axis in "WXYZ"]) for bone in BONES]
            poses[(int(row["frame"]), int(row["side"]))] = quats
    return poses


def rotation_angle_deg(a, b):
    dot = abs(float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return float(np.degrees(2.0 * np.arccos(min(1.0, dot))))


def compare(baseline, variant):
    keys = sorted(set(baseline) & set(variant))
    angles = np.array([[rotation_angle_deg(baseline[k][i], variant[k][i]) for i in range(len(BONES))] for k in keys])
    return len(keys), angles


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("exe")
    parser.add_argument("work_dir")
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--fx-scales", default="0.85,0.90,0.95,0.98,1.02,1.05,1.10,1.15")
    parser.add_argument("--tilt-deltas", default="-10,-5,-2,2,5,10")
    parser.add_argument("--no-undistort", action="store_true")
    args = parser.parse_args()
    fx_scales = [float(v) for v in args.fx_scales.split(",") if v]
    tilt_deltas = [float(v) for v in args.tilt_deltas.split(",") if v]

    pooled = {}
    for take_index, recording in enumerate(args.recordings):
        name = os.path.basename(recording)
        take_dir = os.path.join(args.work_dir, f"take{take_index}")

        def replay(label, **patch):
            variant_dir = os.path.join(take_dir, label)
            copy = os.path.join(variant_dir, name)
            csv_path = os.path.join(variant_dir, "frames.csv")
            if not os.path.exists(csv_path):
                make_variant(recording, copy, args.camera, **patch)
                gate_direct(args.exe, copy, args.camera, csv_path)
            return load_poses(csv_path)

        baseline = replay("base")
        variants = [(f"fx x{scale:.2f}", dict(fx_scale=scale)) for scale in fx_scales]
        variants += [(f"tilt {delta:+.0f} deg", dict(tilt_delta=delta)) for delta in tilt_deltas]
        if args.no_undistort:
            raw_dir = os.path.join(take_dir, "raw_frames")
            if not os.path.exists(raw_dir):
                run([sys.executable, os.path.join(HERE, "redistort_frames.py"), recording, raw_dir,
                     "--camera", str(args.camera)])
            variants += [("raw, fx x1.00", dict(raw_frames=raw_dir))]
            variants += [(f"raw, fx x{scale:.2f}", dict(fx_scale=scale, raw_frames=raw_dir)) for scale in fx_scales]
        for label, patch in variants:
            count, angles = compare(baseline, replay(label.replace(" ", "_").replace(",", ""), **patch))
            pooled.setdefault(label, []).append(angles)
            print(f"{name} {label}: {count} hand frames", flush=True)

    print("\nRotation of each posed bone against the unperturbed run, degrees (p50 / p90), pooled over takes")
    print(f"{'variant':<18}{'frames':>8}" + "".join(f"{bone:>16}" for bone in BONES))
    for label, parts in pooled.items():
        angles = np.concatenate(parts)
        cells = "".join(f"{np.percentile(angles[:, i], 50):>8.2f} /{np.percentile(angles[:, i], 90):>6.2f}"
                        for i in range(len(BONES)))
        print(f"{label:<18}{len(angles):>8}{cells}")


if __name__ == "__main__":
    main()
