"""Marker-free focal length estimates on raw-equivalent frames against the
recording's charuco calibration.

  python estimate_fx.py <recording.jsonl> <raw_frames_dir> [--camera N] [--samples M]
      [--methods nominal,moge,geocalib] [--label NAME]

The frames come from redistort_frames.py. The reference is the header's
distorted camera matrix, the pinhole of the raw image. Per method it prints the
median focal length over the sampled frames, its error against the reference,
the spread across frames, and for GeoCalib its radial term and gravity.
"""

import argparse
import glob
import os

import cv2
import numpy as np

from spike_common import camera_intrinsics, fx_from_hfov, hfov_degrees, matrix_from_config, read_header

NOMINAL_HFOV_DEGREES = [60.0, 70.0, 78.0]
MOGE_MODEL = "Ruicheng/moge-2-vitl-normal"


def sample_paths(frames_dir, camera, count):
    paths = sorted(glob.glob(os.path.join(frames_dir, f"cam{camera}_*.jpg")))
    if not paths:
        raise RuntimeError(f"no cam{camera} frames in {frames_dir}")
    step = max(1, len(paths) // count)
    return paths[::step][:count]


def run_moge(paths):
    import torch
    from moge.model.v2 import MoGeModel

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = MoGeModel.from_pretrained(MOGE_MODEL).to(device).eval()
    focals = []
    for path in paths:
        rgb = cv2.cvtColor(cv2.imread(path, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        image = torch.tensor(rgb / 255.0, dtype=torch.float32, device=device).permute(2, 0, 1)
        with torch.no_grad():
            out = model.infer(image, fov_x=None)
        # Normalized intrinsics: focal over width
        focals.append(float(out["intrinsics"][0, 0].cpu()) * rgb.shape[1])
    return {"fx": np.array(focals)}


def run_geocalib(paths, weights, camera_model, shared):
    import torch
    from geocalib import GeoCalib

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = GeoCalib(weights=weights).to(device)
    images = [model.load_image(path).to(device) for path in paths]
    results = {"fx": [], "k1": [], "roll": [], "pitch": []}

    def collect(result):
        camera, gravity = result["camera"], result["gravity"]
        results["fx"] += [float(v) for v in camera.f[..., 0].reshape(-1).cpu()]
        if camera_model != "pinhole":
            results["k1"] += [float(v) for v in camera.k1.reshape(-1).cpu()]
        results["roll"] += [float(np.degrees(v)) for v in gravity.roll.reshape(-1).cpu()]
        results["pitch"] += [float(np.degrees(v)) for v in gravity.pitch.reshape(-1).cpu()]

    if shared:
        collect(model.calibrate(torch.stack(images), camera_model=camera_model, shared_intrinsics=True))
    else:
        for image in images:
            collect(model.calibrate(image, camera_model=camera_model))
    return {key: np.array(values) for key, values in results.items() if values}


def describe(values):
    return f"{np.median(values):8.3f} ({np.percentile(values, 10):.3f} to {np.percentile(values, 90):.3f})"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("recording")
    parser.add_argument("raw_frames_dir")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--samples", type=int, default=24)
    parser.add_argument("--methods", default="nominal,moge,geocalib")
    parser.add_argument("--label", default="")
    args = parser.parse_args()
    methods = args.methods.split(",")

    header = read_header(args.recording)
    intrinsics = camera_intrinsics(header, args.camera)
    width = int(intrinsics["width"])
    truth_fx = matrix_from_config(intrinsics["distortedCameraMatrix"])[0, 0]
    distortion = intrinsics["distortion"]
    device_name = header["appConfig"]["cameras"][args.camera]["video"]["deviceName"]
    print(f"\n== {args.label or device_name} (camera {args.camera}, {width} px wide) ==")
    print(f"charuco: fx {truth_fx:.1f} px, hfov {hfov_degrees(truth_fx, width):.1f} deg, "
          f"k1 {distortion[0]:.3f}, k2 {distortion[1]:.3f}")
    tilt = header["appConfig"].get("direct", {}).get("cameraTiltDegrees")
    if tilt is not None:
        print(f"configured direct tilt: {tilt:.1f} deg")

    paths = sample_paths(args.raw_frames_dir, args.camera, args.samples)
    rows = []
    if "nominal" in methods:
        for hfov in NOMINAL_HFOV_DEGREES:
            rows.append((f"nominal hfov {hfov:.0f}", {"fx": np.array([fx_from_hfov(hfov, width)])}))
    if "moge" in methods:
        rows.append(("MoGe-2", run_moge(paths)))
    if "geocalib" in methods:
        rows.append(("GeoCalib pinhole", run_geocalib(paths, "pinhole", "pinhole", False)))
        rows.append(("GeoCalib pinhole, shared", run_geocalib(paths, "pinhole", "pinhole", True)))
        rows.append(("GeoCalib radial", run_geocalib(paths, "distorted", "simple_radial", False)))
        rows.append(("GeoCalib radial, shared", run_geocalib(paths, "distorted", "simple_radial", True)))

    print(f"{len(paths)} frames. Columns are the median with the 10th to 90th percentile across frames.")
    print(f"{'method':<26}{'fx err %':>9}  {'fx px':<30}{'k1':<28}{'pitch deg':<28}{'roll deg'}")
    for name, result in rows:
        fx = result["fx"]
        error = 100.0 * (np.median(fx) - truth_fx) / truth_fx
        cells = [describe(result[key]) if key in result else "" for key in ("k1", "pitch", "roll")]
        print(f"{name:<26}{error:>+9.1f}  {describe(fx):<30}{cells[0]:<28}{cells[1]:<28}{cells[2]}")


if __name__ == "__main__":
    main()
