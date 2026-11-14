"""Shared helpers for the intrinsics spike: recording headers, camera matrices, frame folders."""

import json
import os
import subprocess

import numpy as np


def read_header(recording_path):
    with open(recording_path, encoding="utf-8") as f:
        return json.loads(f.readline())


def camera_intrinsics(header, camera_index):
    return header["appConfig"]["cameras"][camera_index]["intrinsics"]


def matrix_from_config(values):
    """The config stores a 3x3 camera matrix as 9 column-major values."""
    return np.array(values, dtype=np.float64).reshape(3, 3).T


def matrix_to_config(matrix):
    return [float(v) for v in np.asarray(matrix, dtype=np.float64).T.reshape(9)]


def frames_directory(recording_path):
    return os.path.splitext(recording_path)[0] + "_frames"


def link_directory(link_path, target_path):
    """A directory junction, so a patched copy of a recording shares the original's frames."""
    if os.path.exists(link_path):
        return
    subprocess.run(["cmd", "/c", "mklink", "/J", os.path.normpath(link_path), os.path.normpath(target_path)],
                   check=True, stdout=subprocess.DEVNULL)


def hfov_degrees(fx, width):
    return float(np.degrees(2.0 * np.arctan(0.5 * width / fx)))


def fx_from_hfov(hfov_deg, width):
    return 0.5 * width / float(np.tan(np.radians(hfov_deg) * 0.5))
