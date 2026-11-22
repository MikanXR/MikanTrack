#!/usr/bin/env python3
"""Assemble the MikanTrack release zip from a built tree.

    python tools/package.py --build-dir build/Release

Writes dist/MikanTrack_<version>_Win64.zip, where <version> is the VERSION of
the project() call in CMakeLists.txt. The zip is flat: MikanTrack.exe and
everything it needs at run time sit at the zip root, so the extracted folder
runs as it is, with that folder as the working directory.

The contents are an allowlist, not a copy of the build folder. A working build
folder also holds logs, imgui.ini, generated calibration images, and models
that only the measurement tools load, none of which ship.

  from the build folder   the exe and the DLLs the post-build step copied
  from models/            the models the app loads at run time, and the avatars
  from resources/         the files git tracks there
  from Visual Studio      the VC++ runtime DLLs, so no redistributable install
                          is needed on the target machine
  from the repo root      NOTICE.md

The release workflow runs this script, and so does a release cut by hand.
"""

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Copied next to the exe by the post-build step.
REQUIRED_BINARIES = [
    "MikanTrack.exe",
    "SDL2.dll",
    "glew32.dll",
    "onnxruntime.dll",
    "DirectML.dll",
    "opencv_world4100.dll",
]

# Present only in a build made with the Oculus Lipsync SDK under deps/.
OPTIONAL_BINARIES = ["OVRLipSync.dll"]

# The models the app itself loads. InitialSetup_x64.bat fetches exactly these.
MODELS = [
    "palm_detection.onnx",
    "hand_landmark.onnx",
    "person_detection.onnx",
    "rtmpose_body.onnx",
    "rtmw3d_wholebody.onnx",
    "avatars/fem_vroid.vrm",
    "avatars/masc_vroid.vrm",
]

ROOT_FILES = ["NOTICE.md"]


def fail(message):
    print(f"package.py: {message}", file=sys.stderr)
    sys.exit(1)


def read_version():
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    match = re.search(r"project\(\s*MikanTrack\s+VERSION\s+([0-9]+(?:\.[0-9]+)*)", text)
    if not match:
        fail("no project(MikanTrack VERSION ...) in CMakeLists.txt")
    return match.group(1)


def tracy_enabled(build_dir):
    """True when the tree the build folder belongs to was configured with Tracy."""
    for candidate in (build_dir, build_dir.parent):
        cache = candidate / "CMakeCache.txt"
        if cache.is_file():
            text = cache.read_text(encoding="utf-8", errors="replace")
            return re.search(r"^MIKANTRACK_TRACY:BOOL=(ON|TRUE|1)\s*$", text, re.M | re.I) is not None
    return False


def find_vc_runtime_dir():
    """The newest x64 Microsoft.VC*.CRT folder of the installed Visual Studio."""
    candidates = []
    redist = os.environ.get("VCToolsRedistDir")
    if redist:
        candidates += glob.glob(os.path.join(redist, "x64", "Microsoft.VC*.CRT"))
    if not candidates:
        vswhere = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) \
            / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
        if vswhere.is_file():
            result = subprocess.run(
                [str(vswhere), "-latest", "-products", "*", "-property", "installationPath"],
                capture_output=True, text=True, check=False)
            install = result.stdout.strip().splitlines()
            if install:
                candidates += glob.glob(os.path.join(
                    install[0], "VC", "Redist", "MSVC", "[0-9]*", "x64", "Microsoft.VC*.CRT"))
    if not candidates:
        return None

    def version_key(path):
        version = Path(path).parent.parent.name
        return tuple(int(part) for part in version.split(".") if part.isdigit())

    return Path(max(candidates, key=version_key))


def tracked_resources():
    result = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files", "-z", "resources"],
        capture_output=True, check=False)
    if result.returncode != 0:
        fail("git ls-files resources failed: " + result.stderr.decode(errors="replace").strip())
    files = [name for name in result.stdout.decode("utf-8").split("\0") if name]
    if not files:
        fail("git tracks no files under resources/")
    return files


def copy_into(stage, source, relative):
    if not source.is_file():
        fail(f"missing {source}")
    target = stage / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def main():
    parser = argparse.ArgumentParser(description="Assemble the MikanTrack release zip.")
    parser.add_argument("--build-dir", default="build/Release",
                        help="folder holding the built MikanTrack.exe (default build/Release)")
    parser.add_argument("--out", default="dist", help="output folder (default dist)")
    parser.add_argument("--version", help="override the version read from CMakeLists.txt")
    parser.add_argument("--allow-tracy", action="store_true",
                        help="package a build configured with MIKANTRACK_TRACY, which a release is not")
    parser.add_argument("--no-vc-runtime", action="store_true",
                        help="leave the VC++ runtime DLLs out")
    parser.add_argument("--require-ovrlipsync", action="store_true",
                        help="fail when the build has no OVRLipSync.dll, as a release must have it")
    parser.add_argument("--print-version", action="store_true",
                        help="print the version from CMakeLists.txt and exit")
    args = parser.parse_args()

    if args.print_version:
        print(read_version())
        return

    build_dir = Path(args.build_dir)
    if not build_dir.is_absolute():
        build_dir = ROOT / build_dir
    out_dir = Path(args.out)
    if not out_dir.is_absolute():
        out_dir = ROOT / out_dir

    version = args.version or read_version()
    name = f"MikanTrack_{version}_Win64"

    if tracy_enabled(build_dir):
        if not args.allow_tracy:
            fail(f"{build_dir} was configured with MIKANTRACK_TRACY=ON. A release build is a "
                 "configure without it. Pass --allow-tracy to package it anyway.")
        print("warning: packaging a build that carries the Tracy profiler client")

    stage = out_dir / name
    if stage.exists():
        shutil.rmtree(stage)
    stage.mkdir(parents=True)

    for binary in REQUIRED_BINARIES:
        copy_into(stage, build_dir / binary, binary)
    for binary in OPTIONAL_BINARIES:
        if (build_dir / binary).is_file():
            copy_into(stage, build_dir / binary, binary)
            print(f"included optional {binary}")
        elif args.require_ovrlipsync:
            fail(f"{binary} is not in {build_dir}: the build was configured without the "
                 "Oculus Lipsync SDK under deps/OVRLipSyncNative")
        else:
            print(f"optional {binary} is not in the build: left out")

    for model in MODELS:
        copy_into(stage, ROOT / "models" / model, Path("models") / model)
    for resource in tracked_resources():
        copy_into(stage, ROOT / resource, resource)
    for root_file in ROOT_FILES:
        copy_into(stage, ROOT / root_file, root_file)

    if not args.no_vc_runtime:
        runtime_dir = find_vc_runtime_dir()
        if runtime_dir is None:
            fail("no VC++ runtime folder found (VCToolsRedistDir or vswhere). "
                 "Pass --no-vc-runtime to package without it.")
        runtime_dlls = sorted(runtime_dir.glob("*.dll"))
        if not runtime_dlls:
            fail(f"no DLLs in {runtime_dir}")
        for dll in runtime_dlls:
            copy_into(stage, dll, dll.name)
        print(f"included {len(runtime_dlls)} VC++ runtime DLLs from {runtime_dir}")

    zip_path = out_dir / f"{name}.zip"
    if zip_path.exists():
        zip_path.unlink()
    files = sorted(path for path in stage.rglob("*") if path.is_file())
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            archive.write(path, path.relative_to(stage).as_posix())

    with zipfile.ZipFile(zip_path) as archive:
        names = archive.namelist()
        if "MikanTrack.exe" not in names:
            fail("MikanTrack.exe is not at the zip root")
        if archive.testzip() is not None:
            fail("the zip failed its CRC check")

    shutil.rmtree(stage)
    size_mb = zip_path.stat().st_size / (1024 * 1024)
    print(f"wrote {zip_path} ({len(names)} files, {size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
