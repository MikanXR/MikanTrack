# Build System

How MikanTrack is configured and built: toolchain, dependency setup, the CMake target, and the ONNX model set it depends on. A copy-paste command cheat sheet is in [commands.md](./commands.md); how to run and interpret the two kinds of test output is in [debugging.md](./debugging.md).

---

## Toolchain

- Windows 10/11 only, MSVC via Visual Studio 2026 or 2022, x64 only.
- CMake minimum 3.15 (`cmake_minimum_required` in the root `CMakeLists.txt`). The `Visual Studio 18 2026` generator needs CMake 4.2 or newer.
- The root `CMakeLists.txt` guards on `MSVC`: `if(NOT MSVC) message(FATAL_ERROR "MikanTrack is Windows/MSVC only") endif()`. There is no non-Windows configuration path.
- C++20 (`CMAKE_CXX_STANDARD 20`), `/MP /W3`, with `NOMINMAX`, `_CRT_SECURE_NO_WARNINGS`, `WIN32_LEAN_AND_MEAN`, and `GLM_ENABLE_EXPERIMENTAL` defined globally. The project also enables the C language, solely for `thirdparty/tinyfiledialogs/tinyfiledialogs.c`.
- The whole project is one file: a single `CMakeLists.txt` at the repo root configuring a single target, `MikanTrack`. There is no `cmake/` subtree of included modules and no multi-target library split.

---

## Build sequence

Three steps, in order:

1. `InitialSetup_x64.bat`: fetches submodules, dependencies, and models. Only needs rerunning when dependency versions change.
2. `GenerateProjectFiles_X64_VS2026.bat` (or `GenerateProjectFiles_X64_VS2022.bat`): configures `build/` and produces `build/MikanTrack.slnx` (or `build/MikanTrack.sln`).
3. `cmake --build build --config Release`, or open the solution in Visual Studio and build there.

---

## First-time setup

`InitialSetup_x64.bat` (run from the repo root):

- Runs `git submodule update --init --recursive` to populate `thirdparty/`.
- Deletes any existing `deps/` folder and recreates it from scratch. This is a full wipe-and-redownload every time, roughly 1.3 GB total, with OpenCV alone around 900 MB. Never store anything by hand in `deps/`; it does not survive a re-run. The one exception is `deps/OVRLipSyncNative`, the optional SDK below, which the script moves aside before the wipe and puts back after it.
- Downloads pinned dependency versions with `curl` and extracts them with the vendored `tools/7zip/7za.exe` (zips, self-extracting `.exe` archives, and NuGet `.nupkg` files, which are zips under a different extension).
- Downloads the ONNX models into `models/`.

`deps/` and `models/` are both gitignored; nothing under either is checked in.

### Dependencies (`deps/`)

| Dependency | Version | Location | Source |
|---|---|---|---|
| SDL2 | 2.30.10 (devel VC) | `deps/SDL2-2.30.10` | GitHub release zip |
| GLEW | 2.2.0 | `deps/glew-2.2.0` | GitHub release zip |
| OpenCV | 4.10.0, `opencv_world` build | `deps/opencv` | GitHub release self-extracting `.exe`, unpacked with `7za` |
| ONNX Runtime | 1.20.1, DirectML flavor | `deps/onnxruntime` | NuGet package `microsoft.ml.onnxruntime.directml` |
| DirectML | 1.15.4 | `deps/directml` | NuGet package `microsoft.ai.directml` |
| Tracy viewer | 0.15.0 | `deps/tracy` | GitHub release zip, the profiler's viewer and capture tools. `RunProfiler.bat` fetches it on first use where `deps/` predates it |
| Tracy MCP server | 0.15.0 | `deps/tracy-mcp` | A venv with `extra/mcp/requirements.txt` from PyPI, and Tracy's server Python bindings built from `thirdparty/tracy` (pybind11 and the server's libraries fetched by its CMake). `RunTracyMcp.bat` makes both on first use |

### Optional: the Oculus Lipsync SDK (`deps/OVRLipSyncNative`)

The microphone mouth stage's Oculus LipSync model needs the Oculus Lipsync native SDK (OVRLipSync 1.61.0), a manual download from https://developer.oculus.com/downloads/package/oculus-lipsync-native/ under the Oculus SDK License. It sits behind the developer portal, so the setup script cannot fetch it, and its 40 MB DLL is not committed. Release builds are made with the SDK present, so `OVRLipSync.dll` ships beside the exe under that license. The release workflow reads the SDK from a private repository (see Release below). Unpack it so these exist:

- `deps/OVRLipSyncNative/Include/OVRLipSync.h`
- `deps/OVRLipSyncNative/Lib/Win64/OVRLipSyncShim.lib`
- `deps/OVRLipSyncNative/Lib/Win64/OVRLipSyncShimd.lib`
- `deps/OVRLipSyncNative/Lib/Win64/OVRLipSync.dll`

Then rerun `GenerateProjectFiles_X64_VS2022.bat`. CMake detects `Include/OVRLipSync.h` at configure time and prints a status line either way. When it is present, CMake defines `MIKANTRACK_OVRLIPSYNC` on the target, adds the include directory, links the shim (`OVRLipSyncShim.lib`, or `OVRLipSyncShimd.lib` under Debug, a static loader that loads `OVRLipSync.dll` at run time and resolves its `ovrLipSyncDll_*` exports), and copies the DLL beside the exe post-build. Without it the Oculus backend's source compiles to nothing and the Audio panel hides the entry. `InitialSetup_x64.bat` preserves `deps\OVRLipSyncNative` across its wipe of `deps/` and prints a note at the end when the SDK is absent.

### Submodules (`thirdparty/`)

- `thirdparty/imgui`: Dear ImGui, `docking` branch, compiled directly into the exe (not a prebuilt library).
- `thirdparty/implot`: ImPlot, pinned to the `v1.1` tag, the plots drawn over Dear ImGui. `implot.cpp` and `implot_items.cpp` are compiled directly into the exe, and its demo is not built. An ImPlot release declares the Dear ImGui versions it supports through its `IMGUI_VERSION_NUM` gates, so the two submodules move together.
- `thirdparty/ImGuizmo`: ImGuizmo, the scene's view cube and transform gizmo, pinned to the commit MikanXR pins. Only `src/ImGuizmo.cpp` is compiled into the exe, and its other widgets and examples are not built. `ImGuizmo::BeginFrame` runs right after `ImGui::NewFrame` in `App::tick`.
- `thirdparty/glm`: header-only.
- `thirdparty/nlohmann_json`: header-only, the include path is `thirdparty/nlohmann_json/single_include`.
- `thirdparty/readerwriterqueue`: header-only.
- `thirdparty/cgltf`: header-only glTF 2.0 parser behind the VRM avatar loader; the one `CGLTF_IMPLEMENTATION` is in `src/Avatar/VrmLoader.cpp`.
- `thirdparty/tracy`: the Tracy profiler, pinned to the `v0.15.0` tag. Only `public/TracyClient.cpp` is compiled, and only with `MIKANTRACK_TRACY` on, which the project generation scripts pass. The viewer under `deps/tracy` is the same version, which the two must be.
- `thirdparty/tinyfiledialogs`: native file dialogs, vendored directly (two files, not a submodule), compiled into the exe. Needs `comdlg32` and `ole32` in `target_link_libraries`.

---

## Models (`models/`)

`InitialSetup_x64.bat` downloads five model files into `models/`. `hand_landmark.onnx`, `palm_detection.onnx`, and `person_detection.onnx` come from the OpenCV Zoo project's ONNX conversions of the MediaPipe models, fetched from Hugging Face. `rtmpose_body.onnx` comes from an OpenMMLab MMDeploy SDK zip; only its `end2end.onnx` is extracted and renamed. `rtmw3d_wholebody.onnx` is the RTMW3D-X torch export published on Hugging Face (369 MB; OpenMMLab ships no SDK bundle for it). The setup script also fetches `rtm_demo.jpg` into the repo root, a reference photo the `--test-posemodel` numeric cross-check scores its RTMPose decode against; if it is missing, that check is skipped rather than failed. The script also fetches the two audio bench clips (`bench_en.wav`, `bench_ja.wav`) into `resources/audio/bench/` from the `bench-audio-1` release of this repository; a failed clip fetch warns rather than fails, and `--viseme-bench` reports a missing clip itself.

| Model file | Consumed by |
|---|---|
| `palm_detection.onnx` | `src/Vision/PalmDetector.h`/`.cpp` |
| `hand_landmark.onnx` | `src/Vision/HandLandmarkModel.h`/`.cpp` |
| `person_detection.onnx` | `src/Vision/PoseDetector.h`/`.cpp` |
| `rtmpose_body.onnx` | `src/Vision/RtmPoseModel.h`/`.cpp` |
| `rtmw3d_wholebody.onnx` | `src/Vision/RtmPoseModel.h`/`.cpp` |

`InitialSetup_x64.bat` also fetches the two CC0 VRoid Studio sample avatars into `models/avatars/` (`fem_vroid.vrm`, `masc_vroid.vrm`, about 12 MB each, skipped when already present). They ride the same post-build `models/` copy, are read by `--test-vrm-samples`, and can be loaded from the Avatar panel by their relative path.

Each model's exact tensor input/output contract (shapes, layout, normalization, decode math) is documented as a comment block directly above its wrapper class in the corresponding header, not in this doc. Read `PalmDetector.h`, `PoseDetector.h`, `HandLandmarkModel.h`, or `RtmPoseModel.h` before changing how a model is invoked.

---

## Configuring and the target

Both `GenerateProjectFiles_X64_VS*.bat` wrappers call `tools/GenerateProjectFiles_X64.bat` with their generator name, which runs `cmake .. -G <generator> -A x64 -DMIKANTRACK_TRACY=ON` from a `build/` directory it creates. A `build/` tree keeps the generator that first configured it, so switching Visual Studio versions means deleting `build/` first. Every dependency path is a fixed relative path under `deps/` or `thirdparty/` set directly in `CMakeLists.txt`. There are two cache variables, `MIKAN_BRANDING_DIR` (see Branding below) and `MIKANTRACK_TRACY`. `MIKANTRACK_TRACY` builds the Tracy profiler client into the exe. It is off by default in `CMakeLists.txt` and the generation scripts turn it on, so a local build carries the profiler and a release build is a configure that does not pass it:

```
cmake -S . -B <release build dir> -G <generator> -A x64
```

An existing `build/` tree keeps the value it was configured with until the generation script, or `cmake -S . -B build -DMIKANTRACK_TRACY=ON`, runs again. With the option on, `TracyClient.cpp` joins the exe's sources and the target is compiled with `MIKANTRACK_TRACY`, `TRACY_ENABLE`, `TRACY_ON_DEMAND`, `TRACY_ONLY_LOCALHOST`, `TRACY_NO_BROADCAST` and `TRACY_DATA_PORT` at 8286 (`MIKANTRACK_TRACY_PORT`). The port is the app's own because Tracy's default, 8086, goes to whichever program on the machine took it first, and a viewer pointed there reaches that program. With it off no Tracy header is included and no Tracy code is linked: `src/Utility/Profiler.h` and `src/Render/ProfilerGpu.h` define their macros as empty. See [debugging.md](./debugging.md) for using it.

The build defines one target: `add_executable(MikanTrack WIN32 ...)`. `WIN32` means a GUI subsystem with no console window attached, so the logger writes to `.log` files next to the exe rather than stdout being visible interactively (console output still appears when the exe is launched from a terminal). ImGui's `.cpp` files are listed directly in the target's sources and compiled into the exe; there is no separate ImGui static library.

Source collection is per-directory `file(GLOB ...)` (one glob per `src/<Dir>`, e.g. `APP_SRC`, `VISION_SRC`, `TESTS_SRC`). Because glob results are captured at configure time, adding a new `.cpp` file to an existing `src/<Dir>` requires re-running CMake configure (either `GenerateProjectFiles_X64_VS*.bat` or the equivalent `cmake ..`) before it is picked up; Visual Studio's F5 build alone will not see it. Adding a whole new `src/<Dir>/` requires editing `CMakeLists.txt`: a new `file(GLOB ...)` line, adding that variable to the `add_executable` and `source_group` calls, and adding the directory to `target_include_directories`.

OpenCV's Windows pack maps `MSVC_VERSION` to a runtime folder only up to 194x (VS2022), so under the VS2026 compiler its `OpenCVConfig.cmake` finds no binaries and configure fails. `CMakeLists.txt` presets `OpenCV_ARCH`/`OpenCV_RUNTIME` to `x64`/`vc16` for `MSVC_VERSION` 1950 and newer, the only runtime the pack ships.

Every module directory under `src/` is on the include path (see the `target_include_directories` block), so includes across the codebase use bare filenames (`#include "HandLandmarkModel.h"`) rather than directory-qualified paths.

---

## Branding

The app icon and the splash image come from one folder holding `icon.ico` and `splash.png`, chosen at configure time:

- `-DMIKAN_BRANDING_DIR=<folder>` when it is passed
- `branding/official/mikantrack` when that folder exists
- `branding/generic` otherwise

`branding/generic` is committed. `branding/official` is a gitignored checkout of the private `MikanXR/MikanBranding` repository, which holds the logo art. That art is trademark art and is never committed here, so a clone without access builds with the generic set. The checkout is not under `deps/`, which the setup script wipes and the release workflow caches. `CMakeLists.txt` prints the folder it resolved as `Branding:` at configure time and fails the configure when either file is missing. `MIKAN_BRANDING_DIR` is a cache variable, so a `build/` tree keeps a passed folder until `cmake -S . -B build -UMIKAN_BRANDING_DIR` clears it.

Both files are compiled into `MikanTrack.exe` through `src/winapp_resources.rc.in`, which CMake configures into the build tree: the icon as the exe's icon and the splash image as an `RCDATA` resource named `MIKAN_SPLASH`. Nothing is copied beside the exe, so the post-build copies and `tools/package.py` do not know about branding. `SplashWindow` (`src/App/SplashWindow.h`) decodes the resource and shows it in a borderless window from `SDL_Init` until the main window's first frame has presented.

---

## Post-build steps

A `POST_BUILD` custom command on the `MikanTrack` target:

- Copies the SDL2, GLEW, ONNX Runtime, DirectML, and OpenCV (`opencv_world4100.dll`, or the `d` debug variant under a Debug config) runtime DLLs next to the exe.
- Copies `OVRLipSync.dll` next to the exe when the optional Oculus Lipsync SDK is present.
- `copy_directory`s `models/` and `resources/` into `<exe dir>/models` and `<exe dir>/resources`.

`VS_DEBUGGER_WORKING_DIRECTORY` is set to the exe's own directory, which is the working directory Visual Studio's F5 launches with. This matters beyond the debugger: every model load path in the code is relative (`models/palm_detection.onnx`, `models/rtmpose_body.onnx`, ...), so the exe must be run with its current working directory set to the exe's directory. Running it with the repo root as the working directory silently fails to load models (or loads the wrong stale copy if the same exe was previously run correctly from its own directory).

---

## Release

A release is one file, `MikanTrack_<version>_Win64.zip`, attached to the GitHub release of the tag `v<version>`. `<version>` is the `VERSION` of the `project()` call in `CMakeLists.txt`, the one place the version is defined. The zip is flat: `MikanTrack.exe` is at its root with everything it needs at run time, and the extracted folder runs as it is.

`tools/package.py` assembles the zip from a built tree into `dist/`. Its contents are an allowlist rather than a copy of the build folder:

- `MikanTrack.exe` and the DLLs the post-build step copied, with `OVRLipSync.dll` when the build has it (`--require-ovrlipsync`, which the workflow passes, fails without it)
- the five models and two avatars `InitialSetup_x64.bat` fetches, from `models/`
- the files git tracks under `resources/`
- the VC++ runtime DLLs of the installed Visual Studio
- `NOTICE.md`

Logs, `imgui.ini`, generated calibration images, the bench clips, and models only the measurement tools load stay out. The script refuses a tree configured with `MIKANTRACK_TRACY` unless `--allow-tracy` is passed.

`.github/workflows/release.yml` cuts the release on a hosted runner. To release:

1. Set the `project()` `VERSION` in `CMakeLists.txt` and merge it to `main`.
2. Dry run: `gh workflow run release.yml`. It builds, packages, runs `--test-vmc` and `--loc-test` from the extracted zip, and uploads the zip as the `MikanTrack-Release` workflow artifact. It publishes nothing.
3. Push the tag `v<version>`. The workflow refuses a tag that does not match `CMakeLists.txt`, then creates a draft release with the zip attached.
4. Read the draft's notes and publish it by hand.

The workflow reads the Oculus Lipsync SDK from the private repository `MikanXR/OVRLipSyncNative` (the SDK's header, its Win64 libraries and its license texts) into `deps/OVRLipSyncNative`, with a read-only deploy key of that repository held in the `OVRLIPSYNC_DEPLOY_KEY` Actions secret. It deletes the folder before the dependency cache is saved, since a cache of a public repository is readable from pull request workflows. A new SDK version is a commit to that repository.

The workflow checks out `MikanXR/MikanBranding` at the tag `v1` into `branding/official` with a read-only deploy key held in the `MIKAN_BRANDING_DEPLOY_KEY` Actions secret, so the configure picks up the official set. The step is skipped when the secret is absent, as in a fork, and that run builds with the generic set. A tag build under the `MikanXR` owner fails when `branding/official/mikantrack/icon.ico` is missing after it. New art is a new tag on that repository and a change of the pinned ref here.

A release can also be cut by hand on a machine that has the SDK, from a tree configured without Tracy:

```
cmake -S . -B build-release -G "Visual Studio 18 2026" -A x64
cmake --build build-release --target MikanTrack --config Release --parallel
python tools/package.py --build-dir build-release/Release --require-ovrlipsync
gh release create v<version> dist/MikanTrack_<version>_Win64.zip --draft --generate-notes
```

The workflow caches `deps/` and `models/` under the hash of `InitialSetup_x64.bat`, so a change to that script refetches both.
