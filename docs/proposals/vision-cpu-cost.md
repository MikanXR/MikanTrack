# Proposal: the CPU cost of the camera frame path

Proposed 2026-11-15. The OpenCV thread cap and piece 5 are built, the rest is not. This document proposes three changes to the CPU the app spends on each camera frame outside the models, a measurement of whether a single-camera project needs to undistort its frames at all, and a measurement of the main thread's spin while it waits for vsync. It starts from four Tracy captures of the `MonoCameraTest` project (one 1280x720 camera at 30 fps, direct mode), the last one taken as administrator with context switches and sampled call stacks.

## Goal

The vision thread keeps up with the camera. Its `Camera process` zone is 15.8 ms at the median and 23.5 ms at p99 against a 33 ms frame. The body model, `rtmw3d_wholebody.onnx` at 14.7 ms on every second frame, is most of that, and it is out of scope here. The goal is the CPU around the models: what the process burns per frame on image work, and what the main thread spends taking each preview frame.

The capture as administrator showed where that CPU goes. Of 238k samples over 25.4 s, `opencv_world4100.dll` held 29% and the ConcRT runtime OpenCV's parallel loops run on held 12%, on 34 threads. `MikanTrack.exe`'s own code held 6%. Matched by time to the vision thread's zones:

| Vision zone | OpenCV and ConcRT samples | On the vision thread |
|---|---|---|
| `Color Undistort` (`cv::remap`) | 59.7k | 3.4k |
| `HandLandmarkModel::estimate`, outside ONNX | 14.3k | 5.2k |
| `Convert to BGR` | 12.4k | 0.6k |
| `RtmPoseModel::estimate`, outside ONNX | 3.6k | 2.2k |

`Color Undistort` takes 0.78 ms of the vision thread's time, and a quarter of every sample in the process. OpenCV split each 1280x720 remap across a pool sized to every core, and the pool spins between calls.

## What is in place

- The OpenCV thread cap. `runApp` (`src/main.cpp`) calls `cv::setNumThreads(4)` before anything else runs. With `OPENCV_FOR_THREADS_NUM` set it calls `cv::setNumThreads(-1)` instead, the one call that makes OpenCV read the variable, so a capture at another count needs no rebuild. A sweep on 2026-11-15 launched the app at each count through the automation server, recorded 20 s with `tracy-capture` and read the process's CPU time from the OS over the same window, with a camera on the room. A second pass read the CPU time over 15 s with no profiler attached:

	| OpenCV threads | 1 | 2 | 4 | 8 | every core |
	|---|---|---|---|---|---|
	| Process CPU, cores, Tracy attached | 0.40 | 0.45 | 0.42 | 0.53 | 0.76 |
	| Process CPU, cores, no profiler | 1.28 | 1.31 | 1.36 | 1.38 | 1.59 |
	| `Color Undistort` p50 | 4.11 ms | 2.36 ms | 1.44 ms | 0.90 ms | 0.77 ms |
	| `Convert to BGR` p50 | 0.30 ms | 0.19 ms | 0.13 ms | 0.12 ms | 0.13 ms |
	| `Camera process` p99 | 28.9 ms | 26.6 ms | 23.6 ms | 23.8 ms | 23.5 ms |

	Four threads cost no more CPU than one or two, within the run-to-run spread of about 0.05 cores, and hold the vision thread's p99 where every core does. Every core costs about 0.3 cores more for 0.7 ms less remap. The count stays at 4.

- The main thread uses a full core whenever no profiler is attached: 0.95 to 0.98 cores at every OpenCV count above, where the vision thread uses 0.1 to 0.27. With Tracy attached it sleeps, because Tracy's GPU collect after the swap is where the vsync wait lands, and that call blocks. Without it the wait lands in a state read that spins. It is piece 5 below, which is built.

- The undistortion. `CameraContext::process` (`src/App/CameraContext.cpp:347`) remaps every frame through `CVVideoFrameProcessor` when the camera has intrinsics, and every consumer after it reads the undistorted frame: the hand pipeline, the body tracker, the ROI quality and flicker checks, the raw frame recorder, and the preview. The maps are `CV_32FC1` pairs built by `cv::initUndistortRectifyMap` against the undistorted camera matrix. `LandmarkTo3D`, the direct solver's camera model and the published `fx`/`fy`/`cx`/`cy` all assume landmarks in that undistorted pixel space.

- A switch already exists. `VisionThread::setUndistortEnabled` turns the remap off per camera, and the intrinsics wizard uses it while it captures. With it off, the pixels are distorted but every consumer still lifts them through the undistorted matrix, so it is not a usable mode for tracking as it stands.

## Piece 1: the preview fetch into the persistent buffer

`MainWindow::update` fetches the newest preview with a fresh `VisionPreviewFrame` each time (`src/UI/MainWindow.cpp:509`), so `CameraContext::fetchPreviewFrame`'s `copyTo` allocates a new frame buffer on each preview, and the move into `m_latestPreviews` frees the old one.

- Measured: `fetchPreviewFrame` is 0.70 ms on a preview frame, against 0.13 ms for the vision thread's publish of the same frame into a buffer it reuses. The thread runs for 98.4% of the zone, so the cost is not the mutex. About two thirds of its samples are in the kernel, which is page faults on the new memory, and a third is the copy. About 0.24 ms more sits outside every zone on preview frames, most likely the free of the old buffer.

- Change: fetch straight into `m_latestPreviews[cameraIndex]`. `fetchPreviewFrame` writes the frame and the result into the slot only when the preview is fresh, so the slot keeps its last frame otherwise, which is what it does today. `copyTo` then reuses the slot's buffer.

- Expected: about 0.8 ms of main-thread CPU per preview frame, 30 times a second. The frame rate does not change, since the main thread waits about 15 ms of each frame for vsync.

- Swapping the frames under the lock instead of copying them takes the copy off the main thread too, but it changes the handoff on both sides for the last 0.13 ms. It is not proposed.

## Piece 2: fixed-point remap maps

`cv::convertMaps` turns the two `CV_32FC1` maps into a `CV_16SC2` map and a `CV_16UC1` interpolation table, once, when `CVVideoFrameProcessor` builds them. `cv::remap` then reads fixed-point offsets, which is OpenCV's fast path for bilinear remapping. The sampling position is quantized to 1/32 of a pixel, so the frame the models see changes only in its rounding.

- Expected: a cheaper `Color Undistort` at any thread count, which the capture after the thread cap measures.

- Cost: a few lines in `CVVideoFrameProcessor`. The headless tools that build the processor share it.

- It is independent of piece 4: if a single-camera project stops remapping, this still serves every project that does.

## Piece 3: undistortion on the GPU

A shader pass could remap the frame from a texture. Every consumer of the undistorted frame is on the CPU, though: the models take their input tensors from a `cv::Mat`, and the recorder, the ROI checks and the flicker check read pixels. A GPU pass therefore needs the frame uploaded, remapped and read back on each frame, about 2.7 MB each way at 1280x720, on a GL context the vision thread does not have today. That trade only pays if the remap stays expensive after pieces 2 and 4 and the thread cap, so this piece is recorded and not proposed. It becomes worth revisiting if the models move to taking GPU-resident input.

## Piece 4: whether a single-camera project needs the undistortion

### How large the correction is

The displacement the remap applies, computed from `MonoCameraTest`'s calibration (k1 0.118, k2 -0.239, `fx` 926.3, reprojection error 0.35 px), over the 1280x720 frame:

| Region | p50 | p90 | p99 | Max |
|---|---|---|---|---|
| Whole frame | 2.9 px | 5.4 px | 9.1 px | 17.8 px (corner) |
| Central half of the frame | 1.3 px | | | 3.3 px |

One pixel is 0.062 degrees at this focal length, so 5 px is a 0.3 degree ray error, about 1 cm sideways at 2 m. Whether that matters next to the landmark models' own frame-to-frame noise is what this piece has to show. A wider lens or a stronger k1 scales it up, so the answer is per camera and not one answer for every webcam.

The undistorted matrix also differs from the distorted one beyond the lens terms. Its `fy` is 939.8 against an `fx` of 928.4, where the distorted matrix has one focal length of 926.3, so turning the remap off also means lifting through the distorted matrix, not only skipping the remap.

### The modes

- Undistort the image (today). The models see corrected pixels and every consumer lifts through the undistorted matrix.

- Undistort the landmarks. The models see the raw frame, and each landmark array is corrected with `cv::undistortPoints` into the undistorted space before anything lifts it. The geometry is as exact as today's at the cost of a few hundred points per frame instead of a million pixels. The models see the lens's own curvature, which they were trained on anyway. The preview and its overlays then live in different spaces, since the overlays draw the corrected points over the raw image.

- No correction. The models see the raw frame and every consumer lifts through the distorted matrix as a pinhole. The error is the table above, on the lens's radial term alone (2.7 px at the bottom centre, 6.3 px at x 1200 on the centre row).

### The measurement

1. A headless report of each calibrated camera's displacement field: the table above, from the project's intrinsics, plus the same statistics inside the region the body occupied in a recording. It decides which cameras could be candidates at all, with no frames needed.

2. An A/B of the tracking itself, on frames. The raw frame recorder stores the frame the models consumed, which is the undistorted one, so a recording today cannot replay the raw frame. The recorder would store the frame before the remap while the A/B is wanted. The replay tools (`--replay-bodypose`, `--replay-hands`) then run each mode on the same frames and report, per landmark in the undistorted pixel space, the difference between modes against each mode's own frame-to-frame step. The direct solver's outputs (the wrist distance, the bone directions) are the second reading, since they are what the avatar shows.

3. A live switch in the single-camera camera settings that picks the mode, so the avatar can be watched in each.

## Piece 5: the main thread's vsync wait

With vsync on, the main thread's wait for the next frame lands in the first OpenGL call after the swap that has to sync with the driver. Under Tracy that is the GPU collect, which sleeps, and the process uses about 0.9 cores less. With no profiler the thread runs at a full core.

Measured 2026-11-15 by suspending the main thread every 3 ms for 10 s and walking its stack with `dbghelp`, on a launch with no profiler attached (2415 samples):

| Innermost app frame | Share | Below it |
|---|---|---|
| `GlFrameBuffer::bindFrameBuffer`, through `Scene3dPanel::renderToTarget` | 47.7% | `nvoglv64` polling `GetExitCodeThread` |
| `GlTexture::uploadBGR`, through `VideoPreviewPanel::setFrame` | 44.9% | the same loop |
| `CameraContext::fetchPreviewFrame` | 2.4% | OpenCV's copy |
| Everything else | about 5% | |

The loop polling `GetExitCodeThread` is the driver waiting on its own worker thread, which exists only while NVIDIA's threaded optimization is active. The `glGetIntegerv` calls at the top of both functions are the first sync points of the frame, so the vsync wait is spent spinning in them. The Tracy GPU collect waits by sleeping, which is why attaching Tracy hid the spin.

Built 2026-11-15. `App::tick` writes a timestamp query (`glQueryCounter`) before the swap and reads its result after it, in the `Swap wait` zone, so the vsync wait is taken there and the driver sleeps in it. `GlFrameBuffer` tracks the binding it replaces instead of reading it, and `GlTexture` puts the unpack alignment back to GL's default instead of reading the previous value.

The candidates were measured with the per-thread CPU time and the stack sampler, and for pacing in a 20 s capture:

| After the swap | Main thread, cores | Process, cores, no profiler | Frame p99 | Frames over 20 ms |
|---|---|---|---|---|
| Nothing (before) | 0.95 | 1.29 | 16.9 ms | 3 of 1216 |
| `glFenceSync` and `glClientWaitSync` | 0.95 | 1.22 with Tracy attached | 20.3 ms | 17 of 1210 |
| Timestamp written after the swap, read at once | 0.07 | 0.42 | 19.9 ms | 12 of 1215 |
| Timestamp written before the swap, read after it | under 0.07 | 0.40 | 16.9 ms | 2 of 1213 |

- The fence wait goes through the same worker poll and spins. Removing the app's state reads alone moves the spin to the next state read, which is ImGui's backend's.

- A timestamp written after the swap waits for the present itself, which leaves no frame queued and costs pacing. Written before the swap, the read waits on the driver's worker as the state reads did, but by sleeping, and the frame queue is as before.

- Not built: the frame timer taking the wait before the swap, and an NVIDIA application profile turning threaded optimization off. The query wait made both unnecessary.

- The renderers' own save and restore of GL state (the skinned mesh renderer's `draw` reads about twenty values, the line renderer two) are left as they are. Each read is a round trip to the driver under threaded optimization, but none is a wait once the swap's has been taken, and replacing them changes the state contract between the renderers.

## Order

1. Piece 1, which is independent of the rest.
2. Piece 2. At the capped count it saves on a 1.4 ms remap instead of a 0.8 ms one.
3. Piece 4's displacement report, then its frame A/B if the report leaves the question open.

## Open questions

- Which modes piece 4 offers if the A/B finds the correction below the noise: landmark undistortion alone, or no correction as well.
- Whether the raw frame recorder stores the frame before the remap permanently, or only during the A/B. Stored raw, a recording can be replayed under any later calibration.
- Whether piece 4's switch is per camera in every project, or for single-camera projects alone. A multi-camera project triangulates across cameras, so its tolerance for a pixel error is not the direct solver's.
