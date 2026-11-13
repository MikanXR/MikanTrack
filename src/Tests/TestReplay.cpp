#include "TestCommon.h"

#include "FaceRecorder.h"

static int runReplayTest(const TestArgs& args)
{
	int result= 0;
	const std::string recordingPath=
		(std::filesystem::temp_directory_path() / "mikan-test-replay.jsonl").string();
	const std::string faceSidecarPath= TrackingRecording::makeFaceSidecarPath(recordingPath);
	// The phone's samples, at their own rate between the frames: odd-valued
	// floats so a lossy round trip shows, and every fifth one a lost face
	std::vector<RecordedFaceSample> faceSamples;
	for (int sampleIndex= 0; sampleIndex < 90; ++sampleIndex)
	{
		RecordedFaceSample sample;
		sample.sequence= 1000 + (uint64_t)sampleIndex;
		sample.timestampMs= 5.0 + 11.1 * (double)sampleIndex;
		sample.bTracked= sampleIndex % 5 != 0;
		for (int column= 0; column < ARKIT_BLENDSHAPE_COUNT; ++column)
			sample.blendshapes[(size_t)column]= 1.f / (float)(3 + column + sampleIndex) - 0.01f;
		// Every third one without a head, as a phone format that sends none
		sample.bHasHead= sampleIndex % 3 != 0;
		if (sample.bHasHead)
			sample.headEulerDegrees= glm::vec3(1.f / (float)(7 + sampleIndex), -12.3f, 0.1f * (float)sampleIndex);
		faceSamples.push_back(sample);
	}

	// -- Synthetic 2-camera rig ------------------------------------
	const float fx= 800.f, fy= 800.f, cx= 640.f, cy= 360.f;

	AppConfig config;
	config.cameras.clear();
	// Camera 0: overhead at (0,0,0.8) looking straight down
	// Camera 1: side at (0,0.7,0.15) looking along -Y
	const glm::dmat4 camFromWorld[2]= {
		glm::dmat4(glm::dvec4(1, 0, 0, 0), glm::dvec4(0, -1, 0, 0), glm::dvec4(0, 0, -1, 0),
				   glm::dvec4(0, 0, 0.8, 1)),
		glm::dmat4(glm::dvec4(-1, 0, 0, 0), glm::dvec4(0, 0, -1, 0), glm::dvec4(0, -1, 0, 0),
				   glm::dvec4(0, 0.15, 0.7, 1)),
	};
	for (int cameraIndex= 0; cameraIndex < 2; ++cameraIndex)
	{
		CameraProfile profile;
		profile.intrinsics.present= true;
		profile.intrinsics.intrinsics.pixel_width= 1280;
		profile.intrinsics.intrinsics.pixel_height= 720;
		profile.intrinsics.intrinsics.undistorted_camera_matrix.x0= fx;
		profile.intrinsics.intrinsics.undistorted_camera_matrix.y1= fy;
		profile.intrinsics.intrinsics.undistorted_camera_matrix.z0= cx;
		profile.intrinsics.intrinsics.undistorted_camera_matrix.z1= cy;
		profile.extrinsics.present= true;
		profile.extrinsics.markerFromCamera= glm::inverse(camFromWorld[cameraIndex]);
		config.cameras.push_back(profile);
	}

	// Canonical hand model, wrist at origin, slightly non-planar (the
	// same shape the PnP self-test uses)
	std::array<glm::vec3, HAND_LANDMARK_COUNT> modelPoints;
	for (int lm= 0; lm < HAND_LANDMARK_COUNT; ++lm)
	{
		const float along= 0.02f + 0.15f * (float)(lm % 4) / 4.f * (lm >= 1 ? 1.f : 0.f);
		const float spread= ((float)(lm / 4) - 2.f) * 0.02f;
		const float curl= 0.012f * (float)(lm % 4);
		modelPoints[lm]= glm::vec3(spread, along, curl);
	}
	modelPoints[0]= glm::vec3(0.f);
	const float boneLength= glm::length(modelPoints[(int)eHandLandmark::MIDDLE_MCP]);
	config.handScale.present= true;
	config.handScale.refLengthMeters= boneLength;

	auto handTransform= [](int frameIdx) {
		const float t= (float)frameIdx / 30.f;
		glm::mat4 xf= glm::translate(glm::mat4(1.f),
									 glm::vec3(0.05f * sinf(t), 0.03f * cosf(t), 0.12f));
		xf= glm::rotate(xf, 0.4f * sinf(0.7f * t), glm::vec3(0.f, 0.f, 1.f));
		xf= glm::rotate(xf, 0.3f * cosf(0.9f * t), glm::vec3(1.f, 0.f, 0.f));
		return xf;
	};

	// Builds one camera's recorded input for a frame (the source of
	// truth for both the live pass and, via the file, replay)
	auto buildCameraInput= [&](int cameraIndex, int frameIdx, bool bValid) {
		RecordedCameraInput input;
		input.cameraIndex= cameraIndex;
		input.timestampMs= 1000.0 + frameIdx * 33.0;
		input.valid= bValid;
		input.frameIndex= frameIdx;
		input.frameWidth= 1280;
		input.frameHeight= 720;
		input.captureFps= 30.f;
		input.inferenceMs= 5.f;
		if (!bValid)
			return input;

		input.refLengthMeters= boneLength * (1.f + 0.0005f * (float)frameIdx);

		const glm::mat4 worldFromModel= handTransform(frameIdx);
		RecordedHandInput& hand= input.hands[(int)eHandSide::Right];
		hand.tracked= true;
		hand.side= (int)eHandSide::Right;
		hand.slotId= 0;
		hand.presence= 0.95f;
		hand.handednessScore= 0.9f;
		hand.rightProb= 0.9f;
		hand.modelPoints= modelPoints;
		for (int lm= 0; lm < HAND_LANDMARK_COUNT; ++lm)
		{
			const glm::vec3 worldPoint=
				glm::vec3(worldFromModel * glm::vec4(modelPoints[lm], 1.f));
			const glm::vec3 cameraPoint=
				glm::vec3(camFromWorld[cameraIndex] * glm::dvec4(glm::dvec3(worldPoint), 1.0));
			hand.imagePoints[lm]= glm::vec3(fx * cameraPoint.x / cameraPoint.z + cx,
											fy * cameraPoint.y / cameraPoint.z + cy, 0.f);
		}
		return input;
	};

	// The display-only stage state a frame carries: every field set, odd-valued
	// floats so a lossy round trip shows
	auto makeTelemetry= [](int frameIdx) {
		const float k= 0.001f * (float)frameIdx;
		RecordedTelemetry telemetry;
		telemetry.present= true;
		telemetry.path= 2 + frameIdx % 3;
		for (int phase= 0; phase < RecordedTelemetry::k_phaseCount; ++phase)
			telemetry.phaseMs[phase]= 1.f / (float)(3 + phase) + k;
		telemetry.totalMs= 12.345f + k;
		telemetry.oscRunning= frameIdx % 2 == 1;
		telemetry.oscSendRateHz= 59.97f + k;
		telemetry.oscLastPacketCount= 3 + frameIdx;
		telemetry.oscLastFrameBytes= 1400 + frameIdx;
		telemetry.dominantCamera[0]= -1;
		telemetry.dominantCamera[1]= 1;
		telemetry.autoHandScaleFactor= 1.0123f + k;
		telemetry.cameraCount= 2;
		for (int cameraIndex= 0; cameraIndex < 2; ++cameraIndex)
		{
			telemetry.observationConfidence[cameraIndex][0]= cameraIndex == 0 ? -1.f : 0.7f + k;
			telemetry.observationConfidence[cameraIndex][1]= 0.9f / (1.3f + (float)cameraIndex) + k;
			RecordedTelemetry::Camera& camera= telemetry.cameras[cameraIndex];
			camera.deviceOpen= true;
			camera.streaming= cameraIndex == 0 || frameIdx % 2 == 0;
			camera.deviceFrameRate= 29.97f + (float)cameraIndex + k;
			camera.droppedFrames= 7 * cameraIndex + frameIdx;
			camera.queuedFrames= cameraIndex + 1;
			camera.callbackCopyMs= 0.333f + k;
			camera.trackingEnabled= cameraIndex != 1 || frameIdx % 4 != 0;
			camera.handPipelineReady= true;
			camera.bodyModelLoaded= cameraIndex == 1;
		}
		telemetry.imuEnabled= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			RecordedImuStatus& imu= telemetry.imu[sideIndex];
			imu.deviceConnected= true;
			imu.streaming= sideIndex == 0;
			imu.calibrated= true;
			imu.orientationValid= sideIndex == 1;
			imu.sampleRateHz= 99.9f + (float)sideIndex + k;
			imu.batteryLevel= 0.55f + 0.1f * (float)sideIndex;
			imu.yawDriftDegPerSecond= -0.0123f * (float)(sideIndex + 1) + k;
			imu.rollErrorDegrees= sideIndex == 0 ? -999.f : 3.21f + k;
		}
		telemetry.face.enabled= true;
		telemetry.face.bound= true;
		telemetry.face.streaming= frameIdx % 2 == 0;
		telemetry.face.faceTracked= true;
		telemetry.face.datagramsPerSecond= 59.94f + k;
		telemetry.audio.enabled= true;
		telemetry.audio.deviceOpen= true;
		telemetry.audio.backendReady= frameIdx % 5 != 0;
		telemetry.audio.gateOpen= frameIdx % 2 == 1;
		telemetry.audio.levelDbfs= -23.456f + k;
		telemetry.audio.inferMsEma= 1.789f + k;
		return telemetry;
	};
	auto makeTimings= [](int frameIdx, int cameraIndex) {
		const float k= 0.001f * (float)frameIdx + 0.0001f * (float)cameraIndex;
		TrackingFrameResult::CaptureTimings timings;
		timings.queueAgeMs= 4.1f + k;
		timings.convertMs= 1.3f + k;
		timings.undistortMs= 0.7f + k;
		timings.flickerMs= 0.11f + k;
		timings.bodyPoseMs= 6.6f + k;
		timings.roiQualityMs= 0.23f + k;
		timings.liftMs= 0.37f + k;
		timings.publishMs= 0.19f + k;
		timings.totalMs= 13.9f + k;
		return timings;
	};
	auto timingsEqual= [](const TrackingFrameResult::CaptureTimings& a, const TrackingFrameResult::CaptureTimings& b) {
		return a.queueAgeMs == b.queueAgeMs && a.convertMs == b.convertMs && a.undistortMs == b.undistortMs &&
			   a.flickerMs == b.flickerMs && a.bodyPoseMs == b.bodyPoseMs && a.roiQualityMs == b.roiQualityMs &&
			   a.liftMs == b.liftMs && a.publishMs == b.publishMs && a.totalMs == b.totalMs;
	};
	// Every field of the block, the unused camera slots included
	auto countTelemetryMismatches= [](const RecordedTelemetry& a, const RecordedTelemetry& b) {
		int mismatches= 0;
		auto expect= [&mismatches](bool bEqual) { mismatches+= bEqual ? 0 : 1; };
		expect(a.present == b.present);
		expect(a.path == b.path);
		for (int phase= 0; phase < RecordedTelemetry::k_phaseCount; ++phase)
			expect(a.phaseMs[phase] == b.phaseMs[phase]);
		expect(a.totalMs == b.totalMs);
		expect(a.oscRunning == b.oscRunning);
		expect(a.oscSendRateHz == b.oscSendRateHz);
		expect(a.oscLastPacketCount == b.oscLastPacketCount);
		expect(a.oscLastFrameBytes == b.oscLastFrameBytes);
		expect(a.dominantCamera[0] == b.dominantCamera[0] && a.dominantCamera[1] == b.dominantCamera[1]);
		expect(a.autoHandScaleFactor == b.autoHandScaleFactor);
		expect(a.cameraCount == b.cameraCount);
		for (int cameraIndex= 0; cameraIndex < RecordedTelemetry::k_maxCameras; ++cameraIndex)
		{
			expect(a.observationConfidence[cameraIndex][0] == b.observationConfidence[cameraIndex][0]);
			expect(a.observationConfidence[cameraIndex][1] == b.observationConfidence[cameraIndex][1]);
			const RecordedTelemetry::Camera& ca= a.cameras[cameraIndex];
			const RecordedTelemetry::Camera& cb= b.cameras[cameraIndex];
			expect(ca.deviceOpen == cb.deviceOpen && ca.streaming == cb.streaming);
			expect(ca.deviceFrameRate == cb.deviceFrameRate && ca.callbackCopyMs == cb.callbackCopyMs);
			expect(ca.droppedFrames == cb.droppedFrames && ca.queuedFrames == cb.queuedFrames);
			expect(ca.trackingEnabled == cb.trackingEnabled && ca.handPipelineReady == cb.handPipelineReady &&
				   ca.bodyModelLoaded == cb.bodyModelLoaded);
		}
		expect(a.imuEnabled == b.imuEnabled);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const RecordedImuStatus& ia= a.imu[sideIndex];
			const RecordedImuStatus& ib= b.imu[sideIndex];
			expect(ia.deviceConnected == ib.deviceConnected && ia.streaming == ib.streaming &&
				   ia.calibrated == ib.calibrated && ia.orientationValid == ib.orientationValid);
			expect(ia.sampleRateHz == ib.sampleRateHz && ia.batteryLevel == ib.batteryLevel);
			expect(ia.yawDriftDegPerSecond == ib.yawDriftDegPerSecond && ia.rollErrorDegrees == ib.rollErrorDegrees);
		}
		expect(a.face.enabled == b.face.enabled && a.face.bound == b.face.bound && a.face.streaming == b.face.streaming &&
			   a.face.faceTracked == b.face.faceTracked && a.face.datagramsPerSecond == b.face.datagramsPerSecond);
		expect(a.audio.enabled == b.audio.enabled && a.audio.deviceOpen == b.audio.deviceOpen &&
			   a.audio.backendReady == b.audio.backendReady && a.audio.gateOpen == b.audio.gateOpen &&
			   a.audio.levelDbfs == b.audio.levelDbfs && a.audio.inferMsEma == b.audio.inferMsEma);
		return mismatches;
	};

	// -- Live pass: the exact VisionThread call sequence, recorded
	// through a real TrackingRecorder ------------------------------
	std::vector<TrackingFrameResult> liveOutputs;
	{
		HandFusion fusion;
		// Build the fusion config exactly as the vision thread does
		fusion.configure(makeHandFusionConfig(config));
		fusion.resetTransientState();

		std::array<LandmarkTo3D, 2> landmarkTo3D;
		std::array<CameraFrameResult, 2> mirrors;
		for (int cameraIndex= 0; cameraIndex < 2; ++cameraIndex)
		{
			landmarkTo3D[cameraIndex].configure(config.cameras[cameraIndex].intrinsics.intrinsics,
												config.handScale.refLengthMeters);
			landmarkTo3D[cameraIndex].resetTransientState();
			mirrors[cameraIndex].cameraIndex= cameraIndex;
		}

		RecordingHeader header;
		header.formatVersion= TrackingRecording::k_formatVersion;
		header.appConfigJsonText= config.toJsonString();
		header.cameraCount= 2;
		header.audio.present= true;
		header.audio.fileName= "mikan-test-replay.wav";
		header.audio.sampleRate= AUDIO_SAMPLE_RATE;
		header.audio.startSampleIndex= 123904;
		header.audio.startTimestampMs= 987.5;
		header.face.present= true;
		header.face.fileName= std::filesystem::path(faceSidecarPath).filename().string();
		header.face.startTimestampMs= 4.5;

		FaceRecorder faceRecorder;
		if (!faceRecorder.start(faceSidecarPath))
		{
			MIKAN_LOG_ERROR("test-replay") << "Face recorder failed to open " << faceSidecarPath;
			return 1;
		}
		for (const RecordedFaceSample& sample : faceSamples)
			faceRecorder.enqueue(sample);
		faceRecorder.stop();

		TrackingRecorder recorder;
		if (!recorder.start(recordingPath, TrackingRecording::headerToJson(header)))
		{
			MIKAN_LOG_ERROR("test-replay") << "Recorder failed to open " << recordingPath;
			return 1;
		}

		for (int frameIdx= 0; frameIdx < 60; ++frameIdx)
		{
			RecordedFrame frame;
			frame.seq= frameIdx;

			// Freshness plan: frame 0 = cam0 only, invalid (the
			// passthrough case); i%5==3 = cam0 only (stale-candidate
			// case); frame 20 = cam1 fresh-but-invalid
			std::vector<RecordedCameraInput> fresh;
			if (frameIdx == 0)
			{
				fresh.push_back(buildCameraInput(0, frameIdx, false));
			}
			else
			{
				fresh.push_back(buildCameraInput(0, frameIdx, true));
				if (frameIdx == 20)
					fresh.push_back(buildCameraInput(1, frameIdx, false));
				else if (frameIdx % 5 != 3)
					fresh.push_back(buildCameraInput(1, frameIdx, true));
			}

			// Capture timings on every even frame's cameras (display only)
			if (frameIdx % 2 == 0)
			{
				for (RecordedCameraInput& input : fresh)
				{
					input.bHaveCaptureTimings= true;
					input.captureTimings= makeTimings(frameIdx, input.cameraIndex);
				}
			}

			// Per-camera stage (mirrors VisionThread::processCameraFrame)
			for (const RecordedCameraInput& input : fresh)
			{
				const int cameraIndex= input.cameraIndex;
				TrackingFrameResult camResult;
				camResult.frameIndex= input.frameIndex;
				camResult.timestampMs= input.timestampMs;
				camResult.frameWidth= input.frameWidth;
				camResult.frameHeight= input.frameHeight;
				camResult.captureFps= input.captureFps;
				camResult.inferenceMs= input.inferenceMs;
				if (input.valid)
				{
					for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
					{
						const RecordedHandInput& recordedHand= input.hands[sideIndex];
						TrackedHand& hand= camResult.hands[sideIndex];
						hand.tracked= recordedHand.tracked;
						if (!recordedHand.tracked)
							continue;
						hand.side= (eHandSide)recordedHand.side;
						hand.slotId= recordedHand.slotId;
						hand.presence= recordedHand.presence;
						hand.handednessScore= recordedHand.handednessScore;
						hand.rightProb= recordedHand.rightProb;
						hand.imagePoints= recordedHand.imagePoints;
						hand.modelPoints= recordedHand.modelPoints;
					}
					landmarkTo3D[cameraIndex].setRefLengthMeters(input.refLengthMeters);
					landmarkTo3D[cameraIndex].process(camResult);
					applyWorldTransform(camResult,
										config.cameras[cameraIndex].extrinsics.markerFromCamera);
				}

				CameraFrameResult& mirror= mirrors[cameraIndex];
				mirror.valid= input.valid;
				mirror.timestampMs= input.timestampMs;
				mirror.hasExtrinsics= true;
				mirror.markerFromCamera= config.cameras[cameraIndex].extrinsics.markerFromCamera;
				mirror.hasIntrinsics= true;
				mirror.fx= fx;
				mirror.fy= fy;
				mirror.cx= cx;
				mirror.cy= cy;
				mirror.result= camResult;

				frame.freshCameras.push_back(input);
			}

			// Fusion stage over all mirrors (fresh or stale)
			double nowMs= 0.0;
			for (const CameraFrameResult& mirror : mirrors)
				nowMs= std::max(nowMs, mirror.timestampMs);
			frame.nowTimestampMs= nowMs;

			bool bAnyWorld= false;
			for (const CameraFrameResult& mirror : mirrors)
				bAnyWorld|= mirror.valid && mirror.hasExtrinsics;

			TrackingFrameResult fused;
			if (bAnyWorld)
			{
				std::vector<const CameraFrameResult*> candidates;
				for (const CameraFrameResult& mirror : mirrors)
					candidates.push_back(&mirror);
				fusion.fuse(candidates, nowMs, fused);
				frame.bFused= true;
			}
			else
			{
				fused= mirrors[0].result;
				frame.bFused= false;
			}

			TrackingRecording::snapshotFusedOutput(fused, frame.outPoses);
			frame.checksum= TrackingRecording::computeFusedChecksum(fused);
			liveOutputs.push_back(fused);

			// The mouth stage's frame on every other iteration, every third
			// one from before the sidecar (untied)
			if (frameIdx % 2 == 0)
			{
				frame.mouth.present= true;
				frame.mouth.audioFrameIndex= frameIdx % 3 == 0 ? -1 : 300 + frameIdx;
				for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
					frame.mouth.frame.weights[viseme]= 0.01f * (float)(viseme + 1) + 0.001f * (float)frameIdx;
			}
			// The stage telemetry on every third frame, outside the checksum
			if (frameIdx % 3 == 1)
				frame.telemetry= makeTelemetry(frameIdx);
			recorder.enqueueFrame(std::move(frame));
		}
		recorder.stop();
	}

	// -- (a) Replay pass: load + runAll must match bit-exactly -----
	TrackingReplay replay;
	std::string error;
	if (!replay.load(recordingPath, error))
	{
		MIKAN_LOG_ERROR("test-replay") << "(a) FAILED: load: " << error;
		return 1;
	}
	replay.runAll();

	MIKAN_LOG_INFO("test-replay")
		<< "(a) frames=" << replay.getFrameCount()
		<< " divergent=" << replay.getDivergentFrames().size();
	if (replay.getFrameCount() != 60 || !replay.getDivergentFrames().empty())
	{
		MIKAN_LOG_ERROR("test-replay") << "(a) FAILED: replay must match every checksum";
		result= 1;
	}
	if (replay.getFrame(0).recorded->bFused)
	{
		MIKAN_LOG_ERROR("test-replay") << "(a) FAILED: frame 0 must be a passthrough frame";
		result= 1;
	}

	// Float-exact comparison of the replayed poses vs the live pass
	{
		int mismatchCount= 0;
		for (int frameIdx= 0; frameIdx < replay.getFrameCount(); ++frameIdx)
		{
			const TrackingFrameResult& live= liveOutputs[frameIdx];
			const TrackingFrameResult& replayed= replay.getFrame(frameIdx).replayedFused;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const HandPose& livePose= live.poses[sideIndex];
				const HandPose& replayedPose= replayed.poses[sideIndex];
				if (livePose.tracked != replayedPose.tracked)
				{
					++mismatchCount;
					continue;
				}
				if (!livePose.tracked)
					continue;
				if (memcmp(&livePose.palmPositionWorld, &replayedPose.palmPositionWorld,
						   sizeof(glm::vec3)) != 0 ||
					memcmp(&livePose.palmOrientationWorld, &replayedPose.palmOrientationWorld,
						   sizeof(glm::quat)) != 0 ||
					memcmp(livePose.fingers.data(), replayedPose.fingers.data(),
						   sizeof(FingerAngles) * FINGER_COUNT) != 0)
					++mismatchCount;
			}
		}
		MIKAN_LOG_INFO("test-replay") << "(b) float-exact pose mismatches=" << mismatchCount;
		if (mismatchCount != 0)
		{
			MIKAN_LOG_ERROR("test-replay")
				<< "(b) FAILED: JSONL round-trip must be byte-lossless";
			result= 1;
		}
	}

	// -- (c) What-if must diverge while plain replay matches -------
	{
		TrackingReplay::WhatIfParams params= replay.makeDefaultWhatIfParams();
		// Any parameter edit that reaches the solve works as the probe; the
		// hand scale multiplier touches every camera-space lift
		params.refLengthScale= 1.1f;
		replay.runWhatIf(params);

		int whatIfDifferent= 0;
		for (int frameIdx= 0; frameIdx < replay.getFrameCount(); ++frameIdx)
		{
			const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIdx);
			if (frame.bHasWhatIf &&
				TrackingRecording::computeFusedChecksum(frame.whatIfFused) !=
					frame.recorded->checksum)
				++whatIfDifferent;
		}
		MIKAN_LOG_INFO("test-replay") << "(c) what-if divergent frames=" << whatIfDifferent;
		if (whatIfDifferent == 0)
		{
			MIKAN_LOG_ERROR("test-replay")
				<< "(c) FAILED: a what-if parameter edit must change the output";
			result= 1;
		}
	}

	// -- (c2) Camera mask: one camera alone still tracks but never
	// triangulates, the masked camera's mirror stays untouched, and an
	// all-enabled mask reproduces the faithful pass bit-exactly ---------
	{
		TrackingReplay::WhatIfParams params= replay.makeDefaultWhatIfParams();
		params.cameraEnabled= {true, false};
		replay.runWhatIf(params);

		int monoTracked= 0;
		int monoStereo= 0;
		int maskedCameraTracked= 0;
		for (int frameIdx= 0; frameIdx < replay.getFrameCount(); ++frameIdx)
		{
			const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIdx);
			if (!frame.bHasWhatIf)
				continue;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const HandPose& pose= frame.whatIfFused.poses[sideIndex];
				if (pose.tracked && pose.hasWorldPose)
					++monoTracked;
				if (pose.tracked && pose.stereoTriangulated)
					++monoStereo;
				if (frame.whatIfPerCamera.size() > 1 && frame.whatIfPerCamera[1].hands[sideIndex].tracked)
					++maskedCameraTracked;
			}
		}
		MIKAN_LOG_INFO("test-replay")
			<< "(c2) masked pass tracked=" << monoTracked << " stereo=" << monoStereo
			<< " maskedCameraTracked=" << maskedCameraTracked;
		if (monoTracked == 0 || monoStereo != 0 || maskedCameraTracked != 0)
		{
			MIKAN_LOG_ERROR("test-replay")
				<< "(c2) FAILED: one camera alone must track monocularly and the masked camera must stay idle";
			result= 1;
		}

		params.cameraEnabled= {true, true};
		replay.runWhatIf(params);
		int mismatches= 0;
		for (int frameIdx= 0; frameIdx < replay.getFrameCount(); ++frameIdx)
		{
			const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIdx);
			if (frame.bHasWhatIf &&
				TrackingRecording::computeFusedChecksum(frame.whatIfFused) != frame.recorded->checksum)
				++mismatches;
		}
		MIKAN_LOG_INFO("test-replay") << "(c2) all-enabled mask checksum mismatches=" << mismatches;
		if (mismatches != 0)
		{
			MIKAN_LOG_ERROR("test-replay") << "(c2) FAILED: an all-enabled mask must equal the faithful pass";
			result= 1;
		}
	}

	// -- (d) Corrupted checksum must be reported -------------------
	{
		const std::string corruptPath=
			(std::filesystem::temp_directory_path() / "mikan-test-replay-corrupt.jsonl").string();
		std::ifstream inFile(recordingPath);
		std::ofstream outFile(corruptPath, std::ios::trunc);
		std::string line;
		int lineIndex= 0;
		while (std::getline(inFile, line))
		{
			if (lineIndex == 41) // an arbitrary mid-recording frame line
			{
				nlohmann::json j= nlohmann::json::parse(line);
				j["checksum"]= "0000000000000000";
				line= j.dump();
			}
			outFile << line << '\n';
			++lineIndex;
		}
		inFile.close();
		outFile.close();

		TrackingReplay corruptReplay;
		if (!corruptReplay.load(corruptPath, error))
		{
			MIKAN_LOG_ERROR("test-replay") << "(d) FAILED: corrupt-copy load: " << error;
			result= 1;
		}
		else
		{
			corruptReplay.runAll();
			MIKAN_LOG_INFO("test-replay")
				<< "(d) corrupt-copy divergent=" << corruptReplay.getDivergentFrames().size();
			if (corruptReplay.getDivergentFrames().size() != 1)
			{
				MIKAN_LOG_ERROR("test-replay")
					<< "(d) FAILED: exactly the corrupted frame must diverge";
				result= 1;
			}
		}
	}

	// -- (e) The audio sidecar header and the mouth frames round-trip --
	{
		const RecordedAudioSidecar& audio= replay.getHeader().audio;
		if (!audio.present || audio.fileName != "mikan-test-replay.wav" || audio.sampleRate != AUDIO_SAMPLE_RATE ||
			audio.startSampleIndex != 123904 || audio.startTimestampMs != 987.5)
		{
			MIKAN_LOG_ERROR("test-replay") << "(e) FAILED: the audio sidecar header must round-trip";
			result= 1;
		}

		int mouthMismatches= 0;
		for (int frameIdx= 0; frameIdx < replay.getFrameCount(); ++frameIdx)
		{
			const RecordedMouth& mouth= replay.getFrame(frameIdx).recorded->mouth;
			if (frameIdx % 2 != 0)
			{
				if (mouth.present)
					++mouthMismatches;
				continue;
			}
			if (!mouth.present || mouth.audioFrameIndex != (frameIdx % 3 == 0 ? -1 : 300 + frameIdx))
			{
				++mouthMismatches;
				continue;
			}
			for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
			{
				if (mouth.frame.weights[viseme] != 0.01f * (float)(viseme + 1) + 0.001f * (float)frameIdx)
					++mouthMismatches;
			}
		}
		MIKAN_LOG_INFO("test-replay") << "(e) mouth frame mismatches=" << mouthMismatches;
		if (mouthMismatches != 0)
		{
			MIKAN_LOG_ERROR("test-replay") << "(e) FAILED: the recorded mouth frames must round-trip float-exact";
			result= 1;
		}

		// A header without the block, and a frame line without the block (an
		// older recording), load as absent
		RecordingHeader plainHeader;
		plainHeader.formatVersion= TrackingRecording::k_formatVersion;
		plainHeader.appConfigJsonText= "{}";
		plainHeader.cameraCount= 1;
		RecordingHeader loadedHeader;
		nlohmann::json plainJson= TrackingRecording::headerToJson(plainHeader);
		RecordedFrame plainFrame;
		nlohmann::json plainFrameJson= TrackingRecording::frameToJson(plainFrame);
		RecordedFrame loadedFrame;
		if (plainJson.contains("audio") || !TrackingRecording::headerFromJson(plainJson, loadedHeader) ||
			loadedHeader.audio.present || plainFrameJson.contains("mouth") ||
			!TrackingRecording::frameFromJson(plainFrameJson, loadedFrame) || loadedFrame.mouth.present)
		{
			MIKAN_LOG_ERROR("test-replay") << "(e) FAILED: a recording without audio must load as absent";
			result= 1;
		}
		if (TrackingRecording::makeAudioSidecarPath("recordings/2026-01-01_12-00-00.jsonl") !=
			"recordings/2026-01-01_12-00-00.wav")
		{
			MIKAN_LOG_ERROR("test-replay") << "(e) FAILED: the sidecar is named after the recording";
			result= 1;
		}
	}

	// -- (f) The face sidecar round-trips and samples onto the records --
	{
		const RecordedFaceSidecar& face= replay.getHeader().face;
		if (!face.present || face.fileName != "mikan-test-replay.face.jsonl" || face.startTimestampMs != 4.5)
		{
			MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: the face sidecar header must round-trip";
			result= 1;
		}
		const std::vector<RecordedFaceSample>& loaded= replay.getFaceSamples();
		int faceMismatches= loaded.size() == faceSamples.size() ? 0 : 1;
		for (size_t sampleIndex= 0; sampleIndex < loaded.size() && sampleIndex < faceSamples.size(); ++sampleIndex)
		{
			const RecordedFaceSample& a= faceSamples[sampleIndex];
			const RecordedFaceSample& b= loaded[sampleIndex];
			if (a.sequence != b.sequence || a.timestampMs != b.timestampMs || a.bTracked != b.bTracked ||
				a.bHasHead != b.bHasHead || a.headEulerDegrees != b.headEulerDegrees ||
				memcmp(a.blendshapes.data(), b.blendshapes.data(), sizeof(float) * ARKIT_BLENDSHAPE_COUNT) != 0)
				++faceMismatches;
		}
		MIKAN_LOG_INFO("test-replay") << "(f) face samples=" << loaded.size() << " mismatches=" << faceMismatches;
		if (faceMismatches != 0)
		{
			MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: the face sidecar must round-trip float-exact";
			result= 1;
		}

		// By time: each record takes the newest sample at or before it
		int samplingMismatches= 0;
		for (int frameIdx= 0; frameIdx < replay.getFrameCount(); ++frameIdx)
		{
			const double frameMs= replay.getFrame(frameIdx).recorded->nowTimestampMs;
			const RecordedFaceSample* found= FaceSidecar::findAtOrBefore(loaded, frameMs);
			const RecordedFaceSample* expected= nullptr;
			for (const RecordedFaceSample& sample : faceSamples)
			{
				if (sample.timestampMs <= frameMs)
					expected= &sample;
			}
			const bool bSame= (found == nullptr && expected == nullptr) ||
							  (found != nullptr && expected != nullptr && found->sequence == expected->sequence);
			samplingMismatches+= bSame ? 0 : 1;
		}
		if (samplingMismatches != 0 || FaceSidecar::findAtOrBefore(loaded, 4.9) != nullptr ||
			FaceSidecar::findAtOrBefore(loaded, 5.0) != &loaded[0] ||
			FaceSidecar::findAtOrBefore(loaded, 1e9) != &loaded.back())
		{
			MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: the sidecar must be sampled by time onto the records";
			result= 1;
		}

		if (TrackingRecording::makeFaceSidecarPath("recordings/2026-01-01_12-00-00.jsonl") !=
			"recordings/2026-01-01_12-00-00.face.jsonl")
		{
			MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: the face sidecar is named after the recording";
			result= 1;
		}

		// A recording list keeps the takes and leaves out the sidecars written
		// beside them, by name: a take, its face and audio sidecars, and takes
		// whose names merely contain "face"
		{
			const std::string take= "recordings/2026-01-01_12-00-00.jsonl";
			const bool bTakeIsTake=
				TrackingRecording::isRecordingFile(take) && !TrackingRecording::isRecordingSidecarFile(take);
			const bool bFaceIsSidecar= TrackingRecording::isRecordingSidecarFile(
										   TrackingRecording::makeFaceSidecarPath(take)) &&
				!TrackingRecording::isRecordingFile(TrackingRecording::makeFaceSidecarPath(take));
			const bool bAudioIsSidecar=
				TrackingRecording::isRecordingSidecarFile(TrackingRecording::makeAudioSidecarPath(take)) &&
				!TrackingRecording::isRecordingFile(TrackingRecording::makeAudioSidecarPath(take));
			const bool bFaceWordIsTake= TrackingRecording::isRecordingFile("recordings/face.jsonl") &&
				TrackingRecording::isRecordingFile("recordings/my_face_take.jsonl") &&
				TrackingRecording::isRecordingFile("recordings/2026-01-01_face.jsonl") &&
				!TrackingRecording::isRecordingSidecarFile("recordings/face.jsonl");
			const bool bOtherIsNeither= !TrackingRecording::isRecordingFile("recordings/notes.txt") &&
				!TrackingRecording::isRecordingFile("recordings/2026-01-01_12-00-00_frames");
			MIKAN_LOG_INFO("test-replay") << "(f) sidecar rule: take " << bTakeIsTake << " face sidecar "
										  << bFaceIsSidecar << " audio sidecar " << bAudioIsSidecar
										  << " face in a take's name " << bFaceWordIsTake << " other " << bOtherIsNeither;
			if (!bTakeIsTake || !bFaceIsSidecar || !bAudioIsSidecar || !bFaceWordIsTake || !bOtherIsNeither)
			{
				MIKAN_LOG_ERROR("test-replay")
					<< "(f) FAILED: a take is a .jsonl file that is not a sidecar, and the sidecars are not takes";
				result= 1;
			}
		}

		// The recording without its sidecar file loads with no phone face
		std::error_code removeError;
		std::filesystem::remove(faceSidecarPath, removeError);
		TrackingReplay withoutFace;
		std::string loadError;
		if (!withoutFace.load(recordingPath, loadError) || !withoutFace.getFaceSamples().empty())
		{
			MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: a recording whose face sidecar is gone loads with no phone face";
			result= 1;
		}

		// A sidecar that received no sample is deleted when it stops
		{
			FaceRecorder emptyRecorder;
			if (!emptyRecorder.start(faceSidecarPath))
			{
				MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: the empty face recorder must open its file";
				result= 1;
			}
			emptyRecorder.stop();
			if (std::filesystem::exists(faceSidecarPath, removeError))
			{
				MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: a face sidecar with no sample must be deleted on stop";
				result= 1;
			}
		}

		// The phone source at a timestamp: the newest sample at or before it,
		// live inside the streaming window
		const double windowMs= 500.0;
		bool bTrackedOk= false;
		bool bUntrackedOk= false;
		bool bStaleOk= false;
		bool bBeforeOk= false;
		if (loaded.size() > 5)
		{
			const RecordedFaceSample& trackedSample= loaded[1];
			const RecordedFaceSample& untrackedSample= loaded[5];
			FaceSources::Phone phone;
			FaceSidecar::samplePhone(loaded, trackedSample.timestampMs + 3.0, windowMs, phone);
			bTrackedOk= trackedSample.bTracked && phone.present && phone.tracked &&
						phone.blendshapes == trackedSample.blendshapes;
			FaceSidecar::samplePhone(loaded, untrackedSample.timestampMs + 3.0, windowMs, phone);
			bUntrackedOk= !untrackedSample.bTracked && phone.present && !phone.tracked &&
						  phone.blendshapes == std::array<float, ARKIT_BLENDSHAPE_COUNT>{};
			FaceSidecar::samplePhone(loaded, loaded.back().timestampMs + windowMs, windowMs, phone);
			bStaleOk= !phone.present && !phone.tracked;
			FaceSidecar::samplePhone(loaded, loaded.back().timestampMs + windowMs + 100.0, windowMs, phone);
			bStaleOk&= !phone.present && !phone.tracked;
			FaceSidecar::samplePhone(loaded, loaded.front().timestampMs - 0.1, windowMs, phone);
			bBeforeOk= !phone.present && !phone.tracked;
		}
		if (!bTrackedOk || !bUntrackedOk || !bStaleOk || !bBeforeOk)
		{
			MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: samplePhone tracked=" << bTrackedOk
										   << " untracked=" << bUntrackedOk << " stale=" << bStaleOk
										   << " before=" << bBeforeOk;
			result= 1;
		}

		// A version 1 header as a recording from before either sidecar wrote
		// it: no audio block and no face block
		const nlohmann::json plainJson= nlohmann::json::parse(
			R"({"type":"header","formatVersion":1,"appConfig":{},"cameraCount":1})", nullptr, false);
		RecordingHeader loadedHeader;
		if (plainJson.is_discarded() || !TrackingRecording::headerFromJson(plainJson, loadedHeader) ||
			loadedHeader.audio.present || loadedHeader.face.present || loadedHeader.formatVersion != 1 ||
			loadedHeader.cameraCount != 1)
		{
			MIKAN_LOG_ERROR("test-replay") << "(f) FAILED: a version 1 header without either block loads with both sidecars absent";
			result= 1;
		}
	}

	MIKAN_LOG_INFO("test-replay") << "sizeof(RecordedFrame)=" << sizeof(RecordedFrame)
								  << " sizeof(RecordedTelemetry)=" << sizeof(RecordedTelemetry)
								  << " sizeof(RecordedCameraInput)=" << sizeof(RecordedCameraInput);

	// -- (g) The stage telemetry and the capture timings round-trip, and the
	// replayed per-camera results carry the recorded timings ------------
	{
		int telemetryMismatches= 0;
		int timingMismatches= 0;
		int mirrorMismatches= 0;
		for (int frameIdx= 0; frameIdx < replay.getFrameCount(); ++frameIdx)
		{
			const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIdx);
			const RecordedTelemetry& loaded= frame.recorded->telemetry;
			if (frameIdx % 3 == 1)
				telemetryMismatches+= countTelemetryMismatches(makeTelemetry(frameIdx), loaded);
			else
				telemetryMismatches+= countTelemetryMismatches(RecordedTelemetry(), loaded);

			for (const RecordedCameraInput& camera : frame.recorded->freshCameras)
			{
				const bool bExpected= frameIdx % 2 == 0;
				const TrackingFrameResult::CaptureTimings expected=
					bExpected ? makeTimings(frameIdx, camera.cameraIndex) : TrackingFrameResult::CaptureTimings();
				if (camera.bHaveCaptureTimings != bExpected || !timingsEqual(camera.captureTimings, expected))
					++timingMismatches;
				if (camera.cameraIndex >= (int)frame.perCamera.size() ||
					!timingsEqual(frame.perCamera[camera.cameraIndex].captureTimings, expected))
					++mirrorMismatches;
			}
		}

		// Keys this build does not know are ignored, inside the block and in
		// its sub-objects
		RecordedFrame source;
		source.seq= 7;
		RecordedCameraInput sourceCamera;
		sourceCamera.cameraIndex= 1;
		sourceCamera.bHaveCaptureTimings= true;
		sourceCamera.captureTimings= makeTimings(7, 1);
		source.freshCameras.push_back(sourceCamera);
		source.telemetry= makeTelemetry(7);
		nlohmann::json sourceJson= TrackingRecording::frameToJson(source);
		sourceJson["telemetry"]["futureBlock"]= {{"a", 1}, {"b", "x"}};
		sourceJson["telemetry"]["osc"]["futureRate"]= 1.5;
		sourceJson["telemetry"]["cameras"][0]["futureFlag"]= true;
		sourceJson["fresh"][0]["timings"]["futureMs"]= 2.5;
		RecordedFrame reloaded;
		const bool bLoaded= TrackingRecording::frameFromJson(nlohmann::json::parse(sourceJson.dump()), reloaded);
		const int unknownKeyMismatches= !bLoaded || reloaded.freshCameras.size() != 1
			? 1
			: countTelemetryMismatches(source.telemetry, reloaded.telemetry) +
				(reloaded.freshCameras[0].bHaveCaptureTimings &&
						 timingsEqual(reloaded.freshCameras[0].captureTimings, sourceCamera.captureTimings)
					 ? 0
					 : 1);

		MIKAN_LOG_INFO("test-replay") << "(g) telemetry mismatches=" << telemetryMismatches
									  << " timing mismatches=" << timingMismatches
									  << " mirror timing mismatches=" << mirrorMismatches
									  << " unknown-key mismatches=" << unknownKeyMismatches;
		if (telemetryMismatches != 0 || timingMismatches != 0 || mirrorMismatches != 0 || unknownKeyMismatches != 0)
		{
			MIKAN_LOG_ERROR("test-replay")
				<< "(g) FAILED: the telemetry and the capture timings must round-trip field-exact and reach the replayed per-camera results";
			result= 1;
		}
	}

	// -- (g2) A hand the direct camera moved to its side carries the flag
	// through a frame's JSON, and a hand it left alone writes no key, as a
	// recording from before the flag has none ---------------------------
	{
		RecordedFrame source;
		RecordedCameraInput sourceCamera;
		sourceCamera.cameraIndex= 0;
		sourceCamera.valid= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			sourceCamera.hands[sideIndex].tracked= true;
			sourceCamera.hands[sideIndex].side= sideIndex;
		}
		sourceCamera.hands[0].bSideCorrected= true;
		source.freshCameras.push_back(sourceCamera);
		const nlohmann::json sourceJson= TrackingRecording::frameToJson(source);
		RecordedFrame reloaded;
		const bool bFlagOk= TrackingRecording::frameFromJson(nlohmann::json::parse(sourceJson.dump()), reloaded) &&
			reloaded.freshCameras.size() == 1 && reloaded.freshCameras[0].hands[0].bSideCorrected &&
			!reloaded.freshCameras[0].hands[1].bSideCorrected &&
			sourceJson["fresh"][0]["hands"][0].contains("sideCorrected") &&
			!sourceJson["fresh"][0]["hands"][1].contains("sideCorrected");
		MIKAN_LOG_INFO("test-replay") << "(g2) corrected-side flag round trip " << (bFlagOk ? "ok" : "FAILED");
		if (!bFlagOk)
		{
			MIKAN_LOG_ERROR("test-replay")
				<< "(g2) FAILED: a moved hand must carry its flag through the frame's JSON and an unmoved one write none";
			result= 1;
		}
	}

	// -- (h) A frame without the blocks loads as absent, and a version 1
	// recording written before them still replays ----------------------
	{
		RecordedFrame plainFrame;
		RecordedCameraInput plainCamera;
		plainCamera.cameraIndex= 0;
		plainFrame.freshCameras.push_back(plainCamera);
		const nlohmann::json plainJson= TrackingRecording::frameToJson(plainFrame);
		RecordedFrame loadedFrame;
		const bool bPlainOk= !plainJson.contains("telemetry") && !plainJson["fresh"][0].contains("timings") &&
							 TrackingRecording::frameFromJson(nlohmann::json::parse(plainJson.dump()), loadedFrame) &&
							 !loadedFrame.telemetry.present && loadedFrame.freshCameras.size() == 1 &&
							 !loadedFrame.freshCameras[0].bHaveCaptureTimings;

		// A literal recording in the format as it stood before the blocks
		char checksumHex[17];
		snprintf(checksumHex, sizeof(checksumHex), "%016llx",
				 (unsigned long long)TrackingRecording::computeFusedChecksum(TrackingFrameResult()));
		const std::string literalPath=
			(std::filesystem::temp_directory_path() / "mikan-test-replay-v1.jsonl").string();
		{
			std::ofstream literalFile(literalPath, std::ios::trunc);
			literalFile << R"({"type":"header","formatVersion":1,"appConfig":)"
						<< nlohmann::json::parse(config.toJsonString()).dump() << R"(,"cameraCount":2})" << '\n';
			literalFile << R"({"type":"frame","seq":0,"nowMs":1000.0,"fused":false,"fresh":[{"cam":0,"tMs":1000.0,)"
						<< R"("valid":false,"refLenM":0.0,"frameIndex":0,"w":1280,"h":720,"captureFps":30.0,)"
						<< R"("inferenceMs":5.0,"lumaInstability":0.0,"lumaFlickerHz":0.0,)"
						<< R"("hands":[{"tracked":false},{"tracked":false}]}],)"
						<< R"("out":[{"tracked":false},{"tracked":false}],"imu":[{"has":false},{"has":false}],)"
						<< R"("checksum":")" << checksumHex << R"("})" << '\n';
			literalFile << R"({"type":"end","frameCount":1,"aborted":false,"reason":""})" << '\n';
		}
		TrackingReplay literalReplay;
		std::string literalError;
		bool bLiteralOk= literalReplay.load(literalPath, literalError);
		if (bLiteralOk)
		{
			literalReplay.runAll();
			const TrackingReplay::ReplayFrame& frame= literalReplay.getFrame(0);
			bLiteralOk= literalReplay.getFrameCount() == 1 && literalReplay.getDivergentFrames().empty() &&
						!frame.recorded->telemetry.present && frame.recorded->freshCameras.size() == 1 &&
						!frame.recorded->freshCameras[0].bHaveCaptureTimings && !frame.perCamera.empty() &&
						timingsEqual(frame.perCamera[0].captureTimings, TrackingFrameResult::CaptureTimings());
		}
		std::error_code removeError;
		std::filesystem::remove(literalPath, removeError);

		MIKAN_LOG_INFO("test-replay") << "(h) plain frame ok=" << bPlainOk << " literal version 1 recording ok=" << bLiteralOk
									  << (literalError.empty() ? "" : " error=") << literalError;
		if (!bPlainOk || !bLiteralOk)
		{
			MIKAN_LOG_ERROR("test-replay")
				<< "(h) FAILED: a frame or recording without the telemetry and timings blocks must load them as absent and replay";
			result= 1;
		}
	}

	// -- (i) The telemetry is outside the checksum: the same recording with
	// the blocks stripped replays to the same checksums ----------------
	{
		RecordedFrame withTelemetry;
		withTelemetry.checksum= 0x0123456789abcdefull;
		withTelemetry.telemetry= makeTelemetry(4);
		RecordedFrame withoutTelemetry= withTelemetry;
		withoutTelemetry.telemetry= RecordedTelemetry();
		const bool bFrameChecksumSame= TrackingRecording::frameToJson(withTelemetry)["checksum"] ==
									   TrackingRecording::frameToJson(withoutTelemetry)["checksum"];

		const std::string strippedPath=
			(std::filesystem::temp_directory_path() / "mikan-test-replay-stripped.jsonl").string();
		{
			std::ifstream inFile(recordingPath);
			std::ofstream outFile(strippedPath, std::ios::trunc);
			std::string line;
			while (std::getline(inFile, line))
			{
				nlohmann::json j= nlohmann::json::parse(line);
				if (j.value("type", "") == "frame")
				{
					j.erase("telemetry");
					for (nlohmann::json& camera : j["fresh"])
						camera.erase("timings");
					line= j.dump();
				}
				outFile << line << '\n';
			}
		}

		int telemetryFrames= 0;
		int checksumMismatches= 0;
		int strippedBlocks= 0;
		TrackingReplay strippedReplay;
		std::string strippedError;
		const bool bStrippedLoaded= strippedReplay.load(strippedPath, strippedError);
		if (bStrippedLoaded)
		{
			strippedReplay.runAll();
			if (strippedReplay.getFrameCount() != replay.getFrameCount())
				++checksumMismatches;
			for (int frameIdx= 0; frameIdx < replay.getFrameCount() && frameIdx < strippedReplay.getFrameCount();
				 ++frameIdx)
			{
				const TrackingReplay::ReplayFrame& a= replay.getFrame(frameIdx);
				const TrackingReplay::ReplayFrame& b= strippedReplay.getFrame(frameIdx);
				telemetryFrames+= a.recorded->telemetry.present ? 1 : 0;
				strippedBlocks+= b.recorded->telemetry.present ? 1 : 0;
				for (const RecordedCameraInput& camera : b.recorded->freshCameras)
					strippedBlocks+= camera.bHaveCaptureTimings ? 1 : 0;
				if (a.recorded->checksum != b.recorded->checksum || a.replayChecksum != b.replayChecksum ||
					a.bChecksumMatch != b.bChecksumMatch ||
					TrackingRecording::computeFusedChecksum(a.replayedFused) !=
						TrackingRecording::computeFusedChecksum(b.replayedFused))
					++checksumMismatches;
			}
		}
		std::error_code removeError;
		std::filesystem::remove(strippedPath, removeError);

		MIKAN_LOG_INFO("test-replay") << "(i) frame checksum same=" << bFrameChecksumSame
									  << " telemetry frames=" << telemetryFrames
									  << " stripped divergent=" << strippedReplay.getDivergentFrames().size()
									  << " checksum mismatches=" << checksumMismatches
									  << " blocks left after stripping=" << strippedBlocks;
		if (!bFrameChecksumSame || !bStrippedLoaded || telemetryFrames == 0 ||
			!strippedReplay.getDivergentFrames().empty() || !replay.getDivergentFrames().empty() ||
			checksumMismatches != 0 || strippedBlocks != 0)
		{
			MIKAN_LOG_ERROR("test-replay")
				<< "(i) FAILED: a recording with telemetry must verify exactly as the same recording without it";
			result= 1;
		}
	}

	if (result == 0)
		MIKAN_LOG_INFO("test-replay") << "All replay checks passed";

	return result;
}

MIKAN_REGISTER_TEST("--test-replay", "Record/replay determinism, checksums, what-if", eTestCategory::SelfTest, runReplayTest);
