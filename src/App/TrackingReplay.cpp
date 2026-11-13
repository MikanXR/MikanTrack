#include "TrackingReplay.h"

#include <filesystem>
#include <fstream>
#include <system_error>

#include "nlohmann/json.hpp"

#include "HandSideMatch.h"
#include "Logger.h"
#include "PathUtils.h"
#include "SpaceTransforms.h"

using json= nlohmann::json;

bool TrackingReplay::load(const std::string& path, std::string& outError)
{
	m_filePath= path;
	m_recordedFrames.clear();
	m_faceSamples.clear();
	m_frames.clear();
	m_divergentFrames.clear();
	m_bDidRun= false;
	m_bHasWhatIf= false;
	m_bHasFooter= false;
	m_footer= RecordingFooter();

	std::ifstream file(path);
	if (!file.is_open())
	{
		outError= "Cannot open " + path;
		return false;
	}

	std::string line;
	if (!std::getline(file, line))
	{
		outError= "Empty file";
		return false;
	}

	json headerJson= json::parse(line, nullptr, false);
	if (headerJson.is_discarded() || !TrackingRecording::headerFromJson(headerJson, m_header))
	{
		outError= "Bad or unsupported header (formatVersion mismatch?)";
		return false;
	}
	if (!m_recordedConfig.loadFromJsonString(m_header.appConfigJsonText))
	{
		outError= "Header config snapshot failed to parse";
		return false;
	}
	if ((int)m_recordedConfig.cameraCount() < m_header.cameraCount)
	{
		outError= "Header camera count exceeds the config snapshot's cameras";
		return false;
	}

	int64_t lineNumber= 1;
	while (std::getline(file, line))
	{
		++lineNumber;
		if (line.empty())
			continue;

		json lineJson= json::parse(line, nullptr, false);
		if (lineJson.is_discarded())
		{
			// A torn final line is expected from a crash-truncated recording
			MIKAN_LOG_WARNING("TrackingReplay")
				<< "Unparseable line " << lineNumber << " - treating as truncation";
			break;
		}

		const std::string type= lineJson.value("type", "");
		if (type == "frame")
		{
			RecordedFrame frame;
			if (TrackingRecording::frameFromJson(lineJson, frame))
				m_recordedFrames.push_back(std::move(frame));
		}
		else if (type == "end")
		{
			m_bHasFooter= TrackingRecording::footerFromJson(lineJson, m_footer);
			break;
		}
	}

	if (m_recordedFrames.empty())
	{
		outError= "Recording contains no frames";
		return false;
	}

	m_frames.resize(m_recordedFrames.size());
	for (size_t frameIndex= 0; frameIndex < m_recordedFrames.size(); ++frameIndex)
		m_frames[frameIndex].recorded= &m_recordedFrames[frameIndex];

	// The face sidecar is optional: a missing one replays with no phone face.
	// The recorder deletes a sidecar the phone never wrote to, so an absent
	// file is the normal no-phone recording.
	if (m_header.face.present && !m_header.face.fileName.empty())
	{
		const std::filesystem::path faceFilePath=
			PathUtils::utf8ToPath(path).parent_path() / PathUtils::utf8ToPath(m_header.face.fileName);
		const std::string facePath= PathUtils::pathToUtf8(faceFilePath);
		std::error_code existsError;
		std::string faceError;
		if (!std::filesystem::exists(faceFilePath, existsError))
			MIKAN_LOG_INFO("TrackingReplay") << "No face sidecar at " << facePath << " - replaying with no phone face";
		else if (!FaceSidecar::load(facePath, m_faceSamples, faceError))
			MIKAN_LOG_WARNING("TrackingReplay") << "Face sidecar not loaded: " << faceError;
	}

	return true;
}

void TrackingReplay::setCaptureDiagnostics(bool bCapture, int firstFrame, int lastFrame)
{
	m_bCaptureDiagnostics= bCapture;
	m_diagnosticsFirstFrame= firstFrame;
	m_diagnosticsLastFrame= lastFrame;
}

HandFusionConfig TrackingReplay::buildFusionConfig() const
{
	// The same mapping the vision thread applies (shared by construction)
	return makeHandFusionConfig(m_recordedConfig);
}

TrackingReplay::WhatIfParams TrackingReplay::makeDefaultWhatIfParams() const
{
	WhatIfParams params;
	params.fusionConfig= buildFusionConfig();
	return params;
}

void TrackingReplay::runAll()
{
	runPass(nullptr);
	m_bDidRun= true;
}

void TrackingReplay::runWhatIf(const WhatIfParams& params)
{
	runPass(&params);
	m_bHasWhatIf= true;
}

void TrackingReplay::clearWhatIf()
{
	for (ReplayFrame& frame : m_frames)
		frame.bHasWhatIf= false;
	m_bHasWhatIf= false;
}

void TrackingReplay::runPass(const WhatIfParams* whatIfParams)
{
	const int cameraCount= m_header.cameraCount;
	const bool bWhatIf= whatIfParams != nullptr;

	// Fresh instances per pass = the exact zero state live reset to when the
	// recording started
	m_fusion= HandFusion();
	m_fusion.configure(bWhatIf ? whatIfParams->fusionConfig : buildFusionConfig());
	m_bodyPoseSolver.reset();

	m_landmarkTo3D.clear();
	m_mirrors.clear();
	for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
	{
		const CameraProfile& profile= m_recordedConfig.camera(cameraIndex);

		std::unique_ptr<LandmarkTo3D> landmarkTo3D;
		if (profile.intrinsics.present)
		{
			landmarkTo3D= std::make_unique<LandmarkTo3D>();
			landmarkTo3D->configure(profile.intrinsics.intrinsics,
									m_recordedConfig.handScale.refLengthMeters);

			// The measured hand geometry the session ran with. Not per camera,
			// and applied the same way the vision thread applies it, or replay
			// would diverge from live for every calibrated recording.
			landmarkTo3D->clearCalibratedSkeleton();
			const bool bApplySkeleton= !bWhatIf || whatIfParams->bApplyCalibratedSkeleton;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				if (bApplySkeleton && m_recordedConfig.handSkeleton.present[sideIndex])
					landmarkTo3D->setCalibratedSkeleton(
						(eHandSide)sideIndex, m_recordedConfig.handSkeleton.skeleton[sideIndex]);
				landmarkTo3D->setDepthScale((eHandSide)sideIndex, profile.liftDepthScale[sideIndex]);
			}
		}
		m_landmarkTo3D.push_back(std::move(landmarkTo3D));

		CameraFrameResult mirror;
		mirror.cameraIndex= cameraIndex;
		m_mirrors.push_back(mirror);
	}

	if (!bWhatIf)
		m_divergentFrames.clear();

	// Extrinsics may be overridden by the what-if pass (two calibrations
	// A/B'd against the same recorded hands). Intrinsics are always the
	// recording's own: the landmarks were detected through them.
	auto effectiveExtrinsics= [&](int cameraIndex) -> const ExtrinsicsConfig& {
		if (bWhatIf && whatIfParams->bOverrideExtrinsics &&
			cameraIndex < (int)whatIfParams->extrinsicsOverride.size())
		{
			return whatIfParams->extrinsicsOverride[cameraIndex];
		}
		return m_recordedConfig.camera(cameraIndex).extrinsics;
	};

	std::vector<const CameraFrameResult*> candidates;
	for (size_t frameIndex= 0; frameIndex < m_recordedFrames.size(); ++frameIndex)
	{
		const RecordedFrame& recorded= m_recordedFrames[frameIndex];
		ReplayFrame& replayFrame= m_frames[frameIndex];

		// Per-camera stage for this iteration's fresh cameras, in recorded
		// (ascending camera index) order - the live loop's processing order
		for (const RecordedCameraInput& fresh : recorded.freshCameras)
		{
			const int cameraIndex= fresh.cameraIndex;
			if (cameraIndex < 0 || cameraIndex >= cameraCount)
				continue;
			if (bWhatIf && !whatIfParams->isCameraEnabled(cameraIndex))
				continue;

			const CameraProfile& profile= m_recordedConfig.camera(cameraIndex);
			CameraFrameResult& mirror= m_mirrors[cameraIndex];

			TrackingFrameResult result;
			result.frameIndex= fresh.frameIndex;
			result.timestampMs= fresh.timestampMs;
			result.frameWidth= fresh.frameWidth;
			result.frameHeight= fresh.frameHeight;
			result.captureFps= fresh.captureFps;
			result.inferenceMs= fresh.inferenceMs;
			result.lumaInstability= fresh.lumaInstability;
			result.lumaFlickerHz= fresh.lumaFlickerHz;
			// Display only: no replayed stage reads the capture timings
			if (fresh.bHaveCaptureTimings)
				result.captureTimings= fresh.captureTimings;

			if (fresh.valid)
			{
				for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
				{
					const RecordedHandInput& recordedHand= fresh.hands[sideIndex];
					TrackedHand& hand= result.hands[sideIndex];
					hand.tracked= recordedHand.tracked;
					if (!recordedHand.tracked)
						continue;
					hand.side= (eHandSide)recordedHand.side;
					hand.slotId= recordedHand.slotId;
					hand.presence= recordedHand.presence;
					hand.handednessScore= recordedHand.handednessScore;
					hand.rightProb= recordedHand.rightProb;
					hand.bSideCorrected= recordedHand.bSideCorrected;
					hand.imagePoints= recordedHand.imagePoints;
					hand.modelPoints= recordedHand.modelPoints;
					hand.imageQuality= recordedHand.imageQuality;
				}

				// Body-pose observation, exactly as the live stage emitted it
				// (cadence and all - the pose models are never re-run)
				if (fresh.bHaveBodyPose)
					result.body= fresh.body;

				if (bWhatIf && whatIfParams->correctHandSidesCamera == cameraIndex)
					applyHandSideCorrection(result);

				// refLengthMeters > 0 <=> live had a LandmarkTo3D for this
				// camera (an existing instance never reports 0)
				if (fresh.refLengthMeters > 0.f && m_landmarkTo3D[cameraIndex] != nullptr)
				{
					LandmarkTo3D* landmarkTo3D= m_landmarkTo3D[cameraIndex].get();
					const float refLength= bWhatIf
						? fresh.refLengthMeters * whatIfParams->refLengthScale
						: fresh.refLengthMeters;
					landmarkTo3D->setRefLengthMeters(refLength);
					landmarkTo3D->process(result);

					const ExtrinsicsConfig& extrinsics= effectiveExtrinsics(cameraIndex);
					if (extrinsics.present)
						applyWorldTransform(result, extrinsics.markerFromCamera);
				}
			}

			// Mirror update, matching the live lastResult assignments
			const ExtrinsicsConfig& extrinsics= effectiveExtrinsics(cameraIndex);
			mirror.valid= fresh.valid;
			mirror.timestampMs= fresh.timestampMs;
			mirror.hasExtrinsics= extrinsics.present;
			mirror.markerFromCamera= extrinsics.markerFromCamera;
			mirror.hasIntrinsics= profile.intrinsics.present;
			if (profile.intrinsics.present)
			{
				const MikanMatrix3d& cameraMatrix= profile.intrinsics.intrinsics.undistorted_camera_matrix;
				mirror.fx= (float)cameraMatrix.x0;
				mirror.fy= (float)cameraMatrix.y1;
				mirror.cx= (float)cameraMatrix.z0;
				mirror.cy= (float)cameraMatrix.z1;
			}
			mirror.result= result;
		}

		// Fusion stage over ALL cameras' mirrors (fresh or stale), exactly
		// like the live candidate vector. A camera mask narrows the vector
		// and re-decides the no-world passthrough the way the live loop
		// would have with only those cameras (any valid mirror with
		// extrinsics fuses); without a mask the recorded decision stands.
		candidates.clear();
		const CameraFrameResult* passthrough= nullptr;
		bool bFuse= recorded.bFused;
		if (bWhatIf && !whatIfParams->cameraEnabled.empty())
			bFuse= false;
		for (const CameraFrameResult& mirror : m_mirrors)
		{
			if (bWhatIf && !whatIfParams->isCameraEnabled(mirror.cameraIndex))
				continue;
			candidates.push_back(&mirror);
			// The first camera, or the direct camera of a direct-mode recording
			if (passthrough == nullptr ||
				(m_recordedConfig.direct.enabled && mirror.cameraIndex == m_recordedConfig.direct.cameraIndex))
				passthrough= &mirror;
			if (bWhatIf && !whatIfParams->cameraEnabled.empty() && recorded.bFused &&
				mirror.valid && mirror.hasExtrinsics)
				bFuse= true;
		}

		TrackingFrameResult fused;
		if (bFuse)
		{
			m_fusion.fuse(candidates, recorded.nowTimestampMs, fused);
		}
		else
		{
			// No-world passthrough: the passthrough camera's result (camera 0,
			// or the direct camera), reconstructed from the REPLAYED mirror so
			// the checksum still verifies this frame
			fused= passthrough == nullptr ? TrackingFrameResult() : passthrough->result;
		}

		if (!bWhatIf)
		{
			// Checksum compares PRE-IMU, PRE-body-solver, matching the live tap
			replayFrame.replayChecksum= TrackingRecording::computeFusedChecksum(fused);
			replayFrame.bChecksumMatch= replayFrame.replayChecksum == recorded.checksum;
			if (!replayFrame.bChecksumMatch)
				m_divergentFrames.push_back((int)frameIndex);

			if (m_bCaptureDiagnostics && recorded.bFused &&
				(int)frameIndex >= m_diagnosticsFirstFrame &&
				(m_diagnosticsLastFrame < 0 || (int)frameIndex <= m_diagnosticsLastFrame))
			{
				replayFrame.diagnostics= m_fusion.getLastDiagnostics();
				replayFrame.bHasDiagnostics= true;
			}
			else
			{
				replayFrame.bHasDiagnostics= false;
			}
		}

		// Post-fusion output stages in live order: overlay the recorded IMU
		// forearm output (the EKF is not re-run), then run the body-pose
		// solver, which fills only sides the IMU didn't claim. Re-running the
		// solver (rather than replaying its output) is what makes solver
		// changes A/B-able against old recordings.
		const bool bApplyImu= !bWhatIf || whatIfParams->bApplyRecordedImu;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			HandPose& pose= fused.poses[sideIndex];
			const RecordedImuOutput& imu= recorded.imu[sideIndex];
			pose.hasForearmPose= bApplyImu && imu.hasForearmPose;
			if (pose.hasForearmPose)
			{
				pose.forearmOrientationWorld= imu.forearmOrientationWorld;
				pose.forearmConfidence= imu.forearmConfidence;
			}
		}
		if (bFuse)
			m_bodyPoseSolver.solve(candidates, makeBodyDimensions(m_recordedConfig), fused);

		if (bWhatIf)
		{
			replayFrame.whatIfFused= fused;
			replayFrame.bHasWhatIf= true;
			replayFrame.whatIfPerCamera.resize(cameraCount);
			for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
				replayFrame.whatIfPerCamera[cameraIndex]= m_mirrors[cameraIndex].result;

			if (m_bCaptureDiagnostics && recorded.bFused &&
				(int)frameIndex >= m_diagnosticsFirstFrame &&
				(m_diagnosticsLastFrame < 0 || (int)frameIndex <= m_diagnosticsLastFrame))
			{
				replayFrame.whatIfDiagnostics= m_fusion.getLastDiagnostics();
				replayFrame.bHasWhatIfDiagnostics= true;
			}
			else
			{
				replayFrame.bHasWhatIfDiagnostics= false;
			}
			continue;
		}

		replayFrame.replayedFused= fused;

		// Per-camera view for the scene panel (stale cameras show their last
		// fresh result, like the live preview panes)
		replayFrame.perCamera.resize(cameraCount);
		replayFrame.perCameraValid.resize(cameraCount);
		for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
		{
			replayFrame.perCamera[cameraIndex]= m_mirrors[cameraIndex].result;
			replayFrame.perCameraValid[cameraIndex]= m_mirrors[cameraIndex].valid;
		}
	}
}
