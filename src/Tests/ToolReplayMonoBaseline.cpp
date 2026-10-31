#include "TestCommon.h"

// Mono baseline: replays recordings twice - faithfully, and as a what-if pass
// with one camera alone and no IMU overlay - and measures the single-camera
// output against the multi-camera output of the same frames: palm position
// (split along the camera's view ray and lateral to it), palm orientation,
// finger angles, elbow, shoulder and head. The truth is the recorded pass's
// stereo-triangulated output, so frames where that pass itself fell back to
// one camera stay out of the hand statistics.
//
//   --replay-mono-baseline <recording.jsonl>... [cameraIndex]
//
// cameraIndex defaults to the recording's body-pose camera (the front camera
// of the tri-camera setup). The gates at the end are the acceptance bar of
// the single-camera plan's first stage, pooled over every recording and both
// sides; the exit code is 1 when any gate fails.
//
// Coverage reads optimistic by one mechanism: the masked camera's recorded
// landmarks include hands it found through cross-camera search hints, which
// a camera running alone would not have had.

namespace
{
constexpr float kPalmP50GateCm= 3.f;
constexpr float kPalmP90GateCm= 8.f;
constexpr float kElbowFlipGatePct= 2.f;
constexpr float kFingerP50GateDeg= 10.f;
// An elbow this far from the truth elbow sits on the other root of the bone
// circle (the two candidates measured about 400 mm apart), not on a noisy
// version of the same one
constexpr float kRootFlipMeters= 0.15f;

struct Stats
{
	std::vector<float> values;
	int worstFrame= -1;
	float worstValue= -1.f;

	void add(float value, int frameIndex= -1)
	{
		values.push_back(value);
		if (value > worstValue)
		{
			worstValue= value;
			worstFrame= frameIndex;
		}
	}
	void append(const Stats& other)
	{
		values.insert(values.end(), other.values.begin(), other.values.end());
		if (other.worstValue > worstValue)
		{
			worstValue= other.worstValue;
			worstFrame= other.worstFrame;
		}
	}
	int count() const { return (int)values.size(); }
	float percentile(float fraction) const
	{
		if (values.empty())
			return 0.f;
		std::vector<float> sorted= values;
		std::sort(sorted.begin(), sorted.end());
		const size_t index= std::min(sorted.size() - 1, (size_t)((float)sorted.size() * fraction));
		return sorted[index];
	}
	float p50() const { return percentile(0.5f); }
	float p90() const { return percentile(0.9f); }
};

struct SideStats
{
	// Records where the truth hand is stereo-triangulated, and how many of
	// those the mono pass tracked at all
	int truthStereoRecords= 0;
	int monoTrackedOfTruth= 0;
	// Mono tracked a hand the truth did not track at all
	int monoOnlyRecords= 0;

	Stats palmErrCm;
	Stats palmAlongSignedCm; // + = mono palm farther from the camera than truth
	Stats palmLateralCm;
	Stats palmRotDeg;
	Stats fingerDeg; // pooled over the 20 DoF

	// Elbow, sampled once per body model frame
	int elbowModelFrames= 0;
	int elbowFlips= 0;
	Stats elbowErrCm;
	Stats forearmDeg;

	Stats shoulderErrCm;

	// Mono estimator attribution over the whole pass
	int estimatorReseeds= 0;
	int estimatorHolds= 0;

	void append(const SideStats& other)
	{
		truthStereoRecords+= other.truthStereoRecords;
		monoTrackedOfTruth+= other.monoTrackedOfTruth;
		monoOnlyRecords+= other.monoOnlyRecords;
		palmErrCm.append(other.palmErrCm);
		palmAlongSignedCm.append(other.palmAlongSignedCm);
		palmLateralCm.append(other.palmLateralCm);
		palmRotDeg.append(other.palmRotDeg);
		fingerDeg.append(other.fingerDeg);
		elbowModelFrames+= other.elbowModelFrames;
		elbowFlips+= other.elbowFlips;
		elbowErrCm.append(other.elbowErrCm);
		forearmDeg.append(other.forearmDeg);
		shoulderErrCm.append(other.shoulderErrCm);
		estimatorReseeds+= other.estimatorReseeds;
		estimatorHolds+= other.estimatorHolds;
	}
};

struct HeadStats
{
	Stats headErrCm;
	Stats headRotDeg;

	void append(const HeadStats& other)
	{
		headErrCm.append(other.headErrCm);
		headRotDeg.append(other.headRotDeg);
	}
};

float angleBetweenDeg(const glm::quat& a, const glm::quat& b)
{
	glm::quat dq= glm::inverse(a) * b;
	if (dq.w < 0.f)
		dq= -dq;
	return glm::degrees(2.f * acosf(std::min(dq.w, 1.f)));
}

float angleBetweenDeg(const glm::vec3& a, const glm::vec3& b)
{
	const float lengths= glm::length(a) * glm::length(b);
	if (lengths < 1e-9f)
		return 0.f;
	return glm::degrees(acosf(std::max(-1.f, std::min(1.f, glm::dot(a, b) / lengths))));
}

float angleDof(const FingerAngles& angles, int dof)
{
	switch (dof)
	{
	case 0: return angles.lateral;
	case 1: return angles.proximal;
	case 2: return angles.intermediate;
	default: return angles.distal;
	}
}

// The chosen camera's body observation this frame, when it carries a model
// result the previous frame did not
bool isNewBodyModelFrame(const RecordedFrame& recorded, int cameraIndex, int64_t& lastModelFrameIndex)
{
	for (const RecordedCameraInput& fresh : recorded.freshCameras)
	{
		if (fresh.cameraIndex != cameraIndex || !fresh.valid || !fresh.bHaveBodyPose ||
			!fresh.body.valid)
			continue;
		if (fresh.body.modelFrameIndex == lastModelFrameIndex)
			return false;
		lastModelFrameIndex= fresh.body.modelFrameIndex;
		return true;
	}
	return false;
}

void accumulateFrame(const TrackingFrameResult& truth, const TrackingFrameResult& mono,
					 const RecordedFrame& recorded, int frameIndex, int cameraIndex,
					 const glm::vec3& cameraPositionWorld, float forearmLengthMeters,
					 int64_t& lastModelFrameIndex, std::array<SideStats, 2>& sides, HeadStats& head)
{
	const bool bNewModelFrame= isNewBodyModelFrame(recorded, cameraIndex, lastModelFrameIndex);

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		SideStats& side= sides[sideIndex];
		const HandPose& truthPose= truth.poses[sideIndex];
		const HandPose& monoPose= mono.poses[sideIndex];
		const bool bTruthTracked= truthPose.tracked && truthPose.hasWorldPose;
		const bool bTruthStereo= bTruthTracked && truthPose.stereoTriangulated;
		const bool bMonoTracked= monoPose.tracked && monoPose.hasWorldPose;

		if (bMonoTracked && !bTruthTracked)
			++side.monoOnlyRecords;
		if (!bTruthStereo)
			continue;
		++side.truthStereoRecords;
		if (!bMonoTracked)
			continue;
		++side.monoTrackedOfTruth;

		const glm::vec3 error= monoPose.palmPositionWorld - truthPose.palmPositionWorld;
		const glm::vec3 ray= glm::normalize(truthPose.palmPositionWorld - cameraPositionWorld);
		const float along= glm::dot(error, ray);
		side.palmErrCm.add(glm::length(error) * 100.f, frameIndex);
		side.palmAlongSignedCm.add(along * 100.f, frameIndex);
		side.palmLateralCm.add(glm::length(error - along * ray) * 100.f, frameIndex);
		side.palmRotDeg.add(angleBetweenDeg(truthPose.palmOrientationWorld, monoPose.palmOrientationWorld),
							frameIndex);

		// Both passes subtract the same rest offset, so the streamed angles
		// compare directly
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
			for (int dof= 0; dof < 4; ++dof)
				side.fingerDeg.add(glm::degrees(fabsf(angleDof(monoPose.fingers[finger], dof) -
													 angleDof(truthPose.fingers[finger], dof))),
								   frameIndex);

		if (bNewModelFrame && truthPose.hasForearmPose && monoPose.hasForearmPose)
		{
			++side.elbowModelFrames;
			const glm::vec3 truthElbow= truthPose.getElbowPositionWorld(forearmLengthMeters);
			const glm::vec3 monoElbow= monoPose.getElbowPositionWorld(forearmLengthMeters);
			const float elbowErrM= glm::length(monoElbow - truthElbow);
			side.elbowErrCm.add(elbowErrM * 100.f, frameIndex);
			if (elbowErrM > kRootFlipMeters)
				++side.elbowFlips;
			const glm::vec3 truthForearm= truthPose.forearmOrientationWorld * glm::vec3(1.f, 0.f, 0.f);
			const glm::vec3 monoForearm= monoPose.forearmOrientationWorld * glm::vec3(1.f, 0.f, 0.f);
			side.forearmDeg.add(angleBetweenDeg(truthForearm, monoForearm), frameIndex);
		}

		if (bNewModelFrame && truthPose.hasShoulder && monoPose.hasShoulder)
		{
			side.shoulderErrCm.add(
				glm::length(monoPose.shoulderPositionWorld - truthPose.shoulderPositionWorld) * 100.f,
				frameIndex);
		}
	}

	if (bNewModelFrame && truth.head.valid && mono.head.valid)
	{
		head.headErrCm.add(glm::length(mono.head.positionWorld - truth.head.positionWorld) * 100.f,
						   frameIndex);
		head.headRotDeg.add(angleBetweenDeg(truth.head.orientationWorld, mono.head.orientationWorld),
							frameIndex);
	}
}

void logSide(const char* tag, int sideIndex, const SideStats& side)
{
	const char* sideName= sideIndex == 0 ? "L" : "R";
	const float coveragePct= side.truthStereoRecords > 0
		? 100.f * (float)side.monoTrackedOfTruth / (float)side.truthStereoRecords
		: 0.f;
	MIKAN_LOG_INFO(tag)
		<< "  " << sideName << ": truth stereo records " << side.truthStereoRecords
		<< ", mono tracked " << coveragePct << "% of them (" << side.monoTrackedOfTruth
		<< "), mono-only records " << side.monoOnlyRecords
		<< ", estimator reseeds " << side.estimatorReseeds << ", bad-fit holds " << side.estimatorHolds;
	MIKAN_LOG_INFO(tag)
		<< "    palm err cm p50/p90 " << side.palmErrCm.p50() << "/" << side.palmErrCm.p90()
		<< " (worst " << side.palmErrCm.worstValue << " at frame " << side.palmErrCm.worstFrame << ")"
		<< ", along-ray signed p50 " << side.palmAlongSignedCm.p50()
		<< ", lateral p50/p90 " << side.palmLateralCm.p50() << "/" << side.palmLateralCm.p90()
		<< ", rot deg p50/p90 " << side.palmRotDeg.p50() << "/" << side.palmRotDeg.p90();
	MIKAN_LOG_INFO(tag)
		<< "    finger deg p50/p90 " << side.fingerDeg.p50() << "/" << side.fingerDeg.p90()
		<< " (" << side.fingerDeg.count() << " samples)";
	const float flipPct= side.elbowModelFrames > 0
		? 100.f * (float)side.elbowFlips / (float)side.elbowModelFrames
		: 0.f;
	MIKAN_LOG_INFO(tag)
		<< "    elbow model frames " << side.elbowModelFrames << ", root flips " << side.elbowFlips
		<< " (" << flipPct << "%), err cm p50/p90 " << side.elbowErrCm.p50() << "/"
		<< side.elbowErrCm.p90() << " (worst at frame " << side.elbowErrCm.worstFrame << ")"
		<< ", forearm deg p50/p90 " << side.forearmDeg.p50() << "/" << side.forearmDeg.p90()
		<< ", shoulder err cm p50/p90 " << side.shoulderErrCm.p50() << "/" << side.shoulderErrCm.p90()
		<< " (" << side.shoulderErrCm.count() << " samples)";
}

void logHead(const char* tag, const HeadStats& head)
{
	MIKAN_LOG_INFO(tag)
		<< "  head err cm p50/p90 " << head.headErrCm.p50() << "/" << head.headErrCm.p90()
		<< ", rot deg p50/p90 " << head.headRotDeg.p50() << "/" << head.headRotDeg.p90()
		<< " (" << head.headErrCm.count() << " samples)";
}

int defaultCameraIndex(const AppConfig& config)
{
	for (int cameraIndex= 0; cameraIndex < (int)config.cameraCount(); ++cameraIndex)
	{
		if (config.camera(cameraIndex).bodyPose.enabled)
			return cameraIndex;
	}
	return -1;
}
} // namespace

static int runReplayMonoBaselineTool(const TestArgs& args)
{
	const char* tag= "replay-mono-baseline";

	std::vector<std::string> recordings;
	int requestedCamera= -1;
	for (const std::string& arg : args)
	{
		if (arg.size() > 6 && arg.substr(arg.size() - 6) == ".jsonl")
			recordings.push_back(arg);
		else
			requestedCamera= atoi(arg.c_str());
	}
	if (recordings.empty())
	{
		MIKAN_LOG_ERROR(tag) << "Usage: --replay-mono-baseline <recording.jsonl>... [cameraIndex]";
		return 1;
	}

	int result= 0;
	std::array<SideStats, 2> pooled{};
	HeadStats pooledHead;
	for (const std::string& path : recordings)
	{
		TrackingReplay replay;
		std::string error;
		if (!replay.load(path, error))
		{
			MIKAN_LOG_ERROR(tag) << path << ": load failed: " << error;
			result= 1;
			continue;
		}
		const AppConfig& config= replay.getRecordedConfig();

		const int cameraIndex= requestedCamera >= 0 ? requestedCamera : defaultCameraIndex(config);
		if (cameraIndex < 0 || cameraIndex >= (int)config.cameraCount())
		{
			MIKAN_LOG_ERROR(tag) << path << ": no body-pose camera to default to; pass a cameraIndex";
			result= 1;
			continue;
		}
		const CameraProfile& profile= config.camera(cameraIndex);
		if (!profile.intrinsics.present || !profile.extrinsics.present)
		{
			MIKAN_LOG_ERROR(tag) << path << ": camera " << cameraIndex
								 << " has no intrinsics or extrinsics, nothing to lift with";
			result= 1;
			continue;
		}
		const glm::dmat4 markerFromCamera= profile.extrinsics.markerFromCamera;
		const glm::vec3 cameraPositionWorld= glm::vec3(markerFromCamera * glm::dvec4(0.0, 0.0, 0.0, 1.0));
		const float forearmLengthMeters= makeBodyDimensions(config).forearmLengthMeters;

		replay.setCaptureDiagnostics(true);
		replay.runAll();

		TrackingReplay::WhatIfParams params= replay.makeDefaultWhatIfParams();
		params.cameraEnabled.assign(config.cameraCount(), false);
		params.cameraEnabled[cameraIndex]= true;
		params.bApplyRecordedImu= false;
		replay.runWhatIf(params);

		std::array<SideStats, 2> sides{};
		HeadStats head;
		int64_t lastModelFrameIndex= -1;
		for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
		{
			const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIndex);
			if (frame.recorded == nullptr || !frame.recorded->bFused || !frame.bHasWhatIf)
				continue;
			accumulateFrame(frame.replayedFused, frame.whatIfFused, *frame.recorded, frameIndex,
							cameraIndex, cameraPositionWorld, forearmLengthMeters, lastModelFrameIndex,
							sides, head);
			if (frame.bHasWhatIfDiagnostics)
			{
				for (const FusionDiagnostics::Cluster& cluster : frame.whatIfDiagnostics.clusters)
				{
					if (cluster.assignedSide < 0 || cluster.assignedSide > 1)
						continue;
					if (cluster.estimatorReseeded)
						++sides[cluster.assignedSide].estimatorReseeds;
					if (cluster.estimatorHeldBadFit)
						++sides[cluster.assignedSide].estimatorHolds;
				}
			}
		}

		MIKAN_LOG_INFO(tag)
			<< std::filesystem::path(path).filename().string() << " (" << replay.getFrameCount()
			<< " frames, " << config.cameraCount() << " cameras, mono camera " << cameraIndex
			<< ", divergent faithful frames " << replay.getDivergentFrames().size() << "):";
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			logSide(tag, sideIndex, sides[sideIndex]);
			pooled[sideIndex].append(sides[sideIndex]);
		}
		logHead(tag, head);
		pooledHead.append(head);
	}

	// Gates over everything, both sides pooled
	SideStats all;
	all.append(pooled[0]);
	all.append(pooled[1]);
	MIKAN_LOG_INFO(tag) << "Pooled over " << recordings.size() << " recording(s):";
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		logSide(tag, sideIndex, pooled[sideIndex]);
	logHead(tag, pooledHead);

	const float flipPct= all.elbowModelFrames > 0
		? 100.f * (float)all.elbowFlips / (float)all.elbowModelFrames
		: 0.f;
	struct Gate
	{
		const char* name;
		float value;
		float limit;
		int samples;
	};
	const Gate gates[]= {
		{"palm err cm p50", all.palmErrCm.p50(), kPalmP50GateCm, all.palmErrCm.count()},
		{"palm err cm p90", all.palmErrCm.p90(), kPalmP90GateCm, all.palmErrCm.count()},
		{"elbow root flips %", flipPct, kElbowFlipGatePct, all.elbowModelFrames},
		{"finger deg p50", all.fingerDeg.p50(), kFingerP50GateDeg, all.fingerDeg.count()},
	};
	for (const Gate& gate : gates)
	{
		const bool bPass= gate.samples > 0 && gate.value < gate.limit;
		if (!bPass)
			result= 1;
		MIKAN_LOG_INFO(tag)
			<< "  gate " << gate.name << ": " << gate.value << " (limit " << gate.limit << ", "
			<< gate.samples << " samples) " << (bPass ? "PASS" : "FAIL");
	}
	MIKAN_LOG_INFO(tag) << (result == 0 ? "All gates passed" : "Gate(s) failed");

	return result;
}

MIKAN_REGISTER_TEST("--replay-mono-baseline",
					"Single-camera output of a recording measured against its multi-camera output",
					eTestCategory::Tool, runReplayMonoBaselineTool);
