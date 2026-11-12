#include "TestCommon.h"

#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "PathUtils.h"
#include "RetargetGolden.h"
#include "SyntheticAvatar.h"

// The retarget golden comparison over a recording: the live retarget path
// beside its frozen copy on every replayed frame, bit for bit. A direct-mode
// recording drives the direct retarget from its camera's replayed result. Any
// other recording drives the retarget stage (the gate, hold, freeze and
// position retarget the frozen stream ran itself) from the replayed fused
// output, bit for bit where every frame is sent and within the decimated
// tolerance on the sent frames alone. Both run on the shared synthetic
// rig with default rig settings, so the comparison pins no avatar file and no
// sidecar: it asks whether the two code paths agree, not what the avatar did.
// The one difference by design is the step clock: from a take's first small
// backward step in the frame clock on, a frame that differs is counted and
// reported apart, with the largest difference, rather than failing.

namespace
{
using namespace RetargetGolden;

const char* kTag= "retarget-golden";

// The backward steps in a take's frame clock. The live retarget steps on the
// source's step clock, which holds across a small backward step where the
// frozen path read a regression, so from the first small step on the two
// paths may differ by design. A larger step is a discontinuity to both.
struct BackwardSteps
{
	int steps= 0;
	int small= 0;
	double largestMs= 0.0;
	bool bAfterSmall= false;
	bool bHasLast= false;
	double lastMs= 0.0;

	void step(double timestampMs)
	{
		if (bHasLast && timestampMs < lastMs)
		{
			steps++;
			largestMs= std::max(largestMs, lastMs - timestampMs);
			if (lastMs - timestampMs <= SourcePose::Clock::kMaxBackwardStepMs)
			{
				small++;
				bAfterSmall= true;
			}
		}
		bHasLast= true;
		lastMs= timestampMs;
	}
};

// The frames that differ from the frozen path by design, counted apart: which
// is first, how many, the largest difference in a pose or a stream float,
// and how many differ in their structure (the bone set, the messages)
struct ExpectedDifference
{
	int frames= 0;
	int structural= 0;
	std::string first;
	float largest= 0.f;

	void note(int frameIndex, bool bStructureSame, float difference)
	{
		if (frames == 0)
			first= "frame " + std::to_string(frameIndex);
		frames++;
		structural+= bStructureSame ? 0 : 1;
		largest= std::max(largest, difference);
	}
};

void report(const Tally& tally, const char* what, const char* agreement= "identical",
			const ExpectedDifference* expected= nullptr)
{
	if (tally.mismatches != 0)
	{
		MIKAN_LOG_ERROR(kTag) << "FAIL " << what << ": " << tally.mismatches << " of " << tally.frames
							  << " frames differ, first at " << tally.firstMismatch;
	}
	else if (expected == nullptr || expected->frames == 0)
	{
		MIKAN_LOG_INFO(kTag) << "PASS " << what << ": " << tally.frames << " frames " << agreement;
	}
	else
	{
		MIKAN_LOG_INFO(kTag) << "PASS " << what << ": " << tally.frames - expected->frames << " of " << tally.frames
							 << " frames " << agreement << ", " << expected->frames
							 << " differ by design after a small backward step (first at " << expected->first
							 << ", largest difference " << expected->largest << ", " << expected->structural
							 << " in structure)";
	}
}

// How far a stream pair's live side is from its frozen side on the frame it
// just encoded, with no tolerance. False when they differ in structure.
bool measureStream(StreamPair& pair, float& outDifference)
{
	const float unbounded= std::numeric_limits<float>::infinity();
	std::string why;
	float poseDifference= 0.f;
	bool bStructureSame= true;
	if (pair.frozenResolve.config.avatarSkeleton != nullptr)
		bStructureSame= samePoseWithin(pair.live.result.pose, pair.frozenResolve.pose, unbounded, why, poseDifference);
	maskStreamClock(pair.livePackets);
	maskStreamClock(pair.frozenPackets);
	float streamDifference= 0.f;
	bStructureSame&= samePacketsWithin(pair.livePackets, pair.frozenPackets, unbounded, why, streamDifference);
	outDifference= std::max(poseDifference, streamDifference);
	return bStructureSame;
}

StreamConfig makeStreamConfig(const AppConfig& config, const AvatarRigSettings& rig,
								   const std::shared_ptr<const AvatarSkeleton>& skeleton)
{
	const BodyDimensions bodyDimensions= makeBodyDimensions(config);
	// The frozen streamer took one hold and one freeze rule for both sides:
	// the left arm's, as the position pipeline performs it from the chains.
	// It knew no rest pose, so an ease to rest takes the stream's freeze.
	const RetargetChains chains= resolveRetargetChains(config, rig);
	const ChainLossPolicy armLoss=
		makeChainLossPolicy(chains, SourcePose::ePipeline::Position, config.osc.minConfidence, false).arms[0];
	StreamConfig oscConfig;
	oscConfig.maxRateHz= 0.f;
	oscConfig.minConfidence= config.osc.minConfidence;
	oscConfig.holdOnDropoutMs= armLoss.hold == eLossHold::Timed ? armLoss.holdMs : 0.f;
	oscConfig.forearmLengthMeters= bodyDimensions.forearmLengthMeters;
	oscConfig.upperArmLengthMeters= bodyDimensions.upperArmLengthMeters;
	oscConfig.shoulderWidthMeters= bodyDimensions.shoulderWidthMeters;
	oscConfig.vmcHeadOffsetMeters= config.osc.vmcHeadOffsetMeters;
	oscConfig.vmcFreezeOnLoss= armLoss.after == eLossAfter::Freeze;
	oscConfig.bodyDimensions= bodyDimensions;
	oscConfig.avatarRetarget= makeAvatarRetargetConfig(config, rig);
	// The frozen retarget took a threshold where the chains hold an elbow
	// weight: the threshold its default function was made with. A weight of
	// another form has no frozen counterpart, so the sidecar's threshold
	// stands in on both sides and the project's weight is not compared.
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		float threshold= 0.f;
		if (findElbowHintConfidence(chains.arms[sideIndex].elbowWeight, sideIndex, threshold))
		{
			oscConfig.elbowHintConfidence[sideIndex]= threshold;
		}
		else
		{
			oscConfig.elbowHintConfidence[sideIndex]= rig.sides[sideIndex].elbowHintConfidence;
			MIKAN_LOG_WARNING(kTag) << (sideIndex == 0 ? "leftArm" : "rightArm")
									<< ": the elbow weight is not the default function of a threshold, so the "
									   "comparison runs the sidecar's threshold "
									<< rig.sides[sideIndex].elbowHintConfidence << " on both sides";
		}
	}
	oscConfig.avatarSkeleton= skeleton;
	return oscConfig;
}

int runDirect(const TrackingReplay& replay, const AppConfig& config, const AvatarRigSettings& rig,
			  const std::shared_ptr<const AvatarSkeleton>& skeleton)
{
	const int cameraIndex= config.direct.cameraIndex;
	if (cameraIndex < 0 || cameraIndex >= (int)config.cameraCount() ||
		!config.camera(cameraIndex).intrinsics.present)
	{
		MIKAN_LOG_ERROR(kTag) << "the direct camera " << cameraIndex << " has no intrinsics";
		return 1;
	}
	const MikanMatrix3d& cameraMatrix= config.camera(cameraIndex).intrinsics.intrinsics.undistorted_camera_matrix;
	const DirectRetargetConfig directConfig= makeDirectRetargetConfig(config, rig);
	const TrackingFrameResult::HeadPose noPhoneHead;

	DirectPair direct;
	StreamPair stream;
	stream.setConfig(makeStreamConfig(config, rig, skeleton));
	Tally tally;
	std::string why;
	int placedFrames= 0;
	BackwardSteps backward;
	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIndex);
		if (cameraIndex >= (int)frame.perCamera.size() || !frame.perCameraValid[cameraIndex])
			continue;
		const TrackingFrameResult& cameraFrame= frame.perCamera[cameraIndex];
		backward.step(cameraFrame.timestampMs);
		DirectCameraModel cameraModel;
		cameraModel.fx= (float)cameraMatrix.x0;
		cameraModel.fy= (float)cameraMatrix.y1;
		cameraModel.cx= (float)cameraMatrix.z0;
		cameraModel.cy= (float)cameraMatrix.z1;
		cameraModel.width= cameraFrame.frameWidth;
		cameraModel.height= cameraFrame.frameHeight;

		tally.frames++;
		const bool bSame= direct.solve(cameraFrame, cameraModel, noPhoneHead, cameraFrame.timestampMs, *skeleton,
									   directConfig, why);
		tally.note(bSame, frameIndex, "direct", why);
		tally.note(stream.encodeDirect(cameraFrame, direct.livePose, direct.frozenPose, why), frameIndex, "stream",
				   why);
		const Frozen::DirectRetarget::SolveState& state= direct.frozen.getSolveState();
		placedFrames+= (state.arms[0].chain.valid || state.arms[1].chain.valid) ? 1 : 0;
	}
	MIKAN_LOG_INFO(kTag) << "direct camera " << cameraIndex << ": an arm placed on " << placedFrames << " of "
						 << tally.frames << " frames";
	// The direct solver restarts on any backward step, so none of them is a
	// difference by design here
	MIKAN_LOG_INFO(kTag) << backward.steps << " backward steps in the frame clock, the largest " << backward.largestMs
						 << " ms";
	report(tally, "direct retarget and stream");
	return tally.mismatches == 0 && tally.frames > 0 ? 0 : 1;
}

int runPosition(const TrackingReplay& replay, const AppConfig& config, const AvatarRigSettings& rig,
				const std::shared_ptr<const AvatarSkeleton>& skeleton)
{
	const StreamConfig streamConfig= makeStreamConfig(config, rig, skeleton);

	// Every frame encoded, the avatar path and the measured-length path, and
	// the retarget alone on the unresolved frame for the root
	StreamPair avatarStream;
	avatarStream.setConfig(streamConfig);
	StreamPair lengthStream;
	{
		StreamConfig lengthConfig= streamConfig;
		lengthConfig.avatarSkeleton= nullptr;
		lengthStream.setConfig(lengthConfig);
	}
	// Only the frames the recorded send rate would have passed. The live
	// retarget stage runs on every frame and the frozen streamer resolved only
	// the sent ones, so this stream differs by design and is compared within
	// the decimated tolerance. A sent frame with an arm held or frozen is
	// compared for its structure alone: its hold carries the last measured
	// frame, which on the live side may be one the frozen streamer never
	// resolved, and a recorded hand can change a long way between two frames.
	StreamPair gatedStream;
	gatedStream.setConfig(streamConfig);
	gatedStream.tolerance= kDecimatedTolerance;
	gatedStream.bCarryOverStructureOnly= true;
	RateGate gate;
	const float maxRateHz= (float)config.osc.maxRateHz;
	PositionPair position;

	Tally avatarTally;
	Tally lengthTally;
	Tally gatedTally;
	Tally retargetTally;
	std::string why;
	int trackedFrames= 0;
	BackwardSteps backward;
	ExpectedDifference avatarExpected;
	ExpectedDifference lengthExpected;
	ExpectedDifference gatedExpected;
	ExpectedDifference retargetExpected;
	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const TrackingFrameResult& frame= replay.getFrame(frameIndex).replayedFused;
		trackedFrames+= (frame.poses[0].tracked || frame.poses[1].tracked) ? 1 : 0;
		backward.step(frame.timestampMs);

		// A frame that differs after a small backward step is counted as a
		// difference by design; one before it is a failure
		auto noteStream= [&](Tally& tally, ExpectedDifference& expected, StreamPair& pair, bool bSame) {
			if (bSame || !backward.bAfterSmall)
			{
				tally.note(bSame, frameIndex, "stream", why);
				return;
			}
			float difference= 0.f;
			const bool bStructureSame= measureStream(pair, difference);
			expected.note(frameIndex, bStructureSame, difference);
		};

		avatarTally.frames++;
		avatarStream.retarget(frame);
		noteStream(avatarTally, avatarExpected, avatarStream, avatarStream.encode(frame, why));
		lengthTally.frames++;
		lengthStream.retarget(frame);
		noteStream(lengthTally, lengthExpected, lengthStream, lengthStream.encode(frame, why));
		retargetTally.frames++;
		const bool bSame= position.solve(frame, streamConfig.bodyDimensions, *skeleton, streamConfig.avatarRetarget,
										 streamConfig.elbowHintConfidence, why);
		if (bSame || !backward.bAfterSmall)
		{
			retargetTally.note(bSame, frameIndex, "retarget", why);
		}
		else
		{
			float difference= 0.f;
			const bool bStructureSame= samePoseWithin(position.livePose, position.frozenPose,
													  std::numeric_limits<float>::infinity(), why, difference);
			retargetExpected.note(frameIndex, bStructureSame, difference);
		}
		gatedStream.retarget(frame);
		if (gate.pass(frame.timestampMs, maxRateHz))
		{
			gatedTally.frames++;
			noteStream(gatedTally, gatedExpected, gatedStream, gatedStream.encode(frame, why));
		}
	}
	MIKAN_LOG_INFO(kTag) << "a hand tracked on " << trackedFrames << " of " << avatarTally.frames << " frames, "
						 << gatedTally.frames << " pass the " << maxRateHz << " Hz send rate";
	MIKAN_LOG_INFO(kTag) << backward.steps << " backward steps in the frame clock, the largest " << backward.largestMs
						 << " ms, " << backward.small << " of them within "
						 << SourcePose::Clock::kMaxBackwardStepMs << " ms, which the step clock holds across";
	report(retargetTally, "position retarget", "identical", &retargetExpected);
	report(avatarTally, "stream with an avatar, every frame", "identical", &avatarExpected);
	report(lengthTally, "stream without an avatar, every frame", "identical", &lengthExpected);
	MIKAN_LOG_INFO(kTag) << "sent frames only, any that differ by design included: "
						 << gatedStream.inexactFrames << " of " << gatedTally.frames
						 << " not bit-identical, largest difference " << gatedStream.maxPoseDifference
						 << " in the pose and " << gatedStream.maxStreamDifference << " in the stream against a tolerance of "
						 << kDecimatedTolerance << "; " << gatedStream.carryOverFrames
						 << " frames with an arm held or frozen compared for their structure, largest difference "
						 << gatedStream.maxCarryOverDifference;
	report(gatedTally, "stream with an avatar, sent frames only",
		   "within tolerance, or alike in structure where an arm is carried over", &gatedExpected);
	const bool bAllSame= retargetTally.mismatches == 0 && avatarTally.mismatches == 0 && lengthTally.mismatches == 0 &&
		gatedTally.mismatches == 0;
	return bAllSame && avatarTally.frames > 0 ? 0 : 1;
}
} // namespace

static int runRetargetGoldenTool(const TestArgs& args)
{
	if (args.empty())
	{
		MIKAN_LOG_ERROR(kTag) << "Usage: --retarget-golden <recording.jsonl>";
		return 1;
	}
	const std::string recordingPath= args[0];

	TrackingReplay replay;
	std::string error;
	if (!replay.load(recordingPath, error))
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": load failed: " << error;
		return 1;
	}
	if (replay.getFrameCount() == 0)
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": no frames";
		return 1;
	}
	replay.runAll();
	const AppConfig& config= replay.getRecordedConfig();

	const AvatarModel avatarModel= SyntheticAvatar::makeModel(eVrmVersion::Vrm1, false);
	const std::shared_ptr<const AvatarSkeleton> skeleton= std::make_shared<AvatarSkeleton>(avatarModel);
	const AvatarRigSettings rig;

	MIKAN_LOG_INFO(kTag) << recordingPath << ": " << replay.getFrameCount() << " frames, "
						 << (config.direct.enabled ? "direct mode" : "fused");
	return config.direct.enabled ? runDirect(replay, config, rig, skeleton)
								 : runPosition(replay, config, rig, skeleton);
}

MIKAN_REGISTER_TEST("--retarget-golden",
					"Retarget golden over a recording: the live retarget path against its frozen copy <recording.jsonl>",
					eTestCategory::Tool, runRetargetGoldenTool)
