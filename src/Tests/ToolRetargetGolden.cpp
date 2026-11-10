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

namespace
{
using namespace RetargetGolden;

const char* kTag= "retarget-golden";

void report(const Tally& tally, const char* what, const char* agreement= "identical")
{
	if (tally.mismatches == 0)
	{
		MIKAN_LOG_INFO(kTag) << "PASS " << what << ": " << tally.frames << " frames " << agreement;
	}
	else
	{
		MIKAN_LOG_ERROR(kTag) << "FAIL " << what << ": " << tally.mismatches << " of " << tally.frames
							  << " frames differ, first at " << tally.firstMismatch;
	}
}

StreamConfig makeStreamConfig(const AppConfig& config, const AvatarRigSettings& rig,
								   const std::shared_ptr<const AvatarSkeleton>& skeleton)
{
	const BodyDimensions bodyDimensions= makeBodyDimensions(config);
	StreamConfig oscConfig;
	oscConfig.maxRateHz= 0.f;
	oscConfig.minConfidence= config.osc.minConfidence;
	oscConfig.holdOnDropoutMs= config.osc.holdOnDropoutMs;
	oscConfig.forearmLengthMeters= bodyDimensions.forearmLengthMeters;
	oscConfig.upperArmLengthMeters= bodyDimensions.upperArmLengthMeters;
	oscConfig.shoulderWidthMeters= bodyDimensions.shoulderWidthMeters;
	oscConfig.vmcHeadOffsetMeters= config.osc.vmcHeadOffsetMeters;
	oscConfig.vmcFreezeOnLoss= config.osc.vmcFreezeOnLoss;
	oscConfig.bodyDimensions= bodyDimensions;
	oscConfig.avatarRetarget= makeAvatarRetargetConfig(config, rig);
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
	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIndex);
		if (cameraIndex >= (int)frame.perCamera.size() || !frame.perCameraValid[cameraIndex])
			continue;
		const TrackingFrameResult& cameraFrame= frame.perCamera[cameraIndex];
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
	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const TrackingFrameResult& frame= replay.getFrame(frameIndex).replayedFused;
		trackedFrames+= (frame.poses[0].tracked || frame.poses[1].tracked) ? 1 : 0;

		avatarTally.frames++;
		avatarStream.retarget(frame);
		avatarTally.note(avatarStream.encode(frame, why), frameIndex, "stream", why);
		lengthTally.frames++;
		lengthStream.retarget(frame);
		lengthTally.note(lengthStream.encode(frame, why), frameIndex, "stream", why);
		retargetTally.frames++;
		const bool bSame= position.solve(frame, streamConfig.bodyDimensions, *skeleton, streamConfig.avatarRetarget, why);
		retargetTally.note(bSame, frameIndex, "retarget", why);
		gatedStream.retarget(frame);
		if (gate.pass(frame.timestampMs, maxRateHz))
		{
			gatedTally.frames++;
			gatedTally.note(gatedStream.encode(frame, why), frameIndex, "stream", why);
		}
	}
	MIKAN_LOG_INFO(kTag) << "a hand tracked on " << trackedFrames << " of " << avatarTally.frames << " frames, "
						 << gatedTally.frames << " pass the " << maxRateHz << " Hz send rate";
	report(retargetTally, "position retarget");
	report(avatarTally, "stream with an avatar, every frame");
	report(lengthTally, "stream without an avatar, every frame");
	MIKAN_LOG_INFO(kTag) << "sent frames only: " << gatedStream.inexactFrames << " of " << gatedTally.frames
						 << " not bit-identical, largest difference " << gatedStream.maxPoseDifference
						 << " in the pose and " << gatedStream.maxStreamDifference << " in the stream against a tolerance of "
						 << kDecimatedTolerance << "; " << gatedStream.carryOverFrames
						 << " frames with an arm held or frozen compared for their structure, largest difference "
						 << gatedStream.maxCarryOverDifference;
	report(gatedTally, "stream with an avatar, sent frames only",
		   "within tolerance, or alike in structure where an arm is carried over");
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
