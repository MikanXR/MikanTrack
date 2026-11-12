#include "TestCommon.h"

#include <functional>

#include "glm/gtx/quaternion.hpp"

#include "AvatarSkeleton.h"
#include "MathGLM.h"
#include "RetargetGolden.h"
#include "SyntheticAvatar.h"
#include "SyntheticDirectUser.h"

// The retarget golden test: the live retarget path beside the frozen copy of
// it over synthetic sequences that walk every loss, hold and restart branch
// of both pipelines. Every frame must agree bit for bit: the avatar pose, the
// direct solve state, and the encoded VMC packets. A case that is expected to
// differ after a deliberate change states the difference here: (p7), where
// the live stage steps on the source's step clock across the backward steps
// face-only frames make.

namespace
{
using B= eHumanoidBone;
using namespace SyntheticAvatar;
using namespace SyntheticDirectUser;
using namespace RetargetGolden;

const char* kTag= "test-retarget-golden";
constexpr double kFrameMs= 33.0;

// -- The position pipeline's input --------------------------------------------

struct PositionRig
{
	AvatarModel model;
	std::shared_ptr<const AvatarSkeleton> skeleton;
};

PositionRig makePositionRig(eVrmVersion version, bool rotateLeftUpperArmNode)
{
	PositionRig rig;
	rig.model= makeModel(version, rotateLeftUpperArmNode);
	rig.skeleton= std::make_shared<AvatarSkeleton>(rig.model);
	return rig;
}

// A user larger than the avatar, so the reach scaling is not the identity
BodyDimensions makeUserDimensions()
{
	BodyDimensions user;
	user.upperArmLengthMeters= 0.30f;
	user.forearmLengthMeters= 0.26f;
	user.shoulderWidthMeters= 0.36f;
	return user;
}

// One hand in motion: the wrist pulled in, forward and down from the T-pose
// so the elbow bends, the palm turning, the forearm bent a little against
// the palm with a confidence that swings across the elbow hint's threshold,
// the shoulder swaying, the fingers curling
HandPose makeMovingHand(const AvatarSkeleton& skeleton, eHandSide side, double motionMs)
{
	const float t= (float)(motionMs * 0.001);
	const bool bLeft= side == eHandSide::Left;
	const float sy= bLeft ? 1.f : -1.f;
	const float phase= bLeft ? 0.f : 1.3f;
	const AvatarSkeleton::HandRest& hand= skeleton.getHand(side);
	const glm::quat restPalm= glm::quat_cast(glm::mat3(hand.palmFrameWorld));
	const glm::vec3 restShoulder= skeleton.getBone(bLeft ? B::LeftUpperArm : B::RightUpperArm).restPositionWorld;
	const glm::vec3 restWrist= skeleton.getBone(bLeft ? B::LeftHand : B::RightHand).restPositionWorld;

	HandPose pose;
	pose.tracked= true;
	pose.side= side;
	pose.presence= 1.f;
	pose.confidence= 0.9f;
	pose.skeleton= hand.skeleton;
	pose.hasWorldPose= true;
	const glm::quat turn= glm::angleAxis(0.5f * sinf(1.1f * t + phase), glm::normalize(glm::vec3(0.3f, 1.f, 0.2f)));
	pose.palmOrientationWorld= turn * restPalm;
	const glm::vec3 wrist= restWrist + glm::vec3(0.14f + 0.05f * sinf(2.1f * t + phase),
												 -sy * (0.12f + 0.04f * sinf(1.3f * t)),
												 -0.16f + 0.06f * sinf(1.7f * t + phase));
	const float halfPalm= pose.skeleton.baseInPalm[(int)eFinger::Middle].x;
	pose.palmPositionWorld= wrist + pose.palmOrientationWorld * glm::vec3(halfPalm, 0.f, 0.f);
	pose.hasForearmPose= true;
	pose.forearmOrientationWorld=
		pose.palmOrientationWorld * glm::angleAxis(0.25f * sinf(0.9f * t + phase), glm::vec3(0.f, 1.f, 0.f));
	pose.forearmConfidence= 0.6f + 0.35f * sinf(0.7f * t + phase);
	pose.hasShoulder= true;
	pose.shoulderPositionWorld=
		restShoulder + glm::vec3(0.02f * sinf(0.5f * t), 0.015f * sinf(0.4f * t + 1.f), 0.01f * sinf(0.6f * t));
	pose.shoulderConfidence= 0.8f;
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		const float f= (float)finger;
		pose.fingers[finger].lateral= 0.1f * sinf(1.4f * t + f + phase);
		pose.fingers[finger].proximal= 0.6f + 0.5f * sinf(1.9f * t + f + phase);
		pose.fingers[finger].intermediate= 0.5f + 0.4f * sinf(1.6f * t + 0.5f * f + phase);
		pose.fingers[finger].distal= 0.3f + 0.25f * sinf(1.3f * t + 0.3f * f + phase);
	}
	return pose;
}

TrackingFrameResult makePositionFrame(const AvatarSkeleton& skeleton, double motionMs, int frameIndex)
{
	const float t= (float)(motionMs * 0.001);
	TrackingFrameResult frame;
	frame.frameIndex= frameIndex;
	frame.poses[0]= makeMovingHand(skeleton, eHandSide::Left, motionMs);
	frame.poses[1]= makeMovingHand(skeleton, eHandSide::Right, motionMs);
	frame.head.valid= true;
	frame.head.positionWorld= skeleton.getBone(B::Head).restPositionWorld;
	frame.head.orientationWorld= glm::angleAxis(0.3f * sinf(0.8f * t), glm::vec3(0.f, 0.f, 1.f)) *
		glm::angleAxis(0.15f * sinf(1.2f * t), glm::vec3(0.f, 1.f, 0.f));
	frame.head.confidence= 0.9f;
	return frame;
}

StreamConfig makeStreamConfig(const std::shared_ptr<const AvatarSkeleton>& skeleton)
{
	StreamConfig config;
	config.maxRateHz= 0.f;
	config.bodyDimensions= makeUserDimensions();
	config.forearmLengthMeters= config.bodyDimensions.forearmLengthMeters;
	config.upperArmLengthMeters= config.bodyDimensions.upperArmLengthMeters;
	config.shoulderWidthMeters= config.bodyDimensions.shoulderWidthMeters;
	config.avatarSkeleton= skeleton;
	return config;
}

// What the frozen stream does with one hand on one frame, tracked beside
// the run so a sequence can assert it reached the branch it was written for
enum class eStreamed
{
	Live,
	Held,
	Frozen,
	Silent,
};

struct StreamShadow
{
	Frozen::OscStreamer::HeldPoseState held[2];
	Frozen::OscStreamer::HeldPoseState last[2];
	int counts[2][4]= {};

	void step(const TrackingFrameResult& frame, const StreamConfig& config)
	{
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const HandPose& pose= frame.poses[sideIndex];
			const bool bLive= pose.tracked && pose.confidence >= config.minConfidence;
			HandPose resolved;
			HandPose streamed;
			const bool bSent= Frozen::OscStreamer::resolveOutputPose(pose, frame.timestampMs, config.minConfidence,
																	 config.holdOnDropoutMs, held[sideIndex], resolved);
			const bool bStreamed= Frozen::OscStreamer::resolveVmcOutputPose(resolved, bSent, config.vmcFreezeOnLoss,
																			last[sideIndex], streamed);
			// A resolved pose without a world palm is not streamed as itself:
			// the freeze rule answers for it
			const bool bPassed= bSent && resolved.hasWorldPose;
			const eStreamed result= !bStreamed ? eStreamed::Silent
				: !bPassed ? eStreamed::Frozen
				: bLive ? eStreamed::Live
				: eStreamed::Held;
			counts[sideIndex][(int)result]++;
		}
	}

	int count(int sideIndex, eStreamed what) const { return counts[sideIndex][(int)what]; }
};

// One frame of a position sequence, as the sequence's edit sees it
struct PositionStep
{
	int index= 0;
	TrackingFrameResult frame;
	// Added to every later frame's timestamp: a gap or a regression
	double timeShiftMs= 0.0;
	StreamConfig config;
	bool bConfigDirty= false;
	// The frame is also sent again as a face-only frame stamped this far
	// ahead, the way a phone or microphone sample re-sends the last output
	double faceFrameAheadMs= -1.0;
};

// One encoded output frame (camera or face-only) as the live stage and the
// frozen streamer each made it, for a sequence that states a difference
struct OutputRecord
{
	int index= 0;
	bool bFaceFrame= false;
	double timestampMs= 0.0;
	SourcePose::Clock clock;
	SourcePose::eGroupState leftArm= SourcePose::eGroupState::Lost;
	// Bit-identical to the frozen side, the pose and the packets
	bool bSame= false;
	double livePoseTimestampMs= 0.0;
	glm::vec3 liveRoot{0.f};
	glm::quat liveRootRotation{1.f, 0.f, 0.f, 0.f};
	glm::vec3 frozenRoot{0.f};
	glm::quat frozenRootRotation{1.f, 0.f, 0.f, 0.f};
};

struct PositionResult
{
	Tally tally;
	StreamShadow shadow;
	// The retarget alone on the camera frames, which sees no face-only frame
	int retargetMismatches= 0;
	std::vector<OutputRecord> outputs;
	int encodedFrames= 0;
	// Under a tolerance: the largest difference the compare saw in the pose
	// and in the stream's floats, and the sent frames not bit-identical
	float maxPoseDifference= 0.f;
	float maxStreamDifference= 0.f;
	int inexactFrames= 0;
};

// The live retarget stage runs on every frame, ahead of the send-rate gate as
// on the vision thread; with bDecimate only the frames the gate passes are
// encoded, and the frozen streamer, which resolved only those, sees only
// those. A nonzero tolerance compares the stream within it.
PositionResult runPositionSequence(const PositionRig& inputRig, const StreamConfig& baseConfig, int frameCount,
								   double frameMs, bool bDecimate,
								   const std::function<void(PositionStep&)>& edit,
								   const std::function<void(StreamPair&)>& skew= nullptr, float tolerance= 0.f)
{
	PositionResult result;
	StreamPair stream;
	stream.tolerance= tolerance;
	PositionPair position;
	RateGate gate;
	std::string why;

	PositionStep step;
	step.config= baseConfig;
	stream.setConfig(step.config);
	if (skew)
		skew(stream);

	for (int index= 0; index < frameCount; ++index)
	{
		const double motionMs= index * frameMs;
		step.index= index;
		step.frame= makePositionFrame(*inputRig.skeleton, motionMs, index);
		step.bConfigDirty= false;
		step.faceFrameAheadMs= -1.0;
		const std::shared_ptr<const AvatarSkeleton> skeletonBefore= step.config.avatarSkeleton;
		if (edit)
			edit(step);
		if (step.bConfigDirty)
		{
			stream.setConfig(step.config);
			if (skew)
				skew(stream);
			// The streamer resets its own retarget on a skeleton change
			if (step.config.avatarSkeleton != skeletonBefore)
				position.reset();
		}
		step.frame.timestampMs= motionMs + step.timeShiftMs;
		result.tally.frames++;

		if (step.config.avatarSkeleton != nullptr)
		{
			const bool bSame= position.solve(step.frame, step.config.bodyDimensions, *step.config.avatarSkeleton,
											 step.config.avatarRetarget, step.config.elbowHintConfidence, why);
			result.tally.note(bSame, index, "retarget", why);
			result.retargetMismatches+= bSame ? 0 : 1;
		}

		auto record= [&](const TrackingFrameResult& frame, bool bFaceFrame, bool bSame) {
			OutputRecord output;
			output.index= index;
			output.bFaceFrame= bFaceFrame;
			output.timestampMs= frame.timestampMs;
			output.clock= stream.live.result.source.clock;
			output.leftArm= stream.live.result.source.arms[0].state;
			output.bSame= bSame;
			output.livePoseTimestampMs= stream.live.result.pose.timestampMs;
			output.liveRoot= stream.live.result.pose.rootPositionWorld;
			output.liveRootRotation= stream.live.result.pose.rootRotationWorld;
			output.frozenRoot= stream.frozenResolve.pose.rootPositionWorld;
			output.frozenRootRotation= stream.frozenResolve.pose.rootRotationWorld;
			result.outputs.push_back(output);
		};

		stream.retarget(step.frame);
		if (bDecimate && !gate.pass(step.frame.timestampMs, step.config.maxRateHz))
			continue;
		result.shadow.step(step.frame, step.config);
		const bool bSame= stream.encode(step.frame, why);
		result.tally.note(bSame, index, "stream", why);
		record(step.frame, false, bSame);
		result.encodedFrames++;

		if (step.faceFrameAheadMs >= 0.0)
		{
			TrackingFrameResult faceFrame= step.frame;
			faceFrame.timestampMs+= step.faceFrameAheadMs;
			result.shadow.step(faceFrame, step.config);
			stream.retarget(faceFrame);
			const bool bFaceSame= stream.encode(faceFrame, why);
			result.tally.note(bFaceSame, index, "stream face frame", why);
			record(faceFrame, true, bFaceSame);
		}
	}
	result.maxPoseDifference= stream.maxPoseDifference;
	result.maxStreamDifference= stream.maxStreamDifference;
	result.inexactFrames= stream.inexactFrames;
	return result;
}

// -- The direct pipeline's input ----------------------------------------------

constexpr float kTiltDegrees= 10.f;

struct DirectRig
{
	AvatarModel model;
	std::shared_ptr<const AvatarSkeleton> skeleton;
};

DirectRig makeDirectRig(eVrmVersion version, bool rotateLeftUpperArmNode)
{
	DirectRig rig;
	rig.model= makeModel(version, rotateLeftUpperArmNode);
	rig.skeleton= std::make_shared<AvatarSkeleton>(rig.model);
	return rig;
}

DirectRetargetConfig makeDirectConfig()
{
	DirectRetargetConfig config;
	config.cameraTiltRadians= glm::radians(kTiltDegrees);
	config.userShoulderWidthMeters= kShoulderWidth;
	config.userUpperArmLengthMeters= kUpperArmLength;
	config.userForearmLengthMeters= kForearmLength;
	return config;
}

// The neutral user in motion: the torso swaying, the upper arms swinging,
// the forearms sweeping across the desk, the palms and the head turning,
// the fingers curling
User makeMovingUser(double motionMs)
{
	User user= makeNeutralUser();
	const float t= (float)(motionMs * 0.001);
	const glm::vec3 sway(0.03f * sinf(0.6f * t), 0.02f * sinf(0.45f * t + 0.5f), 0.01f * sinf(0.5f * t));
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const float sy= sideIndex == 0 ? 1.f : -1.f;
		const float phase= sideIndex == 0 ? 0.f : 1.3f;
		user.shoulder[sideIndex]+= sway;
		user.ear[sideIndex]+= sway;
		const glm::vec3 upper=
			glm::normalize(glm::vec3(0.25f * sinf(0.9f * t + phase), sy * 0.2f * sinf(0.7f * t), -1.f));
		user.elbow[sideIndex]= user.shoulder[sideIndex] + upper * kUpperArmLength;
		const glm::vec3 fore= glm::normalize(
			glm::vec3(1.f, -sy * (0.25f + 0.2f * sinf(1.1f * t + phase)), 0.35f * sinf(1.3f * t + phase)));
		user.wrist[sideIndex]= user.elbow[sideIndex] + fore * kForearmLength;
		user.palm[sideIndex]= makePalm(fore + glm::vec3(0.f, 0.15f * sinf(1.7f * t), 0.1f * sinf(1.5f * t)),
									   glm::vec3(0.2f * sinf(0.8f * t + phase), sy * 0.3f * sinf(0.6f * t), -1.f));
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			const float f= (float)finger;
			user.fingers[sideIndex][finger].lateral= 0.08f * sinf(1.4f * t + f + phase);
			user.fingers[sideIndex][finger].proximal= 0.5f + 0.45f * sinf(1.9f * t + f + phase);
			user.fingers[sideIndex][finger].intermediate= 0.4f + 0.35f * sinf(1.6f * t + 0.5f * f + phase);
			user.fingers[sideIndex][finger].distal= 0.25f + 0.2f * sinf(1.3f * t + 0.3f * f + phase);
		}
	}
	user.nose+= sway + glm::vec3(0.f, 0.03f * sinf(0.8f * t), 0.02f * sinf(0.7f * t));
	return user;
}

struct DirectStep
{
	int index= 0;
	User user;
	double timeShiftMs= 0.0;
	DirectRetargetConfig config;
	TrackingFrameResult::HeadPose phoneHead;
	// The avatar changed: the app resets the solver and swaps the skeleton
	std::shared_ptr<const AvatarSkeleton> skeleton;
	bool bSkeletonDirty= false;
};

struct DirectResult
{
	Tally tally;
	// Frames the frozen solver spent in each state, per side
	int chainFrames[2]= {};
	int restingFrames[2]= {};
	int easingFrames[2]= {};
	int handHeldFrames[2]= {};
	int torsoFrames= 0;
};

DirectResult runDirectSequence(const DirectRig& rig, const DirectRetargetConfig& baseConfig, int frameCount,
							   const std::function<void(DirectStep&)>& edit,
							   const std::function<void(DirectRetargetConfig&)>& skewLive= nullptr)
{
	DirectResult result;
	const Camera camera= makeCamera(kTiltDegrees);
	DirectPair direct;
	StreamPair stream;
	std::string why;
	const TrackingFrameResult::HeadPose noPhone;

	DirectStep step;
	step.config= baseConfig;
	step.skeleton= rig.skeleton;
	stream.setConfig(makeStreamConfig(step.skeleton));

	// The neutral capture, as the app runs it: a few frames of the neutral
	// user, then both solvers' captures must agree before either is used
	double timestampMs= 0.0;
	{
		const User neutralUser= makeNeutralUser();
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, neutralUser, *step.skeleton, timestampMs, repeat);
			const bool bSame= direct.solve(frame, camera.model, noPhone, timestampMs, *step.skeleton, step.config, why);
			result.tally.note(bSame, repeat, "neutral solve", why);
			timestampMs+= kFrameMs;
		}
		result.tally.note(direct.captureNeutral(step.config.neutral, why), 0, "neutral capture", why);
	}
	const double startMs= timestampMs;

	for (int index= 0; index < frameCount; ++index)
	{
		const double motionMs= index * kFrameMs;
		step.index= index;
		step.user= makeMovingUser(motionMs);
		step.phoneHead= noPhone;
		step.bSkeletonDirty= false;
		if (edit)
			edit(step);
		if (step.bSkeletonDirty)
		{
			direct.reset();
			stream.setConfig(makeStreamConfig(step.skeleton));
		}
		timestampMs= startMs + motionMs + step.timeShiftMs;
		const TrackingFrameResult frame= makeFrame(camera, step.user, *step.skeleton, timestampMs, index + 3);
		result.tally.frames++;

		// A skew leaves the frozen side on the true config and hands the live
		// side another, which the compare must notice
		if (skewLive)
		{
			DirectRetargetConfig skewed= step.config;
			skewLive(skewed);
			direct.live.solve(frame, camera.model, step.phoneHead, timestampMs, *step.skeleton, skewed,
							  direct.livePose);
			direct.frozen.solve(frame, toFrozen(camera.model), step.phoneHead, timestampMs, *step.skeleton,
								toFrozen(step.config), direct.frozenPose);
			result.tally.note(samePose(direct.livePose, direct.frozenPose, why), index, "direct", why);
		}
		else
		{
			const bool bSame=
				direct.solve(frame, camera.model, step.phoneHead, timestampMs, *step.skeleton, step.config, why);
			result.tally.note(bSame, index, "direct", why);
		}
		result.tally.note(stream.encodeDirect(frame, direct.livePose, direct.frozenPose, why), index, "stream", why);

		const Frozen::DirectRetarget::SolveState& state= direct.frozen.getSolveState();
		result.torsoFrames+= state.torsoValid ? 1 : 0;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const Frozen::DirectRetarget::SolveState::Arm& arm= state.arms[sideIndex];
			result.chainFrames[sideIndex]+= arm.chain.valid ? 1 : 0;
			result.restingFrames[sideIndex]+= arm.bResting ? 1 : 0;
			result.easingFrames[sideIndex]+= (arm.bResting && arm.restBlend > 0.f && arm.restBlend < 1.f) ? 1 : 0;
			result.handHeldFrames[sideIndex]+=
				(arm.chain.valid && !arm.bResting && !step.user.bHandTracked[sideIndex]) ? 1 : 0;
		}
	}
	return result;
}

bool inRange(int index, int first, int last)
{
	return index >= first && index < last;
}
} // namespace

static int runRetargetGoldenTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const std::string& name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO(kTag) << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR(kTag) << "FAIL " << name;
			failures++;
		}
	};
	// A sequence passes when no frame differs and it reached the branches it
	// was written for
	auto checkSame= [&](const Tally& tally, bool bCovered, const std::string& name) {
		if (tally.mismatches != 0)
			MIKAN_LOG_ERROR(kTag) << "  first mismatch at " << tally.firstMismatch;
		check(tally.mismatches == 0 && bCovered,
			  name + " (" + std::to_string(tally.frames) + " frames, " + std::to_string(tally.mismatches) +
				  " differ" + (bCovered ? ")" : ", branch not reached)"));
	};

	const PositionRig rig= makePositionRig(eVrmVersion::Vrm1, false);
	const PositionRig otherRig= makePositionRig(eVrmVersion::Vrm0, true);
	const StreamConfig baseConfig= makeStreamConfig(rig.skeleton);

	// -- Position pipeline -----------------------------------------------------

	// (p1) Steady motion, both hands live throughout
	{
		const PositionResult result= runPositionSequence(rig, baseConfig, 120, kFrameMs, false, nullptr);
		checkSame(result.tally, result.shadow.count(0, eStreamed::Live) == 120, "(p1) steady motion");
	}

	// (p2) A dropout shorter than the hold, then one longer, freeze on, with
	// a measured forearm on the held pose
	{
		const PositionResult result= runPositionSequence(rig, baseConfig, 110, kFrameMs, false, [](PositionStep& step) {
			if (inRange(step.index, 30, 35) || inRange(step.index, 60, 85))
				step.frame.poses[0].tracked= false;
		});
		checkSame(result.tally,
				  result.shadow.count(0, eStreamed::Held) > 10 && result.shadow.count(0, eStreamed::Frozen) > 10 &&
					  result.shadow.count(0, eStreamed::Silent) == 0,
				  "(p2) short and long dropout, freeze on");
	}

	// (p3) The same with freeze off: past the hold the arm goes silent
	{
		StreamConfig config= baseConfig;
		config.vmcFreezeOnLoss= false;
		const PositionResult result= runPositionSequence(rig, config, 110, kFrameMs, false, [](PositionStep& step) {
			if (inRange(step.index, 30, 35) || inRange(step.index, 60, 85))
				step.frame.poses[1].tracked= false;
		});
		checkSame(result.tally,
				  result.shadow.count(1, eStreamed::Held) > 10 && result.shadow.count(1, eStreamed::Silent) > 10 &&
					  result.shadow.count(1, eStreamed::Frozen) == 0,
				  "(p3) short and long dropout, freeze off");
	}

	// (p4) The confidence gate: a tracked hand under the threshold is held,
	// then frozen
	{
		StreamConfig config= baseConfig;
		config.minConfidence= 0.5f;
		const PositionResult result= runPositionSequence(rig, config, 90, kFrameMs, false, [](PositionStep& step) {
			if (inRange(step.index, 20, 50))
				step.frame.poses[0].confidence= 0.3f;
		});
		checkSame(result.tally,
				  result.shadow.count(0, eStreamed::Held) > 0 && result.shadow.count(0, eStreamed::Frozen) > 0,
				  "(p4) confidence gate");
	}

	// (p5) A timestamp regression in the middle of a hold drops the hold and
	// snaps the root: 2000 ms back is past the step clock's largest small
	// backward step, a discontinuity as before
	{
		const PositionResult result= runPositionSequence(rig, baseConfig, 90, kFrameMs, false, [](PositionStep& step) {
			if (inRange(step.index, 48, 60))
				step.frame.poses[0].tracked= false;
			if (step.index == 50)
				step.timeShiftMs-= 2000.0;
		});
		checkSame(result.tally, result.shadow.count(0, eStreamed::Held) == 2, "(p5) timestamp regression in a hold");
	}

	// (p6) A gap longer than the hold while a hand is lost
	{
		const PositionResult result= runPositionSequence(rig, baseConfig, 90, kFrameMs, false, [](PositionStep& step) {
			if (inRange(step.index, 38, 50))
				step.frame.poses[1].tracked= false;
			if (step.index == 40)
				step.timeShiftMs+= 800.0;
		});
		checkSame(result.tally, result.shadow.count(1, eStreamed::Frozen) > 0, "(p6) gap over the hold");
	}

	// (p7) Face-only frames interleaved, stamped 45 ms after the camera frame
	// they follow, so the next camera frame steps 12 ms back, across a
	// dropout. This differs from the frozen path by design. The frozen stream
	// read each backward step as a regression: it dropped the hold the frame
	// the dropout began (held on 0 frames, frozen on 45) and snapped the root
	// to its measurement. The live stage steps on the source's step clock,
	// which stays where it was across a small backward step, so:
	// - every output frame before the first backward step is bit-identical,
	// and the retarget alone on the camera frames stays bit-identical
	// - the hand is held for the 250 ms hold in forward time from the last
	// frame that measured it (11 frames), then frozen (34 frames)
	// - on a backward step no time passes: the root does not move (where the
	// frozen root snapped), the interval is zero, and the step clock never
	// goes back
	// - the published pose carries the frame's own timestamp throughout
	{
		const PositionResult result= runPositionSequence(rig, baseConfig, 100, kFrameMs, false, [](PositionStep& step) {
			if (inRange(step.index, 40, 70))
				step.frame.poses[0].tracked= false;
			if (step.index % 2 == 1)
				step.faceFrameAheadMs= 45.0;
		});
		using eGroupState= SourcePose::eGroupState;
		const double holdMs= (double)baseConfig.holdOnDropoutMs;
		int firstBackward= -1;
		int firstDiffer= -1;
		int differ= 0;
		int backwardSteps= 0;
		int rootStill= 0;
		int frozenSnapped= 0;
		float largestSnap= 0.f;
		int clockWrong= 0;
		int timestampWrong= 0;
		int liveHeld= 0;
		int liveFrozen= 0;
		int liveLost= 0;
		double holdStartMs= -1.0;
		double lastHeldElapsedMs= -1.0;
		double firstFrozenElapsedMs= -1.0;
		for (size_t output= 0; output < result.outputs.size(); ++output)
		{
			const OutputRecord& record= result.outputs[output];
			const bool bBackward= output > 0 && record.timestampMs < result.outputs[output - 1].timestampMs;
			if (bBackward && firstBackward < 0)
				firstBackward= (int)output;
			if (!record.bSame)
			{
				differ++;
				if (firstDiffer < 0)
					firstDiffer= (int)output;
			}
			if (record.livePoseTimestampMs != record.timestampMs || record.clock.timestampMs != record.timestampMs)
				timestampWrong++;
			if (output > 0)
			{
				const OutputRecord& previous= result.outputs[output - 1];
				if (record.clock.frameIntervalMs < 0.0 || record.clock.stepMs < previous.clock.stepMs ||
					record.clock.frameIntervalMs != record.clock.stepMs - previous.clock.stepMs ||
					(bBackward && record.clock.frameIntervalMs != 0.0))
				{
					clockWrong++;
				}
			}
			if (bBackward)
			{
				backwardSteps++;
				const OutputRecord& previous= result.outputs[output - 1];
				if (sameBits(record.liveRoot, previous.liveRoot) &&
					sameBits(record.liveRootRotation, previous.liveRootRotation))
				{
					rootStill++;
				}
				const float snap= glm::length(record.frozenRoot - previous.frozenRoot);
				frozenSnapped+= snap > 0.f ? 1 : 0;
				largestSnap= std::max(largestSnap, snap);
			}
			switch (record.leftArm)
			{
			case eGroupState::Measured:
				holdStartMs= record.clock.stepMs;
				break;
			case eGroupState::Held:
				liveHeld++;
				lastHeldElapsedMs= record.clock.stepMs - holdStartMs;
				break;
			case eGroupState::Frozen:
				if (liveFrozen == 0)
					firstFrozenElapsedMs= record.clock.stepMs - holdStartMs;
				liveFrozen++;
				break;
			default:
				liveLost++;
				break;
			}
		}
		MIKAN_LOG_INFO(kTag) << "  (p7) " << differ << " of " << result.outputs.size()
							 << " output frames differ from the frozen path, the first at output " << firstDiffer
							 << " where the first backward step is at " << firstBackward << "; " << backwardSteps
							 << " backward steps";
		MIKAN_LOG_INFO(kTag) << "  (p7) left hand held on " << liveHeld << " frames (frozen path "
							 << result.shadow.count(0, eStreamed::Held) << "), frozen on " << liveFrozen
							 << " (frozen path " << result.shadow.count(0, eStreamed::Frozen)
							 << "); the last held frame " << lastHeldElapsedMs << " ms and the first frozen " << firstFrozenElapsedMs
							 << " ms into the hold";
		MIKAN_LOG_INFO(kTag) << "  (p7) the root still on " << rootStill << " of " << backwardSteps
							 << " backward steps, where the frozen root snapped on " << frozenSnapped << " by up to "
							 << largestSnap << " m; clock wrong on " << clockWrong << ", timestamp wrong on "
							 << timestampWrong;
		check(result.retargetMismatches == 0 && firstBackward > 0 && firstDiffer >= firstBackward,
			  "(p7) interleaved face-only frames: bit-identical up to the first backward step, the retarget alone "
			  "throughout");
		check(result.shadow.count(0, eStreamed::Held) == 0 && result.shadow.count(0, eStreamed::Frozen) == 45 &&
				  liveHeld == 11 && liveFrozen == 34 && liveLost == 0 && lastHeldElapsedMs >= 0.0 &&
				  lastHeldElapsedMs <= holdMs && firstFrozenElapsedMs > holdMs,
			  "(p7) interleaved face-only frames: the hand held for the hold in forward time, then frozen");
		check(backwardSteps == 49 && rootStill == backwardSteps && frozenSnapped == backwardSteps && clockWrong == 0 &&
				  timestampWrong == 0,
			  "(p7) interleaved face-only frames: no time passes on a backward step, the root does not snap");
	}

	// (p8) Missing measurements: no shoulder, no forearm pose, a camera-space
	// only palm, no head
	{
		const PositionResult result= runPositionSequence(rig, baseConfig, 120, kFrameMs, false, [](PositionStep& step) {
			if (inRange(step.index, 20, 45))
				step.frame.poses[0].hasShoulder= false;
			if (inRange(step.index, 35, 60))
				step.frame.poses[1].hasForearmPose= false;
			if (inRange(step.index, 70, 80))
				step.frame.poses[0].hasWorldPose= false;
			if (inRange(step.index, 90, 100))
				step.frame.head.valid= false;
		});
		checkSame(result.tally, result.shadow.count(0, eStreamed::Frozen) == 10, "(p8) missing measurements");
	}

	// (p9) No avatar: the measured-length chain, through the same dropouts
	{
		StreamConfig config= baseConfig;
		config.avatarSkeleton= nullptr;
		const PositionResult result= runPositionSequence(rig, config, 110, kFrameMs, false, [](PositionStep& step) {
			if (inRange(step.index, 30, 35) || inRange(step.index, 60, 85))
				step.frame.poses[0].tracked= false;
			if (inRange(step.index, 40, 50))
				step.frame.poses[1].hasShoulder= false;
		});
		checkSame(result.tally, result.shadow.count(0, eStreamed::Frozen) > 10, "(p9) no avatar, measured-length chain");
	}

	// (p10) The avatar swapped for another and back, and the rig edited, in
	// the middle of the stream
	{
		const PositionResult result=
			runPositionSequence(rig, baseConfig, 150, kFrameMs, false, [&](PositionStep& step) {
				if (step.index == 30)
				{
					step.config.avatarSkeleton= otherRig.skeleton;
					step.bConfigDirty= true;
				}
				if (step.index == 60)
				{
					step.config.avatarSkeleton= rig.skeleton;
					step.bConfigDirty= true;
				}
				if (step.index == 80)
				{
					AvatarRetargetConfig& retarget= step.config.avatarRetarget;
					retarget.headTrimRadians= glm::vec3(0.1f, -0.05f, 0.2f);
					retarget.sides[0].elbowHintOffset= glm::vec3(-0.05f, 0.1f, -0.35f);
					step.config.elbowHintConfidence[0]= 0.8f;
					retarget.sides[0].handTrimRadians= glm::vec3(0.2f, 0.1f, -0.15f);
					retarget.sides[0].forearmRollTrimRadians= 0.3f;
					retarget.sides[1].curlGain= 1.4f;
					retarget.sides[1].splayGain= 0.6f;
					retarget.sides[1].fingerEnabled[(int)eFinger::Ring]= false;
					retarget.sides[1].bThumbPronationOverride= true;
					retarget.sides[1].thumbPronationRad= -0.7f;
					step.bConfigDirty= true;
				}
				if (step.index == 110)
				{
					step.config.avatarRetarget.followShoulders= false;
					step.config.avatarRetarget.fixedRootPositionWorld= glm::vec3(0.1f, -0.2f, 0.05f);
					step.config.avatarRetarget.fixedRootYawDegrees= 25.f;
					step.bConfigDirty= true;
				}
				if (inRange(step.index, 120, 140))
					step.frame.poses[0].tracked= false;
			});
		checkSame(result.tally, result.shadow.count(0, eStreamed::Frozen) > 0, "(p10) avatar swap and rig edit");
	}

	// (p11) A camera faster than the send rate: only the frames a live send
	// would pass are encoded. The frozen streamer resolved and retargeted only
	// those, while the live retarget stage runs on every frame ahead of the
	// send-rate gate, so this run differs by design: the hold's start, its
	// decay and the frame it ends on, and the root follow's steps, follow
	// every camera frame instead of every sent one. Every sent frame's pose
	// and stream floats, held and frozen arms included, are compared within
	// kDecimatedTolerance (0.05), the addresses, the bone set, the strings and
	// the ints exactly. The largest differences measured are 0.011 in the pose
	// and 0.019 in the stream, on 123 of the 124 sent frames.
	{
		StreamConfig config= baseConfig;
		config.maxRateHz= 60.f;
		const PositionResult result= runPositionSequence(
			rig, config, 360, 1000.0 / 120.0, true,
			[](PositionStep& step) {
				if (inRange(step.index, 120, 200))
					step.frame.poses[0].tracked= false;
			},
			nullptr, kDecimatedTolerance);
		MIKAN_LOG_INFO(kTag) << "  (p11) " << result.encodedFrames << " of " << result.tally.frames << " frames sent, "
							 << result.inexactFrames << " of them not bit-identical, largest difference "
							 << result.maxPoseDifference << " in the pose and " << result.maxStreamDifference
							 << " in the stream against a tolerance of " << kDecimatedTolerance;
		checkSame(result.tally, result.encodedFrames < result.tally.frames && result.encodedFrames > 0,
				  "(p11) decimating send rate, within tolerance");
	}

	// -- Direct pipeline -------------------------------------------------------

	const DirectRig directRig= makeDirectRig(eVrmVersion::Vrm1, false);
	const DirectRig otherDirectRig= makeDirectRig(eVrmVersion::Vrm0, true);
	const DirectRetargetConfig directConfig= makeDirectConfig();

	// (d1) Steady motion: both chains placed on every frame
	{
		const DirectResult result= runDirectSequence(directRig, directConfig, 150, nullptr);
		MIKAN_LOG_INFO(kTag) << "  (d1) chains placed on " << result.chainFrames[0] << " and " << result.chainFrames[1]
							 << " of " << result.tally.frames << " frames";
		checkSame(result.tally,
				  result.chainFrames[0] > 140 && result.chainFrames[1] > 140 && result.torsoFrames == 150,
				  "(d1) steady motion");
	}

	// (d2) The hand lost while its arm stays placed: the palm and the fingers hold
	{
		const DirectResult result= runDirectSequence(directRig, directConfig, 100, [](DirectStep& step) {
			if (inRange(step.index, 30, 60))
				step.user.bHandTracked[0]= false;
		});
		checkSame(result.tally, result.handHeldFrames[0] > 20, "(d2) hand lost on a placed arm");
	}

	// (d3) The wrist lost for less than the hold, then for longer: the arm
	// holds, eases to rest, and takes the wrist back in the middle of an ease
	{
		const DirectResult result= runDirectSequence(directRig, directConfig, 170, [](DirectStep& step) {
			const bool bLost= inRange(step.index, 30, 35) || inRange(step.index, 60, 100) ||
				inRange(step.index, 120, 139);
			if (bLost)
			{
				step.user.bWristSeen[0]= false;
				step.user.bHandTracked[0]= false;
			}
		});
		MIKAN_LOG_INFO(kTag) << "  (d3) left arm resting on " << result.restingFrames[0] << " frames, easing on "
							 << result.easingFrames[0];
		checkSame(result.tally, result.restingFrames[0] > 20 && result.easingFrames[0] > 4,
				  "(d3) wrist lost: hold, ease to rest, return mid-ease");
	}

	// (d4) A rest blend of zero snaps to the rest pose
	{
		DirectRetargetConfig config= directConfig;
		config.restBlendMs= 0.f;
		const DirectResult result= runDirectSequence(directRig, config, 110, [](DirectStep& step) {
			if (inRange(step.index, 40, 80))
			{
				step.user.bWristSeen[1]= false;
				step.user.bHandTracked[1]= false;
			}
		});
		checkSame(result.tally, result.restingFrames[1] > 10 && result.easingFrames[1] == 0, "(d4) rest blend zero");
	}

	// (d5) A gap over the hold, then a timestamp regression: both restart the solver
	{
		const DirectResult result= runDirectSequence(directRig, directConfig, 120, [](DirectStep& step) {
			if (step.index == 40)
				step.timeShiftMs+= 900.0;
			if (step.index == 80)
				step.timeShiftMs-= 3000.0;
		});
		checkSame(result.tally, result.chainFrames[0] > 100, "(d5) gap and timestamp regression");
	}

	// (d6) The phone head takes the head over and hands it back
	{
		const DirectResult result= runDirectSequence(directRig, directConfig, 100, [](DirectStep& step) {
			if (inRange(step.index, 30, 70))
			{
				const float t= (float)step.index * 0.033f;
				step.phoneHead.valid= true;
				step.phoneHead.orientationWorld= glm::angleAxis(0.6f * sinf(1.5f * t), glm::vec3(0.f, 0.f, 1.f)) *
					glm::angleAxis(0.3f * sinf(1.1f * t), glm::vec3(0.f, 1.f, 0.f));
				step.phoneHead.confidence= 1.f;
			}
		});
		checkSame(result.tally, result.torsoFrames == 100, "(d6) phone head");
	}

	// (d7) Weak evidence: an unseen elbow, no model depth, a biased hand
	// lift, a hand slot on something else, a wrist flickering across the bar
	{
		const DirectResult result= runDirectSequence(directRig, directConfig, 200, [](DirectStep& step) {
			if (inRange(step.index, 20, 50))
				step.user.bElbowSeen[0]= false;
			if (inRange(step.index, 60, 90))
				step.user.bModelDepth= false;
			if (inRange(step.index, 100, 130))
				step.user.pnpDepthScale[1]= 1.3f;
			if (inRange(step.index, 140, 160))
				step.user.handPixelOffset[0]= glm::vec2(140.f, -90.f);
			if (inRange(step.index, 170, 195))
				step.user.wristVisibility[1]= (step.index % 2 == 0) ? 0.45f : 0.55f;
		});
		checkSame(result.tally, result.chainFrames[0] > 150, "(d7) weak evidence");
	}

	// (d8) The avatar swapped for another and back, and the rig edited, in
	// the middle of the stream
	{
		const DirectResult result= runDirectSequence(directRig, directConfig, 190, [&](DirectStep& step) {
			if (step.index == 40)
			{
				step.skeleton= otherDirectRig.skeleton;
				step.bSkeletonDirty= true;
			}
			if (step.index == 80)
			{
				step.skeleton= directRig.skeleton;
				step.bSkeletonDirty= true;
			}
			if (step.index == 110)
			{
				step.config.headTrimRadians= glm::vec3(0.1f, -0.05f, 0.2f);
				step.config.sides[0].handTrimRadians= glm::vec3(0.2f, 0.1f, -0.15f);
				step.config.sides[0].forearmRollTrimRadians= 0.3f;
				step.config.sides[0].elbowHintOffset= glm::vec3(-0.05f, 0.1f, -0.35f);
				step.config.sides[1].curlGain= 1.4f;
				step.config.sides[1].splayGain= 0.6f;
				step.config.sides[1].fingerEnabled[(int)eFinger::Ring]= false;
				step.config.sides[1].bThumbPronationOverride= true;
				step.config.sides[1].thumbPronationRad= -0.7f;
				step.config.sides[1].restWristOffset= glm::vec3(0.2f, 0.1f, -0.3f);
				step.config.sides[1].restFingerCurlRadians= 0.5f;
				step.config.restBlendMs= 150.f;
			}
			if (inRange(step.index, 130, 170))
			{
				step.user.bWristSeen[1]= false;
				step.user.bHandTracked[1]= false;
			}
		});
		checkSame(result.tally, result.restingFrames[1] > 10, "(d8) avatar swap and rig edit");
	}

	// (d9) The rest pose the preview shows, under the default rig and an
	// edited one
	{
		Tally tally;
		std::string why;
		DirectRetargetConfig edited= directConfig;
		edited.sides[0].restElbowOffset= glm::vec3(0.1f, 0.05f, -0.25f);
		edited.sides[1].restWristOffset= glm::vec3(0.2f, 0.1f, -0.3f);
		edited.sides[1].restPalmNormal= glm::vec3(0.f, -0.3f, -1.f);
		edited.sides[0].restFingerCurlRadians= 0.5f;
		edited.sides[0].handTrimRadians= glm::vec3(0.2f, 0.1f, -0.15f);
		const DirectRetargetConfig* configs[2]= {&directConfig, &edited};
		for (int index= 0; index < 2; ++index)
		{
			AvatarPose livePose;
			Frozen::AvatarPose frozenPose;
			DirectRetarget::makeRestPose(*directRig.skeleton, *configs[index], livePose);
			Frozen::DirectRetarget::makeRestPose(*directRig.skeleton, toFrozen(*configs[index]), frozenPose);
			tally.frames++;
			tally.note(samePose(livePose, frozenPose, why), index, "rest pose", why);
		}
		checkSame(tally, true, "(d9) rest pose");
	}

	// -- The source pose's clock and group states ------------------------------

	// (c1) The position source pose: the clock is the frame's, its interval
	// runs from the last fill, a reset starts a new epoch, and a side the
	// stream did not pass, or a lost head, reads as lost
	{
		AvatarRetarget retarget;
		const BodyDimensions user= makeUserDimensions();
		SourcePose source;
		uint32_t firstEpoch= 0;
		int clockWrong= 0;
		int statesWrong= 0;
		int confidencesWrong= 0;
		for (int index= 0; index < 40; ++index)
		{
			const double timestampMs= index * kFrameMs;
			TrackingFrameResult frame= makePositionFrame(*rig.skeleton, timestampMs, index);
			frame.timestampMs= timestampMs;
			const bool bLeftDropped= inRange(index, 10, 20);
			const bool bHeadDropped= inRange(index, 15, 25);
			if (bHeadDropped)
				frame.head.valid= false;
			if (index == 30)
				retarget.reset();
			const bool bSideValid[2]= {!bLeftDropped, true};
			retarget.fillSource(frame.poses, bSideValid, frame.head, frame.timestampMs, user, source);
			if (index == 0)
				firstEpoch= source.clock.resetEpoch;

			const bool bEpochStart= index == 0 || index == 30;
			const uint32_t expectedEpoch= firstEpoch + (index >= 30 ? 1u : 0u);
			if (source.pipeline != SourcePose::ePipeline::Position || source.clock.timestampMs != timestampMs ||
				source.clock.stepMs != timestampMs || source.clock.frameIntervalMs != (bEpochStart ? 0.0 : kFrameMs) ||
				source.clock.resetEpoch != expectedEpoch)
			{
				clockWrong++;
			}

			using eGroupState= SourcePose::eGroupState;
			const eGroupState left= bLeftDropped ? eGroupState::Lost : eGroupState::Measured;
			if (source.arms[0].state != left || source.hands[0].state != left ||
				source.arms[1].state != eGroupState::Measured || source.hands[1].state != eGroupState::Measured ||
				source.torso.state != left ||
				source.head.state != (bHeadDropped ? eGroupState::Lost : eGroupState::Measured))
			{
				statesWrong++;
			}

			const SourcePose::Arm& arm= source.arms[1];
			const SourcePose::Hand& hand= source.hands[1];
			if (!hand.confidence.bPresent || hand.confidence.value != frame.poses[1].confidence ||
				!arm.forearmConfidence.bPresent || arm.forearmConfidence.value != frame.poses[1].forearmConfidence ||
				!arm.shoulderConfidence.bPresent || arm.shoulderConfidence.value != frame.poses[1].shoulderConfidence ||
				hand.fingerZero != SourcePose::eFingerZero::CapturedRest ||
				source.head.confidence.bPresent != !bHeadDropped)
			{
				confidencesWrong++;
			}
		}
		MIKAN_LOG_INFO(kTag) << "  (c1) clock wrong on " << clockWrong << ", states on " << statesWrong
							 << ", confidences on " << confidencesWrong << " of 40 frames";
		check(clockWrong == 0 && statesWrong == 0 && confidencesWrong == 0,
			  "(c1) position source pose: clock, epoch, group states and confidences");
	}

	// (c2) The direct source pose: a dropped hand on a placed arm reads as
	// held, a lost wrist holds the arm and then loses it, a gap, a
	// regression and a reset each start a new epoch, and the phone head
	// reads as measured with its confidence
	{
		const Camera camera= makeCamera(kTiltDegrees);
		DirectRetarget retarget;
		SourcePose source;
		using eGroupState= SourcePose::eGroupState;
		uint32_t firstEpoch= 0;
		uint32_t restarts= 0;
		double timeShiftMs= 0.0;
		double lastTimestampMs= 0.0;
		int clockWrong= 0;
		int steadyWrong= 0;
		int handDropWrong= 0;
		int shortLossWrong= 0;
		int longLossHeld= 0;
		int longLossLost= 0;
		int longLossHandWrong= 0;
		int phoneWrong= 0;
		for (int index= 0; index < 170; ++index)
		{
			User user= makeMovingUser(index * kFrameMs);
			TrackingFrameResult::HeadPose phoneHead;
			const bool bHandDropped= inRange(index, 30, 40);
			const bool bShortLoss= inRange(index, 50, 55);
			const bool bLongLoss= inRange(index, 70, 101);
			const bool bPhone= inRange(index, 150, 160);
			if (bHandDropped)
				user.bHandTracked[0]= false;
			if (bShortLoss || bLongLoss)
			{
				user.bWristSeen[0]= false;
				user.bHandTracked[0]= false;
			}
			if (bPhone)
			{
				phoneHead.valid= true;
				phoneHead.orientationWorld= glm::angleAxis(0.3f, glm::vec3(0.f, 0.f, 1.f));
				phoneHead.confidence= 0.75f;
			}
			bool bRestart= index == 0;
			if (index == 110)
			{
				timeShiftMs+= 900.0;
				bRestart= true;
			}
			if (index == 130)
			{
				timeShiftMs-= 3000.0;
				bRestart= true;
			}
			if (index == 140)
			{
				retarget.reset();
				bRestart= true;
			}
			restarts+= (bRestart && index != 0) ? 1u : 0u;
			const double timestampMs= index * kFrameMs + timeShiftMs;
			const TrackingFrameResult frame= makeFrame(camera, user, *directRig.skeleton, timestampMs, index);
			retarget.measure(frame, camera.model, phoneHead, timestampMs, *directRig.skeleton, directConfig, source);
			if (index == 0)
				firstEpoch= source.clock.resetEpoch;

			if (source.pipeline != SourcePose::ePipeline::Direct || source.clock.timestampMs != timestampMs ||
				source.clock.stepMs != timestampMs ||
				source.clock.frameIntervalMs != (bRestart ? 0.0 : timestampMs - lastTimestampMs) ||
				source.clock.resetEpoch != firstEpoch + restarts)
			{
				clockWrong++;
			}
			lastTimestampMs= timestampMs;

			const SourcePose::Arm& arm= source.arms[0];
			const SourcePose::Hand& hand= source.hands[0];
			if (inRange(index, 20, 30) &&
				(source.torso.state != eGroupState::Measured || source.head.state != eGroupState::Measured ||
				 source.head.confidence.bPresent || arm.state != eGroupState::Measured ||
				 hand.state != eGroupState::Measured || !hand.confidence.bPresent ||
				 hand.fingerZero != SourcePose::eFingerZero::Metacarpal))
			{
				steadyWrong++;
			}
			if (bHandDropped && (arm.state != eGroupState::Measured || hand.state != eGroupState::Held))
				handDropWrong++;
			if (bShortLoss && arm.state != eGroupState::Held)
				shortLossWrong++;
			if (bLongLoss)
			{
				longLossHeld+= arm.state == eGroupState::Held ? 1 : 0;
				longLossLost+= arm.state == eGroupState::Lost ? 1 : 0;
				longLossHandWrong+= (arm.state == eGroupState::Lost && hand.state != eGroupState::Lost) ? 1 : 0;
			}
			if (bPhone &&
				(source.head.state != eGroupState::Measured || !source.head.confidence.bPresent ||
				 source.head.confidence.value != phoneHead.confidence))
			{
				phoneWrong++;
			}
		}
		MIKAN_LOG_INFO(kTag) << "  (c2) clock wrong on " << clockWrong << " frames, " << restarts
							 << " restarts; the long wrist loss held " << longLossHeld << " frames and lost "
							 << longLossLost;
		check(clockWrong == 0 && restarts == 3,
			  "(c2) direct source pose: clock and epoch through a gap, a regression and a reset");
		check(steadyWrong == 0, "(c2) direct source pose: every group measured in steady motion");
		check(handDropWrong == 0, "(c2) direct source pose: a dropped hand on a placed arm reads as held");
		check(shortLossWrong == 0 && longLossHeld > 0 && longLossLost > 0 && longLossHandWrong == 0,
			  "(c2) direct source pose: a lost wrist holds the arm, then loses it with its hand");
		check(phoneWrong == 0, "(c2) direct source pose: the phone head measured with its confidence");
	}

	// -- The compare is not vacuous -------------------------------------------

	// (s1) A live stream configured with another elbow hint must differ
	{
		StreamConfig skewed= baseConfig;
		skewed.avatarRetarget.sides[0].elbowHintOffset= glm::vec3(0.1f, 0.2f, -0.2f);
		const PositionResult result= runPositionSequence(
			rig, baseConfig, 60, kFrameMs, false,
			[](PositionStep& step) { step.frame.poses[0].hasForearmPose= false; },
			[&](StreamPair& stream) { stream.live.setConfig(skewed); });
		check(result.tally.mismatches > 0, "(s1) a skewed live elbow hint is detected in the stream");
	}

	// (s3) A live stream whose elbow weight is not the default function of
	// the frozen threshold must differ: the forearm confidence runs from 0.25
	// to 0.95, below the threshold of 0.5 for part of the run, where the
	// plain weight 1 (the threshold of zero's) bends toward the measured
	// elbow alone
	{
		StreamConfig skewed= baseConfig;
		skewed.elbowHintConfidence= {0.f, 0.f};
		const PositionResult result= runPositionSequence(rig, baseConfig, 120, kFrameMs, false, nullptr,
														 [&](StreamPair& stream) { stream.live.setConfig(skewed); });
		check(result.tally.mismatches > 0, "(s3) a skewed live elbow weight is detected in the stream");
	}

	// (s2) A live direct solver configured with another rest blend must differ
	{
		const DirectResult result= runDirectSequence(
			directRig, directConfig, 110,
			[](DirectStep& step) {
				if (inRange(step.index, 40, 80))
				{
					step.user.bWristSeen[0]= false;
					step.user.bHandTracked[0]= false;
				}
			},
			[](DirectRetargetConfig& config) { config.restBlendMs= 120.f; });
		check(result.tally.mismatches > 0, "(s2) a skewed live rest blend is detected in the direct pose");
	}

	MIKAN_LOG_INFO(kTag) << (failures == 0 ? "ALL PASSED" : "FAILED") << " (" << failures << " failures)";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-retarget-golden",
					"Retarget golden: the live retarget path against its frozen copy, bit for bit",
					eTestCategory::SelfTest, runRetargetGoldenTest)
