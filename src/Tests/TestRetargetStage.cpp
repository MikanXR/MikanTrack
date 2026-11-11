#include "TestCommon.h"

#include <cstring>
#include <functional>

#include "AvatarSkeleton.h"
#include "LossStage.h"
#include "OscStreamer.h"
#include "RetargetChains.h"
#include "RetargetStage.h"
#include "SourcePoseInputs.h"
#include "SyntheticAvatar.h"
#include "SyntheticDirectUser.h"

// The retarget stage: the loss stage's hold and freeze, the display stage
// against the stream stage, a scrub's pre-roll against a straight
// play-through, the step clock across interleaved face-only frames, and the
// elbow weight evaluated over the source pose's inputs.

namespace
{
using B= eHumanoidBone;
using namespace SyntheticAvatar;
using namespace SyntheticDirectUser;
using eGroupState= SourcePose::eGroupState;

const char* kTag= "test-retarget-stage";
constexpr double kFrameMs= 33.0;

// One position frame through the loss stage alone: the frame's left hand
// filled into a source pose, then the stage's position rules
struct LossProbe
{
	AvatarRetarget filler;
	LossStage loss;
	SourcePose source;
	LossHands hands;
	BodyDimensions user;

	void step(const HandPose& left, double timestampMs, const LossPolicy& policy)
	{
		TrackingFrameResult frame;
		frame.timestampMs= timestampMs;
		frame.poses[0]= left;
		frame.poses[0].side= eHandSide::Left;
		frame.poses[1].side= eHandSide::Right;
		filler.fillSource(frame, user, source);
		hands.poses= frame.poses;
		const std::array<AvatarRetargetConfig::Side, 2> sides{};
		loss.apply(policy, nullptr, sides, source, &hands);
	}

	// What the side carries: measured, held or frozen
	bool carried() const { return hands.bCarried[0]; }
	bool held() const { return source.arms[0].state == eGroupState::Held; }
	const HandPose& out() const { return hands.poses[0]; }
};

// The largest difference between two poses' roots and bone deltas, or a
// negative value when they place different bones
float poseDifference(const AvatarPose& a, const AvatarPose& b)
{
	if (a.valid != b.valid)
		return -1.f;
	float difference= 0.f;
	for (int axis= 0; axis < 3; ++axis)
		difference= std::max(difference, std::fabs(a.rootPositionWorld[axis] - b.rootPositionWorld[axis]));
	for (int axis= 0; axis < 4; ++axis)
		difference= std::max(difference, std::fabs(a.rootRotationWorld[axis] - b.rootRotationWorld[axis]));
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		if (a.bones[index].present != b.bones[index].present)
			return -1.f;
		for (int axis= 0; axis < 4; ++axis)
			difference= std::max(difference, std::fabs(a.bones[index].deltaWorld[axis] - b.bones[index].deltaWorld[axis]));
	}
	return difference;
}

bool samePoseBits(const AvatarPose& a, const AvatarPose& b)
{
	if (a.valid != b.valid || std::memcmp(&a.timestampMs, &b.timestampMs, sizeof(double)) != 0 ||
		std::memcmp(&a.rootPositionWorld, &b.rootPositionWorld, sizeof(glm::vec3)) != 0 ||
		std::memcmp(&a.rootRotationWorld, &b.rootRotationWorld, sizeof(glm::quat)) != 0)
		return false;
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		if (a.bones[index].present != b.bones[index].present ||
			std::memcmp(&a.bones[index].deltaWorld, &b.bones[index].deltaWorld, sizeof(glm::quat)) != 0)
			return false;
	}
	return true;
}

bool sameStates(const SourcePose& a, const SourcePose& b)
{
	return a.torso.state == b.torso.state && a.head.state == b.head.state && a.arms[0].state == b.arms[0].state &&
		a.arms[1].state == b.arms[1].state && a.hands[0].state == b.hands[0].state &&
		a.hands[1].state == b.hands[1].state;
}

// -- The position input: both hands moving, the wrists pulled in from the
// T-pose so the elbows bend, the shoulders swaying --------------------------

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
	pose.palmOrientationWorld=
		glm::angleAxis(0.5f * sinf(1.1f * t + phase), glm::normalize(glm::vec3(0.3f, 1.f, 0.2f))) * restPalm;
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
		pose.fingers[finger].proximal= 0.6f + 0.5f * sinf(1.9f * t + f + phase);
		pose.fingers[finger].intermediate= 0.5f + 0.4f * sinf(1.6f * t + 0.5f * f + phase);
	}
	return pose;
}

// The position frames of one run: the left hand lost over [lostFrom,
// lostTo), and every other frame followed by a face-only frame stamped
// 45 ms later, as the vision thread interleaves them
TrackingFrameResult makePositionFrame(const AvatarSkeleton& skeleton, int index, int lostFrom, int lostTo)
{
	const double timestampMs= index * kFrameMs;
	TrackingFrameResult frame;
	frame.frameIndex= index;
	frame.timestampMs= timestampMs;
	frame.poses[0]= makeMovingHand(skeleton, eHandSide::Left, timestampMs);
	frame.poses[1]= makeMovingHand(skeleton, eHandSide::Right, timestampMs);
	if (index >= lostFrom && index < lostTo)
		frame.poses[0].tracked= false;
	frame.head.valid= true;
	frame.head.orientationWorld= glm::angleAxis(0.3f * sinf(0.8f * (float)(timestampMs * 0.001)), glm::vec3(0.f, 0.f, 1.f));
	frame.head.confidence= 0.9f;
	return frame;
}

// -- The direct input: the neutral user's arms swinging, the left wrist lost
// over a span --------------------------------------------------------------

User makeMovingUser(double motionMs)
{
	User user= makeNeutralUser();
	const float t= (float)(motionMs * 0.001);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const float sy= sideIndex == 0 ? 1.f : -1.f;
		const float phase= sideIndex == 0 ? 0.f : 1.3f;
		const glm::vec3 upper=
			glm::normalize(glm::vec3(0.25f * sinf(0.9f * t + phase), sy * 0.2f * sinf(0.7f * t), -1.f));
		user.elbow[sideIndex]= user.shoulder[sideIndex] + upper * kUpperArmLength;
		const glm::vec3 fore= glm::normalize(
			glm::vec3(1.f, -sy * (0.25f + 0.2f * sinf(1.1f * t + phase)), 0.35f * sinf(1.3f * t + phase)));
		user.wrist[sideIndex]= user.elbow[sideIndex] + fore * kForearmLength;
		user.palm[sideIndex]= makePalm(fore, glm::vec3(0.2f * sinf(0.8f * t + phase), sy * 0.3f * sinf(0.6f * t), -1.f));
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
			user.fingers[sideIndex][finger].proximal= 0.5f + 0.45f * sinf(1.9f * t + (float)finger + phase);
	}
	return user;
}

TrackingFrameResult makeDirectFrame(const Camera& camera, const AvatarSkeleton& skeleton, int index, int lostFrom,
									int lostTo)
{
	User user= makeMovingUser(index * kFrameMs);
	if (index >= lostFrom && index < lostTo)
	{
		user.bWristSeen[0]= false;
		user.bHandTracked[0]= false;
	}
	return makeFrame(camera, user, skeleton, index * kFrameMs, index);
}
} // namespace

static int runRetargetStageTest(const TestArgs&)
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

	// (a) The dropout hold and its decay, under the position rules
	{
		HandPose live;
		live.tracked= true;
		live.presence= 0.9f;
		live.confidence= 0.8f;
		live.palmPositionWorld= glm::vec3(0.1f, 0.2f, 0.3f);
		live.hasWorldPose= true;
		live.hasForearmPose= true;
		live.forearmConfidence= 0.6f;
		const HandPose lost; // untracked
		const LossPolicy hold= makePositionLossPolicy(0.f, 250.f, false);

		LossProbe probe;
		probe.step(live, 1000.0, hold);
		check(probe.carried() && probe.source.arms[0].state == eGroupState::Measured && probe.out().confidence == 0.8f &&
				  probe.out().forearmConfidence == 0.6f,
			  "(a) a live pose passes through and arms the hold");

		probe.step(lost, 1100.0, hold);
		check(probe.carried() && probe.held() &&
				  fabsf(probe.out().confidence - 0.8f * (1.f - 100.f / 250.f)) < 1e-4f &&
				  probe.out().palmPositionWorld == live.palmPositionWorld &&
				  fabsf(probe.source.hands[0].confidence.value - 0.8f * (1.f - 100.f / 250.f)) < 1e-4f,
			  "(a) a dropout at +100 ms holds the pose with its confidence decayed by 100/250");
		// The elbow confidence decays with the hand's: the position solve
		// weighs the measured elbow on it, so a held pose keeping its last
		// live value would read as freshly measured
		check(fabsf(probe.out().forearmConfidence - 0.6f * (1.f - 100.f / 250.f)) < 1e-4f &&
				  fabsf(probe.source.arms[0].forearmConfidence.value - 0.6f * (1.f - 100.f / 250.f)) < 1e-4f,
			  "(a) the forearm confidence decays with the hand's");

		probe.step(lost, 1250.0, hold);
		check(probe.held() && probe.out().confidence < 1e-4f, "(a) at +250 ms the last held frame, confidence near 0");

		probe.step(lost, 1251.0, hold);
		const bool bPastWindow= !probe.carried() && probe.source.arms[0].state == eGroupState::Lost;
		// Disarmed: a frame stamped back inside the old window holds nothing
		probe.step(lost, 1200.0, hold);
		check(bPastWindow && !probe.held(), "(a) past the window the hold is over, and disarmed");

		probe.step(live, 2000.0, hold);
		HandPose lowConfidence= live;
		lowConfidence.confidence= 0.1f;
		probe.step(lowConfidence, 2100.0, makePositionLossPolicy(0.5f, 250.f, false));
		check(probe.held() && probe.out().confidence > 0.f && probe.out().confidence < 0.8f,
			  "(a) a reacquired hand rearms the hold, and a low-confidence pose gates like a dropout");

		LossProbe probeOff;
		const LossPolicy noHold= makePositionLossPolicy(0.f, 0.f, false);
		probeOff.step(live, 3000.0, noHold);
		const bool bLive= probeOff.carried();
		probeOff.step(lost, 3016.0, noHold);
		check(bLive && !probeOff.carried(), "(a) a hold of zero reports the dropout at once");

		LossProbe probeRegress;
		probeRegress.step(live, 4000.0, hold);
		const bool bArmed= probeRegress.carried();
		probeRegress.step(lost, 3000.0, hold);
		check(bArmed && !probeRegress.carried(), "(a) a timestamp regression of a second drops the hold");
	}

	// (b) Freeze on loss: VMC has no confidence, so a hand that stops being
	// measured can only be told by an arm that stops moving
	{
		HandPose live;
		live.tracked= true;
		live.confidence= 0.9f;
		live.hasWorldPose= true;
		live.palmPositionWorld= glm::vec3(0.2f, 0.1f, 0.9f);
		live.hasShoulder= true;
		live.shoulderPositionWorld= glm::vec3(0.f, 0.15f, 1.3f);
		const HandPose lost;

		// Past the hold at once, so the freeze rule answers
		LossProbe probe;
		const LossPolicy freeze= makePositionLossPolicy(0.f, 0.f, true);
		probe.step(live, 1000.0, freeze);
		check(probe.carried() && probe.out().palmPositionWorld == live.palmPositionWorld,
			  "(b) a live pose streams unchanged");
		probe.step(lost, 1033.0, freeze);
		check(probe.carried() && probe.source.arms[0].state == eGroupState::Frozen &&
				  probe.source.hands[0].state == eGroupState::Frozen &&
				  probe.out().palmPositionWorld == live.palmPositionWorld,
			  "(b) a lost hand freezes at its last streamed pose");

		LossProbe probeOff;
		const LossPolicy release= makePositionLossPolicy(0.f, 0.f, false);
		probeOff.step(live, 1000.0, release);
		const bool bLive= probeOff.carried();
		probeOff.step(lost, 1033.0, release);
		check(bLive, "(b) freeze off still streams a live pose");
		check(!probeOff.carried() && probeOff.source.arms[0].state == eGroupState::Lost,
			  "(b) freeze off releases the arm on loss");

		// A camera-space pose has no world anchor to hang a skeleton off
		HandPose cameraOnly= live;
		cameraOnly.hasWorldPose= false;
		LossProbe probeCamera;
		probeCamera.step(cameraOnly, 1000.0, freeze);
		check(!probeCamera.carried(), "(b) a camera-space pose is not streamable as bones");
	}

	// (e) The step clock. Camera frames every 33 ms, each odd one followed by
	// a face-only frame 45 ms later that repeats it, so the next camera frame
	// steps 12 ms back; the hand lost from frame 10. The hold runs from the
	// last frame that saw the hand for its 250 ms of forward time: no
	// backward step ends it, no frame's interval is negative, and the step
	// clock never goes back while each frame keeps its own timestamp. Then a
	// backward step of exactly kMaxBackwardStepMs is still small, one a
	// millisecond longer is a discontinuity that ends the hold.
	{
		HandPose live;
		live.tracked= true;
		live.confidence= 0.8f;
		live.hasWorldPose= true;
		live.palmPositionWorld= glm::vec3(0.1f, 0.2f, 0.3f);
		const HandPose lost;
		const double holdMs= 250.0;
		const LossPolicy hold= makePositionLossPolicy(0.f, (float)holdMs, false);

		LossProbe probe;
		int frames= 0;
		int backwardSteps= 0;
		int clockWrong= 0;
		int held= 0;
		int heldPastHold= 0;
		double holdStartMs= -1.0;
		double lastHeldElapsedMs= -1.0;
		double firstReleasedElapsedMs= -1.0;
		double lastStepMs= -1.0;
		double lastTimestampMs= -1.0;
		double largestInterval= 0.0;
		for (int index= 0; index < 30; ++index)
		{
			const HandPose& pose= index < 10 ? live : lost;
			for (int pass= 0; pass < ((index % 2) == 1 ? 2 : 1); ++pass)
			{
				const double timestampMs= index * kFrameMs + (pass == 1 ? 45.0 : 0.0);
				probe.step(pose, timestampMs, hold);
				const SourcePose::Clock& clock= probe.source.clock;
				frames++;
				backwardSteps+= (lastTimestampMs >= 0.0 && timestampMs < lastTimestampMs) ? 1 : 0;
				if (clock.timestampMs != timestampMs || clock.frameIntervalMs < 0.0 ||
					(lastStepMs >= 0.0 && clock.stepMs != lastStepMs + clock.frameIntervalMs))
				{
					clockWrong++;
				}
				largestInterval= std::max(largestInterval, clock.frameIntervalMs);
				lastStepMs= clock.stepMs;
				lastTimestampMs= timestampMs;

				if (probe.source.arms[0].state == eGroupState::Measured)
				{
					holdStartMs= clock.stepMs;
				}
				else if (probe.held())
				{
					held++;
					lastHeldElapsedMs= clock.stepMs - holdStartMs;
					heldPastHold+= lastHeldElapsedMs > holdMs ? 1 : 0;
				}
				else if (firstReleasedElapsedMs < 0.0)
				{
					firstReleasedElapsedMs= clock.stepMs - holdStartMs;
				}
			}
		}
		MIKAN_LOG_INFO(kTag) << "  (e) " << frames << " frames, " << backwardSteps << " backward steps, clock wrong on "
							 << clockWrong << "; held on " << held << " frames, the last " << lastHeldElapsedMs
							 << " ms and the first released " << firstReleasedElapsedMs << " ms into the hold";
		check(backwardSteps > 0 && clockWrong == 0,
			  "(e) across interleaved face-only frames the step clock never goes back and no interval is negative");
		check(held > 0 && heldPastHold == 0 && lastHeldElapsedMs + largestInterval > holdMs &&
				  firstReleasedElapsedMs > holdMs,
			  "(e) a hold spanning interleaved face-only frames lasts its duration in forward time");

		LossProbe probeSmall;
		probeSmall.step(live, 6000.0, hold);
		probeSmall.step(lost, 6100.0, hold);
		probeSmall.step(lost, 6100.0 - SourcePose::Clock::kMaxBackwardStepMs, hold);
		const bool bSmallHeld= probeSmall.held() && probeSmall.source.clock.stepMs == 6100.0 &&
			probeSmall.source.clock.frameIntervalMs == 0.0 &&
			probeSmall.source.clock.timestampMs == 6100.0 - SourcePose::Clock::kMaxBackwardStepMs;
		probeSmall.step(lost, 6150.0, hold);
		const bool bSmallResumed= probeSmall.held() && probeSmall.source.clock.frameIntervalMs == 50.0 &&
			fabsf(probeSmall.out().confidence - 0.8f * (1.f - 150.f / 250.f)) < 1e-4f;
		check(bSmallHeld && bSmallResumed,
			  "(e) a backward step of kMaxBackwardStepMs passes no time and the hold carries on from where it was");

		LossProbe probeLarge;
		probeLarge.step(live, 7000.0, hold);
		probeLarge.step(lost, 7100.0, hold);
		const bool bLargeArmed= probeLarge.held();
		probeLarge.step(lost, 7100.0 - SourcePose::Clock::kMaxBackwardStepMs - 1.0, hold);
		check(bLargeArmed && !probeLarge.carried() && probeLarge.source.arms[0].state == eGroupState::Lost &&
				  probeLarge.source.clock.stepMs == probeLarge.source.clock.timestampMs &&
				  probeLarge.source.clock.frameIntervalMs < 0.0,
			  "(e) a longer backward step is a discontinuity: the step clock follows it and the hold ends");
	}

	const AvatarModel model= makeModel(eVrmVersion::Vrm1, false);
	const std::shared_ptr<const AvatarSkeleton> skeleton= std::make_shared<AvatarSkeleton>(model);
	BodyDimensions user;
	user.upperArmLengthMeters= 0.30f;
	user.forearmLengthMeters= 0.26f;
	user.shoulderWidthMeters= 0.36f;

	RetargetStageConfig positionConfig;
	positionConfig.skeleton= skeleton;
	positionConfig.user= user;
	positionConfig.positionLoss= makePositionLossPolicy(0.f, 250.f, true);

	const Camera camera= makeCamera(10.f);
	RetargetStageConfig directConfig;
	directConfig.skeleton= skeleton;
	directConfig.direct.cameraTiltRadians= glm::radians(10.f);
	directConfig.direct.userShoulderWidthMeters= kShoulderWidth;
	directConfig.direct.userUpperArmLengthMeters= kUpperArmLength;
	directConfig.direct.userForearmLengthMeters= kForearmLength;
	directConfig.directLoss= makeDirectLossPolicy(directConfig.direct.restBlendMs);

	// (c) The display stage beside the stream stage: the stream's stage runs
	// every output frame (camera and face-only) and feeds the streamer only
	// the frames its 60 Hz send rate passes, the display's runs the same
	// frames and sends nothing. Their poses and group states agree on every
	// frame, bit for bit.
	{
		RetargetStage streamStage;
		RetargetStage displayStage;
		streamStage.setConfig(positionConfig);
		displayStage.setConfig(positionConfig);
		OscStreamerConfig oscConfig;
		oscConfig.maxRateHz= 60.f;
		oscConfig.avatarSkeleton= skeleton;
		OscStreamer streamer;
		streamer.setConfig(oscConfig);
		RetargetResult streamResult;
		RetargetResult displayResult;
		std::vector<std::vector<uint8_t>> packets;
		int frames= 0;
		int differ= 0;
		int frozenFrames= 0;
		double lastSentMs= -1.0;
		for (int index= 0; index < 200; ++index)
		{
			TrackingFrameResult frame= makePositionFrame(*skeleton, index, 60, 120);
			for (int pass= 0; pass < ((index % 2) == 1 ? 2 : 1); ++pass)
			{
				if (pass == 1)
					frame.timestampMs+= 45.0;
				streamStage.solvePosition(frame, streamResult);
				if (lastSentMs < 0.0 || frame.timestampMs < lastSentMs || frame.timestampMs - lastSentMs >= 1000.0 / 60.0)
				{
					streamer.encodeFrame(frame, streamResult, packets);
					lastSentMs= frame.timestampMs;
				}
				displayStage.solvePosition(frame, displayResult);
				frames++;
				differ+= (samePoseBits(streamResult.pose, displayResult.pose) &&
						  sameStates(streamResult.source, displayResult.source))
					? 0
					: 1;
				frozenFrames+= displayResult.source.arms[0].state == eGroupState::Frozen ? 1 : 0;
			}
		}
		MIKAN_LOG_INFO(kTag) << "  (c) position: " << differ << " of " << frames << " frames differ, the left arm frozen on "
							 << frozenFrames;
		check(differ == 0 && frozenFrames > 0, "(c) the position display stage poses what the stream stage does");

		RetargetStage directStream;
		RetargetStage directDisplay;
		directStream.setConfig(directConfig);
		directDisplay.setConfig(directConfig);
		int directFrames= 0;
		int directDiffer= 0;
		int restingFrames= 0;
		const TrackingFrameResult::HeadPose noPhone;
		for (int index= 0; index < 160; ++index)
		{
			const TrackingFrameResult frame= makeDirectFrame(camera, *skeleton, index, 60, 110);
			directStream.solveDirect(frame, camera.model, noPhone, frame.timestampMs, streamResult);
			directDisplay.solveDirect(frame, camera.model, noPhone, frame.timestampMs, displayResult);
			directFrames++;
			directDiffer+= (samePoseBits(streamResult.pose, displayResult.pose) &&
							sameStates(streamResult.source, displayResult.source))
				? 0
				: 1;
			restingFrames+= displayResult.source.arms[0].state == eGroupState::Resting ? 1 : 0;
		}
		MIKAN_LOG_INFO(kTag) << "  (c) direct: " << directDiffer << " of " << directFrames
							 << " frames differ, the left arm resting on " << restingFrames;
		check(directDiffer == 0 && restingFrames > 0, "(c) the direct display stage poses what the stream stage does");
	}

	// (d) A scrub: the playhead jumps to a frame from far away, resets the
	// stage and pre-rolls the second before it, against a straight
	// play-through of every frame. The bound is the largest difference in a
	// root or bone delta component. A pre-roll is not a play-through: what
	// the state carries from before the second is gone. The position pipeline
	// lands within 0.005: the root follow (0.3 s first order) has all but
	// converged, and the most it keeps is after a freeze the pre-roll began
	// inside, which the play-through's root followed and the scrub's did not
	// see (0.0017 at frame 150). The direct pipeline lands within 0.08 (about
	// 9 degrees): its wrist depth carries seconds of history through the
	// heavy depth band, the chain's continuity terms and the hold its arm
	// earned, so the held chain a scrub lands in is the one a second of
	// tracking settled on (0.063 at frame 110, 0.015 with three seconds of
	// pre-roll, zero with five). A freeze that began before the pre-roll is
	// not recovered at all, so no target sits inside one.
	{
		struct Run
		{
			const char* name;
			bool bDirect;
			float bound;
			int frameCount;
			int lostFrom;
			int lostTo;
			std::vector<int> targets;
		};
		const Run runs[2]= {
			{"position", false, 0.005f, 220, 100, 140, {50, 110, 125, 150, 210}},
			{"direct", true, 0.08f, 220, 100, 140, {50, 110, 125, 150, 210}},
		};
		const TrackingFrameResult::HeadPose noPhone;
		for (const Run& run : runs)
		{
			auto solveFrame= [&](int frameIndex, RetargetStage& stage, RetargetResult& ioResult) {
				if (run.bDirect)
				{
					const TrackingFrameResult frame= makeDirectFrame(camera, *skeleton, frameIndex, run.lostFrom, run.lostTo);
					stage.solveDirect(frame, camera.model, noPhone, frame.timestampMs, ioResult);
				}
				else
				{
					stage.solvePosition(makePositionFrame(*skeleton, frameIndex, run.lostFrom, run.lostTo), ioResult);
				}
			};
			auto frameTimeMs= [](int frameIndex) { return frameIndex * kFrameMs; };

			// The play-through, every target's pose on the way
			RetargetStage straight;
			straight.setConfig(run.bDirect ? directConfig : positionConfig);
			RetargetResult straightResult;
			std::vector<AvatarPose> straightPoses(run.frameCount);
			std::vector<SourcePose> straightSources(run.frameCount);
			for (int index= 0; index < run.frameCount; ++index)
			{
				solveFrame(index, straight, straightResult);
				straightPoses[index]= straightResult.pose;
				straightSources[index]= straightResult.source;
			}

			// Each target reached by a scrub from the far end
			RetargetStage scrubbed;
			scrubbed.setConfig(run.bDirect ? directConfig : positionConfig);
			RetargetPlayhead playhead;
			RetargetResult scrubResult;
			float worst= 0.f;
			bool bAllPreRolled= true;
			bool bStatesSame= true;
			for (int target : run.targets)
			{
				playhead.seek(scrubbed, 0, run.frameCount - 1, frameTimeMs, solveFrame, scrubResult);
				playhead.seek(scrubbed, 0, target, frameTimeMs, solveFrame, scrubResult);
				bAllPreRolled&= playhead.getLastSolvedCount() == std::min(target + 1, 31);
				const float difference= poseDifference(scrubResult.pose, straightPoses[target]);
				bStatesSame&= sameStates(scrubResult.source, straightSources[target]);
				worst= difference < 0.f || worst < 0.f ? -1.f : std::max(worst, difference);
				MIKAN_LOG_INFO(kTag) << "  (d) " << run.name << " frame " << target << ": "
									 << playhead.getLastSolvedCount() << " frames pre-rolled, difference "
									 << difference;
			}
			check(bAllPreRolled, std::string("(d) ") + run.name + ": a scrub pre-rolls the second before its target");
			MIKAN_LOG_INFO(kTag) << "  (d) " << run.name << ": largest difference " << worst << " against a bound of "
								 << run.bound;
			check(bStatesSame && worst >= 0.f && worst <= run.bound,
				  std::string("(d) ") + run.name + ": a scrub lands within the bound of the play-through's pose");

			// A step forward plays every frame between, and lands exactly
			RetargetStage stepped;
			stepped.setConfig(run.bDirect ? directConfig : positionConfig);
			RetargetPlayhead stepHead;
			RetargetResult stepResult;
			bool bExact= true;
			for (int index= 0; index < run.frameCount; index+= 3)
			{
				stepHead.seek(stepped, 0, index, frameTimeMs, solveFrame, stepResult);
				bExact&= samePoseBits(stepResult.pose, straightPoses[index]);
			}
			check(bExact, std::string("(d) ") + run.name + ": forward steps land on the play-through's pose bit for bit");
		}
	}

	// (f) The elbow weight. The source table's names and its fill; the
	// default function equal to the rule the solve applied before, bit for
	// bit, over a sweep of confidences and thresholds; a plain value and
	// another function changing the weight the solve takes; a text that does
	// not compile falling back on the plain value; and a stateful function
	// keeping its state across frames and across an unchanged edit, and
	// losing it on a reset.
	{
		using namespace SourcePoseInputs;
		const ExprInputTable& inputs= SourcePoseInputs::table();
		const char* const expectedNames[]= {
			"source.head.confidence",
			"source.leftUpperArm.confidence",
			"source.rightUpperArm.confidence",
			"source.leftLowerArm.confidence",
			"source.rightLowerArm.confidence",
			"source.leftHand.confidence",
			"source.rightHand.confidence",
			"source.torso.measured",
			"source.torso.carried",
			"source.head.measured",
			"source.head.carried",
			"source.leftArm.measured",
			"source.leftArm.carried",
			"source.rightArm.measured",
			"source.rightArm.carried",
			"source.leftHand.measured",
			"source.leftHand.carried",
			"source.rightHand.measured",
			"source.rightHand.carried",
		};
		bool bNames= inputs.getCount() == k_count && k_count == (int)(sizeof(expectedNames) / sizeof(expectedNames[0]));
		for (int slot= 0; bNames && slot < k_count; ++slot)
			bNames&= inputs.getName(slot) == expectedNames[slot] && inputs.findSlot(expectedNames[slot]) == slot;
		bNames&= inputs.findSlot("SOURCE.LeftLowerArm.Confidence") == k_lowerArmConfidence &&
			inputs.findSlot("source.leftLowerArm") == -1;
		check(bNames, "(f) the source table holds its 19 names in slot order, found without case");

		SourcePose source;
		source.head.state= eGroupState::Held;
		source.head.confidence= {true, 0.4f};
		source.torso.state= eGroupState::Measured;
		source.arms[0].state= eGroupState::Measured;
		source.arms[0].shoulderConfidence= {true, 0.8f};
		source.arms[0].forearmConfidence= {true, 0.6f};
		source.arms[1].state= eGroupState::Frozen;
		source.arms[1].shoulderConfidence= {false, 0.7f};
		source.arms[1].forearmConfidence= {true, 0.2f};
		source.hands[0].state= eGroupState::Resting;
		source.hands[0].confidence= {true, 0.9f};
		source.hands[1].state= eGroupState::Lost;
		source.hands[1].confidence= {false, 0.5f};
		float values[k_count];
		fill(source, values);
		const float expectedValues[k_count]= {0.4f, 0.8f, 0.f, 0.6f, 0.2f, 0.9f, 0.f, 1.f, 1.f, 0.f, 1.f,
											  1.f,  1.f,  0.f, 1.f, 0.f,  1.f,  0.f, 0.f};
		check(std::memcmp(values, expectedValues, sizeof(values)) == 0,
			  "(f) the fill: confidences as present, an absent one 0, measured only when measured, carried unless lost");

		// The sweep: the runner over the default function against the rule
		const float thresholds[]= {0.5f, 0.3f, 0.7f, 0.123456789f, 1.f / 3.f, 1e-5f, 0.999999f, 1.f, 2.5f, 0.f, -0.5f};
		int compared= 0;
		int differing= 0;
		for (float threshold : thresholds)
		{
			ExprParameterRunner runner;
			runner.setParameter(makeDefaultElbowWeight(0, threshold), inputs);
			for (int step= 0; step <= 1000; ++step)
			{
				SourcePose probe;
				probe.arms[0].forearmConfidence= {true, (float)step / 1000.f};
				if (step == 1000)
					probe.arms[0].forearmConfidence.value= 1e-7f;
				fill(probe, values);
				const float weight= runner.evaluate(values, k_count, step * 0.01, 0.f, 1.f);
				const float expected= elbowWeightFromConfidence(probe.arms[0].forearmConfidence.value, threshold);
				compared++;
				differing+= std::memcmp(&weight, &expected, sizeof(float)) == 0 ? 0 : 1;
			}
		}
		MIKAN_LOG_INFO(kTag) << "  (f) the default function against the rule: " << differing << " of " << compared
							 << " weights differ";
		check(differing == 0, "(f) the default function equals the rule bit for bit over the sweep");

		// Through the stage: the same position frames under four settings
		auto runWeights= [&](const ExprParameter& left, int frameCount, std::vector<RetargetElbowWeight>& outWeights,
							 std::vector<AvatarPose>& outPoses) {
			RetargetStageConfig config= positionConfig;
			config.elbowWeight[0]= left;
			RetargetStage stage;
			stage.setConfig(config);
			RetargetResult result;
			for (int index= 0; index < frameCount; ++index)
			{
				stage.solvePosition(makePositionFrame(*skeleton, index, 1000, 1000), result);
				outWeights.push_back(result.elbowWeights[0]);
				outPoses.push_back(result.pose);
			}
		};
		std::vector<RetargetElbowWeight> defaultWeights, plainWeights, constantWeights, brokenWeights;
		std::vector<AvatarPose> defaultPoses, plainPoses, constantPoses, brokenPoses;
		runWeights(makeDefaultElbowWeight(0, k_defaultElbowHintConfidence), 20, defaultWeights, defaultPoses);
		runWeights(ExprParameter{0.f}, 20, plainWeights, plainPoses);
		runWeights(ExprParameter{0.3f, "2 * 0.125 + 1", true}, 20, constantWeights, constantPoses);
		runWeights(ExprParameter{0.3f, "clamp(source.leftLowerArm.confidence", true}, 20, brokenWeights, brokenPoses);
		bool bDefault= true;
		bool bPlain= true;
		bool bConstant= true;
		bool bBroken= true;
		bool bPosesMoved= false;
		for (int index= 0; index < 20; ++index)
		{
			const float confidence= makeMovingHand(*skeleton, eHandSide::Left, index * kFrameMs).forearmConfidence;
			const RetargetElbowWeight& d= defaultWeights[index];
			bDefault&= d.bEvaluated && d.bExpression && d.bCompiled &&
				d.weight == elbowWeightFromConfidence(confidence, k_defaultElbowHintConfidence) &&
				d.rawValue == d.weight; // the function clamps itself
			const RetargetElbowWeight& p= plainWeights[index];
			bPlain&= p.bEvaluated && !p.bExpression && p.weight == 0.f && p.rawValue == 0.f;
			const RetargetElbowWeight& c= constantWeights[index];
			bConstant&= c.bExpression && c.bCompiled && c.rawValue == 1.25f && c.weight == 1.f;
			const RetargetElbowWeight& b= brokenWeights[index];
			bBroken&= b.bExpression && !b.bCompiled && b.weight == 0.3f;
			bPosesMoved|= !samePoseBits(defaultPoses[index], plainPoses[index]) && d.weight > 0.f;
		}
		check(bDefault, "(f) the stage evaluates the default function on every frame over the source after the loss stage");
		check(bPlain && bPosesMoved, "(f) a plain weight of 0 is what the solve takes, and the pose follows the hint");
		check(bConstant, "(f) another function decides the weight, its raw value clamped to [0, 1]");
		check(bBroken && samePoseBits(brokenPoses[5], [&] {
				  std::vector<RetargetElbowWeight> weights;
				  std::vector<AvatarPose> poses;
				  runWeights(ExprParameter{0.3f}, 6, weights, poses);
				  return poses[5];
			  }()),
			  "(f) a text that does not compile falls back on the plain value, as if no function were set");

		// A stateful function: smoothing lags the confidence; an unchanged
		// edit keeps the state, a reset starts it over from the input
		const ExprParameter smoothed{0.f, "smooth(source.leftLowerArm.confidence, 0.5)", true};
		RetargetStageConfig smoothConfig= positionConfig;
		smoothConfig.elbowWeight[0]= smoothed;
		RetargetStage straight;
		RetargetStage edited;
		straight.setConfig(smoothConfig);
		edited.setConfig(smoothConfig);
		RetargetResult straightResult;
		RetargetResult editedResult;
		bool bKept= true;
		bool bLagged= false;
		for (int index= 0; index < 40; ++index)
		{
			const TrackingFrameResult frame= makePositionFrame(*skeleton, index, 1000, 1000);
			if (index == 20)
			{
				// The same text through an edit of another field
				RetargetStageConfig reEdited= smoothConfig;
				reEdited.elbowWeight[0].value= 0.9f;
				edited.setConfig(reEdited);
			}
			straight.solvePosition(frame, straightResult);
			edited.solvePosition(frame, editedResult);
			bKept&= std::memcmp(&straightResult.elbowWeights[0].weight, &editedResult.elbowWeights[0].weight,
								sizeof(float)) == 0;
			bLagged|= index > 0 && straightResult.elbowWeights[0].weight != frame.poses[0].forearmConfidence;
		}
		check(bKept && bLagged, "(f) a stateful function keeps its state across frames and across an unchanged edit");
		edited.reset();
		const TrackingFrameResult next= makePositionFrame(*skeleton, 40, 1000, 1000);
		straight.solvePosition(next, straightResult);
		edited.solvePosition(next, editedResult);
		check(editedResult.elbowWeights[0].weight == next.poses[0].forearmConfidence &&
				  straightResult.elbowWeights[0].weight != next.poses[0].forearmConfidence,
			  "(f) a reset starts the stateful function over from its input");
	}

	MIKAN_LOG_INFO(kTag) << (failures == 0 ? "ALL PASSED" : "FAILED") << " (" << failures << " failures)";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-retarget-stage",
					"Retarget stage: the loss stage's hold and freeze, the display stage against the stream stage, "
					"a scrub's pre-roll against a straight play-through, the elbow weight over the source inputs",
					eTestCategory::SelfTest, runRetargetStageTest)
