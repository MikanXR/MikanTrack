#pragma once

#include <cstring>
#include <string>
#include <vector>

#include "AvatarRetarget.h"
#include "DirectRetarget.h"
#include "OscStreamer.h"

#include "FrozenAvatarRetarget.h"
#include "FrozenDirectRetarget.h"
#include "FrozenOscStreamer.h"

// The retarget golden comparison: the live retarget path run beside the frozen
// copy of it (namespace Frozen) on the same input, compared bit for bit. The
// live side is reached only through the pairs below, so a restructure of the
// live code edits these adapters and the conversions, never the sequences.
// The conversions say what the frozen code would have been configured with to
// mean the same thing as the live config.

namespace RetargetGolden
{
// Bitwise equality of one padding-free value (a float, a bool, a glm vector
// or quaternion, an array of them)
template <typename T>
bool sameBits(const T& a, const T& b)
{
	return std::memcmp(&a, &b, sizeof(T)) == 0;
}

#define GOLDEN_FIELD(field)            \
	if (!sameBits(a.field, b.field))   \
	{                                  \
		outWhy= prefix + #field;       \
		return false;                  \
	}

inline Frozen::AvatarRetargetConfig::Side toFrozen(const AvatarRetargetConfig::Side& side)
{
	Frozen::AvatarRetargetConfig::Side out;
	out.elbowHintOffset= side.elbowHintOffset;
	out.elbowHintConfidence= side.elbowHintConfidence;
	out.handTrimRadians= side.handTrimRadians;
	out.forearmRollTrimRadians= side.forearmRollTrimRadians;
	out.curlGain= side.curlGain;
	out.splayGain= side.splayGain;
	out.fingerEnabled= side.fingerEnabled;
	out.bThumbPronationOverride= side.bThumbPronationOverride;
	out.thumbPronationRad= side.thumbPronationRad;
	out.restElbowOffset= side.restElbowOffset;
	out.restWristOffset= side.restWristOffset;
	out.restPalmNormal= side.restPalmNormal;
	out.restFingerCurlRadians= side.restFingerCurlRadians;
	return out;
}

inline Frozen::AvatarRetargetConfig toFrozen(const AvatarRetargetConfig& config)
{
	Frozen::AvatarRetargetConfig out;
	out.followShoulders= config.followShoulders;
	out.fixedRootPositionWorld= config.fixedRootPositionWorld;
	out.fixedRootYawDegrees= config.fixedRootYawDegrees;
	out.rootFollowTimeConstantS= config.rootFollowTimeConstantS;
	out.sides[0]= toFrozen(config.sides[0]);
	out.sides[1]= toFrozen(config.sides[1]);
	out.headTrimRadians= config.headTrimRadians;
	return out;
}

inline Frozen::DirectCameraModel toFrozen(const DirectCameraModel& camera)
{
	Frozen::DirectCameraModel out;
	out.fx= camera.fx;
	out.fy= camera.fy;
	out.cx= camera.cx;
	out.cy= camera.cy;
	out.width= camera.width;
	out.height= camera.height;
	return out;
}

inline Frozen::DirectNeutralPose toFrozen(const DirectNeutralPose& neutral)
{
	Frozen::DirectNeutralPose out;
	out.present= neutral.present;
	out.torso= neutral.torso;
	out.shoulderMidpoint= neutral.shoulderMidpoint;
	out.head= neutral.head;
	return out;
}

inline Frozen::DirectRetargetConfig toFrozen(const DirectRetargetConfig& config)
{
	Frozen::DirectRetargetConfig out;
	out.cameraTiltRadians= config.cameraTiltRadians;
	out.neutral= toFrozen(config.neutral);
	out.userShoulderWidthMeters= config.userShoulderWidthMeters;
	out.headWidthMeters= config.headWidthMeters;
	out.noseForwardMeters= config.noseForwardMeters;
	out.userUpperArmLengthMeters= config.userUpperArmLengthMeters;
	out.userForearmLengthMeters= config.userForearmLengthMeters;
	out.headTrimRadians= config.headTrimRadians;
	out.sides[0]= toFrozen(config.sides[0]);
	out.sides[1]= toFrozen(config.sides[1]);
	out.restBlendMs= config.restBlendMs;
	return out;
}

inline Frozen::OscStreamerConfig toFrozen(const OscStreamerConfig& config)
{
	Frozen::OscStreamerConfig out;
	out.enabled= config.enabled;
	out.targetIp= config.targetIp;
	out.port= config.port;
	out.maxRateHz= config.maxRateHz;
	out.minConfidence= config.minConfidence;
	out.holdOnDropoutMs= config.holdOnDropoutMs;
	out.forearmLengthMeters= config.forearmLengthMeters;
	out.shoulderWidthMeters= config.shoulderWidthMeters;
	out.upperArmLengthMeters= config.upperArmLengthMeters;
	out.vmcHeadOffsetMeters= config.vmcHeadOffsetMeters;
	out.vmcFreezeOnLoss= config.vmcFreezeOnLoss;
	out.avatarSkeleton= config.avatarSkeleton;
	out.avatarRetarget= toFrozen(config.avatarRetarget);
	out.bodyDimensions= config.bodyDimensions;
	out.avatarPath= config.avatarPath;
	out.avatarTitle= config.avatarTitle;
	out.avatarSha256= config.avatarSha256;
	out.faceBinding= config.faceBinding;
	return out;
}

// A is the live type, B the frozen one. outWhy names the first field that
// differs.
template <typename A, typename B>
bool samePose(const A& a, const B& b, std::string& outWhy)
{
	const std::string prefix= "pose.";
	GOLDEN_FIELD(valid)
	GOLDEN_FIELD(timestampMs)
	GOLDEN_FIELD(rootPositionWorld)
	GOLDEN_FIELD(rootRotationWorld)
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		if (!sameBits(a.bones[index].present, b.bones[index].present) ||
			!sameBits(a.bones[index].deltaWorld, b.bones[index].deltaWorld))
		{
			outWhy= prefix + "bones[" + humanoidBoneName((eHumanoidBone)index) + "]";
			return false;
		}
	}
	return true;
}

template <typename A, typename B>
bool sameChain(const A& a, const B& b, const std::string& prefix, std::string& outWhy)
{
	GOLDEN_FIELD(valid)
	GOLDEN_FIELD(shoulder)
	GOLDEN_FIELD(elbow)
	GOLDEN_FIELD(wrist)
	return true;
}

template <typename A, typename B>
bool sameSolveArm(const A& a, const B& b, const std::string& prefix, std::string& outWhy)
{
	if (!sameChain(a.chain, b.chain, prefix + "chain.", outWhy))
		return false;
	GOLDEN_FIELD(bHandSeen)
	GOLDEN_FIELD(bElbowSeen)
	GOLDEN_FIELD(bDepthChosen)
	GOLDEN_FIELD(depthLow)
	GOLDEN_FIELD(depthHigh)
	GOLDEN_FIELD(modelDepthDelta)
	GOLDEN_FIELD(pnpDistance)
	GOLDEN_FIELD(liftDepth)
	GOLDEN_FIELD(chosenDepth)
	GOLDEN_FIELD(chosenCost)
	GOLDEN_FIELD(chosenElbowPx)
	GOLDEN_FIELD(filteredDepth)
	GOLDEN_FIELD(pnpScale.scale)
	GOLDEN_FIELD(pnpScale.sampleCount)
	GOLDEN_FIELD(bHandMeasured)
	GOLDEN_FIELD(twist)
	GOLDEN_FIELD(wristRotation)
	GOLDEN_FIELD(filteredPalm)
	GOLDEN_FIELD(measuredFingers)
	GOLDEN_FIELD(fingers)
	GOLDEN_FIELD(bResting)
	GOLDEN_FIELD(restBlend)
	return true;
}

template <typename A, typename B>
bool sameSolveState(const A& a, const B& b, std::string& outWhy)
{
	const std::string prefix= "state.";
	GOLDEN_FIELD(camera.fx)
	GOLDEN_FIELD(camera.fy)
	GOLDEN_FIELD(camera.cx)
	GOLDEN_FIELD(camera.cy)
	GOLDEN_FIELD(camera.width)
	GOLDEN_FIELD(camera.height)
	GOLDEN_FIELD(tiltRadians)
	GOLDEN_FIELD(torsoValid)
	GOLDEN_FIELD(shoulder)
	GOLDEN_FIELD(shoulderMidpoint)
	GOLDEN_FIELD(torsoFrame)
	GOLDEN_FIELD(spineDelta)
	GOLDEN_FIELD(headMeasured)
	GOLDEN_FIELD(headFrame)
	GOLDEN_FIELD(headDelta)
	return sameSolveArm(a.arms[0], b.arms[0], prefix + "arms[0].", outWhy) &&
		sameSolveArm(a.arms[1], b.arms[1], prefix + "arms[1].", outWhy);
}

template <typename A, typename B>
bool sameNeutral(const A& a, const B& b, std::string& outWhy)
{
	const std::string prefix= "neutral.";
	GOLDEN_FIELD(present)
	GOLDEN_FIELD(torso)
	GOLDEN_FIELD(shoulderMidpoint)
	GOLDEN_FIELD(head)
	return true;
}

#undef GOLDEN_FIELD

// /VMC/Ext/T carries the sender's wall clock, which two streamers read at
// two instants: its float is zeroed on both sides before the compare
inline void maskStreamClock(std::vector<std::vector<uint8_t>>& ioPackets)
{
	// The OSC address padded to four bytes, then the ",f" type tag padded
	static const uint8_t k_pattern[16]= {'/', 'V', 'M', 'C', '/', 'E', 'x', 't', '/', 'T', 0, 0, ',', 'f', 0, 0};
	for (std::vector<uint8_t>& packet : ioPackets)
	{
		for (size_t offset= 0; offset + sizeof(k_pattern) + 4 <= packet.size(); offset+= 4)
		{
			if (std::memcmp(packet.data() + offset, k_pattern, sizeof(k_pattern)) == 0)
				std::memset(packet.data() + offset + sizeof(k_pattern), 0, 4);
		}
	}
}

inline bool samePackets(const std::vector<std::vector<uint8_t>>& a, const std::vector<std::vector<uint8_t>>& b,
						std::string& outWhy)
{
	if (a.size() != b.size())
	{
		outWhy= "packet count " + std::to_string(a.size()) + " against " + std::to_string(b.size());
		return false;
	}
	for (size_t index= 0; index < a.size(); ++index)
	{
		if (a[index] != b[index])
		{
			size_t byte= 0;
			while (byte < a[index].size() && byte < b[index].size() && a[index][byte] == b[index][byte])
				++byte;
			outWhy= "packet " + std::to_string(index) + " differs at byte " + std::to_string(byte) + " of " +
				std::to_string(a[index].size()) + " against " + std::to_string(b[index].size());
			return false;
		}
	}
	return true;
}

// The VMC stream, live beside frozen. The avatar path is left empty by the
// callers, so the wall-clock-throttled /VMC/Ext/VRM never goes out.
struct StreamPair
{
	OscStreamer live;
	Frozen::OscStreamer frozen;
	std::vector<std::vector<uint8_t>> livePackets;
	std::vector<std::vector<uint8_t>> frozenPackets;

	void setConfig(const OscStreamerConfig& config)
	{
		live.setConfig(config);
		frozen.setConfig(toFrozen(config));
	}

	// The stereo path: the streamer's own gate, hold, freeze and retarget
	bool encode(const TrackingFrameResult& frame, std::string& outWhy)
	{
		live.encodeFrame(frame, livePackets);
		frozen.encodeFrame(frame, frozenPackets);
		return compare(outWhy);
	}

	// The direct path: each side streams the pose its own solver produced
	bool encodeDirect(const TrackingFrameResult& frame, const AvatarPose& livePose,
					  const Frozen::AvatarPose& frozenPose, std::string& outWhy)
	{
		live.encodeFrame(frame, livePackets, &livePose);
		frozen.encodeFrame(frame, frozenPackets, &frozenPose);
		return compare(outWhy);
	}

private:
	bool compare(std::string& outWhy)
	{
		maskStreamClock(livePackets);
		maskStreamClock(frozenPackets);
		return samePackets(livePackets, frozenPackets, outWhy);
	}
};

// The position retarget on an unresolved frame, which also compares what the
// stream does not carry: the root placement. Two live solvers run beside the
// frozen one: the facade, and the split it is made of (the source pose
// filled, then solved), so both entry points are held to the frozen bits.
struct PositionPair
{
	AvatarRetarget live;
	AvatarRetarget liveSplit;
	Frozen::AvatarRetarget frozen;
	AvatarPose livePose;
	AvatarPose liveSplitPose;
	SourcePose liveSource;
	Frozen::AvatarPose frozenPose;

	void reset()
	{
		live.reset();
		liveSplit.reset();
		frozen.reset();
	}

	bool solve(const TrackingFrameResult& frame, const BodyDimensions& user, const AvatarSkeleton& skeleton,
			   const AvatarRetargetConfig& config, std::string& outWhy)
	{
		live.solve(frame, user, skeleton, config, livePose);
		const bool bSideValid[2]= {
			frame.poses[0].tracked && frame.poses[0].hasWorldPose,
			frame.poses[1].tracked && frame.poses[1].hasWorldPose,
		};
		liveSplit.fillSource(frame.poses, bSideValid, frame.head, frame.timestampMs, user, liveSource);
		liveSplit.solve(liveSource, skeleton, config, liveSplitPose);
		frozen.solve(frame, user, skeleton, toFrozen(config), frozenPose);
		if (!samePose(livePose, frozenPose, outWhy))
			return false;
		if (!samePose(liveSplitPose, frozenPose, outWhy))
		{
			outWhy= "fill then solve: " + outWhy;
			return false;
		}
		return true;
	}
};

// The direct retarget: the pose, the solve state, the chains and the lift
// scale. Two live solvers run beside the frozen one: the facade, and the
// split it is made of (measure, then place).
struct DirectPair
{
	DirectRetarget live;
	DirectRetarget liveSplit;
	Frozen::DirectRetarget frozen;
	AvatarPose livePose;
	AvatarPose liveSplitPose;
	SourcePose liveSource;
	Frozen::AvatarPose frozenPose;

	void reset()
	{
		live.reset();
		liveSplit.reset();
		frozen.reset();
	}

	bool solve(const TrackingFrameResult& cameraFrame, const DirectCameraModel& cameraModel,
			   const TrackingFrameResult::HeadPose& phoneHead, double timestampMs, const AvatarSkeleton& skeleton,
			   const DirectRetargetConfig& config, std::string& outWhy)
	{
		live.solve(cameraFrame, cameraModel, phoneHead, timestampMs, skeleton, config, livePose);
		liveSplit.measure(cameraFrame, cameraModel, phoneHead, timestampMs, skeleton, config, liveSource);
		liveSplit.place(liveSource, skeleton, config, liveSplitPose);
		frozen.solve(cameraFrame, toFrozen(cameraModel), phoneHead, timestampMs, skeleton, toFrozen(config),
					 frozenPose);
		if (!sameAsFrozen(live, livePose, skeleton, outWhy))
			return false;
		if (!sameAsFrozen(liveSplit, liveSplitPose, skeleton, outWhy))
		{
			outWhy= "measure then place: " + outWhy;
			return false;
		}
		return true;
	}

	bool captureNeutral(DirectNeutralPose& outNeutral, std::string& outWhy)
	{
		Frozen::DirectNeutralPose frozenNeutral;
		DirectNeutralPose splitNeutral;
		const bool bLive= live.captureNeutral(outNeutral);
		const bool bSplit= liveSplit.captureNeutral(splitNeutral);
		const bool bFrozen= frozen.captureNeutral(frozenNeutral);
		if (bLive != bFrozen || bSplit != bFrozen)
		{
			outWhy= "captureNeutral result";
			return false;
		}
		if (!sameNeutral(splitNeutral, frozenNeutral, outWhy))
		{
			outWhy= "measure then place: " + outWhy;
			return false;
		}
		return sameNeutral(outNeutral, frozenNeutral, outWhy);
	}

private:
	bool sameAsFrozen(const DirectRetarget& solver, const AvatarPose& pose, const AvatarSkeleton& skeleton,
					  std::string& outWhy) const
	{
		if (!samePose(pose, frozenPose, outWhy) || !sameSolveState(solver.getSolveState(), frozen.getSolveState(), outWhy))
			return false;
		const glm::mat4 cameraToWorld= DirectRetarget::makeCameraToWorld(skeleton, pose, solver.getSolveState());
		const glm::mat4 frozenCameraToWorld=
			Frozen::DirectRetarget::makeCameraToWorld(skeleton, frozenPose, frozen.getSolveState());
		if (!sameBits(cameraToWorld, frozenCameraToWorld))
		{
			outWhy= "makeCameraToWorld";
			return false;
		}
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const std::string prefix= "getArmChain(" + std::to_string(sideIndex) + ").";
			if (!sameChain(solver.getArmChain(sideIndex), frozen.getArmChain(sideIndex), prefix, outWhy))
				return false;
			if (!sameBits(solver.getPnpScale(sideIndex).scale, frozen.getPnpScale(sideIndex).scale) ||
				!sameBits(solver.getPnpScale(sideIndex).sampleCount, frozen.getPnpScale(sideIndex).sampleCount))
			{
				outWhy= "getPnpScale(" + std::to_string(sideIndex) + ")";
				return false;
			}
		}
		return true;
	}
};

// The streamer's send-rate decimation, for a run that encodes only the
// frames a live sendFrame would have sent
struct RateGate
{
	double lastSendTimestampMs= -1.0;

	bool pass(double timestampMs, float maxRateHz)
	{
		if (maxRateHz > 0.f && lastSendTimestampMs >= 0.0 && timestampMs >= lastSendTimestampMs)
		{
			if ((timestampMs - lastSendTimestampMs) < 1000.0 / (double)maxRateHz)
				return false;
		}
		lastSendTimestampMs= timestampMs;
		return true;
	}
};

// Mismatch bookkeeping for one sequence
struct Tally
{
	int frames= 0;
	int mismatches= 0;
	std::string firstMismatch;

	void note(bool bSame, int frameIndex, const char* what, const std::string& why)
	{
		if (bSame)
			return;
		if (mismatches == 0)
			firstMismatch= "frame " + std::to_string(frameIndex) + " " + what + ": " + why;
		++mismatches;
	}
};
} // namespace RetargetGolden
