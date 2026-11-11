#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "AvatarRetarget.h"
#include "DirectRetarget.h"
#include "DirectStageSolver.h"
#include "LossStage.h"
#include "OscStreamer.h"
#include "RetargetChains.h"
#include "RetargetStage.h"

#include "FrozenAvatarRetarget.h"
#include "FrozenDirectRetarget.h"
#include "FrozenOscStreamer.h"

// The retarget golden comparison: the live retarget path run beside the frozen
// copy of it (namespace Frozen) on the same input, compared bit for bit. The
// live side is reached only through the pairs below, so a restructure of the
// live code edits these adapters and the conversions, never the sequences.
// The conversions say what the frozen code would have been configured with to
// mean the same thing as the live config. The live path is the retarget
// stage (the source fill, the loss stage, the target solve) feeding the
// encode-only streamer; the frozen path is the streamer that gated, held,
// froze and retargeted on its own.

namespace RetargetGolden
{
// What a stream run is configured with, in the shape the frozen streamer
// took it: the stream's settings, the loss rules the live side moved into
// the retarget stage, and the position retarget's config and lengths
struct StreamConfig
{
	bool enabled= true;
	std::string targetIp= "127.0.0.1";
	uint16_t port= 39539;
	float maxRateHz= 60.f;
	float minConfidence= 0.f;
	float holdOnDropoutMs= 250.f;
	float forearmLengthMeters= 0.25f;
	float shoulderWidthMeters= 0.40f;
	float upperArmLengthMeters= 0.30f;
	float vmcHeadOffsetMeters= 0.08f;
	bool vmcFreezeOnLoss= true;
	std::shared_ptr<const AvatarSkeleton> avatarSkeleton;
	AvatarRetargetConfig avatarRetarget;
	// The threshold of each arm's elbow rule, which the frozen retarget took
	// in its config and the live stage runs as the default elbow weight
	// function the chains migrate it to
	std::array<float, 2> elbowHintConfidence{k_defaultElbowHintConfidence, k_defaultElbowHintConfidence};
	BodyDimensions bodyDimensions;
	std::string avatarPath;
	std::string avatarTitle;
	std::string avatarSha256;
	std::shared_ptr<const FaceProfileBinding> faceBinding;
};

// The live streamer's share of a stream config
inline OscStreamerConfig toLiveStream(const StreamConfig& config)
{
	OscStreamerConfig out;
	out.enabled= config.enabled;
	out.targetIp= config.targetIp;
	out.port= config.port;
	out.maxRateHz= config.maxRateHz;
	out.forearmLengthMeters= config.forearmLengthMeters;
	out.shoulderWidthMeters= config.shoulderWidthMeters;
	out.upperArmLengthMeters= config.upperArmLengthMeters;
	out.vmcHeadOffsetMeters= config.vmcHeadOffsetMeters;
	out.avatarSkeleton= config.avatarSkeleton;
	out.avatarPath= config.avatarPath;
	out.avatarTitle= config.avatarTitle;
	out.avatarSha256= config.avatarSha256;
	out.faceBinding= config.faceBinding;
	return out;
}

// The retarget chains a multi-camera project with these stream settings and
// these elbow thresholds runs on: what the live code reads its loss rules
// and elbow weights from, each elbow weight the default function of its
// threshold as the migration writes it
inline RetargetChains toLiveChains(const StreamConfig& config)
{
	return migrateRetargetChains(false, config.holdOnDropoutMs, config.vmcFreezeOnLoss, 300.f,
								 config.elbowHintConfidence);
}

// The live retarget stage's share of a stream config: the position
// pipeline's solve, its loss rules and its elbow weights, through the chains
inline RetargetStageConfig toLiveStage(const StreamConfig& config)
{
	const RetargetChains chains= toLiveChains(config);
	RetargetStageConfig out;
	out.skeleton= config.avatarSkeleton;
	out.position= config.avatarRetarget;
	out.user= config.bodyDimensions;
	out.positionLoss= makeChainLossPolicy(chains, SourcePose::ePipeline::Position, config.minConfidence);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		out.elbowWeight[sideIndex]= chains.arms[sideIndex].elbowWeight;
	return out;
}
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

// The live side has no threshold: it runs the elbow weight its threshold
// migrates to, so the frozen side takes the threshold beside the config
inline Frozen::AvatarRetargetConfig::Side toFrozen(const AvatarRetargetConfig::Side& side, float elbowHintConfidence)
{
	Frozen::AvatarRetargetConfig::Side out;
	out.elbowHintOffset= side.elbowHintOffset;
	out.elbowHintConfidence= elbowHintConfidence;
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

inline Frozen::AvatarRetargetConfig toFrozen(const AvatarRetargetConfig& config,
											  const std::array<float, 2>& elbowHintConfidence)
{
	Frozen::AvatarRetargetConfig out;
	out.followShoulders= config.followShoulders;
	out.fixedRootPositionWorld= config.fixedRootPositionWorld;
	out.fixedRootYawDegrees= config.fixedRootYawDegrees;
	out.rootFollowTimeConstantS= config.rootFollowTimeConstantS;
	out.sides[0]= toFrozen(config.sides[0], elbowHintConfidence[0]);
	out.sides[1]= toFrozen(config.sides[1], elbowHintConfidence[1]);
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
	// The direct solve never read the elbow threshold
	out.sides[0]= toFrozen(config.sides[0], k_defaultElbowHintConfidence);
	out.sides[1]= toFrozen(config.sides[1], k_defaultElbowHintConfidence);
	out.restBlendMs= config.restBlendMs;
	return out;
}

inline Frozen::OscStreamerConfig toFrozen(const StreamConfig& config)
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
	out.avatarRetarget= toFrozen(config.avatarRetarget, config.elbowHintConfidence);
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

// -- Within a tolerance, for a run that differs by design -------------------

// A stream that sends only some frames: the frozen streamer resolved and
// retargeted the sent frames alone, the live retarget stage runs on every
// frame ahead of the send-rate gate, so the hold's start, its decay and the
// frame it ends on, and the root follow's steps, differ. Every float the
// stream and the pose carry is held within this: a position in meters, or a
// rotation's largest quaternion component, q and -q being one rotation.
constexpr float kDecimatedTolerance= 0.05f;

// One OSC message read back from an encoded bundle
struct DecodedOscMessage
{
	std::string address;
	std::string tags;
	std::vector<int32_t> ints;
	std::vector<float> floats;
	std::vector<std::string> strings;
};

inline uint32_t readOscWord(const std::vector<uint8_t>& bytes, size_t offset)
{
	return ((uint32_t)bytes[offset] << 24) | ((uint32_t)bytes[offset + 1] << 16) | ((uint32_t)bytes[offset + 2] << 8) |
		(uint32_t)bytes[offset + 3];
}

// A null-terminated string padded to four bytes, ending before end
inline bool readOscString(const std::vector<uint8_t>& bytes, size_t& ioOffset, size_t end, std::string& outText)
{
	size_t terminator= ioOffset;
	while (terminator < end && bytes[terminator] != 0)
		++terminator;
	if (terminator >= end)
		return false;
	outText.assign((const char*)bytes.data() + ioOffset, terminator - ioOffset);
	ioOffset= (terminator + 4) & ~(size_t)3;
	return ioOffset <= end;
}

// The messages of one bundle, appended in order. False on anything that is
// not a well-formed bundle of messages with i, f and s arguments.
inline bool decodeOscBundle(const std::vector<uint8_t>& packet, std::vector<DecodedOscMessage>& ioMessages)
{
	static const char k_header[8]= {'#', 'b', 'u', 'n', 'd', 'l', 'e', 0};
	if (packet.size() < 16 || std::memcmp(packet.data(), k_header, 8) != 0)
		return false;
	size_t offset= 16;
	while (offset < packet.size())
	{
		if (offset + 4 > packet.size())
			return false;
		const size_t size= readOscWord(packet, offset);
		offset+= 4;
		const size_t end= offset + size;
		if (end > packet.size())
			return false;
		DecodedOscMessage message;
		if (!readOscString(packet, offset, end, message.address) || !readOscString(packet, offset, end, message.tags) ||
			message.tags.empty() || message.tags[0] != ',')
			return false;
		for (size_t tag= 1; tag < message.tags.size(); ++tag)
		{
			const char type= message.tags[tag];
			if (type == 's')
			{
				std::string text;
				if (!readOscString(packet, offset, end, text))
					return false;
				message.strings.push_back(text);
				continue;
			}
			if (offset + 4 > end || (type != 'i' && type != 'f'))
				return false;
			const uint32_t word= readOscWord(packet, offset);
			offset+= 4;
			if (type == 'i')
			{
				message.ints.push_back((int32_t)word);
			}
			else
			{
				float value;
				std::memcpy(&value, &word, 4);
				message.floats.push_back(value);
			}
		}
		offset= end;
		ioMessages.push_back(message);
	}
	return true;
}

// The largest component difference between two rotations, q and -q being
// the same rotation
inline float rotationDifference(const glm::quat& a, const glm::quat& b)
{
	float same= 0.f;
	float flipped= 0.f;
	for (int axis= 0; axis < 4; ++axis)
	{
		same= std::max(same, std::fabs(a[axis] - b[axis]));
		flipped= std::max(flipped, std::fabs(a[axis] + b[axis]));
	}
	return std::min(same, flipped);
}

// The two streams carry the same messages in the same order, with the same
// addresses, type tags, strings and ints, and every float within tolerance,
// a root or a bone's rotation (its last four floats) compared as a rotation.
// ioMaxDifference keeps the largest float difference seen.
inline bool samePacketsWithin(const std::vector<std::vector<uint8_t>>& a, const std::vector<std::vector<uint8_t>>& b,
							  float tolerance, std::string& outWhy, float& ioMaxDifference)
{
	std::vector<DecodedOscMessage> messagesA;
	std::vector<DecodedOscMessage> messagesB;
	for (const std::vector<uint8_t>& packet : a)
	{
		if (!decodeOscBundle(packet, messagesA))
		{
			outWhy= "a live packet does not decode";
			return false;
		}
	}
	for (const std::vector<uint8_t>& packet : b)
	{
		if (!decodeOscBundle(packet, messagesB))
		{
			outWhy= "a frozen packet does not decode";
			return false;
		}
	}
	if (messagesA.size() != messagesB.size())
	{
		outWhy= "message count " + std::to_string(messagesA.size()) + " against " + std::to_string(messagesB.size());
		return false;
	}
	bool bSame= true;
	for (size_t index= 0; index < messagesA.size(); ++index)
	{
		const DecodedOscMessage& messageA= messagesA[index];
		const DecodedOscMessage& messageB= messagesB[index];
		const std::string where= "message " + std::to_string(index) + " " + messageA.address;
		if (messageA.address != messageB.address || messageA.tags != messageB.tags ||
			messageA.strings != messageB.strings || messageA.ints != messageB.ints)
		{
			outWhy= where + " differs in its address, tags, strings or ints";
			return false;
		}
		// A transform message is a position and an xyzw rotation
		const bool bTransform= messageA.tags == ",sfffffff";
		const size_t positionFloats= bTransform ? 3 : messageA.floats.size();
		for (size_t value= 0; value <= positionFloats; ++value)
		{
			float difference= 0.f;
			if (value < positionFloats)
			{
				difference= std::fabs(messageA.floats[value] - messageB.floats[value]);
			}
			else if (bTransform)
			{
				const std::vector<float>& fa= messageA.floats;
				const std::vector<float>& fb= messageB.floats;
				difference= rotationDifference(glm::quat(fa[6], fa[3], fa[4], fa[5]), glm::quat(fb[6], fb[3], fb[4], fb[5]));
			}
			else
			{
				break;
			}
			if (!(difference <= tolerance))
			{
				if (bSame)
				{
					outWhy= where + (messageA.strings.empty() ? "" : " " + messageA.strings[0]) +
						(value < positionFloats ? " float " + std::to_string(value) : std::string(" rotation")) +
						" differs by " + std::to_string(difference);
				}
				bSame= false;
			}
			if (difference > ioMaxDifference || difference != difference)
				ioMaxDifference= difference;
		}
	}
	return bSame;
}

// The pose's root and bone deltas within tolerance, the rest exact
template <typename A, typename B>
bool samePoseWithin(const A& a, const B& b, float tolerance, std::string& outWhy, float& ioMaxDifference)
{
	if (a.valid != b.valid)
	{
		outWhy= "pose.valid";
		return false;
	}
	float difference= 0.f;
	std::string worst;
	auto widen= [&difference, &worst](float value, const std::string& what) {
		if (!(value <= difference))
		{
			difference= value;
			worst= what;
		}
	};
	for (int axis= 0; axis < 3; ++axis)
		widen(std::fabs(a.rootPositionWorld[axis] - b.rootPositionWorld[axis]), "rootPositionWorld");
	widen(rotationDifference(a.rootRotationWorld, b.rootRotationWorld), "rootRotationWorld");
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		const std::string bone= std::string("bones[") + humanoidBoneName((eHumanoidBone)index) + "]";
		if (a.bones[index].present != b.bones[index].present)
		{
			outWhy= "pose." + bone + ".present";
			return false;
		}
		widen(rotationDifference(a.bones[index].deltaWorld, b.bones[index].deltaWorld), bone);
	}
	if (!(difference <= ioMaxDifference))
		ioMaxDifference= difference;
	if (!(difference <= tolerance))
	{
		outWhy= "pose." + worst + " differs by " + std::to_string(difference);
		return false;
	}
	return true;
}

// -- The pairs ---------------------------------------------------------------

// The live stream: the retarget stage over every output frame, ahead of the
// send-rate gate as on the vision thread, and the encode-only streamer over
// what the stage made of a sent frame
struct LiveStream
{
	RetargetStage stage;
	OscStreamer streamer;
	RetargetResult result;

	void setConfig(const StreamConfig& config)
	{
		stage.setConfig(toLiveStage(config));
		streamer.setConfig(toLiveStream(config));
	}

	void retarget(const TrackingFrameResult& frame) { stage.solvePosition(frame, result); }

	void encode(const TrackingFrameResult& frame, std::vector<std::vector<uint8_t>>& outPackets)
	{
		streamer.encodeFrame(frame, result, outPackets);
	}
};

// What the frozen streamer resolves and retargets inside its encode, rebuilt
// from its public pieces so the live stage's pose can be compared with it
// (the stream's bytes carry the root only as identity): the gate and the
// hold, the freeze, and the frozen retarget over the resolved poses. It
// advances on the frames the frozen streamer encodes, as that streamer does.
struct FrozenResolve
{
	StreamConfig config;
	Frozen::OscStreamer::HeldPoseState held[2];
	Frozen::OscStreamer::HeldPoseState last[2];
	Frozen::AvatarRetarget retarget;
	Frozen::AvatarPose pose;

	void setConfig(const StreamConfig& newConfig)
	{
		// The frozen streamer resets its retarget on a new skeleton
		if (newConfig.avatarSkeleton != config.avatarSkeleton)
			retarget.reset();
		config= newConfig;
	}

	void step(const TrackingFrameResult& frame)
	{
		std::array<HandPose, 2> streamed;
		bool bSideValid[2]= {false, false};
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			HandPose resolved;
			const bool bSent= Frozen::OscStreamer::resolveOutputPose(frame.poses[sideIndex], frame.timestampMs,
																	 config.minConfidence, config.holdOnDropoutMs,
																	 held[sideIndex], resolved);
			bSideValid[sideIndex]= Frozen::OscStreamer::resolveVmcOutputPose(resolved, bSent, config.vmcFreezeOnLoss,
																			last[sideIndex], streamed[sideIndex]);
		}
		pose= Frozen::AvatarPose();
		if (config.avatarSkeleton != nullptr)
		{
			retarget.solve(streamed, bSideValid, frame.head, frame.timestampMs, config.bodyDimensions,
						   *config.avatarSkeleton, toFrozen(config.avatarRetarget, config.elbowHintConfidence), pose);
		}
	}
};

// The VMC stream, live beside frozen. The avatar path is left empty by the
// callers, so the wall-clock-throttled /VMC/Ext/VRM never goes out. A
// position run calls retarget on every output frame and encode on each sent
// one; with no decimation that is every frame. A nonzero tolerance compares
// the floats within it instead of the bits, for a decimated run.
//
// bCarryOverStructureOnly: on a decimated run a hold or a freeze carries the
// last measured frame, which on the live side may be a frame the frozen
// streamer never resolved, so its values differ by whatever the tracking did
// between the two frames. A sent frame on which the live stage carries an
// arm over (held or frozen) is then compared for its structure alone (the
// bones, the addresses, the strings, the ints), and its difference is kept
// apart.
struct StreamPair
{
	LiveStream live;
	Frozen::OscStreamer frozen;
	FrozenResolve frozenResolve;
	std::vector<std::vector<uint8_t>> livePackets;
	std::vector<std::vector<uint8_t>> frozenPackets;
	float tolerance= 0.f;
	bool bCarryOverStructureOnly= false;
	// Under a tolerance: the largest difference in the pose and in the
	// stream's floats on the frames held to it, the sent frames that are not
	// bit-identical, and the frames compared for their structure alone with
	// the largest difference among them
	float maxPoseDifference= 0.f;
	float maxStreamDifference= 0.f;
	int inexactFrames= 0;
	int carryOverFrames= 0;
	float maxCarryOverDifference= 0.f;

	void setConfig(const StreamConfig& config)
	{
		live.setConfig(config);
		frozen.setConfig(toFrozen(config));
		frozenResolve.setConfig(config);
	}

	// The live retarget stage on an output frame, sent or not
	void retarget(const TrackingFrameResult& frame) { live.retarget(frame); }

	// A sent frame, after retarget: the live stream of what the stage made of
	// it beside the frozen streamer's own resolve, the packets and, with an
	// avatar, the pose
	bool encode(const TrackingFrameResult& frame, std::string& outWhy)
	{
		live.encode(frame, livePackets);
		frozen.encodeFrame(frame, frozenPackets);
		frozenResolve.step(frame);
		if (tolerance > 0.f)
		{
			std::string exactWhy;
			std::vector<std::vector<uint8_t>> liveMasked= livePackets;
			std::vector<std::vector<uint8_t>> frozenMasked= frozenPackets;
			maskStreamClock(liveMasked);
			maskStreamClock(frozenMasked);
			const bool bExactPose= frozenResolve.config.avatarSkeleton == nullptr ||
				samePose(live.result.pose, frozenResolve.pose, exactWhy);
			if (!bExactPose || !samePackets(liveMasked, frozenMasked, exactWhy))
				inexactFrames++;
		}
		const SourcePose& source= live.result.source;
		const bool bCarryOver= tolerance > 0.f && bCarryOverStructureOnly &&
			(source.arms[0].state == SourcePose::eGroupState::Held ||
			 source.arms[0].state == SourcePose::eGroupState::Frozen ||
			 source.arms[1].state == SourcePose::eGroupState::Held ||
			 source.arms[1].state == SourcePose::eGroupState::Frozen);
		const float frameTolerance= bCarryOver ? std::numeric_limits<float>::infinity() : tolerance;
		float poseDifference= 0.f;
		float streamDifference= 0.f;
		bool bSame= true;
		if (frozenResolve.config.avatarSkeleton != nullptr)
		{
			bSame= tolerance > 0.f
				? samePoseWithin(live.result.pose, frozenResolve.pose, frameTolerance, outWhy, poseDifference)
				: samePose(live.result.pose, frozenResolve.pose, outWhy);
		}
		if (bSame)
		{
			maskStreamClock(livePackets);
			maskStreamClock(frozenPackets);
			bSame= tolerance > 0.f
				? samePacketsWithin(livePackets, frozenPackets, frameTolerance, outWhy, streamDifference)
				: samePackets(livePackets, frozenPackets, outWhy);
		}
		if (bCarryOver)
		{
			carryOverFrames++;
			maxCarryOverDifference= std::max(maxCarryOverDifference, std::max(poseDifference, streamDifference));
		}
		else
		{
			maxPoseDifference= std::max(maxPoseDifference, poseDifference);
			maxStreamDifference= std::max(maxStreamDifference, streamDifference);
		}
		return bSame;
	}

	// The direct path: each side streams the pose its own solver produced
	bool encodeDirect(const TrackingFrameResult& frame, const AvatarPose& livePose,
					  const Frozen::AvatarPose& frozenPose, std::string& outWhy)
	{
		RetargetResult liveResult;
		liveResult.pose= livePose;
		live.streamer.encodeFrame(frame, liveResult, livePackets);
		frozen.encodeFrame(frame, frozenPackets, &frozenPose);
		maskStreamClock(livePackets);
		maskStreamClock(frozenPackets);
		return samePackets(livePackets, frozenPackets, outWhy);
	}
};

// The position retarget on an unresolved frame, which also compares what the
// stream does not carry: the root placement. Two live solvers run beside the
// frozen one: the facade, which applies the elbow rule to its threshold, and
// the split it is made of (the source pose filled, each arm's elbow weight
// evaluated through the default function the threshold migrates to, then
// solved), so both entry points are held to the frozen bits.
struct PositionPair
{
	AvatarRetarget live;
	AvatarRetarget liveSplit;
	ElbowWeightEvaluator liveSplitElbow;
	Frozen::AvatarRetarget frozen;
	AvatarPose livePose;
	AvatarPose liveSplitPose;
	SourcePose liveSource;
	std::array<RetargetElbowWeight, 2> liveSplitElbowReadout{};
	Frozen::AvatarPose frozenPose;

	void reset()
	{
		live.reset();
		liveSplit.reset();
		liveSplitElbow.reset();
		frozen.reset();
	}

	bool solve(const TrackingFrameResult& frame, const BodyDimensions& user, const AvatarSkeleton& skeleton,
			   const AvatarRetargetConfig& config, const std::array<float, 2>& elbowHintConfidence,
			   std::string& outWhy)
	{
		live.solve(frame, user, skeleton, config, livePose, elbowHintConfidence);
		const bool bSideValid[2]= {
			frame.poses[0].tracked && frame.poses[0].hasWorldPose,
			frame.poses[1].tracked && frame.poses[1].hasWorldPose,
		};
		liveSplit.fillSource(frame.poses, bSideValid, frame.head, frame.timestampMs, user, liveSource);
		liveSplitElbow.setWeights({makeDefaultElbowWeight(0, elbowHintConfidence[0]),
								   makeDefaultElbowWeight(1, elbowHintConfidence[1])});
		std::array<float, 2> elbowWeights;
		liveSplitElbow.evaluate(liveSource, elbowWeights, liveSplitElbowReadout);
		liveSplit.solve(liveSource, skeleton, config, elbowWeights, liveSplitPose);
		frozen.solve(frame, user, skeleton, toFrozen(config, elbowHintConfidence), frozenPose);
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
// scale. Two live paths run beside the frozen solver: the retarget stage,
// and the pieces it is made of run by hand (measure, the loss stage under the
// direct rules, place, and the rest readout written into the solve state).
struct DirectPair
{
	DirectStageSolver live;
	DirectRetarget liveSplit;
	LossStage liveSplitLoss;
	Frozen::DirectRetarget frozen;
	AvatarPose livePose;
	AvatarPose liveSplitPose;
	SourcePose liveSource;
	DirectRetarget::SolveState liveSplitState;
	Frozen::AvatarPose frozenPose;

	void reset()
	{
		live.reset();
		liveSplit.reset();
		liveSplitLoss.reset();
		frozen.reset();
	}

	bool solve(const TrackingFrameResult& cameraFrame, const DirectCameraModel& cameraModel,
			   const TrackingFrameResult::HeadPose& phoneHead, double timestampMs, const AvatarSkeleton& skeleton,
			   const DirectRetargetConfig& config, std::string& outWhy)
	{
		live.solve(cameraFrame, cameraModel, phoneHead, timestampMs, skeleton, config, livePose);
		liveSplit.measure(cameraFrame, cameraModel, phoneHead, timestampMs, skeleton, config, liveSource);
		// The direct rules built straight from the config, where the stage
		// builds them from the chains, so the two are held to each other too
		liveSplitLoss.apply(makeDirectLossPolicy(config.restBlendMs), &skeleton, config.sides, liveSource, nullptr);
		liveSplit.place(liveSource, skeleton, config, liveSplitPose);
		liveSplitState= liveSplit.getSolveState();
		RetargetStage::writeDirectRestReadout(liveSource, liveSplitState);
		frozen.solve(cameraFrame, toFrozen(cameraModel), phoneHead, timestampMs, skeleton, toFrozen(config),
					 frozenPose);
		if (!sameAsFrozen(live.getStage().getDirectRetarget(), live.getSolveState(), livePose, skeleton, outWhy))
			return false;
		if (!sameAsFrozen(liveSplit, liveSplitState, liveSplitPose, skeleton, outWhy))
		{
			outWhy= "measure, loss, place: " + outWhy;
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
			outWhy= "measure, loss, place: " + outWhy;
			return false;
		}
		return sameNeutral(outNeutral, frozenNeutral, outWhy);
	}

private:
	// solver: the measuring solver, for its chains and its lift scale
	bool sameAsFrozen(const DirectRetarget& solver, const DirectRetarget::SolveState& state, const AvatarPose& pose,
					  const AvatarSkeleton& skeleton, std::string& outWhy) const
	{
		if (!samePose(pose, frozenPose, outWhy) || !sameSolveState(state, frozen.getSolveState(), outWhy))
			return false;
		const glm::mat4 cameraToWorld= DirectRetarget::makeCameraToWorld(skeleton, pose, state);
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
