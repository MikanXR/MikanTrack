#include "TestCommon.h"

#include "glm/matrix.hpp"

#include "FaceAutoProfile.h"
#include "FaceProfileRunner.h"
#include "AvatarRetarget.h"
#include "AvatarSkeleton.h"
#include "AvatarTypes.h"
#include "FaceHead.h"
#include "OscStreamer.h"
#include "SyntheticAvatar.h"

// Synthetic tests for the VMC retarget. The whole conversion is pure, so every
// case here is exact: a hand at the rest pose must emit identity rotations, and
// composing the emitted bones the way a VMC receiver does must rebuild the
// measured skeleton.

namespace
{
using namespace VmcRetarget;

constexpr float kShoulderWidth= 0.30f;
constexpr float kUpperArm= 0.32f;
constexpr float kForearm= 0.26f;
constexpr float kHalfPalm= 0.045f;

VmcBodyLengths makeLengths()
{
	VmcBodyLengths lengths;
	lengths.shoulderWidthMeters= kShoulderWidth;
	lengths.upperArmLengthMeters= kUpperArm;
	lengths.forearmLengthMeters= kForearm;
	lengths.headOffsetMeters= 0.08f;
	return lengths;
}

// A plausible metric hand: the four fingers along palm +X, the thumb off to
// the side, with the flat-hand neutral directions the app defaults to.
// The thumb side is MIRRORED relative to a real hand (a real right hand has
// its index base at negative palm Y); the retarget is side-agnostic given
// the rest palm frame, so every assertion here holds either way.
HandSkeleton makeSkeleton(eHandSide side)
{
	HandSkeleton skeleton;
	const float thumbSign= side == eHandSide::Right ? 1.f : -1.f;

	skeleton.baseInPalm[(int)eFinger::Thumb]= glm::vec3(-0.01f, thumbSign * 0.025f, 0.005f);
	skeleton.baseInPalm[(int)eFinger::Index]= glm::vec3(kHalfPalm, thumbSign * 0.020f, 0.f);
	skeleton.baseInPalm[(int)eFinger::Middle]= glm::vec3(kHalfPalm, 0.f, 0.f);
	skeleton.baseInPalm[(int)eFinger::Ring]= glm::vec3(kHalfPalm, thumbSign * -0.019f, 0.f);
	skeleton.baseInPalm[(int)eFinger::Pinky]= glm::vec3(kHalfPalm - 0.005f, thumbSign * -0.038f, 0.f);

	const float lengths[FINGER_COUNT][3]= {
		{0.035f, 0.032f, 0.026f}, // thumb
		{0.040f, 0.024f, 0.018f}, // index
		{0.044f, 0.027f, 0.020f}, // middle
		{0.041f, 0.025f, 0.019f}, // ring
		{0.032f, 0.019f, 0.017f}, // pinky
	};
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
		for (int phalanx= 0; phalanx < 3; ++phalanx)
			skeleton.phalanxLengths[finger][phalanx]= lengths[finger][phalanx];

	skeleton.neutralDirInPalm= HandPoseModel::makeDefaultNeutralDirections(skeleton);
	return skeleton;
}

// A hand whose palm sits at `palmRotation` about `palmPosition`, with the
// forearm and shoulder filled in so the whole arm chain is exercised
HandPose makePose(eHandSide side, const glm::quat& palmRotation, const glm::vec3& palmPosition,
				  const glm::quat& forearmRotation, const glm::vec3& shoulderPosition)
{
	HandPose pose;
	pose.tracked= true;
	pose.side= side;
	pose.presence= 1.f;
	pose.confidence= 1.f;
	pose.skeleton= makeSkeleton(side);

	pose.hasWorldPose= true;
	pose.palmPositionWorld= palmPosition;
	pose.palmOrientationWorld= palmRotation;

	pose.hasForearmPose= true;
	pose.forearmOrientationWorld= forearmRotation;
	pose.forearmConfidence= 1.f;

	pose.hasShoulder= true;
	pose.shoulderPositionWorld= shoulderPosition;
	pose.shoulderConfidence= 1.f;

	return pose;
}

// The pose a side rests in: arm straight out to the side, palm down, fingers
// flat. This is the reference the whole retarget is measured against, so it
// must emit nothing but identity rotations.
HandPose makeRestPose(eHandSide side, const glm::vec3& chestOrigin, const VmcBodyLengths& lengths)
{
	const glm::vec3 armAxis= restArmDirection(side);
	const glm::quat restPalm= glm::quat_cast(restPalmFrame(side));

	const glm::vec3 shoulder= chestOrigin + armAxis * (lengths.shoulderWidthMeters * 0.5f);
	const glm::vec3 elbow= shoulder + armAxis * lengths.upperArmLengthMeters;
	const glm::vec3 wrist= elbow + armAxis * lengths.forearmLengthMeters;
	// The palm origin is half a palm PAST the wrist along the palm's +X, which
	// at rest points down the arm
	const glm::vec3 palm= wrist + armAxis * kHalfPalm;

	return makePose(side, restPalm, palm, restPalm, shoulder);
}

// Composes the emitted bones the way a VMC receiver does: each bone's local
// transform applied against its parent's, with an absent bone left at the
// avatar's rest pose (identity rotation, and a position this test does not
// need since it only walks streamed chains).
struct BoneWorld
{
	glm::quat rotation{1.f, 0.f, 0.f, 0.f};
	glm::vec3 position{0.f};
};

// Unity -> avatar frame, so the composed chain can be compared against the
// measured world geometry it came from. Inverse of the emitted conversion.
glm::vec3 unityToWorldPosition(const glm::vec3& unity)
{
	return glm::vec3(unity.z, -unity.x, unity.y);
}
glm::quat unityToWorldRotation(const glm::quat& unity)
{
	const glm::mat3 toWorld(
		glm::vec3(0.f, -1.f, 0.f), glm::vec3(0.f, 0.f, 1.f), glm::vec3(1.f, 0.f, 0.f));
	return glm::quat_cast(toWorld * glm::mat3_cast(unity) * glm::transpose(toWorld));
}

BoneWorld composeBone(const VmcPose& pose, eVmcBone bone, const BoneWorld& parent)
{
	const VmcBone& streamed= pose.bones[(int)bone];
	BoneWorld out;
	if (!streamed.present)
	{
		// Unstreamed bones sit at the avatar's rest pose, whose local rotation
		// is the identity - which is exactly why identity is the reference
		out= parent;
		return out;
	}

	out.rotation= parent.rotation * unityToWorldRotation(streamed.localRotation);
	out.position= parent.position + parent.rotation * unityToWorldPosition(streamed.localPosition);
	return out;
}

bool nearlyEqual(const glm::vec3& a, const glm::vec3& b, float tolerance)
{
	return glm::length(a - b) <= tolerance;
}

bool isIdentity(const glm::quat& q, float tolerance)
{
	return glm::angle(glm::normalize(q.w < 0.f ? -q : q)) <= tolerance;
}

// Angle between two directions, radians
float angleBetween(const glm::vec3& a, const glm::vec3& b)
{
	const float cosAngle= std::clamp(glm::dot(glm::normalize(a), glm::normalize(b)), -1.f, 1.f);
	return std::acos(cosAngle);
}

eVmcBone armBone(eHandSide side, int index) // 0 clavicle, 1 upper, 2 lower, 3 hand
{
	const int base= side == eHandSide::Left ? (int)eVmcBone::LeftShoulder : (int)eVmcBone::RightShoulder;
	return (eVmcBone)(base + index);
}

// -- Minimal OSC 1.0 bundle reader ------------------------------------------
// Deliberately written against the spec rather than against OscWriter, so it
// reads the bundle the way a receiver does: a decoder built from the encoder
// would agree with it however wrong both were.
struct DecodedMessage
{
	std::string address;
	std::string tags; // without the leading comma
	std::vector<float> floats;
	std::vector<int32_t> ints;
	std::vector<std::string> strings;
};

uint32_t readBigEndian32(const uint8_t* data)
{
	return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) |
		   (uint32_t)data[3];
}

// Reads a null-terminated string padded to a 4-byte boundary
bool readPaddedString(const std::vector<uint8_t>& buffer, size_t& offset, std::string& outText)
{
	const size_t start= offset;
	while (offset < buffer.size() && buffer[offset] != 0)
		offset++;
	if (offset >= buffer.size())
		return false;

	outText.assign((const char*)buffer.data() + start, offset - start);
	offset= start + ((offset - start) / 4 + 1) * 4;
	return offset <= buffer.size();
}

bool decodeMessage(const std::vector<uint8_t>& buffer, size_t offset, size_t size, DecodedMessage& out)
{
	const size_t end= offset + size;
	if (end > buffer.size() || !readPaddedString(buffer, offset, out.address))
		return false;

	std::string tagText;
	if (!readPaddedString(buffer, offset, tagText) || tagText.empty() || tagText[0] != ',')
		return false;
	out.tags= tagText.substr(1);

	for (char tag : out.tags)
	{
		if (tag == 'f' || tag == 'i')
		{
			if (offset + 4 > end)
				return false;
			const uint32_t raw= readBigEndian32(buffer.data() + offset);
			if (tag == 'f')
			{
				float value= 0.f;
				std::memcpy(&value, &raw, sizeof(value));
				out.floats.push_back(value);
			}
			else
			{
				out.ints.push_back((int32_t)raw);
			}
			offset+= 4;
		}
		else if (tag == 's')
		{
			std::string text;
			if (!readPaddedString(buffer, offset, text))
				return false;
			out.strings.push_back(text);
		}
		else
		{
			return false;
		}
	}
	return offset == end;
}

bool decodeBundle(const std::vector<uint8_t>& buffer, std::vector<DecodedMessage>& outMessages)
{
	if (buffer.size() < 16 || std::memcmp(buffer.data(), "#bundle\0", 8) != 0)
		return false;

	size_t offset= 16; // "#bundle\0" + the 8-byte time tag
	while (offset + 4 <= buffer.size())
	{
		const size_t size= readBigEndian32(buffer.data() + offset);
		offset+= 4;

		DecodedMessage message;
		if (!decodeMessage(buffer, offset, size, message))
			return false;
		outMessages.push_back(message);
		offset+= size;
	}
	return offset == buffer.size();
}

const DecodedMessage* findMessage(const std::vector<DecodedMessage>& messages, const char* address)
{
	for (const DecodedMessage& message : messages)
	{
		if (message.address == address)
			return &message;
	}
	return nullptr;
}
} // namespace

static int runVmcTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO("test-vmc") << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR("test-vmc") << "FAIL " << name;
			failures++;
		}
	};

	const VmcBodyLengths lengths= makeLengths();
	const bool bBothValid[2]= {true, true};
	TrackingFrameResult::HeadPose noHead;

	// (a) The basis change. World is right-handed (+X forward, +Y the person's
	// left, +Z up); Unity is left-handed (+X right, +Y up, +Z forward). This is
	// the one place a handedness error can hide, so pin both halves down.
	{
		check(nearlyEqual(worldToUnityPosition(glm::vec3(1.f, 2.f, 3.f)), glm::vec3(-2.f, 3.f, 1.f), 1e-6f),
			  "world position maps onto the Unity axes");

		// Yaw to the person's left is a POSITIVE turn about world +Z and a
		// NEGATIVE one about Unity +Y: the flip reverses the rotation sense
		const glm::quat worldYaw= glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.f, 0.f, 1.f));
		const glm::quat unityYaw= worldToUnityRotation(worldYaw);
		const glm::vec3 turnedForward= unityYaw * glm::vec3(0.f, 0.f, 1.f);
		check(nearlyEqual(turnedForward, glm::vec3(-1.f, 0.f, 0.f), 1e-5f),
			  "a left turn in world reads as a left turn in Unity");

		// A rotation must survive as a rotation: conjugating by a determinant
		// -1 basis change cancels, so no mirrored quaternion comes out
		const glm::mat3 rebuilt= glm::mat3_cast(unityYaw);
		check(fabsf(glm::determinant(rebuilt) - 1.f) < 1e-5f, "the converted rotation stays proper");

		// Round trip through the test's own inverse, which every later case
		// leans on
		const glm::quat tilted= glm::angleAxis(0.7f, glm::normalize(glm::vec3(0.3f, -0.5f, 0.8f)));
		check(isIdentity(glm::inverse(tilted) * unityToWorldRotation(worldToUnityRotation(tilted)), 1e-4f),
			  "the Unity conversion round trips");
	}

	// (b) The rest palm frame, stated rather than derived. Everything else here
	// uses restPalmFrame on both the producing and the checking side, so a
	// mirrored or swapped pair would cancel out and go unnoticed - this is the
	// only place the avatar's rest hand is pinned to an anatomical fact.
	{
		bool bFramesValid= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const eHandSide side= (eHandSide)sideIndex;
			const glm::mat3 frame= restPalmFrame(side);
			const glm::vec3 fingers= frame[0]; // palm +X
			const glm::vec3 thumbSide= frame[1]; // palm +Y
			const glm::vec3 palmar= frame[2]; // palm +Z, out of the palmar surface

			// Arms out to the sides, so the fingers run down the arm
			bFramesValid&= nearlyEqual(fingers, restArmDirection(side), 1e-6f);
			// Palms DOWN, which is the VRM rest pose
			bFramesValid&= nearlyEqual(palmar, glm::vec3(0.f, 0.f, -1.f), 1e-6f);
			// A left hand palm-down has its thumb forward, a right hand back.
			// This is the chirality, and it is the whole difference between the
			// two matrices.
			const float forward= side == eHandSide::Left ? 1.f : -1.f;
			bFramesValid&= nearlyEqual(thumbSide, glm::vec3(forward, 0.f, 0.f), 1e-6f);
			// Right-handed, not mirrored: a determinant of -1 here would flip
			// every finger silently
			bFramesValid&= fabsf(glm::determinant(frame) - 1.f) < 1e-6f;
		}
		check(bFramesValid, "the rest palm frames are palms-down T-pose hands of the right chirality");
	}

	// (c) Rest identity. A body in the avatar's rest pose must emit nothing but
	// identity rotations - that is what makes the streamed values deltas from
	// the avatar's own T-pose rather than from an assumed anatomy.
	{
		const glm::vec3 chest(0.f, 0.f, 1.35f);
		std::array<HandPose, 2> poses= {makeRestPose(eHandSide::Left, chest, lengths),
										makeRestPose(eHandSide::Right, chest, lengths)};

		VmcPose vmc;
		buildPose(poses, bBothValid, noHead, lengths, vmc);

		int emitted= 0;
		int nonIdentity= 0;
		for (int boneIndex= 0; boneIndex < VMC_BONE_COUNT; ++boneIndex)
		{
			const VmcBone& bone= vmc.bones[boneIndex];
			if (!bone.present)
				continue;
			emitted++;
			if (!isIdentity(bone.localRotation, 1e-3f))
				nonIdentity++;
		}
		// 8 arm bones + 30 finger bones; the head is absent (no head estimate)
		check(emitted == 38, "the rest pose emits every arm, hand and finger bone");
		check(nonIdentity == 0, "the rest pose emits only identity rotations");

		// Negative control: the finger rest directions come from the measured
		// skeleton, so bending the hand must break the identity
		poses[0].fingers[(int)eFinger::Index].proximal= 0.6f;
		buildPose(poses, bBothValid, noHead, lengths, vmc);
		check(!isIdentity(vmc.bones[(int)eVmcBone::LeftIndexProximal].localRotation, 1e-3f),
			  "a bent finger stops emitting identity");
		check(isIdentity(vmc.bones[(int)eVmcBone::LeftMiddleProximal].localRotation, 1e-3f),
			  "bending one finger leaves the others alone");
	}

	// (d) Chain round trip: compose the emitted bones the way the receiver does
	// and check the rebuilt skeleton against the geometry it was measured from.
	// This is what proves the retarget rather than just the encoder.
	{
		// Shoulders raised and rolled forward, mirrored about the chest: the
		// clavicle carries the DIRECTION and takes its length from the measured
		// shoulder width, so the composed joint lands on the measured one
		// exactly when the pair is symmetric at that separation - which is the
		// definition the width was measured under.
		const glm::vec3 chest(0.f, 0.f, 1.35f);
		const glm::vec3 clavicleDir= glm::normalize(glm::vec3(0.10f, 1.f, 0.20f));
		const glm::vec3 shoulderL= chest + clavicleDir * (kShoulderWidth * 0.5f);
		const glm::vec3 shoulderR= chest - clavicleDir * (kShoulderWidth * 0.5f);

		// A left arm reaching forward and down with a rolled forearm, i.e. a
		// pose whose every joint differs from the rest reference
		const glm::quat forearmRotation=
			glm::angleAxis(0.9f, glm::vec3(0.f, 0.f, 1.f)) *
			glm::angleAxis(-0.4f, glm::vec3(0.f, 1.f, 0.f)) *
			glm::angleAxis(0.8f, glm::vec3(1.f, 0.f, 0.f));
		const glm::quat palmRotation= forearmRotation * glm::angleAxis(0.35f, glm::vec3(0.f, 1.f, 0.f));

		// Place the wrist so the forearm bone is exactly kForearm long, which
		// is what the streamed offsets assert
		const glm::vec3 elbow= shoulderL + glm::normalize(glm::vec3(0.35f, 0.5f, -0.6f)) * kUpperArm;
		const glm::vec3 wrist= elbow + forearmRotation * glm::vec3(1.f, 0.f, 0.f) * kForearm;
		const glm::vec3 palm= wrist + palmRotation * glm::vec3(kHalfPalm, 0.f, 0.f);

		HandPose left= makePose(eHandSide::Left, palmRotation, palm, forearmRotation, shoulderL);
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			left.fingers[finger].lateral= 0.05f * (finger - 2);
			left.fingers[finger].proximal= 0.30f + 0.05f * finger;
			left.fingers[finger].intermediate= 0.45f;
			left.fingers[finger].distal= 0.25f;
		}

		// The right side is here only to fix the shoulder midpoint the left
		// clavicle is measured against; nothing below asserts on it
		HandPose right= makeRestPose(eHandSide::Right, chest, lengths);
		right.shoulderPositionWorld= shoulderR;

		std::array<HandPose, 2> poses= {left, right};
		VmcPose vmc;
		buildPose(poses, bBothValid, noHead, lengths, vmc);

		// Walk the chain from the chest, the bone this deliberately does not
		// stream (so the receiver holds it at rest, i.e. the identity)
		BoneWorld chestBone;
		chestBone.position= chest;

		const BoneWorld clavicle= composeBone(vmc, armBone(eHandSide::Left, 0), chestBone);
		const BoneWorld upperArm= composeBone(vmc, armBone(eHandSide::Left, 1), clavicle);
		const BoneWorld lowerArm= composeBone(vmc, armBone(eHandSide::Left, 2), upperArm);
		const BoneWorld hand= composeBone(vmc, armBone(eHandSide::Left, 3), lowerArm);

		// The clavicle points at the measured shoulder joint, so the upper arm
		// bone lands on it
		check(nearlyEqual(upperArm.position, shoulderL, 1e-4f),
			  "the composed chain reaches the measured shoulder");
		check(nearlyEqual(lowerArm.position, elbow, 1e-4f),
			  "the composed chain reaches the measured elbow");
		check(nearlyEqual(hand.position, wrist, 1e-4f),
			  "the composed chain reaches the measured wrist");

		// Directions, independently of the lengths: this is what a receiver
		// with its own proportions actually consumes
		const glm::vec3 restLeft= restArmDirection(eHandSide::Left);
		check(angleBetween(upperArm.rotation * restLeft, elbow - shoulderL) < 1e-4f,
			  "the upper arm points down the measured upper arm");
		check(angleBetween(lowerArm.rotation * restLeft, wrist - elbow) < 1e-4f,
			  "the forearm points down the measured forearm");

		// The hand carries the full palm frame, roll included, not just its
		// direction
		const glm::mat3 restPalm= restPalmFrame(eHandSide::Left);
		const glm::quat rebuiltPalm= hand.rotation * glm::quat_cast(restPalm);
		check(isIdentity(glm::inverse(palmRotation) * rebuiltPalm, 1e-4f),
			  "the hand rebuilds the measured palm orientation");

		// Fingers: rebuild every joint through the composed chain and compare
		// against the same forward kinematics the app draws and streams
		glm::mat4 palmTransform= glm::mat4_cast(palmRotation);
		palmTransform[3]= glm::vec4(palm, 1.f);
		std::array<std::array<glm::vec3, 4>, FINGER_COUNT> truthJoints;
		HandPoseModel::buildFingerJoints(palmTransform, left.skeleton, left.fingers, truthJoints);

		float worstJointError= 0.f;
		const int firstBone= (int)firstFingerBone(eHandSide::Left);
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			BoneWorld parent= hand;
			for (int phalanx= 0; phalanx < 3; ++phalanx)
			{
				const BoneWorld joint=
					composeBone(vmc, (eVmcBone)(firstBone + finger * 3 + phalanx), parent);
				worstJointError=
					std::max(worstJointError, glm::length(joint.position - truthJoints[finger][phalanx]));
				parent= joint;
			}
			// The tip is one more offset along the distal bone
			const glm::vec3 distalDir= parent.rotation * (restPalm * left.skeleton.neutralDirInPalm[finger]);
			const glm::vec3 tip= parent.position + distalDir * left.skeleton.phalanxLengths[finger][2];
			worstJointError= std::max(worstJointError, glm::length(tip - truthJoints[finger][3]));
		}
		MIKAN_LOG_INFO("test-vmc") << "worst rebuilt finger joint error " << worstJointError * 1000.f << " mm";
		check(worstJointError < 0.001f, "the composed fingers rebuild the forward-kinematics joints");
	}

	// (e) Degradation. A bone the receiver never hears about stays at the
	// avatar's REST pose - which for an arm is the T-pose, not a neutral. The
	// hand's world orientation survives either way (rest locals are identity),
	// but skipping the arm leaves the T-posed forearm under a correctly
	// oriented hand, so the arm's whole rotation surfaces as WRIST BEND. That
	// is what the elbow dropouts looked like in VSeeFace, so the arm chain is
	// always streamed and the wrist has to stay near neutral without an elbow.
	{
		const glm::quat palmRotation= glm::angleAxis(0.8f, glm::normalize(glm::vec3(0.2f, 0.4f, 0.9f)));
		HandPose left= makePose(eHandSide::Left, palmRotation, glm::vec3(0.3f, 0.2f, 0.9f),
								glm::quat(1.f, 0.f, 0.f, 0.f), glm::vec3(0.f));
		left.hasShoulder= false;
		left.hasForearmPose= false;

		std::array<HandPose, 2> poses= {left, HandPose()};
		const bool bLeftOnly[2]= {true, false};
		VmcPose vmc;
		buildPose(poses, bLeftOnly, noHead, lengths, vmc);

		check(vmc.bones[(int)eVmcBone::LeftUpperArm].present &&
				  vmc.bones[(int)eVmcBone::LeftLowerArm].present,
			  "an arm with no elbow still streams its chain");
		check(!vmc.bones[(int)eVmcBone::RightHand].present, "an invalid side streams nothing");
		check(vmc.bones[(int)eVmcBone::LeftHand].present, "the hand still streams without an arm");

		BoneWorld chestBone;
		const BoneWorld forearm= composeBone(vmc, eVmcBone::LeftLowerArm,
											 composeBone(vmc, eVmcBone::LeftUpperArm,
														 composeBone(vmc, eVmcBone::LeftShoulder,
																	 chestBone)));
		const BoneWorld hand= composeBone(vmc, eVmcBone::LeftHand, forearm);

		const glm::quat rebuiltPalm= hand.rotation * glm::quat_cast(restPalmFrame(eHandSide::Left));
		check(isIdentity(glm::inverse(palmRotation) * rebuiltPalm, 1e-4f),
			  "the hand orientation survives the missing arm");

		// The point of the change: with no elbow the forearm takes the hand's
		// own frame, so the wrist reads neutral instead of carrying the arm
		const glm::quat wristBend= glm::inverse(forearm.rotation) * hand.rotation;
		check(isIdentity(wristBend, 1e-4f),
			  "a missing elbow leaves the wrist neutral, not carrying the arm's rotation");
	}

	// (f) Head. Its measured frame is already the avatar frame, so the streamed
	// rotation must be the measured one with nothing but the basis change.
	{
		TrackingFrameResult::HeadPose head;
		head.valid= true;
		head.positionWorld= glm::vec3(0.1f, 0.f, 1.5f);
		head.orientationWorld= glm::angleAxis(0.5f, glm::vec3(0.f, 0.f, 1.f));
		head.confidence= 0.8f;

		std::array<HandPose, 2> poses;
		const bool bNone[2]= {false, false};
		VmcPose vmc;
		buildPose(poses, bNone, head, lengths, vmc);

		const VmcBone& headBone= vmc.bones[(int)eVmcBone::Head];
		check(headBone.present, "a valid head estimate streams the head bone");
		check(isIdentity(glm::inverse(head.orientationWorld) * unityToWorldRotation(headBone.localRotation),
						 1e-4f),
			  "the head rotation is the measured one");
		// The offset is the setting, carried up the avatar's own up axis
		check(nearlyEqual(headBone.localPosition, glm::vec3(0.f, lengths.headOffsetMeters, 0.f), 1e-6f),
			  "the head offset rides the avatar's up axis");

		head.valid= false;
		buildPose(poses, bNone, head, lengths, vmc);
		check(!vmc.bones[(int)eVmcBone::Head].present, "an invalid head streams nothing");
	}

	// (g) Freeze on loss. VMC has no confidence, so a hand that stops being
	// measured can only be expressed as an arm that stops moving.
	{
		HandPose live= makePose(eHandSide::Left, glm::quat(1.f, 0.f, 0.f, 0.f), glm::vec3(0.2f, 0.1f, 0.9f),
								glm::quat(1.f, 0.f, 0.f, 0.f), glm::vec3(0.f, 0.15f, 1.3f));
		HandPose lost;

		OscStreamer::HeldPoseState held;
		HandPose out;

		check(OscStreamer::resolveVmcOutputPose(live, true, true, held, out) &&
				  nearlyEqual(out.palmPositionWorld, live.palmPositionWorld, 1e-6f),
			  "a live pose streams unchanged");

		check(OscStreamer::resolveVmcOutputPose(lost, false, true, held, out) &&
				  nearlyEqual(out.palmPositionWorld, live.palmPositionWorld, 1e-6f),
			  "a lost hand freezes at its last streamed pose");

		OscStreamer::HeldPoseState heldOff;
		check(OscStreamer::resolveVmcOutputPose(live, true, false, heldOff, out),
			  "freeze off still streams a live pose");
		check(!OscStreamer::resolveVmcOutputPose(lost, false, false, heldOff, out),
			  "freeze off releases the arm on loss");

		// A camera-space pose has no world anchor to hang a skeleton off
		HandPose cameraOnly= live;
		cameraOnly.hasWorldPose= false;
		OscStreamer::HeldPoseState heldCamera;
		check(!OscStreamer::resolveVmcOutputPose(cameraOnly, true, true, heldCamera, out),
			  "a camera-space pose is not streamable as bones");
	}

	// (h) The wire itself. Everything above checks the retarget; this checks
	// what a receiver actually parses, decoded straight from the encoded bundle
	// against the OSC 1.0 spec. A wrong address or type tag is invisible from
	// inside - a VMC receiver is required to ignore both silently.
	{
		const glm::vec3 chest(0.f, 0.f, 1.35f);
		TrackingFrameResult frame;
		frame.frameIndex= 7;
		frame.timestampMs= 1000.0;
		frame.poses[0]= makeRestPose(eHandSide::Left, chest, lengths);
		frame.poses[1]= makeRestPose(eHandSide::Right, chest, lengths);
		frame.head.valid= true;
		frame.head.positionWorld= glm::vec3(0.f, 0.f, 1.6f);
		frame.head.orientationWorld= glm::angleAxis(0.3f, glm::vec3(0.f, 0.f, 1.f));
		frame.head.confidence= 0.9f;

		OscStreamerConfig config;
		config.maxRateHz= 0.f; // no decimation, so one call encodes one bundle
		config.shoulderWidthMeters= kShoulderWidth;
		config.upperArmLengthMeters= kUpperArm;
		config.forearmLengthMeters= kForearm;
		config.vmcHeadOffsetMeters= lengths.headOffsetMeters;

		OscStreamer streamer;
		streamer.setConfig(config);

		std::vector<std::vector<uint8_t>> packets;
		streamer.encodeFrame(frame, packets);

		// Every datagram must be a COMPLETE bundle in its own right - UDP does
		// not reassemble at the OSC layer - and must fit the size a receiver
		// will actually read
		size_t totalBytes= 0;
		size_t largestPacket= 0;
		bool bPacketsWellFormed= !packets.empty();
		std::vector<DecodedMessage> messages;
		for (const std::vector<uint8_t>& packet : packets)
		{
			bPacketsWellFormed&= packet.size() <= OscStreamer::k_maxDatagramBytes;
			bPacketsWellFormed&= decodeBundle(packet, messages);
			totalBytes+= packet.size();
			largestPacket= std::max(largestPacket, packet.size());
		}
		MIKAN_LOG_INFO("test-vmc") << "fully tracked VMC frame is " << packets.size() << " datagrams, "
								   << totalBytes << " bytes, largest " << largestPacket;
		check(bPacketsWellFormed, "every VMC datagram is a complete bundle within the size limit");

		const DecodedMessage* ok= findMessage(messages, "/VMC/Ext/OK");
		check(ok != nullptr && ok->tags == "iiii" && ok->ints.size() == 4 && ok->ints[0] == 1 &&
				  ok->ints[1] == 3 && ok->ints[3] == 1,
			  "/VMC/Ext/OK reports loaded, calibrated and tracking");

		const DecodedMessage* time= findMessage(messages, "/VMC/Ext/T");
		check(time != nullptr && time->tags == "f", "/VMC/Ext/T carries one float");

		const DecodedMessage* root= findMessage(messages, "/VMC/Ext/Root/Pos");
		check(root != nullptr && root->tags == "sfffffff" && root->strings.size() == 1 &&
				  root->strings[0] == "root" && root->floats.size() == 7 &&
				  root->floats[6] == 1.f,
			  "/VMC/Ext/Root/Pos is an identity root named \"root\"");

		// Every bone, exactly once, with the name and argument layout the
		// receiver matches on
		std::vector<std::string> boneNames;
		bool bBonesWellFormed= true;
		for (const DecodedMessage& message : messages)
		{
			if (message.address != "/VMC/Ext/Bone/Pos")
				continue;
			bBonesWellFormed&= message.tags == "sfffffff" && message.strings.size() == 1 &&
							   message.floats.size() == 7;
			if (!message.strings.empty())
				boneNames.push_back(message.strings[0]);
		}
		check(bBonesWellFormed, "every bone message is ,sfffffff");
		check(boneNames.size() == 39,
			  "a measured-length frame streams 39 bones: the head, 8 arm bones and 30 finger bones");

		std::sort(boneNames.begin(), boneNames.end());
		check(std::adjacent_find(boneNames.begin(), boneNames.end()) == boneNames.end(),
			  "no bone is streamed twice");

		bool bNamesKnown= true;
		for (const std::string& name : boneNames)
		{
			bool bFound= false;
			for (int boneIndex= 0; boneIndex < VMC_BONE_COUNT; ++boneIndex)
				bFound|= name == boneName((eVmcBone)boneIndex);
			bNamesKnown&= bFound;
		}
		check(bNamesKnown, "every streamed name is one this retarget knows");

		// The arguments carry the retarget's own values, in order
		VmcPose expected;
		buildPose(frame.poses, bBothValid, frame.head, lengths, expected);
		const VmcBone& expectedHand= expected.bones[(int)eVmcBone::LeftHand];

		bool bHandMatches= false;
		for (const DecodedMessage& message : messages)
		{
			if (message.address != "/VMC/Ext/Bone/Pos" || message.strings.empty() ||
				message.strings[0] != "LeftHand")
				continue;
			bHandMatches=
				nearlyEqual(glm::vec3(message.floats[0], message.floats[1], message.floats[2]),
							expectedHand.localPosition, 1e-6f) &&
				fabsf(message.floats[3] - expectedHand.localRotation.x) < 1e-6f &&
				fabsf(message.floats[4] - expectedHand.localRotation.y) < 1e-6f &&
				fabsf(message.floats[5] - expectedHand.localRotation.z) < 1e-6f &&
				fabsf(message.floats[6] - expectedHand.localRotation.w) < 1e-6f;
		}
		check(bHandMatches, "bone arguments are position xyz then rotation xyzw");
	}

	// (i) Bone names, which are the whole contract with the receiver: a typo
	// here is silently ignored on the far end
	{
		bool bNamesValid= true;
		for (int boneIndex= 0; boneIndex < VMC_BONE_COUNT; ++boneIndex)
		{
			const char* name= boneName((eVmcBone)boneIndex);
			bNamesValid&= name != nullptr && name[0] != '\0';
			for (int other= 0; other < boneIndex; ++other)
				bNamesValid&= strcmp(name, boneName((eVmcBone)other)) != 0;
		}
		check(bNamesValid, "every bone has a unique non-empty name");
		check(strcmp(boneName(eVmcBone::LeftLittleDistal), "LeftLittleDistal") == 0 &&
				  strcmp(boneName(eVmcBone::RightThumbProximal), "RightThumbProximal") == 0,
			  "finger bones use Unity's HumanBodyBones spelling");
	}

	// (i2) The torso and neck bones. They only go out when a pose places them,
	// which the direct retarget does and the position retarget never does.
	{
		check(strcmp(boneName(eVmcBone::Spine), "Spine") == 0 && strcmp(boneName(eVmcBone::Chest), "Chest") == 0 &&
				  strcmp(boneName(eVmcBone::UpperChest), "UpperChest") == 0 &&
				  strcmp(boneName(eVmcBone::Neck), "Neck") == 0,
			  "torso and neck bones use Unity's HumanBodyBones spelling");
		check(humanoidBoneForVmc(eVmcBone::Spine) == eHumanoidBone::Spine &&
				  humanoidBoneForVmc(eVmcBone::Chest) == eHumanoidBone::Chest &&
				  humanoidBoneForVmc(eVmcBone::UpperChest) == eHumanoidBone::UpperChest &&
				  humanoidBoneForVmc(eVmcBone::Neck) == eHumanoidBone::Neck,
			  "torso and neck VMC bones name the matching humanoid bones");

		// The synthetic rig has a spine, chest and neck but no upper chest
		const AvatarModel model= SyntheticAvatar::makeModel(eVrmVersion::Vrm1, false);
		OscStreamerConfig config;
		config.maxRateHz= 0.f;
		config.avatarSkeleton= std::make_shared<AvatarSkeleton>(model);
		OscStreamer streamer;
		streamer.setConfig(config);

		// A direct pose leaning 20 degrees about up, torso chain at one world
		// delta, with one arm placed at identity
		const glm::quat lean= glm::angleAxis(glm::radians(20.f), glm::vec3(0.f, 0.f, 1.f));
		AvatarPose pose;
		pose.valid= true;
		for (eHumanoidBone torsoBone : {eHumanoidBone::Spine, eHumanoidBone::Chest, eHumanoidBone::Neck})
		{
			pose.bones[(int)torsoBone].present= true;
			pose.bones[(int)torsoBone].deltaWorld= lean;
		}
		pose.bones[(int)eHumanoidBone::LeftUpperArm].present= true;

		auto decodeFrame= [&streamer](const AvatarPose* directPose, std::vector<DecodedMessage>& outMessages) {
			TrackingFrameResult emptyFrame;
			std::vector<std::vector<uint8_t>> packets;
			streamer.encodeFrame(emptyFrame, packets, directPose);
			outMessages.clear();
			bool bWellFormed= !packets.empty();
			for (const std::vector<uint8_t>& packet : packets)
				bWellFormed&= decodeBundle(packet, outMessages);
			return bWellFormed;
		};
		auto findBone= [](const std::vector<DecodedMessage>& messages, const char* name) -> const DecodedMessage* {
			for (const DecodedMessage& message : messages)
			{
				if (message.address == "/VMC/Ext/Bone/Pos" && !message.strings.empty() && message.strings[0] == name &&
					message.floats.size() == 7)
					return &message;
			}
			return nullptr;
		};

		std::vector<DecodedMessage> messages;
		check(decodeFrame(&pose, messages), "a direct pose frame is complete bundles");

		const DecodedMessage* spine= findBone(messages, "Spine");
		const DecodedMessage* neck= findBone(messages, "Neck");
		check(spine != nullptr && neck != nullptr && findBone(messages, "Chest") != nullptr,
			  "a direct pose streams the torso bones it placed");
		check(findBone(messages, "UpperChest") == nullptr, "an upper chest the pose never placed is not streamed");
		check(findBone(messages, "LeftUpperArm") != nullptr, "a direct pose streams the arm it placed");

		bool bFingerStreamed= false;
		for (const DecodedMessage& message : messages)
		{
			if (message.address != "/VMC/Ext/Bone/Pos" || message.strings.empty())
				continue;
			const std::string& name= message.strings[0];
			bFingerStreamed|= name.find("Thumb") != std::string::npos || name.find("Index") != std::string::npos ||
							  name.find("Middle") != std::string::npos || name.find("Ring") != std::string::npos ||
							  name.find("Little") != std::string::npos;
		}
		check(!bFingerStreamed, "a direct pose that placed no fingers streams none");

		// The spine hangs off the unplaced hips, so it carries the whole lean
		const glm::quat expectedSpine= worldToUnityRotation(lean);
		check(spine != nullptr &&
				  fabsf(fabsf(glm::dot(glm::quat(spine->floats[6], spine->floats[3], spine->floats[4], spine->floats[5]),
									   expectedSpine)) - 1.f) < 1e-4f,
			  "the spine carries the lean as its local rotation");
		// The arm hangs off the leaned chest, so it undoes the lean locally
		const DecodedMessage* upperArm= findBone(messages, "LeftUpperArm");
		const glm::quat expectedArm= worldToUnityRotation(glm::inverse(lean));
		check(upperArm != nullptr &&
				  fabsf(fabsf(glm::dot(glm::quat(upperArm->floats[6], upperArm->floats[3], upperArm->floats[4],
												 upperArm->floats[5]),
									   expectedArm)) - 1.f) < 1e-4f,
			  "an arm measured against the leaned chest streams relative to it");

		const DecodedMessage* ok= findMessage(messages, "/VMC/Ext/OK");
		check(ok != nullptr && ok->ints.size() == 4 && ok->ints[3] == 1, "a direct pose with an arm reports tracking");

		AvatarPose torsoOnly= pose;
		torsoOnly.bones[(int)eHumanoidBone::LeftUpperArm].present= false;
		decodeFrame(&torsoOnly, messages);
		ok= findMessage(messages, "/VMC/Ext/OK");
		check(ok != nullptr && ok->ints.size() == 4 && ok->ints[3] == 0, "a direct pose with no arm reports not tracking");

		// The position path never places the torso
		decodeFrame(nullptr, messages);
		check(findBone(messages, "Spine") == nullptr && findBone(messages, "Neck") == nullptr,
			  "with no direct pose the torso bones are not streamed");

		// No skeleton, no bones, but the rest of the frame still goes out
		OscStreamerConfig bareConfig;
		bareConfig.maxRateHz= 0.f;
		OscStreamer bareStreamer;
		bareStreamer.setConfig(bareConfig);
		TrackingFrameResult emptyFrame;
		std::vector<std::vector<uint8_t>> packets;
		bareStreamer.encodeFrame(emptyFrame, packets, &pose);
		messages.clear();
		for (const std::vector<uint8_t>& packet : packets)
			decodeBundle(packet, messages);
		bool bAnyBone= false;
		for (const DecodedMessage& message : messages)
			bAnyBone|= message.address == "/VMC/Ext/Bone/Pos";
		check(!bAnyBone && findMessage(messages, "/VMC/Ext/OK") != nullptr &&
				  findMessage(messages, "/VMC/Ext/T") != nullptr && findMessage(messages, "/VMC/Ext/Root/Pos") != nullptr,
			  "a direct pose with no avatar skeleton streams no bones but the rest of the frame");
	}

	// (j) The face stream and the avatar announcement on the wire. The names
	// are the contract here too: a receiver matches them exactly and ignores
	// the rest. The face reaches the wire through the Auto face profile.
	{
		auto arkit= [](const char* name) { return arkitBlendshapeFromName(name); };

		AvatarModel model;
		auto addExpression= [&model](const char* name, const char* preset) {
			AvatarExpression expression;
			expression.name= name;
			expression.preset= preset;
			model.expressions.push_back(expression);
		};
		addExpression("A", "aa");
		addExpression("U", "ou");
		addExpression("Blink", "blink");
		addExpression("Blink_L", "blinkLeft");
		addExpression("Blink_R", "blinkRight");
		addExpression("Joy", "happy");
		AvatarMesh mesh;
		mesh.morphTargetNames= {"jawOpen", "eyeBlinkLeft", "Custom"};
		model.meshes.push_back(mesh);

		FaceProfileRunner runner;
		runner.setBinding(makeAutoFaceBinding(&model));
		check(runner.getBinding() != nullptr, "the Auto profile of a preset avatar compiles");

		TrackingFrameResult frame;
		frame.timestampMs= 1000.0;
		frame.face.phone.present= true;
		frame.face.phone.tracked= true;
		frame.face.phone.blendshapes[arkit("jawOpen")]= 0.6f;
		frame.face.phone.blendshapes[arkit("eyeBlinkLeft")]= 0.8f;
		frame.face.phone.blendshapes[arkit("eyeBlinkRight")]= 0.2f;
		frame.face.phone.blendshapes[arkit("mouthSmileLeft")]= 0.4f;
		runner.evaluate(frame.face, frame.timestampMs);

		OscStreamerConfig config;
		config.maxRateHz= 0.f;
		config.avatarPath= "C:\\avatars\\test.vrm";
		config.avatarTitle= "Test";
		config.avatarSha256= std::string(64, 'a');
		config.faceBinding= runner.getBinding();

		OscStreamer streamer;
		streamer.setConfig(config);

		auto encodeOn= [](OscStreamer& targetStreamer, const TrackingFrameResult& sourceFrame,
						  std::vector<DecodedMessage>& outMessages) {
			std::vector<std::vector<uint8_t>> packets;
			targetStreamer.encodeFrame(sourceFrame, packets);
			outMessages.clear();
			bool bWellFormed= !packets.empty();
			for (const std::vector<uint8_t>& packet : packets)
			{
				bWellFormed&= packet.size() <= OscStreamer::k_maxDatagramBytes;
				bWellFormed&= decodeBundle(packet, outMessages);
			}
			return bWellFormed;
		};
		auto encode= [&encodeOn, &streamer](const TrackingFrameResult& sourceFrame,
											std::vector<DecodedMessage>& outMessages) {
			return encodeOn(streamer, sourceFrame, outMessages);
		};
		auto collectBlends= [](const std::vector<DecodedMessage>& messages) {
			std::vector<std::pair<std::string, float>> blends;
			for (const DecodedMessage& message : messages)
			{
				if (message.address == "/VMC/Ext/Blend/Val" && message.strings.size() == 1 && message.floats.size() == 1)
					blends.emplace_back(message.strings[0], message.floats[0]);
			}
			return blends;
		};
		auto blendValue= [](const std::vector<std::pair<std::string, float>>& blends, const char* name, float& outValue) {
			int count= 0;
			for (const auto& blend : blends)
			{
				if (blend.first == name)
				{
					outValue= blend.second;
					++count;
				}
			}
			return count == 1;
		};

		std::vector<DecodedMessage> messages;
		check(encode(frame, messages), "a frame with a face is complete bundles within the size limit");

		bool bBlendsWellFormed= true;
		size_t lastValueIndex= 0;
		size_t applyIndex= SIZE_MAX;
		for (size_t messageIndex= 0; messageIndex < messages.size(); ++messageIndex)
		{
			const DecodedMessage& message= messages[messageIndex];
			if (message.address == "/VMC/Ext/Blend/Val")
			{
				bBlendsWellFormed&= message.tags == "sf" && message.strings.size() == 1 && message.floats.size() == 1;
				lastValueIndex= messageIndex;
			}
			else if (message.address == "/VMC/Ext/Blend/Apply")
			{
				bBlendsWellFormed&= message.tags.empty();
				applyIndex= messageIndex;
			}
		}
		check(bBlendsWellFormed, "every blendshape is ,sf and Apply carries no arguments");
		check(applyIndex != SIZE_MAX && applyIndex > lastValueIndex, "Apply follows the last blendshape value");

		std::vector<std::pair<std::string, float>> blends= collectBlends(messages);
		float value= 0.f;
		check(blends.size() == 6, "the avatar's presets and ARKit morph names are streamed, nothing else");
		check(blendValue(blends, "A", value) && fabsf(value - 0.6f) < 1e-6f, "the aa preset follows the jaw");
		check(blendValue(blends, "Blink_L", value) && fabsf(value - 0.8f) < 1e-6f &&
				  blendValue(blends, "Blink_R", value) && fabsf(value - 0.2f) < 1e-6f,
			  "each eye's blink preset follows its own eye, in the avatar's spelling");
		check(!blendValue(blends, "Blink", value), "the two-eyed blink stays silent when each eye blinks on its own");
		check(!blendValue(blends, "Joy", value), "an emotion preset is never driven");
		check(blendValue(blends, "jawOpen", value) && fabsf(value - 0.6f) < 1e-6f &&
				  blendValue(blends, "eyeBlinkLeft", value),
			  "ARKit-named morph targets ride along under their own names");

		const DecodedMessage* avatar= findMessage(messages, "/VMC/Ext/VRM");
		check(avatar != nullptr && avatar->tags == "sss" && avatar->strings.size() == 3 &&
				  avatar->strings[0] == config.avatarPath && avatar->strings[1] == "Test" &&
				  avatar->strings[2] == config.avatarSha256,
			  "/VMC/Ext/VRM is ,sss path, title, sha256");

		encode(frame, messages);
		check(findMessage(messages, "/VMC/Ext/VRM") == nullptr, "/VMC/Ext/VRM goes out at most once a second");

		// The microphone alone: the vowel reads its viseme, as before the
		// profiles
		TrackingFrameResult micFrame;
		micFrame.timestampMs= 1010.0;
		micFrame.face.viseme.present= true;
		micFrame.face.viseme.weights[(int)eViseme::aa]= 0.9f;
		runner.evaluate(micFrame.face, micFrame.timestampMs);
		encode(micFrame, messages);
		blends= collectBlends(messages);
		check(blendValue(blends, "A", value) && fabsf(value - 0.9f) < 1e-6f &&
				  blendValue(blends, "jawOpen", value) && fabsf(value - 0.85f * 0.9f) < 1e-6f &&
				  blendValue(blends, "Blink_L", value) && value == 0.f,
			  "the microphone alone drives the vowel from its viseme and the ARKit morphs through the table");

		TrackingFrameResult puckerFrame;
		puckerFrame.timestampMs= 1012.0;
		puckerFrame.face.viseme.present= true;
		puckerFrame.face.viseme.weights[(int)eViseme::U]= 0.7f;
		runner.evaluate(puckerFrame.face, puckerFrame.timestampMs);
		encode(puckerFrame, messages);
		blends= collectBlends(messages);
		check(blendValue(blends, "U", value) && fabsf(value - 0.7f) < 1e-6f &&
				  blendValue(blends, "A", value) && value == 0.f,
			  "the microphone alone drives the ou preset from the U viseme");

		// aa and O both open the jaw: 0.85 + 0.5 on the shared column
		TrackingFrameResult sumFrame;
		sumFrame.timestampMs= 1014.0;
		sumFrame.face.viseme.present= true;
		sumFrame.face.viseme.weights[(int)eViseme::aa]= 1.f;
		sumFrame.face.viseme.weights[(int)eViseme::O]= 1.f;
		runner.evaluate(sumFrame.face, sumFrame.timestampMs);
		encode(sumFrame, messages);
		blends= collectBlends(messages);
		check(blendValue(blends, "jawOpen", value) && value == 1.f,
			  "viseme weights summing past one on a shared column arrive clamped to one");

		// A phone that is present but has lost the face reads zero
		TrackingFrameResult lostFace= frame;
		lostFace.timestampMs= 1020.0;
		lostFace.face.phone.tracked= false;
		runner.evaluate(lostFace.face, lostFace.timestampMs);
		encode(lostFace, messages);
		blends= collectBlends(messages);
		bool bLostZero= blends.size() == 6;
		for (const auto& blend : blends)
			bLostZero&= blend.second == 0.f;
		check(bLostZero, "a source that is present but not tracked reads zero, and the face still goes out");

		TrackingFrameResult lostFrame= frame;
		lostFrame.timestampMs= 1030.0;
		lostFrame.face.phone= FaceSources::Phone();
		lostFrame.face.viseme= FaceSources::Viseme();
		runner.evaluate(lostFrame.face, lostFrame.timestampMs);
		encode(lostFrame, messages);
		bool bAllZero= true;
		int zeroCount= 0;
		for (const DecodedMessage& message : messages)
		{
			if (message.address != "/VMC/Ext/Blend/Val")
				continue;
			bAllZero&= message.floats.size() == 1 && message.floats[0] == 0.f;
			++zeroCount;
		}
		check(bAllZero && zeroCount == 6 && findMessage(messages, "/VMC/Ext/Blend/Apply") != nullptr,
			  "the frame both sources stop relaxes every blendshape to zero once");
		encode(lostFrame, messages);
		check(findMessage(messages, "/VMC/Ext/Blend/Val") == nullptr &&
				  findMessage(messages, "/VMC/Ext/Blend/Apply") == nullptr,
			  "no blendshapes go out while both sources stay stopped");

		// Presence is the union of the sources: the phone stopping while the
		// microphone continues is not the face stopping
		auto countBlends= [](const std::vector<DecodedMessage>& frameMessages, int& outZeroCount) {
			int count= 0;
			outZeroCount= 0;
			for (const DecodedMessage& message : frameMessages)
			{
				if (message.address != "/VMC/Ext/Blend/Val")
					continue;
				++count;
				if (message.floats.size() == 1 && message.floats[0] == 0.f)
					++outZeroCount;
			}
			return count;
		};
		int unionZeros= 0;
		TrackingFrameResult bothFrame= frame;
		bothFrame.timestampMs= 1040.0;
		bothFrame.face.viseme.present= true;
		bothFrame.face.viseme.weights[(int)eViseme::aa]= 0.5f;
		runner.evaluate(bothFrame.face, bothFrame.timestampMs);
		encode(bothFrame, messages);
		const int bothCount= countBlends(messages, unionZeros);
		check(bothCount == 6 && unionZeros < bothCount, "the phone and the microphone together send values");

		TrackingFrameResult micOnlyFrame= bothFrame;
		micOnlyFrame.timestampMs= 1050.0;
		micOnlyFrame.face.phone= FaceSources::Phone();
		runner.evaluate(micOnlyFrame.face, micOnlyFrame.timestampMs);
		encode(micOnlyFrame, messages);
		const int micOnlyCount= countBlends(messages, unionZeros);
		blends= collectBlends(messages);
		check(micOnlyCount == 6 && unionZeros < micOnlyCount && blendValue(blends, "A", value) &&
				  fabsf(value - 0.5f) < 1e-6f,
			  "the phone stopping while the microphone continues sends values, no zero frame");

		TrackingFrameResult noneFrame= micOnlyFrame;
		noneFrame.timestampMs= 1060.0;
		noneFrame.face.viseme= FaceSources::Viseme();
		runner.evaluate(noneFrame.face, noneFrame.timestampMs);
		encode(noneFrame, messages);
		const int noneCount= countBlends(messages, unionZeros);
		check(noneCount == 6 && unionZeros == 6, "only both sources stopping sends the one relaxing zero frame");
		noneFrame.timestampMs= 1070.0;
		runner.evaluate(noneFrame.face, noneFrame.timestampMs);
		encode(noneFrame, messages);
		check(countBlends(messages, unionZeros) == 0 && findMessage(messages, "/VMC/Ext/Blend/Apply") == nullptr,
			  "silence follows the relaxing zero frame");

		// The perfect-sync Auto profile on the wire: the ARKit-named
		// expressions alone, under the avatar's spelling
		{
			AvatarModel perfectSync;
			auto addPerfectSync= [&perfectSync](const char* name, const char* preset) {
				AvatarExpression expression;
				expression.name= name;
				expression.preset= preset;
				perfectSync.expressions.push_back(expression);
			};
			addPerfectSync("EyeBlinkLeft", "");
			addPerfectSync("jawOpen", "");
			addPerfectSync("A", "aa");
			FaceProfileRunner syncRunner;
			syncRunner.setBinding(makeAutoFaceBinding(&perfectSync));
			check(syncRunner.getBinding() != nullptr, "the Auto profile of a perfect-sync avatar compiles");

			TrackingFrameResult syncFrame= frame;
			syncFrame.timestampMs= 1080.0;
			syncRunner.evaluate(syncFrame.face, syncFrame.timestampMs);
			OscStreamerConfig syncConfig= config;
			syncConfig.faceBinding= syncRunner.getBinding();
			OscStreamer syncStreamer;
			syncStreamer.setConfig(syncConfig);
			std::vector<DecodedMessage> syncMessages;
			check(encodeOn(syncStreamer, syncFrame, syncMessages), "a perfect-sync frame is complete bundles");
			const std::vector<std::pair<std::string, float>> syncBlends= collectBlends(syncMessages);
			float syncValue= 0.f;
			check(syncBlends.size() == 2 && blendValue(syncBlends, "EyeBlinkLeft", syncValue) &&
					  fabsf(syncValue - 0.8f) < 1e-6f && blendValue(syncBlends, "jawOpen", syncValue) &&
					  fabsf(syncValue - 0.6f) < 1e-6f,
				  "the perfect-sync Auto profile sends its ARKit-named expressions with their values, nothing else");
		}

		// The no-avatar Auto profile
		FaceProfileRunner rawRunner;
		rawRunner.setBinding(makeAutoFaceBinding(nullptr));
		TrackingFrameResult rawFrame= frame;
		rawRunner.evaluate(rawFrame.face, rawFrame.timestampMs);
		config.faceBinding= rawRunner.getBinding();
		config.avatarPath.clear();
		OscStreamer rawStreamer;
		rawStreamer.setConfig(config);
		std::vector<std::vector<uint8_t>> packets;
		rawStreamer.encodeFrame(rawFrame, packets);
		messages.clear();
		for (const std::vector<uint8_t>& packet : packets)
			decodeBundle(packet, messages);
		int rawCount= 0;
		bool bRawNamesKnown= true;
		for (const DecodedMessage& message : messages)
		{
			if (message.address != "/VMC/Ext/Blend/Val" || message.strings.size() != 1)
				continue;
			bRawNamesKnown&= arkitBlendshapeFromName(message.strings[0].c_str()) >= 0;
			++rawCount;
		}
		check(rawCount == ARKIT_BLENDSHAPE_COUNT && bRawNamesKnown,
			  "with no avatar the 52 ARKit names go out verbatim");
		check(findMessage(messages, "/VMC/Ext/VRM") == nullptr, "with no avatar nothing is announced");

		// A frame evaluated under another binding sends no face
		OscStreamerConfig mismatched= config;
		mismatched.faceBinding= runner.getBinding();
		OscStreamer mismatchedStreamer;
		mismatchedStreamer.setConfig(mismatched);
		mismatchedStreamer.encodeFrame(rawFrame, packets);
		messages.clear();
		for (const std::vector<uint8_t>& packet : packets)
			decodeBundle(packet, messages);
		check(findMessage(messages, "/VMC/Ext/Blend/Val") == nullptr,
			  "values from another binding's generation are not sent under this binding's names");
	}

	// (k) The binding's rules, which decide what any receiver can drive
	{
		auto addExpression= [](AvatarModel& model, const char* name, const char* preset) {
			AvatarExpression expression;
			expression.name= name;
			expression.preset= preset;
			model.expressions.push_back(expression);
		};
		auto phoneFrame= [](std::initializer_list<std::pair<const char*, float>> values) {
			FaceSources sources;
			sources.phone.present= true;
			sources.phone.tracked= true;
			for (const auto& entry : values)
				sources.phone.blendshapes[arkitBlendshapeFromName(entry.first)]= entry.second;
			return sources;
		};

		AvatarModel perfectSync;
		addExpression(perfectSync, "EyeBlinkLeft", "");
		addExpression(perfectSync, "jawOpen", "");
		addExpression(perfectSync, "A", "aa");
		FaceProfileRunner runner;
		runner.setBinding(makeAutoFaceBinding(&perfectSync));
		const FaceProfileBinding* binding= runner.getBinding().get();
		check(binding != nullptr && binding->getWireNames().size() == 2 && binding->getWireNames()[0] == "EyeBlinkLeft",
			  "perfect sync streams its ARKit expressions alone, in the avatar's spelling");
		FaceSources sources= phoneFrame({{"jawOpen", 1.7f}, {"eyeBlinkLeft", 0.25f}});
		runner.evaluate(sources, 0.0);
		check(binding != nullptr && sources.outputs.targetValues.size() == 2 && sources.outputs.targetValues[1] == 1.f &&
				  fabsf(sources.outputs.targetValues[0] - 0.25f) < 1e-6f,
			  "a perfect-sync expression follows its column, a value past full clamped to one");
		sources= FaceSources();
		sources.viseme.present= true;
		sources.viseme.weights[(int)eViseme::aa]= 1.f;
		runner.evaluate(sources, 0.0);
		check(binding != nullptr && fabsf(sources.outputs.targetValues[1] - 0.85f) < 1e-6f,
			  "a perfect-sync jawOpen reads the visemes through the table while the phone sees no face");

		AvatarModel blinkOnly;
		addExpression(blinkOnly, "Blink", "blink");
		runner.setBinding(makeAutoFaceBinding(&blinkOnly));
		sources= phoneFrame({{"eyeBlinkLeft", 1.f}});
		runner.evaluate(sources, 0.0);
		check(runner.getBinding() != nullptr && runner.getBinding()->getWireNames().size() == 1 &&
				  fabsf(sources.outputs.targetValues[0] - 0.5f) < 1e-6f,
			  "an avatar with only the two-eyed blink gets the average of both eyes");

		// The receiver half and the combine rules on one avatar
		AvatarModel morphed;
		AvatarMesh face;
		face.name= "Face";
		face.morphTargetNames= {"jawOpen", "blinkShape", "browShape"};
		morphed.meshes.push_back(face);
		AvatarExpression aa;
		aa.name= "A";
		aa.preset= "aa";
		aa.morphBinds.push_back(AvatarMorphBind{0, 0, 1.f});
		morphed.expressions.push_back(aa);
		AvatarExpression blinkLeft;
		blinkLeft.name= "Blink_L";
		blinkLeft.preset= "blinkLeft";
		blinkLeft.morphBinds.push_back(AvatarMorphBind{0, 1, 0.5f});
		morphed.expressions.push_back(blinkLeft);

		runner.setBinding(makeAutoFaceBinding(&morphed));
		sources= phoneFrame({{"jawOpen", 0.6f}, {"eyeBlinkLeft", 1.f}});
		runner.evaluate(sources, 0.0);
		std::vector<std::vector<float>> meshWeights;
		bool bAutoMorphs= runner.getBinding() != nullptr;
		if (bAutoMorphs)
			runner.getBinding()->evaluateMorphs(&sources.outputs.targetValues, meshWeights);
		check(bAutoMorphs && meshWeights.size() == 1 && meshWeights[0].size() == 3 &&
				  fabsf(meshWeights[0][0] - 0.6f) < 1e-6f,
			  "a morph driven by a preset and by its own ARKit name takes the larger weight, not the sum");
		check(bAutoMorphs && fabsf(meshWeights[0][1] - 0.5f) < 1e-6f,
			  "an expression scales the morphs it binds by the bind weight");
		if (bAutoMorphs)
			runner.getBinding()->evaluateMorphs(nullptr, meshWeights);
		check(bAutoMorphs && meshWeights[0][0] == 0.f && meshWeights[0][1] == 0.f, "no face rests every morph");

		// A user profile: a direct morph output, two outputs on one target,
		// and a target the avatar lacks
		FaceProfile profile;
		profile.id= "test";
		auto addOutput= [&profile](eFaceTargetKind kind, const char* target, const char* expr, const char* mesh= "") {
			FaceProfileOutput output;
			output.targetKind= kind;
			output.target= target;
			output.mesh= mesh;
			output.vrmExpressionFunction= expr;
			profile.outputs.push_back(output);
		};
		addOutput(eFaceTargetKind::Morph, "browShape", "phone.browInnerUp", "Face");
		addOutput(eFaceTargetKind::Expression, "A", "phone.jawOpen");
		addOutput(eFaceTargetKind::Expression, "a", "viseme.aa");
		addOutput(eFaceTargetKind::Morph, "jawOpen", "0.3");
		addOutput(eFaceTargetKind::Expression, "Missing", "1");
		addOutput(eFaceTargetKind::Morph, "browShape", "1", "Body");
		std::vector<FaceExprCompileError> errors;
		std::shared_ptr<const FaceProfileBinding> userBinding= compileFaceBinding(profile, &morphed, &errors);
		const bool bUser= userBinding != nullptr && errors.empty();
		check(bUser, "a user profile compiles and binds");
		if (bUser)
		{
			const std::vector<int>& outputTargets= userBinding->getOutputTargets();
			check(outputTargets[1] >= 0 && outputTargets[1] == outputTargets[2],
				  "two outputs naming one expression, in either case, share its target");
			check(outputTargets[4] < 0 && !userBinding->getOutputProblems()[4].empty() && outputTargets[5] < 0 &&
					  !userBinding->getOutputProblems()[5].empty(),
				  "a missing expression and a missing mesh are reported and skipped");
			check(std::find(userBinding->getWireNames().begin(), userBinding->getWireNames().end(), "Missing") ==
					  userBinding->getWireNames().end(),
				  "an unresolved target is not sent");
			AvatarModel reshaped= morphed;
			reshaped.meshes[0].morphTargetNames.push_back("extraShape");
			check(userBinding->fitsModel(morphed) && !userBinding->fitsModel(reshaped),
				  "a binding fits the model it was built against and not one with another morph count");

			FaceProfileRunner userRunner;
			userRunner.setBinding(userBinding);
			FaceSources userSources= phoneFrame({{"jawOpen", 0.2f}, {"browInnerUp", 0.7f}});
			userSources.viseme.present= true;
			userSources.viseme.weights[(int)eViseme::aa]= 0.9f;
			userRunner.evaluate(userSources, 0.0);

			std::vector<int> targetWinners;
			std::vector<float> targetValues;
			userBinding->combineTargets(userSources.outputs.outputValues, targetValues, &targetWinners);
			const int aTarget= outputTargets[1];
			check(fabsf(targetValues[(size_t)aTarget] - 0.9f) < 1e-6f && targetWinners[(size_t)aTarget] == 2,
				  "two outputs on one target combine with max, and the winner is reported");

			std::vector<std::vector<int>> morphWinners;
			userBinding->evaluateMorphs(&userSources.outputs.targetValues, meshWeights, &morphWinners);
			check(fabsf(meshWeights[0][2] - 0.7f) < 1e-6f && morphWinners[0][2] == outputTargets[0],
				  "a direct morph target output reaches its morph at weight 1");
			check(fabsf(meshWeights[0][0] - 0.9f) < 1e-6f && morphWinners[0][0] == aTarget,
				  "max combine across targets reports the winning target per morph");
			// The morph's winning output: through its winning target to the
			// output that won that target, the second of the two on it
			const int jawWinner= morphWinners[0][0];
			check(jawWinner == aTarget && targetWinners[(size_t)jawWinner] == 2 &&
					  userBinding->getTargets()[(size_t)jawWinner].outputs.size() == 2 &&
					  userBinding->getTargets()[(size_t)jawWinner].outputs[1] == 2,
				  "a morph's winner names the target, and that target's winner is the second output on it");

			OscStreamerConfig config;
			config.maxRateHz= 0.f;
			config.faceBinding= userBinding;
			OscStreamer streamer;
			streamer.setConfig(config);
			TrackingFrameResult frame;
			frame.timestampMs= 1000.0;
			frame.face= userSources;
			std::vector<std::vector<uint8_t>> packets;
			streamer.encodeFrame(frame, packets);
			std::vector<DecodedMessage> messages;
			for (const std::vector<uint8_t>& packet : packets)
				decodeBundle(packet, messages);
			int aCount= 0;
			float aValue= 0.f;
			for (const DecodedMessage& message : messages)
			{
				if (message.address == "/VMC/Ext/Blend/Val" && message.strings.size() == 1 && message.strings[0] == "A")
				{
					++aCount;
					aValue= message.floats.empty() ? 0.f : message.floats[0];
				}
			}
			check(aCount == 1 && fabsf(aValue - 0.9f) < 1e-6f, "two outputs on one target send one message carrying the max");

			// A binding swap relaxes the names the new binding does not send
			OscStreamerConfig swapped= config;
			std::shared_ptr<const FaceProfileBinding> autoBinding= makeAutoFaceBinding(&morphed);
			swapped.faceBinding= autoBinding;
			streamer.setConfig(swapped);
			FaceProfileRunner autoRunner;
			autoRunner.setBinding(autoBinding);
			frame.timestampMs= 1010.0;
			autoRunner.evaluate(frame.face, frame.timestampMs);
			streamer.encodeFrame(frame, packets);
			messages.clear();
			for (const std::vector<uint8_t>& packet : packets)
				decodeBundle(packet, messages);
			bool bBrowRelaxed= false;
			for (const DecodedMessage& message : messages)
			{
				if (message.address == "/VMC/Ext/Blend/Val" && message.strings.size() == 1 &&
					message.strings[0] == "browShape")
					bBrowRelaxed= message.floats.size() == 1 && message.floats[0] == 0.f;
			}
			check(bBrowRelaxed, "a binding swap sends zero once for a name the new binding drops");
		}
	}

	// (l) The phone's head in the head frame (+X facing, +Y left, +Z up)
	{
		check(isIdentity(faceHeadDeltaFromPhoneEuler(glm::vec3(0.f)), 1e-6f),
			  "looking straight at the phone is no rotation");

		const glm::vec3 facing(1.f, 0.f, 0.f);
		const glm::quat yaw= faceHeadDeltaFromPhoneEuler(glm::vec3(0.f, 90.f, 0.f));
		check(nearlyEqual(yaw * facing, glm::vec3(0.f, 1.f, 0.f), 1e-5f), "yaw turns about up");
		const glm::quat pitch= faceHeadDeltaFromPhoneEuler(glm::vec3(90.f, 0.f, 0.f));
		check(nearlyEqual(pitch * facing, glm::vec3(0.f, 0.f, -1.f), 1e-5f), "pitch turns about the head's left axis");
		const glm::quat roll= faceHeadDeltaFromPhoneEuler(glm::vec3(0.f, 0.f, 90.f));
		check(nearlyEqual(roll * glm::vec3(0.f, 1.f, 0.f), glm::vec3(0.f, 0.f, 1.f), 1e-5f),
			  "roll turns about the facing axis");

		const glm::vec3 euler(12.f, -35.f, 8.f);
		const glm::quat composed= faceHeadDeltaFromPhoneEuler(euler);
		const glm::quat expected= faceHeadDeltaFromPhoneEuler(glm::vec3(0.f, euler.y, 0.f)) *
								  faceHeadDeltaFromPhoneEuler(glm::vec3(euler.x, 0.f, 0.f)) *
								  faceHeadDeltaFromPhoneEuler(glm::vec3(0.f, 0.f, euler.z));
		check(fabsf(fabsf(glm::dot(composed, expected)) - 1.f) < 1e-5f, "yaw, then pitch, then roll about the head");

		const glm::quat headWorld= glm::angleAxis(0.7f, glm::normalize(glm::vec3(0.2f, -0.4f, 1.f)));
		const glm::quat anchor= faceHeadAnchorFromWorld(headWorld, euler);
		check(fabsf(fabsf(glm::dot(faceHeadWorldOrientation(anchor, euler), headWorld)) - 1.f) < 1e-5f,
			  "the captured anchor reproduces the camera head it was aligned to");
	}

	if (failures == 0)
		MIKAN_LOG_INFO("test-vmc") << "All VMC retarget tests passed";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-vmc",
					"VMC retarget: axis conversion, rest identity, chain round trip, the face profile binding and "
					"blend messages",
					eTestCategory::SelfTest, runVmcTest);
