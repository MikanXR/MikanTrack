#include "AvatarRetargetMath.h"

#include <algorithm>
#include <cmath>

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/quaternion.hpp"

#include "AvatarSkeleton.h"
#include "HandPoseModel.h"
#include "MathGLM.h"

namespace AvatarRetargetMath
{
glm::vec3 safeNormalize(const glm::vec3& v)
{
	const float lengthSquared= glm::dot(v, v);
	return lengthSquared > 1e-12f ? v / std::sqrt(lengthSquared) : glm::vec3(0.f);
}

float signedAngle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& axis)
{
	return atan2f(glm::dot(glm::cross(a, b), axis), glm::dot(a, b));
}

glm::vec3 projectPerpendicular(const glm::vec3& v, const glm::vec3& unitAxis)
{
	return v - unitAxis * glm::dot(v, unitAxis);
}

float wrapAngle(float radians)
{
	while (radians > glm::pi<float>())
		radians-= glm::two_pi<float>();
	while (radians < -glm::pi<float>())
		radians+= glm::two_pi<float>();
	return radians;
}

eHumanoidBone sideBone(int sideIndex, eHumanoidBone left, eHumanoidBone right)
{
	return sideIndex == 0 ? left : right;
}

eHumanoidBone fingerBone(int sideIndex, int finger, int phalanx)
{
	return (eHumanoidBone)((int)firstHumanoidFingerBone(sideIndex) + finger * 3 + phalanx);
}

glm::quat swingTo(const glm::quat& parentDelta, const glm::vec3& rootedRestDirection, const glm::vec3& worldDirection)
{
	return glm_shortest_arc(parentDelta * rootedRestDirection, worldDirection) * parentDelta;
}

void swingTwist(const glm::quat& rotation, const glm::vec3& unitAxis, glm::quat& outSwing, glm::quat& outTwist)
{
	// The twist is the rotation's projection onto the axis: keep the
	// quaternion's vector part along the axis, renormalize
	const glm::vec3 vectorPart(rotation.x, rotation.y, rotation.z);
	const glm::vec3 projected= unitAxis * glm::dot(vectorPart, unitAxis);
	glm::quat twist(rotation.w, projected.x, projected.y, projected.z);
	const float twistLength= glm::length(twist);
	if (twistLength < 1e-6f)
	{
		// A half turn about an axis perpendicular to this one: no twist
		twist= glm::quat(1.f, 0.f, 0.f, 0.f);
	}
	else
	{
		twist/= twistLength;
	}
	outTwist= twist;
	outSwing= rotation * glm::inverse(twist);
}

glm::quat limitRotationAngle(const glm::quat& rotation, float maxRadians)
{
	const glm::quat unit= glm::normalize(rotation);
	// Keep the short-arc representation so the angle reads below pi
	const glm::quat shortArc= unit.w < 0.f ? -unit : unit;
	const float angle= 2.f * acosf(std::min(shortArc.w, 1.f));
	if (angle <= maxRadians)
		return shortArc;
	const glm::vec3 axis= safeNormalize(glm::vec3(shortArc.x, shortArc.y, shortArc.z));
	if (glm::dot(axis, axis) <= 0.f)
		return glm::quat(1.f, 0.f, 0.f, 0.f);
	return glm::angleAxis(maxRadians, axis);
}

void solveFingerBones(const AvatarSkeleton& skeleton, int sideIndex, const glm::quat& rootRotation,
					  const glm::quat& handDelta, const glm::vec3& handPositionWorld,
					  const std::array<FingerAngles, FINGER_COUNT>& measuredAngles,
					  const AvatarRetargetConfig::Side& sideConfig, AvatarPose& outPose)
{
	using B= eHumanoidBone;
	const eHandSide side= (eHandSide)sideIndex;
	const B handBone= sideBone(sideIndex, B::LeftHand, B::RightHand);
	const AvatarSkeleton::HandRest& hand= skeleton.getHand(side);
	const glm::quat restPalmRotation= glm::quat_cast(glm::mat3(hand.palmFrameWorld));

	// The posed palm: the hand delta over the rooted rest palm, placed so the
	// palm center keeps its rest offset from the hand joint
	const glm::vec3 restPalmCenter(hand.palmFrameWorld[3]);
	const glm::vec3 restHandPosition= skeleton.getBone(handBone).restPositionWorld;
	const glm::quat posedPalmRotation= handDelta * rootRotation * restPalmRotation;
	glm::mat4 palmTransform= glm::mat4_cast(posedPalmRotation);
	palmTransform[3]=
		glm::vec4(handPositionWorld + handDelta * (rootRotation * (restPalmCenter - restHandPosition)), 1.f);

	// The rig's gains scale the measured angles first. Then a rig with fewer
	// bones than the measured finger folds the bends past its last bone into
	// that bone, so a fist still closes on a two-bone finger.
	std::array<FingerAngles, FINGER_COUNT> angles= measuredAngles;
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		FingerAngles& fingerAngles= angles[finger];
		fingerAngles.lateral*= sideConfig.splayGain;
		fingerAngles.proximal*= sideConfig.curlGain;
		fingerAngles.intermediate*= sideConfig.curlGain;
		fingerAngles.distal*= sideConfig.curlGain;
		switch (hand.fingerBoneCount[finger])
		{
		case 1:
			fingerAngles.proximal+= fingerAngles.intermediate + fingerAngles.distal;
			fingerAngles.intermediate= 0.f;
			fingerAngles.distal= 0.f;
			break;
		case 2:
			fingerAngles.intermediate+= fingerAngles.distal;
			fingerAngles.distal= 0.f;
			break;
		default:
			break;
		}
	}

	std::array<std::array<glm::vec3, 4>, FINGER_COUNT> joints;
	HandPoseModel::buildFingerJoints(palmTransform, hand.skeleton, angles, joints,
									 sideConfig.bThumbPronationOverride ? &sideConfig.thumbPronationRad : nullptr);

	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		// A disabled finger stays absent and follows the hand at rest
		if (!sideConfig.fingerEnabled[finger])
			continue;
		glm::quat fingerDelta= handDelta;
		for (int index= 0; index < hand.fingerBoneCount[finger]; ++index)
		{
			const glm::vec3 direction= safeNormalize(joints[finger][index + 1] - joints[finger][index]);
			if (glm::dot(direction, direction) > 0.f)
				fingerDelta= swingTo(fingerDelta, rootRotation * hand.restPhalanxDirWorld[finger][index], direction);
			AvatarPose::Bone& bone= outPose.bones[(int)fingerBone(sideIndex, finger, hand.fingerSlots[finger][index])];
			bone.present= true;
			bone.deltaWorld= fingerDelta;
		}
	}
}
} // namespace AvatarRetargetMath
