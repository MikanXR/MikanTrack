#include "KalidokitStyleRetarget.h"

#include <cmath>

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"

#include "AvatarRetargetMath.h"
#include "AvatarSkeleton.h"
#include "BodyPoseTypes.h"
#include "HandPoseModel.h"

using B= eHumanoidBone;
using namespace AvatarRetargetMath;

namespace
{
// The direct solver's landmark rules, so both candidates read the same joints
constexpr float kMinVisibility= 0.5f;
constexpr float kFrameMarginPx= 16.f;
constexpr float kWristLimitRadians= 80.f * glm::pi<float>() / 180.f;

glm::vec3 restDirection(const AvatarSkeleton& skeleton, B bone)
{
	return safeNormalize(skeleton.getBone(bone).restOffsetFromParentWorld);
}
} // namespace

bool KalidokitStyleRetarget::solveRootDepthFromShoulders(const glm::vec3& leftRay, float leftDepth,
														   const glm::vec3& rightRay, float rightDepth,
														   float shoulderWidthMeters, float& outRootDepth)
{
	const glm::vec3 a= leftRay - rightRay;
	const glm::vec3 b= leftRay * leftDepth - rightRay * rightDepth;
	const float qa= glm::dot(a, a);
	const float qb= 2.f * glm::dot(a, b);
	const float qc= glm::dot(b, b) - shoulderWidthMeters * shoulderWidthMeters;
	if (qa < 1e-12f)
		return false;
	const float discriminant= qb * qb - 4.f * qa * qc;
	if (discriminant < 0.f)
		return false;
	outRootDepth= (-qb + sqrtf(discriminant)) / (2.f * qa);
	return outRootDepth > 0.f;
}

void KalidokitStyleRetarget::reset()
{
	m_bones= {};
}

void KalidokitStyleRetarget::solve(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
								   const AvatarSkeleton& skeleton, const DirectRetargetConfig& config,
								   AvatarPose& outPose)
{
	outPose.clear();
	outPose.valid= true;
	outPose.timestampMs= camera.timestampMs;

	const glm::quat worldFromCam= DirectRetarget::worldFromCamera(config.cameraTiltRadians);
	const BodyPoseObservation& body= camera.body;
	const bool bFrameKnown= cameraModel.width > 0 && cameraModel.height > 0;
	auto usable= [&](int index) {
		if (!body.valid || !body.isProvided(index) || body.visibility[index] < kMinVisibility)
			return false;
		if (!bFrameKnown)
			return true;
		const glm::vec3& point= body.imagePoints[index];
		return point.x >= kFrameMarginPx && point.y >= kFrameMarginPx &&
			point.x <= (float)cameraModel.width - kFrameMarginPx && point.y <= (float)cameraModel.height - kFrameMarginPx;
	};
	// A pixel's ray at unit depth, OpenCV camera axes
	auto rayAtUnitDepth= [&](const glm::vec3& point) {
		return glm::vec3((point.x - cameraModel.cx) / cameraModel.fx, (point.y - cameraModel.cy) / cameraModel.fy, 1.f);
	};

	// The targets this frame; a bone with none keeps its last
	std::array<bool, HUMANOID_BONE_COUNT> bTargeted{};
	std::array<glm::quat, HUMANOID_BONE_COUNT> targets{};
	auto target= [&](B bone, const glm::quat& delta) {
		if (!skeleton.getBone(bone).present)
			return;
		bTargeted[(int)bone]= true;
		targets[(int)bone]= delta;
	};
	target(B::Hips, glm::quat(1.f, 0.f, 0.f, 0.f));

	// The root depth: the shoulders a known width apart, the model's relative
	// depth stretched by the measured compression
	const int leftShoulderIndex= (int)ePoseLandmark::LEFT_SHOULDER;
	const int rightShoulderIndex= (int)ePoseLandmark::RIGHT_SHOULDER;
	const float depthGain= body.hasDepth ? 1.f / kModelDepthScale : 0.f;
	bool bHaveRoot= false;
	float rootDepth= 0.f;
	if (usable(leftShoulderIndex) && usable(rightShoulderIndex))
	{
		bHaveRoot= solveRootDepthFromShoulders(
			rayAtUnitDepth(body.imagePoints[leftShoulderIndex]), body.imagePoints[leftShoulderIndex].z * depthGain,
			rayAtUnitDepth(body.imagePoints[rightShoulderIndex]), body.imagePoints[rightShoulderIndex].z * depthGain,
			config.userShoulderWidthMeters, rootDepth);
	}
	auto lift= [&](int index) {
		return worldFromCam * (rayAtUnitDepth(body.imagePoints[index]) * (rootDepth + body.imagePoints[index].z * depthGain));
	};

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const eHandSide side= (eHandSide)sideIndex;
		const AvatarRetargetConfig::Side& sideConfig= config.sides[sideIndex];
		const TrackedHand& hand= camera.hands[sideIndex];
		const int shoulderIndex= sideIndex == 0 ? leftShoulderIndex : rightShoulderIndex;
		const int elbowIndex= sideIndex == 0 ? (int)ePoseLandmark::LEFT_ELBOW : (int)ePoseLandmark::RIGHT_ELBOW;
		const int wristIndex= sideIndex == 0 ? (int)ePoseLandmark::LEFT_WRIST : (int)ePoseLandmark::RIGHT_WRIST;
		const B upperArmBone= sideBone(sideIndex, B::LeftUpperArm, B::RightUpperArm);
		const B lowerArmBone= sideBone(sideIndex, B::LeftLowerArm, B::RightLowerArm);
		const B handBone= sideBone(sideIndex, B::LeftHand, B::RightHand);

		// The arm: each bone swung onto the lifted direction, the parent's
		// placement (this frame's target, else the last) under it
		if (!bHaveRoot)
			continue;
		glm::quat chainDelta(1.f, 0.f, 0.f, 0.f);
		bool bUpper= false;
		if (usable(shoulderIndex) && usable(elbowIndex))
		{
			const glm::vec3 upperDirection= safeNormalize(lift(elbowIndex) - lift(shoulderIndex));
			if (glm::dot(upperDirection, upperDirection) > 0.f)
			{
				chainDelta= swingTo(chainDelta, restDirection(skeleton, lowerArmBone), upperDirection);
				target(upperArmBone, chainDelta);
				bUpper= true;
			}
		}
		if (!bUpper)
			chainDelta= m_bones[(int)upperArmBone].deltaWorld;

		glm::vec3 forearmDirection= chainDelta * restDirection(skeleton, handBone);
		bool bForearm= false;
		if (usable(elbowIndex) && usable(wristIndex))
		{
			const glm::vec3 direction= safeNormalize(lift(wristIndex) - lift(elbowIndex));
			if (glm::dot(direction, direction) > 0.f)
			{
				forearmDirection= direction;
				chainDelta= swingTo(chainDelta, restDirection(skeleton, handBone), forearmDirection);
				bForearm= true;
			}
		}
		if (!bForearm)
			chainDelta= m_bones[(int)lowerArmBone].deltaWorld;

		// The hand: the palm plane from the hand model's landmarks with their
		// relative depth, in pixel units at one depth (directions only), the
		// same palm frame definition the lift uses, through the direct
		// conversion. No memory of the palmar side, no mirror rule.
		const AvatarSkeleton::HandRest& handRest= skeleton.getHand(side);
		float twist= sideConfig.forearmRollTrimRadians;
		glm::quat relative(1.f, 0.f, 0.f, 0.f);
		bool bHand= false;
		if (hand.tracked)
		{
			std::array<glm::vec3, HAND_LANDMARK_COUNT> points;
			for (int i= 0; i < HAND_LANDMARK_COUNT; ++i)
			{
				const glm::vec3& px= hand.imagePoints[i];
				points[i]= glm::vec3((px.x - cameraModel.cx) / cameraModel.fx, (px.y - cameraModel.cy) / cameraModel.fy,
									 px.z / cameraModel.fx);
			}
			const glm::mat4 palmFrame= HandPoseModel::computePalmFrame(points, side);
			const glm::quat palm= glm::normalize(worldFromCam * glm::quat_cast(glm::mat3(palmFrame)));
			const glm::vec3 palmY= palm * glm::vec3(0.f, 1.f, 0.f);
			const glm::vec3 palmZ= palm * glm::vec3(0.f, 0.f, 1.f);
			const glm::vec3 restPalmNormal= safeNormalize(
				projectPerpendicular(chainDelta * glm::vec3(handRest.palmFrameWorld[2]), forearmDirection));
			glm::vec3 forearmZ= safeNormalize(projectPerpendicular(palmZ, forearmDirection));
			if (glm::dot(forearmZ, forearmZ) <= 0.f)
				forearmZ= safeNormalize(projectPerpendicular(palmY, forearmDirection));
			if (glm::dot(forearmZ, forearmZ) > 0.f)
			{
				if (glm::dot(restPalmNormal, restPalmNormal) > 0.f)
					twist+= signedAngle(restPalmNormal, forearmZ, forearmDirection);
				const glm::vec3 forearmY= glm::cross(forearmZ, forearmDirection);
				const glm::quat forearmFrame= glm::quat_cast(glm::mat3(forearmDirection, forearmY, forearmZ));
				relative= limitRotationAngle(glm::inverse(forearmFrame) * palm, kWristLimitRadians);
				bHand= true;
			}
		}
		if (bForearm)
		{
			chainDelta= glm::angleAxis(twist, forearmDirection) * chainDelta;
			target(lowerArmBone, chainDelta);
		}
		if (bHand)
		{
			const glm::quat restLowerArm= skeleton.getBone(lowerArmBone).restRotationWorld;
			const glm::quat trimmedRestPalm=
				glm::quat_cast(glm::mat3(handRest.palmFrameWorld)) * glm::quat(sideConfig.handTrimRadians);
			const glm::quat posedForearm= chainDelta * restLowerArm;
			const glm::quat posedPalm= posedForearm * (glm::inverse(restLowerArm) * trimmedRestPalm) * relative;
			target(handBone, posedPalm * glm::inverse(trimmedRestPalm));
		}
	}

	// The slerp toward every target, then the pose: a bone placed before
	// stays placed
	for (int boneIndex= 0; boneIndex < HUMANOID_BONE_COUNT; ++boneIndex)
	{
		AvatarPose::Bone& bone= m_bones[boneIndex];
		if (bTargeted[boneIndex])
		{
			bone.deltaWorld= bone.present ? glm::slerp(bone.deltaWorld, targets[boneIndex], kSlerp) : targets[boneIndex];
			bone.present= true;
		}
		if (bone.present)
			outPose.bones[boneIndex]= bone;
	}

	// The fingers on the slerped hand: the camera's raw angles less the rest
	// zero, through the shared forward kinematics, each bone slerped like
	// the rest
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= camera.poses[sideIndex];
		const B handBone= sideBone(sideIndex, B::LeftHand, B::RightHand);
		if (!pose.tracked || !outPose.bones[(int)handBone].present)
			continue;
		std::array<FingerAngles, FINGER_COUNT> angles= pose.fingers;
		if (config.bHasRestAngles[sideIndex])
		{
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				const FingerAngles& rest= config.restAngles[sideIndex][finger];
				angles[finger].lateral-= rest.lateral;
				angles[finger].proximal-= rest.proximal;
				angles[finger].intermediate-= rest.intermediate;
				angles[finger].distal-= rest.distal;
			}
		}
		solveFingerBones(skeleton, sideIndex, glm::quat(1.f, 0.f, 0.f, 0.f), outPose.bones[(int)handBone].deltaWorld,
						 skeleton.getBone(handBone).restPositionWorld, angles, config.sides[sideIndex], outPose);
		// The finger bones slerp like the rest
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			for (int phalanx= 0; phalanx < 3; ++phalanx)
			{
				const int boneIndex= (int)fingerBone(sideIndex, finger, phalanx);
				AvatarPose::Bone& out= outPose.bones[boneIndex];
				if (!out.present)
					continue;
				AvatarPose::Bone& last= m_bones[boneIndex];
				if (last.present)
					out.deltaWorld= glm::slerp(last.deltaWorld, out.deltaWorld, kSlerp);
				last= out;
			}
		}
	}
}
