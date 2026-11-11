#include "AvatarRetarget.h"

#include <algorithm>
#include <cmath>

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/quaternion.hpp"

#include "AvatarRetargetMath.h"
#include "AvatarSkeleton.h"
#include "MathGLM.h"

namespace
{
using B= eHumanoidBone;
using namespace AvatarRetargetMath;

// Rest geometry under the root placement, which every delta is measured from
struct RootedRest
{
	const AvatarSkeleton* skeleton= nullptr;
	glm::vec3 position{0.f};
	glm::quat rotation{1.f, 0.f, 0.f, 0.f};

	glm::vec3 bonePosition(B bone) const { return position + rotation * skeleton->getBone(bone).restPositionWorld; }
	glm::vec3 offset(B bone) const { return rotation * skeleton->getBone(bone).restOffsetFromParentWorld; }
	glm::vec3 direction(B bone) const { return safeNormalize(offset(bone)); }
};

// Elbow on the circle where the upper-arm sphere about the shoulder meets
// the forearm sphere about the wrist, at the point nearest the pole. A wrist
// beyond reach straightens the arm; the caller clamps reach beforehand.
glm::vec3 solveElbow(const glm::vec3& shoulder, const glm::vec3& wrist, float upperArm, float forearm,
					 const glm::vec3& polePoint, const glm::vec3& fallbackPoleDirection, const glm::vec3& restDirection)
{
	const glm::vec3 toWrist= wrist - shoulder;
	const float reach= glm::length(toWrist);
	const glm::vec3 axis= reach > 1e-6f ? toWrist / reach : restDirection;

	// At (or within rounding of) full reach the arm is straight; the bend
	// direction is meaningless there and would only jitter
	if (reach >= (upperArm + forearm) * (1.f - 1e-5f))
		return shoulder + axis * upperArm;

	// Law of cosines: distance along the axis from the shoulder to the
	// elbow circle's center, then the circle's radius
	float along= (upperArm * upperArm - forearm * forearm + reach * reach) / std::max(2.f * reach, 1e-6f);
	along= std::clamp(along, -upperArm, upperArm);
	const float radius= std::sqrt(std::max(upperArm * upperArm - along * along, 0.f));
	const glm::vec3 center= shoulder + axis * along;

	glm::vec3 bend= projectPerpendicular(polePoint - center, axis);
	if (glm::dot(bend, bend) < 1e-8f)
		bend= projectPerpendicular(fallbackPoleDirection, axis);
	if (glm::dot(bend, bend) < 1e-8f)
	{
		// The pole sits on the arm axis: any perpendicular will do, chosen
		// deterministically
		bend= glm::cross(axis, glm::vec3(0.f, 0.f, 1.f));
		if (glm::dot(bend, bend) < 1e-8f)
			bend= glm::cross(axis, glm::vec3(1.f, 0.f, 0.f));
	}
	return center + safeNormalize(bend) * radius;
}
} // namespace

void AvatarPose::clear()
{
	valid= false;
	timestampMs= 0.0;
	rootPositionWorld= glm::vec3(0.f);
	rootRotationWorld= glm::quat(1.f, 0.f, 0.f, 0.f);
	bones= {};
}

void AvatarRetarget::reset()
{
	m_root= RootState();
	m_resetEpoch++;
	m_bHasFillTime= false;
	m_lastFillStepMs= 0.0;
}

void AvatarRetarget::fillSource(const std::array<HandPose, 2>& poses, const bool bSideValid[2],
								const TrackingFrameResult::HeadPose& head, double timestampMs,
								const BodyDimensions& user, SourcePose& outSource)
{
	using eGroupState= SourcePose::eGroupState;
	outSource= SourcePose();
	outSource.pipeline= SourcePose::ePipeline::Position;
	// A small backward step holds the step clock at its high-water mark
	double stepMs= timestampMs;
	if (m_bHasFillTime && timestampMs < m_lastFillStepMs &&
		m_lastFillStepMs - timestampMs <= SourcePose::Clock::kMaxBackwardStepMs)
	{
		stepMs= m_lastFillStepMs;
	}
	outSource.clock.timestampMs= timestampMs;
	outSource.clock.stepMs= stepMs;
	outSource.clock.frameIntervalMs= m_bHasFillTime ? stepMs - m_lastFillStepMs : 0.0;
	outSource.clock.resetEpoch= m_resetEpoch;
	m_bHasFillTime= true;
	m_lastFillStepMs= stepMs;
	outSource.upperArmLength= user.upperArmLengthMeters;
	outSource.forearmLength= user.forearmLengthMeters;

	if (head.valid)
	{
		outSource.head.state= eGroupState::Measured;
		outSource.head.confidence.bPresent= true;
		outSource.head.confidence.value= head.confidence;
		outSource.head.orientation= head.orientationWorld;
	}

	const bool bBothShoulders= bSideValid[0] && bSideValid[1] && poses[0].hasShoulder && poses[1].hasShoulder;
	outSource.torso.state= bBothShoulders ? eGroupState::Measured : eGroupState::Lost;

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (!bSideValid[sideIndex])
			continue;
		const HandPose& pose= poses[sideIndex];

		SourcePose::Arm& arm= outSource.arms[sideIndex];
		arm.state= eGroupState::Measured;
		arm.bHasShoulder= pose.hasShoulder;
		if (pose.hasShoulder)
		{
			arm.shoulderPosition= pose.shoulderPositionWorld;
			arm.shoulderConfidence.bPresent= true;
			arm.shoulderConfidence.value= pose.shoulderConfidence;
		}
		arm.wristPosition= pose.getWristPositionWorld();
		arm.bHasForearmOrientation= pose.hasForearmPose;
		arm.bHasElbow= pose.hasForearmPose;
		if (pose.hasForearmPose)
		{
			arm.forearmOrientation= pose.forearmOrientationWorld;
			arm.forearmConfidence.bPresent= true;
			arm.forearmConfidence.value= pose.forearmConfidence;
			arm.elbowPosition= pose.getElbowPositionWorld(user.forearmLengthMeters);
		}

		SourcePose::Hand& hand= outSource.hands[sideIndex];
		hand.state= eGroupState::Measured;
		hand.confidence.bPresent= true;
		hand.confidence.value= pose.confidence;
		hand.palmOrientation= pose.palmOrientationWorld;
		hand.fingerZero= SourcePose::eFingerZero::CapturedRest;
		hand.fingers= pose.fingers;
	}
}

void AvatarRetarget::fillSource(const TrackingFrameResult& frame, const BodyDimensions& user, SourcePose& outSource)
{
	const bool bSideValid[2]= {
		frame.poses[0].tracked && frame.poses[0].hasWorldPose,
		frame.poses[1].tracked && frame.poses[1].hasWorldPose,
	};
	fillSource(frame.poses, bSideValid, frame.head, frame.timestampMs, user, outSource);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= frame.poses[sideIndex];
		if (pose.tracked && !pose.hasWorldPose)
		{
			outSource.hands[sideIndex].confidence.bPresent= true;
			outSource.hands[sideIndex].confidence.value= pose.confidence;
		}
	}
}

void AvatarRetarget::resetRootFollow()
{
	m_root= RootState();
}

void AvatarRetarget::solve(const std::array<HandPose, 2>& poses, const bool bSideValid[2],
						   const TrackingFrameResult::HeadPose& head, double timestampMs, const BodyDimensions& user,
						   const AvatarSkeleton& skeleton, const AvatarRetargetConfig& config, AvatarPose& outPose)
{
	SourcePose source;
	fillSource(poses, bSideValid, head, timestampMs, user, source);
	solve(source, skeleton, config, outPose);
}

void AvatarRetarget::solve(const TrackingFrameResult& frame, const BodyDimensions& user,
						   const AvatarSkeleton& skeleton, const AvatarRetargetConfig& config, AvatarPose& outPose)
{
	const bool bSideValid[2]= {
		frame.poses[0].tracked && frame.poses[0].hasWorldPose,
		frame.poses[1].tracked && frame.poses[1].hasWorldPose,
	};
	solve(frame.poses, bSideValid, frame.head, frame.timestampMs, user, skeleton, config, outPose);
}

void AvatarRetarget::resolveRoot(const SourcePose& source, const AvatarSkeleton& skeleton,
								 const AvatarRetargetConfig& config, AvatarPose& outPose)
{
	const double stepMs= source.clock.stepMs;
	const bool bBothShoulders= source.torso.state != SourcePose::eGroupState::Lost;

	if (config.followShoulders && bBothShoulders)
	{
		// The shoulder line fixes the yaw (at rest it runs along world +Y,
		// right to left) and the midpoint fixes the translation
		const glm::vec3 midpoint= (source.arms[0].shoulderPosition + source.arms[1].shoulderPosition) * 0.5f;
		const glm::vec3 line= source.arms[0].shoulderPosition - source.arms[1].shoulderPosition;
		float yaw= m_root.valid ? m_root.yawRadians : glm::radians(config.fixedRootYawDegrees);
		if (line.x * line.x + line.y * line.y > 1e-6f)
			yaw= atan2f(line.y, line.x) - glm::half_pi<float>();

		const glm::quat rotation= glm::angleAxis(yaw, glm::vec3(0.f, 0.f, 1.f));
		const glm::vec3 position= midpoint - rotation * skeleton.getRestShoulderMidpointWorld();

		// First-order follow on the step clock; a discontinuity (the clock
		// going backward) snaps rather than filtering across the jump
		const double dtSeconds= m_root.valid ? (stepMs - m_root.stepMs) * 0.001 : -1.0;
		if (dtSeconds < 0.0 || config.rootFollowTimeConstantS <= 0.f)
		{
			m_root.position= position;
			m_root.yawRadians= yaw;
		}
		else
		{
			const float alpha= 1.f - expf(-(float)std::min(dtSeconds, 1.0) / config.rootFollowTimeConstantS);
			m_root.position+= (position - m_root.position) * alpha;
			m_root.yawRadians= wrapAngle(m_root.yawRadians + wrapAngle(yaw - m_root.yawRadians) * alpha);
		}
		m_root.valid= true;
		m_root.stepMs= stepMs;
	}
	else if (!config.followShoulders)
	{
		m_root.valid= false;
	}

	if (m_root.valid)
	{
		outPose.rootPositionWorld= m_root.position;
		outPose.rootRotationWorld= glm::angleAxis(m_root.yawRadians, glm::vec3(0.f, 0.f, 1.f));
	}
	else
	{
		outPose.rootPositionWorld= config.fixedRootPositionWorld;
		outPose.rootRotationWorld= glm::angleAxis(glm::radians(config.fixedRootYawDegrees), glm::vec3(0.f, 0.f, 1.f));
	}
}

void AvatarRetarget::solve(const SourcePose& source, const AvatarSkeleton& skeleton,
						   const AvatarRetargetConfig& config, AvatarPose& outPose, AvatarRetargetGoals* outGoals)
{
	using eGroupState= SourcePose::eGroupState;
	if (outGoals != nullptr)
		*outGoals= AvatarRetargetGoals();

	// The root follow belongs to one epoch of the source clock
	if (source.clock.resetEpoch != m_solveEpoch)
	{
		m_root= RootState();
		m_solveEpoch= source.clock.resetEpoch;
	}

	outPose.clear();
	outPose.valid= true;
	outPose.timestampMs= source.clock.timestampMs;
	resolveRoot(source, skeleton, config, outPose);

	RootedRest rest;
	rest.skeleton= &skeleton;
	rest.position= outPose.rootPositionWorld;
	rest.rotation= outPose.rootRotationWorld;

	// The hips carry the root placement into the node hierarchy; everything
	// not posed below follows them at rest
	outPose.bones[(int)B::Hips].present= true;

	// Head: its measured frame is +X facing, +Y left, +Z up, which is the
	// avatar's rest frame, so the measured orientation IS the posed world
	// rotation of the canonical head frame and the delta is what remains
	// after the root yaw
	if (source.head.state != eGroupState::Lost && skeleton.getBone(B::Head).present)
	{
		AvatarPose::Bone& headBone= outPose.bones[(int)B::Head];
		headBone.present= true;
		const glm::quat headTrim= glm::quat(config.headTrimRadians);
		headBone.deltaWorld= source.head.orientation * glm::inverse(rest.rotation * headTrim);
	}

	const bool bBothShoulders= source.torso.state != eGroupState::Lost;
	const glm::vec3 measuredShoulderMidpoint=
		bBothShoulders ? (source.arms[0].shoulderPosition + source.arms[1].shoulderPosition) * 0.5f : glm::vec3(0.f);

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SourcePose::Arm& arm= source.arms[sideIndex];
		const SourcePose::Hand& sourceHand= source.hands[sideIndex];
		if (arm.state == eGroupState::Lost)
			continue;
		const eHandSide side= (eHandSide)sideIndex;
		const AvatarRetargetConfig::Side& sideConfig= config.sides[sideIndex];

		const B shoulderBone= sideBone(sideIndex, B::LeftShoulder, B::RightShoulder);
		const B upperArmBone= sideBone(sideIndex, B::LeftUpperArm, B::RightUpperArm);
		const B lowerArmBone= sideBone(sideIndex, B::LeftLowerArm, B::RightLowerArm);
		const B handBone= sideBone(sideIndex, B::LeftHand, B::RightHand);

		// Clavicle: measured against the line between the shoulders, so it
		// needs both; with one shoulder there is no reference for a shrug
		glm::quat chainDelta(1.f, 0.f, 0.f, 0.f);
		glm::vec3 avatarShoulder;
		if (skeleton.getBone(shoulderBone).present)
		{
			const glm::vec3 clavicleBase= rest.bonePosition(shoulderBone);
			if (bBothShoulders)
			{
				const glm::vec3 direction= safeNormalize(arm.shoulderPosition - measuredShoulderMidpoint);
				if (glm::dot(direction, direction) > 0.f)
				{
					chainDelta= swingTo(chainDelta, rest.direction(upperArmBone), direction);
					AvatarPose::Bone& clavicle= outPose.bones[(int)shoulderBone];
					clavicle.present= true;
					clavicle.deltaWorld= chainDelta;
				}
			}
			avatarShoulder= clavicleBase + chainDelta * rest.offset(upperArmBone);
		}
		else
		{
			avatarShoulder= rest.bonePosition(upperArmBone);
		}

		// The wrist target: the user's reach from their shoulder, scaled onto
		// the avatar's arm. Without a measured shoulder the avatar's own
		// stands in, which keeps the hand where it was seen.
		const float upperArm= skeleton.getUpperArmLength(side);
		const float forearm= skeleton.getForearmLength(side);
		const float avatarReach= upperArm + forearm;
		const float userReach= source.upperArmLength + source.forearmLength;
		const float ratio= userReach > 1e-3f ? avatarReach / userReach : 1.f;

		const glm::vec3 userShoulder= arm.bHasShoulder ? arm.shoulderPosition : avatarShoulder;
		const glm::vec3 userWrist= arm.wristPosition;
		glm::vec3 wrist= avatarShoulder + (userWrist - userShoulder) * ratio;
		if (outGoals != nullptr)
		{
			outGoals->bHasWristGoal[sideIndex]= true;
			outGoals->wristGoalWorld[sideIndex]= wrist;
		}
		const glm::vec3 toWrist= wrist - avatarShoulder;
		const float reach= glm::length(toWrist);
		if (reach > avatarReach && reach > 1e-6f)
			wrist= avatarShoulder + toWrist * (avatarReach / reach);

		// Elbow: the measured elbow, scaled the same way, hints which way the
		// arm bends, giving way to the rig's hint point (in the rooted torso
		// frame, so it turns with the body) as the forearm confidence drops
		const glm::vec3 hintOffset= rest.rotation * sideConfig.elbowHintOffset;
		glm::vec3 polePoint= avatarShoulder + hintOffset;
		if (arm.bHasElbow)
		{
			const glm::vec3 userElbow= arm.elbowPosition;
			const glm::vec3 measuredPole= avatarShoulder + (userElbow - userShoulder) * ratio;
			const float weight= sideConfig.elbowHintConfidence > 0.f
				? std::clamp(arm.forearmConfidence.value / sideConfig.elbowHintConfidence, 0.f, 1.f)
				: 1.f;
			polePoint+= (measuredPole - polePoint) * weight;
		}
		const glm::vec3 elbow= solveElbow(avatarShoulder, wrist, upperArm, forearm, polePoint, safeNormalize(hintOffset),
										  rest.direction(lowerArmBone));

		// Upper arm: direction only
		const glm::vec3 upperDirection= safeNormalize(elbow - avatarShoulder);
		if (glm::dot(upperDirection, upperDirection) > 0.f)
			chainDelta= swingTo(chainDelta, rest.direction(lowerArmBone), upperDirection);
		{
			AvatarPose::Bone& bone= outPose.bones[(int)upperArmBone];
			bone.present= true;
			bone.deltaWorld= chainDelta;
		}

		// Forearm: direction from the solve, roll from the measured forearm
		// (the IMU or the body solver) or, with none, from the palm, which
		// puts the wrist at zero bend rather than inventing a twist
		const AvatarSkeleton::HandRest& hand= skeleton.getHand(side);
		const glm::quat restPalmRotation= glm::quat_cast(glm::mat3(hand.palmFrameWorld));
		{
			glm::vec3 forearmDirection= safeNormalize(wrist - elbow);
			if (glm::dot(forearmDirection, forearmDirection) <= 0.f)
				forearmDirection= upperDirection;
			chainDelta= swingTo(chainDelta, rest.direction(handBone), forearmDirection);

			{
				const glm::quat measuredForearm=
					arm.bHasForearmOrientation ? arm.forearmOrientation : sourceHand.palmOrientation;
				const glm::vec3 measuredPalmNormal=
					safeNormalize(projectPerpendicular(measuredForearm * glm::vec3(0.f, 0.f, 1.f), forearmDirection));
				const glm::vec3 restPalmNormal= safeNormalize(projectPerpendicular(
					chainDelta * (rest.rotation * glm::vec3(hand.palmFrameWorld[2])), forearmDirection));
				if (glm::dot(measuredPalmNormal, measuredPalmNormal) > 0.f && glm::dot(restPalmNormal, restPalmNormal) > 0.f)
				{
					const float twist= signedAngle(restPalmNormal, measuredPalmNormal, forearmDirection) +
						sideConfig.forearmRollTrimRadians;
					chainDelta= glm::angleAxis(twist, forearmDirection) * chainDelta;
				}
			}
			AvatarPose::Bone& bone= outPose.bones[(int)lowerArmBone];
			bone.present= true;
			bone.deltaWorld= chainDelta;
		}
		const glm::vec3 handPosition= elbow + chainDelta * rest.offset(handBone);

		// Hand: the full measured palm frame against the avatar's rest palm
		// The trim rotates the rest palm the measurement is matched to, in its
		// own axes: a measured palm equal to the trimmed one leaves the hand
		// at rest
		const glm::quat assumedRestPalmRotation= restPalmRotation * glm::quat(sideConfig.handTrimRadians);
		const glm::quat handDelta= sourceHand.palmOrientation * glm::inverse(rest.rotation * assumedRestPalmRotation);
		{
			AvatarPose::Bone& bone= outPose.bones[(int)handBone];
			bone.present= true;
			bone.deltaWorld= handDelta;
		}

		solveFingerBones(skeleton, sideIndex, rest.rotation, handDelta, handPosition, sourceHand.fingers, sideConfig,
						 outPose);
	}
}

void computePosedBones(const AvatarSkeleton& skeleton, const AvatarPose& pose,
					   std::array<AvatarPosedBone, HUMANOID_BONE_COUNT>& outBones)
{
	// Enum order lists every parent before its children, so one pass composes
	// the chain
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		const AvatarSkeleton::Bone& restBone= skeleton.getBone((B)index);
		AvatarPosedBone& posed= outBones[index];
		if (!restBone.present)
		{
			posed= AvatarPosedBone();
			continue;
		}

		const glm::quat& delta= pose.bones[index].deltaWorld;
		posed.rotationWorld= delta * pose.rootRotationWorld * restBone.restRotationWorld;
		if (restBone.parent == HUMANOID_BONE_NONE)
		{
			posed.positionWorld= pose.rootPositionWorld + pose.rootRotationWorld * restBone.restPositionWorld;
		}
		else
		{
			const AvatarPosedBone& parent= outBones[(int)restBone.parent];
			const glm::quat& parentDelta= pose.bones[(int)restBone.parent].deltaWorld;
			posed.positionWorld=
				parent.positionWorld + parentDelta * (pose.rootRotationWorld * restBone.restOffsetFromParentWorld);
		}
	}
}

namespace
{
void composePosedGlobals(const AvatarModel& model, const std::vector<glm::mat4>& restLocals, int node,
						 const glm::mat4& parentGlobal, const std::vector<const glm::mat4*>& overrides,
						 std::vector<glm::mat4>& outGlobals)
{
	outGlobals[node]= overrides[node] != nullptr ? *overrides[node] : parentGlobal * restLocals[node];
	for (int child : model.nodes[node].children)
	{
		if (child >= 0 && child < (int)model.nodes.size())
			composePosedGlobals(model, restLocals, child, outGlobals[node], overrides, outGlobals);
	}
}
} // namespace

void computePosedGlobals(const AvatarModel& model, const AvatarSkeleton& skeleton, const AvatarPose& pose,
						 std::vector<glm::mat4>& outGlobals)
{
	if (!pose.valid)
	{
		outGlobals= skeleton.getRestGlobalsAvatar();
		return;
	}

	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posedBones;
	computePosedBones(skeleton, pose, posedBones);

	// Present bones override their node's global (back in avatar space);
	// everything else composes from its parent as authored
	const glm::mat4 avatarFromWorld= glm::inverse(skeleton.getWorldFromAvatar());
	std::array<glm::mat4, HUMANOID_BONE_COUNT> overrideStorage;
	std::vector<const glm::mat4*> overrides(model.nodes.size(), nullptr);
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		const AvatarSkeleton::Bone& restBone= skeleton.getBone((B)index);
		if (!restBone.present || !pose.bones[index].present)
			continue;
		glm::mat4 posedWorld= glm::mat4_cast(posedBones[index].rotationWorld);
		posedWorld[3]= glm::vec4(posedBones[index].positionWorld, 1.f);
		overrideStorage[index]= avatarFromWorld * posedWorld;
		if (restBone.node >= 0 && restBone.node < (int)model.nodes.size())
			overrides[restBone.node]= &overrideStorage[index];
	}

	outGlobals.assign(model.nodes.size(), glm::mat4(1.f));
	for (int root : model.rootNodes)
		composePosedGlobals(model, skeleton.getRestLocalsAvatar(), root, glm::mat4(1.f), overrides, outGlobals);
}
