#pragma once

#include <array>

#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/vector_float3.hpp"

#include "AvatarRetarget.h"
#include "AvatarTypes.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

// Rotation helpers shared by the retargets that pose an AvatarPose: the
// position-based retarget over fused world poses and the direct retarget
// over one camera's landmarks. Every delta here is a WORLD rotation measured
// against a bone's rooted rest rotation, the AvatarPose convention.
namespace AvatarRetargetMath
{
glm::vec3 safeNormalize(const glm::vec3& v);

// Signed angle from a to b about axis (all unit length, a and b perpendicular
// to axis)
float signedAngle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& axis);

glm::vec3 projectPerpendicular(const glm::vec3& v, const glm::vec3& unitAxis);

float wrapAngle(float radians);

eHumanoidBone sideBone(int sideIndex, eHumanoidBone left, eHumanoidBone right);
eHumanoidBone fingerBone(int sideIndex, int finger, int phalanx);

// The elbow of a two-bone arm whose wrist is at the given point: on the
// circle where the upper-arm sphere about the shoulder meets the forearm
// sphere about the wrist, at the point nearest the pole, or along the
// fallback direction when the pole sits on the arm's axis. A wrist at or
// beyond full reach straightens the arm, so a caller clamps the reach first.
glm::vec3 solveTwoBoneElbow(const glm::vec3& shoulder, const glm::vec3& wrist, float upperArm, float forearm,
							const glm::vec3& polePoint, const glm::vec3& fallbackPoleDirection,
							const glm::vec3& restDirection);

// An arm part way from its angle mapping to its IK: the wrist is the
// angle-mapped wrist moved toward the goal (already clamped to the reach) by
// the IK weight, the pole the angle-mapped elbow moved toward the IK's pole
// by the same weight, and the elbow the two-bone solve between them. Both
// move together so the arm meets the angle mapping as the weight goes to 0
// and the IK as it goes to 1, whatever pole the IK takes.
void pullArmTowardGoal(const glm::vec3& shoulder, const glm::vec3& angleElbow, const glm::vec3& angleWrist,
					   const glm::vec3& goal, const glm::vec3& polePoint, float ikWeight, float upperArm, float forearm,
					   const glm::vec3& fallbackPoleDirection, const glm::vec3& restDirection, glm::vec3& outElbow,
					   glm::vec3& outWrist);

// A bone whose own twist is not measured: swing the parent-posed rest
// direction onto the measured one and chain the result
glm::quat swingTo(const glm::quat& parentDelta, const glm::vec3& rootedRestDirection, const glm::vec3& worldDirection);

// Splits a rotation into its twist about a unit axis and the swing that
// remains, with rotation = swing * twist
void swingTwist(const glm::quat& rotation, const glm::vec3& unitAxis, glm::quat& outSwing, glm::quat& outTwist);

// The same rotation with its angle capped at maxRadians about its own axis
glm::quat limitRotationAngle(const glm::quat& rotation, float maxRadians);

// Poses one hand's finger bones: the measured angles scaled by the rig's
// gains, folded into a finger with fewer bones than measured, played through
// the forward kinematics on the AVATAR's hand skeleton under the posed palm,
// then each physical phalanx swung from its rest direction relative to the
// bone before it, so the hand's roll carries down the chain. Fingers the rig
// disables, and fingers the avatar lacks, stay absent.
//
// handDelta is the hand bone's world delta, handPositionWorld its posed
// position, and rootRotation the pose's root rotation (identity when the
// root is fixed).
void solveFingerBones(const AvatarSkeleton& skeleton, int sideIndex, const glm::quat& rootRotation,
					  const glm::quat& handDelta, const glm::vec3& handPositionWorld,
					  const std::array<FingerAngles, FINGER_COUNT>& measuredAngles,
					  const AvatarRetargetConfig::Side& sideConfig, AvatarPose& outPose);
} // namespace AvatarRetargetMath
