#pragma once

// Frozen copy of src/Avatar/AvatarRetargetMath.h at commit a8ee475, in namespace Frozen: the reference the
// retarget golden test compares the live code against. Never edit it to follow
// a change in the live code. A deliberate change in behavior is recorded by the
// golden test stating the difference it expects.

#include <array>

#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/vector_float3.hpp"

#include "FrozenAvatarRetarget.h"
#include "AvatarTypes.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

namespace Frozen
{
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
} // namespace Frozen
