#pragma once

#include "nlohmann/json.hpp"

#include "DirectRetarget.h"

class AvatarSkeleton;
struct AvatarPose;

// JSON for the avatar side of a capture: the posed avatar and the direct
// solver's state behind it, read by the diagnostic dump and the pose
// report. Vectors are [x, y, z], quaternions [x, y, z, w], angles radians.
namespace AvatarJson
{
// The root, then every present bone by humanoid name: its world delta and
// its posed world position and rotation over the rest skeleton
nlohmann::json avatarPoseToJson(const AvatarSkeleton& skeleton, const AvatarPose& pose);
nlohmann::json directSolveStateToJson(const DirectRetarget::SolveState& state);
} // namespace AvatarJson
