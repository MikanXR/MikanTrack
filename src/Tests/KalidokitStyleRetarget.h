#pragma once

#include <array>

#include "glm/gtc/quaternion.hpp"

#include "AvatarRetarget.h"
#include "DirectRetarget.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

// The webcam vtuber tools' arm pattern (Kalidokit's), as the baseline the
// direct retarget is measured against in --gate-direct. The model's own 3D
// landmarks give every direction: the pixels lifted at the root depth the
// shoulder width implies, with the relative depth stretched by the measured
// compression, and each arm bone swung onto the lifted direction. The hand
// bone comes from the hand model's own palm plane (the wrist and the index
// and pinky knuckles with their relative depth), no lift. Every placed bone
// is slerped a fixed fraction toward its target each frame, the consumer-side
// smoothing those tools apply. No bone lengths, no holds: an unseen joint
// keeps its last bone. Test code, not shipped.
class KalidokitStyleRetarget
{
public:
	// The relative depth the whole-body model reports is this fraction of
	// the truth on the webcam view (the gate tool's "scale p50")
	static constexpr float kModelDepthScale= 0.6f;
	// The per-frame slerp fraction toward the target, Kalidokit's demo value
	static constexpr float kSlerp= 0.3f;

	// The root depth at which two shoulder rays (unit z) with the model's
	// relative depths sit the given width apart: |a * root + b| = width,
	// the positive root of the quadratic. Shared with the gate tool's lifts.
	static bool solveRootDepthFromShoulders(const glm::vec3& leftRay, float leftDepth, const glm::vec3& rightRay,
											 float rightDepth, float shoulderWidthMeters, float& outRootDepth);

	void reset();

	// Same shape as DirectStageSolver::solve, without the phone head
	void solve(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
			   const AvatarSkeleton& skeleton, const DirectRetargetConfig& config, AvatarPose& outPose);

private:
	// Every bone placed so far, the slerp's state; a bone unseen this frame
	// keeps it
	std::array<AvatarPose::Bone, HUMANOID_BONE_COUNT> m_bones{};
	// The finger measurement's palmar memory per hand, as the direct solver
	// keeps one
	std::array<HandPoseModel::PalmarSideMemory, 2> m_palmarMemory{};
};
