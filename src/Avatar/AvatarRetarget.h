#pragma once

#include <array>
#include <vector>

#include "glm/ext/matrix_float4x4.hpp"
#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/vector_float3.hpp"

#include "AvatarTypes.h"
#include "BodyPoseSolver.h" // BodyDimensions
#include "SourcePose.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

// Retargets the measured world-space pose onto a loaded avatar's humanoid
// skeleton. The avatar keeps its own proportions: every bone keeps its rest
// offset from its parent and only rotates, so the measured wrist is first
// scaled from the user's arm onto the avatar's (by the arm's reach scale),
// the elbow comes from a
// two-bone solve over the avatar's arm lengths hinted by the measured elbow,
// and the fingers are rebuilt through the shipping forward kinematics on the
// avatar's own hand skeleton. Pure math over the frame it is given plus a
// small filter state for the root, advanced on frame timestamps, so the same
// frames replay to the same poses.
struct AvatarRetargetConfig
{
	// Slide and yaw the avatar so its shoulders sit on the measured ones
	// whenever the body-pose stage tracks both. Held at the last followed
	// placement across a shoulder dropout.
	bool followShoulders= true;
	// Placement while the shoulders are not followed: where the avatar's
	// origin sits in the world frame, and its yaw about world +Z (0 = facing
	// world +X, the rest-pose facing)
	glm::vec3 fixedRootPositionWorld{0.f};
	float fixedRootYawDegrees= 0.f;
	// Time constant of the root follow filter, seconds. The measured
	// shoulders carry the body solver's jitter, which would otherwise shake
	// the whole character.
	float rootFollowTimeConstantS= 0.3f;

	// Per arm and hand, left then right (copied from the avatar's rig
	// settings, see AvatarRig.h). The weight toward the measured elbow is
	// not here: it is evaluated per frame and handed to the solve.
	struct Side
	{
		// The point the elbow bends toward when nothing measured it, meters,
		// in the rooted torso frame relative to the avatar's upper-arm joint,
		// so it turns with the root yaw. Down and back, the way a resting arm
		// hangs.
		glm::vec3 elbowHintOffset{-0.15f, 0.f, -0.30f};
		// Trims on the rest frames the retarget assumes, radians: the palm
		// frame in its own axes (XYZ Euler), and a roll about the forearm
		glm::vec3 handTrimRadians{0.f};
		float forearmRollTrimRadians= 0.f;
		// Finger angle scales: curl on the three bends, splay on the lateral
		float curlGain= 1.f;
		float splayGain= 1.f;
		std::array<bool, FINGER_COUNT> fingerEnabled{true, true, true, true, true};
		// Explicit signed thumb hinge pronation; otherwise the hand
		// skeleton's own (HandPoseModel::getThumbPronationRad)
		bool bThumbPronationOverride= false;
		float thumbPronationRad= 0.f;
		// The arm's rest pose (the direct retarget's, for an arm whose
		// tracking is lost): the elbow and the wrist relative to the upper-arm
		// joint in the rooted torso frame, meters, of which only the
		// directions are used; the direction the palm faces; the finger
		// curl, radians
		glm::vec3 restElbowOffset{0.05f, 0.f, -0.28f};
		glm::vec3 restWristOffset{0.28f, 0.f, -0.34f};
		glm::vec3 restPalmNormal{0.f, 0.f, -1.f};
		float restFingerCurlRadians= 0.f;
	};
	std::array<Side, 2> sides{};
	// Trim on the head's assumed rest frame (+X facing, +Y left, +Z up), in
	// its own axes, XYZ Euler radians
	glm::vec3 headTrimRadians{0.f};
};

// The posed avatar: a root placement plus one world rotation DELTA per
// humanoid bone, measured against the bone's rooted rest rotation. A bone's
// posed world rotation is `delta * rootRotation * restRotationWorld`, its
// posed position hangs off its humanoid parent's posed position by the
// parent's delta applied to the rest offset, and an absent bone (present ==
// false) sits at identity delta, following its parent. Defined this way so
// the renderer, the VMC stream, and a test can each compose it the same way.
struct AvatarPose
{
	bool valid= false;
	double timestampMs= 0.0;
	glm::vec3 rootPositionWorld{0.f};
	glm::quat rootRotationWorld{1.f, 0.f, 0.f, 0.f};

	struct Bone
	{
		bool present= false;
		glm::quat deltaWorld{1.f, 0.f, 0.f, 0.f};
	};
	std::array<Bone, HUMANOID_BONE_COUNT> bones{};

	void clear();
};

// Where an arm solve aimed each arm's wrist, for display: the source's
// shoulder-to-wrist offset placed off the avatar's shoulder by the arm's
// reach scale, plus the arm's goal offset, before the clamp to the avatar's
// reach, in the world the pose is posed in. Set for every arm solved at an
// IK weight above 0, in either pipeline. Beside it, the IK weight each posed
// arm was solved at. Nothing reads it back into a pose or the stream.
struct AvatarRetargetGoals
{
	std::array<bool, 2> bHasWristGoal{};       // by eHandSide
	std::array<glm::vec3, 2> wristGoalWorld{}; // by eHandSide
	std::array<bool, 2> bPosed{};              // by eHandSide
	std::array<float, 2> ikWeight{};           // by eHandSide
};

// One arm's settings from its chain that an arm solve takes per frame
struct AvatarRetargetArm
{
	// How far the wrist is pulled from the angle-mapped arm toward the goal,
	// [0, 1]: 0 is the angle mapping alone (the avatar's bones take the
	// source bones' directions), 1 the IK solve alone. Zero for an arm mapped
	// by bone angles.
	float ikWeight= 1.f;
	// How far the pole moves from the rig's hint toward the measured elbow,
	// [0, 1]: at 1 the measured elbow alone decides the bend, at 0 the hint
	// alone
	float elbowWeight= 1.f;
	// The factor on the measured shoulder-to-wrist offset, [0, 1]: at 1 the
	// user's reach is scaled onto the avatar's arm (the avatar-to-user arm
	// length ratio), at 0 the offset is kept in meters, and between them the
	// factor blends linearly from 1 to the ratio
	float reachScale= 1.f;
	// Added to the wrist goal after the scaling and before the clamp to the
	// avatar's reach, meters, in the rooted torso frame (+X facing, +Y the
	// avatar's left, +Z up), so it turns with the root yaw
	glm::vec3 goalOffset{0.f};
};

// The factor the solve puts on the measured shoulder-to-wrist offset for a
// reach scale (clamped to [0, 1]) and the avatar-to-user arm length ratio:
// exactly the ratio at 1 and exactly 1 at 0
float reachFactor(float reachScale, float armLengthRatio);

// The forearm confidence at and above which the elbow rule the avatar's rig
// has always defaulted to lets the measured elbow alone decide the bend
constexpr float k_defaultElbowHintConfidence= 0.5f;

// The elbow rule the solve applied before the weight was a setting: the
// weight toward the measured elbow is the forearm confidence over the
// threshold, clamped to [0, 1], and 1 for a threshold of zero or less. What
// the default elbow weight function computes, and what the solve entry
// points that take a threshold apply.
float elbowWeightFromConfidence(float forearmConfidence, float elbowHintConfidence);

class AvatarRetarget
{
public:
	// Forgets the root follow state, so the next solve snaps to its
	// measurement instead of filtering from a stale placement, and starts a
	// new epoch on the source pose clock
	void reset();

	// The source pose of a frame: a copy of exactly what the solve reads.
	// bSideValid selects which sides carry an arm and a hand, each written as
	// measured; the torso is measured while both valid sides carry a
	// shoulder. Owns the clock: the frame time, the step clock that holds
	// across a small backward step, the interval since the last fill, and
	// the epoch reset() advances.
	void fillSource(const std::array<HandPose, 2>& poses, const bool bSideValid[2],
					const TrackingFrameResult::HeadPose& head, double timestampMs, const BodyDimensions& user,
					SourcePose& outSource);

	// The source pose of a fused frame as tracking delivered it, before any
	// loss rule: a side tracked with a world pose reads as measured, and a
	// side tracked without one reads as lost with its hand's confidence
	// present, so the loss stage can tell a hand seen but not placeable from
	// one not seen at all
	void fillSource(const TrackingFrameResult& frame, const BodyDimensions& user, SourcePose& outSource);

	// Forgets the root follow alone, so the next solve snaps to its
	// measurement, without starting a new epoch: what a new skeleton needs,
	// since the source's clock and everything that follows it carry on
	void resetRootFollow();

	// Poses the avatar from a position pipeline's source pose; an arm that
	// is lost stays at rest. Always produces a valid pose with a root,
	// tracked or not. The root follow restarts when the source's epoch
	// changes. arms (by eHandSide) holds each arm's IK weight, elbow weight,
	// reach scale and goal offset for this frame. At an IK weight of exactly
	// 1 an arm is the IK solve alone. Below it the arm starts from its angle
	// mapping, the avatar's upper arm and forearm along the source's shoulder
	// to elbow and elbow to wrist, which needs a measured shoulder and elbow:
	// an arm without them takes the IK, eased in and out over
	// kAngleMappingRampMs on the source's step clock from the directions it
	// last had, so an elbow that drops out for a frame does not snap the arm.
	// outGoals, when given, receives each posed arm's wrist goal and the
	// weight it was solved at.
	void solve(const SourcePose& source, const AvatarSkeleton& skeleton, const AvatarRetargetConfig& config,
			   const std::array<AvatarRetargetArm, 2>& arms, AvatarPose& outPose,
			   AvatarRetargetGoals* outGoals= nullptr);

	// fillSource then solve, each arm's elbow weight from its forearm
	// confidence by elbowWeightFromConfidence over the threshold given, at a
	// reach scale of 1 and no goal offset
	void solve(const std::array<HandPose, 2>& poses, const bool bSideValid[2],
			   const TrackingFrameResult::HeadPose& head, double timestampMs, const BodyDimensions& user,
			   const AvatarSkeleton& skeleton, const AvatarRetargetConfig& config, AvatarPose& outPose,
			   const std::array<float, 2>& elbowHintConfidence= {k_defaultElbowHintConfidence,
																 k_defaultElbowHintConfidence});

	// Convenience over a fused frame: a side is valid when it is tracked
	// with a world pose
	void solve(const TrackingFrameResult& frame, const BodyDimensions& user, const AvatarSkeleton& skeleton,
			   const AvatarRetargetConfig& config, AvatarPose& outPose,
			   const std::array<float, 2>& elbowHintConfidence= {k_defaultElbowHintConfidence,
																 k_defaultElbowHintConfidence});

	// How long an arm that cannot be angle mapped takes to hand its pose
	// over to the IK, and to take it back once it can, milliseconds
	static constexpr double kAngleMappingRampMs= 250.0;

private:
	void resolveRoot(const SourcePose& source, const AvatarSkeleton& skeleton, const AvatarRetargetConfig& config,
					 AvatarPose& outPose);

	// Advances one arm's angle mapping state over a carried arm and returns
	// the IK weight the arm is solved at: the chain's weight while the arm is
	// angle mapped, moving to 1 while it cannot be. outUpper and outForearm
	// receive the directions the angle mapping takes, world, unit, valid
	// whenever the returned weight is below 1.
	float advanceAngleMapping(int sideIndex, const SourcePose::Arm& arm, const glm::quat& rootRotation, double stepMs,
							  float ikWeight, glm::vec3& outUpper, glm::vec3& outForearm);

	// The source clock, owned by fillSource
	uint32_t m_resetEpoch= 0;
	bool m_bHasFillTime= false;
	double m_lastFillStepMs= 0.0;
	// The epoch the root follow state belongs to
	uint32_t m_solveEpoch= 0;

	struct RootState
	{
		bool valid= false;
		glm::vec3 position{0.f};
		float yawRadians= 0.f;
		double stepMs= 0.0;
	};
	RootState m_root;

	// Per arm, what its angle mapping last had: the source's directions in
	// the rooted frame, so held ones turn with the root, and how far the arm
	// is angle mapped, 1 with a measured shoulder and elbow and easing to 0
	// without. Belongs to the root follow's epoch.
	struct AngleMappingState
	{
		bool bValid= false;
		bool bHasDirections= false;
		glm::vec3 upperRooted{0.f};
		glm::vec3 forearmRooted{0.f};
		float availability= 0.f;
		double stepMs= 0.0;
	};
	std::array<AngleMappingState, 2> m_angleMapping{};
};

// One humanoid bone's posed world transform
struct AvatarPosedBone
{
	glm::vec3 positionWorld{0.f};
	glm::quat rotationWorld{1.f, 0.f, 0.f, 0.f};
};

// Composes the pose over the rest skeleton: every bone's posed world position
// and rotation (absent bones follow their parents at rest)
void computePosedBones(const AvatarSkeleton& skeleton, const AvatarPose& pose,
					   std::array<AvatarPosedBone, HUMANOID_BONE_COUNT>& outBones);

// Node globals (avatar space) for rendering: present humanoid bones take
// their posed transform, every other node composes from its parent as
// authored, so helper nodes, hair and clothing bones ride along
void computePosedGlobals(const AvatarModel& model, const AvatarSkeleton& skeleton, const AvatarPose& pose,
						 std::vector<glm::mat4>& outGlobals);
