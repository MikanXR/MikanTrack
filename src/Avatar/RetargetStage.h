#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>

#include "AvatarRetarget.h"
#include "BodyPoseSolver.h" // BodyDimensions
#include "DirectRetarget.h"
#include "ExprParameter.h"
#include "LossStage.h"
#include "RetargetChains.h" // makeDefaultElbowWeight
#include "SourcePose.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

// The retarget stage: one frame of either pipeline through the source fill,
// the loss stage and the target solve, in that order. The vision thread runs
// one on every output frame and streams what it produces, so the window,
// which shows that same output live, shows what is streamed. The window runs
// its own over a replay and for the preview poses, and the tools run one.

// One arm's settings from its chain that the position solve reads: the
// parameters that are a value or a function, each evaluated per frame over
// the source pose the loss stage handed on, and the goal offset as stored.
// The direct placement reads none of them.
struct RetargetArmSettings
{
	// The weight toward the measured elbow, 0 to 1
	ExprParameter elbowWeight{1.f};
	// The scale on the user's reach, 0 to 1: 1 scales it onto the avatar's
	// arm, 0 keeps it in meters
	ExprParameter reachScale{1.f};
	// Added to the wrist goal, meters, in the rooted torso frame
	glm::vec3 goalOffset{0.f};
};

struct RetargetStageConfig
{
	// The avatar the target solve poses; null poses nothing, while the
	// position pipeline still resolves its hands for the measured-length
	// stream. A change of skeleton restarts the direct solver and the
	// position root follow, and nothing else.
	std::shared_ptr<const AvatarSkeleton> skeleton;
	// The position pipeline's solve and the user's lengths its source is
	// measured in
	AvatarRetargetConfig position;
	BodyDimensions user;
	// The direct pipeline's solve
	DirectRetargetConfig direct;
	// Each pipeline's loss rules
	LossPolicy positionLoss;
	LossPolicy directLoss;
	// Each arm's settings from its chain (by eHandSide), its elbow weight
	// defaulting to the function of the rule the solve always applied
	std::array<RetargetArmSettings, 2> arms{
		RetargetArmSettings{makeDefaultElbowWeight(0, k_defaultElbowHintConfidence)},
		RetargetArmSettings{makeDefaultElbowWeight(1, k_defaultElbowHintConfidence)},
	};
};

// What one arm parameter evaluated to on a frame, for display
struct RetargetParameterReadout
{
	// The position solve ran and took the value; false on a direct frame and
	// without a skeleton, with the rest zero
	bool bEvaluated= false;
	// What the solve took, clamped to the parameter's range
	float value= 0.f;
	// The function's value before the clamp, 0 while the plain value is in
	// force
	float rawValue= 0.f;
	// The setting is a function, and its text compiled; the function decides
	// the value when both hold, and the plain value otherwise
	bool bExpression= false;
	bool bCompiled= false;
};

// What each of one arm's parameters evaluated to on a frame
struct RetargetArmReadout
{
	RetargetParameterReadout elbowWeight;
	RetargetParameterReadout reachScale;
};

// Each arm's parameters evaluated at one site over a stream of source poses:
// one runner per parameter per arm, their stateful calls stepping on the
// source clock's step time, their state starting over on a reset and when
// the source's epoch changes. A new text recompiles; the same text keeps its
// state.
class ArmParameterEvaluator
{
public:
	ArmParameterEvaluator();

	void setArms(const std::array<RetargetArmSettings, 2>& arms);
	void reset();

	// Both arms' settings for the position solve over a source pose, and
	// what each parameter evaluated to
	void evaluate(const SourcePose& source, std::array<AvatarRetargetArm, 2>& outArms,
				  std::array<RetargetArmReadout, 2>& outReadout);

private:
	// One arm's evaluation sites and its stored goal offset
	struct Arm
	{
		ExprParameterRunner elbowWeight;
		ExprParameterRunner reachScale;
		glm::vec3 goalOffset{0.f};
	};
	std::array<Arm, 2> m_arms;
	bool m_bHasEpoch= false;
	uint32_t m_epoch= 0;
};

// One frame through the stage
struct RetargetResult
{
	// The source pose as the loss stage handed it on, each group's state
	// saying what it carries
	SourcePose source;
	// The posed avatar, invalid without a skeleton
	AvatarPose pose;
	// The position pipeline's hands as the loss stage resolved them, what
	// the measured-length stream builds its bones from with no avatar
	LossHands hands;
	// The direct solve behind the pose, its arms' rest readout taken from
	// the source pose
	bool bDirectState= false;
	DirectRetarget::SolveState directState;
	// Where the position solve aimed each arm's wrist, for display; none on a
	// direct frame, whose placement has no goal
	AvatarRetargetGoals goals;
	// What each arm's parameters evaluated to for the position solve, by
	// eHandSide; not evaluated on a direct frame
	std::array<RetargetArmReadout, 2> armParameters{};
};

class RetargetStage
{
public:
	void setConfig(const RetargetStageConfig& config);
	const RetargetStageConfig& getConfig() const { return m_config; }

	// Starts both pipelines over: new epochs on their source clocks, so every
	// filter, hold, freeze, ease and root follow begins again
	void reset();

	// A fused (or camera-space passthrough) frame through the position
	// pipeline: AvatarRetarget::fillSource, the loss stage, then the target
	// solve at IK weight 1
	void solvePosition(const TrackingFrameResult& frame, RetargetResult& outResult);

	// One camera's frame through the direct pipeline: DirectRetarget::measure,
	// the loss stage, then the target solve at IK weight 0. phoneHead is
	// valid while the phone drives the head. False, with nothing solved and
	// an invalid pose, without a skeleton, which the measurement needs.
	bool solveDirect(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
					 const TrackingFrameResult::HeadPose& phoneHead, double timestampMs, RetargetResult& outResult);

	// The arm chains' IK weight, fixed by the pipeline that filled the
	// source: 1 for the position pipeline, 0 for the direct one
	static float getArmIkWeight(SourcePose::ePipeline pipeline);

	// The target solve over a source pose the loss stage handed on: at IK
	// weight 1 the position solve (AvatarRetarget::solve) with each arm's
	// parameters evaluated over the source, each resting arm's bones then
	// blended by its rest blend into the direct pipeline's rest pose
	// (DirectRetarget::makeRestPose) with no wrist goal, at 0 the direct
	// placement (DirectRetarget::place). Nothing blends between the two yet,
	// so any other weight leaves the pose invalid. outGoals and
	// outArmParameters, when given, receive the position solve's wrist goals
	// and what each arm's parameters evaluated to, and none otherwise.
	void solveTarget(const SourcePose& source, float armIkWeight, AvatarPose& outPose,
					 AvatarRetargetGoals* outGoals= nullptr,
					 std::array<RetargetArmReadout, 2>* outArmParameters= nullptr);

	// The target solve again over a result's source pose under the current
	// config, so a rig edit shows on a frame that is not solved again. The
	// fill and the loss stage do not run.
	void resolveTarget(RetargetResult& ioResult);

	// The direct solve state's rest readout from the source pose the loss
	// stage handed on: which arms rest and how far into the rest pose
	static void writeDirectRestReadout(const SourcePose& source, DirectRetarget::SolveState& ioState);

	// The direct solver, for its chains and the hand lift's depth scale
	const DirectRetarget& getDirectRetarget() const { return m_direct; }
	// The direct solver's current filtered measurements as a neutral pose
	bool captureNeutral(DirectNeutralPose& outNeutral) const { return m_direct.captureNeutral(outNeutral); }

private:
	RetargetStageConfig m_config;
	AvatarRetarget m_position;
	DirectRetarget m_direct;
	LossStage m_loss;
	// This stage's own evaluation of the arm parameters, so every stage (the
	// vision thread's, the window's, a tool's) keeps its own stateful calls
	ArmParameterEvaluator m_armParameters;
};

// Brings a stage to a frame of a replayed timeline. A forward step over at
// most kPreRollMs of recorded time plays every frame between, as a live run
// would have; anything else (another timeline, a step back, a longer jump)
// is a scrub, which resets the stage and plays the frames of the kPreRollMs
// before the target first, so the holds, the filters and the root follow
// have state when the target is solved.
class RetargetPlayhead
{
public:
	static constexpr double kPreRollMs= 1000.0;

	// The frames' recorded times, and one frame through the stage into the
	// result
	using FrameTime= std::function<double(int frameIndex)>;
	using SolveFrame= std::function<void(int frameIndex, RetargetStage& stage, RetargetResult& ioResult)>;

	// Forgets the frame last solved, so the next seek pre-rolls
	void invalidate();

	// True when it solved anything; a seek to the frame last solved on the
	// same timeline solves nothing
	bool seek(RetargetStage& stage, int timelineId, int frameIndex, const FrameTime& frameTimeMs,
			  const SolveFrame& solveFrame, RetargetResult& ioResult);

	// How many frames the last seek solved, the target included
	int getLastSolvedCount() const { return m_lastSolvedCount; }

private:
	int m_timelineId= -1;
	int m_frameIndex= -1;
	int m_lastSolvedCount= 0;
};
