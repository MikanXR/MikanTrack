#pragma once

#include <functional>
#include <memory>

#include "AvatarRetarget.h"
#include "BodyPoseSolver.h" // BodyDimensions
#include "DirectRetarget.h"
#include "LossStage.h"
#include "SourcePose.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

// The retarget stage: one frame of either pipeline through the source fill,
// the loss stage and the target solve, in that order. The vision thread runs
// one on every output frame and streams what it produces, so the window,
// which shows that same output live, shows what is streamed. The window runs
// its own over a replay and for the preview poses, and the tools run one.

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
	// weight 1 the position solve (AvatarRetarget::solve), at 0 the direct
	// placement (DirectRetarget::place). Nothing blends between the two yet,
	// so any other weight leaves the pose invalid. outGoals, when given,
	// receives the position solve's wrist goals, and none otherwise.
	void solveTarget(const SourcePose& source, float armIkWeight, AvatarPose& outPose,
					 AvatarRetargetGoals* outGoals= nullptr);

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
