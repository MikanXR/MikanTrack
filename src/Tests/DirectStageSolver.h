#pragma once

#include <memory>

#include "LossStage.h"
#include "RetargetChains.h"
#include "RetargetStage.h"

// A retarget stage driven one camera frame at a time with the skeleton and
// the direct config passed on each call, the pose out: the shape the tests
// and the tools that solve direct poses frame by frame are written in. The
// stage runs the direct pipeline whole, measure, the loss stage under the
// direct rules (the earned hold, then the ease into rest over the config's
// rest blend) and the placement. The skeleton is the caller's and must
// outlive the solver's use of it.
class DirectStageSolver
{
public:
	void reset() { m_stage.reset(); }

	void solve(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
			   const TrackingFrameResult::HeadPose& phoneHead, double timestampMs, const AvatarSkeleton& skeleton,
			   const DirectRetargetConfig& config, AvatarPose& outPose)
	{
		RetargetStageConfig stageConfig= m_stage.getConfig();
		// A pointer the stage does not own: a change of skeleton is still a
		// change of address, which is what the stage compares
		if (stageConfig.skeleton.get() != &skeleton)
			stageConfig.skeleton= std::shared_ptr<const AvatarSkeleton>(std::shared_ptr<const AvatarSkeleton>(), &skeleton);
		stageConfig.direct= config;
		// The rules from the chains a direct project with this config runs
		// on, as the live code builds them
		const RetargetChains chains=
			migrateRetargetChains(true, 250.f, true, config.restBlendMs,
								  {config.sides[0].elbowHintConfidence, config.sides[1].elbowHintConfidence});
		stageConfig.directLoss= makeChainLossPolicy(chains, SourcePose::ePipeline::Direct, 0.f);
		m_stage.setConfig(stageConfig);
		m_stage.solveDirect(camera, cameraModel, phoneHead, timestampMs, m_result);
		outPose= m_result.pose;
	}

	// The direct solve state with the loss stage's rest readout
	const DirectRetarget::SolveState& getSolveState() const { return m_result.directState; }
	const DirectRetarget::ArmChain& getArmChain(int sideIndex) const
	{
		return m_stage.getDirectRetarget().getArmChain(sideIndex);
	}
	DirectRetarget::PnpScale getPnpScale(int sideIndex) const { return m_stage.getDirectRetarget().getPnpScale(sideIndex); }
	bool captureNeutral(DirectNeutralPose& outNeutral) const { return m_stage.captureNeutral(outNeutral); }

	const RetargetResult& getResult() const { return m_result; }
	const RetargetStage& getStage() const { return m_stage; }

private:
	RetargetStage m_stage;
	RetargetResult m_result;
};
