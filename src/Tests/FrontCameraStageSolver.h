#pragma once

#include <memory>

#include "LossStage.h"
#include "RetargetChains.h"
#include "RetargetStage.h"

// A retarget stage driven one camera frame at a time with the skeleton and
// the front-camera config passed on each call, the pose out: the shape the tests
// and the tools that solve front-camera poses frame by frame are written in. The
// stage runs the front-camera pipeline whole, measure, the loss stage under the
// front-camera rules (the earned hold, then the ease into rest over the config's
// rest blend) and the placement. The skeleton is the caller's and must
// outlive the solver's use of it.
class FrontCameraStageSolver
{
public:
	void reset() { m_stage.reset(); }

	void solve(const TrackingFrameResult& camera, const FrontCameraModel& cameraModel,
			   const TrackingFrameResult::HeadPose& phoneHead, double timestampMs, const AvatarSkeleton& skeleton,
			   const FrontCameraRetargetConfig& config, AvatarPose& outPose)
	{
		RetargetStageConfig stageConfig= m_stage.getConfig();
		// A pointer the stage does not own: a change of skeleton is still a
		// change of address, which is what the stage compares
		if (stageConfig.skeleton.get() != &skeleton)
			stageConfig.skeleton= std::shared_ptr<const AvatarSkeleton>(std::shared_ptr<const AvatarSkeleton>(), &skeleton);
		stageConfig.frontCamera= config;
		// The rules from the chains a front-camera project with this config runs
		// on, as the live code builds them
		const RetargetChains chains=
			migrateRetargetChains(true, 250.f, true, config.restBlendMs,
								  {k_defaultElbowHintConfidence, k_defaultElbowHintConfidence});
		stageConfig.frontCameraLoss= makeChainLossPolicy(chains, SourcePose::ePipeline::FrontCamera, 0.f, true);
		// and its arms' mapping, bone angles
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			stageConfig.arms[sideIndex].mapping= chains.arms[sideIndex].mapping;
			stageConfig.arms[sideIndex].ikWeight= chains.arms[sideIndex].ikWeight;
		}
		m_stage.setConfig(stageConfig);
		m_stage.solveFrontCamera(camera, cameraModel, phoneHead, timestampMs, m_result);
		outPose= m_result.pose;
	}

	// The front-camera solve state with the loss stage's rest readout
	const FrontCameraRetarget::SolveState& getSolveState() const { return m_result.frontCameraState; }
	const FrontCameraRetarget::ArmChain& getArmChain(int sideIndex) const
	{
		return m_stage.getFrontCameraRetarget().getArmChain(sideIndex);
	}
	bool captureNeutral(FrontCameraNeutralPose& outNeutral) const { return m_stage.captureNeutral(outNeutral); }

	const RetargetResult& getResult() const { return m_result; }
	const RetargetStage& getStage() const { return m_stage; }

private:
	RetargetStage m_stage;
	RetargetResult m_result;
};
