#include "RetargetStage.h"

#include "AvatarSkeleton.h"
#include "SourcePoseInputs.h"

namespace
{
// One parameter at its site over the filled inputs, clamped to [minValue,
// maxValue], and what it evaluated to
float evaluateParameter(ExprParameterRunner& runner, const float* inputs, double timeSeconds, float minValue,
						float maxValue, RetargetParameterReadout& outReadout)
{
	const float value= runner.evaluate(inputs, SourcePoseInputs::k_count, timeSeconds, minValue, maxValue);
	outReadout.bEvaluated= true;
	outReadout.value= value;
	outReadout.bExpression= runner.getParameter().bUseExpression;
	outReadout.bCompiled= runner.getCompileError() == nullptr;
	outReadout.rawValue= runner.getLastUsedExpression() ? runner.getLastRawValue() : 0.f;
	return value;
}
} // namespace

ArmParameterEvaluator::ArmParameterEvaluator()
{
	setArms(RetargetStageConfig().arms);
}

void ArmParameterEvaluator::setArms(const std::array<RetargetArmSettings, 2>& arms)
{
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		Arm& arm= m_arms[sideIndex];
		arm.elbowWeight.setParameter(arms[sideIndex].elbowWeight, SourcePoseInputs::table());
		arm.reachScale.setParameter(arms[sideIndex].reachScale, SourcePoseInputs::table());
		arm.goalOffset= arms[sideIndex].goalOffset;
	}
}

void ArmParameterEvaluator::reset()
{
	for (Arm& arm : m_arms)
	{
		arm.elbowWeight.reset();
		arm.reachScale.reset();
	}
	m_bHasEpoch= false;
}

void ArmParameterEvaluator::evaluate(const SourcePose& source, std::array<AvatarRetargetArm, 2>& outArms,
									 std::array<RetargetArmReadout, 2>& outReadout)
{
	// The state belongs to one epoch of the source clock
	if (!m_bHasEpoch || source.clock.resetEpoch != m_epoch)
	{
		reset();
		m_bHasEpoch= true;
		m_epoch= source.clock.resetEpoch;
	}

	float inputs[SourcePoseInputs::k_count];
	SourcePoseInputs::fill(source, inputs);
	const double timeSeconds= source.clock.stepMs * 0.001;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		Arm& arm= m_arms[sideIndex];
		RetargetArmReadout& readout= outReadout[sideIndex];
		AvatarRetargetArm& out= outArms[sideIndex];
		out.elbowWeight= evaluateParameter(arm.elbowWeight, inputs, timeSeconds, 0.f, 1.f, readout.elbowWeight);
		out.reachScale= evaluateParameter(arm.reachScale, inputs, timeSeconds, 0.f, 1.f, readout.reachScale);
		out.goalOffset= arm.goalOffset;
	}
}

void RetargetStage::setConfig(const RetargetStageConfig& config)
{
	// A new skeleton means new lengths: the direct solver starts over on
	// them, and the root follow snaps to the new rest shoulders. The source
	// clocks of the position pipeline carry on, so its holds and freezes do.
	if (config.skeleton != m_config.skeleton)
	{
		m_position.resetRootFollow();
		m_direct.reset();
	}
	m_config= config;
	// A changed text recompiles; an unchanged one keeps its state
	m_armParameters.setArms(m_config.arms);
}

void RetargetStage::reset()
{
	m_position.reset();
	m_direct.reset();
	m_loss.reset();
	m_armParameters.reset();
}

float RetargetStage::getArmIkWeight(SourcePose::ePipeline pipeline)
{
	return pipeline == SourcePose::ePipeline::Direct ? 0.f : 1.f;
}

void RetargetStage::solvePosition(const TrackingFrameResult& frame, RetargetResult& outResult)
{
	m_position.fillSource(frame, m_config.user, outResult.source);
	outResult.hands.poses= frame.poses;
	m_loss.apply(m_config.positionLoss, m_config.skeleton.get(), m_config.position.sides, outResult.source,
				 &outResult.hands);
	solveTarget(outResult.source, getArmIkWeight(outResult.source.pipeline), outResult.pose, &outResult.goals,
				&outResult.armParameters);
	outResult.bDirectState= false;
	outResult.directState= DirectRetarget::SolveState();
}

bool RetargetStage::solveDirect(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
								const TrackingFrameResult::HeadPose& phoneHead, double timestampMs,
								RetargetResult& outResult)
{
	outResult= RetargetResult();
	if (m_config.skeleton == nullptr)
		return false;
	const AvatarSkeleton& skeleton= *m_config.skeleton;

	m_direct.measure(camera, cameraModel, phoneHead, timestampMs, skeleton, m_config.direct, outResult.source);
	m_loss.apply(m_config.directLoss, &skeleton, m_config.direct.sides, outResult.source, nullptr);
	solveTarget(outResult.source, getArmIkWeight(outResult.source.pipeline), outResult.pose, &outResult.goals,
				&outResult.armParameters);
	outResult.bDirectState= true;
	outResult.directState= m_direct.getSolveState();
	writeDirectRestReadout(outResult.source, outResult.directState);
	return true;
}

void RetargetStage::solveTarget(const SourcePose& source, float armIkWeight, AvatarPose& outPose,
								AvatarRetargetGoals* outGoals, std::array<RetargetArmReadout, 2>* outArmParameters)
{
	outPose.clear();
	if (outGoals != nullptr)
		*outGoals= AvatarRetargetGoals();
	if (outArmParameters != nullptr)
		*outArmParameters= {};
	if (m_config.skeleton == nullptr)
		return;
	if (armIkWeight == 1.f)
	{
		std::array<AvatarRetargetArm, 2> arms;
		std::array<RetargetArmReadout, 2> readout;
		m_armParameters.evaluate(source, arms, readout);
		if (outArmParameters != nullptr)
			*outArmParameters= readout;
		m_position.solve(source, *m_config.skeleton, m_config.position, arms, outPose, outGoals);
	}
	else if (armIkWeight == 0.f)
		m_direct.place(source, *m_config.skeleton, m_config.direct, outPose);
}

void RetargetStage::resolveTarget(RetargetResult& ioResult)
{
	if (ioResult.source.pipeline == SourcePose::ePipeline::None)
		return;
	solveTarget(ioResult.source, getArmIkWeight(ioResult.source.pipeline), ioResult.pose, &ioResult.goals,
				&ioResult.armParameters);
	// The placement writes the head's delta into the solve state
	if (ioResult.bDirectState)
		ioResult.directState.headDelta= m_direct.getSolveState().headDelta;
}

void RetargetStage::writeDirectRestReadout(const SourcePose& source, DirectRetarget::SolveState& ioState)
{
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SourcePose::Arm& arm= source.arms[sideIndex];
		const bool bResting= arm.state == SourcePose::eGroupState::Resting;
		ioState.arms[sideIndex].bResting= bResting;
		ioState.arms[sideIndex].restBlend= bResting ? arm.restBlend : 0.f;
	}
}

void RetargetPlayhead::invalidate()
{
	m_timelineId= -1;
	m_frameIndex= -1;
}

bool RetargetPlayhead::seek(RetargetStage& stage, int timelineId, int frameIndex, const FrameTime& frameTimeMs,
							const SolveFrame& solveFrame, RetargetResult& ioResult)
{
	m_lastSolvedCount= 0;
	if (frameIndex < 0)
		return false;
	if (timelineId == m_timelineId && frameIndex == m_frameIndex)
		return false;

	const double targetMs= frameTimeMs(frameIndex);
	int firstFrame= frameIndex;
	const bool bStep= timelineId == m_timelineId && m_frameIndex >= 0 && frameIndex > m_frameIndex &&
		targetMs - frameTimeMs(m_frameIndex) <= kPreRollMs;
	if (bStep)
	{
		firstFrame= m_frameIndex + 1;
	}
	else
	{
		stage.reset();
		while (firstFrame > 0 && targetMs - frameTimeMs(firstFrame - 1) <= kPreRollMs)
			--firstFrame;
	}

	for (int index= firstFrame; index <= frameIndex; ++index)
		solveFrame(index, stage, ioResult);
	m_lastSolvedCount= frameIndex - firstFrame + 1;
	m_timelineId= timelineId;
	m_frameIndex= frameIndex;
	return true;
}
