#include "LossStage.h"

#include <algorithm>

#include "AvatarSkeleton.h"

namespace
{
using eGroupState= SourcePose::eGroupState;

float smoothstep(float t)
{
	t= std::clamp(t, 0.f, 1.f);
	return t * t * (3.f - 2.f * t);
}

// The confidence a torso or a head is gated on, null where it has none
const SourcePose::Confidence* gateConfidence(const SourcePose::Torso&)
{
	return nullptr;
}

const SourcePose::Confidence* gateConfidence(const SourcePose::Head& head)
{
	return &head.confidence;
}

// A held group's confidences, decayed from the held values
void decayConfidences(const SourcePose::Torso&, float, SourcePose::Torso&)
{
}

void decayConfidences(const SourcePose::Head& held, float decay, SourcePose::Head& outHead)
{
	if (held.confidence.bPresent)
		outHead.confidence.value= held.confidence.value * decay;
}

// What places a direct arm's bones, read off the source's arm and hand
DirectRetarget::ArmPlacement placementOf(const SourcePose::Arm& arm, const SourcePose::Hand& hand)
{
	DirectRetarget::ArmPlacement placement;
	placement.upperDirection= arm.upperDirection;
	placement.forearmDirection= arm.forearmDirection;
	placement.twist= arm.twist;
	placement.wristRotation= hand.wristRotation;
	placement.fingers= hand.fingers;
	return placement;
}

// The Timed hold's weight on a frame the group is not seen: the linear decay
// of its confidences while the hold lasts, or a negative value once it is
// over (past its length, or a timestamp regression)
float timedHoldDecay(const ChainLossPolicy& policy, bool bHeld, double heldMs, double timestampMs)
{
	if (policy.holdMs <= 0.f || !bHeld)
		return -1.f;
	const double elapsedMs= timestampMs - heldMs;
	if (elapsedMs < 0.0 || elapsedMs > (double)policy.holdMs)
		return -1.f;
	return (float)(1.0 - elapsedMs / (double)policy.holdMs);
}
} // namespace

LossPolicy makePositionLossPolicy(float minConfidence, float holdOnDropoutMs, bool bFreezeOnLoss)
{
	LossPolicy policy;
	policy.torso.bFollowsParent= true;
	policy.head.hold= eLossHold::None;
	policy.head.after= eLossAfter::Release;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		ChainLossPolicy& arm= policy.arms[sideIndex];
		arm.minConfidence= minConfidence;
		arm.hold= eLossHold::Timed;
		arm.holdMs= holdOnDropoutMs;
		arm.after= bFreezeOnLoss ? eLossAfter::Freeze : eLossAfter::Release;
		policy.hands[sideIndex].bFollowsParent= true;
	}
	return policy;
}

LossPolicy makeDirectLossPolicy(float restBlendMs)
{
	LossPolicy policy;
	policy.torso.hold= eLossHold::Pipeline;
	policy.torso.after= eLossAfter::EaseToRest;
	policy.head.hold= eLossHold::Pipeline;
	policy.head.after= eLossAfter::EaseToRest;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		ChainLossPolicy& arm= policy.arms[sideIndex];
		arm.hold= eLossHold::Pipeline;
		arm.after= eLossAfter::EaseToRest;
		arm.restBlendMs= restBlendMs;
		policy.hands[sideIndex].bFollowsParent= true;
	}
	return policy;
}

void LossStage::reset()
{
	m_position= PipelineMemory();
	m_direct= PipelineMemory();
}

template <typename Group>
void LossStage::resolveGroup(const ChainLossPolicy& policy, double timestampMs, Group& ioGroup,
							 GroupMemory<Group>& memory)
{
	const SourcePose::Confidence* confidence= gateConfidence(ioGroup);
	const bool bGated= confidence != nullptr && confidence->bPresent && confidence->value < policy.minConfidence;
	const bool bMeasured= ioGroup.state == eGroupState::Measured && !bGated;

	bool bCarried= false;
	switch (policy.hold)
	{
	case eLossHold::None:
		bCarried= bMeasured;
		break;
	case eLossHold::Pipeline:
		bCarried= bMeasured || ioGroup.state == eGroupState::Held;
		break;
	case eLossHold::Timed:
		if (bMeasured)
		{
			memory.bHeld= true;
			memory.heldMs= timestampMs;
			memory.held= ioGroup;
			bCarried= true;
		}
		else
		{
			const float decay= timedHoldDecay(policy, memory.bHeld, memory.heldMs, timestampMs);
			if (decay >= 0.f)
			{
				ioGroup= memory.held;
				ioGroup.state= eGroupState::Held;
				decayConfidences(memory.held, decay, ioGroup);
				bCarried= true;
			}
			else
			{
				memory.bHeld= false;
			}
		}
		break;
	}

	if (bCarried)
	{
		memory.bCarried= true;
		memory.carried= ioGroup;
		return;
	}

	switch (policy.after)
	{
	case eLossAfter::Freeze:
		if (memory.bCarried)
		{
			ioGroup= memory.carried;
			ioGroup.state= eGroupState::Frozen;
			return;
		}
		break;
	case eLossAfter::EaseToRest:
		// The torso and the head rest at the identity, the default values
		ioGroup= Group();
		ioGroup.state= eGroupState::Resting;
		return;
	case eLossAfter::Release:
		break;
	}
	ioGroup= Group();
}

void LossStage::apply(const LossPolicy& policy, const AvatarSkeleton* skeleton,
					  const std::array<AvatarRetargetConfig::Side, 2>& rigSides, SourcePose& ioSource,
					  LossHands* ioHands)
{
	if (ioSource.pipeline == SourcePose::ePipeline::None)
		return;

	// The memory belongs to one epoch of its pipeline's clock
	PipelineMemory& memory= ioSource.pipeline == SourcePose::ePipeline::Direct ? m_direct : m_position;
	if (!memory.bStarted || memory.epoch != ioSource.clock.resetEpoch)
	{
		memory= PipelineMemory();
		memory.bStarted= true;
		memory.epoch= ioSource.clock.resetEpoch;
	}
	const double timestampMs= ioSource.clock.timestampMs;

	// A torso with a loss of its own resolves first, so an arm's rest pose
	// hangs off the lean the torso hands on
	if (!policy.torso.bFollowsParent)
		resolveGroup(policy.torso, timestampMs, ioSource.torso, memory.torso);

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		resolveSide(policy.arms[sideIndex], sideIndex, skeleton, rigSides[sideIndex], memory, ioSource,
					ioHands);
	}

	// A torso that is the line between the shoulders is carried while both
	// arms are carried with a shoulder, measured only when both arms are
	if (policy.torso.bFollowsParent)
	{
		const SourcePose::Arm& left= ioSource.arms[0];
		const SourcePose::Arm& right= ioSource.arms[1];
		SourcePose::Torso& torso= ioSource.torso;
		const bool bBothShoulders= left.state != eGroupState::Lost && right.state != eGroupState::Lost &&
			left.bHasShoulder && right.bHasShoulder;
		if (!bBothShoulders)
			torso.state= eGroupState::Lost;
		else if (left.state == eGroupState::Measured && right.state == eGroupState::Measured)
			torso.state= eGroupState::Measured;
		else if (left.state == eGroupState::Frozen || right.state == eGroupState::Frozen)
			torso.state= eGroupState::Frozen;
		else
			torso.state= eGroupState::Held;
	}

	resolveGroup(policy.head, timestampMs, ioSource.head, memory.head);
}

void LossStage::resolveSide(const ChainLossPolicy& policy, int sideIndex, const AvatarSkeleton* skeleton,
							const AvatarRetargetConfig::Side& rigSide, PipelineMemory& memory, SourcePose& ioSource,
							LossHands* ioHands)
{
	const double timestampMs= ioSource.clock.timestampMs;
	SourcePose::Arm& arm= ioSource.arms[sideIndex];
	SourcePose::Hand& hand= ioSource.hands[sideIndex];
	HandPose* pose= ioHands != nullptr ? &ioHands->poses[sideIndex] : nullptr;
	if (ioHands != nullptr)
		ioHands->bCarried[sideIndex]= false;

	// Seen: measured, or a hand the pipeline saw with nothing to place it by
	// (its confidence present on a lost arm); either below the gate is not
	// seen. Placed: seen and measured.
	const bool bConfidence= hand.confidence.bPresent;
	const bool bGated= bConfidence && hand.confidence.value < policy.minConfidence;
	const bool bSeen= (arm.state == eGroupState::Measured || bConfidence) && !bGated;
	const bool bPlaced= bSeen && arm.state == eGroupState::Measured;

	bool bCarried= false;
	switch (policy.hold)
	{
	case eLossHold::None:
		bCarried= bPlaced;
		break;
	case eLossHold::Pipeline:
		bCarried= (arm.state == eGroupState::Measured && !bGated) || arm.state == eGroupState::Held;
		break;
	case eLossHold::Timed:
	{
		SideValues& held= memory.held[sideIndex];
		if (bSeen)
		{
			// A seen side restarts the hold on this frame, with nothing to
			// hold when it could not be placed
			held.valid= bPlaced;
			held.timestampMs= timestampMs;
			if (bPlaced)
			{
				held.arm= arm;
				held.hand= hand;
				if (pose != nullptr)
					held.pose= *pose;
			}
			bCarried= bPlaced;
			break;
		}
		const float decay= timedHoldDecay(policy, held.valid, held.timestampMs, timestampMs);
		if (decay < 0.f)
		{
			held.valid= false;
			break;
		}
		// The held values, their confidences decaying with the hold so a
		// held arm is never read as freshly measured (the position solve
		// weighs the measured elbow on its forearm's confidence)
		arm= held.arm;
		hand= held.hand;
		arm.state= eGroupState::Held;
		hand.state= eGroupState::Held;
		if (held.hand.confidence.bPresent)
			hand.confidence.value= held.hand.confidence.value * decay;
		if (held.arm.forearmConfidence.bPresent)
			arm.forearmConfidence.value= held.arm.forearmConfidence.value * decay;
		if (held.arm.shoulderConfidence.bPresent)
			arm.shoulderConfidence.value= held.arm.shoulderConfidence.value * decay;
		if (pose != nullptr)
		{
			*pose= held.pose;
			pose->confidence= held.pose.confidence * decay;
			pose->forearmConfidence= held.pose.forearmConfidence * decay;
			pose->shoulderConfidence= held.pose.shoulderConfidence * decay;
		}
		bCarried= true;
		break;
	}
	}

	EaseState& ease= memory.ease[sideIndex];
	if (bCarried)
	{
		// The hand goes as the pipeline or the hold gave it
		SideValues& carried= memory.carried[sideIndex];
		carried.valid= true;
		carried.timestampMs= timestampMs;
		carried.arm= arm;
		carried.hand= hand;
		if (pose != nullptr)
		{
			carried.pose= *pose;
			ioHands->bCarried[sideIndex]= true;
		}
		ease.easeStartMs= -1.0;
		ease.lastPlacement= placementOf(arm, hand);
		ease.bPlaced= true;
		return;
	}

	switch (policy.after)
	{
	case eLossAfter::Freeze:
	{
		const SideValues& carried= memory.carried[sideIndex];
		if (!carried.valid)
			break;
		arm= carried.arm;
		hand= carried.hand;
		arm.state= eGroupState::Frozen;
		hand.state= eGroupState::Frozen;
		if (pose != nullptr)
		{
			*pose= carried.pose;
			ioHands->bCarried[sideIndex]= true;
		}
		return;
	}
	case eLossAfter::EaseToRest:
	{
		if (ioSource.pipeline != SourcePose::ePipeline::Direct || skeleton == nullptr)
			break;
		// The ease into the rig's rest pose, from what the bones took last
		// frame, timed from that frame so the first frame without a chain
		// already moves; an arm nothing has placed in this epoch is at rest
		// at once
		if (ease.easeStartMs < 0.0)
		{
			ease.easeStartMs= timestampMs - ioSource.clock.frameIntervalMs;
			ease.easeFrom= ease.lastPlacement;
		}
		const DirectRetarget::ArmPlacement rest=
			DirectRetarget::makeRestPlacement(*skeleton, sideIndex, ioSource.torso.spineDelta, rigSide);
		float blend= 1.f;
		if (ease.bPlaced && policy.restBlendMs > 0.f)
			blend= smoothstep((float)((timestampMs - ease.easeStartMs) / policy.restBlendMs));
		const DirectRetarget::ArmPlacement placement=
			blend < 1.f ? DirectRetarget::blendPlacements(ease.easeFrom, rest, blend) : rest;
		ease.lastPlacement= placement;
		ease.bPlaced= true;

		arm= SourcePose::Arm();
		arm.state= eGroupState::Resting;
		arm.restBlend= blend;
		arm.upperDirection= placement.upperDirection;
		arm.forearmDirection= placement.forearmDirection;
		arm.twist= placement.twist;
		hand= SourcePose::Hand();
		hand.state= eGroupState::Resting;
		hand.fingerZero= SourcePose::eFingerZero::Metacarpal;
		hand.wristRotation= placement.wristRotation;
		hand.fingers= placement.fingers;
		return;
	}
	case eLossAfter::Release:
		break;
	}

	arm= SourcePose::Arm();
	hand= SourcePose::Hand();
}
