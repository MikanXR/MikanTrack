#pragma once

#include <array>
#include <cstdint>

#include "AvatarRetarget.h" // AvatarRetargetConfig::Side
#include "DirectRetarget.h" // DirectRetarget::ArmPlacement
#include "SourcePose.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

// The loss stage: between the pipeline that fills the source pose and the
// target solve that poses the avatar from it, it decides per group what the
// target gets on a frame the group is not measured. Whether a group is
// measured is the pipeline's call; what happens after a loss is this
// stage's, one policy per chain. It keeps no clock of its own: it reads the
// source pose's, and drops its state when the source's epoch changes.

// How long a group's last values carry it once the pipeline stops measuring
// it
enum class eLossHold : uint8_t
{
	// Nothing held: a group not measured goes straight to what follows the
	// hold
	None,
	// The pipeline's own hold: a group it reports as held passes as held,
	// with the values it gave. The direct solver holds its arms for the hold
	// they earned and its torso and head for its maximum, because its
	// measurement continues across those frames.
	Pipeline,
	// The last measured values for holdMs, their confidences decaying
	// linearly to zero across it. A timestamp regression ends the hold.
	Timed,
};

// What a group becomes past its hold
enum class eLossAfter : uint8_t
{
	// Lost: the target solve leaves its bones out
	Release,
	// The last values handed on, repeated, so the target solve re-solves
	// them every frame and whatever else moves (the root follow) still moves
	Freeze,
	// The rest pose: an arm of the direct pipeline eases into the rig's rest
	// pose over restBlendMs from the last placement handed on, the torso and
	// the head rest at the identity at once. An arm of the position pipeline
	// has no rest placement and is released.
	EaseToRest,
};

// One chain's loss rule
struct ChainLossPolicy
{
	// The group has no loss of its own and goes with its parent: a hand with
	// its arm (held, frozen, rested or released together), and the position
	// pipeline's torso, which is the line between the shoulders, carried
	// while both arms are carried with a shoulder. A hand always goes with
	// its arm: a hand chain with a loss of its own is not implemented.
	bool bFollowsParent= false;
	// A measured group whose confidence is below this reads as not seen.
	// Zero passes every group. An arm reads its hand's confidence, the one
	// confidence the position pipeline's tracking scores a side by.
	float minConfidence= 0.f;
	eLossHold hold= eLossHold::None;
	// The Timed hold's length, milliseconds; zero or less holds nothing
	float holdMs= 0.f;
	eLossAfter after= eLossAfter::Release;
	// The arm's ease into rest, milliseconds; zero snaps
	float restBlendMs= 0.f;
};

// The loss rule of every chain: plain values, set per pipeline
struct LossPolicy
{
	ChainLossPolicy torso;
	ChainLossPolicy head;
	std::array<ChainLossPolicy, 2> arms{};  // by eHandSide
	std::array<ChainLossPolicy, 2> hands{}; // by eHandSide
};

// The position pipeline's rules: each arm gated on its hand's confidence (an
// avatar holding its last good pose looks better than one following a
// jittering estimate), held for holdOnDropoutMs with the confidences
// decaying, which bridges the short dropouts, then frozen or released; the
// hands with their arms, the torso with the arms, the head released the
// frame it is not measured. VMC carries no tracked flag, so a lost arm can
// only be told by its motion: frozen, it reads better than bones that stop
// arriving, which a receiver drops back to the avatar's T-pose.
LossPolicy makePositionLossPolicy(float minConfidence, float holdOnDropoutMs, bool bFreezeOnLoss);

// The direct pipeline's rules: each arm through the hold the solver earned,
// then eased into the rig's rest pose over restBlendMs; the hands with their
// arms; the torso and the head through the solver's hold, then at rest
LossPolicy makeDirectLossPolicy(float restBlendMs);

// The position pipeline's hand poses beside its source pose. In: the frame's.
// Out: what each side carries (measured, held with its confidences decayed,
// or frozen), which the measured-length stream builds its bones from with no
// avatar loaded.
struct LossHands
{
	std::array<HandPose, 2> poses{};
	std::array<bool, 2> bCarried{};
};

class LossStage
{
public:
	// Forgets every hold, freeze and ease
	void reset();

	// Rewrites the source pose with what the target solve gets on this
	// frame, each group's state saying how it got there. skeleton and
	// rigSides place the direct pipeline's rest pose (no skeleton, no rest:
	// the arm is released). ioHands: the position pipeline's hand poses,
	// resolved alongside the source, or null.
	void apply(const LossPolicy& policy, const AvatarSkeleton* skeleton,
			   const std::array<AvatarRetargetConfig::Side, 2>& rigSides, SourcePose& ioSource, LossHands* ioHands);

private:
	// One side's arm and hand as they were handed on, with the hand pose
	// they came from
	struct SideValues
	{
		bool valid= false;
		double timestampMs= 0.0;
		SourcePose::Arm arm;
		SourcePose::Hand hand;
		HandPose pose;
	};
	// The ease into rest: what the bones took last frame and whether
	// anything placed them in this epoch, and when the ease began (negative
	// while the arm is carried) and the placement it started from
	struct EaseState
	{
		bool bPlaced= false;
		DirectRetarget::ArmPlacement lastPlacement;
		double easeStartMs= -1.0;
		DirectRetarget::ArmPlacement easeFrom;
	};
	// A torso or a head: its timed hold and the last values handed on
	template <typename Group>
	struct GroupMemory
	{
		bool bHeld= false;
		double heldMs= 0.0;
		Group held;
		bool bCarried= false;
		Group carried;
	};
	// One pipeline's memory and the epoch it belongs to. The two pipelines
	// keep theirs apart: a project in direct mode falls back to the position
	// pipeline on a frame the direct solve cannot run, and each pipeline's
	// epoch counts its own filler's restarts.
	struct PipelineMemory
	{
		bool bStarted= false;
		uint32_t epoch= 0;
		std::array<SideValues, 2> held{};
		std::array<SideValues, 2> carried{};
		std::array<EaseState, 2> ease{};
		GroupMemory<SourcePose::Torso> torso;
		GroupMemory<SourcePose::Head> head;
	};

	template <typename Group>
	static void resolveGroup(const ChainLossPolicy& policy, double timestampMs, Group& ioGroup,
							 GroupMemory<Group>& memory);
	void resolveSide(const ChainLossPolicy& policy, int sideIndex, const AvatarSkeleton* skeleton,
					 const AvatarRetargetConfig::Side& rigSide, PipelineMemory& memory, SourcePose& ioSource,
					 LossHands* ioHands);

	PipelineMemory m_position;
	PipelineMemory m_direct;
};
