#pragma once

#include <array>
#include <cstdint>

#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/vector_float3.hpp"

#include "TrackingTypes.h" // FingerAngles

// The hand-off from a pipeline to the target solve: what the pipeline
// measured this frame, per group (the torso, the head, each arm, each hand),
// with the frame's clock. Both pipelines fill it. The position pipeline fills
// world positions and orientations in the user's own units; the direct
// pipeline fills its arm chain in the avatar's units and the placement the
// avatar's bones take (the chain's directions, the forearm's twist against
// the avatar's rest palm, the wrist). A plain value with no pointers, so it
// can be copied across threads.
struct SourcePose
{
	// Which pipeline filled it, which fixes the units and which fields carry
	// values (each field says which pipeline fills it)
	enum class ePipeline : uint8_t
	{
		None,
		Position, // the fused world pose, meters, the user's proportions
		Direct,   // the single camera's world, the avatar's units
	};

	// What a group carries this frame: measured on this frame, held from an
	// earlier one (by the pipeline, or by the loss stage within its hold), or
	// nothing. Past a hold the loss stage writes the two states after it:
	// frozen, the last values it handed on repeated, and resting, the rig's
	// rest pose or the ease into it. A filler writes only the first three.
	enum class eGroupState : uint8_t
	{
		Lost,
		Measured,
		Held,
		Frozen,
		Resting,
	};

	// What a finger angle of zero means: the user's captured rest pose (the
	// position pipeline), or the finger lying along its own metacarpal with
	// the hand's natural fan (the direct pipeline)
	enum class eFingerZero : uint8_t
	{
		CapturedRest,
		Metacarpal,
	};

	// A confidence the pipeline's input carries, [0,1]. Absent where the
	// input has none.
	struct Confidence
	{
		bool bPresent= false;
		float value= 0.f;
	};

	// The frame's clock, written by the filler alone. The epoch counts the
	// filler's restarts: a consumer that keeps state across frames drops it
	// when the epoch changes, rather than keeping a clock of its own.
	struct Clock
	{
		double timestampMs= 0.0;
		// Since the filler's previous frame, zero on the first frame of an
		// epoch (negative on a timestamp regression the filler does not
		// restart on)
		double frameIntervalMs= 0.0;
		uint32_t resetEpoch= 0;
	};

	// The torso. Position: measured while both arms carry a shoulder (the
	// shoulder line places the root). Direct: the spine's rotation away from
	// the neutral torso, already limited, since the limited spine is what the
	// forearm's twist was measured under.
	struct Torso
	{
		eGroupState state= eGroupState::Lost;
		glm::quat spineDelta{1.f, 0.f, 0.f, 0.f}; // Direct
	};

	// The head. Position: the world head frame (+X facing, +Y left, +Z up).
	// Direct: the head's rotation from the neutral head, or the phone's head,
	// before the joint limit and the rig's trim. Identity while lost.
	struct Head
	{
		eGroupState state= eGroupState::Lost;
		Confidence confidence;
		glm::quat orientation{1.f, 0.f, 0.f, 0.f};
	};

	// One arm. The positions are world, in the pipeline's units. Position:
	// the shoulder when the body solver measured it, the wrist joint, and the
	// forearm (IMU or body solver) with the elbow it implies at the user's
	// forearm length. Direct: the solved chain, every joint present, plus the
	// placement: the chain's directions and the forearm's twist about its
	// axis against the avatar's rest palm (zero while the hand was never
	// measured). A resting arm carries the placement the loss stage eased it
	// to and no chain, its hand the rest wrist and fingers.
	struct Arm
	{
		eGroupState state= eGroupState::Lost;
		bool bHasShoulder= false;
		glm::vec3 shoulderPosition{0.f};
		Confidence shoulderConfidence;
		bool bHasElbow= false;
		glm::vec3 elbowPosition{0.f};
		glm::vec3 wristPosition{0.f};
		bool bHasForearmOrientation= false; // Position
		glm::quat forearmOrientation{1.f, 0.f, 0.f, 0.f};
		Confidence forearmConfidence;
		glm::vec3 upperDirection{0.f};   // Direct
		glm::vec3 forearmDirection{0.f}; // Direct
		float twist= 0.f;                // Direct
		// Resting: how far the ease has carried the arm into the rest pose, 1
		// at rest
		float restBlend= 0.f;
	};

	// One hand. Position: the world palm frame. Direct: the palm in the
	// measured forearm frame (identity, the avatar's rest wrist, while the
	// hand was never measured). Both: the finger angle table against the
	// zero the enum names. A hand the position pipeline tracked without a
	// world anchor reads lost with its confidence present: seen, but nothing
	// to place, which ends a hold where an unseen hand would bridge one.
	struct Hand
	{
		eGroupState state= eGroupState::Lost;
		Confidence confidence;
		glm::quat palmOrientation{1.f, 0.f, 0.f, 0.f}; // Position
		glm::quat wristRotation{1.f, 0.f, 0.f, 0.f};   // Direct
		eFingerZero fingerZero= eFingerZero::CapturedRest;
		std::array<FingerAngles, FINGER_COUNT> fingers{};
	};

	ePipeline pipeline= ePipeline::None;
	Clock clock;
	// The arm lengths the source's positions were measured with, in its
	// units: the user's own (Position), or the user's proportions in the
	// avatar's units (Direct)
	float upperArmLength= 0.f;
	float forearmLength= 0.f;
	Torso torso;
	Head head;
	std::array<Arm, 2> arms{};   // by eHandSide
	std::array<Hand, 2> hands{}; // by eHandSide
};
