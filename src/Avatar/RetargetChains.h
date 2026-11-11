#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "glm/ext/vector_float3.hpp"
#include "nlohmann/json_fwd.hpp"

#include "AvatarRetarget.h" // k_defaultElbowHintConfidence
#include "ExprParameter.h"
#include "LossStage.h"
#include "SourcePose.h"

// The retarget's settings per chain of the avatar (the torso, the head, each
// arm, each hand): how the source pose maps onto the chain and what the chain
// does after a loss. They are the project's, persisted in its project file,
// where the rig's corrections persist in the avatar's sidecar and travel with
// the model. Plain values with a JSON round trip, free of any UI.

// How an arm chain takes the source pose
enum class eArmMapping : uint8_t
{
	// The source's bone directions onto the avatar's bones
	BoneAngles,
	// The bone directions, then the chain's end pulled toward a goal by the
	// arm's IK weight
	BoneAnglesThenIk,
};

struct RetargetChains
{
	// A chain's loss policy as stored. Which group is measured is the
	// pipeline's call, and the minimum confidence that reads a group as not
	// seen is the delivery rules' setting, so neither is stored here.
	struct Loss
	{
		eLossHold hold= eLossHold::None;
		// The timed hold's length, milliseconds
		float holdMs= 0.f;
		eLossAfter after= eLossAfter::Release;
		// The ease into rest, milliseconds; zero snaps
		float restBlendMs= 0.f;
	};

	struct Arm
	{
		eArmMapping mapping= eArmMapping::BoneAnglesThenIk;
		// How far the chain's end is pulled toward the goal after the angle
		// mapping, 0 to 1, read only by BoneAnglesThenIk. Both pipelines run
		// the mapping and the weight as stored.
		ExprParameter ikWeight{1.f};
		// The scale on the source's reach before the goal is placed, 0 to 1:
		// at 1 the user's reach is scaled onto the avatar's arm, at 0 the
		// measured shoulder-to-wrist offset is kept as measured (meters, or
		// the direct source's avatar units). A function reads the source
		// pose's inputs, as the elbow weight's does.
		ExprParameter reachScale{1.f};
		// Added to the goal after the scaling and before the clamp to the
		// avatar's reach, meters, in the rooted torso frame (+X facing, +Y
		// the avatar's left, +Z up)
		glm::vec3 goalOffset{0.f};
		// The weight toward the measured elbow, 0 to 1: at 1 the measured
		// elbow alone decides the bend, at 0 the rig's hint alone. A function
		// reads the source pose's inputs (SourcePoseInputs). A chain set's
		// arms default to the function of the rule the solve always applied
		// (makeDefaultElbowWeight); an Arm constructed alone holds a plain 1.
		ExprParameter elbowWeight{1.f};
		Loss loss{eLossHold::Timed, 250.f, eLossAfter::Freeze, 300.f};
	};

	// A hand follows its arm's loss and holds nothing else yet
	struct Hand
	{
	};

	// The torso and the head
	struct Group
	{
		Loss loss;
	};

	// An arm with its side's default elbow weight
	static Arm makeDefaultArm(int sideIndex);

	// Default-constructed: a multi-camera project's defaults
	Group torso;
	Group head;
	std::array<Arm, 2> arms{makeDefaultArm(0), makeDefaultArm(1)}; // by eHandSide
	std::array<Hand, 2> hands{};                                   // by eHandSide
};

// The elbow weight that reproduces the rule the solve applied before the
// weight was a setting (elbowWeightFromConfidence) for one side and one
// threshold: the function
//   clamp(source.<side>LowerArm.confidence / elbowHintConfidence, 0, 1)
// with the threshold written so it reads back as the same float, and a
// plain 1 behind it. The expression language divides by zero to 0 where the
// rule gives 1, so a threshold of zero or less (or not a number) is the
// plain weight 1, and an infinite one the plain weight 0.
ExprParameter makeDefaultElbowWeight(int sideIndex, float elbowHintConfidence);

// The threshold an elbow weight of makeDefaultElbowWeight's form was made
// with: false for any other weight. The plain weight 1 reads as a threshold
// of 0.
bool findElbowHintConfidence(const ExprParameter& elbowWeight, int sideIndex, float& outElbowHintConfidence);

// The chains of a project from before chain settings existed. The arms take
// the shape's mapping (bone angles for a direct project, bone angles then IK
// otherwise) and its IK weight (0 direct, 1 otherwise), a reach scale of 1,
// no goal offset, and the elbow weight of the sidecar's elbow hint
// confidence (makeDefaultElbowWeight). A multi-camera
// arm holds for holdOnDropoutMs and then freezes or releases, a direct arm
// takes the pipeline's hold and then eases to rest; both keep the other
// shape's numbers (the hold length, the rest blend) so a change of shape
// keeps them. The torso and the head take what the shape's loss policy gives
// them.
RetargetChains migrateRetargetChains(bool bDirect, float holdOnDropoutMs, bool bFreezeOnLoss, float restBlendMs,
									 const std::array<float, 2>& elbowHintConfidence);

// A shape's defaults: the migration of the default legacy settings
RetargetChains makeDefaultRetargetChains(bool bDirect);

// The loss rules one pipeline performs from the chains. A stored kind the
// pipeline cannot perform takes that pipeline's default for the chain, the
// stored numbers stay. What follows the hold (release, freeze, or the ease
// to rest over the stored rest blend) is performed as stored by both
// pipelines, but for an ease to rest in the position pipeline without an
// avatar (bAvatar false), whose measured-length stream has no rest pose:
// it takes the default, a freeze for an arm and a release for the head. The
// position pipeline performs no hold or a timed one (its own hold becomes a
// timed one of the stored length) on the arms and the head, gates the arms
// on minConfidence, and carries its torso with the arms. The direct pipeline
// keeps its own hold on every chain, whatever kind is stored, since that
// hold also keeps the solver's chain and filters alive.
LossPolicy makeChainLossPolicy(const RetargetChains& chains, SourcePose::ePipeline pipeline, float minConfidence,
							   bool bAvatar);

// The other arm's copy of an arm: the goal offset reflected across the body's
// midplane, the text of each parameter that can be a function (the IK
// weight, the reach scale, the elbow weight) with each source name of one
// side spelled for the other (SourcePoseInputs::swapSides), every other value
// as it is
RetargetChains::Arm mirrorArm(const RetargetChains::Arm& arm);

// JSON: an object of the six chains, "torso", "head", "leftArm", "rightArm",
// "leftHand" and "rightHand". An arm's elbowWeight is an ExprParameter.
nlohmann::json retargetChainsToJson(const RetargetChains& chains);
// Tolerant: a missing chain or field, a value of the wrong type and an
// unknown enum string each take the value in defaults, and the load logs one
// warning naming them (also appended to outDefaulted when given). Unknown
// keys are ignored. An arm written before the elbow weight, with an
// elbowHintConfidence and no elbowWeight, takes makeDefaultElbowWeight of
// that threshold.
RetargetChains retargetChainsFromJson(const nlohmann::json& j, const RetargetChains& defaults,
									  std::vector<std::string>* outDefaulted= nullptr);

const char* armMappingName(eArmMapping mapping);
const char* lossHoldName(eLossHold hold);
const char* lossAfterName(eLossAfter after);
