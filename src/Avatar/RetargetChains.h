#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "glm/ext/vector_float3.hpp"
#include "nlohmann/json_fwd.hpp"

#include "AvatarRetarget.h" // AvatarRetargetConfig::Side
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
		// mapping, 0 to 1
		ExprParameter ikWeight{1.f};
		// The scale on the source's reach before the goal is placed
		ExprParameter reachScale{1.f};
		// Added to the goal, meters, in the rooted torso frame (+X facing, +Y
		// the avatar's left, +Z up)
		glm::vec3 goalOffset{0.f};
		// The forearm confidence at and above which the measured elbow fully
		// decides the bend; below it the bend blends toward the rig's hint
		float elbowHintConfidence= 0.5f;
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

	// Default-constructed: a multi-camera project's defaults
	Group torso;
	Group head;
	std::array<Arm, 2> arms{};   // by eHandSide
	std::array<Hand, 2> hands{}; // by eHandSide
};

// The chains of a project from before chain settings existed. The arms take
// the shape's mapping (bone angles for a direct project, bone angles then IK
// otherwise) and its IK weight (0 direct, 1 otherwise), a reach scale of 1,
// no goal offset, and the sidecar's elbow hint confidence. A multi-camera
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
// stored numbers stay. The position pipeline performs no hold or a timed one
// (its own hold becomes a timed one) and a release or a freeze (an ease to
// rest becomes a freeze) on the arms and the head, gates the arms on
// minConfidence, and carries its torso with the arms. The direct pipeline
// performs its own hold then the ease to rest on every chain, each arm over
// its own rest blend.
LossPolicy makeChainLossPolicy(const RetargetChains& chains, SourcePose::ePipeline pipeline, float minConfidence);

// The solver's per-side fields that are chain settings: the elbow hint
// confidence
void applyRetargetChains(const RetargetChains& chains, std::array<AvatarRetargetConfig::Side, 2>& ioSides);

// JSON: an object of the six chains, "torso", "head", "leftArm", "rightArm",
// "leftHand" and "rightHand"
nlohmann::json retargetChainsToJson(const RetargetChains& chains);
// Tolerant: a missing chain or field, a value of the wrong type and an
// unknown enum string each take the value in defaults, and the load logs one
// warning naming them (also appended to outDefaulted when given). Unknown
// keys are ignored.
RetargetChains retargetChainsFromJson(const nlohmann::json& j, const RetargetChains& defaults,
									  std::vector<std::string>* outDefaulted= nullptr);

const char* armMappingName(eArmMapping mapping);
const char* lossHoldName(eLossHold hold);
const char* lossAfterName(eLossAfter after);
