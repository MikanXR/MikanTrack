#include "TestCommon.h"

#include "AvatarRig.h"
#include "DirectRetarget.h"
#include "LossStage.h"
#include "RetargetChains.h"
#include "RetargetStage.h"

// The retarget chain settings: their JSON round trip and tolerant loading,
// the migration of a project without them from its legacy settings and the
// avatar's sidecar, the block taking the place of the legacy keys, the stage
// config each pipeline derives from a block written under either shape, and
// the sidecar left as it was. All in memory.

namespace
{
bool sameParameter(const ExprParameter& a, const ExprParameter& b)
{
	return a.value == b.value && a.expression == b.expression && a.bUseExpression == b.bUseExpression;
}

bool sameLoss(const RetargetChains::Loss& a, const RetargetChains::Loss& b)
{
	return a.hold == b.hold && a.holdMs == b.holdMs && a.after == b.after && a.restBlendMs == b.restBlendMs;
}

bool sameChains(const RetargetChains& a, const RetargetChains& b)
{
	bool bSame= sameLoss(a.torso.loss, b.torso.loss) && sameLoss(a.head.loss, b.head.loss);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const RetargetChains::Arm& x= a.arms[sideIndex];
		const RetargetChains::Arm& y= b.arms[sideIndex];
		bSame&= x.mapping == y.mapping && sameParameter(x.ikWeight, y.ikWeight) &&
			sameParameter(x.reachScale, y.reachScale) && x.goalOffset == y.goalOffset &&
			x.elbowHintConfidence == y.elbowHintConfidence && sameLoss(x.loss, y.loss);
	}
	return bSame;
}

bool samePolicy(const ChainLossPolicy& a, const ChainLossPolicy& b)
{
	return a.bFollowsParent == b.bFollowsParent && a.minConfidence == b.minConfidence && a.hold == b.hold &&
		a.holdMs == b.holdMs && a.after == b.after && a.restBlendMs == b.restBlendMs;
}

bool samePolicy(const LossPolicy& a, const LossPolicy& b)
{
	bool bSame= samePolicy(a.torso, b.torso) && samePolicy(a.head, b.head);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		bSame&= samePolicy(a.arms[sideIndex], b.arms[sideIndex]) && samePolicy(a.hands[sideIndex], b.hands[sideIndex]);
	return bSame;
}

// A rig whose legacy fields differ from the defaults and per side
AvatarRigSettings makeLegacyRig()
{
	AvatarRigSettings rig;
	rig.restBlendMs= 500.f;
	rig.sides[0].elbowHintConfidence= 0.3f;
	rig.sides[1].elbowHintConfidence= 0.7f;
	return rig;
}

// A project whose legacy stream settings differ from the defaults
AppConfig makeLegacyConfig(bool bDirect)
{
	AppConfig config;
	config.direct.enabled= bDirect;
	config.osc.minConfidence= 0.2f;
	config.osc.holdOnDropoutMs= 400.f;
	config.osc.vmcFreezeOnLoss= false;
	return config;
}

// Every field away from both shapes' defaults, and an expression on one arm
RetargetChains makeEditedChains()
{
	RetargetChains chains;
	chains.torso.loss= {eLossHold::Pipeline, 60.f, eLossAfter::EaseToRest, 0.f};
	chains.head.loss= {eLossHold::Timed, 120.f, eLossAfter::Freeze, 0.f};
	RetargetChains::Arm& left= chains.arms[0];
	left.mapping= eArmMapping::BoneAngles;
	left.ikWeight= ExprParameter{0.6f, "clamp(source.leftLowerArm.confidence / 0.5, 0, 1)", true};
	left.reachScale= ExprParameter{1.2f};
	left.goalOffset= glm::vec3(0.01f, -0.02f, 0.03f);
	left.elbowHintConfidence= 0.35f;
	left.loss= {eLossHold::None, 0.f, eLossAfter::Release, 150.f};
	RetargetChains::Arm& right= chains.arms[1];
	right.ikWeight= ExprParameter{0.25f, "0.5", false};
	right.elbowHintConfidence= 0.65f;
	right.loss= {eLossHold::Timed, 333.f, eLossAfter::Freeze, 410.f};
	return chains;
}
} // namespace

static int runRetargetChainsTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO("test-retarget-chains") << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR("test-retarget-chains") << "FAIL " << name;
			failures++;
		}
	};

	// (a) The round trip: every field survives a write and a read, the IK
	// weight with an expression in object form and the plain reach scale as
	// a bare number, with nothing taking a default
	{
		const RetargetChains saved= makeEditedChains();
		const std::string text= retargetChainsToJson(saved).dump(2);
		const nlohmann::json parsed= nlohmann::json::parse(text);
		std::vector<std::string> defaulted;
		const RetargetChains loaded= retargetChainsFromJson(parsed, RetargetChains(), &defaulted);
		check(sameChains(saved, loaded) && defaulted.empty(), "(a) every chain field round-trips with none defaulted");
		check(parsed["leftArm"]["ikWeight"].is_object() &&
				  parsed["leftArm"]["ikWeight"].value("useExpression", false) &&
				  parsed["rightArm"]["ikWeight"].is_object() && !parsed["rightArm"]["ikWeight"].value("useExpression", true) &&
				  parsed["leftArm"]["reachScale"].is_number() && parsed["rightArm"]["reachScale"].is_number(),
			  "(a) a parameter with text is an object, a plain one a bare number");
		check(parsed["leftArm"].value("mapping", std::string()) == "boneAngles" &&
				  parsed["rightArm"].value("mapping", std::string()) == "boneAnglesThenIk" &&
				  parsed["torso"]["loss"].value("hold", std::string()) == "pipeline" &&
				  parsed["torso"]["loss"].value("after", std::string()) == "easeToRest" &&
				  parsed["head"]["loss"].value("hold", std::string()) == "timed" &&
				  parsed["leftArm"]["loss"].value("hold", std::string()) == "none" &&
				  parsed["leftArm"]["loss"].value("after", std::string()) == "release" &&
				  parsed["rightArm"]["loss"].value("after", std::string()) == "freeze" &&
				  parsed["leftHand"].is_object() && parsed["rightHand"].is_object(),
			  "(a) the enums are written as readable strings and every chain has its object");
		check(sameChains(RetargetChains(), makeDefaultRetargetChains(false)),
			  "(a) a default-constructed chain set is a multi-camera project's defaults");
	}

	// (b) Tolerant loading: a missing chain, a missing field, an unknown enum
	// string and a value of the wrong type each take the default given, the
	// rest is read, and unknown keys are ignored
	{
		const RetargetChains defaults= makeDefaultRetargetChains(true);
		nlohmann::json j= retargetChainsToJson(makeEditedChains());
		j.erase("rightArm");
		j["leftArm"]["loss"]["hold"]= "sometimes";
		j["leftArm"]["loss"]["holdMs"]= "long";
		j["leftArm"]["ikWeight"]= "half";
		j["leftArm"].erase("goalOffset");
		j["head"]["loss"].erase("after");
		j["leftArm"]["somethingNew"]= 3;
		j["shoulders"]= nlohmann::json::object();
		std::vector<std::string> defaulted;
		const RetargetChains loaded= retargetChainsFromJson(j, defaults, &defaulted);
		const RetargetChains edited= makeEditedChains();
		check(sameLoss(loaded.arms[1].loss, defaults.arms[1].loss) &&
				  loaded.arms[1].mapping == defaults.arms[1].mapping &&
				  sameParameter(loaded.arms[1].ikWeight, defaults.arms[1].ikWeight),
			  "(b) a missing chain takes its default whole");
		check(loaded.arms[0].loss.hold == defaults.arms[0].loss.hold &&
				  loaded.arms[0].loss.holdMs == defaults.arms[0].loss.holdMs &&
				  sameParameter(loaded.arms[0].ikWeight, defaults.arms[0].ikWeight) &&
				  loaded.arms[0].goalOffset == defaults.arms[0].goalOffset &&
				  loaded.head.loss.after == defaults.head.loss.after,
			  "(b) an unknown enum, a wrong type and a missing field each take their default");
		check(loaded.arms[0].loss.after == edited.arms[0].loss.after &&
				  loaded.arms[0].loss.restBlendMs == edited.arms[0].loss.restBlendMs &&
				  loaded.arms[0].elbowHintConfidence == edited.arms[0].elbowHintConfidence &&
				  sameParameter(loaded.arms[0].reachScale, edited.arms[0].reachScale) &&
				  loaded.head.loss.hold == edited.head.loss.hold && sameLoss(loaded.torso.loss, edited.torso.loss),
			  "(b) the readable fields beside them are read");
		auto named= [&defaulted](const char* name) {
			return std::find(defaulted.begin(), defaulted.end(), name) != defaulted.end();
		};
		check(defaulted.size() == 6 && named("rightArm") && named("leftArm.loss.hold") &&
				  named("leftArm.loss.holdMs") && named("leftArm.ikWeight") && named("leftArm.goalOffset") &&
				  named("head.loss.after"),
			  "(b) the defaulted fields are named, and the unknown keys are not");
		std::vector<std::string> notAnObject;
		const RetargetChains fromArray= retargetChainsFromJson(nlohmann::json::array(), defaults, &notAnObject);
		check(sameChains(fromArray, defaults) && notAnObject.size() == 6,
			  "(b) a block that is not an object takes every default");
	}

	// (c) The migration of a multi-camera project without chains: the legacy
	// hold of 400 ms with freeze off, the sidecar's 500 ms rest blend and its
	// elbow hint confidences of 0.3 and 0.7, onto the arms; the stage config
	// is the one the legacy settings built
	{
		const AppConfig config= makeLegacyConfig(false);
		const AvatarRigSettings rig= makeLegacyRig();
		const RetargetChains chains= resolveRetargetChains(config, rig);
		bool bArms= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const RetargetChains::Arm& arm= chains.arms[sideIndex];
			bArms&= arm.mapping == eArmMapping::BoneAnglesThenIk && sameParameter(arm.ikWeight, ExprParameter{1.f}) &&
				sameParameter(arm.reachScale, ExprParameter{1.f}) && arm.goalOffset == glm::vec3(0.f) &&
				arm.elbowHintConfidence == rig.sides[sideIndex].elbowHintConfidence &&
				arm.loss.hold == eLossHold::Timed && arm.loss.holdMs == 400.f &&
				arm.loss.after == eLossAfter::Release && arm.loss.restBlendMs == 500.f;
		}
		check(bArms, "(c) multi-camera: the arms take bone angles then IK at weight 1, the timed hold, the release, "
					 "the rest blend and each side's elbow hint confidence");
		check(chains.head.loss.hold == eLossHold::None && chains.head.loss.after == eLossAfter::Release &&
				  chains.torso.loss.hold == eLossHold::None && chains.torso.loss.after == eLossAfter::Release,
			  "(c) multi-camera: the head and the torso take the position policy's");
		const RetargetStageConfig stage= makeRetargetStageConfig(config, rig, nullptr);
		check(samePolicy(stage.positionLoss, makePositionLossPolicy(0.2f, 400.f, false)) &&
				  samePolicy(stage.directLoss, makeDirectLossPolicy(500.f)),
			  "(c) multi-camera: both pipelines' loss rules are the ones the legacy settings built");
		check(stage.position.sides[0].elbowHintConfidence == 0.3f && stage.position.sides[1].elbowHintConfidence == 0.7f &&
				  stage.direct.sides[0].elbowHintConfidence == 0.3f && stage.direct.sides[1].elbowHintConfidence == 0.7f &&
				  stage.direct.restBlendMs == 500.f,
			  "(c) multi-camera: each arm's elbow hint confidence reaches both solvers");
	}

	// (d) The migration of a direct project without chains, from the same
	// legacy values: the pipeline's hold and the ease to rest, the hold
	// length kept for a change of shape
	{
		const AppConfig config= makeLegacyConfig(true);
		const AvatarRigSettings rig= makeLegacyRig();
		const RetargetChains chains= resolveRetargetChains(config, rig);
		bool bArms= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const RetargetChains::Arm& arm= chains.arms[sideIndex];
			bArms&= arm.mapping == eArmMapping::BoneAngles && sameParameter(arm.ikWeight, ExprParameter{0.f}) &&
				sameParameter(arm.reachScale, ExprParameter{1.f}) &&
				arm.elbowHintConfidence == rig.sides[sideIndex].elbowHintConfidence &&
				arm.loss.hold == eLossHold::Pipeline && arm.loss.holdMs == 400.f &&
				arm.loss.after == eLossAfter::EaseToRest && arm.loss.restBlendMs == 500.f;
		}
		check(bArms, "(d) direct: the arms take bone angles at IK weight 0, the pipeline's hold, the ease to rest over "
					 "the sidecar's blend, and each side's elbow hint confidence");
		check(chains.head.loss.hold == eLossHold::Pipeline && chains.head.loss.after == eLossAfter::EaseToRest &&
				  chains.torso.loss.hold == eLossHold::Pipeline && chains.torso.loss.after == eLossAfter::EaseToRest,
			  "(d) direct: the head and the torso take the direct policy's");
		const RetargetStageConfig stage= makeRetargetStageConfig(config, rig, nullptr);
		check(samePolicy(stage.directLoss, makeDirectLossPolicy(500.f)),
			  "(d) direct: the direct pipeline's loss rules are the ones the legacy settings built");
		// The position pipeline, which a direct project runs on a frame the
		// direct solve cannot, takes the stored hold length and its default
		// freeze in place of the ease to rest
		check(samePolicy(stage.positionLoss, makePositionLossPolicy(0.2f, 400.f, true)),
			  "(d) direct: the position fallback holds 400 ms then freezes, the ease to rest taking its default");
	}

	// (e) A block takes the place of the legacy keys: it decides over them
	// once present, the project writes it and no longer writes them, a file
	// with both reads the block, and a project without one writes the legacy
	// keys and no block
	{
		AppConfig config= makeLegacyConfig(false);
		const AvatarRigSettings rig= makeLegacyRig();
		RetargetChains block= makeDefaultRetargetChains(false);
		block.arms[0].loss.holdMs= 100.f;
		block.arms[1].loss.holdMs= 100.f;
		block.arms[0].elbowHintConfidence= 0.9f;
		config.retargetChains= std::make_shared<const RetargetChains>(block);
		check(sameChains(resolveRetargetChains(config, rig), block), "(e) the block decides over the legacy settings");
		const RetargetStageConfig stage= makeRetargetStageConfig(config, rig, nullptr);
		check(samePolicy(stage.positionLoss, makePositionLossPolicy(0.2f, 100.f, true)) &&
				  stage.position.sides[0].elbowHintConfidence == 0.9f &&
				  stage.position.sides[1].elbowHintConfidence == 0.5f,
			  "(e) the stage takes the block's hold, freeze and elbow hint confidences");

		nlohmann::json written= nlohmann::json::parse(config.toJsonString());
		check(written.contains("retarget") && written["retarget"].contains("chains") &&
				  !written["osc"].contains("holdOnDropoutMs") && !written["osc"].contains("vmcFreezeOnLoss") &&
				  written["osc"].contains("minConfidence"),
			  "(e) a project with a block writes it and not the legacy loss keys");
		written["osc"]["holdOnDropoutMs"]= 999.f;
		written["osc"]["vmcFreezeOnLoss"]= false;
		AppConfig reloaded;
		check(reloaded.loadFromJsonString(written.dump()) && reloaded.retargetChains != nullptr &&
				  sameChains(resolveRetargetChains(reloaded, rig), block),
			  "(e) a file with both the block and the legacy keys reads the block");

		AppConfig legacy= makeLegacyConfig(false);
		const nlohmann::json legacyWritten= nlohmann::json::parse(legacy.toJsonString());
		AppConfig legacyReloaded;
		check(!legacyWritten.contains("retarget") && legacyWritten["osc"].value("holdOnDropoutMs", 0.f) == 400.f &&
				  legacyWritten["osc"].value("vmcFreezeOnLoss", true) == false &&
				  legacyReloaded.loadFromJsonString(legacyWritten.dump()) && legacyReloaded.retargetChains == nullptr,
			  "(e) a project without a block writes the legacy keys and reads back without one");

		// The form a thread holding its own snapshot writes: that block, or
		// none, whatever the config holds
		const std::shared_ptr<const RetargetChains> snapshot=
			std::make_shared<const RetargetChains>(resolveRetargetChains(legacy, rig));
		AppConfig fromSnapshot;
		check(fromSnapshot.loadFromJsonString(legacy.toJsonString(snapshot)) && fromSnapshot.retargetChains != nullptr &&
				  sameChains(*fromSnapshot.retargetChains, *snapshot) &&
				  !nlohmann::json::parse(config.toJsonString(nullptr)).contains("retarget"),
			  "(e) a snapshot written in place of the config's block reads back as that block");
		// A literal header from a take made before chain settings existed
		AppConfig olderTake;
		const char* olderHeaderConfig= R"({"direct": {"enabled": false},)"
									   R"( "osc": {"minConfidence": 0.2, "holdOnDropoutMs": 400, "vmcFreezeOnLoss": false}})";
		check(olderTake.loadFromJsonString(olderHeaderConfig) &&
				  olderTake.retargetChains == nullptr &&
				  samePolicy(makeRetargetStageConfig(olderTake, rig, nullptr).positionLoss,
							 makePositionLossPolicy(0.2f, 400.f, false)),
			  "(e) an older take's config without a block runs on the migration of its own legacy settings");
	}

	// (f) The stage config each pipeline derives from a block written under
	// the other shape: kinds the pipeline cannot perform take its defaults,
	// the stored numbers are kept, and the block is left as stored
	{
		const AvatarRigSettings rig= makeLegacyRig();
		{
			// A direct project's block, under a multi-camera project
			AppConfig config= makeLegacyConfig(false);
			RetargetChains block= migrateRetargetChains(true, 400.f, false, 500.f, {0.3f, 0.7f});
			block.arms[1].loss.holdMs= 180.f;
			config.retargetChains= std::make_shared<const RetargetChains>(block);
			const RetargetStageConfig stage= makeRetargetStageConfig(config, rig, nullptr);
			LossPolicy expected= makePositionLossPolicy(0.2f, 400.f, true);
			expected.arms[1].holdMs= 180.f;
			check(samePolicy(stage.positionLoss, expected),
				  "(f) a direct block under the position pipeline: the arms' own hold becomes a timed one of the "
				  "stored length and the ease to rest a freeze, and the head is released at once");
			check(sameChains(*config.retargetChains, block), "(f) the stored block is left as it was");
		}
		{
			// A multi-camera project's block, under a direct project, with
			// a rest blend that differs per arm
			AppConfig config= makeLegacyConfig(true);
			RetargetChains block= migrateRetargetChains(false, 400.f, false, 500.f, {0.3f, 0.7f});
			block.arms[1].loss.restBlendMs= 120.f;
			block.head.loss= {eLossHold::Timed, 200.f, eLossAfter::Freeze, 0.f};
			config.retargetChains= std::make_shared<const RetargetChains>(block);
			const RetargetStageConfig stage= makeRetargetStageConfig(config, rig, nullptr);
			LossPolicy expected= makeDirectLossPolicy(500.f);
			expected.arms[1].restBlendMs= 120.f;
			check(samePolicy(stage.directLoss, expected),
				  "(f) a multi-camera block under the direct pipeline: its own hold then the ease to rest on every "
				  "chain, each arm over its own rest blend");
			LossPolicy expectedPosition= makePositionLossPolicy(0.2f, 400.f, false);
			expectedPosition.head.hold= eLossHold::Timed;
			expectedPosition.head.holdMs= 200.f;
			expectedPosition.head.after= eLossAfter::Freeze;
			check(samePolicy(stage.positionLoss, expectedPosition),
				  "(f) the position pipeline performs a stored timed hold and freeze, on the head too");
			check(RetargetStage::getArmIkWeight(SourcePose::ePipeline::Position) == 1.f &&
					  RetargetStage::getArmIkWeight(SourcePose::ePipeline::Direct) == 0.f &&
					  block.arms[0].ikWeight.value == 1.f,
				  "(f) the target solve's IK weight stays fixed by the pipeline whatever the block stores");
		}
	}

	// (g) The sidecar is untouched by a project's chains: one passed through
	// a load and a save beside a project with a chain block comes out the
	// same bytes as one saved beside a project without
	{
		AvatarRigSettings rig= makeLegacyRig();
		rig.sides[1].curlGain= 0.8f;
		rig.headLimitDegrees= 60.f;
		const std::string original= avatarRigToJson(rig);

		auto passThrough= [&original](const AppConfig& config) {
			AvatarRigSettings loaded;
			std::string error;
			if (!avatarRigFromJson(original, loaded, error))
				return std::string();
			// Everything the project builds from the rig and its chains
			const RetargetChains chains= resolveRetargetChains(config, loaded);
			makeRetargetStageConfig(config, loaded, chains, nullptr);
			makeAvatarRetargetConfig(config, loaded);
			makeDirectRetargetConfig(config, loaded);
			return avatarRigToJson(loaded);
		};
		AppConfig withBlock= makeLegacyConfig(false);
		RetargetChains block= makeEditedChains();
		withBlock.retargetChains= std::make_shared<const RetargetChains>(block);
		const AppConfig withoutBlock= makeLegacyConfig(false);
		const std::string savedWith= passThrough(withBlock);
		const std::string savedWithout= passThrough(withoutBlock);
		check(!savedWith.empty() && savedWith == savedWithout && savedWith == original,
			  "(g) a sidecar saved beside a project with a chain block is byte-identical to one saved without");
	}

	MIKAN_LOG_INFO("test-retarget-chains") << (failures == 0 ? "ALL PASSED" : "FAILURES") << " (" << failures
										   << " failed)";
	return failures == 0 ? 0 : 1;
}
MIKAN_REGISTER_TEST("--test-retarget-chains",
					"Retarget chains: JSON round trip, tolerant loading, migration, precedence over the legacy keys, "
					"the stage config per pipeline, the sidecar untouched",
					eTestCategory::SelfTest, runRetargetChainsTest);
