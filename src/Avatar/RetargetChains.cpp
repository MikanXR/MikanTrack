#include "RetargetChains.h"

#include <sstream>

#include "nlohmann/json.hpp"

#include "Logger.h"

using json= nlohmann::json;

namespace
{
const char* k_logLabel= "RetargetChains";
const char* k_armKeys[2]= {"leftArm", "rightArm"};
const char* k_handKeys[2]= {"leftHand", "rightHand"};

const eArmMapping k_mappings[]= {eArmMapping::BoneAngles, eArmMapping::BoneAnglesThenIk};
const eLossHold k_holds[]= {eLossHold::None, eLossHold::Pipeline, eLossHold::Timed};
const eLossAfter k_afters[]= {eLossAfter::Release, eLossAfter::Freeze, eLossAfter::EaseToRest};

// Reads the fields of one chain, each falling back on its own default and
// naming itself in the defaulted list when it does
class FieldReader
{
public:
	FieldReader(const json& object, std::string path, std::vector<std::string>& defaulted)
		: m_object(object), m_path(std::move(path)), m_defaulted(defaulted)
	{
	}

	const json* find(const char* key) const
	{
		if (!m_object.is_object())
			return nullptr;
		const auto it= m_object.find(key);
		return it != m_object.end() ? &*it : nullptr;
	}

	void markDefaulted(const char* key) { m_defaulted.push_back(m_path + "." + key); }

	float readFloat(const char* key, float fallback)
	{
		const json* value= find(key);
		if (value != nullptr && value->is_number())
			return value->get<float>();
		markDefaulted(key);
		return fallback;
	}

	glm::vec3 readVec3(const char* key, const glm::vec3& fallback)
	{
		const json* value= find(key);
		if (value != nullptr && value->is_array() && value->size() == 3 && (*value)[0].is_number() &&
			(*value)[1].is_number() && (*value)[2].is_number())
		{
			return glm::vec3((*value)[0].get<float>(), (*value)[1].get<float>(), (*value)[2].get<float>());
		}
		markDefaulted(key);
		return fallback;
	}

	ExprParameter readParameter(const char* key, const ExprParameter& fallback)
	{
		const json* value= find(key);
		if (value != nullptr && (value->is_number() || value->is_object()))
			return exprParameterFromJson(*value, fallback.value);
		markDefaulted(key);
		return fallback;
	}

	template <typename Enum, size_t Count>
	Enum readEnum(const char* key, const Enum (&values)[Count], const char* (*nameOf)(Enum), Enum fallback)
	{
		const json* value= find(key);
		if (value != nullptr && value->is_string())
		{
			const std::string text= value->get<std::string>();
			for (Enum candidate : values)
			{
				if (text == nameOf(candidate))
					return candidate;
			}
		}
		markDefaulted(key);
		return fallback;
	}

	const std::string& getPath() const { return m_path; }

private:
	const json& m_object;
	std::string m_path;
	std::vector<std::string>& m_defaulted;
};

json lossToJson(const RetargetChains::Loss& loss)
{
	return {
		{"hold", lossHoldName(loss.hold)},
		{"holdMs", loss.holdMs},
		{"after", lossAfterName(loss.after)},
		{"restBlendMs", loss.restBlendMs},
	};
}

RetargetChains::Loss lossFromJson(FieldReader& chain, const RetargetChains::Loss& defaults,
								  std::vector<std::string>& defaulted)
{
	const json* lossJson= chain.find("loss");
	if (lossJson == nullptr || !lossJson->is_object())
	{
		chain.markDefaulted("loss");
		return defaults;
	}
	FieldReader reader(*lossJson, chain.getPath() + ".loss", defaulted);
	RetargetChains::Loss loss;
	loss.hold= reader.readEnum("hold", k_holds, lossHoldName, defaults.hold);
	loss.holdMs= reader.readFloat("holdMs", defaults.holdMs);
	loss.after= reader.readEnum("after", k_afters, lossAfterName, defaults.after);
	loss.restBlendMs= reader.readFloat("restBlendMs", defaults.restBlendMs);
	return loss;
}

// A chain object, or null with the chain named as defaulted
const json* findChain(const json& j, const char* key, std::vector<std::string>& defaulted)
{
	if (j.is_object())
	{
		const auto it= j.find(key);
		if (it != j.end() && it->is_object())
			return &*it;
	}
	defaulted.push_back(key);
	return nullptr;
}

RetargetChains::Loss migratedGroupLoss(bool bDirect)
{
	// The direct pipeline holds its torso and head for its own hold, then
	// rests them; the position pipeline releases its head at once and carries
	// its torso with the arms
	RetargetChains::Loss loss;
	loss.hold= bDirect ? eLossHold::Pipeline : eLossHold::None;
	loss.after= bDirect ? eLossAfter::EaseToRest : eLossAfter::Release;
	return loss;
}

// The position pipeline performs no hold or a timed one and a release or a
// freeze; any other stored kind keeps the default already in ioPolicy
void applyPositionLoss(const RetargetChains::Loss& stored, ChainLossPolicy& ioPolicy)
{
	if (stored.hold == eLossHold::None || stored.hold == eLossHold::Timed)
		ioPolicy.hold= stored.hold;
	ioPolicy.holdMs= stored.holdMs;
	if (stored.after == eLossAfter::Release || stored.after == eLossAfter::Freeze)
		ioPolicy.after= stored.after;
}
} // namespace

const char* armMappingName(eArmMapping mapping)
{
	switch (mapping)
	{
	case eArmMapping::BoneAngles:
		return "boneAngles";
	case eArmMapping::BoneAnglesThenIk:
		return "boneAnglesThenIk";
	}
	return "";
}

const char* lossHoldName(eLossHold hold)
{
	switch (hold)
	{
	case eLossHold::None:
		return "none";
	case eLossHold::Pipeline:
		return "pipeline";
	case eLossHold::Timed:
		return "timed";
	}
	return "";
}

const char* lossAfterName(eLossAfter after)
{
	switch (after)
	{
	case eLossAfter::Release:
		return "release";
	case eLossAfter::Freeze:
		return "freeze";
	case eLossAfter::EaseToRest:
		return "easeToRest";
	}
	return "";
}

RetargetChains migrateRetargetChains(bool bDirect, float holdOnDropoutMs, bool bFreezeOnLoss, float restBlendMs,
									 const std::array<float, 2>& elbowHintConfidence)
{
	RetargetChains chains;
	chains.torso.loss= migratedGroupLoss(bDirect);
	chains.head.loss= migratedGroupLoss(bDirect);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		RetargetChains::Arm& arm= chains.arms[sideIndex];
		arm.mapping= bDirect ? eArmMapping::BoneAngles : eArmMapping::BoneAnglesThenIk;
		arm.ikWeight= ExprParameter{bDirect ? 0.f : 1.f};
		arm.reachScale= ExprParameter{1.f};
		arm.goalOffset= glm::vec3(0.f);
		arm.elbowHintConfidence= elbowHintConfidence[sideIndex];
		arm.loss.hold= bDirect ? eLossHold::Pipeline : eLossHold::Timed;
		arm.loss.holdMs= holdOnDropoutMs;
		arm.loss.after= bDirect ? eLossAfter::EaseToRest : (bFreezeOnLoss ? eLossAfter::Freeze : eLossAfter::Release);
		arm.loss.restBlendMs= restBlendMs;
	}
	return chains;
}

RetargetChains makeDefaultRetargetChains(bool bDirect)
{
	return migrateRetargetChains(bDirect, 250.f, true, 300.f, {0.5f, 0.5f});
}

LossPolicy makeChainLossPolicy(const RetargetChains& chains, SourcePose::ePipeline pipeline, float minConfidence)
{
	if (pipeline == SourcePose::ePipeline::Direct)
	{
		// Every chain takes the pipeline's own hold and then rests; only each
		// arm's rest blend comes from its chain
		LossPolicy policy= makeDirectLossPolicy(0.f);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			policy.arms[sideIndex].restBlendMs= chains.arms[sideIndex].loss.restBlendMs;
		return policy;
	}

	// The position pipeline's defaults (a timed hold then a freeze on the
	// arms, the head released at once, the torso and the hands with the arms)
	// under what the chains store
	LossPolicy policy= makePositionLossPolicy(minConfidence, 0.f, true);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		applyPositionLoss(chains.arms[sideIndex].loss, policy.arms[sideIndex]);
	applyPositionLoss(chains.head.loss, policy.head);
	return policy;
}

void applyRetargetChains(const RetargetChains& chains, std::array<AvatarRetargetConfig::Side, 2>& ioSides)
{
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		ioSides[sideIndex].elbowHintConfidence= chains.arms[sideIndex].elbowHintConfidence;
}

json retargetChainsToJson(const RetargetChains& chains)
{
	json j= json::object();
	j["torso"]= {{"loss", lossToJson(chains.torso.loss)}};
	j["head"]= {{"loss", lossToJson(chains.head.loss)}};
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const RetargetChains::Arm& arm= chains.arms[sideIndex];
		j[k_armKeys[sideIndex]]= {
			{"mapping", armMappingName(arm.mapping)},
			{"ikWeight", exprParameterToJson(arm.ikWeight)},
			{"reachScale", exprParameterToJson(arm.reachScale)},
			{"goalOffset", json::array({arm.goalOffset.x, arm.goalOffset.y, arm.goalOffset.z})},
			{"elbowHintConfidence", arm.elbowHintConfidence},
			{"loss", lossToJson(arm.loss)},
		};
		j[k_handKeys[sideIndex]]= json::object();
	}
	return j;
}

RetargetChains retargetChainsFromJson(const json& j, const RetargetChains& defaults,
									  std::vector<std::string>* outDefaulted)
{
	std::vector<std::string> defaulted;
	RetargetChains chains= defaults;

	if (const json* torso= findChain(j, "torso", defaulted))
	{
		FieldReader reader(*torso, "torso", defaulted);
		chains.torso.loss= lossFromJson(reader, defaults.torso.loss, defaulted);
	}
	if (const json* head= findChain(j, "head", defaulted))
	{
		FieldReader reader(*head, "head", defaulted);
		chains.head.loss= lossFromJson(reader, defaults.head.loss, defaulted);
	}
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const RetargetChains::Arm& fallback= defaults.arms[sideIndex];
		RetargetChains::Arm& arm= chains.arms[sideIndex];
		if (const json* armJson= findChain(j, k_armKeys[sideIndex], defaulted))
		{
			FieldReader reader(*armJson, k_armKeys[sideIndex], defaulted);
			arm.mapping= reader.readEnum("mapping", k_mappings, armMappingName, fallback.mapping);
			arm.ikWeight= reader.readParameter("ikWeight", fallback.ikWeight);
			arm.reachScale= reader.readParameter("reachScale", fallback.reachScale);
			arm.goalOffset= reader.readVec3("goalOffset", fallback.goalOffset);
			arm.elbowHintConfidence= reader.readFloat("elbowHintConfidence", fallback.elbowHintConfidence);
			arm.loss= lossFromJson(reader, fallback.loss, defaulted);
		}
		// A hand holds nothing yet; the object is there for what it will hold
		findChain(j, k_handKeys[sideIndex], defaulted);
	}

	if (!defaulted.empty())
	{
		std::ostringstream names;
		for (size_t index= 0; index < defaulted.size(); ++index)
			names << (index > 0 ? ", " : "") << defaulted[index];
		MIKAN_LOG_WARNING(k_logLabel) << "Took the default for the retarget chain settings missing or unreadable: "
									  << names.str();
		if (outDefaulted != nullptr)
			outDefaulted->insert(outDefaulted->end(), defaulted.begin(), defaulted.end());
	}
	return chains;
}
