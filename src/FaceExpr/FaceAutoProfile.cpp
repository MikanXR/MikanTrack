#include "FaceAutoProfile.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>

#include "AvatarTypes.h"
#include "FaceProfileBinding.h"
#include "Logger.h"
#include "FaceTypes.h"
#include "VisemeTable.h"
#include "VisemeTypes.h"

namespace
{
// ARKit's own spelling only, compared without case: avatars spell perfect
// sync names both "eyeBlinkLeft" and "EyeBlinkLeft"
int arkitIndexIgnoringCase(const std::string& name)
{
	for (int index= 0; index < ARKIT_BLENDSHAPE_COUNT; ++index)
	{
		const char* arkitName= arkitBlendshapeName(index);
		if (name.size() != strlen(arkitName))
			continue;

		bool bMatch= true;
		for (size_t charIndex= 0; charIndex < name.size() && bMatch; ++charIndex)
			bMatch= std::tolower((unsigned char)name[charIndex]) == std::tolower((unsigned char)arkitName[charIndex]);
		if (bMatch)
			return index;
	}
	return -1;
}

std::string formatWeight(float weight)
{
	char text[32];
	snprintf(text, sizeof(text), "%g", weight);
	return text;
}

std::string phone(const char* arkitName)
{
	return std::string("phone.") + arkitName;
}

// A pair of columns averaged, the shape of every two-sided preset
std::string average(const char* first, const char* second)
{
	return "0.5 * " + phone(first) + " + 0.5 * " + phone(second);
}

struct PresetRule
{
	const char* preset;
	// The phone's side
	std::string phoneExpr;
	// The viseme a vowel reads while the phone sees no face; -1 for the rest
	int viseme= -1;
};

std::vector<PresetRule> makePresetRules()
{
	return {
		{"blinkLeft", phone("eyeBlinkLeft")},
		{"blinkRight", phone("eyeBlinkRight")},
		{"aa", phone("jawOpen"), (int)eViseme::aa},
		{"ih", average("mouthSmileLeft", "mouthSmileRight"), (int)eViseme::I},
		{"ee", average("mouthStretchLeft", "mouthStretchRight"), (int)eViseme::E},
		{"ou", phone("mouthPucker"), (int)eViseme::U},
		{"oh", phone("mouthFunnel"), (int)eViseme::O},
		{"lookUp", average("eyeLookUpLeft", "eyeLookUpRight")},
		{"lookDown", average("eyeLookDownLeft", "eyeLookDownRight")},
		// The person's left: the left eye looks out, the right eye in
		{"lookLeft", average("eyeLookOutLeft", "eyeLookInRight")},
		{"lookRight", average("eyeLookInLeft", "eyeLookOutRight")},
	};
}

std::string presetExpr(const PresetRule& rule)
{
	if (rule.viseme < 0)
		return rule.phoneExpr;
	return "select(phone.tracked, " + rule.phoneExpr + ", viseme." + visemeName(rule.viseme) + ")";
}

FaceProfileOutput makeOutput(eFaceTargetKind kind, const std::string& target, const std::string& expr)
{
	FaceProfileOutput output;
	output.targetKind= kind;
	output.target= target;
	output.vrmExpressionFunction= expr;
	return output;
}
} // namespace

bool isAutoFaceProfileId(const std::string& id)
{
	return id == k_autoFaceProfileId;
}

std::string makeAutoArkitExpr(int arkitIndex)
{
	const std::string phoneTerm= phone(arkitBlendshapeName(arkitIndex));
	std::string visemeTerms;
	for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
	{
		for (const VisemeTerm& term : visemeTableRow(viseme))
		{
			if (term.arkitIndex != arkitIndex)
				continue;
			if (!visemeTerms.empty())
				visemeTerms+= " + ";
			visemeTerms+= formatWeight(term.weight) + " * viseme." + visemeName(viseme);
		}
	}
	if (visemeTerms.empty())
		return phoneTerm;
	return "select(phone.tracked, " + phoneTerm + ", " + visemeTerms + ")";
}

FaceProfile makeAutoFaceProfile(const AvatarModel* model)
{
	FaceProfile profile;
	profile.id= k_autoFaceProfileId;
	profile.name= "Auto";

	if (model == nullptr)
	{
		for (int index= 0; index < ARKIT_BLENDSHAPE_COUNT; ++index)
			profile.outputs.push_back(makeOutput(eFaceTargetKind::Morph, arkitBlendshapeName(index), makeAutoArkitExpr(index)));
		return profile;
	}

	for (const AvatarExpression& expression : model->expressions)
	{
		const int arkitIndex= arkitIndexIgnoringCase(expression.name);
		if (arkitIndex >= 0)
			profile.outputs.push_back(makeOutput(eFaceTargetKind::Expression, expression.name, makeAutoArkitExpr(arkitIndex)));
	}
	if (!profile.outputs.empty())
		return profile;

	for (const PresetRule& rule : makePresetRules())
	{
		if (const AvatarExpression* expression= model->findExpressionByPreset(rule.preset))
			profile.outputs.push_back(makeOutput(eFaceTargetKind::Expression, expression->name, presetExpr(rule)));
	}
	// The two-eyed blink only when the avatar cannot blink each eye on its
	// own: driving both would close the eyes twice
	const AvatarExpression* blink= model->findExpressionByPreset("blink");
	const bool bHasSidedBlinks=
		model->findExpressionByPreset("blinkLeft") != nullptr && model->findExpressionByPreset("blinkRight") != nullptr;
	if (blink != nullptr && !bHasSidedBlinks)
		profile.outputs.push_back(makeOutput(eFaceTargetKind::Expression, blink->name, average("eyeBlinkLeft", "eyeBlinkRight")));

	for (const AvatarMesh& mesh : model->meshes)
	{
		for (const std::string& morphName : mesh.morphTargetNames)
		{
			const int arkitIndex= arkitIndexIgnoringCase(morphName);
			if (arkitIndex < 0)
				continue;

			// One output per name: a morph on several meshes is one target,
			// and a name an expression already sends is not sent twice
			const bool bAlreadyMapped=
				std::any_of(profile.outputs.begin(), profile.outputs.end(),
							[&morphName](const FaceProfileOutput& output) { return output.target == morphName; });
			if (!bAlreadyMapped)
				profile.outputs.push_back(makeOutput(eFaceTargetKind::Morph, morphName, makeAutoArkitExpr(arkitIndex)));
		}
	}

	return profile;
}

std::shared_ptr<const FaceProfileBinding> makeAutoFaceBinding(const AvatarModel* model)
{
	std::vector<FaceExprCompileError> errors;
	std::shared_ptr<const FaceProfileBinding> binding= compileFaceBinding(makeAutoFaceProfile(model), model, &errors);
	// The generator writes only names the namespaces hold, so this is a bug
	// in the generator rather than a state to recover from
	for (const FaceExprCompileError& error : errors)
		MIKAN_LOG_ERROR("FaceAutoProfile") << "Auto profile output " << error.index << ": " << error.message;
	return binding;
}
