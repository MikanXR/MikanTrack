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
#include "WebcamFace.h"

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

// The webcam's raw column inside Auto's dead zone and smoothing. The dead
// zone clears the column's scatter at rest and its drift as the head turns,
// so landmark noise does not open a silent mouth through the max.
std::string webcamTerm(const char* arkitName)
{
	struct Shaping
	{
		const char* prefix;
		const char* deadZone;
		const char* tauSeconds;
	};
	static const Shaping kShapings[]= {
		{"eyeBlink", "0.5", "0.05"}, {"jawOpen", "0.12", "0.08"},     {"mouthSmile", "0.2", "0.08"},
		{"mouthPucker", "0.2", "0.08"}, {"brow", "0.25", "0.08"},
	};
	const Shaping* shaping= &kShapings[1];
	for (const Shaping& candidate : kShapings)
	{
		if (strncmp(arkitName, candidate.prefix, strlen(candidate.prefix)) == 0)
			shaping= &candidate;
	}
	return std::string("smooth(remap(webcam.") + arkitName + ", " + shaping->deadZone + ", 1), " + shaping->tauSeconds +
		")";
}

bool inMask(uint64_t mask, const char* arkitName)
{
	return (mask >> arkitBlendshapeFromName(arkitName)) & 1ull;
}

struct PresetRule
{
	const char* preset;
	// The one column, or the two it averages
	const char* first;
	const char* second= nullptr;
	// The viseme a vowel reads while the phone sees no face; -1 for the rest
	int viseme= -1;
};

std::vector<PresetRule> makePresetRules()
{
	return {
		{"blinkLeft", "eyeBlinkLeft"},
		{"blinkRight", "eyeBlinkRight"},
		{"aa", "jawOpen", nullptr, (int)eViseme::aa},
		{"ih", "mouthSmileLeft", "mouthSmileRight", (int)eViseme::I},
		{"ee", "mouthStretchLeft", "mouthStretchRight", (int)eViseme::E},
		{"ou", "mouthPucker", nullptr, (int)eViseme::U},
		{"oh", "mouthFunnel", nullptr, (int)eViseme::O},
		{"lookUp", "eyeLookUpLeft", "eyeLookUpRight"},
		{"lookDown", "eyeLookDownLeft", "eyeLookDownRight"},
		// The person's left: the left eye looks out, the right eye in
		{"lookLeft", "eyeLookOutLeft", "eyeLookInRight"},
		{"lookRight", "eyeLookInLeft", "eyeLookOutRight"},
	};
}

// The phone's side of a rule, then what stands in while the phone sees no
// face: the webcam where the mask holds the rule's columns, the viseme where
// the rule has one, and their max where it has both
std::string presetExpr(const PresetRule& rule, uint64_t webcamMask)
{
	const std::string phoneExpr= rule.second == nullptr ? phone(rule.first) : average(rule.first, rule.second);
	std::string webcamExpr;
	if (inMask(webcamMask, rule.first) && (rule.second == nullptr || inMask(webcamMask, rule.second)))
	{
		webcamExpr= rule.second == nullptr
			? webcamTerm(rule.first)
			: "0.5 * " + webcamTerm(rule.first) + " + 0.5 * " + webcamTerm(rule.second);
	}
	const std::string visemeExpr= rule.viseme < 0 ? "" : std::string("viseme.") + visemeName(rule.viseme);

	if (webcamExpr.empty() && visemeExpr.empty())
		return phoneExpr;
	if (webcamExpr.empty())
		return "select(phone.tracked, " + phoneExpr + ", " + visemeExpr + ")";
	if (visemeExpr.empty())
		return "select(phone.tracked, " + phoneExpr + ", " + webcamExpr + ")";
	return "select(phone.tracked, " + phoneExpr + ", max(" + webcamExpr + ", " + visemeExpr + "))";
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

uint64_t autoFaceWebcamMask()
{
	const uint64_t blinks=
		(1ull << arkitBlendshapeFromName("eyeBlinkLeft")) | (1ull << arkitBlendshapeFromName("eyeBlinkRight"));
	return webcamFaceMeasuredMask() & ~blinks;
}

std::string makeAutoArkitExpr(int arkitIndex)
{
	return makeAutoArkitExpr(arkitIndex, autoFaceWebcamMask());
}

std::string makeAutoArkitExpr(int arkitIndex, uint64_t webcamMask)
{
	const std::string phoneTerm= phone(arkitBlendshapeName(arkitIndex));
	const bool bWebcam= (webcamMask >> arkitIndex) & 1ull;
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
	if (!bWebcam)
	{
		if (visemeTerms.empty())
			return phoneTerm;
		return "select(phone.tracked, " + phoneTerm + ", " + visemeTerms + ")";
	}
	const std::string webcam= webcamTerm(arkitBlendshapeName(arkitIndex));
	if (visemeTerms.empty())
		return "select(phone.tracked, " + phoneTerm + ", " + webcam + ")";
	return "select(phone.tracked, " + phoneTerm + ", max(" + webcam + ", " + visemeTerms + "))";
}

FaceProfile makeAutoFaceProfile(const AvatarModel* model)
{
	return makeAutoFaceProfile(model, autoFaceWebcamMask());
}

FaceProfile makeAutoFaceProfile(const AvatarModel* model, uint64_t webcamMask)
{
	FaceProfile profile;
	profile.id= k_autoFaceProfileId;
	profile.name= "Auto";

	if (model == nullptr)
	{
		for (int index= 0; index < ARKIT_BLENDSHAPE_COUNT; ++index)
			profile.outputs.push_back(makeOutput(eFaceTargetKind::Morph, arkitBlendshapeName(index), makeAutoArkitExpr(index, webcamMask)));
		return profile;
	}

	for (const AvatarExpression& expression : model->expressions)
	{
		const int arkitIndex= arkitIndexIgnoringCase(expression.name);
		if (arkitIndex >= 0)
			profile.outputs.push_back(makeOutput(eFaceTargetKind::Expression, expression.name, makeAutoArkitExpr(arkitIndex, webcamMask)));
	}
	if (!profile.outputs.empty())
		return profile;

	for (const PresetRule& rule : makePresetRules())
	{
		if (const AvatarExpression* expression= model->findExpressionByPreset(rule.preset))
			profile.outputs.push_back(makeOutput(eFaceTargetKind::Expression, expression->name, presetExpr(rule, webcamMask)));
	}
	// The two-eyed blink only when the avatar cannot blink each eye on its
	// own: driving both would close the eyes twice
	const AvatarExpression* blink= model->findExpressionByPreset("blink");
	const bool bHasSidedBlinks=
		model->findExpressionByPreset("blinkLeft") != nullptr && model->findExpressionByPreset("blinkRight") != nullptr;
	if (blink != nullptr && !bHasSidedBlinks)
	{
		const PresetRule bothEyes{"blink", "eyeBlinkLeft", "eyeBlinkRight"};
		profile.outputs.push_back(
			makeOutput(eFaceTargetKind::Expression, blink->name, presetExpr(bothEyes, webcamMask)));
	}

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
				profile.outputs.push_back(makeOutput(eFaceTargetKind::Morph, morphName, makeAutoArkitExpr(arkitIndex, webcamMask)));
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
