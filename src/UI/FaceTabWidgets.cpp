#include "FaceTabWidgets.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <unordered_map>
#include <vector>

#include "misc/cpp/imgui_stdlib.h"

#include "App.h"
#include "AvatarTypes.h"
#include "FaceProfileBinding.h"
#include "LocText.h"

namespace
{
bool containsIgnoringCase(const std::string& text, const std::string& filter)
{
	if (filter.empty())
		return true;
	auto it= std::search(text.begin(), text.end(), filter.begin(), filter.end(), [](char a, char b) {
		return std::tolower((unsigned char)a) == std::tolower((unsigned char)b);
	});
	return it != text.end();
}
} // namespace

namespace FaceTabWidgets
{
std::string errorText(const FaceExprCompileError& error)
{
	std::array<std::string, 4> args;
	for (size_t index= 0; index < args.size() && index < error.args.size(); ++index)
		args[index]= error.args[index];

	if (error.code == eFaceExprErrorCode::ArgumentCount)
	{
		// The function, its minimum, its maximum ("" unbounded), the count
		// given
		const std::string& function= args[0];
		const std::string& minimum= args[1];
		const std::string& maximum= args[2];
		const std::string& given= args[3];
		if (maximum.empty())
			return locFormat("faceTab.error.argumentCountAtLeast", function.c_str(), minimum.c_str(), given.c_str());
		if (maximum == minimum)
			return locFormat("faceTab.error.argumentCountExact", function.c_str(), minimum.c_str(), given.c_str());
		return locFormat("faceTab.error.argumentCountEither", function.c_str(), minimum.c_str(), maximum.c_str(),
						 given.c_str());
	}

	const char* key= nullptr;
	switch (error.code)
	{
	case eFaceExprErrorCode::UnexpectedCharacter:
		key= "faceTab.error.unexpectedCharacter";
		break;
	case eFaceExprErrorCode::UnexpectedToken:
		key= "faceTab.error.unexpectedToken";
		break;
	case eFaceExprErrorCode::ExpressionEndsEarly:
		key= "faceTab.error.expressionEndsEarly";
		break;
	case eFaceExprErrorCode::ExpectedCloseParen:
		key= "faceTab.error.expectedCloseParen";
		break;
	case eFaceExprErrorCode::ExpectedCommaOrParen:
		key= "faceTab.error.expectedCommaOrParen";
		break;
	case eFaceExprErrorCode::UnknownInput:
		key= "faceTab.error.unknownInput";
		break;
	case eFaceExprErrorCode::UnknownName:
		key= "faceTab.error.unknownName";
		break;
	case eFaceExprErrorCode::UnknownFunction:
		key= "faceTab.error.unknownFunction";
		break;
	case eFaceExprErrorCode::InvalidVariableName:
		key= "faceTab.error.invalidVariableName";
		break;
	case eFaceExprErrorCode::VariableIsFunctionName:
		key= "faceTab.error.variableIsFunctionName";
		break;
	case eFaceExprErrorCode::VariableDefinedTwice:
		key= "faceTab.error.variableDefinedTwice";
		break;
	case eFaceExprErrorCode::Cycle:
		key= "faceTab.error.cycle";
		break;
	case eFaceExprErrorCode::VariableHasError:
		key= "faceTab.error.variableHasError";
		break;
	default:
		return error.message;
	}
	// A key reads only the args its code carries; the rest are ignored
	return locFormat(key, args[0].c_str(), args[1].c_str(), args[2].c_str(), args[3].c_str());
}

bool textInputWithError(const char* id, std::string& text, const FaceExprCompileError* error)
{
	ImGui::SetNextItemWidth(-FLT_MIN);
	const bool bChanged= ImGui::InputText(id, &text);
	if (error != nullptr)
	{
		const ImVec2 rectMin= ImGui::GetItemRectMin();
		const ImVec2 rectMax= ImGui::GetItemRectMax();
		const float textX= rectMin.x + ImGui::GetStyle().FramePadding.x;
		const int begin= std::clamp(error->span.begin, 0, (int)text.size());
		const int end= std::clamp(std::max(error->span.end, error->span.begin + 1), begin, (int)text.size() + 1);
		const float x0= textX + ImGui::CalcTextSize(text.c_str(), text.c_str() + begin).x;
		float x1= textX + ImGui::CalcTextSize(text.c_str(), text.c_str() + std::min(end, (int)text.size())).x;
		if (x1 <= x0)
			x1= x0 + ImGui::CalcTextSize(" ").x;
		const ImU32 color= ImGui::GetColorU32(k_errorColor);
		ImGui::GetWindowDrawList()->AddLine(ImVec2(x0, rectMax.y - 2.f), ImVec2(std::min(x1, rectMax.x), rectMax.y - 2.f),
											color, 2.f);
		ImGui::PushStyleColor(ImGuiCol_Text, k_errorColor);
		ImGui::TextWrapped("%s", errorText(*error).c_str());
		ImGui::PopStyleColor();
	}
	return bChanged;
}

std::string problemText(const FaceProfileBinding& binding, size_t outputIndex)
{
	const FaceProfileOutput& output= binding.getProfile().outputs[outputIndex];
	switch (binding.getOutputProblemCodes()[outputIndex])
	{
	case eFaceTargetProblem::NoTarget:
		return locText("faceTab.problem.noTarget");
	case eFaceTargetProblem::NoExpression:
		return locFormat("faceTab.problem.noExpression", output.target.c_str());
	case eFaceTargetProblem::NoMesh:
		return locFormat("faceTab.problem.noMesh", output.mesh.c_str());
	case eFaceTargetProblem::NoMorph:
		return locFormat("faceTab.problem.noMorph", output.target.c_str());
	default:
		return std::string();
	}
}

std::string targetLabel(const FaceProfileOutput& output)
{
	if (output.target.empty())
		return locText("faceTab.noTargetText");
	std::string label= output.target;
	if (output.targetKind == eFaceTargetKind::Morph)
		label+= output.mesh.empty() ? std::string(" [") + locText("faceTab.kindMorph") + "]"
									: " [" + output.mesh + "]";
	return label;
}

bool drawTargetPicker(App* app, FaceProfileOutput& output, std::string& filter)
{
	bool bChanged= false;
	const std::shared_ptr<const AvatarModel> model= app->getAvatarModel();
	ImGui::SetNextItemWidth(-FLT_MIN);
	if (ImGui::BeginCombo("##target", targetLabel(output).c_str(), ImGuiComboFlags_HeightLarge))
	{
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::InputTextWithHint("##filter", locText("faceTab.targetFilterHint"), &filter);
		if (model != nullptr)
		{
			ImGui::SeparatorText(locText("faceTab.expressionsHeader"));
			for (const AvatarExpression& expression : model->expressions)
			{
				if (!containsIgnoringCase(expression.name, filter))
					continue;
				const bool bSelected= output.targetKind == eFaceTargetKind::Expression && output.target == expression.name;
				const std::string label=
					expression.preset.empty() ? expression.name : expression.name + "  (" + expression.preset + ")";
				if (ImGui::Selectable(label.c_str(), bSelected))
				{
					output.targetKind= eFaceTargetKind::Expression;
					output.target= expression.name;
					output.mesh.clear();
					bChanged= true;
				}
			}
			ImGui::SeparatorText(locText("faceTab.morphsHeader"));
			// Each morph name once, in first-seen order, with the meshes
			// carrying it
			std::vector<std::pair<const std::string*, std::vector<int>>> morphs;
			std::unordered_map<std::string, size_t> morphIndices;
			for (size_t meshIndex= 0; meshIndex < model->meshes.size(); ++meshIndex)
			{
				for (const std::string& morphName : model->meshes[meshIndex].morphTargetNames)
				{
					const auto inserted= morphIndices.emplace(morphName, morphs.size());
					if (inserted.second)
						morphs.emplace_back(&morphName, std::vector<int>());
					morphs[inserted.first->second].second.push_back((int)meshIndex);
				}
			}

			const bool bMorph= output.targetKind == eFaceTargetKind::Morph;
			for (size_t morphIndex= 0; morphIndex < morphs.size(); ++morphIndex)
			{
				const std::string& morphName= *morphs[morphIndex].first;
				const std::vector<int>& meshIndices= morphs[morphIndex].second;
				if (!containsIgnoringCase(morphName, filter))
					continue;
				ImGui::PushID((int)morphIndex);
				// The name alone is that morph on every mesh carrying it
				if (ImGui::Selectable(morphName.c_str(), bMorph && output.target == morphName && output.mesh.empty()))
				{
					output.targetKind= eFaceTargetKind::Morph;
					output.target= morphName;
					output.mesh.clear();
					bChanged= true;
				}
				// A name several meshes carry can also be narrowed to one
				if (meshIndices.size() > 1)
				{
					for (int meshIndex : meshIndices)
					{
						const AvatarMesh& mesh= model->meshes[(size_t)meshIndex];
						const std::string label=
							morphName + " (" + (mesh.name.empty() ? locText("faceTab.unnamedMesh") : mesh.name) + ")";
						ImGui::PushID(meshIndex);
						if (ImGui::Selectable(label.c_str(),
											  bMorph && output.target == morphName && output.mesh == mesh.name))
						{
							output.targetKind= eFaceTargetKind::Morph;
							output.target= morphName;
							output.mesh= mesh.name;
							bChanged= true;
						}
						ImGui::PopID();
					}
				}
				ImGui::PopID();
			}
		}
		ImGui::EndCombo();
	}
	return bChanged;
}
} // namespace FaceTabWidgets
