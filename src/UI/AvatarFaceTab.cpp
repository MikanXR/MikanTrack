#include "AvatarFaceTab.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <unordered_map>
#include <vector>

#include "imgui.h"
#include "misc/cpp/imgui_stdlib.h"

#include "App.h"
#include "AvatarTypes.h"
#include "FaceExprCompiler.h"
#include "FaceProfileBinding.h"
#include "FaceProfileLibrary.h"
#include "LocText.h"

namespace
{
const ImVec4 k_errorColor(1.f, 0.4f, 0.4f, 1.f);
const ImVec4 k_warningColor(1.f, 0.8f, 0.4f, 1.f);

bool containsIgnoringCase(const std::string& text, const std::string& filter)
{
	if (filter.empty())
		return true;
	auto it= std::search(text.begin(), text.end(), filter.begin(), filter.end(), [](char a, char b) {
		return std::tolower((unsigned char)a) == std::tolower((unsigned char)b);
	});
	return it != text.end();
}

const FaceExprCompileError* findError(const std::vector<FaceExprCompileError>& errors, bool bLet, int index)
{
	for (const FaceExprCompileError& error : errors)
	{
		if (error.bLet == bLet && error.index == index)
			return &error;
	}
	return nullptr;
}

// A compile error in the UI's language
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

// Why a binding's output has no target, in the UI's language
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

// An InputText with its error, if any, underlined at its span and spelled
// out below
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
} // namespace

// The per-draw facts every section reads
struct AvatarFaceTab::Frame
{
	const FaceSources* face= nullptr;
	std::shared_ptr<const FaceProfileBinding> binding;
	const std::vector<FaceExprCompileError>* errors= nullptr;
	bool bReadOnly= false;
	// The binding has the edited profile's shape: its id, its output count
	// and its variable count. The binding's indices then name the edited
	// rows. A compile error does not clear it: the last good binding keeps
	// running.
	bool bStructural= false;
	// bStructural, and the frame's values came from the binding's
	// generation: the numbers belong to the rows shown
	bool bLive= false;
};

void AvatarFaceTab::draw(App* app, const FaceSources& face)
{
	drawProfileBar(app);

	m_profile= app->getActiveFaceProfile();
	FaceProfile& profile= m_profile;

	Frame frame;
	frame.face= &face;
	frame.binding= app->getFaceBinding();
	frame.errors= &app->getFaceProfileErrors();
	frame.bReadOnly= app->isActiveFaceProfileAuto();
	if (frame.binding != nullptr)
	{
		const FaceProfile& bound= frame.binding->getProfile();
		frame.bStructural= frame.binding->getProgram() != nullptr && bound.id == profile.id &&
						   bound.outputs.size() == profile.outputs.size() && bound.lets.size() == profile.lets.size();
		frame.bLive= frame.bStructural && face.outputs.generation == frame.binding->getGeneration();
	}

	bool bChanged= false;
	ImGui::BeginDisabled(frame.bReadOnly);
	bChanged|= drawLets(profile, frame);
	bChanged|= drawOutputs(app, profile, frame);
	ImGui::EndDisabled();

	if (bChanged && !frame.bReadOnly)
		app->editActiveFaceProfile(profile);
}

void AvatarFaceTab::drawProfileBar(App* app)
{
	FaceProfileLibrary* library= app->getFaceProfileLibrary();
	const AvatarRigSettings& rig= app->getAvatarRig();
	const FaceProfile& active= app->getActiveFaceProfile();
	const bool bAuto= app->isActiveFaceProfileAuto();

	// The active picker: Auto and the assigned profiles
	auto profileName= [library](const std::string& id) {
		const FaceProfile* profile= library->find(id);
		return profile != nullptr ? profile->name : locFormat("faceTab.missingItemFmt", id.c_str());
	};
	const std::string activeLabel=
		app->isActiveFaceProfileMissing() ? profileName(rig.activeFaceProfile)
		: bAuto                           ? std::string(locText("faceTab.autoItem"))
										  : active.name;
	if (ImGui::BeginCombo(locLabel("faceTab.activeProfileCombo"), activeLabel.c_str()))
	{
		if (ImGui::Selectable(locText("faceTab.autoItem"), rig.activeFaceProfile.empty()))
			app->setActiveFaceProfile(std::string());
		for (const std::string& id : rig.faceProfiles)
		{
			ImGui::PushID(id.c_str());
			if (ImGui::Selectable(profileName(id).c_str(), rig.activeFaceProfile == id))
				app->setActiveFaceProfile(id);
			ImGui::PopID();
		}
		ImGui::EndCombo();
	}

	if (ImGui::Button(locLabel("faceTab.newButton")))
		app->createFaceProfile(locText("faceTab.newProfileName"));
	ImGui::SameLine();
	if (ImGui::Button(locLabel(bAuto ? "faceTab.duplicateAutoButton" : "faceTab.duplicateButton")))
	{
		const std::string sourceName= bAuto ? std::string(locText("faceTab.autoItem")) : active.name;
		app->duplicateFaceProfile(active, locFormat("faceTab.copyNameFmt", sourceName.c_str()));
	}
	ImGui::BeginDisabled(bAuto);
	ImGui::SameLine();
	if (ImGui::Button(locLabel("faceTab.deleteButton")))
	{
		m_pendingDeleteId= active.id;
		ImGui::OpenPopup(locWindowTitle("windows.modalDeleteFaceProfile"));
	}
	ImGui::EndDisabled();

	// Assignment from the library
	std::vector<const FaceProfile*> unassigned;
	for (const FaceProfile& profile : library->getProfiles())
	{
		if (std::find(rig.faceProfiles.begin(), rig.faceProfiles.end(), profile.id) == rig.faceProfiles.end())
			unassigned.push_back(&profile);
	}
	if (ImGui::BeginCombo(locLabel("faceTab.assignCombo"), "", ImGuiComboFlags_NoPreview))
	{
		if (unassigned.empty())
			ImGui::TextDisabled("%s", locText("faceTab.assignNone"));
		for (const FaceProfile* profile : unassigned)
		{
			ImGui::PushID(profile->id.c_str());
			if (ImGui::Selectable(profile->name.c_str()))
				app->assignFaceProfile(profile->id);
			ImGui::PopID();
		}
		ImGui::EndCombo();
	}

	// The assigned profiles, each with its own unassign, the ones the
	// library lacks included
	std::string unassignId;
	if (!rig.faceProfiles.empty())
	{
		ImGui::TextDisabled("%s", locText("faceTab.assignedLabel"));
		for (const std::string& id : rig.faceProfiles)
		{
			ImGui::PushID(id.c_str());
			if (ImGui::SmallButton(locLabel("faceTab.unassignButton")))
				unassignId= id;
			ImGui::SameLine();
			const FaceProfile* profile= library->find(id);
			if (profile == nullptr)
				ImGui::TextColored(k_warningColor, "%s", locFormat("faceTab.missingTextFmt", id.c_str()).c_str());
			else
				ImGui::TextUnformatted(profile->name.c_str());
			ImGui::PopID();
		}
	}
	// After the loop: unassigning edits the list it walks
	if (!unassignId.empty())
		app->unassignFaceProfile(unassignId);

	// Rename in place: the name inside the file changes, the id stays
	if (!bAuto)
	{
		if (m_nameEditId != active.id)
		{
			m_nameEditId= active.id;
			m_nameEdit= active.name;
		}
		ImGui::InputText(locLabel("faceTab.nameInput"), &m_nameEdit);
		if (ImGui::IsItemDeactivatedAfterEdit() && !m_nameEdit.empty() && m_nameEdit != active.name)
			app->renameFaceProfile(active.id, m_nameEdit);
	}
	else
	{
		m_nameEditId.clear();
		ImGui::TextWrapped("%s", locText("faceTab.autoReadOnlyText"));
	}

	if (app->isActiveFaceProfileMissing())
		ImGui::TextColored(k_warningColor, "%s", locText("faceTab.missingActiveText"));
	if (!app->getFaceProfileErrors().empty())
	{
		ImGui::TextColored(k_errorColor, "%s",
						   locFormat("faceTab.errorsTextFmt", (int)app->getFaceProfileErrors().size()).c_str());
	}

	if (ImGui::BeginPopupModal(locWindowTitle("windows.modalDeleteFaceProfile"), nullptr,
							   ImGuiWindowFlags_AlwaysAutoResize))
	{
		const FaceProfile* pending= library->find(m_pendingDeleteId);
		const std::string pendingName= pending != nullptr ? pending->name : m_pendingDeleteId;
		ImGui::TextWrapped("%s", locFormat("faceTab.deletePromptFmt", pendingName.c_str()).c_str());
		if (ImGui::Button(locLabel("faceTab.deleteConfirmButton")))
		{
			app->deleteFaceProfile(m_pendingDeleteId);
			m_pendingDeleteId.clear();
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button(locLabel("common.cancel")))
		{
			m_pendingDeleteId.clear();
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

bool AvatarFaceTab::drawLets(FaceProfile& profile, const Frame& frame)
{
	bool bChanged= false;
	ImGui::SeparatorText(locText("faceTab.variablesHeader"));
	int removed= -1;
	if (!profile.lets.empty() &&
		ImGui::BeginTable("faceLets", 4, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn(locText("faceTab.nameColumn"), ImGuiTableColumnFlags_WidthStretch, 1.f);
		ImGui::TableSetupColumn(locText("faceTab.functionColumn"), ImGuiTableColumnFlags_WidthStretch, 3.f);
		ImGui::TableSetupColumn(locText("faceTab.valueColumn"), ImGuiTableColumnFlags_WidthFixed, 70.f);
		ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 24.f);
		ImGui::TableHeadersRow();
		for (size_t letIndex= 0; letIndex < profile.lets.size(); ++letIndex)
		{
			FaceProfileLet& let= profile.lets[letIndex];
			// A name error is marked on the name, any other on the expression
			const FaceExprCompileError* error= findError(*frame.errors, true, (int)letIndex);
			const bool bNameError= error != nullptr && error->bOnName;
			ImGui::PushID((int)letIndex);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			bChanged|= textInputWithError("##name", let.name, bNameError ? error : nullptr);
			ImGui::TableNextColumn();
			bChanged|= textInputWithError("##expr", let.expr, bNameError ? nullptr : error);
			ImGui::TableNextColumn();
			if (frame.bLive && letIndex < frame.face->outputs.letValues.size())
				ImGui::Text("%.3f", frame.face->outputs.letValues[letIndex]);
			ImGui::TableNextColumn();
			if (ImGui::SmallButton("x"))
				removed= (int)letIndex;
			ImGui::SetItemTooltip("%s", locText("faceTab.removeTooltip"));
			ImGui::PopID();
		}
		ImGui::EndTable();
	}
	if (removed >= 0)
	{
		profile.lets.erase(profile.lets.begin() + removed);
		bChanged= true;
	}
	if (ImGui::Button(locLabel("faceTab.addVariableButton")))
	{
		profile.lets.push_back(FaceProfileLet{"variable" + std::to_string(profile.lets.size() + 1), "0"});
		bChanged= true;
	}
	return bChanged;
}

bool AvatarFaceTab::drawOutputs(App* app, FaceProfile& profile, const Frame& frame)
{
	bool bChanged= false;
	ImGui::SeparatorText(locText("faceTab.bindingsHeader"));
	int removed= -1;
	if (!profile.outputs.empty() &&
		ImGui::BeginTable("faceOutputs", 4, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn(locText("faceTab.targetColumn"), ImGuiTableColumnFlags_WidthStretch, 1.4f);
		ImGui::TableSetupColumn(locText("faceTab.functionColumn"), ImGuiTableColumnFlags_WidthStretch, 3.f);
		ImGui::TableSetupColumn(locText("faceTab.valueColumn"), ImGuiTableColumnFlags_WidthFixed, 70.f);
		ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 24.f);
		ImGui::TableHeadersRow();
		for (size_t outputIndex= 0; outputIndex < profile.outputs.size(); ++outputIndex)
		{
			FaceProfileOutput& output= profile.outputs[outputIndex];
			ImGui::PushID((int)outputIndex);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			bChanged|= drawTargetPicker(app, output);
			if (frame.bStructural &&
				frame.binding->getOutputProblemCodes()[outputIndex] != eFaceTargetProblem::None)
			{
				ImGui::SameLine();
				ImGui::TextColored(k_warningColor, "(!)");
				ImGui::SetItemTooltip(
					"%s", locFormat("faceTab.unresolvedTooltipFmt", problemText(*frame.binding, outputIndex).c_str()).c_str());
			}

			ImGui::TableNextColumn();
			bChanged|= textInputWithError("##function", output.vrmExpressionFunction,
										  findError(*frame.errors, false, (int)outputIndex));

			ImGui::TableNextColumn();
			const float value=
				frame.bLive && outputIndex < frame.face->outputs.outputValues.size() ? frame.face->outputs.outputValues[outputIndex]
																					 : 0.f;
			// The bar draws undimmed, editable or not
			ImGui::EndDisabled();
			ImGui::ProgressBar(value, ImVec2(-FLT_MIN, 0.f), frame.bLive ? nullptr : "");
			ImGui::BeginDisabled(frame.bReadOnly);

			ImGui::TableNextColumn();
			if (ImGui::SmallButton("x"))
				removed= (int)outputIndex;
			ImGui::SetItemTooltip("%s", locText("faceTab.removeTooltip"));
			ImGui::PopID();
		}
		ImGui::EndTable();
	}
	if (removed >= 0)
	{
		profile.outputs.erase(profile.outputs.begin() + removed);
		bChanged= true;
	}
	if (ImGui::Button(locLabel("faceTab.addBindingButton")))
	{
		FaceProfileOutput output;
		output.vrmExpressionFunction= "0";
		profile.outputs.push_back(output);
		bChanged= true;
	}
	return bChanged;
}

bool AvatarFaceTab::drawTargetPicker(App* app, FaceProfileOutput& output)
{
	bool bChanged= false;
	const std::shared_ptr<const AvatarModel> model= app->getAvatarModel();
	ImGui::SetNextItemWidth(-FLT_MIN);
	if (ImGui::BeginCombo("##target", targetLabel(output).c_str(), ImGuiComboFlags_HeightLarge))
	{
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::InputTextWithHint("##filter", locText("faceTab.targetFilterHint"), &m_targetFilter);
		if (model != nullptr)
		{
			ImGui::SeparatorText(locText("faceTab.expressionsHeader"));
			for (const AvatarExpression& expression : model->expressions)
			{
				if (!containsIgnoringCase(expression.name, m_targetFilter))
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
				if (!containsIgnoringCase(morphName, m_targetFilter))
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
