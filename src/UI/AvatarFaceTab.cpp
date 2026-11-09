#include "AvatarFaceTab.h"

#include <algorithm>
#include <vector>

#include "imgui.h"
#include "misc/cpp/imgui_stdlib.h"

#include "App.h"
#include "FaceExprCompiler.h"
#include "FaceProfileBinding.h"
#include "FaceProfileLibrary.h"
#include "FaceTabWidgets.h"
#include "LocText.h"

namespace
{
const FaceExprCompileError* findError(const std::vector<FaceExprCompileError>& errors, bool bLet, int index)
{
	for (const FaceExprCompileError& error : errors)
	{
		if (error.bLet == bLet && error.index == index)
			return &error;
	}
	return nullptr;
}

using FaceTabWidgets::textInputWithError;
} // namespace

// The per-draw facts every section reads
struct AvatarFaceTab::Frame
{
	const FaceSources* face= nullptr;
	std::shared_ptr<const FaceProfileBinding> binding;
	const std::vector<FaceExprCompileError>* errors= nullptr;
	const FaceExprEditTarget* activeEdit= nullptr;
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

std::optional<FaceExprEditTarget> AvatarFaceTab::draw(App* app, const FaceSources& face,
													  const FaceExprEditTarget* activeEdit)
{
	m_editRequest.reset();
	drawProfileBar(app);

	m_profile= app->getActiveFaceProfile();
	FaceProfile& profile= m_profile;

	Frame frame;
	frame.face= &face;
	frame.binding= app->getFaceBinding();
	frame.errors= &app->getFaceProfileErrors();
	frame.activeEdit= activeEdit;
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
	return m_editRequest;
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
				ImGui::TextColored(FaceTabWidgets::k_warningColor, "%s", locFormat("faceTab.missingTextFmt", id.c_str()).c_str());
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
		ImGui::TextColored(FaceTabWidgets::k_warningColor, "%s", locText("faceTab.missingActiveText"));
	if (!app->getFaceProfileErrors().empty())
	{
		ImGui::TextColored(FaceTabWidgets::k_errorColor, "%s",
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

void AvatarFaceTab::drawEditCell(const Frame& frame, bool bLet, int index)
{
	const FaceExprEditTarget target{bLet, index};
	if (frame.activeEdit != nullptr && *frame.activeEdit == target)
		ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, ImGui::GetColorU32(ImGuiCol_Header));
	if (ImGui::SmallButton(locLabel("exprEditor.editButton")))
		m_editRequest= target;
	ImGui::SetItemTooltip("%s", locText("exprEditor.editTooltip"));
}

bool AvatarFaceTab::drawLets(FaceProfile& profile, const Frame& frame)
{
	bool bChanged= false;
	ImGui::SeparatorText(locText("faceTab.variablesHeader"));
	int removed= -1;
	if (!profile.lets.empty() &&
		ImGui::BeginTable("faceLets", 5, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn(locText("faceTab.nameColumn"), ImGuiTableColumnFlags_WidthStretch, 1.f);
		ImGui::TableSetupColumn(locText("faceTab.functionColumn"), ImGuiTableColumnFlags_WidthStretch, 3.f);
		ImGui::TableSetupColumn(locText("faceTab.valueColumn"), ImGuiTableColumnFlags_WidthFixed, 70.f);
		ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 44.f);
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
			drawEditCell(frame, true, (int)letIndex);
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
		ImGui::BeginTable("faceOutputs", 5, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn(locText("faceTab.targetColumn"), ImGuiTableColumnFlags_WidthStretch, 1.4f);
		ImGui::TableSetupColumn(locText("faceTab.functionColumn"), ImGuiTableColumnFlags_WidthStretch, 3.f);
		ImGui::TableSetupColumn(locText("faceTab.valueColumn"), ImGuiTableColumnFlags_WidthFixed, 70.f);
		ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 44.f);
		ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 24.f);
		ImGui::TableHeadersRow();
		for (size_t outputIndex= 0; outputIndex < profile.outputs.size(); ++outputIndex)
		{
			FaceProfileOutput& output= profile.outputs[outputIndex];
			ImGui::PushID((int)outputIndex);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			bChanged|= FaceTabWidgets::drawTargetPicker(app, output, m_targetFilter);
			if (frame.bStructural &&
				frame.binding->getOutputProblemCodes()[outputIndex] != eFaceTargetProblem::None)
			{
				ImGui::SameLine();
				ImGui::TextColored(FaceTabWidgets::k_warningColor, "(!)");
				ImGui::SetItemTooltip(
					"%s", locFormat("faceTab.unresolvedTooltipFmt",
									FaceTabWidgets::problemText(*frame.binding, outputIndex).c_str()).c_str());
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
			drawEditCell(frame, false, (int)outputIndex);
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
