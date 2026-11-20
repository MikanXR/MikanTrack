#include "NewProjectModal.h"

#include <filesystem>

#include "imgui.h"

#include "LocText.h"
#include "ProjectManager.h"

void NewProjectModal::request()
{
	m_nameBuffer[0]= '\0';
	m_bRequested= true;
}

bool NewProjectModal::draw(std::string& outProjectName)
{
	bool bConfirmed= false;

	// Opened here rather than where it was asked for: a popup belongs to the
	// window that opens it, and a menu item's window closes with its menu
	if (m_bRequested)
	{
		ImGui::OpenPopup(locWindowTitle("windows.modalNewProject"));
		m_bRequested= false;
	}

	const ImGuiViewport* viewport= ImGui::GetMainViewport();
	const ImVec2 center(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
						viewport->WorkPos.y + viewport->WorkSize.y * 0.5f);
	ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
	if (ImGui::BeginPopupModal(locWindowTitle("windows.modalNewProject"), nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::TextUnformatted(locText("mainMenu.projectNameLabel"));
		if (ImGui::IsWindowAppearing())
			ImGui::SetKeyboardFocusHere();
		ImGui::SetNextItemWidth(280.f);
		const bool bEnterPressed= ImGui::InputText("##projectName", m_nameBuffer, sizeof(m_nameBuffer),
												   ImGuiInputTextFlags_EnterReturnsTrue);

		const std::string name(m_nameBuffer);
		std::string nameError;
		bool bValid= ProjectManager::isValidProjectName(name, nameError);
		if (bValid && std::filesystem::exists(ProjectManager::getProjectsRootDirectory() / name))
		{
			bValid= false;
			nameError= locText("errors.projectNameExists");
		}
		if (!name.empty() && !bValid)
			ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), "%s", nameError.c_str());

		ImGui::BeginDisabled(!bValid);
		const bool bCreatePressed= ImGui::Button(locLabel("common.create"), ImVec2(120, 0));
		ImGui::EndDisabled();
		ImGui::SameLine();
		const bool bCancelPressed= ImGui::Button(locLabel("common.cancel"), ImVec2(120, 0));

		if ((bCreatePressed || bEnterPressed) && bValid)
		{
			outProjectName= name;
			bConfirmed= true;
			ImGui::CloseCurrentPopup();
		}
		else if (bCancelPressed)
		{
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}

	return bConfirmed;
}
