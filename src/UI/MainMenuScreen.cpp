#include "MainMenuScreen.h"

#include "imgui.h"
#include "tinyfiledialogs.h"

#include "LocText.h"
#include "PathUtils.h"
#include "ProjectManager.h"
#include "SettingsPanels.h"

MainMenuScreen::Action MainMenuScreen::draw(bool bHasLastProject, const std::string& lastProjectName)
{
	Action action;

	const ImGuiViewport* viewport= ImGui::GetMainViewport();
	const ImVec2 center(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
						viewport->WorkPos.y + viewport->WorkSize.y * 0.5f);

	ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
	ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_Always);
	const ImGuiWindowFlags flags= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
								  ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
								  ImGuiWindowFlags_AlwaysAutoResize;
	ImGui::Begin("MainMenu", nullptr, flags);

	{
		const char* title= "MikanTrack";
		const float textWidth= ImGui::CalcTextSize(title).x;
		ImGui::SetCursorPosX((ImGui::GetWindowSize().x - textWidth) * 0.5f);
		ImGui::TextUnformatted(title);
	}
	ImGui::Separator();
	ImGui::Spacing();

	const ImVec2 buttonSize(-1.f, 40.f);

	if (bHasLastProject)
	{
		const std::string resumeLabel= locFormat("mainMenu.resumeFmt", lastProjectName.c_str());
		if (ImGui::Button(resumeLabel.c_str(), buttonSize))
			action.type= Action::Type::Resume;
	}

	if (ImGui::Button(locLabel("mainMenu.newProjectButton"), buttonSize))
		m_newProjectModal.request();

	if (ImGui::Button(locLabel("mainMenu.loadProjectButton"), buttonSize))
	{
		// Blocks the frame loop while open; fine here, nothing is streaming in
		// the menu state
		const std::string defaultDir= (ProjectManager::getProjectsRootDirectory() / "").string();
		const char* filterPatterns[]= {"project.json"};
		const char* selectedPath= tinyfd_openFileDialog(
			locText("mainMenu.loadProjectDialogTitle"), defaultDir.c_str(), 1, filterPatterns,
			locText("mainMenu.loadProjectDialogFilterDescription"), 0);
		if (selectedPath != nullptr)
		{
			action.type= Action::Type::LoadProject;
			// tinyfd returns UTF-8 on Windows by default
			action.projectFile= PathUtils::utf8ToPath(selectedPath);
		}
	}

	if (ImGui::Button(locLabel("mainMenu.exitButton"), buttonSize))
		action.type= Action::Type::Exit;

	ImGui::Separator();
	// The combo's label draws to its right, so the combo leaves it room
	ImGui::SetNextItemWidth(-(ImGui::CalcTextSize(locText("settingsPanel.language")).x +
							  ImGui::GetStyle().ItemInnerSpacing.x));
	SettingsPanels::drawLanguageCombo();

	if (!m_statusMessage.empty())
	{
		ImGui::Spacing();
		ImGui::PushTextWrapPos(0.f);
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), "%s", m_statusMessage.c_str());
		ImGui::PopTextWrapPos();
	}

	if (m_newProjectModal.draw(action.projectName))
		action.type= Action::Type::NewProject;

	ImGui::End();
	return action;
}
