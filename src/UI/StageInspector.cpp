#include "StageInspector.h"

#include <string>
#include <vector>

#include "imgui.h"

#include "LocText.h"
#include "PipelineText.h"

namespace
{
constexpr float kDotRadius= 4.5f;

// The sections under Status, in the template's order. Diagnostics starts
// closed.
struct SectionInfo
{
	eStageSection section;
	const char* titleKey;
	ImGuiTreeNodeFlags headerFlags;
};
const SectionInfo k_sections[]= {
	{eStageSection::Inputs, "stageInspector.inputsSection", ImGuiTreeNodeFlags_DefaultOpen},
	{eStageSection::Settings, "stageInspector.settingsSection", ImGuiTreeNodeFlags_DefaultOpen},
	{eStageSection::Diagnostics, "stageInspector.diagnosticsSection", ImGuiTreeNodeFlags_None},
};

void drawRow(const char* labelKey, const char* value)
{
	ImGui::TableNextRow();
	ImGui::TableNextColumn();
	ImGui::TextDisabled("%s", locText(labelKey));
	ImGui::TableNextColumn();
	ImGui::TextWrapped("%s", value);
}

// A stage as a row that selects it: its state dot, its name (a camera's
// sub-stage with its camera, so two cameras' rows differ) and value, and its
// reason on hover. Returns true when clicked.
bool drawStageRow(const PipelineStatusInputs& inputs, const PipelineStageId& stage)
{
	const StageStatus status= PipelineModel::computeStatus(inputs, stage);
	std::string label= PipelineText::stageDisplayName(stage);
	label+= "   " + PipelineText::format(status.value);

	ImGui::PushID(PipelineModel::toString(stage).c_str());
	const ImVec2 rowStart= ImGui::GetCursorScreenPos();
	const float indent= kDotRadius * 2.f + 8.f;
	ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent);
	const bool bClicked= ImGui::Selectable(label.c_str());
	PipelineText::drawStateDot(ImGui::GetWindowDrawList(),
							   ImVec2(rowStart.x + kDotRadius + 2.f, rowStart.y + ImGui::GetTextLineHeight() * 0.5f),
							   kDotRadius, status.state);
	if (ImGui::IsItemHovered())
	{
		const std::string reason= PipelineText::format(status.reason);
		if (!reason.empty())
		{
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.f);
			ImGui::TextUnformatted(reason.c_str());
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
	}
	ImGui::PopID();
	return bClicked;
}
} // namespace

PipelineStageId StageInspector::draw(bool* pOpen, const PipelineStatusInputs& inputs, const PipelineStageId& selected,
									 IStageContent* content)
{
	PipelineStageId request;
	if (!ImGui::Begin(locWindowTitle("windows.stageInspector"), pOpen))
	{
		ImGui::End();
		return request;
	}
	if (selected.isValid())
		request= drawStageBody(inputs, selected, content);
	else
		request= drawProject(inputs);
	ImGui::End();
	return request;
}

PipelineStageId StageInspector::drawStageBody(const PipelineStatusInputs& inputs, const PipelineStageId& stage,
											  IStageContent* content)
{
	PipelineStageId request;
	const StageStatus status= PipelineModel::computeStatus(inputs, stage);
	const PipelineStageId topLevel= PipelineModel::getTopLevel(stage);

	// Where the stage sits, when it is a sub-stage
	if (topLevel != stage)
	{
		std::string crumb= PipelineText::stageName(topLevel);
		if (stage.cameraIndex >= 0 && stage.stage != ePipelineStage::Camera)
		{
			PipelineStageId camera;
			camera.stage= ePipelineStage::Camera;
			camera.cameraIndex= stage.cameraIndex;
			crumb+= " / " + PipelineText::stageName(camera);
		}
		ImGui::TextDisabled("%s", crumb.c_str());
	}

	ImGui::TextUnformatted(PipelineText::stageName(stage).c_str());
	ImGui::SameLine();
	ImGui::PushStyleColor(ImGuiCol_Text, PipelineText::stateColor(status.state));
	ImGui::TextUnformatted(locText(PipelineModel::getStateKey(status.state)));
	ImGui::PopStyleColor();
	ImGui::PushTextWrapPos(0.f);
	ImGui::TextDisabled("%s", locText(PipelineModel::getDescriptionKey(stage.stage)));
	ImGui::PopTextWrapPos();

	ImGui::SeparatorText(locText("stageInspector.statusSection"));
	if (ImGui::BeginTable("##status", 2, ImGuiTableFlags_SizingFixedFit))
	{
		ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthFixed);
		ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthStretch);
		drawRow("stageInspector.stateLabel", locText(PipelineModel::getStateKey(status.state)));
		drawRow("stageInspector.nowLabel", PipelineText::format(status.value).c_str());
		drawRow("stageInspector.shownFrameLabel",
				locText(inputs.bReplay ? "stageInspector.shownReplay" : "stageInspector.shownLive"));
		ImGui::EndTable();
	}
	const std::string reason= PipelineText::format(status.reason);
	if (!reason.empty())
	{
		ImGui::Spacing();
		// A running stage's reason is a note, not a fault
		if (status.state == eStageState::Running)
			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		else
			ImGui::PushStyleColor(ImGuiCol_Text, PipelineText::stateColor(status.state));
		ImGui::TextWrapped("%s", reason.c_str());
		ImGui::PopStyleColor();
	}

	// The stage's own sections, each under a header, a section the stage
	// does not have left out
	bool bAnySection= false;
	if (content != nullptr)
	{
		for (const SectionInfo& section : k_sections)
		{
			if (!content->hasSection(stage, section.section))
				continue;
			if (!bAnySection)
				ImGui::Spacing();
			bAnySection= true;
			if (ImGui::CollapsingHeader(locLabel(section.titleKey), section.headerFlags))
				content->drawSection(stage, section.section);
		}
	}

	if (bAnySection)
		return request;

	// A top-level stage with no section of its own but a sub-stage with one
	// holds its controls under its sub-stages, and lists them one row each
	std::vector<PipelineStageId> subStages;
	if (topLevel == stage)
	{
		for (const std::vector<PipelineStageId>& lane : PipelineModel::getSubStageLanes(stage, inputs.cameraCount))
			subStages.insert(subStages.end(), lane.begin(), lane.end());
	}
	bool bSubStageSection= false;
	if (content != nullptr)
	{
		for (const PipelineStageId& subStage : subStages)
		{
			for (const SectionInfo& section : k_sections)
				bSubStageSection|= content->hasSection(subStage, section.section);
		}
	}

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::PushTextWrapPos(0.f);
	ImGui::TextDisabled(
		"%s", locText(bSubStageSection ? "stageInspector.settingsInSubStages" : "stageInspector.settingsElsewhere"));
	ImGui::PopTextWrapPos();
	if (bSubStageSection)
	{
		for (const PipelineStageId& subStage : subStages)
		{
			if (drawStageRow(inputs, subStage))
				request= subStage;
		}
	}
	return request;
}

PipelineStageId StageInspector::drawProject(const PipelineStatusInputs& inputs)
{
	PipelineStageId request;
	ImGui::PushTextWrapPos(0.f);
	ImGui::TextDisabled("%s", locText("stageInspector.nothingSelected"));
	ImGui::PopTextWrapPos();
	ImGui::Spacing();

	if (ImGui::BeginTable("##project", 2, ImGuiTableFlags_SizingFixedFit))
	{
		ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthFixed);
		ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthStretch);
		drawRow("stageInspector.summaryPipeline",
				locText(inputs.bDirect ? "stageInspector.pipelineDirect" : "stageInspector.pipelineMulti"));
		drawRow("stageInspector.summaryCameras", std::to_string(inputs.cameraCount).c_str());
		drawRow("stageInspector.summaryImu",
				locText(inputs.bImuEnabled && !inputs.bDirect ? "stageInspector.summaryEnabled"
															  : "stageInspector.summaryNone"));
		drawRow("stageInspector.summaryAvatar",
				locText(inputs.bAvatarLoaded ? "stageInspector.summaryLoaded" : "stageInspector.summaryNone"));
		ImGui::EndTable();
	}

	// Every stage that is degraded or not configured, in pipeline order
	ImGui::SeparatorText(locText("stageInspector.needsAttention"));
	int listed= 0;
	for (const PipelineStageId& stage : PipelineModel::getAllStages(inputs.bDirect, inputs.cameraCount))
	{
		const StageStatus status= PipelineModel::computeStatus(inputs, stage);
		if (status.state != eStageState::Degraded && status.state != eStageState::NotConfigured)
			continue;
		++listed;
		if (drawStageRow(inputs, stage))
			request= stage;
	}
	if (listed == 0)
		ImGui::TextDisabled("%s", locText("stageInspector.allRunning"));
	return request;
}
