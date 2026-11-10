#include "StageInspector.h"

#include <string>
#include <vector>

#include "imgui.h"

#include "LocText.h"
#include "PipelineText.h"
#include "VisionThread.h"

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
		drawStage(inputs, selected, content);
	else
		request= drawProject(inputs);
	ImGui::End();
	return request;
}

void StageInspector::drawStage(const PipelineStatusInputs& inputs, const PipelineStageId& stage,
							   IStageContent* content)
{
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

	if (!bAnySection)
	{
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::PushTextWrapPos(0.f);
		ImGui::TextDisabled("%s", locText("stageInspector.settingsElsewhere"));
		ImGui::PopTextWrapPos();
	}
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
		// A camera's sub-stage reads with its camera, so two cameras' rows differ
		std::string label= PipelineText::stageName(stage);
		if (stage.cameraIndex >= 0 && stage.stage != ePipelineStage::Camera)
		{
			PipelineStageId camera;
			camera.stage= ePipelineStage::Camera;
			camera.cameraIndex= stage.cameraIndex;
			label= PipelineText::stageName(camera) + " / " + label;
		}
		label+= "   " + PipelineText::format(status.value);

		ImGui::PushID(PipelineModel::toString(stage).c_str());
		const ImVec2 rowStart= ImGui::GetCursorScreenPos();
		const float indent= kDotRadius * 2.f + 8.f;
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent);
		if (ImGui::Selectable(label.c_str()))
			request= stage;
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
	}
	if (listed == 0)
		ImGui::TextDisabled("%s", locText("stageInspector.allRunning"));
	return request;
}

void PerformancePanel::draw(bool* pOpen, const VisionThread& visionThread)
{
	if (!ImGui::Begin(locWindowTitle("windows.performance"), pOpen))
	{
		ImGui::End();
		return;
	}
	const VisionThread::Telemetry telemetry= visionThread.getTelemetry();
	if (!telemetry.bValid)
	{
		ImGui::TextDisabled("%s", locText("performancePanel.noTelemetry"));
		ImGui::End();
		return;
	}

	ImGui::TextUnformatted(locFormat("performancePanel.totalFmt", (double)telemetry.totalMs).c_str());
	ImGui::SameLine(0.f, 24.f);
	const int hitchCount= visionThread.getHitchCount();
	if (hitchCount == 0)
	{
		ImGui::TextDisabled("%s", locText("performancePanel.hitchesNone"));
	}
	else
	{
		ImGui::TextUnformatted(
			locFormat("performancePanel.hitchesFmt", hitchCount, (double)visionThread.getLastHitchMs()).c_str());
	}

	if (ImGui::BeginTable("##phases", 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn(locText("performancePanel.phaseColumn"));
		ImGui::TableSetupColumn(locText("performancePanel.msColumn"));
		ImGui::TableHeadersRow();
		for (int phaseIndex= 1; phaseIndex < (int)VisionThread::eVisionPhase::Count; ++phaseIndex)
		{
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(VisionThread::getPhaseName((VisionThread::eVisionPhase)phaseIndex));
			ImGui::TableNextColumn();
			ImGui::Text("%.2f", telemetry.phaseMs[phaseIndex]);
		}
		ImGui::EndTable();
	}
	ImGui::End();
}
