#include "PipelineText.h"

#include "LocText.h"

namespace PipelineText
{
std::string format(const StageText& text)
{
	if (text.isEmpty())
		return std::string();
	const std::string& key= text.key;
	const bool bIsFormat= key.size() > 3 && key.compare(key.size() - 3, 3, "Fmt") == 0;
	if (!bIsFormat)
		return locText(key.c_str());
	return locFormat(key.c_str(), text.args[0], text.args[1], text.args[2]);
}

std::string stageName(const PipelineStageId& stage)
{
	if (!stage.isValid())
		return std::string();
	if (stage.stage == ePipelineStage::Camera)
		return locFormat("pipelinePanel.cameraLabelFmt", stage.cameraIndex + 1);
	return locText(PipelineModel::getNameKey(stage.stage));
}

ImU32 stateColor(eStageState state, float alpha)
{
	ImVec4 color(0.6f, 0.65f, 0.7f, 1.f);
	switch (state)
	{
	case eStageState::Running:
		color= ImVec4(0.5f, 0.83f, 0.54f, 1.f);
		break;
	case eStageState::Degraded:
		color= ImVec4(1.f, 0.8f, 0.4f, 1.f);
		break;
	case eStageState::Inactive:
		color= ImVec4(0.49f, 0.53f, 0.6f, 1.f);
		break;
	case eStageState::NotConfigured:
		color= ImVec4(1.f, 0.4f, 0.4f, 1.f);
		break;
	}
	color.w= alpha;
	return ImGui::ColorConvertFloat4ToU32(color);
}

void drawStateDot(ImDrawList* drawList, const ImVec2& center, float radius, eStageState state, float alpha)
{
	const ImU32 color= stateColor(state, alpha);
	if (state == eStageState::Running || state == eStageState::Degraded)
		drawList->AddCircleFilled(center, radius, color);
	else
		drawList->AddCircle(center, radius - 0.5f, color, 0, 2.f);
}
} // namespace PipelineText
