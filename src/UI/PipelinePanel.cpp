#include "PipelinePanel.h"

#include <algorithm>
#include <string>
#include <vector>

#include "imgui.h"

#include "LocText.h"
#include "PipelineText.h"

namespace
{
using S= ePipelineStage;

constexpr float kTopHeight= 46.f;
constexpr float kLineHeight= 30.f;
constexpr float kLaneHeight= 26.f;
constexpr float kLaneGap= 5.f;
constexpr float kRowGap= 6.f;
constexpr float kArrowLength= 26.f;
constexpr float kLaneArrowLength= 16.f;
constexpr float kLaneSetGap= 10.f;
constexpr float kPadX= 9.f;
constexpr float kMinTopWidth= 118.f;
constexpr float kDotRadius= 4.5f;
constexpr float kDotSlot= 16.f;
constexpr float kTextGap= 8.f;
constexpr float kRounding= 4.f;
// The value's slot is a fixed width per box style, in font heights, so a
// number that changes never resizes its box. A longer value is cut short
// with an ellipsis and reads in full in the tooltip and the inspector.
constexpr float kTopValueEms= 8.5f;
constexpr float kLineValueEms= 6.f;
constexpr float kPillValueEms= 3.5f;
const ImU32 kAccent= IM_COL32(128, 166, 230, 255);
const ImU32 kLine= IM_COL32(93, 107, 136, 255);

enum class eBoxStyle
{
	// The name over the dot and the value
	Top,
	// One line: dot, name, value
	Line,
	// One line, rounded ends: the data pill
	Pill,
};

struct BoxText
{
	std::string name;
	std::string value;
	StageStatus status;
};

BoxText makeBoxText(const PipelineStatusInputs& inputs, const PipelineStageId& stage)
{
	BoxText text;
	text.name= PipelineText::stageName(stage);
	text.status= PipelineModel::computeStatus(inputs, stage);
	text.value= PipelineText::format(text.status.value);
	return text;
}

float getValueSlotWidth(eBoxStyle style)
{
	const float em= ImGui::GetFontSize();
	switch (style)
	{
	case eBoxStyle::Top:
		return kTopValueEms * em;
	case eBoxStyle::Line:
		return kLineValueEms * em;
	case eBoxStyle::Pill:
		return kPillValueEms * em;
	}
	return kLineValueEms * em;
}

// The box's width from its name and its style alone, never from its value
float measureBox(const BoxText& text, eBoxStyle style)
{
	const float nameWidth= ImGui::CalcTextSize(text.name.c_str()).x;
	const float valueSlot= getValueSlotWidth(style);
	if (style == eBoxStyle::Top)
		return std::max(kMinTopWidth, std::max(nameWidth, kDotSlot + valueSlot) + 2.f * kPadX);
	return kDotSlot + nameWidth + kTextGap + valueSlot + 2.f * kPadX;
}

// The text cut to a width, ending in an ellipsis when it does not fit
std::string fitText(const std::string& text, float maxWidth)
{
	if (ImGui::CalcTextSize(text.c_str()).x <= maxWidth)
		return text;
	const char* ellipsis= "...";
	std::string cut= text;
	while (!cut.empty())
	{
		// Step back one whole UTF-8 character
		size_t last= cut.size() - 1;
		while (last > 0 && ((unsigned char)cut[last] & 0xC0) == 0x80)
			--last;
		cut.erase(last);
		if (ImGui::CalcTextSize((cut + ellipsis).c_str()).x <= maxWidth)
			break;
	}
	return cut + ellipsis;
}

void drawArrow(ImDrawList* drawList, const ImVec2& from, const ImVec2& to)
{
	drawList->AddLine(from, ImVec2(to.x - 5.f, to.y), kLine, 2.f);
	drawList->AddTriangleFilled(ImVec2(to.x, to.y), ImVec2(to.x - 7.f, to.y - 4.5f), ImVec2(to.x - 7.f, to.y + 4.5f),
								kLine);
}

void drawStageTooltip(const PipelineStageId& stage, const BoxText& text)
{
	ImGui::BeginTooltip();
	ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.f);
	ImGui::TextUnformatted(text.name.c_str());
	ImGui::TextDisabled("%s", locText(PipelineModel::getDescriptionKey(stage.stage)));
	const std::string reason= PipelineText::format(text.status.reason);
	if (!reason.empty())
	{
		ImGui::Spacing();
		ImGui::PushStyleColor(ImGuiCol_Text, PipelineText::stateColor(text.status.state));
		ImGui::Text("%s: %s", locText(PipelineModel::getStateKey(text.status.state)), reason.c_str());
		ImGui::PopStyleColor();
	}
	ImGui::PopTextWrapPos();
	ImGui::EndTooltip();
}

// One stage box at a screen position. Returns true when it was clicked.
bool drawStageBox(const PipelineStageId& stage, const BoxText& text, eBoxStyle style, const ImVec2& position,
				  const ImVec2& size, bool bSelected, bool bParentOfSelected)
{
	ImDrawList* drawList= ImGui::GetWindowDrawList();
	const ImVec2 boxMax(position.x + size.x, position.y + size.y);

	ImGui::SetCursorScreenPos(position);
	ImGui::PushID(PipelineModel::toString(stage).c_str());
	const bool bClicked= ImGui::InvisibleButton("##stage", size);
	const bool bHovered= ImGui::IsItemHovered();
	ImGui::PopID();

	// A stage the project does not use draws faded
	const float alpha= text.status.state == eStageState::Inactive ? 0.5f : 1.f;
	const float rounding= style == eBoxStyle::Pill ? size.y * 0.5f : kRounding;
	ImU32 fill= ImGui::GetColorU32(bHovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg, alpha);
	if (bSelected)
		fill= ImGui::GetColorU32(ImGuiCol_Header);
	drawList->AddRectFilled(position, boxMax, fill, rounding);
	if (bSelected)
		drawList->AddRect(position, boxMax, kAccent, rounding, 0, 2.f);
	else if (bParentOfSelected)
		drawList->AddRect(position, boxMax, kAccent, rounding, 0, 1.f);
	else
		drawList->AddRect(position, boxMax, ImGui::GetColorU32(ImGuiCol_Border, alpha), rounding);

	const ImU32 nameColor= ImGui::GetColorU32(ImGuiCol_Text, alpha);
	const ImU32 valueColor= ImGui::GetColorU32(ImGuiCol_TextDisabled, alpha);
	const float textHeight= ImGui::GetTextLineHeight();
	if (style == eBoxStyle::Top)
	{
		const float nameY= position.y + (size.y - 2.f * textHeight - 2.f) * 0.5f;
		const float valueY= nameY + textHeight + 2.f;
		drawList->AddText(ImVec2(position.x + kPadX, nameY), nameColor, text.name.c_str());
		PipelineText::drawStateDot(drawList, ImVec2(position.x + kPadX + kDotRadius, valueY + textHeight * 0.5f),
								   kDotRadius, text.status.state, alpha);
		drawList->AddText(ImVec2(position.x + kPadX + kDotSlot, valueY), valueColor,
						  fitText(text.value, getValueSlotWidth(style)).c_str());
	}
	else
	{
		const float textY= position.y + (size.y - textHeight) * 0.5f;
		float x= position.x + kPadX;
		PipelineText::drawStateDot(drawList, ImVec2(x + kDotRadius, position.y + size.y * 0.5f), kDotRadius,
								   text.status.state, alpha);
		x+= kDotSlot;
		drawList->AddText(ImVec2(x, textY), nameColor, text.name.c_str());
		x+= ImGui::CalcTextSize(text.name.c_str()).x + kTextGap;
		if (!text.value.empty())
			drawList->AddText(ImVec2(x, textY), valueColor, fitText(text.value, boxMax.x - kPadX - x).c_str());
	}

	if (bHovered)
		drawStageTooltip(stage, text);
	return bClicked;
}

float drawLegend(ImDrawList* drawList, const ImVec2& position)
{
	float x= position.x;
	const float textHeight= ImGui::GetTextLineHeight();
	for (const eStageState state :
		 {eStageState::Running, eStageState::Degraded, eStageState::Inactive, eStageState::NotConfigured})
	{
		const char* label= locText(PipelineModel::getStateKey(state));
		PipelineText::drawStateDot(drawList, ImVec2(x + kDotRadius, position.y + textHeight * 0.5f), kDotRadius, state);
		drawList->AddText(ImVec2(x + kDotSlot, position.y), ImGui::GetColorU32(ImGuiCol_TextDisabled), label);
		x+= kDotSlot + ImGui::CalcTextSize(label).x + 14.f;
	}
	return x - position.x;
}

float measureLegend()
{
	float width= 0.f;
	for (const eStageState state :
		 {eStageState::Running, eStageState::Degraded, eStageState::Inactive, eStageState::NotConfigured})
	{
		width+= kDotSlot + ImGui::CalcTextSize(locText(PipelineModel::getStateKey(state))).x + 14.f;
	}
	return width;
}
} // namespace

void PipelinePanel::draw(bool* pOpen, const PipelineStatusInputs& inputs)
{
	// A selection the project no longer has (a camera removed, the pipeline
	// switched) is dropped before anything reads it
	if (m_selected.isValid())
	{
		const std::vector<PipelineStageId> all= PipelineModel::getAllStages(inputs.bDirect, inputs.cameraCount);
		if (std::find(all.begin(), all.end(), m_selected) == all.end())
			m_selected= PipelineStageId();
	}

	if (!ImGui::Begin(locWindowTitle("windows.pipeline"), pOpen))
	{
		ImGui::End();
		return;
	}
	ImGui::BeginChild("##pipelineStrip", ImVec2(0.f, 0.f), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);

	ImDrawList* drawList= ImGui::GetWindowDrawList();
	const ImVec2 origin= ImGui::GetCursorScreenPos();
	const PipelineStageId selectedTop= PipelineModel::getTopLevel(m_selected);
	PipelineStageId clicked;
	auto box= [&](const PipelineStageId& stage, const BoxText& text, eBoxStyle style, const ImVec2& position,
				  const ImVec2& size) {
		const bool bSelected= stage == m_selected;
		const bool bParent= !bSelected && m_selected.isValid() && stage == selectedTop;
		if (drawStageBox(stage, text, style, position, size, bSelected, bParent))
			clicked= stage;
	};

	// The top row: each stage in frame order, the source pose as a pill
	const std::vector<PipelineStageId> topRow= PipelineModel::getTopRow(inputs.bDirect);
	const float topMidY= origin.y + kTopHeight * 0.5f;
	float x= origin.x;
	float retargetX= origin.x;
	float retargetRight= origin.x;
	float outputX= origin.x;
	for (size_t index= 0; index < topRow.size(); ++index)
	{
		const PipelineStageId& stage= topRow[index];
		const bool bPill= stage.stage == S::SourcePose;
		const BoxText text= makeBoxText(inputs, stage);
		const float width= measureBox(text, bPill ? eBoxStyle::Pill : eBoxStyle::Top);
		if (index > 0)
		{
			drawArrow(drawList, ImVec2(x, topMidY), ImVec2(x + kArrowLength, topMidY));
			x+= kArrowLength;
		}
		if (stage.stage == S::Retarget)
			retargetX= x;
		if (stage.stage == S::Output)
			outputX= x;
		if (bPill)
			box(stage, text, eBoxStyle::Pill, ImVec2(x, topMidY - kLineHeight * 0.5f), ImVec2(width, kLineHeight));
		else
			box(stage, text, eBoxStyle::Top, ImVec2(x, origin.y), ImVec2(width, kTopHeight));
		x+= width;
		if (stage.stage == S::Retarget)
			retargetRight= x;
	}
	float contentRight= x;

	// The face joins at the output: its box sits under the retarget and an
	// elbow carries it up into the arrow between the two
	const float sideY= origin.y + kTopHeight + kRowGap;
	{
		const PipelineStageId side= PipelineModel::getSideStage();
		const BoxText text= makeBoxText(inputs, side);
		const float width= measureBox(text, eBoxStyle::Line);
		box(side, text, eBoxStyle::Line, ImVec2(retargetX, sideY), ImVec2(width, kLineHeight));
		const float sideMidY= sideY + kLineHeight * 0.5f;
		const float joinX= std::max(retargetX + width + 10.f, (retargetRight + outputX) * 0.5f);
		drawList->AddLine(ImVec2(retargetX + width, sideMidY), ImVec2(joinX, sideMidY), kLine, 2.f);
		if (joinX <= outputX)
		{
			drawList->AddLine(ImVec2(joinX, sideMidY), ImVec2(joinX, topMidY), kLine, 2.f);
		}
		else
		{
			// The face box is wider than the retarget: the elbow rises past
			// the output's left edge into the output box's underside instead
			drawList->AddLine(ImVec2(joinX, sideMidY), ImVec2(joinX, origin.y + kTopHeight), kLine, 2.f);
		}
		contentRight= std::max(contentRight, joinX);
	}

	// The second row: the selected stage's sub-stages, one lane per camera
	// for the capture
	const float lanesY= sideY + kLineHeight + 10.f;
	drawList->AddLine(ImVec2(origin.x, lanesY - 5.f),
					  ImVec2(std::max(contentRight, origin.x + ImGui::GetContentRegionAvail().x), lanesY - 5.f),
					  ImGui::GetColorU32(ImGuiCol_Separator));
	float contentBottom= lanesY + kLaneHeight;
	float lanesRight= origin.x;
	const std::vector<std::vector<PipelineStageId>> lanes=
		PipelineModel::getSubStageLanes(m_selected, inputs.cameraCount);
	if (lanes.empty())
	{
		const std::string hint= m_selected.isValid()
			? locFormat("pipelinePanel.noSubStagesFmt", PipelineText::stageName(selectedTop).c_str())
			: std::string(locText("pipelinePanel.selectHint"));
		drawList->AddText(ImVec2(origin.x, lanesY + (kLaneHeight - ImGui::GetTextLineHeight()) * 0.5f),
						  ImGui::GetColorU32(ImGuiCol_TextDisabled), hint.c_str());
	}
	else
	{
		const bool bFlow= PipelineModel::isLaneFlow(selectedTop.stage);
		// Columns line up across lanes: each column takes its widest box
		std::vector<std::vector<BoxText>> texts(lanes.size());
		std::vector<float> columnWidths;
		for (size_t laneIndex= 0; laneIndex < lanes.size(); ++laneIndex)
		{
			for (size_t column= 0; column < lanes[laneIndex].size(); ++column)
			{
				texts[laneIndex].push_back(makeBoxText(inputs, lanes[laneIndex][column]));
				if (column >= columnWidths.size())
					columnWidths.push_back(0.f);
				columnWidths[column]=
					std::max(columnWidths[column], measureBox(texts[laneIndex][column], eBoxStyle::Line));
			}
		}
		float laneY= lanesY;
		for (size_t laneIndex= 0; laneIndex < lanes.size(); ++laneIndex)
		{
			float laneX= origin.x;
			const float laneMidY= laneY + kLaneHeight * 0.5f;
			for (size_t column= 0; column < lanes[laneIndex].size(); ++column)
			{
				if (column > 0)
				{
					if (bFlow)
					{
						drawArrow(drawList, ImVec2(laneX, laneMidY), ImVec2(laneX + kLaneArrowLength, laneMidY));
						laneX+= kLaneArrowLength;
					}
					else
					{
						laneX+= kLaneSetGap;
					}
				}
				box(lanes[laneIndex][column], texts[laneIndex][column], eBoxStyle::Line, ImVec2(laneX, laneY),
					ImVec2(columnWidths[column], kLaneHeight));
				laneX+= columnWidths[column];
			}
			contentRight= std::max(contentRight, laneX);
			lanesRight= std::max(lanesRight, laneX);
			laneY+= kLaneHeight + kLaneGap;
		}
		contentBottom= laneY - kLaneGap;
	}

	// The legend, in the panel's bottom right corner. Dropped when the lanes
	// reach it, so it never covers a stage and never forces a scroll.
	{
		const float legendWidth= measureLegend();
		const float legendX=
			ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - ImGui::GetStyle().ScrollbarSize - legendWidth;
		const float legendY= ImGui::GetWindowPos().y + ImGui::GetWindowHeight() - ImGui::GetTextLineHeight() - 6.f;
		if (legendX >= lanesRight + 30.f || legendY >= contentBottom + 4.f)
			drawLegend(drawList, ImVec2(legendX, legendY));
	}

	// Reserve what was drawn, so the child scrolls when the panel is small
	ImGui::SetCursorScreenPos(origin);
	ImGui::Dummy(ImVec2(contentRight - origin.x, contentBottom - origin.y));

	ImGui::EndChild();
	ImGui::End();

	if (clicked.isValid())
		m_selected= clicked == m_selected ? PipelineStageId() : clicked;
}
