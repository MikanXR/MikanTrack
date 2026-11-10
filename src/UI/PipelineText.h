#pragma once

#include <string>

#include "imgui.h"

#include "PipelineModel.h"

// What the pipeline panels share to show a stage: its localized name, a
// status text expanded from its key and numbers, and the state's color and dot
namespace PipelineText
{
// A key ending in "Fmt" is expanded with the text's numbers; any other key
// is looked up as it is. Empty for an empty text.
std::string format(const StageText& text);

// The stage's name; a camera's lane reads "Camera <n>"
std::string stageName(const PipelineStageId& stage);

ImU32 stateColor(eStageState state, float alpha= 1.f);
// Running and degraded are filled; inactive and not configured are rings,
// so the four states differ by shape as well as by color
void drawStateDot(ImDrawList* drawList, const ImVec2& center, float radius, eStageState state, float alpha= 1.f);
} // namespace PipelineText
