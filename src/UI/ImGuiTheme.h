#pragma once

#include "imgui.h" // ImWchar

// The app's ImGui look: a dark pastel palette over StyleColorsDark, rounded
// metrics, the Mochiy Pop One UI font (with Japanese glyph coverage), and the
// M PLUS 1 Code monospace font for expression text (with its own Japanese
// glyphs, since a target name can be Japanese). applyStyle also sets the
// ImPlot style every plot draws in. Both are applied once in App::startup,
// after ImGui::CreateContext and ImPlot::CreateContext and before the first
// frame.
namespace ImGuiTheme
{
void applyStyle();
void loadFonts();
// The monospace font, or null when its file is missing (callers then draw in
// the UI font by not pushing one)
ImFont* getMonoFont();
// The exact glyph ranges the font atlas bakes ([start,end] pairs, zero
// terminated). Shared with the localization self-test so translated strings
// are validated against what actually renders.
const ImWchar* getJapaneseGlyphRanges();
} // namespace ImGuiTheme
