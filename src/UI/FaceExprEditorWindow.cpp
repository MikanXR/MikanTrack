#include "FaceExprEditorWindow.h"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "GL/glew.h"

#include "imgui.h"
#include "implot.h"
#include "misc/cpp/imgui_stdlib.h"

#include "glm/ext/matrix_float4x4.hpp"
#include "glm/geometric.hpp"

#include "App.h"
#include "AutomationProtocol.h"
#include "AutomationServer.h"
#include "AvatarSkeleton.h"
#include "AvatarTypes.h"
#include "FaceExprCompiler.h"
#include "FaceExprEditorText.h"
#include "FaceExprFunctions.h"
#include "FaceProfileBinding.h"
#include "FaceTabWidgets.h"
#include "GlFrameBuffer.h"
#include "ImGuiTheme.h"
#include "LocText.h"
#include "Scene3dPanel.h" // displayFromWorld
#include "TrackingTypes.h"

namespace
{
constexpr double k_traceWindowMs= 6000.0;
constexpr size_t k_traceMaxSamples= 2000;
constexpr float k_previewFovDegrees= 30.f;
constexpr float k_previewRadius= 0.45f;
constexpr float k_previewMinRadius= 0.2f;
constexpr float k_previewMaxRadius= 1.5f;
constexpr int k_drivenTargetRows= 8;
// The most lines the header's row picker shows before its list scrolls
constexpr size_t k_rowPickerLines= 18;

// The input series' colors, cycled; the result is white
constexpr ImU32 k_seriesColors[]= {
	IM_COL32(94, 190, 255, 255),  IM_COL32(255, 170, 80, 255),  IM_COL32(120, 230, 140, 255),
	IM_COL32(240, 120, 200, 255), IM_COL32(255, 230, 90, 255),  IM_COL32(160, 140, 255, 255),
	IM_COL32(90, 220, 220, 255),  IM_COL32(255, 120, 120, 255),
};
constexpr ImU32 k_resultColor= IM_COL32(245, 245, 245, 255);
// The trace's line weights in pixels: the result over the inputs over the
// variables
constexpr float k_resultLineWeight= 2.5f;
constexpr float k_inputLineWeight= 1.5f;
constexpr float k_variableLineWeight= 1.f;
// A line whose legend entry is hovered draws this much heavier
constexpr float k_hoveredLineScale= 2.f;
// The gaps between the trace legend's cells
constexpr float k_legendColumnGap= 14.f;
constexpr float k_legendRowGap= 2.f;

// The operators the lexer knows, as the palette lists them. Word-like ones
// get a tooltip; every one inserts with a space around it, bar '!'.
struct PaletteOperator
{
	const char* text;
	const char* tooltipKey;
};
constexpr PaletteOperator k_operators[]= {
	{"||", "exprEditor.op.or"}, {"&&", "exprEditor.op.and"}, {"==", nullptr}, {"!=", nullptr}, {"<", nullptr},
	{"<=", nullptr},            {">", nullptr},               {">=", nullptr}, {"+", nullptr},  {"-", nullptr},
	{"*", nullptr},             {"/", nullptr},               {"!", "exprEditor.op.not"},
};
struct PalettePunctuation
{
	const char* label;
	const char* insert;
};
constexpr PalettePunctuation k_punctuation[]= {{"(", "("}, {")", ")"}, {",", ", "}};

bool containsIgnoringCase(const std::string& text, const std::string& filter)
{
	if (filter.empty())
		return true;
	auto it= std::search(text.begin(), text.end(), filter.begin(), filter.end(), [](char a, char b) {
		return std::tolower((unsigned char)a) == std::tolower((unsigned char)b);
	});
	return it != text.end();
}

// The input slots grouped by namespace, flags (present, tracked) first in
// each group, built once from the constant table
struct InputGroup
{
	std::string prefix;
	std::vector<int> slots;
};
const std::vector<InputGroup>& inputGroups()
{
	static const std::vector<InputGroup> s_groups= [] {
		std::vector<InputGroup> groups;
		for (int slot= 0; slot < FaceExprInputs::k_count; ++slot)
		{
			const std::string name= FaceExprInputs::name(slot);
			const size_t dot= name.find('.');
			const std::string prefix= dot == std::string::npos ? name : name.substr(0, dot);
			auto it= std::find_if(groups.begin(), groups.end(),
								  [&](const InputGroup& group) { return group.prefix == prefix; });
			if (it == groups.end())
			{
				groups.push_back({prefix, {}});
				it= groups.end() - 1;
			}
			it->slots.push_back(slot);
		}
		for (InputGroup& group : groups)
		{
			std::stable_partition(group.slots.begin(), group.slots.end(), [](int slot) {
				const std::string name= FaceExprInputs::name(slot);
				const size_t dot= name.find('.');
				const std::string suffix= dot == std::string::npos ? name : name.substr(dot + 1);
				return suffix == "present" || suffix == "tracked";
			});
		}
		return groups;
	}();
	return s_groups;
}

const char* groupLabel(const std::string& prefix)
{
	if (prefix == "phone")
		return locText("exprEditor.groupPhone");
	if (prefix == "viseme")
		return locText("exprEditor.groupViseme");
	return prefix.c_str();
}

std::string functionSignature(const FaceExprFunctionInfo& function)
{
	std::string signature= function.name;
	signature+= '(';
	for (size_t index= 0; index < function.placeholders.size(); ++index)
	{
		if (index > 0)
			signature+= ", ";
		signature+= function.placeholders[index];
	}
	if (function.maxArgs == INT_MAX)
		signature+= ", ...";
	signature+= ')';
	return signature;
}

bool parseIndexToken(const std::string& token, int& outValue)
{
	if (token.empty())
		return false;
	for (char c : token)
	{
		if (!std::isdigit((unsigned char)c))
			return false;
	}
	outValue= std::atoi(token.c_str());
	return true;
}

std::string formatValue(float value)
{
	char text[32];
	std::snprintf(text, sizeof(text), "%.3f", value);
	return text;
}
} // namespace

FaceExprEditorWindow::FaceExprEditorWindow(App* app)
	: m_app(app)
	, m_frameBuffer(std::make_unique<GlFrameBuffer>())
{
}

FaceExprEditorWindow::~FaceExprEditorWindow()= default;

// -- Opening and closing ------------------------------------------------------

void FaceExprEditorWindow::open(const FaceExprEditTarget& target)
{
	const FaceProfile& profile= m_app->getActiveFaceProfile();
	if (m_app->isActiveFaceProfileAuto())
		return;
	FaceExprEditTarget previous= m_target;
	m_target= target;
	if (!rowExists(profile))
	{
		m_target= previous;
		return;
	}
	m_profileId= profile.id;
	m_openedText= rowText(profile);
	m_bOpen= true;
	m_bRaiseWindow= true;
	m_field.resetCaret();
	m_series.clear();
	m_samples.clear();
	m_seriesInputSlots.clear();
	m_seriesLetIndices.clear();
	m_lastSampleTimestampMs= -1.0;
}

void FaceExprEditorWindow::close()
{
	m_bOpen= false;
	m_samples.clear();
	m_series.clear();
}

// -- The row --------------------------------------------------------------------

bool FaceExprEditorWindow::rowExists(const FaceProfile& profile) const
{
	if (m_target.index < 0)
		return false;
	return m_target.bLet ? (size_t)m_target.index < profile.lets.size() : (size_t)m_target.index < profile.outputs.size();
}

std::string& FaceExprEditorWindow::rowText(FaceProfile& profile) const
{
	return m_target.bLet ? profile.lets[(size_t)m_target.index].expr
						 : profile.outputs[(size_t)m_target.index].vrmExpressionFunction;
}

const std::string& FaceExprEditorWindow::rowText(const FaceProfile& profile) const
{
	return m_target.bLet ? profile.lets[(size_t)m_target.index].expr
						 : profile.outputs[(size_t)m_target.index].vrmExpressionFunction;
}

std::string FaceExprEditorWindow::rowLabel(const FaceProfile& profile) const
{
	return m_target.bLet ? profile.lets[(size_t)m_target.index].name
						 : FaceTabWidgets::targetLabel(profile.outputs[(size_t)m_target.index]);
}

FaceExprEditorWindow::Frame FaceExprEditorWindow::makeFrame(const FaceProfile& profile) const
{
	Frame frame;
	frame.face= &m_shownFace;
	frame.binding= m_app->getFaceBinding();
	frame.errors= &m_app->getFaceProfileErrors();
	if (frame.binding != nullptr)
	{
		const FaceProfile& bound= frame.binding->getProfile();
		frame.bStructural= frame.binding->getProgram() != nullptr && bound.id == profile.id &&
						   bound.outputs.size() == profile.outputs.size() && bound.lets.size() == profile.lets.size();
		frame.bLive= frame.bStructural && m_bHasShownFrame &&
					 m_shownFace.outputs.generation == frame.binding->getGeneration();
	}
	return frame;
}

const FaceExprCompileError* FaceExprEditorWindow::rowError(const Frame& frame, bool bOnName) const
{
	if (frame.errors == nullptr)
		return nullptr;
	for (const FaceExprCompileError& error : *frame.errors)
	{
		if (error.bLet == m_target.bLet && error.index == m_target.index && error.bOnName == bOnName)
			return &error;
	}
	return nullptr;
}

void FaceExprEditorWindow::readReferences(const Frame& frame, std::vector<int>& outInputSlots,
										  std::vector<int>& outLetIndices) const
{
	outInputSlots.clear();
	outLetIndices.clear();
	if (!frame.bStructural)
		return;
	const FaceExprProgram& program= *frame.binding->getProgram();
	const std::vector<FaceExprProgram::Expression>& expressions= m_target.bLet ? program.lets : program.outputs;
	if ((size_t)m_target.index >= expressions.size())
		return;
	outInputSlots= expressions[(size_t)m_target.index].directInputs;
	outLetIndices= expressions[(size_t)m_target.index].directLets;
}

float FaceExprEditorWindow::rowValue(const Frame& frame) const
{
	if (!frame.bLive)
		return 0.f;
	const std::vector<float>& values= m_target.bLet ? frame.face->outputs.letValues : frame.face->outputs.outputValues;
	return (size_t)m_target.index < values.size() ? values[(size_t)m_target.index] : 0.f;
}

// -- Per frame ------------------------------------------------------------------

void FaceExprEditorWindow::update(const TrackingFrameResult& shown)
{
	m_shownFace= shown.face;
	m_shownTimestampMs= shown.timestampMs;
	m_bHasShownFrame= true;
	FaceExprInputs::fill(m_shownFace, m_inputValues);

	if (!m_bOpen)
	{
		// The GPU copy of the avatar lives only while the editor shows it
		if (m_meshView.hasModel())
			m_meshView.clear();
		return;
	}

	const FaceProfile& active= m_app->getActiveFaceProfile();
	if (m_app->isActiveFaceProfileAuto() || active.id != m_profileId || !rowExists(active))
	{
		close();
		return;
	}
	m_profile= active;
	FaceProfile& profile= m_profile;
	const Frame frame= makeFrame(profile);
	syncSeries(profile, frame);
	pushSample(frame);

	ImGui::SetNextWindowSize(ImVec2(1180.f, 720.f), ImGuiCond_FirstUseEver);
	if (m_bRaiseWindow)
	{
		ImGui::SetNextWindowFocus();
		m_bRaiseWindow= false;
	}
	bool bStayOpen= true;
	if (!ImGui::Begin(locWindowTitle("windows.expressionEditor"), &bStayOpen))
	{
		ImGui::End();
		if (!bStayOpen)
			close();
		return;
	}

	bool bChanged= false;
	drawHeader(profile, frame);
	ImGui::Separator();
	if (ImGui::BeginTable("##columns", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV))
	{
		ImGui::TableSetupColumn("##inputs", ImGuiTableColumnFlags_WidthFixed, 270.f);
		ImGui::TableSetupColumn("##function", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("##preview", ImGuiTableColumnFlags_WidthFixed, 320.f);
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		drawInputs(profile, frame);
		ImGui::TableNextColumn();
		bChanged|= drawFunction(profile, frame);
		ImGui::TableNextColumn();
		drawPreview(frame);
		ImGui::EndTable();
	}
	ImGui::End();

	if (bChanged)
		m_app->editActiveFaceProfile(profile);
	// A row picked in the header takes effect once this draw is over
	const FaceExprEditTarget switchTarget= m_switchTarget;
	m_switchTarget= FaceExprEditTarget();
	if (!bStayOpen)
		close();
	else if (switchTarget.index >= 0)
		open(switchTarget);
}

void FaceExprEditorWindow::drawHeader(const FaceProfile& profile, const Frame& frame)
{
	if (!ImGui::BeginTable("##header", 4, ImGuiTableFlags_SizingFixedFit))
		return;
	ImGui::TableSetupColumn("##kind", ImGuiTableColumnFlags_WidthFixed);
	ImGui::TableSetupColumn("##target", ImGuiTableColumnFlags_WidthFixed, 320.f);
	ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthStretch);
	ImGui::TableSetupColumn("##revert", ImGuiTableColumnFlags_WidthFixed);
	ImGui::TableNextRow();

	ImGui::TableNextColumn();
	ImGui::AlignTextToFramePadding();
	ImGui::TextDisabled("%s", locText(m_target.bLet ? "exprEditor.kindVariable" : "exprEditor.kindBinding"));

	// The row picker: every row of the profile, the variables and then the
	// bindings. Picking one is the Edit button on that row. The row's name
	// and target are edited in the Profile stage.
	ImGui::TableNextColumn();
	ImGui::SetNextItemWidth(-FLT_MIN);
	if (ImGui::BeginCombo("##row", rowLabel(profile).c_str(), ImGuiComboFlags_HeightLargest))
	{
		bool bPicked= false;
		// The filter stays put above the list. It starts empty and takes the
		// keyboard as the picker opens, and the list opens scrolled to the
		// row being edited.
		const bool bAppearing= ImGui::IsWindowAppearing();
		if (bAppearing)
		{
			m_rowFilter.clear();
			ImGui::SetKeyboardFocusHere();
		}
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::InputTextWithHint("##filter", locText("exprEditor.filterHint"), &m_rowFilter);
		const size_t lineCount= profile.lets.size() + profile.outputs.size() + 2;
		const float listHeight= ImGui::GetTextLineHeightWithSpacing() * (float)std::min(lineCount, k_rowPickerLines);
		if (ImGui::BeginChild("##rows", ImVec2(0.f, listHeight)))
		{
			auto drawRow= [&](bool bLet, size_t index, const std::string& label) {
				if (!containsIgnoringCase(label, m_rowFilter))
					return;
				const FaceExprEditTarget target{bLet, (int)index};
				const bool bCurrent= target == m_target;
				ImGui::PushID(bLet ? "let" : "out");
				ImGui::PushID((int)index);
				if (ImGui::Selectable(label.c_str(), bCurrent) && !bCurrent)
				{
					m_switchTarget= target;
					bPicked= true;
				}
				if (bCurrent && bAppearing)
					ImGui::SetScrollHereY();
				ImGui::PopID();
				ImGui::PopID();
			};
			if (!profile.lets.empty())
			{
				ImGui::SeparatorText(locText("faceTab.variablesHeader"));
				for (size_t letIndex= 0; letIndex < profile.lets.size(); ++letIndex)
					drawRow(true, letIndex, profile.lets[letIndex].name);
			}
			if (!profile.outputs.empty())
			{
				ImGui::SeparatorText(locText("faceTab.bindingsHeader"));
				for (size_t outputIndex= 0; outputIndex < profile.outputs.size(); ++outputIndex)
					drawRow(false, outputIndex, FaceTabWidgets::targetLabel(profile.outputs[outputIndex]));
			}
		}
		ImGui::EndChild();
		// A Selectable inside the child does not close the picker by itself
		if (bPicked)
			ImGui::CloseCurrentPopup();
		ImGui::EndCombo();
	}
	if (m_target.bLet)
	{
		const FaceExprCompileError* nameError= rowError(frame, true);
		if (nameError != nullptr)
		{
			ImGui::PushStyleColor(ImGuiCol_Text, FaceTabWidgets::k_errorColor);
			ImGui::TextWrapped("%s", FaceTabWidgets::errorText(*nameError).c_str());
			ImGui::PopStyleColor();
		}
	}
	else if (frame.bStructural &&
			 frame.binding->getOutputProblemCodes()[(size_t)m_target.index] != eFaceTargetProblem::None)
	{
		ImGui::PushStyleColor(ImGuiCol_Text, FaceTabWidgets::k_warningColor);
		ImGui::TextWrapped("%s", locFormat("faceTab.unresolvedTooltipFmt",
										   FaceTabWidgets::problemText(*frame.binding, (size_t)m_target.index).c_str())
									 .c_str());
		ImGui::PopStyleColor();
	}

	ImGui::TableNextColumn();
	const float value= rowValue(frame);
	char overlay[32];
	std::snprintf(overlay, sizeof(overlay), "%.2f", value);
	ImGui::ProgressBar(std::clamp(value, 0.f, 1.f), ImVec2(-FLT_MIN, 0.f), frame.bLive ? overlay : "");

	ImGui::TableNextColumn();
	ImGui::BeginDisabled(rowText(profile) == m_openedText);
	if (ImGui::Button(locLabel("exprEditor.revertButton")))
		revert();
	ImGui::EndDisabled();
	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
		ImGui::SetTooltip("%s", locText("exprEditor.revertTooltip"));

	ImGui::EndTable();
}

// -- Inputs ---------------------------------------------------------------------

void FaceExprEditorWindow::drawInputs(const FaceProfile& profile, const Frame& frame)
{
	ImGui::TextDisabled("%s", locText("exprEditor.inputsHeader"));
	ImGui::SetNextItemWidth(-FLT_MIN);
	ImGui::InputTextWithHint("##filter", locText("exprEditor.filterHint"), &m_inputFilter);

	std::vector<int> referencedSlots;
	std::vector<int> referencedLets;
	readReferences(frame, referencedSlots, referencedLets);

	if (!ImGui::BeginChild("##inputList", ImVec2(0.f, 0.f)))
	{
		ImGui::EndChild();
		return;
	}
	constexpr ImGuiTableFlags k_tableFlags= ImGuiTableFlags_SizingStretchProp;
	auto drawRow= [&](const char* id, const std::string& name, bool bReferenced, bool bHasValue, float value,
					  bool bDisabled) {
		ImGui::PushID(id);
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		ImGui::BeginDisabled(bDisabled);
		const bool bClicked= ImGui::Selectable(name.c_str(), bReferenced, ImGuiSelectableFlags_SpanAllColumns);
		ImGui::EndDisabled();
		ImGui::TableNextColumn();
		if (bHasValue)
		{
			char text[32];
			std::snprintf(text, sizeof(text), "%.2f", value);
			const float width= ImGui::CalcTextSize(text).x;
			ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.f, ImGui::GetContentRegionAvail().x - width));
			ImGui::TextUnformatted(text);
		}
		else
		{
			ImGui::TextDisabled("-");
		}
		ImGui::PopID();
		if (bClicked)
			queueInsert(profile, name);
	};

	for (const InputGroup& group : inputGroups())
	{
		ImGui::PushID(group.prefix.c_str());
		if (ImGui::CollapsingHeader(groupLabel(group.prefix), ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::BeginTable("##rows", 2, k_tableFlags))
			{
				ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthStretch);
				ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthFixed, 44.f);
				for (int slot : group.slots)
				{
					const std::string name= FaceExprInputs::name(slot);
					if (!containsIgnoringCase(name, m_inputFilter))
						continue;
					const bool bReferenced=
						std::find(referencedSlots.begin(), referencedSlots.end(), slot) != referencedSlots.end();
					drawRow(name.c_str(), name, bReferenced, m_bHasShownFrame, m_inputValues[slot], false);
				}
				ImGui::EndTable();
			}
		}
		ImGui::PopID();
	}

	ImGui::PushID("variables");
	if (ImGui::CollapsingHeader(locText("exprEditor.groupVariables"), ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::BeginTable("##rows", 2, k_tableFlags))
		{
			ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthStretch);
			ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthFixed, 44.f);
			for (size_t letIndex= 0; letIndex < profile.lets.size(); ++letIndex)
			{
				const std::string& name= profile.lets[letIndex].name;
				if (!containsIgnoringCase(name, m_inputFilter))
					continue;
				const bool bReferenced=
					std::find(referencedLets.begin(), referencedLets.end(), (int)letIndex) != referencedLets.end();
				const bool bHasValue= frame.bLive && letIndex < frame.face->outputs.letValues.size();
				const float value= bHasValue ? frame.face->outputs.letValues[letIndex] : 0.f;
				// The variable being edited cannot read itself
				const bool bSelf= m_target.bLet && (int)letIndex == m_target.index;
				char id[16];
				std::snprintf(id, sizeof(id), "%zu", letIndex);
				drawRow(id, name, bReferenced, bHasValue, value, bSelf);
			}
			ImGui::EndTable();
		}
	}
	ImGui::PopID();
	ImGui::EndChild();
}

void FaceExprEditorWindow::queueInsert(const FaceProfile& profile, const std::string& insert)
{
	const std::string& text= rowText(profile);
	int begin= 0;
	int end= 0;
	m_field.getSelection(text, begin, end);
	m_field.requestEdit(insertFaceExprText(text, begin, end, insert));
}

// -- The function, the trace, the palette -------------------------------------------

bool FaceExprEditorWindow::drawFunction(FaceProfile& profile, const Frame& frame)
{
	ImGui::TextDisabled("%s", locText("exprEditor.functionHeader"));
	const FaceExprCompileError* error= rowError(frame, false);
	const bool bChanged= m_field.draw("##expr", rowText(profile), error, ImGuiTheme::getMonoFont());
	if (error != nullptr)
	{
		ImGui::PushStyleColor(ImGuiCol_Text, FaceTabWidgets::k_errorColor);
		ImGui::TextWrapped("%s", FaceTabWidgets::errorText(*error).c_str());
		ImGui::PopStyleColor();
	}
	else
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", locText("exprEditor.compilesText"));
		ImGui::PopStyleColor();
	}

	// The palette keeps its three rows at the bottom; the trace takes the rest
	const float paletteHeight= ImGui::GetTextLineHeightWithSpacing() * 3.f + ImGui::GetFrameHeightWithSpacing() * 3.f +
							   ImGui::GetStyle().ItemSpacing.y * 2.f;
	const float traceHeight= std::max(140.f, ImGui::GetContentRegionAvail().y - paletteHeight);
	if (ImGui::BeginChild("##trace", ImVec2(0.f, traceHeight)))
		drawTrace();
	ImGui::EndChild();
	drawPalette(profile);
	return bChanged;
}

void FaceExprEditorWindow::syncSeries(const FaceProfile& profile, const Frame& frame)
{
	std::vector<int> inputSlots;
	std::vector<int> letIndices;
	readReferences(frame, inputSlots, letIndices);
	const bool bSameSet= !m_series.empty() && inputSlots == m_seriesInputSlots && letIndices == m_seriesLetIndices;
	if (bSameSet)
	{
		// Names follow the profile (a renamed variable, a retargeted binding)
		m_series[0].label= rowLabel(profile);
		for (Series& series : m_series)
		{
			if (series.bLet && (size_t)series.index < profile.lets.size())
				series.label= profile.lets[(size_t)series.index].name;
		}
		return;
	}

	// A series that stays in the set keeps its legend toggle
	const std::vector<Series> previous= std::move(m_series);
	auto wasOn= [&previous](bool bLet, int index) {
		for (const Series& series : previous)
		{
			if (series.bLet == bLet && series.index == index)
				return series.bOn;
		}
		return true;
	};

	m_series.clear();
	Series result;
	result.label= rowLabel(profile);
	result.color= k_resultColor;
	result.bOn= wasOn(false, -1);
	m_series.push_back(result);
	size_t colorIndex= 0;
	const size_t colorCount= sizeof(k_seriesColors) / sizeof(k_seriesColors[0]);
	for (int slot : inputSlots)
	{
		Series series;
		series.index= slot;
		series.label= FaceExprInputs::name(slot);
		series.color= k_seriesColors[colorIndex++ % colorCount];
		series.bOn= wasOn(false, slot);
		m_series.push_back(series);
	}
	for (int letIndex : letIndices)
	{
		Series series;
		series.bLet= true;
		series.index= letIndex;
		series.label= (size_t)letIndex < profile.lets.size() ? profile.lets[(size_t)letIndex].name : std::string("?");
		series.color= k_seriesColors[colorIndex++ % colorCount];
		series.bOn= wasOn(true, letIndex);
		m_series.push_back(series);
	}
	m_seriesInputSlots= inputSlots;
	m_seriesLetIndices= letIndices;
	m_legendHoveredSeries= -1;
	m_samples.clear();
	m_lastSampleTimestampMs= -1.0;
}

void FaceExprEditorWindow::pushSample(const Frame& frame)
{
	if (!m_bHasShownFrame || m_shownTimestampMs == m_lastSampleTimestampMs)
		return;
	// A seek backwards in a replay starts the trace over
	if (m_shownTimestampMs < m_lastSampleTimestampMs)
		m_samples.clear();
	m_lastSampleTimestampMs= m_shownTimestampMs;

	Sample sample;
	sample.timestampMs= m_shownTimestampMs;
	sample.values.reserve(m_series.size());
	for (const Series& series : m_series)
	{
		float value= 0.f;
		if (series.index < 0)
			value= rowValue(frame);
		else if (series.bLet)
			value= frame.bLive && (size_t)series.index < frame.face->outputs.letValues.size()
					   ? frame.face->outputs.letValues[(size_t)series.index]
					   : 0.f;
		else
			value= m_inputValues[series.index];
		sample.values.push_back(value);
	}
	m_samples.push_back(std::move(sample));
	while (!m_samples.empty() &&
		   (m_samples.front().timestampMs < m_shownTimestampMs - k_traceWindowMs || m_samples.size() > k_traceMaxSamples))
		m_samples.pop_front();
}

void FaceExprEditorWindow::drawTrace()
{
	ImGui::TextDisabled("%s", locText("exprEditor.traceHeader"));
	ImGui::SameLine();
	ImGui::TextDisabled("%s", locText("exprEditor.traceNote"));

	// 0..1 unless a shown series leaves it
	float yMin= 0.f;
	float yMax= 1.f;
	for (const Sample& sample : m_samples)
	{
		for (size_t index= 0; index < sample.values.size() && index < m_series.size(); ++index)
		{
			if (!m_series[index].bOn || !std::isfinite(sample.values[index]))
				continue;
			yMin= std::min(yMin, sample.values[index]);
			yMax= std::max(yMax, sample.values[index]);
		}
	}
	if (yMax - yMin < 1e-3f)
		yMax= yMin + 1e-3f;

	// The legend is a grid under the plot that wraps at the plot's width.
	// Every cell is as wide as the longest name plus a fixed value slot, so
	// the grid does not reflow as the digits change.
	const ImGuiStyle& style= ImGui::GetStyle();
	const float lineHeight= ImGui::GetTextLineHeight();
	float labelWidth= 0.f;
	for (const Series& series : m_series)
		labelWidth= std::max(labelWidth, ImGui::CalcTextSize(series.label.c_str()).x);
	const float cellWidth= lineHeight + labelWidth + style.ItemInnerSpacing.x + ImGui::CalcTextSize("-0.00").x;
	const float cellPitch= cellWidth + k_legendColumnGap;
	const ImVec2 avail= ImGui::GetContentRegionAvail();
	const int columnCount= std::max(1, (int)((avail.x + k_legendColumnGap) / cellPitch));
	const int rowCount= ((int)m_series.size() + columnCount - 1) / columnCount;
	const float rowPitch= lineHeight + k_legendRowGap;
	const float legendHeight= rowCount > 0 ? (float)rowCount * rowPitch - k_legendRowGap + style.ItemSpacing.y : 0.f;

	// The axes are locked: time is seconds before the shown frame, and the
	// value range is the one fitted above
	const ImVec2 plotSize(std::max(avail.x, 60.f), std::max(avail.y - legendHeight, 80.f));
	constexpr ImPlotFlags k_plotFlags= ImPlotFlags_NoTitle | ImPlotFlags_NoLegend | ImPlotFlags_NoMenus |
									   ImPlotFlags_NoBoxSelect | ImPlotFlags_NoMouseText;
	constexpr ImPlotAxisFlags k_axisFlags=
		ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoHighlight | ImPlotAxisFlags_NoSideSwitch;
	if (ImPlot::BeginPlot("##trace", plotSize, k_plotFlags))
	{
		ImPlot::SetupAxes(nullptr, nullptr, k_axisFlags, k_axisFlags);
		ImPlot::SetupAxisLimits(ImAxis_X1, -k_traceWindowMs / 1000.0, 0.0, ImPlotCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, (double)yMin, (double)yMax, ImPlotCond_Always);
		ImPlot::SetupAxisFormat(ImAxis_X1, "%g s");
		drawTracePlot(yMin, yMax);
		ImPlot::EndPlot();
	}

	const Sample* last= m_samples.empty() ? nullptr : &m_samples.back();
	const ImVec2 origin= ImGui::GetCursorScreenPos();
	ImDrawList* drawList= ImGui::GetWindowDrawList();
	const ImU32 textColor= ImGui::GetColorU32(ImGuiCol_Text);
	const ImU32 offColor= ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.5f);
	m_legendHoveredSeries= -1;
	for (size_t seriesIndex= 0; seriesIndex < m_series.size(); ++seriesIndex)
	{
		Series& series= m_series[seriesIndex];
		const ImVec2 cellMin(origin.x + (float)((int)seriesIndex % columnCount) * cellPitch,
							 origin.y + (float)((int)seriesIndex / columnCount) * rowPitch);
		ImGui::SetCursorScreenPos(cellMin);
		ImGui::PushID((int)seriesIndex);
		if (ImGui::InvisibleButton("##legend", ImVec2(cellWidth, lineHeight)))
			series.bOn= !series.bOn;
		ImGui::PopID();
		const bool bHovered= ImGui::IsItemHovered();
		if (bHovered)
			m_legendHoveredSeries= (int)seriesIndex;

		// A color swatch, the name, and the live value against the cell's
		// right edge
		constexpr float k_swatchInset= 2.f;
		const ImU32 swatchColor= series.bOn ? series.color : offColor;
		const ImU32 labelColor= !series.bOn ? offColor : bHovered ? series.color : textColor;
		drawList->AddRectFilled(ImVec2(cellMin.x + k_swatchInset, cellMin.y + k_swatchInset),
								ImVec2(cellMin.x + lineHeight - k_swatchInset, cellMin.y + lineHeight - k_swatchInset),
								swatchColor);
		drawList->AddText(ImVec2(cellMin.x + lineHeight, cellMin.y), labelColor, series.label.c_str());
		if (last != nullptr && seriesIndex < last->values.size())
		{
			char value[32];
			std::snprintf(value, sizeof(value), "%.2f", last->values[seriesIndex]);
			drawList->AddText(ImVec2(cellMin.x + cellWidth - ImGui::CalcTextSize(value).x, cellMin.y),
							  series.bOn ? textColor : offColor, value);
		}
	}
}

void FaceExprEditorWindow::drawTracePlot(float yMin, float yMax)
{
	struct PointSource
	{
		const std::deque<Sample>* samples;
		size_t seriesIndex;
		double shownTimestampMs;
	};
	const ImPlotGetter getPoint= [](int index, void* data) -> ImPlotPoint {
		const PointSource& source= *static_cast<const PointSource*>(data);
		const Sample& sample= (*source.samples)[(size_t)index];
		// A missing or non-finite value is a gap in the line
		double value= std::numeric_limits<double>::quiet_NaN();
		if (source.seriesIndex < sample.values.size() && std::isfinite(sample.values[source.seriesIndex]))
			value= (double)sample.values[source.seriesIndex];
		return ImPlotPoint((sample.timestampMs - source.shownTimestampMs) / 1000.0, value);
	};

	auto plotSeries= [&](size_t seriesIndex) {
		const Series& series= m_series[seriesIndex];
		if (!series.bOn)
			return;
		ImPlotSpec spec;
		spec.LineColor= ImGui::ColorConvertU32ToFloat4(series.color);
		spec.LineWeight= series.index < 0 ? k_resultLineWeight : series.bLet ? k_variableLineWeight : k_inputLineWeight;
		if ((int)seriesIndex == m_legendHoveredSeries)
			spec.LineWeight*= k_hoveredLineScale;
		const std::string label= "##" + std::to_string(seriesIndex);
		PointSource source{&m_samples, seriesIndex, m_shownTimestampMs};
		ImPlot::PlotLineG(label.c_str(), getPoint, &source, (int)m_samples.size(), spec);
	};
	// The result draws last, over what it reads
	for (size_t seriesIndex= 1; seriesIndex < m_series.size(); ++seriesIndex)
		plotSeries(seriesIndex);
	if (!m_series.empty())
		plotSeries(0);

	// Hover: a cursor at the nearest sample and every shown series' value there
	if (ImPlot::IsPlotHovered() && !m_samples.empty())
	{
		const double hoverMs= m_shownTimestampMs + ImPlot::GetPlotMousePos().x * 1000.0;
		auto nearest= std::lower_bound(m_samples.begin(), m_samples.end(), hoverMs,
									   [](const Sample& sample, double timestampMs) { return sample.timestampMs < timestampMs; });
		if (nearest == m_samples.end())
			--nearest;
		else if (nearest != m_samples.begin() && hoverMs - (nearest - 1)->timestampMs < nearest->timestampMs - hoverMs)
			--nearest;
		const double seconds= (nearest->timestampMs - m_shownTimestampMs) / 1000.0;

		ImPlot::PushPlotClipRect();
		ImPlot::GetPlotDrawList()->AddLine(ImPlot::PlotToPixels(ImPlotPoint(seconds, (double)yMax)),
										   ImPlot::PlotToPixels(ImPlotPoint(seconds, (double)yMin)),
										   IM_COL32(255, 255, 255, 110), 1.f);
		ImPlot::PopPlotClipRect();

		ImGui::BeginTooltip();
		ImGui::TextDisabled("%.2f s", seconds);
		for (size_t seriesIndex= 0; seriesIndex < m_series.size() && seriesIndex < nearest->values.size(); ++seriesIndex)
		{
			const Series& series= m_series[seriesIndex];
			if (!series.bOn)
				continue;
			ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(series.color), "%s", series.label.c_str());
			ImGui::SameLine();
			ImGui::Text("%.2f", nearest->values[seriesIndex]);
		}
		ImGui::EndTooltip();
	}
}

bool FaceExprEditorWindow::paletteButton(const char* label, bool bFirstInRow)
{
	const float width= ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x * 2.f;
	if (!bFirstInRow)
	{
		ImGui::SameLine();
		if (ImGui::GetContentRegionAvail().x < width)
			ImGui::NewLine();
	}
	return ImGui::SmallButton(label);
}

void FaceExprEditorWindow::drawPalette(const FaceProfile& profile)
{
	const std::string& text= rowText(profile);
	int begin= 0;
	int end= 0;

	ImGui::TextDisabled("%s", locText("exprEditor.functionsRow"));
	bool bFirst= true;
	for (const FaceExprFunctionInfo& function : faceExprFunctions())
	{
		const std::string label= std::string(function.name) + "()";
		if (function.bStateful)
			ImGui::PushStyleColor(ImGuiCol_Text, FaceTabWidgets::k_warningColor);
		const bool bPressed= paletteButton(label.c_str(), bFirst);
		if (function.bStateful)
			ImGui::PopStyleColor();
		bFirst= false;
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
		{
			const std::string noteKey= std::string("exprEditor.fn.") + function.name;
			ImGui::SetTooltip("%s\n%s", functionSignature(function).c_str(), locText(noteKey.c_str()));
		}
		if (bPressed)
		{
			m_field.getSelection(text, begin, end);
			m_field.requestEdit(insertFaceExprFunction(text, begin, end, function));
		}
	}

	ImGui::TextDisabled("%s", locText("exprEditor.operatorsRow"));
	bFirst= true;
	for (const PaletteOperator& op : k_operators)
	{
		const bool bPressed= paletteButton(op.text, bFirst);
		bFirst= false;
		if (op.tooltipKey != nullptr && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
			ImGui::SetTooltip("%s", locText(op.tooltipKey));
		if (bPressed)
		{
			const std::string insert= op.text[0] == '!' && op.text[1] == '\0' ? std::string("!")
																			  : std::string(" ") + op.text + " ";
			queueInsert(profile, insert);
		}
	}

	ImGui::TextDisabled("%s", locText("exprEditor.punctuationRow"));
	bFirst= true;
	for (const PalettePunctuation& punctuation : k_punctuation)
	{
		if (paletteButton(punctuation.label, bFirst))
			queueInsert(profile, punctuation.insert);
		bFirst= false;
	}
}

// -- The preview ------------------------------------------------------------------

void FaceExprEditorWindow::frameHead(const AvatarSkeleton& skeleton)
{
	const AvatarSkeleton::Bone& head= skeleton.getBone(eHumanoidBone::Head);
	glm::vec3 target(0.f, 1.5f, 0.f);
	if (head.present)
		target= glm::vec3(displayFromWorld() * glm::vec4(head.restPositionWorld, 1.f));
	target.y+= 0.06f;
	m_camera.setOrbitTargetPosition(target);
	m_camera.setOrbitLocation(90.f, 0.f, k_previewRadius);
}

bool FaceExprEditorWindow::previewTargetValues(const Frame& frame, std::vector<float>& outTargetValues) const
{
	if (!frame.bLive)
		return false;
	const FaceProfileBinding& binding= *frame.binding;
	const FaceSources::Outputs& outputs= frame.face->outputs;
	if (!m_bSolo)
	{
		if (outputs.targetValues.size() != binding.getTargets().size())
			return false;
		outTargetValues= outputs.targetValues;
		return true;
	}

	// Solo: the rest face plus this row's own targets. A variable reaches
	// every output reading it, each at its full value.
	outTargetValues.assign(binding.getTargets().size(), 0.f);
	std::vector<int> outputIndices;
	if (m_target.bLet)
		outputIndices= binding.outputsReadingLet(m_target.index);
	else
		outputIndices.push_back(m_target.index);
	const std::vector<int>& outputTargets= binding.getOutputTargets();
	for (int outputIndex : outputIndices)
	{
		if (outputIndex < 0 || (size_t)outputIndex >= outputTargets.size() ||
			(size_t)outputIndex >= outputs.outputValues.size())
			continue;
		const int target= outputTargets[(size_t)outputIndex];
		if (target >= 0)
			outTargetValues[(size_t)target]= std::max(outTargetValues[(size_t)target], outputs.outputValues[(size_t)outputIndex]);
	}
	return true;
}

void FaceExprEditorWindow::drawPreview(const Frame& frame)
{
	ImGui::TextDisabled("%s", locText("exprEditor.previewHeader"));
	ImGui::Checkbox(locLabel("exprEditor.soloCheckbox"), &m_bSolo);
	ImGui::SameLine();
	if (ImGui::Button(locLabel("exprEditor.resetViewButton")))
		m_framedSkeleton= nullptr;
	ImGui::TextDisabled("%s", locText(m_bSolo ? "exprEditor.soloOnNote" : "exprEditor.soloOffNote"));

	const std::shared_ptr<const AvatarModel> model= m_app->getAvatarModel();
	const std::shared_ptr<const AvatarSkeleton> skeleton= m_app->getAvatarSkeleton();
	if (model == nullptr || skeleton == nullptr)
	{
		m_meshView.clear();
		ImGui::TextDisabled("%s", locText("exprEditor.noAvatarText"));
		return;
	}
	m_meshView.setAvatar(model, m_app->getAvatarGeneration());
	if (m_framedSkeleton != skeleton.get())
	{
		frameHead(*skeleton);
		m_framedSkeleton= skeleton.get();
	}

	// The image takes what the driven-target table leaves
	const float tableHeight= ImGui::GetTextLineHeightWithSpacing() * (float)(k_drivenTargetRows + 1) + 8.f;
	const ImVec2 avail= ImGui::GetContentRegionAvail();
	const uint16_t width= (uint16_t)std::max(64.f, avail.x);
	const uint16_t height= (uint16_t)std::max(120.f, avail.y - tableHeight);
	if (!m_frameBuffer->getIsValid())
	{
		if (!m_frameBuffer->init(width, height))
		{
			ImGui::TextDisabled("%s", locText("scene3dPanel.rendererInitFailedText"));
			return;
		}
	}
	else
	{
		m_frameBuffer->resize(width, height);
	}

	const bool bFits= frame.binding != nullptr && frame.binding->fitsModel(*model);
	if (bFits && previewTargetValues(frame, m_previewTargetValues))
	{
		frame.binding->evaluateMorphs(&m_previewTargetValues, m_morphWeights);
	}
	else
	{
		m_previewTargetValues.clear();
		m_morphWeights.resize(model->meshes.size());
		for (size_t meshIndex= 0; meshIndex < model->meshes.size(); ++meshIndex)
			m_morphWeights[meshIndex].assign(model->meshes[meshIndex].morphTargetNames.size(), 0.f);
	}

	m_frameBuffer->bindFrameBuffer();
	glClearColor(0.05f, 0.05f, 0.07f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	m_camera.setPerspectiveProjection(k_previewFovDegrees, (float)width / (float)height, 0.01f, 100.f);
	const glm::mat4 modelMatrix= displayFromWorld() * skeleton->getWorldFromAvatar();
	const glm::vec3 lightDirection= glm::normalize(glm::vec3(0.3f, 1.f, 0.8f));
	m_meshView.draw(m_camera.getViewProjection(), modelMatrix, skeleton->getRestGlobalsAvatar(), lightDirection,
					m_morphWeights);
	m_frameBuffer->unbindFrameBuffer();

	// FBO textures are bottom-up; flip V
	ImGui::Image((ImTextureID)(intptr_t)m_frameBuffer->getColorTextureId(), ImVec2((float)width, (float)height),
				 ImVec2(0, 1), ImVec2(1, 0));
	if (ImGui::IsItemHovered())
	{
		const ImGuiIO& io= ImGui::GetIO();
		if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
			m_camera.adjustOrbitAngles(io.MouseDelta.x * 0.4f, -io.MouseDelta.y * 0.4f);
		if (io.MouseWheel != 0.f)
		{
			const float radius=
				std::clamp(m_camera.getOrbitRadius() - io.MouseWheel * 0.05f, k_previewMinRadius, k_previewMaxRadius);
			m_camera.setOrbitLocation(m_camera.getOrbitYawDegrees(), m_camera.getOrbitPitchDegrees(), radius);
		}
	}

	// The targets the preview drives, strongest first
	std::vector<std::pair<float, int>> driven;
	for (size_t target= 0; target < m_previewTargetValues.size(); ++target)
	{
		if (m_previewTargetValues[target] > 0.001f)
			driven.emplace_back(m_previewTargetValues[target], (int)target);
	}
	std::sort(driven.begin(), driven.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
	if (driven.empty())
	{
		ImGui::TextDisabled("%s", locText("exprEditor.nothingDrivenText"));
		return;
	}
	if (ImGui::BeginTable("##driven", 2, ImGuiTableFlags_SizingStretchProp))
	{
		ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthFixed, 44.f);
		const std::vector<FaceProfileBinding::Target>& targets= frame.binding->getTargets();
		for (size_t row= 0; row < driven.size() && row < (size_t)k_drivenTargetRows; ++row)
		{
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(targets[(size_t)driven[row].second].name.c_str());
			ImGui::TableNextColumn();
			ImGui::Text("%.2f", driven[row].first);
		}
		ImGui::EndTable();
	}
}

// -- Programmatic edits and automation ------------------------------------------

void FaceExprEditorWindow::setText(const std::string& text)
{
	if (!m_bOpen)
		return;
	FaceProfile profile= m_app->getActiveFaceProfile();
	if (profile.id != m_profileId || !rowExists(profile))
		return;
	rowText(profile)= text;
	m_app->editActiveFaceProfile(profile);
	FaceExprTextEdit edit;
	edit.text= text;
	edit.selectionBegin= (int)text.size();
	edit.selectionEnd= (int)text.size();
	m_field.requestEdit(edit);
}

void FaceExprEditorWindow::revert()
{
	setText(m_openedText);
}

FaceExprEditorWindow::Status FaceExprEditorWindow::getStatus() const
{
	Status status;
	status.bOpen= m_bOpen;
	status.bSolo= m_bSolo;
	if (!m_bOpen)
		return status;
	const FaceProfile& profile= m_app->getActiveFaceProfile();
	if (profile.id != m_profileId || !rowExists(profile))
		return status;
	status.target= m_target;
	status.rowLabel= rowLabel(profile);
	status.text= rowText(profile);
	const Frame frame= makeFrame(profile);
	status.bLive= frame.bLive;
	status.value= rowValue(frame);
	const FaceExprCompileError* error= rowError(frame, false);
	if (error == nullptr)
		error= rowError(frame, true);
	if (error != nullptr)
		status.error= error->message;
	std::vector<int> inputSlots;
	std::vector<int> letIndices;
	readReferences(frame, inputSlots, letIndices);
	for (int slot : inputSlots)
		status.reads.push_back(FaceExprInputs::name(slot));
	for (int letIndex : letIndices)
	{
		if ((size_t)letIndex < profile.lets.size())
			status.reads.push_back(profile.lets[(size_t)letIndex].name);
	}
	return status;
}

void FaceExprEditorWindow::registerAutomation(AutomationServer* server)
{
	server->registerCommandNamespace(
		"face",
		{"face profile", "face rows", "face edit let|out <index>", "face expr [<text...>]", "face value",
		 "face solo on|off", "face revert", "face close"},
		[this](const std::vector<std::string>& args, std::vector<std::string>& outLines, std::string& outError) {
			return handleAutomation(args, outLines, outError);
		});
}

bool FaceExprEditorWindow::handleAutomation(const std::vector<std::string>& args, std::vector<std::string>& outLines,
											std::string& outError)
{
	if (args.empty())
	{
		outError= "missing verb";
		return false;
	}
	const std::string& verb= args[0];
	const FaceProfile& profile= m_app->getActiveFaceProfile();

	if (verb == "profile")
	{
		outLines.push_back("name " + profile.name);
		outLines.push_back("id " + profile.id);
		outLines.push_back(std::string("auto ") + (m_app->isActiveFaceProfileAuto() ? "1" : "0"));
		outLines.push_back("variables " + std::to_string(profile.lets.size()));
		outLines.push_back("bindings " + std::to_string(profile.outputs.size()));
		outLines.push_back("errors " + std::to_string(m_app->getFaceProfileErrors().size()));
		return true;
	}
	if (verb == "rows")
	{
		// The rows read through a frame on the whole profile, so the live
		// rule is the editor's own
		const Frame frame= makeFrame(profile);
		const std::vector<FaceExprCompileError>& errors= m_app->getFaceProfileErrors();
		auto errorFor= [&](bool bLet, int index) -> const FaceExprCompileError* {
			for (const FaceExprCompileError& error : errors)
			{
				if (error.bLet == bLet && error.index == index)
					return &error;
			}
			return nullptr;
		};
		for (size_t letIndex= 0; letIndex < profile.lets.size(); ++letIndex)
		{
			const FaceProfileLet& let= profile.lets[letIndex];
			std::string line= "let " + std::to_string(letIndex) + " " + let.name + " = " + let.expr + " -> ";
			line+= frame.bLive && letIndex < m_shownFace.outputs.letValues.size()
					   ? formatValue(m_shownFace.outputs.letValues[letIndex])
					   : std::string("none");
			if (const FaceExprCompileError* error= errorFor(true, (int)letIndex))
				line+= " error: " + error->message;
			outLines.push_back(line);
		}
		for (size_t outputIndex= 0; outputIndex < profile.outputs.size(); ++outputIndex)
		{
			const FaceProfileOutput& output= profile.outputs[outputIndex];
			std::string line= "out " + std::to_string(outputIndex) + " " + FaceTabWidgets::targetLabel(output) + " = " +
							  output.vrmExpressionFunction + " -> ";
			line+= frame.bLive && outputIndex < m_shownFace.outputs.outputValues.size()
					   ? formatValue(m_shownFace.outputs.outputValues[outputIndex])
					   : std::string("none");
			if (const FaceExprCompileError* error= errorFor(false, (int)outputIndex))
				line+= " error: " + error->message;
			outLines.push_back(line);
		}
		return true;
	}
	if (verb == "edit")
	{
		int index= 0;
		if (args.size() != 3 || (args[1] != "let" && args[1] != "out") || !parseIndexToken(args[2], index))
		{
			outError= "usage: face edit let|out <index>";
			return false;
		}
		if (m_app->isActiveFaceProfileAuto())
		{
			outError= "the active profile is Auto, which is read-only";
			return false;
		}
		FaceExprEditTarget target;
		target.bLet= args[1] == "let";
		target.index= index;
		const size_t count= target.bLet ? profile.lets.size() : profile.outputs.size();
		if ((size_t)index >= count)
		{
			outError= "no such row";
			return false;
		}
		open(target);
		return true;
	}

	// The rest act on the open editor
	if (!m_bOpen)
	{
		outError= "the expression editor is not open";
		return false;
	}
	if (verb == "expr")
	{
		if (args.size() == 1)
		{
			outLines.push_back(getStatus().text);
			return true;
		}
		setText(AutomationProtocol::remainderAfterTokens(m_app->getAutomationServer()->getCurrentCommandLine(), 2));
		return true;
	}
	if (verb == "value")
	{
		const Status status= getStatus();
		outLines.push_back(std::string("row ") + (status.target.bLet ? "let " : "out ") +
						   std::to_string(status.target.index) + " " + status.rowLabel);
		outLines.push_back("value " + (status.bLive ? formatValue(status.value) : std::string("none")));
		if (!status.error.empty())
			outLines.push_back("error " + status.error);
		std::string reads= "reads";
		for (const std::string& name : status.reads)
			reads+= " " + name;
		outLines.push_back(reads);
		return true;
	}
	if (verb == "solo")
	{
		if (args.size() != 2 || (args[1] != "on" && args[1] != "off"))
		{
			outError= "usage: face solo on|off";
			return false;
		}
		setSolo(args[1] == "on");
		return true;
	}
	if (verb == "revert")
	{
		revert();
		return true;
	}
	if (verb == "close")
	{
		close();
		return true;
	}
	outError= "unknown verb '" + verb + "'";
	return false;
}
