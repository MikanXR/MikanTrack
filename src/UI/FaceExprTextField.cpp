#include "FaceExprTextField.h"

#include <algorithm>
#include <vector>

#include "imgui.h"
#include "imgui_internal.h" // the field's child window and its scroll, for the underline
#include "misc/cpp/imgui_stdlib.h"

#include "FaceExprCompiler.h"
#include "FaceTabWidgets.h"

namespace
{
constexpr int k_minLines= 2;
constexpr int k_maxLines= 10;
} // namespace

void FaceExprTextField::requestEdit(const FaceExprTextEdit& edit)
{
	m_pending= edit;
	m_bPendingText= true;
	m_bPendingSelection= true;
	m_bFocusRequested= true;
}

void FaceExprTextField::getSelection(const std::string& text, int& outBegin, int& outEnd) const
{
	const int size= (int)text.size();
	outBegin= m_selectBegin < 0 ? size : std::clamp(m_selectBegin, 0, size);
	outEnd= m_selectEnd < 0 ? size : std::clamp(m_selectEnd, outBegin, size);
}

void FaceExprTextField::resetCaret()
{
	m_selectBegin= -1;
	m_selectEnd= -1;
}

bool FaceExprTextField::draw(const char* id, std::string& text, const FaceExprCompileError* error, ImFont* font)
{
	bool bChanged= false;
	// Inactive: ImGui reloads the string when the field activates, so the
	// text goes in directly and only the selection waits for the callback
	if (m_bPendingText && !m_bActive)
	{
		bChanged= text != m_pending.text;
		text= m_pending.text;
		m_bPendingText= false;
		m_selectBegin= m_pending.selectionBegin;
		m_selectEnd= m_pending.selectionEnd;
	}

	if (font != nullptr)
		ImGui::PushFont(font, 0.f);

	const int lineCount= (int)std::count(text.begin(), text.end(), '\n') + 1;
	const int shownLines= std::clamp(lineCount, k_minLines, k_maxLines);
	const float height= (float)shownLines * ImGui::GetTextLineHeight() + ImGui::GetStyle().FramePadding.y * 2.f;

	if (m_bFocusRequested)
	{
		// Activates the field on the next frame; the callback runs that frame
		ImGui::SetKeyboardFocusHere();
		m_bFocusRequested= false;
	}
	const ImGuiID fieldId= ImGui::GetID(id);
	bChanged|= ImGui::InputTextMultiline(id, &text, ImVec2(-FLT_MIN, height), ImGuiInputTextFlags_CallbackAlways,
										 &FaceExprTextField::onCallback, this);
	m_bActive= ImGui::IsItemActive();
	if (error != nullptr)
		drawUnderline(id, fieldId, text, *error);

	if (font != nullptr)
		ImGui::PopFont();
	return bChanged;
}

int FaceExprTextField::onCallback(ImGuiInputTextCallbackData* data)
{
	FaceExprTextField& self= *static_cast<FaceExprTextField*>(data->UserData);
	if (self.m_bPendingText)
	{
		// Active while the edit was queued: replace through the buffer, which
		// also lands in the undo stack
		data->DeleteChars(0, data->BufTextLen);
		data->InsertChars(0, self.m_pending.text.c_str());
		self.m_bPendingText= false;
	}
	if (self.m_bPendingSelection)
	{
		// A re-activation resets the caret unless the text is byte-identical
		// to the state's copy, so the selection is set here, not assumed
		const int size= data->BufTextLen;
		const int begin= self.m_pending.selectionBegin < 0 ? size : std::clamp(self.m_pending.selectionBegin, 0, size);
		const int end= self.m_pending.selectionEnd < 0 ? size : std::clamp(self.m_pending.selectionEnd, begin, size);
		data->SelectionStart= begin;
		data->SelectionEnd= end;
		data->CursorPos= end;
		self.m_bPendingSelection= false;
	}
	self.m_selectBegin= std::min(data->SelectionStart, data->SelectionEnd);
	self.m_selectEnd= std::max(data->SelectionStart, data->SelectionEnd);
	return 0;
}

void FaceExprTextField::drawUnderline(const char* id, ImGuiID fieldId, const std::string& text,
									  const FaceExprCompileError& error)
{
	// The multiline is a child window painting an opaque frame over the
	// parent's draw list, so the line goes into the child's own list
	ImGuiWindow* parent= ImGui::GetCurrentWindow();
	char childName[256];
	ImFormatString(childName, sizeof(childName), "%s/%s_%08X", parent->Name, id, fieldId);
	ImGuiWindow* child= ImGui::FindWindowByName(childName);
	if (child == nullptr)
		return;
	// ImGui scrolls the text horizontally only while it renders the cursor
	const ImGuiInputTextState* state= ImGui::GetInputTextState(fieldId);
	const float scrollX= m_bActive && state != nullptr ? state->Scroll.x : 0.f;
	const ImVec2 rectMin= ImGui::GetItemRectMin();
	const ImVec2 padding= ImGui::GetStyle().FramePadding;
	const float originX= rectMin.x + padding.x - scrollX;
	const float originY= rectMin.y + padding.y - child->Scroll.y;
	const float lineHeight= ImGui::GetTextLineHeight();

	std::vector<FaceExprLineRun> runs;
	faceExprSpanToLines(text, error.span, runs);

	// Line start offsets, for measuring a run against its line's prefix
	std::vector<int> lineStarts{0};
	for (int offset= 0; offset < (int)text.size(); ++offset)
	{
		if (text[(size_t)offset] == '\n')
			lineStarts.push_back(offset + 1);
	}

	const ImU32 color= ImGui::GetColorU32(FaceTabWidgets::k_errorColor);
	ImDrawList* drawList= child->DrawList;
	drawList->PushClipRect(child->InnerClipRect.Min, child->InnerClipRect.Max, false);
	for (const FaceExprLineRun& run : runs)
	{
		if (run.line < 0 || run.line >= (int)lineStarts.size())
			continue;
		const char* lineBegin= text.c_str() + lineStarts[(size_t)run.line];
		const int lineLength= (int)text.size() - lineStarts[(size_t)run.line];
		const int columnBegin= std::clamp(run.columnBegin, 0, lineLength);
		const int columnEnd= std::clamp(run.columnEnd, columnBegin, lineLength);
		const float x0= originX + ImGui::CalcTextSize(lineBegin, lineBegin + columnBegin).x;
		float x1= originX + ImGui::CalcTextSize(lineBegin, lineBegin + columnEnd).x;
		if (x1 <= x0)
			x1= x0 + ImGui::CalcTextSize(" ").x;
		const float y= originY + (float)(run.line + 1) * lineHeight - 1.f;
		drawList->AddLine(ImVec2(x0, y), ImVec2(x1, y), color, 2.f);
	}
	drawList->PopClipRect();
}
