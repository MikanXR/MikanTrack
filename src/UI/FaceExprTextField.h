#pragma once

#include <string>

#include "imgui.h"

#include "FaceExprEditorText.h"

struct FaceExprCompileError;
struct ImGuiInputTextCallbackData;

// The expression editor's formula field: a multi-line text area in the
// monospace font that grows with its line count, mirrors its caret and
// selection every frame it is active, takes edits queued from outside (the
// palette, the inputs list, Revert, the automation channel), and underlines
// a compile error at its span.
//
// Queued edits replace the whole text and set the selection. The field is
// inactive when a button outside it is released (the press took the focus),
// so the text is written straight into the string and the selection is
// applied in the callback when the requested focus activates the field. An
// edit queued while the field is active goes through the callback alone,
// since ImGui writes its own copy back over the string every active frame.
class FaceExprTextField
{
public:
	// Draws the field in font (null: the current font), with error's span
	// underlined when given. True when text changed this frame, by typing or
	// by a queued edit.
	bool draw(const char* id, std::string& text, const FaceExprCompileError* error, ImFont* font);

	void requestEdit(const FaceExprTextEdit& edit);
	// The selection to insert at, clamped to text; the text's end when the
	// field has not been active since the last reset
	void getSelection(const std::string& text, int& outBegin, int& outEnd) const;
	// Forgets the caret, so the next insert goes to the end of the text
	void resetCaret();
	bool isActive() const { return m_bActive; }

private:
	static int onCallback(ImGuiInputTextCallbackData* data);
	void drawUnderline(const char* id, ImGuiID fieldId, const std::string& text, const FaceExprCompileError& error);

	// The edit waiting for the callback: the whole text when it still has
	// to be written, else the selection alone
	FaceExprTextEdit m_pending;
	bool m_bPendingText= false;
	bool m_bPendingSelection= false;
	bool m_bFocusRequested= false;
	bool m_bActive= false;
	// Byte offsets, -1 for the end of the text
	int m_selectBegin= -1;
	int m_selectEnd= -1;
};
