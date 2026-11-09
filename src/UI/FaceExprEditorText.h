#pragma once

#include <string>
#include <vector>

#include "FaceExprFunctions.h"
#include "FaceExprLexer.h"

// The expression editor's text operations, free of any UI. Offsets are byte
// offsets into the text, clamped to it.

// A text with the selection to show after an edit; begin == end is a caret
struct FaceExprTextEdit
{
	std::string text;
	int selectionBegin= 0;
	int selectionEnd= 0;
};

// Inserts a call to function at the selection [selBegin, selEnd) of text. A
// non-empty selection becomes the first argument and the second placeholder
// is selected; an empty selection inserts every placeholder with the first
// selected. A wrapped selection with a single-placeholder function leaves the
// caret after the closing paren.
FaceExprTextEdit insertFaceExprFunction(const std::string& text, int selBegin, int selEnd,
										const FaceExprFunctionInfo& function);
// Replaces the selection with insert, caret after it
FaceExprTextEdit insertFaceExprText(const std::string& text, int selBegin, int selEnd, const std::string& insert);

// One line's share of a span, byte columns within that line
struct FaceExprLineRun
{
	int line= 0;
	int columnBegin= 0;
	int columnEnd= 0;
};

// Splits span over the lines of text (split at '\n'). A span that is empty
// or starts at or past the end of text yields one zero-width run at that
// position (the caller widens it), so an error at the end of the text is
// still placed.
void faceExprSpanToLines(const std::string& text, const FaceExprSpan& span, std::vector<FaceExprLineRun>& outRuns);
