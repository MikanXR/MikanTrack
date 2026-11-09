#include "FaceExprEditorText.h"

#include <algorithm>
#include <utility>

namespace
{
// The selection clamped to the text, begin first
void clampSelection(const std::string& text, int& selBegin, int& selEnd)
{
	const int length= (int)text.size();
	selBegin= std::clamp(selBegin, 0, length);
	selEnd= std::clamp(selEnd, 0, length);
	if (selEnd < selBegin)
		std::swap(selBegin, selEnd);
}

// The line holding offset and where that line starts
void lineOf(const std::string& text, int offset, int& outLine, int& outLineStart)
{
	outLine= 0;
	outLineStart= 0;
	for (int index= 0; index < offset; ++index)
	{
		if (text[(size_t)index] == '\n')
		{
			++outLine;
			outLineStart= index + 1;
		}
	}
}
} // namespace

FaceExprTextEdit insertFaceExprFunction(const std::string& text, int selBegin, int selEnd,
										const FaceExprFunctionInfo& function)
{
	clampSelection(text, selBegin, selEnd);
	const bool bWrap= selEnd > selBegin;

	std::vector<std::string> args;
	if (bWrap)
		args.push_back(text.substr((size_t)selBegin, (size_t)(selEnd - selBegin)));
	for (size_t placeholderIndex= bWrap ? 1 : 0; placeholderIndex < function.placeholders.size(); ++placeholderIndex)
		args.push_back(function.placeholders[placeholderIndex]);

	// The argument the selection lands on: the first placeholder inserted,
	// which follows the wrapped selection when there is one
	const size_t selectedArg= bWrap ? 1 : 0;
	std::string call= std::string(function.name) + "(";
	int selectedBegin= -1;
	int selectedEnd= -1;
	for (size_t argIndex= 0; argIndex < args.size(); ++argIndex)
	{
		if (argIndex > 0)
			call+= ", ";
		if (argIndex == selectedArg)
			selectedBegin= selBegin + (int)call.size();
		call+= args[argIndex];
		if (argIndex == selectedArg)
			selectedEnd= selBegin + (int)call.size();
	}
	call+= ")";

	FaceExprTextEdit edit;
	edit.text= text.substr(0, (size_t)selBegin) + call + text.substr((size_t)selEnd);
	if (selectedBegin >= 0)
	{
		edit.selectionBegin= selectedBegin;
		edit.selectionEnd= selectedEnd;
	}
	else if (bWrap)
	{
		// Nothing left to fill in: the caret after the closing paren
		edit.selectionBegin= selBegin + (int)call.size();
		edit.selectionEnd= edit.selectionBegin;
	}
	else
	{
		// A function without placeholders: the caret between the parens
		edit.selectionBegin= selBegin + (int)call.size() - 1;
		edit.selectionEnd= edit.selectionBegin;
	}
	return edit;
}

FaceExprTextEdit insertFaceExprText(const std::string& text, int selBegin, int selEnd, const std::string& insert)
{
	clampSelection(text, selBegin, selEnd);
	FaceExprTextEdit edit;
	edit.text= text.substr(0, (size_t)selBegin) + insert + text.substr((size_t)selEnd);
	edit.selectionBegin= selBegin + (int)insert.size();
	edit.selectionEnd= edit.selectionBegin;
	return edit;
}

void faceExprSpanToLines(const std::string& text, const FaceExprSpan& span, std::vector<FaceExprLineRun>& outRuns)
{
	outRuns.clear();
	const int length= (int)text.size();
	const int begin= std::clamp(span.begin, 0, length);
	const int end= std::clamp(span.end, begin, length);

	int line= 0;
	int lineStart= 0;
	lineOf(text, begin, line, lineStart);

	// Each line's share, skipping the zero-width share a span starting on a
	// line break leaves
	int position= begin;
	while (position < end)
	{
		const size_t lineBreak= text.find('\n', (size_t)position);
		const int lineEnd= lineBreak == std::string::npos ? length : (int)lineBreak;
		const int runEnd= std::min(end, lineEnd);
		if (runEnd > position)
		{
			FaceExprLineRun run;
			run.line= line;
			run.columnBegin= position - lineStart;
			run.columnEnd= runEnd - lineStart;
			outRuns.push_back(run);
		}
		position= lineEnd + 1;
		lineStart= position;
		++line;
	}

	if (outRuns.empty())
	{
		// An empty span, one past the end, or only line breaks: a zero-width
		// run where it starts
		int startLine= 0;
		int startLineStart= 0;
		lineOf(text, begin, startLine, startLineStart);
		FaceExprLineRun run;
		run.line= startLine;
		run.columnBegin= begin - startLineStart;
		run.columnEnd= run.columnBegin;
		outRuns.push_back(run);
	}
}
