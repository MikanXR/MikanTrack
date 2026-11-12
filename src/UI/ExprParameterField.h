#pragma once

#include "ExprInputTable.h"
#include "ExprParameter.h"

// What the solve made of a parameter on the shown frame, for the field's bar
struct ExprParameterLiveValue
{
	// The shown frame evaluated the parameter
	bool bValid= false;
	// What the solve took, clamped to the parameter's range
	float value= 0.f;
	// The function's value before the clamp
	float rawValue= 0.f;
	// The function decided the value (in force and compiled); otherwise the
	// plain value did
	bool bFromExpression= false;
};

// An ExprParameter edited inline, in a row with an fx button that switches
// between its two forms:
// - the value: a slider over [minValue, maxValue]
// - the function: a single-line text input with a compile error underlined
//   at its span and spelled out below it, and under it a thin bar of the
//   live value on the shown frame within the range, the number beside it. A
//   text that does not compile draws the plain value it falls back on,
//   marked as the fallback.
// Switching to the value keeps the text. A help marker lists the table's
// names. The text compiles here against table, so an error shows at once,
// while the live value comes from the solve.
//
// True when the parameter changed and is to be committed: a slider move, a
// switch of form, or a text committed with Enter or by leaving the input.
// Typing alone commits nothing.
//
// Safe to draw twice in a frame. The text being typed lives in one buffer
// keyed by the input's ImGui id, which differs per window, and ImGui has one
// active item at a time: the copy being typed in shows the buffer, every
// other copy the stored text.
bool drawExprParameterField(const char* id, const char* label, ExprParameter& ioParameter, float minValue,
							float maxValue, const char* valueFormat, const char* tooltip, const ExprInputTable& table,
							const ExprParameterLiveValue& live);
