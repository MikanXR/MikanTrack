#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nlohmann/json_fwd.hpp"

#include "ExprInputTable.h"
#include "FaceExprCompiler.h"
#include "FaceExprEvaluator.h"

// A numeric setting that is either a plain value or a function written in the
// expression language. Switching to the value keeps the text, so switching
// back restores the function.
struct ExprParameter
{
	float value= 0.f;
	std::string expression;
	bool bUseExpression= false;
};

// JSON: a setting with no text and the value in force is a bare number, so a
// plain numeric setting reads and writes as it always has. Otherwise it is an
// object: {"value": 0.5, "expression": "...", "useExpression": true}.
nlohmann::json exprParameterToJson(const ExprParameter& parameter);
// A bare number is a plain value. An object takes its fields, each falling
// back on its own: a missing or non-numeric value to defaultValue, a missing
// expression to "", a missing flag to false. Anything else is defaultValue as
// a plain value.
ExprParameter exprParameterFromJson(const nlohmann::json& j, float defaultValue);

// One evaluation site for an ExprParameter: the compiled text, the evaluator
// and its state, over a stream of frames.
//
// The rules:
// - setParameter compiles only when the text or the table changed. The
//   expression's stateful calls keep their state across a setParameter that
//   keeps the text.
// - evaluate returns the expression's value clamped to [minValue, maxValue]
//   when the expression is in force, compiled, and finite. Otherwise it
//   returns the plain value clamped the same way.
class ExprParameterRunner
{
public:
	// table must outlive the runner or the next setParameter
	void setParameter(const ExprParameter& parameter, const ExprInputTable& table);
	const ExprParameter& getParameter() const { return m_parameter; }

	// Every stateful call starts over from its input (a seek)
	void reset() { m_state.reset(); }

	// inputs holds one value per slot of the table, inputCount of them. A
	// count that differs from the table's falls back on the plain value.
	float evaluate(const float* inputs, int inputCount, double timeSeconds, float minValue, float maxValue);

	// The text's compile error, whichever form is in force; null when it
	// compiled. One expression has at most one error.
	const FaceExprCompileError* getCompileError() const;
	// The text's program, null after a compile error
	const std::shared_ptr<const FaceExprProgram>& getProgram() const { return m_evaluator.getProgram(); }
	// The expression's unclamped value at the last evaluate that ran it, 0
	// before one has
	float getLastRawValue() const { return m_lastRawValue; }
	// Whether the last evaluate's value came from the expression
	bool getLastUsedExpression() const { return m_bLastUsedExpression; }

private:
	ExprParameter m_parameter;
	const ExprInputTable* m_compiledTable= nullptr;
	std::string m_compiledText;
	std::vector<FaceExprCompileError> m_errors;
	FaceExprEvaluator m_evaluator;
	FaceExprState m_state;
	float m_lastRawValue= 0.f;
	bool m_bLastUsedExpression= false;
};
