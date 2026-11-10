#include "ExprParameter.h"

#include <algorithm>
#include <cmath>

#include "nlohmann/json.hpp"

#include "FaceProfile.h"

using json= nlohmann::json;

namespace
{
float clampToRange(float value, float minValue, float maxValue)
{
	// A reversed range clamps to its lower bound rather than asserting
	return std::max(minValue, std::min(value, maxValue));
}
} // namespace

json exprParameterToJson(const ExprParameter& parameter)
{
	if (!parameter.bUseExpression && parameter.expression.empty())
		return parameter.value;
	return {{"value", parameter.value}, {"expression", parameter.expression}, {"useExpression", parameter.bUseExpression}};
}

ExprParameter exprParameterFromJson(const json& j, float defaultValue)
{
	ExprParameter parameter;
	parameter.value= defaultValue;
	if (j.is_number())
	{
		parameter.value= j.get<float>();
		return parameter;
	}
	if (!j.is_object())
		return parameter;

	const auto value= j.find("value");
	if (value != j.end() && value->is_number())
		parameter.value= value->get<float>();
	const auto expression= j.find("expression");
	if (expression != j.end() && expression->is_string())
		parameter.expression= expression->get<std::string>();
	const auto useExpression= j.find("useExpression");
	if (useExpression != j.end() && useExpression->is_boolean())
		parameter.bUseExpression= useExpression->get<bool>();
	return parameter;
}

void ExprParameterRunner::setParameter(const ExprParameter& parameter, const ExprInputTable& table)
{
	m_parameter= parameter;
	if (m_compiledTable == &table && m_compiledText == parameter.expression)
		return;

	// One expression is a profile with one output, so the parameter shares
	// the profile compiler and its state slot keys
	FaceProfile profile;
	FaceProfileOutput output;
	output.target= "parameter";
	output.vrmExpressionFunction= parameter.expression;
	profile.outputs.push_back(output);
	FaceExprCompileResult result= compileFaceProfile(profile, table);

	m_compiledTable= &table;
	m_compiledText= parameter.expression;
	m_errors= std::move(result.errors);
	m_evaluator.setProgram(std::move(result.program));
}

float ExprParameterRunner::evaluate(const float* inputs, int inputCount, double timeSeconds, float minValue,
									float maxValue)
{
	m_bLastUsedExpression= false;
	if (m_parameter.bUseExpression && m_evaluator.getProgram() != nullptr)
	{
		float rawValue= 0.f;
		if (m_evaluator.evaluate(inputs, inputCount, timeSeconds, m_state, &rawValue))
		{
			m_lastRawValue= rawValue;
			if (std::isfinite(rawValue))
			{
				m_bLastUsedExpression= true;
				return clampToRange(rawValue, minValue, maxValue);
			}
		}
	}
	return clampToRange(m_parameter.value, minValue, maxValue);
}

const FaceExprCompileError* ExprParameterRunner::getCompileError() const
{
	return m_errors.empty() ? nullptr : &m_errors.front();
}
