#include "FaceProfileRunner.h"

#include <algorithm>

void FaceProfileRunner::setBinding(std::shared_ptr<const FaceProfileBinding> binding)
{
	m_binding= std::move(binding);
	m_evaluator.setProgram(m_binding != nullptr ? m_binding->getProgram() : nullptr);
}

void FaceProfileRunner::evaluate(FaceSources& ioSources, double timestampMs)
{
	FaceSources::Outputs& outputs= ioSources.outputs;
	if (m_binding == nullptr || m_binding->getProgram() == nullptr)
	{
		outputs= FaceSources::Outputs();
		return;
	}

	outputs.generation= m_binding->getGeneration();
	outputs.outputValues.resize(m_binding->getProgram()->outputs.size());
	m_evaluator.evaluate(ioSources, timestampMs / 1000.0, m_state, outputs.outputValues.data());
	for (float& value : outputs.outputValues)
		value= std::clamp(value, 0.f, 1.f);
	m_binding->combineTargets(outputs.outputValues, outputs.targetValues);
	outputs.letValues.resize(m_binding->getProgram()->lets.size());
	for (size_t letIndex= 0; letIndex < outputs.letValues.size(); ++letIndex)
		outputs.letValues[letIndex]= m_evaluator.getLetValue((int)letIndex);
}
