#pragma once

#include <memory>
#include <vector>

#include "FaceExprEvaluator.h"
#include "FaceProfileBinding.h"
#include "FaceSources.h"

// One evaluation site for a face profile binding: the evaluator and its
// state, over a stream of frames. The live stream owns one on the vision
// thread, so the stateful slots advance exactly once per output frame; the
// timeline's replay owns another.
class FaceProfileRunner
{
public:
	// A new binding keeps the state of the call sites it still has
	void setBinding(std::shared_ptr<const FaceProfileBinding> binding);
	const std::shared_ptr<const FaceProfileBinding>& getBinding() const { return m_binding; }

	// Every stateful call starts over from its input (a seek)
	void reset() { m_state.reset(); }

	// Evaluates the profile over ioSources' phone and viseme, at the frame's
	// own timestamp, and writes ioSources.outputs. Without a binding the
	// outputs are cleared.
	void evaluate(FaceSources& ioSources, double timestampMs);

	// The last evaluate's inputs and lets, for the inspector
	const FaceExprEvaluator& getEvaluator() const { return m_evaluator; }

private:
	std::shared_ptr<const FaceProfileBinding> m_binding;
	FaceExprEvaluator m_evaluator;
	FaceExprState m_state;
};
