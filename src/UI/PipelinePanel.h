#pragma once

#include "PipelineModel.h"

// The Pipeline panel: the project's stages as a strip of boxes, each with a
// state dot and one number, joined in the order a frame passes through them.
// A click selects a stage (a second click clears it), a double-click selects
// it and asks for it to be pinned in its own window, and a stage with
// sub-stages opens them in a second row. The panel draws what the model says
// and holds only the selection and the pin request.
class PipelinePanel
{
public:
	void draw(bool* pOpen, const PipelineStatusInputs& inputs);

	// Invalid when nothing is selected
	const PipelineStageId& getSelected() const { return m_selected; }
	void setSelected(const PipelineStageId& stage) { m_selected= stage; }

	// The stage a double-click asked to pin since the last call, invalid for
	// none; cleared by the call
	PipelineStageId consumePinRequest()
	{
		const PipelineStageId request= m_pinRequest;
		m_pinRequest= PipelineStageId();
		return request;
	}

private:
	PipelineStageId m_selected;
	PipelineStageId m_pinRequest;
};
