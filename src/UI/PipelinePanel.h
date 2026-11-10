#pragma once

#include "PipelineModel.h"

// The Pipeline panel: the project's stages as a strip of boxes, each with a
// state dot and one number, joined in the order a frame passes through them.
// A click selects a stage (a second click clears it), and a stage with
// sub-stages opens them in a second row. The panel draws what the model says
// and holds only the selection.
class PipelinePanel
{
public:
	void draw(bool* pOpen, const PipelineStatusInputs& inputs);

	// Invalid when nothing is selected
	const PipelineStageId& getSelected() const { return m_selected; }
	void setSelected(const PipelineStageId& stage) { m_selected= stage; }

private:
	PipelineStageId m_selected;
};
