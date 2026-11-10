#pragma once

#include "PipelineModel.h"

// The Inspector panel: the stage selected in the Pipeline panel on one
// template. With nothing selected it shows the project in brief and the
// stages that need attention, each a click away.
class StageInspector
{
public:
	// Returns the stage a click on a needs-attention row asks to select, an
	// invalid id otherwise
	PipelineStageId draw(bool* pOpen, const PipelineStatusInputs& inputs, const PipelineStageId& selected);

private:
	void drawStage(const PipelineStatusInputs& inputs, const PipelineStageId& stage);
	PipelineStageId drawProject(const PipelineStatusInputs& inputs);
};

// The Performance panel: the vision thread's last frame loop iteration by phase
class VisionThread;
namespace PerformancePanel
{
void draw(bool* pOpen, const VisionThread& visionThread);
}
