#pragma once

#include "PipelineModel.h"

// The Inspector's sections under a stage's Status, in the template's order
enum class eStageSection
{
	Inputs,
	Settings,
	Diagnostics
};

// What a stage shows under the Inspector's template, supplied by the window
class IStageContent
{
public:
	virtual ~IStageContent()= default;
	virtual bool hasSection(const PipelineStageId& stage, eStageSection section) const= 0;
	virtual void drawSection(const PipelineStageId& stage, eStageSection section)= 0;
};

// The Inspector panel: the stage selected in the Pipeline panel on one
// template. With nothing selected it shows the project in brief and the
// stages that need attention, each a click away.
class StageInspector
{
public:
	// Returns the stage a click on a needs-attention row asks to select, an
	// invalid id otherwise. content: the selected stage's sections, null for
	// none.
	PipelineStageId draw(bool* pOpen, const PipelineStatusInputs& inputs, const PipelineStageId& selected,
						 IStageContent* content);

private:
	void drawStage(const PipelineStatusInputs& inputs, const PipelineStageId& stage, IStageContent* content);
	PipelineStageId drawProject(const PipelineStatusInputs& inputs);
};

// The Performance panel: the vision thread's last frame loop iteration by phase
class VisionThread;
namespace PerformancePanel
{
void draw(bool* pOpen, const VisionThread& visionThread);
}
