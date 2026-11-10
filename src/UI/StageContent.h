#pragma once

#include "CalibrationPanel.h" // CalibrationPanel::DrawResult
#include "SettingsPanels.h"   // TrackingSectionContext
#include "StageInspector.h"   // IStageContent

class DevicePanel;

// The sections each stage shows in the Inspector, drawn from the hostable
// sections of the device, calibration and tracking controls. The window calls
// beginFrame before anything draws a stage and reads the wizard launches the
// draws raised after the last one.
class StageContent : public IStageContent
{
public:
	StageContent(DevicePanel* devicePanel, CalibrationPanel* calibrationPanel);

	// What this frame's draws read, and a clear launch request
	void beginFrame(const TrackingSectionContext& tracking, bool bWizardActive);
	// The launches raised by every section drawn since beginFrame
	const CalibrationPanel::DrawResult& getCalibrationAction() const { return m_calibrationAction; }

	bool hasSection(const PipelineStageId& stage, eStageSection section) const override;
	void drawSection(const PipelineStageId& stage, eStageSection section) override;

private:
	// A per-camera stage's camera exists in the config
	bool isCameraInRange(const PipelineStageId& stage) const;
	void mergeCalibrationAction(const CalibrationPanel::DrawResult& result);

	DevicePanel* m_devicePanel;
	CalibrationPanel* m_calibrationPanel;

	TrackingSectionContext m_tracking;
	bool m_bWizardActive= false;
	CalibrationPanel::DrawResult m_calibrationAction;
};
