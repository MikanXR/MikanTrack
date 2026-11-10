#pragma once

#include <optional>

#include "AudioPanel.h"        // AudioPanel::DrawResult, MicrophoneReadout
#include "CalibrationPanel.h"  // CalibrationPanel::DrawResult
#include "FaceExprEditTarget.h"
#include "SettingsPanels.h"    // TrackingSectionContext
#include "StageInspector.h"    // IStageContent

class App;
class AvatarFaceTab;
class AvatarMouthReadout;
class DevicePanel;

// The sections each stage shows in the Inspector, drawn from the hostable
// sections of the device, calibration, tracking, audio and face profile
// controls. The window calls beginFrame before anything draws a stage and
// reads the requests the draws raised after the last one.
class StageContent : public IStageContent
{
public:
	StageContent(App* app, DevicePanel* devicePanel, CalibrationPanel* calibrationPanel, AudioPanel* audioPanel,
				 AvatarFaceTab* faceTab, AvatarMouthReadout* mouthReadout);

	// What this frame's draws read, and clear requests. fusion, imu, face and
	// microphone: the shown frame's readouts, read in place when a section
	// draws, so they must outlive the frame's draws. activeEdit: the row the
	// expression editor is on, which the face profile rows highlight (null:
	// none).
	void beginFrame(const TrackingSectionContext& tracking, const FusionReadout& fusion, const ImuReadout& imu,
					const FaceReadout& face, const MicrophoneReadout& microphone,
					const FaceExprEditTarget* activeEdit, bool bWizardActive);
	// The requests raised by every section drawn since beginFrame: the
	// calibration wizards, the audio calibration wizard, and the face profile
	// row an Edit button asked the expression editor to open
	const CalibrationPanel::DrawResult& getCalibrationAction() const { return m_calibrationAction; }
	const AudioPanel::DrawResult& getAudioAction() const { return m_audioAction; }
	const std::optional<FaceExprEditTarget>& getFaceEditRequest() const { return m_faceEditRequest; }

	bool hasSection(const PipelineStageId& stage, eStageSection section) const override;
	void drawSection(const PipelineStageId& stage, eStageSection section) override;

private:
	// A per-camera stage's camera exists in the config
	bool isCameraInRange(const PipelineStageId& stage) const;
	void mergeCalibrationAction(const CalibrationPanel::DrawResult& result);

	App* m_app;
	DevicePanel* m_devicePanel;
	CalibrationPanel* m_calibrationPanel;
	AudioPanel* m_audioPanel;
	AvatarFaceTab* m_faceTab;
	AvatarMouthReadout* m_mouthReadout;

	TrackingSectionContext m_tracking;
	const MicrophoneReadout* m_microphone= nullptr;
	const FaceExprEditTarget* m_activeEdit= nullptr;
	bool m_bWizardActive= false;
	CalibrationPanel::DrawResult m_calibrationAction;
	AudioPanel::DrawResult m_audioAction;
	std::optional<FaceExprEditTarget> m_faceEditRequest;
};
