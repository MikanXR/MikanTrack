#pragma once

#include <optional>

#include "AudioPanel.h"        // AudioPanel::DrawResult, MicrophoneReadout
#include "CalibrationPanel.h"  // CalibrationPanel::DrawResult
#include "FaceExprEditTarget.h"
#include "SettingsPanels.h"    // TrackingSectionContext, OscSectionContext
#include "StageInspector.h"    // IStageContent

class App;
class AvatarFaceTab;
class AvatarMouthReadout;
class AvatarPanel;
class DevicePanel;

// The sections each stage shows in the Inspector, drawn from the hostable
// sections of the device, calibration, tracking, OSC, audio, avatar and face
// profile controls. The window calls beginFrame before anything draws a stage
// and reads the requests the draws raised after the last one.
class StageContent : public IStageContent
{
public:
	StageContent(App* app, DevicePanel* devicePanel, CalibrationPanel* calibrationPanel, AudioPanel* audioPanel,
				 AvatarPanel* avatarPanel, AvatarFaceTab* faceTab, AvatarMouthReadout* mouthReadout);

	// What this frame's draws read, and clear requests. fusion, imu, face,
	// microphone, oscReadout and direct: the shown frame's readouts, read in
	// place when a section draws, so they must outlive the frame's draws and be
	// refilled for the shown frame before the Inspector draws. activeEdit: the
	// row the expression editor is on, which the face profile rows highlight
	// (null: none).
	void beginFrame(const TrackingSectionContext& tracking, const OscSectionContext& osc, const FusionReadout& fusion,
					const ImuReadout& imu, const FaceReadout& face, const MicrophoneReadout& microphone,
					const OscReadout& oscReadout, const DirectReadout& direct, const FaceExprEditTarget* activeEdit,
					bool bWizardActive);
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
	// The Retarget stage's settings: placement, mapping and retarget with a
	// loaded model, and the loss rules always
	void drawRetargetSettings();

	App* m_app;
	DevicePanel* m_devicePanel;
	CalibrationPanel* m_calibrationPanel;
	AudioPanel* m_audioPanel;
	AvatarPanel* m_avatarPanel;
	AvatarFaceTab* m_faceTab;
	AvatarMouthReadout* m_mouthReadout;

	TrackingSectionContext m_tracking;
	OscSectionContext m_osc;
	const MicrophoneReadout* m_microphone= nullptr;
	const FaceExprEditTarget* m_activeEdit= nullptr;
	bool m_bWizardActive= false;
	CalibrationPanel::DrawResult m_calibrationAction;
	AudioPanel::DrawResult m_audioAction;
	std::optional<FaceExprEditTarget> m_faceEditRequest;
};
