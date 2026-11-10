#include "StageContent.h"

#include "imgui.h"

#include "AppConfig.h"
#include "DevicePanel.h"

StageContent::StageContent(DevicePanel* devicePanel, CalibrationPanel* calibrationPanel)
	: m_devicePanel(devicePanel)
	, m_calibrationPanel(calibrationPanel)
{
}

void StageContent::beginFrame(const TrackingSectionContext& tracking, const FusionReadout& fusion, bool bWizardActive)
{
	m_tracking= tracking;
	m_tracking.fusion= &fusion;
	m_bWizardActive= bWizardActive;
	m_calibrationAction= CalibrationPanel::DrawResult();
}

bool StageContent::isCameraInRange(const PipelineStageId& stage) const
{
	return m_tracking.config != nullptr && stage.cameraIndex >= 0 &&
		   stage.cameraIndex < (int)m_tracking.config->cameraCount();
}

void StageContent::mergeCalibrationAction(const CalibrationPanel::DrawResult& result)
{
	if (result.bLaunchIntrinsicsWizard)
	{
		m_calibrationAction.bLaunchIntrinsicsWizard= true;
		m_calibrationAction.cameraIndex= result.cameraIndex;
	}
	if (result.bLaunchExtrinsicsWizard)
		m_calibrationAction.bLaunchExtrinsicsWizard= true;
}

bool StageContent::hasSection(const PipelineStageId& stage, eStageSection section) const
{
	if (m_tracking.config == nullptr)
		return false;

	switch (stage.stage)
	{
	case ePipelineStage::Capture:
		return true;
	case ePipelineStage::Camera:
	case ePipelineStage::Lift:
		return section == eStageSection::Inputs && isCameraInRange(stage);
	case ePipelineStage::Video:
		return section == eStageSection::Settings && isCameraInRange(stage) &&
			   m_devicePanel->hasCameraSettings(stage.cameraIndex);
	case ePipelineStage::HandModels:
	case ePipelineStage::BodyModel:
		return section == eStageSection::Settings && isCameraInRange(stage);
	case ePipelineStage::Fusion:
		// The per-camera skeletons toggle is the only setting, and it needs
		// a second camera
		return section != eStageSection::Settings || m_tracking.config->cameraCount() > 1;
	default:
		return false;
	}
}

void StageContent::drawSection(const PipelineStageId& stage, eStageSection section)
{
	if (!hasSection(stage, section))
		return;

	const int cameraIndex= stage.cameraIndex;
	switch (stage.stage)
	{
	case ePipelineStage::Capture:
		if (section == eStageSection::Inputs)
		{
			m_devicePanel->drawRescanControl();
			m_devicePanel->drawAddCameraControl();
			mergeCalibrationAction(m_calibrationPanel->drawExtrinsicsSection(m_bWizardActive));
		}
		else if (section == eStageSection::Settings)
		{
			SettingsPanels::drawFlipHandednessSection(m_tracking);
		}
		else
		{
			// Three readouts share the section, so each keeps its heading
			SettingsPanels::drawInferenceSection(m_tracking);
			SettingsPanels::drawImageQualitySection(m_tracking);
			SettingsPanels::drawCaptureTimingSection(m_tracking);
		}
		break;
	case ePipelineStage::Camera:
		// The Inspector's title names the camera, and its Camera Settings
		// are the Video stage's
		m_devicePanel->drawCameraSection(cameraIndex, false, false);
		break;
	case ePipelineStage::Video:
		// The tree is the section's only content, so it starts open
		ImGui::PushID(cameraIndex);
		m_devicePanel->drawCameraSettings(cameraIndex, true);
		ImGui::PopID();
		break;
	case ePipelineStage::HandModels:
		SettingsPanels::drawHandOverlayControls(m_tracking);
		break;
	case ePipelineStage::BodyModel:
		SettingsPanels::drawBodyPoseIntro();
		SettingsPanels::drawBodyPoseCameraControls(m_tracking, cameraIndex);
		SettingsPanels::drawBodyOverlayControls(m_tracking);
		break;
	case ePipelineStage::Lift:
		mergeCalibrationAction(m_calibrationPanel->drawCameraCalibration(cameraIndex, m_bWizardActive, false));
		break;
	case ePipelineStage::Fusion:
		if (section == eStageSection::Inputs)
		{
			SettingsPanels::drawHandCalibrationSection(m_tracking);
			m_calibrationPanel->drawHandScaleSection();
		}
		else if (section == eStageSection::Settings)
		{
			SettingsPanels::drawPerCameraSkeletonsControl(m_tracking);
		}
		else
		{
			// Five readouts share the section, so each keeps its heading
			SettingsPanels::drawFusionSection(m_tracking);
			SettingsPanels::drawHandScaleSection(m_tracking);
			SettingsPanels::drawFusedPalmReadout(m_tracking);
			SettingsPanels::drawFusionDiagnosticsSection(m_tracking);
			SettingsPanels::drawHoldStillSection(m_tracking);
		}
		break;
	default:
		break;
	}
}
