#include "StageContent.h"

#include "imgui.h"

#include "App.h"
#include "AppConfig.h"
#include "AvatarFaceTab.h"
#include "AvatarMouthReadout.h"
#include "AvatarPanel.h"
#include "DevicePanel.h"
#include "LocText.h"

StageContent::StageContent(App* app, DevicePanel* devicePanel, CalibrationPanel* calibrationPanel,
						   AudioPanel* audioPanel, AvatarPanel* avatarPanel, AvatarFaceTab* faceTab,
						   AvatarMouthReadout* mouthReadout)
	: m_app(app)
	, m_devicePanel(devicePanel)
	, m_calibrationPanel(calibrationPanel)
	, m_audioPanel(audioPanel)
	, m_avatarPanel(avatarPanel)
	, m_faceTab(faceTab)
	, m_mouthReadout(mouthReadout)
{
}

void StageContent::beginFrame(const TrackingSectionContext& tracking, const OscSectionContext& osc,
							  const FusionReadout& fusion, const ImuReadout& imu, const FaceReadout& face,
							  const MicrophoneReadout& microphone, const OscReadout& oscReadout,
							  const DirectReadout& direct, const SourcePoseReadout& sourcePose,
							  const FaceExprEditTarget* activeEdit, bool bWizardActive)
{
	m_tracking= tracking;
	m_osc= osc;
	m_osc.readout= &oscReadout;
	m_tracking.fusion= &fusion;
	m_tracking.imu= &imu;
	m_tracking.face= &face;
	m_tracking.direct= &direct;
	m_tracking.sourcePose= &sourcePose;
	m_microphone= &microphone;
	m_activeEdit= activeEdit;
	m_bWizardActive= bWizardActive;
	m_calibrationAction= CalibrationPanel::DrawResult();
	m_audioAction= AudioPanel::DrawResult();
	m_faceEditRequest.reset();
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
	case ePipelineStage::Imu:
	case ePipelineStage::BodySolver:
	case ePipelineStage::Microphone:
	case ePipelineStage::DirectSolver:
		return true;
	case ePipelineStage::Filters:
	case ePipelineStage::Torso:
	case ePipelineStage::ArmChain:
	case ePipelineStage::Palm:
		return section == eStageSection::Diagnostics;
	case ePipelineStage::HoldRest:
		return section == eStageSection::Diagnostics;
	case ePipelineStage::Phone:
		// The stream status shows only while the shown frame's stream is
		// enabled, or to say the take did not record it
		return section == eStageSection::Settings ||
			   (section == eStageSection::Diagnostics &&
				(!m_tracking.face->bAvailable || m_tracking.face->bEnabled));
	case ePipelineStage::Profile:
		// The profiles belong to the loaded avatar
		return section == eStageSection::Diagnostics || m_app->getAvatarModel() != nullptr;
	case ePipelineStage::Retarget:
		// The arms' loss rules apply with or without an avatar, so the
		// settings always show, and the chains' states read the shown frame
		return true;
	case ePipelineStage::Output:
	case ePipelineStage::DeliveryRules:
		return section != eStageSection::Inputs;
	case ePipelineStage::VmcStream:
		return section == eStageSection::Settings;
	case ePipelineStage::SourcePose:
		return section == eStageSection::Diagnostics;
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
	case ePipelineStage::Imu:
		if (section == eStageSection::Inputs)
		{
			SettingsPanels::drawMountingCalibrationControl(m_tracking);
		}
		else if (section == eStageSection::Settings)
		{
			SettingsPanels::drawWristImuEnableControl(m_tracking);
			SettingsPanels::drawWristImuArmControls(m_tracking);
		}
		else
		{
			// The device table leads without a heading, and the wrist lines
			// keep theirs to set them apart from it
			SettingsPanels::drawWristImuDeviceTable(m_tracking);
			SettingsPanels::drawWristReadoutSection(m_tracking);
		}
		break;
	case ePipelineStage::BodySolver:
		if (section == eStageSection::Inputs)
		{
			SettingsPanels::drawBodyCalibrationControl(m_tracking);
		}
		else if (section == eStageSection::Settings)
		{
			SettingsPanels::drawBodyDimensionControls(m_tracking);
		}
		else
		{
			// The readout is the section's only content
			SettingsPanels::drawBodySolveReadout(m_tracking);
		}
		break;
	case ePipelineStage::DirectSolver:
		if (section == eStageSection::Inputs)
		{
			SettingsPanels::drawDirectCameraControl(m_tracking);
			SettingsPanels::drawDirectNeutralControl(m_tracking);
			SettingsPanels::drawHandCalibrationSection(m_tracking);
			ImGui::SeparatorText(locText("directStage.bodySection"));
			SettingsPanels::drawBodyCalibrationControl(m_tracking);
		}
		else if (section == eStageSection::Settings)
		{
			SettingsPanels::drawDirectEnableControl(m_tracking);
			SettingsPanels::drawDirectTiltControl(m_tracking);
			// The proportions the arm chain and the head are solved with
			ImGui::SeparatorText(locText("directStage.bodySection"));
			SettingsPanels::drawBodyDimensionControls(m_tracking);
			SettingsPanels::drawForearmLengthControl(m_tracking, "directStage.forearmLengthTooltip");
		}
		else
		{
			// Two readouts share the section, so each keeps its heading
			SettingsPanels::drawDirectSolveSummary(m_tracking);
			SettingsPanels::drawHoldStillSection(m_tracking);
		}
		break;
	case ePipelineStage::Filters:
		SettingsPanels::drawDirectFiltersReadout(m_tracking);
		break;
	case ePipelineStage::Torso:
		SettingsPanels::drawDirectTorsoReadout(m_tracking);
		break;
	case ePipelineStage::ArmChain:
		SettingsPanels::drawDirectArmChainReadout(m_tracking);
		break;
	case ePipelineStage::Palm:
		SettingsPanels::drawDirectPalmReadout(m_tracking);
		break;
	case ePipelineStage::HoldRest:
		SettingsPanels::drawDirectHoldRestReadout(m_tracking);
		break;
	case ePipelineStage::Phone:
		if (section == eStageSection::Settings)
		{
			SettingsPanels::drawFaceStreamControls(m_tracking);
			SettingsPanels::drawFaceHeadAlignControls(m_tracking);
		}
		else
		{
			SettingsPanels::drawFaceStreamStatus(m_tracking);
		}
		break;
	case ePipelineStage::Microphone:
		if (section == eStageSection::Inputs)
		{
			m_audioPanel->drawDeviceControls();
			m_audioPanel->drawProfileControls(m_bWizardActive, m_audioAction);
		}
		else if (section == eStageSection::Settings)
		{
			m_audioPanel->drawSettingsControls();
		}
		else
		{
			// The weights read the shown fused result and draw in a replay
			// without recorded stage state too
			m_audioPanel->drawLevelMeter(*m_microphone);
			m_audioPanel->drawStatus(*m_microphone);
			ImGui::Separator();
			m_audioPanel->drawWeights(*m_tracking.fusion->fused, m_tracking.fusion->bReplay);
		}
		break;
	case ePipelineStage::Profile:
		if (section == eStageSection::Inputs)
		{
			m_faceTab->drawProfileBar(m_app);
		}
		else if (section == eStageSection::Settings)
		{
			const std::optional<FaceExprEditTarget> editRequest=
				m_faceTab->drawRows(m_app, m_tracking.fusion->fused->face, m_activeEdit);
			if (editRequest.has_value())
				m_faceEditRequest= editRequest;
		}
		else
		{
			// The avatar's mouth targets as the stream carries them on the
			// shown frame, the section's only content
			m_mouthReadout->draw(m_app->getAvatarModel(), m_app->getFaceBinding(), m_tracking.fusion->fused->face);
		}
		break;
	case ePipelineStage::Retarget:
		if (section == eStageSection::Inputs)
		{
			m_avatarPanel->drawLoadControls();
			if (m_app->getAvatarModel() != nullptr)
			{
				ImGui::Separator();
				m_avatarPanel->drawModelSummary();
			}
		}
		else if (section == eStageSection::Settings)
		{
			drawRetargetSettings();
		}
		else
		{
			SettingsPanels::drawRetargetChainReadout(m_tracking, m_app->getAvatarModel() != nullptr);
		}
		break;
	case ePipelineStage::Output:
		if (section == eStageSection::Settings)
			SettingsPanels::drawOscEnableControl(m_osc);
		else
			SettingsPanels::drawOscStreamReadout(m_osc);
		break;
	case ePipelineStage::DeliveryRules:
		if (section == eStageSection::Settings)
			SettingsPanels::drawOscDeliveryControls(m_osc);
		else
			SettingsPanels::drawOscDeliveryReadout(m_osc, *m_tracking.fusion->fused, m_tracking.sourcePose->source);
		break;
	case ePipelineStage::VmcStream:
		SettingsPanels::drawOscTargetControls(m_osc);
		SettingsPanels::drawOscVmcControls(m_osc);
		break;
	case ePipelineStage::SourcePose:
		SettingsPanels::drawSourcePoseReadout(m_tracking);
		break;
	default:
		break;
	}
}

void StageContent::drawRetargetSettings()
{
	// Placement, Mapping and the preview pose need a loaded model, read as
	// they draw since the inputs above can unload it this frame. Placement
	// and Mapping start closed.
	if (m_app->getAvatarModel() != nullptr)
	{
		if (ImGui::TreeNode(locLabel("avatarPanel.tabPlacement")))
		{
			m_avatarPanel->drawPlacement();
			ImGui::TreePop();
		}
		if (ImGui::TreeNode(locLabel("avatarPanel.tabMapping")))
		{
			m_avatarPanel->drawMapping();
			ImGui::TreePop();
		}
		m_avatarPanel->drawPreviewPose();
	}
	m_avatarPanel->drawChains();
}
