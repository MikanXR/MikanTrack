#include "StageView.h"

#include "imgui.h"
#include "imgui_internal.h" // window settings, dock builder

#include "LocText.h"
#include "PipelineText.h"
#include "VideoPreviewPanel.h"

SceneLayers makeStageSceneLayers(const PipelineStageId& topLevel, bool bShowAvatar, bool bShowPerCameraSkeletons)
{
	SceneLayers layers;
	layers.bPerCameraSkeletons= false;
	switch (topLevel.stage)
	{
		case ePipelineStage::Fusion:
			// The cameras' agreement is what fusion is about
			layers.bAvatar= false;
			layers.bPerCameraSkeletons= true;
			break;
		case ePipelineStage::Imu:
		case ePipelineStage::BodySolver:
			layers.bAvatar= false;
			break;
		case ePipelineStage::SourcePose:
			// The source pose alone, as the target solve takes it
			layers.bCameraFrustums= false;
			layers.bFusedSkeleton= false;
			layers.bAvatar= false;
			layers.bSourcePose= true;
			break;
		case ePipelineStage::Retarget:
			// The source pose beside the avatar posed from it, and where the
			// solve aimed the avatar's wrists
			layers.bCameraFrustums= false;
			layers.bFusedSkeleton= false;
			layers.bSourcePose= true;
			layers.bIkGoals= true;
			break;
		case ePipelineStage::Output:
		case ePipelineStage::Face:
			// The avatar alone: the pose and the face a receiver is sent
			layers.bCameraFrustums= false;
			layers.bFusedSkeleton= false;
			break;
		default:
			// Nothing selected: the whole scene
			break;
	}
	// The solver's chains sit beside the avatar they posed, except where the
	// avatar stands alone
	layers.bDirectSolve= layers.bAvatar && (layers.bFusedSkeleton || layers.bSourcePose);
	if (!bShowAvatar)
		layers.bAvatar= false;
	layers.bGizmos= layers.bAvatar;
	layers.bIkGoals= layers.bIkGoals && layers.bAvatar;
	if (bShowPerCameraSkeletons && layers.bFusedSkeleton)
		layers.bPerCameraSkeletons= true;
	return layers;
}

StageView::StageView(VideoPreviewPanel* previewPanel, Scene3dPanel* scenePanel)
	: m_previewPanel(previewPanel)
	, m_scenePanel(scenePanel)
{
}

void StageView::update(const PipelineStageId& selected, bool bDirect, int directCameraIndex, bool bCameraWizardActive)
{
	m_selected= selected;
	m_bDirect= bDirect;
	m_directCameraIndex= directCameraIndex;
	m_bCameraWizardActive= bCameraWizardActive;
	decide();
}

void StageView::setMode(eStageViewMode mode)
{
	m_mode= mode;
	decide();
}

StageViewContent decideStageViewContent(const PipelineStageId& stage, bool bDirect, int directCameraIndex,
										int cameraCount)
{
	const PipelineStageId topLevel= stage.isValid() ? PipelineModel::getTopLevel(stage) : PipelineStageId();

	// The capture and the direct solver are about the frames themselves:
	// the camera a lane's stage belongs to, or the direct solver's camera
	StageViewContent content;
	if (topLevel.stage == ePipelineStage::Capture)
	{
		content.bCameras= true;
		content.cameraIndex= stage.cameraIndex;
	}
	else if (topLevel.stage == ePipelineStage::DirectSolver)
	{
		content.bCameras= true;
		content.cameraIndex= bDirect ? directCameraIndex : -1;
	}
	else
	{
		content.sceneStage= topLevel;
	}
	if (content.cameraIndex >= cameraCount)
		content.cameraIndex= -1;
	return content;
}

void StageView::decide()
{
	const StageViewContent follow=
		decideStageViewContent(m_selected, m_bDirect, m_directCameraIndex, (int)m_previewPanel->getCameraCount());

	StageViewContent content;
	if (m_bCameraWizardActive)
	{
		// The pattern feed is the wizard's whole UI
		content.bCameras= true;
		content.bWizard= true;
	}
	else if (m_mode == eStageViewMode::Cameras || (m_mode == eStageViewMode::Follow && follow.bCameras))
	{
		content.bCameras= true;
		content.cameraIndex= follow.bCameras ? follow.cameraIndex : -1;
	}
	else if (!follow.bCameras)
	{
		content.sceneStage= follow.sceneStage;
	}
	m_content= content;
}

void StageView::draw(bool* pOpen, const Frame& frame)
{
	m_previewPanel->clearLastDrawList();

	const char* title= locWindowTitle("windows.stageView");
	// An imgui.ini saved before the Stage View existed has no place for it,
	// so it would float. Its first appearance takes the dock node the window
	// it replaced was in. A fresh layout has docked it already, which leaves
	// it settings of its own.
	if (!m_bDockChecked)
	{
		m_bDockChecked= true;
		if (ImGui::FindWindowSettingsByID(ImHashStr(title)) == nullptr && ImGui::FindWindowByName(title) == nullptr)
		{
			// The stable ids of the 3D scene and video preview windows
			static const char* k_replacedWindowIds[]= {"3D Scene", "Video Preview"};
			for (const char* windowId : k_replacedWindowIds)
			{
				const ImGuiWindowSettings* settings= ImGui::FindWindowSettingsByID(ImHashStr(windowId));
				if (settings != nullptr && settings->DockId != 0 && ImGui::DockBuilderGetNode(settings->DockId) != nullptr)
				{
					ImGui::SetNextWindowDockID(settings->DockId, ImGuiCond_FirstUseEver);
					break;
				}
			}
		}
	}

	if (!ImGui::Begin(title, pOpen))
	{
		ImGui::End();
		return;
	}

	drawHeader();
	if (m_content.bCameras)
	{
		m_previewPanel->drawBody(*frame.cameraResults, *frame.executionProviders, frame.forearms,
								 m_content.cameraIndex);
	}
	else
	{
		const SceneLayers layers= makeStageSceneLayers(m_content.sceneStage, m_scenePanel->getShowAvatar(),
													   m_scenePanel->getShowPerCameraSkeletons());
		SceneViewOptions options;
		options.bPickSourceChains= isRetargetView();
		options.bSideBySide= options.bPickSourceChains && m_retargetLayout == eRetargetLayout::SideBySide;
		m_scenePanel->drawBody(*frame.fused, *frame.sceneCameras, *frame.scenePerCamera, *frame.gizmos, layers,
							   options);
	}

	ImGui::End();
}

void StageView::drawHeader()
{
	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted(makeTitle().c_str());

	static const char* k_modeKeys[]= {"stageView.modeFollow", "stageView.modeCameras", "stageView.modeScene"};
	ImGui::SameLine();
	ImGui::BeginDisabled(m_content.bWizard);
	ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.f);
	if (ImGui::BeginCombo("##stageViewMode", locText(k_modeKeys[(int)m_mode])))
	{
		for (int modeIndex= 0; modeIndex < IM_ARRAYSIZE(k_modeKeys); ++modeIndex)
		{
			if (ImGui::Selectable(locLabel(k_modeKeys[modeIndex]), modeIndex == (int)m_mode))
				setMode((eStageViewMode)modeIndex);
		}
		ImGui::EndCombo();
	}
	ImGui::EndDisabled();
	ImGui::SetItemTooltip("%s", locText(m_content.bWizard ? "stageView.wizardNote" : "stageView.modeTooltip"));

	if (!m_content.bCameras && m_scenePanel->hasCameraMatch())
	{
		ImGui::SameLine();
		bool bViewFromCamera= m_scenePanel->getViewFromCamera();
		if (ImGui::Checkbox(locLabel("stageView.viewFromCamera"), &bViewFromCamera))
			m_scenePanel->setViewFromCamera(bViewFromCamera);
		ImGui::SetItemTooltip("%s", locText("stageView.viewFromCameraTooltip"));
	}

	if (isRetargetView())
	{
		static const char* k_layoutKeys[]= {"stageView.layoutOverlay", "stageView.layoutSideBySide"};
		for (int layoutIndex= 0; layoutIndex < IM_ARRAYSIZE(k_layoutKeys); ++layoutIndex)
		{
			ImGui::SameLine();
			if (ImGui::RadioButton(locLabel(k_layoutKeys[layoutIndex]), layoutIndex == (int)m_retargetLayout))
				m_retargetLayout= (eRetargetLayout)layoutIndex;
			ImGui::SetItemTooltip("%s", locText("stageView.layoutTooltip"));
		}
		ImGui::TextDisabled("%s", locText("stageView.retargetLegend"));
	}
}

bool StageView::isRetargetView() const
{
	return !m_content.bCameras && m_content.sceneStage.isValid() &&
		m_content.sceneStage.stage == ePipelineStage::Retarget;
}

std::string StageView::makeTitle() const
{
	if (m_content.bWizard)
		return locText("stageView.titleWizard");
	if (m_content.bCameras)
	{
		return m_content.cameraIndex < 0 ? std::string(locText("stageView.titleAllCameras"))
										 : locFormat("stageView.titleCameraFmt", m_content.cameraIndex + 1);
	}
	if (!m_content.sceneStage.isValid())
		return locText("stageView.titleScene");
	return locFormat("stageView.titleStageFmt", PipelineText::stageName(m_content.sceneStage).c_str());
}

std::string StageView::describe() const
{
	std::string text;
	if (m_content.bCameras)
		text= m_content.cameraIndex < 0 ? "camera all" : "camera " + std::to_string(m_content.cameraIndex);
	else
		text= "scene " + (m_content.sceneStage.isValid() ? PipelineModel::toString(m_content.sceneStage) : "none");
	text+= std::string(" mode ") + getModeName(m_mode);
	if (isRetargetView())
		text+= std::string(" layout ") + getRetargetLayoutName(m_retargetLayout);
	if (m_content.bWizard)
		text+= " wizard";
	return text;
}

const char* StageView::getModeName(eStageViewMode mode)
{
	switch (mode)
	{
		case eStageViewMode::Cameras:
			return "cameras";
		case eStageViewMode::Scene:
			return "scene";
		default:
			return "follow";
	}
}

bool StageView::parseMode(const std::string& text, eStageViewMode& outMode)
{
	for (const eStageViewMode mode : {eStageViewMode::Follow, eStageViewMode::Cameras, eStageViewMode::Scene})
	{
		if (text == getModeName(mode))
		{
			outMode= mode;
			return true;
		}
	}
	return false;
}

const char* StageView::getRetargetLayoutName(eRetargetLayout layout)
{
	return layout == eRetargetLayout::SideBySide ? "sidebyside" : "overlay";
}

bool StageView::parseRetargetLayout(const std::string& text, eRetargetLayout& outLayout)
{
	for (const eRetargetLayout layout : {eRetargetLayout::Overlay, eRetargetLayout::SideBySide})
	{
		if (text == getRetargetLayoutName(layout))
		{
			outLayout= layout;
			return true;
		}
	}
	return false;
}
