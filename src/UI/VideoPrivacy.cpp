#include "VideoPrivacy.h"

#include <algorithm>
#include <cmath>

#include "imgui.h"

#include "opencv2/imgproc.hpp"

#include "CameraContext.h" // VisionPreviewFrame
#include "GlTexture.h"
#include "LocText.h"
#include "WizardHost.h" // eWizardKind

// The pixelated preview: how wide it is drawn, and how many blocks cross it.
// At this count a face fills a handful of blocks, enough to tell which
// camera this is and not who is in it.
static constexpr float k_previewWidth= 384.f;
static constexpr int k_previewBlockColumns= 48;
static constexpr float k_promptWrapWidth= 460.f;
static const ImVec4 k_colorStreamingWarning(1.f, 0.45f, 0.35f, 1.f);

// -- VideoPrivacyState ------------------------------------------------------

void VideoPrivacyState::setCameras(const std::vector<std::string>& deviceNames)
{
	if (deviceNames.size() != m_cameras.size())
	{
		m_cameras.assign(deviceNames.size(), Camera());
		for (size_t cameraIndex= 0; cameraIndex < deviceNames.size(); ++cameraIndex)
			m_cameras[cameraIndex].deviceName= deviceNames[cameraIndex];
		return;
	}

	for (size_t cameraIndex= 0; cameraIndex < deviceNames.size(); ++cameraIndex)
	{
		Camera& camera= m_cameras[cameraIndex];
		if (camera.deviceName != deviceNames[cameraIndex])
		{
			camera.deviceName= deviceNames[cameraIndex];
			camera.bUserShown= false;
		}
	}
}

void VideoPrivacyState::setGrants(int grantedCamera, bool bAllGranted)
{
	m_grantedCamera= grantedCamera;
	m_bAllGranted= bAllGranted;
}

void VideoPrivacyState::setUserShown(int cameraIndex, bool bShown)
{
	if (cameraIndex >= 0 && cameraIndex < (int)m_cameras.size())
		m_cameras[cameraIndex].bUserShown= bShown;
}

void VideoPrivacyState::reset()
{
	for (Camera& camera : m_cameras)
		camera.bUserShown= false;
	m_grantedCamera= -1;
	m_bAllGranted= false;
}

bool VideoPrivacyState::getUserShown(int cameraIndex) const
{
	return cameraIndex >= 0 && cameraIndex < (int)m_cameras.size() && m_cameras[cameraIndex].bUserShown;
}

bool VideoPrivacyState::isGranted(int cameraIndex) const
{
	return cameraIndex >= 0 && cameraIndex < (int)m_cameras.size() &&
		(m_bAllGranted || cameraIndex == m_grantedCamera);
}

bool VideoPrivacyState::isAnyVisible() const
{
	for (int cameraIndex= 0; cameraIndex < (int)m_cameras.size(); ++cameraIndex)
	{
		if (isVisible(cameraIndex))
			return true;
	}
	return false;
}

// -- Pixelated preview ------------------------------------------------------

cv::Mat makePixelatedPreview(const cv::Mat& bgr, int blockColumns, int outputWidth)
{
	if (bgr.empty() || blockColumns <= 0)
		return cv::Mat();

	const int columns= std::min(blockColumns, bgr.cols);
	const int rows= std::max(1, (int)std::lround((double)bgr.rows * (double)columns / (double)bgr.cols));
	cv::Mat blocks;
	cv::resize(bgr, blocks, cv::Size(columns, rows), 0.0, 0.0, cv::INTER_AREA);

	const int blockSize= std::max(1, outputWidth / columns);
	cv::Mat pixelated;
	cv::resize(blocks, pixelated, cv::Size(columns * blockSize, rows * blockSize), 0.0, 0.0, cv::INTER_NEAREST);
	return pixelated;
}

// -- VideoPrivacy -----------------------------------------------------------

// Opens (if needed) and begins a viewport-centered modal
static bool beginCenteredModal(const char* title)
{
	const ImGuiViewport* viewport= ImGui::GetMainViewport();
	const ImVec2 center(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
						viewport->WorkPos.y + viewport->WorkSize.y * 0.5f);
	ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

	if (!ImGui::IsPopupOpen(title))
		ImGui::OpenPopup(title);
	return ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize);
}

VideoPrivacy::VideoPrivacy()
	: m_promptWizard(eWizardKind::None)
	, m_pixelatedTexture(std::make_unique<GlTexture>())
{
}

VideoPrivacy::~VideoPrivacy()= default;

void VideoPrivacy::requestShow(int cameraIndex)
{
	if (m_prompt != ePrompt::None || cameraIndex < 0 || cameraIndex >= (int)m_state.getCameraCount())
		return;
	m_prompt= ePrompt::ShowVideo;
	m_promptCamera= cameraIndex;
	m_bDismissRequested= false;
}

void VideoPrivacy::requestWizardLaunch(eWizardKind kind, int cameraIndex)
{
	if (m_prompt != ePrompt::None)
		return;
	m_prompt= ePrompt::WizardLaunch;
	m_promptWizard= kind;
	m_promptCamera= cameraIndex;
	m_bDismissRequested= false;
}

void VideoPrivacy::dismiss()
{
	// The prompt closes itself on its next draw, inside its popup
	if (m_prompt != ePrompt::None)
		m_bDismissRequested= true;
}

void VideoPrivacy::reset()
{
	m_state.reset();
	dismiss();
}

void VideoPrivacy::closePrompt()
{
	ImGui::CloseCurrentPopup();
	m_pixelatedTexture->dispose();
	m_prompt= ePrompt::None;
	m_promptCamera= -1;
	m_bDismissRequested= false;
}

bool VideoPrivacy::drawPrompts(const std::vector<VisionPreviewFrame>& previews, eWizardKind& outKind,
							   int& outCameraIndex)
{
	switch (m_prompt)
	{
		case ePrompt::ShowVideo:
			drawShowVideoPrompt(previews);
			return false;
		case ePrompt::WizardLaunch:
		{
			// The prompt's fields are cleared as it closes
			const eWizardKind kind= m_promptWizard;
			const int cameraIndex= m_promptCamera;
			if (!drawWizardLaunchPrompt())
				return false;
			outKind= kind;
			outCameraIndex= cameraIndex;
			return true;
		}
		default:
			return false;
	}
}

void VideoPrivacy::drawStreamingWarning(const std::string& streamingSoftwareName)
{
	if (streamingSoftwareName.empty())
		return;
	ImGui::Spacing();
	ImGui::PushStyleColor(ImGuiCol_Text, k_colorStreamingWarning);
	ImGui::TextUnformatted(locFormat("videoPrivacy.streamingWarningFmt", streamingSoftwareName.c_str()).c_str());
	ImGui::PopStyleColor();
}

void VideoPrivacy::drawShowVideoPrompt(const std::vector<VisionPreviewFrame>& previews)
{
	if (!beginCenteredModal(locWindowTitle("windows.modalShowVideo")))
		return;

	ImGui::PushTextWrapPos(k_promptWrapWidth);
	ImGui::TextUnformatted(locFormat("videoPrivacy.showVideoPromptFmt", m_promptCamera + 1).c_str());
	ImGui::Spacing();
	ImGui::TextDisabled("%s", locText("videoPrivacy.showVideoNote"));
	drawStreamingWarning(getStreamingSoftwareName());
	ImGui::PopTextWrapPos();
	ImGui::Spacing();

	const cv::Mat* frame= nullptr;
	if (m_promptCamera < (int)previews.size() && !previews[m_promptCamera].bgr.empty())
		frame= &previews[m_promptCamera].bgr;
	const float aspect= frame != nullptr ? (float)frame->cols / (float)frame->rows : 16.f / 9.f;

	// The preview's area, dark until its button is held
	const ImVec2 previewMin= ImGui::GetCursorScreenPos();
	const ImVec2 previewSize(k_previewWidth, k_previewWidth / aspect);
	const ImVec2 previewMax(previewMin.x + previewSize.x, previewMin.y + previewSize.y);
	ImGui::Dummy(previewSize);
	ImDrawList* drawList= ImGui::GetWindowDrawList();
	drawList->AddRectFilled(previewMin, previewMax, IM_COL32(12, 12, 14, 255));

	ImGui::Button(locLabel("videoPrivacy.holdToPreviewButton"), ImVec2(k_previewWidth, 0.f));
	const bool bHeld= ImGui::IsItemActive();
	if (bHeld && frame != nullptr)
	{
		// Reduced on the CPU, so the frame itself is never uploaded from here
		const cv::Mat pixelated= makePixelatedPreview(*frame, k_previewBlockColumns, (int)k_previewWidth);
		// A new texture shows its second upload first
		const bool bNewTexture= !m_pixelatedTexture->getIsValid();
		m_pixelatedTexture->uploadBGR(pixelated);
		if (bNewTexture)
			m_pixelatedTexture->uploadBGR(pixelated);
		drawList->AddImage((ImTextureID)(intptr_t)m_pixelatedTexture->getGlTextureId(), previewMin, previewMax);
	}
	else
	{
		m_pixelatedTexture->dispose();
		const char* message=
			locText(frame != nullptr ? "videoPrivacy.previewHiddenText" : "videoPreviewPanel.noStreamText");
		const ImVec2 textSize= ImGui::CalcTextSize(message);
		drawList->AddText(ImVec2(previewMin.x + (previewSize.x - textSize.x) * 0.5f,
								 previewMin.y + (previewSize.y - textSize.y) * 0.5f),
						  IM_COL32(140, 140, 140, 255), message);
	}

	ImGui::Separator();
	bool bClose= m_bDismissRequested;
	if (ImGui::Button(locLabel("videoPrivacy.showVideoConfirmButton"), ImVec2(140, 0)))
	{
		m_state.setUserShown(m_promptCamera, true);
		bClose= true;
	}
	ImGui::SameLine();
	if (ImGui::Button(locLabel("common.cancel"), ImVec2(120, 0)))
		bClose= true;
	if (bClose)
		closePrompt();
	ImGui::EndPopup();
}

bool VideoPrivacy::drawWizardLaunchPrompt()
{
	if (!beginCenteredModal(locWindowTitle("windows.modalVideoCalibration")))
		return false;

	ImGui::PushTextWrapPos(k_promptWrapWidth);
	if (m_promptWizard == eWizardKind::Intrinsics)
		ImGui::TextUnformatted(locFormat("videoPrivacy.wizardOneCameraPromptFmt", m_promptCamera + 1).c_str());
	else
		ImGui::TextUnformatted(locText("videoPrivacy.wizardAllCamerasPrompt"));
	drawStreamingWarning(getStreamingSoftwareName());
	ImGui::PopTextWrapPos();

	ImGui::Separator();
	bool bAccepted= false;
	bool bClose= m_bDismissRequested;
	if (ImGui::Button(locLabel("common.continueLabel"), ImVec2(120, 0)))
	{
		bAccepted= true;
		bClose= true;
	}
	ImGui::SameLine();
	if (ImGui::Button(locLabel("common.cancel"), ImVec2(120, 0)))
		bClose= true;
	if (bClose)
		closePrompt();
	ImGui::EndPopup();
	return bAccepted;
}
