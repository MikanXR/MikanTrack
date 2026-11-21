#include "VideoPreviewPanel.h"

#include "LocText.h"

#include <algorithm>

#include "GlTexture.h"

VideoPreviewPanel::VideoPreviewPanel()
{
	setCameraCount(1);
}

VideoPreviewPanel::~VideoPreviewPanel()= default;

void VideoPreviewPanel::setCameraCount(size_t count)
{
	count= std::max<size_t>(count, 1);

	while (m_panes.size() > count)
		m_panes.pop_back();
	while (m_panes.size() < count)
	{
		auto pane= std::make_unique<CameraPane>();
		pane->texture= std::make_unique<GlTexture>();
		m_panes.push_back(std::move(pane));
	}

	m_activeCamera= std::min(m_activeCamera, (int)m_panes.size() - 1);
}

void VideoPreviewPanel::setVideoVisible(int cameraIndex, bool bVisible, bool bGranted)
{
	if (cameraIndex < 0 || cameraIndex >= (int)m_panes.size())
		return;

	CameraPane& pane= *m_panes[cameraIndex];
	// A hidden camera keeps no pixels, so none can be drawn by a later frame
	if (!bVisible)
		pane.texture->dispose();
	pane.bVideoVisible= bVisible;
	pane.bVideoGranted= bGranted;
}

bool VideoPreviewPanel::getVideoVisible(int cameraIndex) const
{
	return cameraIndex >= 0 && cameraIndex < (int)m_panes.size() && m_panes[cameraIndex]->bVideoVisible;
}

void VideoPreviewPanel::setFrame(int cameraIndex, const cv::Mat& bgr)
{
	if (cameraIndex < 0 || cameraIndex >= (int)m_panes.size() || bgr.empty())
		return;

	CameraPane& pane= *m_panes[cameraIndex];
	pane.bHasFrame= true;
	pane.frameWidth= bgr.cols;
	pane.frameHeight= bgr.rows;
	if (!pane.bVideoVisible)
		return;

	// A new texture shows its second upload first
	const bool bNewTexture= !pane.texture->getIsValid();
	pane.texture->uploadBGR(bgr);
	if (bNewTexture)
		pane.texture->uploadBGR(bgr);
}

int VideoPreviewPanel::consumeVideoToggleRequest()
{
	const int cameraIndex= m_videoToggleRequest;
	m_videoToggleRequest= -1;
	return cameraIndex;
}

uint32_t VideoPreviewPanel::getCameraTextureId(int cameraIndex) const
{
	if (cameraIndex < 0 || cameraIndex >= (int)m_panes.size())
		return 0;

	const CameraPane& pane= *m_panes[cameraIndex];
	if (!pane.bHasFrame || !pane.bVideoVisible || !pane.texture->getIsValid())
		return 0;
	return pane.texture->getGlTextureId();
}

float VideoPreviewPanel::getCameraAspect(int cameraIndex) const
{
	if (cameraIndex >= 0 && cameraIndex < (int)m_panes.size())
	{
		const CameraPane& pane= *m_panes[cameraIndex];
		if (pane.bHasFrame && pane.frameHeight > 0)
			return (float)pane.frameWidth / (float)pane.frameHeight;
	}
	return 16.f / 9.f;
}

bool VideoPreviewPanel::getCameraFrameSize(int cameraIndex, int& outWidth, int& outHeight) const
{
	if (cameraIndex >= 0 && cameraIndex < (int)m_panes.size())
	{
		const CameraPane& pane= *m_panes[cameraIndex];
		if (pane.bHasFrame && pane.frameWidth > 0 && pane.frameHeight > 0)
		{
			outWidth= pane.frameWidth;
			outHeight= pane.frameHeight;
			return true;
		}
	}
	return false;
}

const ImageToScreenMapping& VideoPreviewPanel::getImageToScreenMapping(int cameraIndex) const
{
	if (cameraIndex >= 0 && cameraIndex < (int)m_panes.size())
		return m_panes[cameraIndex]->mapping;

	return m_fallbackMapping;
}

void VideoPreviewPanel::drawBody(const std::vector<const TrackingFrameResult*>& results,
								 const std::vector<const char*>& executionProviders,
								 const std::vector<ForearmOverlay>* forearms,
								 int cameraFilter)
{
	// The one body the wizards' overlays read: its draw list, its mappings
	// and its clicks are the panel's
	m_lastDrawList= ImGui::GetWindowDrawList();
	std::vector<ImageToScreenMapping> mappings;
	for (const std::unique_ptr<CameraPane>& pane : m_panes)
		mappings.push_back(pane->mapping);
	const PaneClicks clicks= drawPanes(results, executionProviders, forearms, cameraFilter, mappings);
	for (size_t cameraIndex= 0; cameraIndex < m_panes.size(); ++cameraIndex)
		m_panes[cameraIndex]->mapping= mappings[cameraIndex];
	if (clicks.paneCamera >= 0)
		m_activeCamera= clicks.paneCamera;
	if (clicks.videoToggleCamera >= 0)
		m_videoToggleRequest= clicks.videoToggleCamera;
}

void VideoPreviewPanel::drawSecondaryBody(const std::vector<const TrackingFrameResult*>& results,
										  const std::vector<const char*>& executionProviders,
										  const std::vector<ForearmOverlay>* forearms, int cameraFilter)
{
	std::vector<ImageToScreenMapping> mappings(m_panes.size());
	const PaneClicks clicks= drawPanes(results, executionProviders, forearms, cameraFilter, mappings);
	if (clicks.videoToggleCamera >= 0)
		m_videoToggleRequest= clicks.videoToggleCamera;
}

VideoPreviewPanel::PaneClicks VideoPreviewPanel::drawPanes(const std::vector<const TrackingFrameResult*>& results,
														   const std::vector<const char*>& executionProviders,
														   const std::vector<ForearmOverlay>* forearms,
														   int cameraFilter,
														   std::vector<ImageToScreenMapping>& inOutMappings) const
{
	const bool bOneCamera= cameraFilter >= 0 && cameraFilter < (int)m_panes.size();
	const int firstCamera= bOneCamera ? cameraFilter : 0;
	const int paneCount= bOneCamera ? 1 : (int)m_panes.size();
	const ImVec2 panelSize= ImGui::GetContentRegionAvail();
	const float paneSpacing= 4.f;
	const float paneWidth= (panelSize.x - paneSpacing * (float)(paneCount - 1)) / (float)paneCount;
	const float paneHeight= panelSize.y;

	ImDrawList* drawList= ImGui::GetWindowDrawList();
	const ImVec2 panelOrigin= ImGui::GetCursorScreenPos();
	PaneClicks clicks;

	for (int paneIndex= 0; paneIndex < paneCount; ++paneIndex)
	{
		const int cameraIndex= firstCamera + paneIndex;
		const CameraPane& pane= *m_panes[cameraIndex];
		ImageToScreenMapping& mapping= inOutMappings[cameraIndex];
		const ImVec2 paneOrigin(panelOrigin.x + (paneWidth + paneSpacing) * (float)paneIndex, panelOrigin.y);

		// Pane background + active highlight
		drawList->AddRectFilled(
			paneOrigin, ImVec2(paneOrigin.x + paneWidth, paneOrigin.y + paneHeight), IM_COL32(12, 12, 14, 255));
		if (paneCount > 1 && cameraIndex == m_activeCamera)
		{
			drawList->AddRect(
				paneOrigin, ImVec2(paneOrigin.x + paneWidth, paneOrigin.y + paneHeight),
				IM_COL32(120, 180, 255, 180), 0.f, 0, 2.f);
		}

		if (!pane.bHasFrame || pane.frameWidth <= 0 || pane.frameHeight <= 0)
		{
			const char* message= paneCount > 1 ? locText("videoPreviewPanel.noStreamText")
												: locText("videoPreviewPanel.noVideoStreamText");
			const ImVec2 textSize= ImGui::CalcTextSize(message);
			drawList->AddText(
				ImVec2(paneOrigin.x + (paneWidth - textSize.x) * 0.5f, paneOrigin.y + (paneHeight - textSize.y) * 0.5f),
				IM_COL32(140, 140, 140, 255), message);
			continue;
		}

		// Letterbox the image inside the pane
		const float imageWidth= (float)pane.frameWidth;
		const float imageHeight= (float)pane.frameHeight;
		const float scale= std::min(paneWidth / imageWidth, paneHeight / imageHeight);
		const ImVec2 displaySize(imageWidth * scale, imageHeight * scale);
		const ImVec2 imagePos(
			paneOrigin.x + (paneWidth - displaySize.x) * 0.5f,
			paneOrigin.y + (paneHeight - displaySize.y) * 0.5f);
		const ImVec2 imageMax(imagePos.x + displaySize.x, imagePos.y + displaySize.y);

		// The camera's video, or the frame's shape alone while it is hidden
		const bool bShowVideo= pane.bVideoVisible && pane.texture->getIsValid();
		if (bShowVideo)
			drawList->AddImage((ImTextureID)(intptr_t)pane.texture->getGlTextureId(), imagePos, imageMax);
		else
			drawList->AddRectFilled(imagePos, imageMax, IM_COL32(24, 25, 30, 255));

		mapping.screenOrigin= imagePos;
		mapping.scale= scale;

		// The pane's click, which drawBody makes the active camera. The
		// video button drawn over it below takes its own clicks.
		ImGui::SetCursorScreenPos(paneOrigin);
		ImGui::PushID(cameraIndex);
		ImGui::SetNextItemAllowOverlap();
		if (ImGui::InvisibleButton("pane", ImVec2(paneWidth, paneHeight)))
			clicks.paneCamera= cameraIndex;

		const TrackingFrameResult* result=
			cameraIndex < (int)results.size() ? results[cameraIndex] : nullptr;

		if (m_bShowOverlay)
		{
			if (result != nullptr)
			{
				const std::array<bool, 2>* handUsed= cameraIndex == m_handUseCamera ? &m_handUsed : nullptr;
				HandOverlay::drawTrackingResult(drawList, *result, mapping, m_bShowDetectionBoxes, handUsed,
												m_handSlotDrawMode);
				// Only body-pose cameras carry an observation, so this draws
				// on exactly the camera the stage is enabled for
				if (m_bShowBodyPose)
					HandOverlay::drawBodyPose(drawList, result->body, mapping, m_bShowLowVisibilityLandmarks);
				HandOverlay::drawWholeBodyExtras(drawList, result->body, mapping, m_bShowWholeBodyFace,
												 m_bShowWholeBodyHands, m_bShowLowVisibilityLandmarks);
			}
			if (forearms != nullptr && cameraIndex < (int)forearms->size())
				HandOverlay::drawForearmOverlay(drawList, (*forearms)[cameraIndex], mapping);
		}

		// HUD: camera label + fps / inference / EP badge
		const ImVec2 hudPos(imagePos.x + 8.f, imagePos.y + 8.f);
		float hudBottom= hudPos.y;
		{
			const char* executionProvider=
				cameraIndex < (int)executionProviders.size() ? executionProviders[cameraIndex] : "?";

			std::string hud;
			if (result != nullptr)
			{
				hud= locFormat("videoPreviewPanel.hudWithResultFmt",
							   cameraIndex + 1, result->captureFps, result->inferenceMs, executionProvider);
			}
			else
			{
				hud= locFormat("videoPreviewPanel.hudFmt", cameraIndex + 1, executionProvider);
			}

			const ImVec2 hudSize= ImGui::CalcTextSize(hud.c_str());
			drawList->AddRectFilled(
				ImVec2(hudPos.x - 4.f, hudPos.y - 2.f),
				ImVec2(hudPos.x + hudSize.x + 4.f, hudPos.y + hudSize.y + 2.f),
				IM_COL32(0, 0, 0, 160), 3.f);

			const bool bGpu= executionProvider != nullptr && strstr(executionProvider, "DirectML") != nullptr;
			drawList->AddText(hudPos, bGpu ? IM_COL32(120, 255, 120, 255) : IM_COL32(255, 220, 100, 255), hud.c_str());
			hudBottom= hudPos.y + hudSize.y + 2.f;
		}

		// The video button under the HUD, and beside it the badge that says
		// the camera's video is on screen
		{
			ImGui::SetCursorScreenPos(ImVec2(hudPos.x - 4.f, hudBottom + 4.f));
			ImGui::BeginDisabled(pane.bVideoGranted);
			if (ImGui::SmallButton(locLabel(pane.bVideoVisible ? "videoPreviewPanel.hideVideoButton"
															   : "videoPreviewPanel.showVideoButton")))
				clicks.videoToggleCamera= cameraIndex;
			ImGui::EndDisabled();
			if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			{
				ImGui::SetTooltip("%s", locText(pane.bVideoGranted    ? "videoPreviewPanel.videoCalibrationTooltip"
												  : pane.bVideoVisible ? "videoPreviewPanel.hideVideoTooltip"
																	   : "videoPreviewPanel.showVideoTooltip"));
			}

			if (pane.bVideoVisible)
			{
				const std::string badge= m_streamingSoftwareName.empty()
					? std::string(locText("videoPreviewPanel.videoVisibleBadge"))
					: locFormat("videoPreviewPanel.videoVisibleStreamingBadgeFmt", m_streamingSoftwareName.c_str());
				const ImVec2 buttonMin= ImGui::GetItemRectMin();
				const ImVec2 buttonMax= ImGui::GetItemRectMax();
				const ImVec2 badgeSize= ImGui::CalcTextSize(badge.c_str());
				const ImVec2 badgePos(buttonMax.x + 10.f, buttonMin.y + (buttonMax.y - buttonMin.y - badgeSize.y) * 0.5f);
				drawList->AddRectFilled(ImVec2(badgePos.x - 4.f, buttonMin.y),
										ImVec2(badgePos.x + badgeSize.x + 4.f, buttonMax.y),
										IM_COL32(190, 30, 30, 230), 3.f);
				drawList->AddText(badgePos, IM_COL32(255, 255, 255, 255), badge.c_str());
			}
		}
		ImGui::PopID();
	}
	return clicks;
}
