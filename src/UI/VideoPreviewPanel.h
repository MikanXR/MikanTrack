#pragma once

#include <memory>
#include <vector>

#include "opencv2/core/mat.hpp"

#include "HandOverlay.h"
#include "TrackingTypes.h"

class GlTexture;

// The camera view: the cameras side by side, each letterboxed in its own
// pane with its own tracking overlay and image->screen mapping, or one
// camera filling the region. The "active" camera (used by the calibration
// wizards) gets a highlight border; wizards pin it via setActiveCamera(). It
// owns no window: a host draws the body.
class VideoPreviewPanel
{
public:
	VideoPreviewPanel();
	~VideoPreviewPanel();

	void setCameraCount(size_t count);
	size_t getCameraCount() const { return m_panes.size(); }

	// Uploads a new BGR frame for one camera
	void setFrame(int cameraIndex, const cv::Mat& bgr);

	// Draws the cameras into the current window's remaining content region.
	// results and executionProviders are indexed by camera. forearms
	// (optional, indexed by camera) draws the wrist-IMU forearm projected
	// into each camera's image; pass nullptr to skip. cameraFilter -1 draws
	// every camera, a camera's index draws that camera alone; an index the
	// panel has no camera for draws every camera.
	void drawBody(const std::vector<const TrackingFrameResult*>& results,
				  const std::vector<const char*>& executionProviders,
				  const std::vector<ForearmOverlay>* forearms,
				  int cameraFilter);

	// Called on a frame the body is not drawn, so the wizard overlays see no
	// draw list rather than last frame's
	void clearLastDrawList() { m_lastDrawList= nullptr; }

	// Per-camera mapping from full-frame image pixels to screen pixels for the
	// most recent drawBody, valid for the cameras it drew
	const ImageToScreenMapping& getImageToScreenMapping(int cameraIndex) const;

	// Draw list for wizard overlays (nullptr on a frame the body is not drawn)
	ImDrawList* getLastDrawList() const { return m_lastDrawList; }

	// Live GL texture of one camera's latest frame (0 until a frame arrived),
	// for embedding small previews outside the panel (e.g. the setup flow's
	// camera-selection prompt)
	uint32_t getCameraTextureId(int cameraIndex) const;
	// Width over height of that texture; 16:9 until a frame arrived
	float getCameraAspect(int cameraIndex) const;

	void setActiveCamera(int cameraIndex) { m_activeCamera= cameraIndex; }
	int getActiveCamera() const { return m_activeCamera; }

	bool getShowDetectionBoxes() const { return m_bShowDetectionBoxes; }
	void setShowDetectionBoxes(bool bShow) { m_bShowDetectionBoxes= bShow; }

	bool getShowBodyPose() const { return m_bShowBodyPose; }
	void setShowBodyPose(bool bShow) { m_bShowBodyPose= bShow; }
	// The direct solver's verdict on one camera's hand slots this frame
	// (whether each was the hand), -1 for no camera: a rejected slot draws
	// dimmed unless every slot is shown
	void setHandUse(int cameraIndex, const std::array<bool, 2>& handUsed)
	{
		m_handUseCamera= cameraIndex;
		m_handUsed= handUsed;
	}
	bool getShowAllHands() const { return m_bShowAllHands; }
	void setShowAllHands(bool bShow) { m_bShowAllHands= bShow; }
	bool getShowOverlay() const { return m_bShowOverlay; }
	void setShowOverlay(bool bShow) { m_bShowOverlay= bShow; }

private:
	struct CameraPane
	{
		std::unique_ptr<GlTexture> texture;
		ImageToScreenMapping mapping;
		bool bHasFrame= false;
	};

	std::vector<std::unique_ptr<CameraPane>> m_panes;
	ImageToScreenMapping m_fallbackMapping;
	ImDrawList* m_lastDrawList= nullptr;
	int m_activeCamera= 0;

	bool m_bShowOverlay= true;
	bool m_bShowDetectionBoxes= false;
	bool m_bShowAllHands= false;
	int m_handUseCamera= -1;
	std::array<bool, 2> m_handUsed{false, false};
	// Raw body landmarks on body-pose cameras (the solver's source data)
	bool m_bShowBodyPose= true;
};
