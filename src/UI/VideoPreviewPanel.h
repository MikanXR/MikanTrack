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
	// panel has no camera for draws every camera. This is the body the
	// wizards draw on: it publishes its draw list and the drawn cameras'
	// mappings, and a click on a pane makes that camera the active one.
	void drawBody(const std::vector<const TrackingFrameResult*>& results,
				  const std::vector<const char*>& executionProviders,
				  const std::vector<ForearmOverlay>* forearms,
				  int cameraFilter);

	// Draws the cameras the way drawBody does for another window showing
	// them: nothing is published, so the wizards keep drawing on drawBody's
	// panes, and a click changes no active camera
	void drawSecondaryBody(const std::vector<const TrackingFrameResult*>& results,
						   const std::vector<const char*>& executionProviders,
						   const std::vector<ForearmOverlay>* forearms, int cameraFilter) const;

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
	// The size of the camera's newest frame, false before one has arrived
	bool getCameraFrameSize(int cameraIndex, int& outWidth, int& outHeight) const;

	void setActiveCamera(int cameraIndex) { m_activeCamera= cameraIndex; }
	int getActiveCamera() const { return m_activeCamera; }

	bool getShowDetectionBoxes() const { return m_bShowDetectionBoxes; }
	void setShowDetectionBoxes(bool bShow) { m_bShowDetectionBoxes= bShow; }

	bool getShowBodyPose() const { return m_bShowBodyPose; }
	void setShowBodyPose(bool bShow) { m_bShowBodyPose= bShow; }
	bool getShowWholeBodyFace() const { return m_bShowWholeBodyFace; }
	void setShowWholeBodyFace(bool bShow) { m_bShowWholeBodyFace= bShow; }
	bool getShowWholeBodyHands() const { return m_bShowWholeBodyHands; }
	void setShowWholeBodyHands(bool bShow) { m_bShowWholeBodyHands= bShow; }
	bool getShowLowVisibilityLandmarks() const { return m_bShowLowVisibilityLandmarks; }
	void setShowLowVisibilityLandmarks(bool bShow) { m_bShowLowVisibilityLandmarks= bShow; }
	// The direct solver's verdict on one camera's hand slots this frame
	// (whether each was the hand), -1 for no camera: a rejected slot draws as
	// the hand slot draw mode says
	void setHandUse(int cameraIndex, const std::array<bool, 2>& handUsed)
	{
		m_handUseCamera= cameraIndex;
		m_handUsed= handUsed;
	}
	eHandSlotDrawMode getHandSlotDrawMode() const { return m_handSlotDrawMode; }
	void setHandSlotDrawMode(eHandSlotDrawMode mode) { m_handSlotDrawMode= mode; }
	bool getShowOverlay() const { return m_bShowOverlay; }
	void setShowOverlay(bool bShow) { m_bShowOverlay= bShow; }

private:
	struct CameraPane
	{
		std::unique_ptr<GlTexture> texture;
		ImageToScreenMapping mapping;
		bool bHasFrame= false;
	};

	// Draws the panes into the current window's remaining region on its
	// draw list. inOutMappings holds one mapping per camera, and each pane
	// drawn with a frame writes its own. Returns the camera whose pane was
	// clicked, -1 for none.
	int drawPanes(const std::vector<const TrackingFrameResult*>& results,
				  const std::vector<const char*>& executionProviders, const std::vector<ForearmOverlay>* forearms,
				  int cameraFilter, std::vector<ImageToScreenMapping>& inOutMappings) const;

	std::vector<std::unique_ptr<CameraPane>> m_panes;
	ImageToScreenMapping m_fallbackMapping;
	ImDrawList* m_lastDrawList= nullptr;
	int m_activeCamera= 0;

	bool m_bShowOverlay= true;
	bool m_bShowDetectionBoxes= false;
	eHandSlotDrawMode m_handSlotDrawMode= eHandSlotDrawMode::DimRejected;
	int m_handUseCamera= -1;
	std::array<bool, 2> m_handUsed{false, false};
	// Raw body landmarks on body-pose cameras (the solver's source data)
	bool m_bShowBodyPose= true;
	// The whole-body backend's face and hand blocks, each on its own toggle
	bool m_bShowWholeBodyFace= true;
	bool m_bShowWholeBodyHands= true;
	// Body and whole-body landmarks under the visibility gate, drawn dimmed
	// when on. Off by default: the unseen landmarks jump around the frame
	bool m_bShowLowVisibilityLandmarks= false;
};
