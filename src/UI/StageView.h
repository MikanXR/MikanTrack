#pragma once

#include <string>
#include <vector>

#include "HandOverlay.h" // ForearmOverlay
#include "PipelineModel.h"
#include "Scene3dPanel.h" // SceneLayers, SceneCameraView, SceneGizmos
#include "TrackingTypes.h"

class VideoPreviewPanel;

// Which body the Stage View shows: the one the Pipeline strip's selection
// asks for, or one of the two whatever the selection
enum class eStageViewMode : int
{
	Follow,
	Cameras,
	Scene,
};

// What the Stage View shows on a frame, decided from the selection, the mode
// and the calibration wizards
struct StageViewContent
{
	bool bCameras= false;
	// The camera view's camera, -1 for every camera
	int cameraIndex= -1;
	// The scene's top-level stage, whose layers it draws; invalid for the
	// whole scene
	PipelineStageId sceneStage;
	// A camera calibration wizard holds the view on every camera
	bool bWizard= false;
};

// What a scene shown for a top-level stage draws (an invalid stage is the
// whole scene, nothing selected), with the user's two settings on top: the
// avatar only while it is shown in the scene, and the per-camera skeletons
// wherever the fused skeleton is while the Fusion stage's toggle is on. The
// direct solve's chains and the gizmos go with the stage's avatar layer.
SceneLayers makeStageSceneLayers(const PipelineStageId& topLevel, bool bShowAvatar, bool bShowPerCameraSkeletons);

// The central view: one window that shows the camera view or the 3D scene
// for the stage selected in the Pipeline strip, with a header row naming
// what it shows, the mode override and the view-from-camera toggle. The two
// bodies belong to the panels it is given, which stay the providers.
class StageView
{
public:
	StageView(VideoPreviewPanel* previewPanel, Scene3dPanel* scenePanel);

	// The frame's inputs to the two bodies
	struct Frame
	{
		// The camera view: the live per-camera results, execution providers
		// and forearm overlays, indexed by camera
		const std::vector<const TrackingFrameResult*>* cameraResults= nullptr;
		const std::vector<const char*>* executionProviders= nullptr;
		const std::vector<ForearmOverlay>* forearms= nullptr;
		// The scene: the shown frame and the cameras it was made with
		const TrackingFrameResult* fused= nullptr;
		const std::vector<SceneCameraView>* sceneCameras= nullptr;
		const std::vector<const TrackingFrameResult*>* scenePerCamera= nullptr;
		SceneGizmos* gizmos= nullptr;
	};

	// Decides what the view shows this frame, whether or not it is drawn.
	// selected is the strip's selection, bDirect and directCameraIndex the
	// project shape the strip shows.
	void update(const PipelineStageId& selected, bool bDirect, int directCameraIndex, bool bCameraWizardActive);

	// Draws the window and the body update chose. The camera body is the
	// frame's only one: on a frame it is not drawn here, the preview's draw
	// list is cleared, and the caller clears it on a frame this is not called.
	void draw(bool* pOpen, const Frame& frame);

	const StageViewContent& getContent() const { return m_content; }
	eStageViewMode getMode() const { return m_mode; }
	// Decides again on the last update's inputs, so the content follows at once
	void setMode(eStageViewMode mode);

	// The automation reply: "camera all", "camera <index>" or "scene <stage
	// id or none>", then " mode follow|cameras|scene", then " wizard" while
	// a wizard holds the view
	std::string describe() const;
	static const char* getModeName(eStageViewMode mode);
	static bool parseMode(const std::string& text, eStageViewMode& outMode);

private:
	// The content for the mode and the last update's inputs
	void decide();
	void drawHeader();
	std::string makeTitle() const;

	VideoPreviewPanel* m_previewPanel;
	Scene3dPanel* m_scenePanel;
	eStageViewMode m_mode= eStageViewMode::Follow;
	PipelineStageId m_selected;
	bool m_bDirect= false;
	int m_directCameraIndex= -1;
	bool m_bCameraWizardActive= false;
	StageViewContent m_content;
	// Whether the first-appearance docking has been looked at this run
	bool m_bDockChecked= false;
};
