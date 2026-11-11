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

// How the Retarget view lays out the source pose and the avatar: the one
// over the other, or each in its own half of the view turning together
enum class eRetargetLayout : int
{
	Overlay,
	SideBySide,
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
// direct solve's chains and the gizmos go with the stage's avatar layer, and
// so do the IK goals. The Source pose stage draws the source pose alone and
// the Retarget stage the source pose with the avatar, each in place of the
// fused skeleton.
SceneLayers makeStageSceneLayers(const PipelineStageId& topLevel, bool bShowAvatar, bool bShowPerCameraSkeletons);

// What a stage is shown with when the view follows it: the cameras for
// Capture and its sub-stages (a lane's stage its own camera alone) and for
// the Direct solver and its sub-stages (the direct camera in a direct
// project), and the scene with the top-level stage's layers for everything
// else and for nothing selected. A camera index the view has no camera for
// shows every camera. Never a wizard's content.
StageViewContent decideStageViewContent(const PipelineStageId& stage, bool bDirect, int directCameraIndex,
										int cameraCount);

// The central view: one window that shows the camera view or the 3D scene
// for the stage selected in the Pipeline strip, with a header row naming
// what it shows, the mode override and the scene's camera choice. The two
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

	// Draws the window and the body update chose. Its camera body is the
	// only one the wizards draw on: on a frame it is not drawn here, the
	// preview's draw list is cleared, and the caller clears it on a frame
	// this is not called.
	void draw(bool* pOpen, const Frame& frame);

	const StageViewContent& getContent() const { return m_content; }
	eStageViewMode getMode() const { return m_mode; }
	// Decides again on the last update's inputs, so the content follows at once
	void setMode(eStageViewMode mode);

	// The Retarget view's layout, not persisted. It applies while the view
	// shows the Retarget stage's scene, where a click on the source pose
	// also selects the chain under it.
	eRetargetLayout getRetargetLayout() const { return m_retargetLayout; }
	void setRetargetLayout(eRetargetLayout layout) { m_retargetLayout= layout; }
	bool isRetargetView() const;

	// The automation reply: "camera all", "camera <index>" or "scene <stage
	// id or none>", then " mode follow|cameras|scene", then on a scene
	// " pov free|<index>" (the camera the scene is looked through, the word
	// apart from the camera view's "camera"), then on the Retarget view
	// " layout overlay|sidebyside", then " wizard" while a wizard holds the
	// view
	std::string describe() const;
	static const char* getModeName(eStageViewMode mode);
	static bool parseMode(const std::string& text, eStageViewMode& outMode);
	// The scene's camera choice as automation names it: "free" for -1, else
	// the camera's index from 0
	static std::string describePov(int cameraIndex);
	static bool parsePov(const std::string& text, int& outCameraIndex);
	static const char* getPovAvailabilityName(eScenePovAvailability availability);
	static const char* getRetargetLayoutName(eRetargetLayout layout);
	static bool parseRetargetLayout(const std::string& text, eRetargetLayout& outLayout);

private:
	// The content for the mode and the last update's inputs
	void decide();
	void drawHeader();
	// The scene's camera choice: the free camera and every configured camera
	void drawPovCombo();
	static const char* getPovUnavailableKey(eScenePovAvailability availability);
	std::string makeTitle() const;

	VideoPreviewPanel* m_previewPanel;
	Scene3dPanel* m_scenePanel;
	eStageViewMode m_mode= eStageViewMode::Follow;
	eRetargetLayout m_retargetLayout= eRetargetLayout::Overlay;
	PipelineStageId m_selected;
	bool m_bDirect= false;
	int m_directCameraIndex= -1;
	bool m_bCameraWizardActive= false;
	StageViewContent m_content;
	// Whether the first-appearance docking has been looked at this run
	bool m_bDockChecked= false;
};
