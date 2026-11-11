#pragma once

#include <memory>
#include <string>
#include <vector>

#include "AudioPanel.h" // MicrophoneReadout
#include "AvatarRetarget.h"
#include "PipelineModel.h"
#include "DirectRetarget.h"
#include "RetargetStage.h"
#include "Scene3dPanel.h" // SceneGizmos
#include "SettingsPanels.h" // TrackingPanelState
#include "VisionThread.h"    // VisionPreviewFrame

class App;
class AutomationServer;
class AvatarFaceTab;
class AvatarMouthReadout;
class AvatarPanel;
class CalibrationPanel;
class DevicePanel;
class FaceExprEditorWindow;
class MainMenuScreen;
class PerformancePanel;
class PinnedStageWindows;
class PipelinePanel;
class Scene3dPanel;
class SetupFlow;
class StageContent;
class StageView;
class StageInspector;
class TimelinePanel;
class VideoPreviewPanel;
class WizardHost;
enum class eWizardKind;

// Owns the ImGui layout: dockspace, menu bar, all panels and the wizard host.
// Created after ImGui/GL are initialized.
class MainWindow
{
public:
	explicit MainWindow(App* app);
	~MainWindow();

	// Per-frame UI update (inside an active ImGui frame)
	void update(float deltaSeconds);

	// Reopens each configured camera's device/mode (called when a project is
	// activated)
	void tryRestoreVideoDeviceFromConfig();

	// Re-syncs the device panel's cached device list and per-camera state
	// with the config and the open devices
	void refreshDevicePanelState();

	// Registers the command namespaces the window's panels own (the
	// expression editor's face namespace), once the server exists
	void registerAutomation(AutomationServer* server);

	// The window's copies of the vision thread's latest output (the thread's
	// fetch is one-shot, so other readers on the main thread read these)
	const TrackingFrameResult& getLatestFused() const { return m_latestFused; }
	// The direct pipeline's pose and solve state for that result, an invalid
	// pose and an empty state when the position pipeline posed it
	const AvatarPose& getLatestDirectPose() const
	{
		return m_latestRetarget.bDirectState ? m_latestRetarget.pose : m_noDirectPose;
	}
	const DirectRetarget::SolveState& getLatestDirectState() const { return m_latestRetarget.directState; }

private:
	// The MainMenu app state: draws the startup menu and dispatches its
	// actions to App
	void drawMainMenu();
	void drawDockspaceAndMenuBar();
	// Manual wizard launches (menu, panels). Refused while a wizard or the
	// guided setup flow is running.
	void launchWizard(eWizardKind kind, int cameraIndex= 0);
	// Global hotkeys, active anywhere in the tracking UI: fires the action
	// bound to any key pressed this frame (the table is in the source file)
	void handleHotkeys();
	// The diagnostic dump: at once, or after a settle countdown so the user
	// can get into the pose it captures. Both land in performDiagnosticDump
	// after the avatar is posed for the frame, so the avatar's render and
	// the vision thread's state describe the same pose.
	void dumpDiagnostics();
	void dumpDiagnosticsDelayed();
	void performDiagnosticDump(const std::vector<SceneCameraView>& cameras,
							   const std::vector<const TrackingFrameResult*>& perCameraResults);
	void toggleRecording();
	// Restores one camera's persisted device by path, then by friendly name
	bool restoreCameraDevice(int cameraIndex);

	App* m_app;

	std::unique_ptr<MainMenuScreen> m_mainMenuScreen;
	std::unique_ptr<VideoPreviewPanel> m_videoPreviewPanel;
	std::unique_ptr<Scene3dPanel> m_scene3dPanel;
	std::unique_ptr<DevicePanel> m_devicePanel;
	std::unique_ptr<CalibrationPanel> m_calibrationPanel;
	std::unique_ptr<WizardHost> m_wizardHost;
	std::unique_ptr<TimelinePanel> m_timelinePanel;
	// The Retarget stage's sections, which the Inspector hosts, and the
	// preview pose and elbow gizmo toggles the scene reads
	std::unique_ptr<AvatarPanel> m_avatarPanel;
	// The expression editor the Profile stage's Edit buttons open, updated
	// after the Inspector on the same shown frame
	std::unique_ptr<FaceExprEditorWindow> m_faceExprEditor;
	// The Profile stage's face profile editor and mouth readout, and the
	// Microphone stage's sections, which the Inspector hosts
	std::unique_ptr<AvatarFaceTab> m_faceTab;
	std::unique_ptr<AudioPanel> m_audioPanel;
	std::unique_ptr<AvatarMouthReadout> m_mouthReadout;
	std::unique_ptr<PipelinePanel> m_pipelinePanel;
	std::unique_ptr<StageInspector> m_stageInspector;
	// The central view over the preview and scene panels, following the
	// strip's selection
	std::unique_ptr<StageView> m_stageView;
	// The stages pinned into windows of their own, each its view and its
	// inspector on the shown frame
	std::unique_ptr<PinnedStageWindows> m_pinnedWindows;
	// The frame loop's timings and their recent history, always live
	std::unique_ptr<PerformancePanel> m_performancePanel;
	// The stage sections the Inspector hosts, drawn from the device,
	// calibration, audio and avatar panels, the tracking and OSC sections and
	// the face profile editor
	std::unique_ptr<StageContent> m_stageContent;
	// Guided new-project setup chain; needs the wizards above, so it is
	// constructed last (in the constructor body)
	std::unique_ptr<SetupFlow> m_setupFlow;

	// Latest per-camera previews + the fused result (kept between updates so
	// the UI still has data when no new frame arrived this tick)
	std::vector<VisionPreviewFrame> m_latestPreviews;
	TrackingFrameResult m_latestFused;
	// The fusion diagnostics behind m_latestFused
	FusionDiagnostics m_latestFusionDiagnostics;
	// What the vision thread's retarget stage made of that result: the pose
	// the stream carries, the source pose after the loss stage, and in direct
	// mode the solve behind the pose
	RetargetResult m_latestRetarget;
	AvatarPose m_noDirectPose;
	// An immediate dump requested this frame, taken after the avatar update
	bool m_bDumpPending= false;

	// The frame every view and readout shows this UI frame: the Timeline's
	// replayed frame while its replay view is on, the live output otherwise.
	// Computed once per update, after the Timeline has advanced. The wizards,
	// the camera overlays, the dump and the hold-still test read the live
	// members above, never this.
	struct ShownFrame
	{
		bool bReplay= false;
		const TrackingFrameResult* fused= nullptr;
		// One entry per camera, null where a camera has no result
		std::vector<const TrackingFrameResult*> perCamera;
		// The config the frame was produced under: the recording's snapshot
		// in replay, the project's live
		const AppConfig* config= nullptr;
	};
	ShownFrame m_shown;
	// What fusion reported for the shown frame, refilled with it. The
	// Fusion stage's sections read it in place.
	FusionReadout m_fusionReadout;
	// What the wrist IMUs reported for the shown frame, refilled with it. The
	// IMU stage's sections read it in place.
	ImuReadout m_imuReadout;
	// What the phone stream and the microphone reported for the shown frame,
	// refilled with it. The Phone and Microphone stages' sections read them
	// in place.
	FaceReadout m_faceReadout;
	MicrophoneReadout m_microphoneReadout;
	// What the OSC stream reported for the shown frame, refilled with it. The
	// Output stage's sections read it in place.
	OscReadout m_oscReadout;
	void updateShownFrame(const std::vector<const TrackingFrameResult*>& livePerCamera);
	void updateFusionReadout();
	void updateImuReadout();
	void updateFaceReadouts();
	void updateOscReadout();
	// The direct solve behind the shown avatar pose (its state null when the
	// shown frame is not a direct-mode frame), refilled when the avatar is
	// posed for the shown frame. The pipeline's status rules and the Direct
	// solver stages' sections read it, the sections in place.
	DirectReadout m_directReadout;
	void updateDirectReadout(const DirectRetarget::SolveState* state);
	// The source pose behind the shown avatar pose (null when the shown frame
	// has none), refilled when the avatar is posed for the shown frame. The
	// Source pose and Delivery rules stages' sections read it in place.
	SourcePoseReadout m_sourcePoseReadout;
	void updateSourcePoseReadout(const RetargetResult* result);
	// The scene's source pose layer on the same result, its head drawn at the
	// head the frame measured (the position pipeline's head is a world
	// frame) or at the posed avatar's head (the direct pipeline's is a
	// rotation alone); null for either draws no head
	void updateSceneSourcePose(const RetargetResult* result, const TrackingFrameResult::HeadPose* measuredHead,
							   const glm::vec3* posedHeadWorld);

	// What the pipeline's status rules read, gathered once per update after
	// the avatar is posed for the shown frame
	PipelineStatusInputs m_pipelineInputs;
	void gatherPipelineInputs();
	bool handlePipelineAutomation(const std::vector<std::string>& args, std::vector<std::string>& outLines,
								  std::string& outError);
	// pipeline fx: a chain's value-or-function setting, its compile state,
	// and what the shown frame evaluated it to; with a form, sets it
	bool handleFxAutomation(const std::vector<std::string>& args, std::vector<std::string>& outLines,
							std::string& outError);

	TrackingPanelState m_trackingPanelState;

	// The pose the scene shows. Live it is the vision thread's, the one the
	// stream carries. Over a replay and for the Rest and Demo preview poses
	// the window's own retarget stage solves it, a replay through the
	// playhead, which plays forward steps and pre-rolls a scrub.
	RetargetStage m_displayStage;
	RetargetResult m_displayResult;
	RetargetPlayhead m_replayPlayhead;
	AvatarPose m_avatarPose;
	// Which feed the scene last showed (live, replay, or a preview pose), so
	// a switch starts the window's stage over
	int m_avatarPoseFeed= -1;
	// Poses the scene's avatar and fills the elbow hint gizmos from it
	void updateDisplayAvatar(SceneGizmos& outGizmos);
	// The camera the scene can look through: the direct solve's placement
	// of its camera when one is shown (directState), else camera 0's
	// calibration; and the solve's chains drawn beside the avatar
	void updateSceneCameraMatch(const DirectRetarget::SolveState* directState, const AvatarSkeleton* skeleton);
	// The window's stage brought to the Timeline's current frame: a
	// direct-mode recording's frames through the direct pipeline under the
	// recorded config (a frame it cannot solve through the position
	// pipeline), any other through the position pipeline with the
	// recording's lengths and loss rules. A frame already solved runs the
	// target solve alone, so a rig edit shows. Returns the direct solve state
	// when the direct pipeline posed it.
	const DirectRetarget::SolveState* updateReplayRetarget(const class AppConfig& liveConfig,
														   const AvatarRigSettings& rig,
														   const std::shared_ptr<const AvatarSkeleton>& skeleton);
	// Writes a dragged elbow hint back into the rig settings
	void applyGizmoDrags(const SceneGizmos& gizmos);

	// Panel visibility, the View menu's toggles and the close box of a panel's
	// window. Not persisted: every panel is shown again on the next launch.
	bool m_bShowStageViewPanel= true;
	bool m_bShowTrackingPanel= true;
	bool m_bShowTimelinePanel= true;
	bool m_bShowLogPanel= true;
	bool m_bShowSettingsPanel= true;
	bool m_bShowPipelinePanel= true;
	bool m_bShowInspectorPanel= true;
	bool m_bShowPerformancePanel= true;
	bool m_bDockLayoutInitialized= false;
	// View > Reset Layout: rebuild the default dock layout on the next frame
	bool m_bResetDockLayout= false;
	// Frames until the freshly built layout's front tabs are chosen, once
	// every docked window has been submitted
	int m_dockFrontTabsCountdown= 0;
};
