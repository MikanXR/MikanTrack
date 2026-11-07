#pragma once

#include <memory>
#include <vector>

#include "AvatarRetarget.h"
#include "DirectRetarget.h"
#include "Scene3dPanel.h" // SceneGizmos
#include "SettingsPanels.h" // TrackingPanelState
#include "VisionThread.h"    // VisionPreviewFrame

class App;
class AudioPanel;
class AvatarPanel;
class CalibrationPanel;
class DevicePanel;
class MainMenuScreen;
class Scene3dPanel;
class SetupFlow;
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

	// The window's copies of the vision thread's latest output (the thread's
	// fetch is one-shot, so other readers on the main thread read these)
	const TrackingFrameResult& getLatestFused() const { return m_latestFused; }
	const AvatarPose& getLatestDirectPose() const { return m_latestDirectPose; }
	const DirectRetarget::SolveState& getLatestDirectState() const { return m_latestDirectState; }

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
	std::unique_ptr<AvatarPanel> m_avatarPanel;
	std::unique_ptr<AudioPanel> m_audioPanel;
	// Guided new-project setup chain; needs the wizards above, so it is
	// constructed last (in the constructor body)
	std::unique_ptr<SetupFlow> m_setupFlow;

	// Latest per-camera previews + the fused result (kept between updates so
	// the UI still has data when no new frame arrived this tick)
	std::vector<VisionPreviewFrame> m_latestPreviews;
	TrackingFrameResult m_latestFused;
	// The direct retarget's pose for that result and the solve behind it,
	// valid only in direct mode
	AvatarPose m_latestDirectPose;
	DirectRetarget::SolveState m_latestDirectState;
	// An immediate dump requested this frame, taken after the avatar update
	bool m_bDumpPending= false;

	TrackingPanelState m_trackingPanelState;

	// The display-side avatar retarget, run on whatever fused result the 3D
	// scene shows (live, replay, or a preview pose); the OSC streamer runs its own on the
	// vision thread over the resolved poses
	AvatarRetarget m_avatarRetarget;
	// The replay twin of the vision thread's direct retarget, solving a
	// direct-mode recording's camera frames under its recorded config
	DirectRetarget m_directRetarget;
	AvatarPose m_avatarPose;
	// Which feed the display retarget last solved (live, replay, or a
	// preview pose), so a switch resets its root follow
	int m_avatarPoseFeed= -1;
	// Poses the scene's avatar and fills the elbow hint gizmos from it
	void updateDisplayAvatar(SceneGizmos& outGizmos);
	// The camera the scene can look through: the direct solve's placement
	// of its camera when one is shown (directState), else camera 0's
	// calibration; and the solve's chains drawn beside the avatar
	void updateSceneCameraMatch(const DirectRetarget::SolveState* directState, const AvatarSkeleton* skeleton);
	// A direct-mode recording's frame through the window's direct retarget
	// into m_avatarPose; false when the recording is not direct mode or the
	// frame cannot be solved
	bool solveReplayDirectPose(const class AppConfig& recordedConfig, const AvatarSkeleton& skeleton,
							   const AvatarRigSettings& rig);
	// Writes a dragged elbow hint back into the rig settings
	void applyGizmoDrags(const SceneGizmos& gizmos);

	bool m_bShowLogPanel= true;
	bool m_bShowSettingsPanel= true;
	bool m_bShowAvatarPanel= true;
	bool m_bShowAudioPanel= true;
	bool m_bDockLayoutInitialized= false;
};
