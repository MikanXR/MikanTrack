#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "AvatarRig.h"
#include "FaceExprCompiler.h"
#include "FaceProfile.h"

struct SDL_Window;
typedef void* SDL_GLContext;

class AppConfig;
struct AvatarModel;
class AvatarSkeleton;
class GlobalSettings;
class AudioService;
class FaceProfileBinding;
class FaceProfileLibrary;
class FaceService;
class ImuService;
class LocalizationManager;
class ProjectManager;
class VideoCaptureSystem;
class VisionThread;
class MainWindow;

class AutomationServer;

class App
{
public:
	// MainMenu shows only the startup menu (no project loaded, cameras closed,
	// vision thread stopped); Project is the full tracking UI
	enum class eAppState
	{
		MainMenu,
		Project,
	};

	App();
	~App();

	static App* getInstance() { return m_instance; }

	int exec(int argc, char** argv);
	void requestShutdown() { m_bShutdownRequested= true; }

	eAppState getAppState() const { return m_appState; }
	// Stops tracking, loads the project file into the config in place, and
	// enters the tracking UI
	bool activateProject(const std::filesystem::path& projectFile);
	// Creates a default-config project folder, loads it, and flags the setup
	// flow to start on entering the tracking UI
	bool activateNewProject(const std::string& projectName);
	// Saves and closes out the current project and returns to the main menu
	void returnToMainMenu();
	// Cancelled guided setup: deletes the just-created project from disk and
	// returns to the main menu, with Resume pointing back at whatever was
	// loaded before it
	void discardNewProjectAndReturnToMenu();
	// One-shot flag raised by activateNewProject, consumed by MainWindow to
	// start the guided setup flow
	bool consumeStartSetupFlowFlag();

	// Project actions requested from inside the tracking UI. They are applied
	// at the top of the next tick rather than when clicked: switching or
	// closing a project mid-frame would mutate the config under panels that
	// already sized their per-camera state. A later request in the same frame
	// replaces an earlier one.
	void requestLoadProject(const std::filesystem::path& projectFile);
	void requestCloseProject();
	void requestDiscardNewProject();

	// Applies a config camera-count change: restarts the vision thread (its
	// context list is fixed while running) and resizes the capture slots
	void applyCameraCountChange();

	// The loaded VRM avatar, if any. Loaded on the main thread (the renderer
	// builds GL resources from it) when a project activates and from the
	// Avatar panel; cleared with the project. The generation counter advances
	// on every change so the renderer knows when to re-upload.
	bool loadAvatar(const std::filesystem::path& path);
	void clearAvatar();
	std::shared_ptr<const AvatarModel> getAvatarModel() const { return m_avatarModel; }
	std::shared_ptr<const AvatarSkeleton> getAvatarSkeleton() const { return m_avatarSkeleton; }

	// The loaded avatar's rig settings, read from its sidecar on load. An
	// edit applies at once: a mapping change reapplies the overrides and
	// rebuilds the skeleton, every change goes to the vision thread, and the
	// sidecar is saved after a short cooldown.
	const AvatarRigSettings& getAvatarRig() const { return m_avatarRig; }
	void setAvatarRig(const AvatarRigSettings& rig);
	// What applying the rig and building the skeleton reported
	const std::vector<std::string>& getAvatarRigWarnings() const { return m_avatarRigWarnings; }
	uint32_t getAvatarGeneration() const { return m_avatarGeneration; }
	const std::string& getAvatarLoadError() const { return m_avatarLoadError; }
	// Resolves AvatarConfig::modelPath: absolute as is, else the project
	// folder, else the exe folder
	std::filesystem::path resolveAvatarPath(const std::string& modelPath) const;

	// -- Face profiles ------------------------------------------------------
	// The global profile library. The loaded avatar's rig sidecar assigns
	// profiles from it by id and names the active one.
	FaceProfileLibrary* getFaceProfileLibrary() { return m_faceProfiles.get(); }
	// The active profile bound to the loaded avatar, the one the vision thread
	// evaluates. It runs the built-in Auto profile with no avatar, with no
	// active id, with an id the library lacks, and for a profile that has
	// never compiled; after an edit that does not compile it keeps the last
	// good binding.
	const std::shared_ptr<const FaceProfileBinding>& getFaceBinding() const { return m_faceBinding; }
	// The active profile as last edited (Auto's generated text for Auto),
	// which may not compile, and its compile errors
	const FaceProfile& getActiveFaceProfile() const { return m_activeFaceProfile; }
	const std::vector<FaceExprCompileError>& getFaceProfileErrors() const { return m_faceProfileErrors; }
	bool isActiveFaceProfileAuto() const;
	// The rig names an active id the library does not hold
	bool isActiveFaceProfileMissing() const;
	// Makes an assigned profile active; "" for Auto. Needs an avatar.
	void setActiveFaceProfile(const std::string& id);
	void assignFaceProfile(const std::string& id);
	// Unassigning the active profile returns the avatar to Auto
	void unassignFaceProfile(const std::string& id);
	// Applies an edit of the active profile: compiles it, binds it when it
	// compiles, and saves it to the library after a short settle. Auto is
	// read-only and ignores edits.
	void editActiveFaceProfile(const FaceProfile& profile);
	// A new profile (empty, or a copy of source under a fresh id), saved,
	// assigned to the avatar and made active; its id, or "" on failure
	std::string createFaceProfile(const std::string& name);
	std::string duplicateFaceProfile(const FaceProfile& source, const std::string& name);
	bool renameFaceProfile(const std::string& id, const std::string& name);
	// Deletes the profile's file. An avatar that assigned it lists it as
	// missing and runs Auto while it is active.
	bool deleteFaceProfile(const std::string& id);

	AppConfig* getConfig() { return m_config.get(); }
	GlobalSettings* getGlobalSettings() { return m_globalSettings.get(); }
	LocalizationManager* getLocalization() { return m_localization.get(); }
	ProjectManager* getProjectManager() { return m_projectManager.get(); }
	VideoCaptureSystem* getVideoCapture() { return m_videoCapture.get(); }
	ImuService* getImuService() { return m_imuService.get(); }
	FaceService* getFaceService() { return m_faceService.get(); }
	// Pushes AppConfig::face to the face service (rebinds its socket when the
	// port or enable flag changed)
	void applyFaceConfig();
	AudioService* getAudioService() { return m_audioService.get(); }
	// Pushes AppConfig::audio to the audio service (reopens the device or
	// recreates the backend when those changed)
	void applyAudioConfig();
	VisionThread* getVisionThread() { return m_visionThread.get(); }
	SDL_Window* getSdlWindow() { return m_sdlWindow; }
	AutomationServer* getAutomationServer() { return m_automationServer.get(); }

protected:
	// The app's own flags: --automation-server and --automation-port=<n>. The
	// self-test flags are consumed in main.cpp before exec is reached.
	void parseCommandLine(int argc, char** argv);
	bool startup();
	void shutdown();
	void tick(float deltaSeconds);
	// Renders and presents the torn-out panels' OS windows after the main
	// window's draw data, leaving the main GL context current again
	void renderPlatformWindows();

private:
	static App* m_instance;

	std::unique_ptr<AppConfig> m_config;
	std::unique_ptr<GlobalSettings> m_globalSettings;
	std::unique_ptr<LocalizationManager> m_localization;
	std::unique_ptr<ProjectManager> m_projectManager;
	std::unique_ptr<VideoCaptureSystem> m_videoCapture;
	// Started with a project and stopped with it; the vision thread is its
	// only caller while running, so it survives a vision thread restart
	std::unique_ptr<ImuService> m_imuService;
	// Started with a project like the IMU service. Its update() is driven by
	// the vision thread.
	std::unique_ptr<FaceService> m_faceService;
	// Started with a project like the face service. Owns the audio thread;
	// the vision thread reads its published mouth sample.
	std::unique_ptr<AudioService> m_audioService;
	std::unique_ptr<VisionThread> m_visionThread;
	std::unique_ptr<MainWindow> m_mainWindow;
	// The loopback command channel, created at startup and listening only when
	// the command line asked for it
	std::unique_ptr<AutomationServer> m_automationServer;
	bool m_bAutomationServerRequested= false;
	uint16_t m_automationPort= 0;

	// Loads the configured avatar, if any, for the active project
	void loadConfiguredAvatar();
	// Applies m_avatarRig's mapping to the model and rebuilds the skeleton
	void rebuildAvatarSkeleton();
	// Writes the loaded avatar's elbow hint, over its upper-arm length, into
	// the body config's elbow seed. Only on load and rig edits: the seed
	// stays in the config when the avatar is cleared.
	void applyAvatarElbowSeed();
	void updateAvatarRigAutoSave(float deltaSeconds);
	void saveAvatarRigIfDirty();
	void markAvatarRigDirty();
	// Rebuilds the active profile's binding from the library and the rig and
	// hands it to the vision thread
	void rebuildFaceBinding();
	// The avatar's identity and the face binding to the vision thread
	void pushAvatarIdentity();
	void updateFaceProfileAutoSave(float deltaSeconds);
	void saveFaceProfileIfDirty();
	// Mutable here only so the rig's mapping can be reapplied on the main
	// thread; everything else sees it const (the vision thread never does)
	std::shared_ptr<AvatarModel> m_avatarModel;
	std::shared_ptr<const AvatarSkeleton> m_avatarSkeleton;
	AvatarRigSettings m_avatarRig;
	std::vector<std::string> m_avatarRigLoadWarnings;
	std::vector<std::string> m_avatarRigWarnings;
	bool m_bAvatarRigDirty= false;
	float m_avatarRigSecondsSinceDirty= 0.f;
	uint32_t m_avatarGeneration= 0;
	std::string m_avatarLoadError;

	std::unique_ptr<FaceProfileLibrary> m_faceProfiles;
	std::shared_ptr<const FaceProfileBinding> m_faceBinding;
	FaceProfile m_activeFaceProfile;
	std::vector<FaceExprCompileError> m_faceProfileErrors;
	bool m_bFaceProfileDirty= false;
	float m_faceProfileSecondsSinceDirty= 0.f;

	SDL_Window* m_sdlWindow= nullptr;
	SDL_GLContext m_glContext= nullptr;

	eAppState m_appState= eAppState::MainMenu;
	bool m_bStartSetupFlowOnEnter= false;

	enum class ePendingProjectAction
	{
		None,
		Load,
		Close,
		Discard,
	};
	// Services the pending project action at the top of tick()
	void applyPendingProjectAction();
	ePendingProjectAction m_pendingProjectAction= ePendingProjectAction::None;
	std::filesystem::path m_pendingLoadProjectFile;
	// The last-project pointer as it was before activateNewProject overwrote
	// it, so a discarded new project can hand Resume back to its predecessor
	std::filesystem::path m_lastProjectPathBeforeNew;
	bool m_bShutdownRequested= false;
};
