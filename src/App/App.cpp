#include "App.h"

#include <algorithm>
#include <chrono>
#include <filesystem>

#include "GL/glew.h"
#include "SDL.h"

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl2.h"

#include "AppConfig.h"
#include "AutomationServer.h"
#include "AvatarSkeleton.h"
#include "FrameTimer.h"
#include "AudioService.h"
#include "FaceAutoProfile.h"
#include "FaceProfileBinding.h"
#include "FaceProfileLibrary.h"
#include "FaceService.h"
#include "GlobalSettings.h"
#include "GuiViewports.h"
#include "ImuService.h"
#include "ImGuiTheme.h"
#include "LocalizationManager.h"
#include "Logger.h"
#include "LogPanel.h"
#include "MainWindow.h"
#include "PathUtils.h"
#include "ProjectManager.h"
#include "VideoCaptureSystem.h"
#include "VisionThread.h"
#include "VrmLoader.h"

App* App::m_instance= nullptr;

// Rig edits arrive per frame while a control is dragged; the sidecar is
// written once they settle
static constexpr float k_avatarRigAutoSaveSeconds= 3.f;
// Face profile edits arrive per keystroke; the file is written once they
// settle, the same way
static constexpr float k_faceProfileAutoSaveSeconds= 3.f;

App::App()
{
	m_instance= this;
}

App::~App()
{
	m_instance= nullptr;
}

void App::parseCommandLine(int argc, char** argv)
{
	m_automationPort= AutomationServer::k_defaultPort;

	for (int argIndex= 1; argIndex < argc; ++argIndex)
	{
		const std::string arg(argv[argIndex]);
		const std::string portPrefix= "--automation-port=";

		if (arg == "--automation-server")
		{
			m_bAutomationServerRequested= true;
		}
		else if (arg.rfind(portPrefix, 0) == 0)
		{
			// A port implies the server, since a port with nothing listening is a trap
			const int port= atoi(arg.c_str() + portPrefix.size());
			if (port > 0 && port <= 65535)
			{
				m_automationPort= (uint16_t)port;
				m_bAutomationServerRequested= true;
			}
			else
			{
				MIKAN_LOG_ERROR("App::parseCommandLine") << "Ignoring invalid automation port '" << arg << "'";
			}
		}
	}
}

int App::exec(int argc, char** argv)
{
	parseCommandLine(argc, argv);

	if (!startup())
	{
		shutdown();
		return 1;
	}

	auto lastFrameTime= std::chrono::steady_clock::now();
	while (!m_bShutdownRequested)
	{
		FrameTimer frameTimer(11); // ~90Hz UI cap

		const auto now= std::chrono::steady_clock::now();
		float deltaSeconds= std::chrono::duration<float>(now - lastFrameTime).count();
		lastFrameTime= now;
		deltaSeconds= std::min(deltaSeconds, 0.1f);

		tick(deltaSeconds);

		frameTimer.waitForNextFrame();
	}

	shutdown();
	return 0;
}

bool App::startup()
{
	// Logging first
	LoggerSettings loggerSettings= {};
	loggerSettings.min_log_level= LogSeverityLevel::debug;
	loggerSettings.log_filename= "MikanTrack.log";
	loggerSettings.enable_console= false;
	loggerSettings.log_callback= &LogPanel::logCallback;
	log_init(loggerSettings);

	MIKAN_LOG_INFO("App::startup") << "MikanTrack starting up";

	m_config= std::make_unique<AppConfig>();
	m_globalSettings= std::make_unique<GlobalSettings>();
	m_projectManager= std::make_unique<ProjectManager>(m_globalSettings.get(), m_config.get());

	m_projectManager->migrateLegacyConfigIfNeeded();
	m_globalSettings->load();

	// Before any UI exists: everything drawn from here on fetches its strings
	// through this
	m_localization= std::make_unique<LocalizationManager>();
	m_localization->startup(m_globalSettings.get());

	// SDL + GL context
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
	{
		MIKAN_LOG_ERROR("App::startup") << "SDL_Init failed: " << SDL_GetError();
		return false;
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

	m_sdlWindow= SDL_CreateWindow(
		"MikanTrack",
		SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		1600, 900,
		SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	if (m_sdlWindow == nullptr)
	{
		MIKAN_LOG_ERROR("App::startup") << "SDL_CreateWindow failed: " << SDL_GetError();
		return false;
	}

	m_glContext= SDL_GL_CreateContext(m_sdlWindow);
	if (m_glContext == nullptr)
	{
		MIKAN_LOG_ERROR("App::startup") << "SDL_GL_CreateContext failed: " << SDL_GetError();
		return false;
	}
	SDL_GL_MakeCurrent(m_sdlWindow, m_glContext);
	SDL_GL_SetSwapInterval(1); // vsync

	glewExperimental= GL_TRUE;
	const GLenum glewResult= glewInit();
	if (glewResult != GLEW_OK)
	{
		MIKAN_LOG_ERROR("App::startup") << "glewInit failed: " << (const char*)glewGetErrorString(glewResult);
		return false;
	}
	MIKAN_LOG_INFO("App::startup") << "OpenGL " << (const char*)glGetString(GL_VERSION)
		<< " on " << (const char*)glGetString(GL_RENDERER);

	// ImGui (docking)
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io= ImGui::GetIO();
	io.ConfigFlags|= ImGuiConfigFlags_DockingEnable;
	io.ConfigFlags|= ImGuiConfigFlags_NavEnableKeyboard;
	ImGui::StyleColorsDark();
	ImGuiTheme::applyStyle();
	ImGuiTheme::loadFonts();
	ImGui_ImplSDL2_InitForOpenGL(m_sdlWindow, m_glContext);
	ImGui_ImplOpenGL3_Init("#version 330 core");

	// Subsystems
	m_videoCapture= std::make_unique<VideoCaptureSystem>();
	if (!m_videoCapture->startup())
	{
		MIKAN_LOG_ERROR("App::startup") << "VideoCaptureSystem startup failed";
		return false;
	}
	m_videoCapture->setCameraSlotCount(m_config->cameraCount());

	// Constructed but not started: the app boots into the main menu, and the
	// vision thread, cameras, and IMU service only spin up when a project is
	// activated
	m_imuService= std::make_unique<ImuService>();
	m_faceService= std::make_unique<FaceService>();
	m_audioService= std::make_unique<AudioService>();
	m_visionThread= std::make_unique<VisionThread>(m_videoCapture.get(), m_imuService.get(), m_faceService.get(),
												   m_audioService.get(), m_config.get());

	m_faceProfiles= std::make_unique<FaceProfileLibrary>();
	m_faceProfiles->refresh();
	rebuildFaceBinding();

	m_mainWindow= std::make_unique<MainWindow>(this);

	// The command channel registers its namespaces whether or not it listens
	m_automationServer= std::make_unique<AutomationServer>();
	m_automationServer->initialize(this);
	if (m_bAutomationServerRequested && !m_automationServer->startListener(m_automationPort))
	{
		// Tolerated: the app runs on, it just cannot be driven externally
		MIKAN_LOG_ERROR("App::startup") << "Automation server failed to bind port " << m_automationPort;
	}

	return true;
}

void App::applyFaceConfig()
{
	if (m_faceService == nullptr || m_config == nullptr)
		return;

	FaceServiceConfig faceConfig;
	faceConfig.enabled= m_config->face.enabled;
	faceConfig.port= (uint16_t)std::clamp(m_config->face.port, 1, 65535);
	faceConfig.phoneAddress= m_config->face.phoneAddress;
	m_faceService->setConfig(faceConfig);
}

void App::applyAudioConfig()
{
	if (m_audioService == nullptr || m_config == nullptr)
		return;

	AudioServiceConfig audioConfig;
	audioConfig.enabled= m_config->audio.enabled;
	audioConfig.deviceName= m_config->audio.deviceName;
	audioConfig.gainDb= m_config->audio.gainDb;
	audioConfig.gateDbfs= m_config->audio.gateDbfs;
	audioConfig.model= m_config->audio.model;
	audioConfig.profilePath= m_config->audio.profilePath;
	audioConfig.smoothingMs= m_config->audio.smoothingMs[(size_t)m_config->audio.model];
	audioConfig.engineSmoothing= m_config->audio.ovrSmoothing;
	m_audioService->setConfig(audioConfig);
}

bool App::activateProject(const std::filesystem::path& projectFile)
{
	// Stop the config's consumers before loadProject mutates it in place
	m_visionThread->stop();
	for (int cameraIndex= 0; cameraIndex < (int)m_videoCapture->getCameraSlotCount(); ++cameraIndex)
		m_videoCapture->closeDevice(cameraIndex);

	if (!m_projectManager->loadProject(projectFile))
	{
		m_imuService->shutdown();
		m_faceService->shutdown();
		m_audioService->shutdown();
		m_appState= eAppState::MainMenu;
		return false;
	}

	m_videoCapture->setCameraSlotCount(m_config->cameraCount());
	m_imuService->startup();
	m_faceService->startup();
	applyFaceConfig();
	m_audioService->startup();
	applyAudioConfig();
	m_visionThread->start();
	m_mainWindow->tryRestoreVideoDeviceFromConfig();
	loadConfiguredAvatar();
	m_appState= eAppState::Project;
	return true;
}

bool App::activateNewProject(const std::string& projectName)
{
	// Remembered so a cancelled setup can hand Resume back to this project
	// (loadProject below overwrites the last-project pointer)
	m_lastProjectPathBeforeNew= m_globalSettings->lastProjectPath;

	// New projects start from the defaults, not whatever project was loaded
	// before; stop the config's consumer before resetting it in place
	m_visionThread->stop();
	*m_config= AppConfig();

	std::filesystem::path projectFile;
	if (!m_projectManager->createProject(projectName, projectFile))
		return false;
	if (!activateProject(projectFile))
		return false;

	m_bStartSetupFlowOnEnter= true;
	return true;
}

void App::returnToMainMenu()
{
	m_config->save();
	clearAvatar();
	m_visionThread->stop();
	m_imuService->shutdown();
	m_faceService->shutdown();
	m_audioService->shutdown();
	for (int cameraIndex= 0; cameraIndex < (int)m_videoCapture->getCameraSlotCount(); ++cameraIndex)
		m_videoCapture->closeDevice(cameraIndex);
	m_appState= eAppState::MainMenu;
}

void App::discardNewProjectAndReturnToMenu()
{
	const std::filesystem::path projectFile= m_config->getProjectFilePath();

	clearAvatar();
	m_visionThread->stop();
	m_imuService->shutdown();
	m_faceService->shutdown();
	m_audioService->shutdown();
	for (int cameraIndex= 0; cameraIndex < (int)m_videoCapture->getCameraSlotCount(); ++cameraIndex)
		m_videoCapture->closeDevice(cameraIndex);

	// Clear the in-place config (and its remembered path) before touching the
	// folder, so the autosave cannot recreate the deleted file
	*m_config= AppConfig();
	PathUtils::setProjectDirectory(std::filesystem::path());

	ProjectManager::deleteProject(projectFile);

	if (!m_lastProjectPathBeforeNew.empty() &&
		std::filesystem::is_regular_file(m_lastProjectPathBeforeNew))
	{
		m_globalSettings->lastProjectPath= m_lastProjectPathBeforeNew;
	}
	else
	{
		m_globalSettings->lastProjectPath.clear();
	}
	m_globalSettings->save();

	m_appState= eAppState::MainMenu;
}

std::filesystem::path App::resolveAvatarPath(const std::string& modelPath) const
{
	const std::filesystem::path path= PathUtils::utf8ToPath(modelPath);
	if (path.empty() || path.is_absolute())
		return path;

	const std::filesystem::path projectDirectory= m_config->getProjectDirectory();
	if (!projectDirectory.empty() && std::filesystem::is_regular_file(projectDirectory / path))
		return projectDirectory / path;
	return PathUtils::getModulePath() / path;
}

bool App::loadAvatar(const std::filesystem::path& path)
{
	const VrmLoader::LoadResult result= VrmLoader::loadFile(path);
	if (result.model == nullptr)
	{
		m_avatarLoadError= result.error;
		return false;
	}

	// The previous avatar's pending rig edits belong to its own sidecar
	saveAvatarRigIfDirty();
	saveFaceProfileIfDirty();

	m_avatarModel= result.model;
	m_avatarRigLoadWarnings.clear();
	m_avatarRig= loadAvatarRig(path, &m_avatarRigLoadWarnings);
	m_bAvatarRigDirty= false;
	rebuildAvatarSkeleton();
	m_avatarLoadError.clear();
	++m_avatarGeneration;
	m_visionThread->setAvatarSkeleton(m_avatarSkeleton, m_avatarRig);
	rebuildFaceBinding();
	applyAvatarElbowSeed();
	return true;
}

void App::pushAvatarIdentity()
{
	VisionThread::AvatarIdentity identity;
	if (m_avatarModel != nullptr)
	{
		identity.path= m_avatarModel->sourcePath;
		identity.title= m_avatarModel->meta.name;
		identity.sha256= m_avatarModel->sha256Hex;
	}
	identity.faceBinding= m_faceBinding;
	m_visionThread->setAvatarIdentity(identity);
}

void App::rebuildAvatarSkeleton()
{
	m_avatarRigWarnings= m_avatarRigLoadWarnings;
	applyRigToModel(m_avatarRig, *m_avatarModel, m_avatarRigWarnings);
	m_avatarSkeleton= std::make_shared<const AvatarSkeleton>(*m_avatarModel);
	m_avatarRigWarnings.insert(m_avatarRigWarnings.end(), m_avatarSkeleton->getWarnings().begin(),
							   m_avatarSkeleton->getWarnings().end());
	for (const std::string& warning : m_avatarRigWarnings)
		MIKAN_LOG_WARNING("App::rebuildAvatarSkeleton") << warning;
}

void App::applyAvatarElbowSeed()
{
	// The solver reads the seed from the config rather than the avatar, so a
	// recording carries it and replays with it. Written only on change, so
	// reloading the same avatar leaves the config clean.
	bool bChanged= false;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		glm::vec3 seed(0.f);
		if (m_avatarSkeleton != nullptr)
		{
			const float upperArmLength= m_avatarSkeleton->getUpperArmLength((eHandSide)sideIndex);
			if (upperArmLength > 1e-3f)
				seed= m_avatarRig.sides[sideIndex].elbowHintOffset / upperArmLength;
		}
		if (m_config->body.elbowSeedPerUpperArm[sideIndex] != seed)
		{
			m_config->body.elbowSeedPerUpperArm[sideIndex]= seed;
			bChanged= true;
		}
	}
	if (bChanged)
		m_config->markDirty();
}

void App::setAvatarRig(const AvatarRigSettings& rig)
{
	if (m_avatarModel == nullptr)
		return;

	// Only the mapping changes the skeleton. Keeping the same skeleton for
	// every other edit also keeps the streamer's root follow running.
	const bool bMappingChanged= !rig.sameMapping(m_avatarRig);
	m_avatarRig= rig;
	if (bMappingChanged)
		rebuildAvatarSkeleton();
	m_visionThread->setAvatarSkeleton(m_avatarSkeleton, m_avatarRig);
	applyAvatarElbowSeed();

	markAvatarRigDirty();
}

void App::markAvatarRigDirty()
{
	if (!m_bAvatarRigDirty)
		m_avatarRigSecondsSinceDirty= 0.f;
	m_bAvatarRigDirty= true;
}

void App::updateAvatarRigAutoSave(float deltaSeconds)
{
	if (!m_bAvatarRigDirty)
		return;
	m_avatarRigSecondsSinceDirty+= deltaSeconds;
	if (m_avatarRigSecondsSinceDirty >= k_avatarRigAutoSaveSeconds)
		saveAvatarRigIfDirty();
}

void App::saveAvatarRigIfDirty()
{
	if (!m_bAvatarRigDirty)
		return;
	m_bAvatarRigDirty= false;
	m_avatarRigSecondsSinceDirty= 0.f;
	if (m_avatarModel != nullptr && !m_avatarModel->sourcePath.empty())
		saveAvatarRig(PathUtils::utf8ToPath(m_avatarModel->sourcePath), m_avatarRig);
}

void App::clearAvatar()
{
	saveAvatarRigIfDirty();
	saveFaceProfileIfDirty();
	if (m_avatarModel == nullptr && m_avatarLoadError.empty())
		return;
	m_avatarModel= nullptr;
	m_avatarSkeleton= nullptr;
	m_avatarRig= AvatarRigSettings();
	m_avatarRigLoadWarnings.clear();
	m_avatarRigWarnings.clear();
	m_avatarLoadError.clear();
	++m_avatarGeneration;
	m_visionThread->setAvatarSkeleton(nullptr, m_avatarRig);
	rebuildFaceBinding();
	// The elbow seed outlives its avatar on purpose: closing a project clears
	// the avatar after the save, and zeroing the seed there would dirty the
	// saved config every time, while the seed itself is only a prior
}

bool App::isActiveFaceProfileAuto() const
{
	return isAutoFaceProfileId(m_activeFaceProfile.id);
}

bool App::isActiveFaceProfileMissing() const
{
	const std::string& id= m_avatarRig.activeFaceProfile;
	return m_avatarModel != nullptr && !id.empty() && !isAutoFaceProfileId(id) && m_faceProfiles->find(id) == nullptr;
}

void App::rebuildFaceBinding()
{
	// Pending edits belong to the profile that was active
	saveFaceProfileIfDirty();
	m_faceProfileErrors.clear();
	const AvatarModel* model= m_avatarModel.get();
	const std::string id= model != nullptr ? m_avatarRig.activeFaceProfile : std::string();
	const FaceProfile* stored= !id.empty() && !isAutoFaceProfileId(id) ? m_faceProfiles->find(id) : nullptr;

	std::shared_ptr<const FaceProfileBinding> binding;
	if (stored != nullptr)
	{
		m_activeFaceProfile= *stored;
		binding= compileFaceBinding(m_activeFaceProfile, model, &m_faceProfileErrors);
	}
	else
	{
		m_activeFaceProfile= makeAutoFaceProfile(model);
	}
	// A profile that never compiled runs Auto, with its errors shown
	if (binding == nullptr)
		binding= makeAutoFaceBinding(model);
	m_faceBinding= binding;
	if (m_visionThread != nullptr)
		pushAvatarIdentity();
}

void App::setActiveFaceProfile(const std::string& id)
{
	const std::string activeId= isAutoFaceProfileId(id) ? std::string() : id;
	if (m_avatarModel == nullptr || m_avatarRig.activeFaceProfile == activeId)
		return;
	saveFaceProfileIfDirty();
	m_avatarRig.activeFaceProfile= activeId;
	markAvatarRigDirty();
	rebuildFaceBinding();
}

void App::assignFaceProfile(const std::string& id)
{
	if (m_avatarModel == nullptr || id.empty() || isAutoFaceProfileId(id))
		return;
	std::vector<std::string>& assigned= m_avatarRig.faceProfiles;
	if (std::find(assigned.begin(), assigned.end(), id) != assigned.end())
		return;
	assigned.push_back(id);
	markAvatarRigDirty();
}

void App::unassignFaceProfile(const std::string& id)
{
	if (m_avatarModel == nullptr)
		return;
	std::vector<std::string>& assigned= m_avatarRig.faceProfiles;
	assigned.erase(std::remove(assigned.begin(), assigned.end(), id), assigned.end());
	markAvatarRigDirty();
	if (m_avatarRig.activeFaceProfile == id)
		setActiveFaceProfile(std::string());
}

void App::editActiveFaceProfile(const FaceProfile& profile)
{
	if (isActiveFaceProfileAuto() || profile.id != m_activeFaceProfile.id)
		return;
	m_activeFaceProfile= profile;
	std::shared_ptr<const FaceProfileBinding> binding=
		compileFaceBinding(m_activeFaceProfile, m_avatarModel.get(), &m_faceProfileErrors);
	// An edit that does not compile leaves the last good binding running
	if (binding != nullptr)
	{
		m_faceBinding= binding;
		pushAvatarIdentity();
	}
	if (!m_bFaceProfileDirty)
		m_faceProfileSecondsSinceDirty= 0.f;
	m_bFaceProfileDirty= true;
}

std::string App::createFaceProfile(const std::string& name)
{
	if (m_avatarModel == nullptr)
		return std::string();
	saveFaceProfileIfDirty();
	const std::string id= m_faceProfiles->create(name);
	if (!id.empty())
	{
		assignFaceProfile(id);
		setActiveFaceProfile(id);
	}
	return id;
}

std::string App::duplicateFaceProfile(const FaceProfile& source, const std::string& name)
{
	if (m_avatarModel == nullptr)
		return std::string();
	saveFaceProfileIfDirty();
	const std::string id= m_faceProfiles->duplicate(source, name);
	if (!id.empty())
	{
		assignFaceProfile(id);
		setActiveFaceProfile(id);
	}
	return id;
}

bool App::renameFaceProfile(const std::string& id, const std::string& name)
{
	saveFaceProfileIfDirty();
	if (!m_faceProfiles->rename(id, name))
		return false;
	if (m_activeFaceProfile.id == id)
		m_activeFaceProfile.name= name;
	return true;
}

bool App::deleteFaceProfile(const std::string& id)
{
	if (m_activeFaceProfile.id == id)
		m_bFaceProfileDirty= false;
	const bool bRemoved= m_faceProfiles->remove(id);
	if (m_avatarRig.activeFaceProfile == id)
		rebuildFaceBinding();
	return bRemoved;
}

void App::updateFaceProfileAutoSave(float deltaSeconds)
{
	if (!m_bFaceProfileDirty)
		return;
	m_faceProfileSecondsSinceDirty+= deltaSeconds;
	if (m_faceProfileSecondsSinceDirty >= k_faceProfileAutoSaveSeconds)
		saveFaceProfileIfDirty();
}

void App::saveFaceProfileIfDirty()
{
	if (!m_bFaceProfileDirty)
		return;
	m_bFaceProfileDirty= false;
	m_faceProfileSecondsSinceDirty= 0.f;
	if (!isActiveFaceProfileAuto())
		m_faceProfiles->save(m_activeFaceProfile);
}

void App::loadConfiguredAvatar()
{
	clearAvatar();
	if (m_config->avatar.modelPath.empty())
		return;
	loadAvatar(resolveAvatarPath(m_config->avatar.modelPath));
}

bool App::consumeStartSetupFlowFlag()
{
	const bool bStart= m_bStartSetupFlowOnEnter;
	m_bStartSetupFlowOnEnter= false;
	return bStart;
}

void App::requestLoadProject(const std::filesystem::path& projectFile)
{
	m_pendingProjectAction= ePendingProjectAction::Load;
	m_pendingLoadProjectFile= projectFile;
}

void App::requestCloseProject()
{
	m_pendingProjectAction= ePendingProjectAction::Close;
}

void App::requestDiscardNewProject()
{
	m_pendingProjectAction= ePendingProjectAction::Discard;
}

void App::applyPendingProjectAction()
{
	const ePendingProjectAction action= m_pendingProjectAction;
	m_pendingProjectAction= ePendingProjectAction::None;

	switch (action)
	{
		case ePendingProjectAction::Load:
			activateProject(m_pendingLoadProjectFile);
			m_pendingLoadProjectFile.clear();
			break;
		case ePendingProjectAction::Close:
			returnToMainMenu();
			break;
		case ePendingProjectAction::Discard:
			discardNewProjectAndReturnToMenu();
			break;
		default:
			break;
	}
}

void App::applyCameraCountChange()
{
	MIKAN_LOG_INFO("App::applyCameraCountChange")
		<< "Applying camera count change: " << m_config->cameraCount() << " cameras";

	// Order matters: stop the consumer (vision thread) before touching the
	// capture slots, then restart with the new context count
	m_visionThread->stop();
	m_videoCapture->setCameraSlotCount(m_config->cameraCount());
	m_visionThread->start();
	m_config->markDirty();
}

void App::shutdown()
{
	MIKAN_LOG_INFO("App::shutdown") << "Shutting down";

	if (m_config != nullptr)
		m_config->save();
	saveAvatarRigIfDirty();
	saveFaceProfileIfDirty();

	if (m_visionThread != nullptr)
	{
		m_visionThread->stop();
		m_visionThread= nullptr;
	}

	if (m_imuService != nullptr)
	{
		m_imuService->shutdown();
		m_imuService= nullptr;
	}

	if (m_faceService != nullptr)
	{
		m_faceService->shutdown();
		m_faceService= nullptr;
	}

	if (m_audioService != nullptr)
	{
		m_audioService->shutdown();
		m_audioService= nullptr;
	}

	if (m_videoCapture != nullptr)
	{
		m_videoCapture->shutdown();
		m_videoCapture= nullptr;
	}

	// Before the window goes, so a quit reply still in flight flushes first
	m_automationServer= nullptr;
	m_mainWindow= nullptr;

	if (m_localization != nullptr)
		m_localization->shutdown();

	if (ImGui::GetCurrentContext() != nullptr)
	{
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplSDL2_Shutdown();
		ImGui::DestroyContext();
	}

	if (m_glContext != nullptr)
	{
		SDL_GL_DeleteContext(m_glContext);
		m_glContext= nullptr;
	}
	if (m_sdlWindow != nullptr)
	{
		SDL_DestroyWindow(m_sdlWindow);
		m_sdlWindow= nullptr;
	}
	SDL_Quit();

	log_dispose();
}

void App::tick(float deltaSeconds)
{
	// Project switches land here, between frames, never under a UI that is
	// mid-draw
	applyPendingProjectAction();

	// SDL events
	SDL_Event event;
	while (SDL_PollEvent(&event))
	{
		ImGui_ImplSDL2_ProcessEvent(&event);
		if (event.type == SDL_QUIT)
			m_bShutdownRequested= true;
		if (event.type == SDL_WINDOWEVENT &&
			event.window.event == SDL_WINDOWEVENT_CLOSE &&
			event.window.windowID == SDL_GetWindowID(m_sdlWindow))
		{
			m_bShutdownRequested= true;
		}
	}

	// The automation channel's commands run here, between frames, so a project
	// action or a window move lands the way the UI's own would. An input it
	// injects sits in SDL's queue for the next tick's event loop.
	m_automationServer->poll();

	// Pump the device manager (hotplug HWND lives on this thread)
	m_videoCapture->update(deltaSeconds);

	// UI frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplSDL2_NewFrame();
	ImGui::NewFrame();

	// Before any window begins this frame, so a moved window is placed by the
	// Begin that follows
	GuiViewports::applyPendingFloatRequests();

	m_mainWindow->update(deltaSeconds);

	ImGui::Render();

	int drawableWidth= 0, drawableHeight= 0;
	SDL_GL_GetDrawableSize(m_sdlWindow, &drawableWidth, &drawableHeight);
	glViewport(0, 0, drawableWidth, drawableHeight);
	glClearColor(0.08f, 0.08f, 0.09f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	// A pending screenshot reads the back buffer before it presents. No panel
	// renders outside the main window yet, so a torn-out viewport capture
	// answers with an error rather than leaving its client waiting.
	m_automationServer->servicePendingWindowCapture(drawableWidth, drawableHeight);
	m_automationServer->failUnservicedViewportCapture();

	SDL_GL_SwapWindow(m_sdlWindow);

	m_config->updateAutoSave(deltaSeconds);
	updateAvatarRigAutoSave(deltaSeconds);
	updateFaceProfileAutoSave(deltaSeconds);
}
