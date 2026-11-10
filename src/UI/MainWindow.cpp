#include "MainWindow.h"

#include <algorithm>
#include <filesystem>

#include "imgui.h"
#include "imgui_internal.h" // dock builder

#include "glm/ext/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"

#include "App.h"
#include "AppConfig.h"
#include "AudioPanel.h"
#include "AudioService.h"
#include "AutomationServer.h"
#include "AvatarFaceTab.h"
#include "AvatarMouthReadout.h"
#include "AvatarPanel.h"
#include "AvatarPreviewPoses.h"
#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "BodyPoseSolver.h"
#include "CalibrationPanel.h"
#include "DevicePanel.h"
#include "FaceExprEditorWindow.h"
#include "FaceProfileBinding.h"
#include "FaceService.h"
#include "GlobalSettings.h"
#include "HandOverlay.h"
#include "LocText.h"
#include "LogPanel.h"
#include "Logger.h"
#include "MainMenuScreen.h"
#include "PathUtils.h"
#include "PipelinePanel.h"
#include "PipelineText.h"
#include "ProjectManager.h"
#include "Scene3dPanel.h"
#include "SettingsPanels.h"
#include "SetupFlow.h"
#include "StageContent.h"
#include "StageInspector.h"
#include "SteadyClock.h"
#include "TimelinePanel.h"
#include "TrackingRecording.h"
#include "VideoModeUtils.h"
#include "VideoCaptureSystem.h"
#include "VideoPreviewPanel.h"
#include "WizardHost.h"

#include "tinyfiledialogs.h"

MainWindow::MainWindow(App* app)
	: m_app(app)
	, m_mainMenuScreen(std::make_unique<MainMenuScreen>())
	, m_videoPreviewPanel(std::make_unique<VideoPreviewPanel>())
	, m_scene3dPanel(std::make_unique<Scene3dPanel>())
	, m_devicePanel(std::make_unique<DevicePanel>(app, app->getVideoCapture(), app->getConfig()))
	, m_calibrationPanel(std::make_unique<CalibrationPanel>(app->getConfig()))
	, m_wizardHost(std::make_unique<WizardHost>(app, app->getConfig(), app->getVisionThread()))
	, m_timelinePanel(std::make_unique<TimelinePanel>())
	, m_avatarPanel(std::make_unique<AvatarPanel>(app, app->getConfig()))
	, m_faceExprEditor(std::make_unique<FaceExprEditorWindow>(app))
	, m_faceTab(std::make_unique<AvatarFaceTab>())
	, m_audioPanel(std::make_unique<AudioPanel>(app, app->getConfig()))
	, m_mouthReadout(std::make_unique<AvatarMouthReadout>())
	, m_pipelinePanel(std::make_unique<PipelinePanel>())
	, m_stageInspector(std::make_unique<StageInspector>())
	, m_stageContent(std::make_unique<StageContent>(app, m_devicePanel.get(), m_calibrationPanel.get(),
												   m_audioPanel.get(), m_avatarPanel.get(), m_faceTab.get(),
												   m_mouthReadout.get()))
{
	SetupFlow::WizardSet wizards;
	wizards.intrinsics= m_wizardHost->getIntrinsicsWizard();
	wizards.extrinsics= m_wizardHost->getExtrinsicsWizard();
	wizards.hand= m_wizardHost->getHandCalibrationWizard();
	wizards.mounting= m_wizardHost->getMountingWizard();
	wizards.body= m_wizardHost->getBodyCalibrationWizard();
	m_setupFlow= std::make_unique<SetupFlow>(app, this, m_videoPreviewPanel.get(), wizards);
}

MainWindow::~MainWindow()= default;

void MainWindow::registerAutomation(AutomationServer* server)
{
	m_faceExprEditor->registerAutomation(server);
	server->registerCommandNamespace(
		"pipeline",
		{"pipeline stages", "pipeline status <stage>", "pipeline selected", "pipeline select <stage>|none"},
		[this](const std::vector<std::string>& args, std::vector<std::string>& outLines, std::string& outError) {
			return handlePipelineAutomation(args, outLines, outError);
		});
}

bool MainWindow::restoreCameraDevice(int cameraIndex)
{
	AppConfig* config= m_app->getConfig();
	VideoCaptureSystem* videoCapture= m_app->getVideoCapture();
	const CameraProfile& profile= config->camera(cameraIndex);

	if (profile.video.devicePath.empty())
		return false;

	// Match by path first, then by friendly name
	std::string pathToOpen;
	const size_t deviceCount= videoCapture->getDeviceCount();
	for (size_t i= 0; i < deviceCount; ++i)
	{
		std::string path, name;
		if (!videoCapture->getDevicePath(i, path) || !videoCapture->getDeviceFriendlyName(i, name))
			continue;
		if (path == profile.video.devicePath)
		{
			pathToOpen= path;
			break;
		}
		if (pathToOpen.empty() && !profile.video.deviceName.empty() && name == profile.video.deviceName)
			pathToOpen= path;
	}

	if (pathToOpen.empty())
	{
		MIKAN_LOG_INFO("MainWindow") << "Camera " << cameraIndex
			<< ": persisted video device not found: " << profile.video.deviceName;
		return false;
	}

	if (!videoCapture->openDeviceByPath(cameraIndex, pathToOpen))
		return false;

	if (!profile.video.modeName.empty())
		videoCapture->setVideoModeByName(cameraIndex, profile.video.modeName);

	videoCapture->startStream(cameraIndex);

	MIKAN_LOG_INFO("MainWindow") << "Camera " << cameraIndex << " restored: "
		<< videoCapture->getCurrentDeviceFriendlyName(cameraIndex)
		<< " @ " << videoCapture->getCurrentVideoModeName(cameraIndex);
	return true;
}

void MainWindow::tryRestoreVideoDeviceFromConfig()
{
	AppConfig* config= m_app->getConfig();
	for (int cameraIndex= 0; cameraIndex < (int)config->cameraCount(); ++cameraIndex)
		restoreCameraDevice(cameraIndex);

	refreshDevicePanelState();
}

void MainWindow::refreshDevicePanelState()
{
	m_devicePanel->refreshDeviceList();
	for (int cameraIndex= 0; cameraIndex < (int)m_app->getConfig()->cameraCount(); ++cameraIndex)
		m_devicePanel->refreshModeOptions(cameraIndex);
}

void MainWindow::launchWizard(eWizardKind kind, int cameraIndex)
{
	// The host refuses while one of its wizards runs; the flow launches its
	// own wizards and must not be interrupted by a manual one
	if (!m_setupFlow->isActive())
		m_wizardHost->launch(kind, cameraIndex);
}

void MainWindow::drawMainMenu()
{
	GlobalSettings* globalSettings= m_app->getGlobalSettings();
	// Existence-checked so a project deleted on disk does not present a
	// Resume that can only fail
	const bool bHasLastProject= globalSettings->hasLastProjectPath() &&
								std::filesystem::is_regular_file(globalSettings->lastProjectPath);
	const std::string lastProjectName= bHasLastProject
		? globalSettings->lastProjectPath.parent_path().filename().string()
		: std::string();

	const MainMenuScreen::Action action= m_mainMenuScreen->draw(bHasLastProject, lastProjectName);
	if (action.type != MainMenuScreen::Action::Type::None)
		m_mainMenuScreen->setStatusMessage(std::string());

	switch (action.type)
	{
		case MainMenuScreen::Action::Type::Resume:
			if (!m_app->activateProject(globalSettings->lastProjectPath))
				m_mainMenuScreen->setStatusMessage(locText("errors.loadLastProjectFailed"));
			break;
		case MainMenuScreen::Action::Type::NewProject:
			if (!m_app->activateNewProject(action.projectName))
				m_mainMenuScreen->setStatusMessage(locText("errors.createProjectFailed"));
			break;
		case MainMenuScreen::Action::Type::LoadProject:
			if (!m_app->activateProject(action.projectFile))
				m_mainMenuScreen->setStatusMessage(locText("errors.loadProjectFailed"));
			break;
		case MainMenuScreen::Action::Type::Exit:
			m_app->requestShutdown();
			break;
		default:
			break;
	}
}

void MainWindow::drawDockspaceAndMenuBar()
{
	const ImGuiViewport* viewport= ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(viewport->WorkPos);
	ImGui::SetNextWindowSize(viewport->WorkSize);
	ImGui::SetNextWindowViewport(viewport->ID);

	const ImGuiWindowFlags hostFlags=
		ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
		ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground;

	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
	ImGui::Begin("MikanTrackDockHost", nullptr, hostFlags);
	ImGui::PopStyleVar();

	// The id names the layout's generation: an imgui.ini saved by a build
	// whose default layout had no Pipeline strip holds no node under this id,
	// so that install gets the new default once
	const ImGuiID dockspaceId= ImGui::GetID("MikanTrackDockspace2");

	// Default layout on first run, and again on View > Reset Layout
	if (m_bResetDockLayout || (!m_bDockLayoutInitialized && ImGui::DockBuilderGetNode(dockspaceId) == nullptr))
	{
		m_bResetDockLayout= false;
		ImGui::DockBuilderRemoveNode(dockspaceId);
		ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace);
		ImGui::DockBuilderSetNodeSize(dockspaceId, viewport->WorkSize);

		ImGuiID centerId= dockspaceId;
		// The Pipeline strip across the top: two rows of stages and three
		// camera lanes
		const float stripRatio= std::clamp(250.f / std::max(viewport->WorkSize.y, 1.f), 0.15f, 0.4f);
		const ImGuiID topId= ImGui::DockBuilderSplitNode(centerId, ImGuiDir_Up, stripRatio, nullptr, &centerId);
		const ImGuiID leftId= ImGui::DockBuilderSplitNode(centerId, ImGuiDir_Left, 0.20f, nullptr, &centerId);
		const ImGuiID rightId= ImGui::DockBuilderSplitNode(centerId, ImGuiDir_Right, 0.25f, nullptr, &centerId);
		const ImGuiID bottomId= ImGui::DockBuilderSplitNode(centerId, ImGuiDir_Down, 0.22f, nullptr, &centerId);

		ImGui::DockBuilderDockWindow(locWindowTitle("windows.pipeline"), topId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.videoPreview"), centerId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.scene3d"), centerId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.tracking"), leftId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.settings"), leftId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.stageInspector"), rightId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.oscOutput"), rightId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.log"), bottomId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.timeline"), bottomId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.performance"), bottomId);
		ImGui::DockBuilderFinish(dockspaceId);
		// A node builds its tab bar a frame or two after its windows dock,
		// then the front tabs are chosen one per frame
		m_dockFrontTabsCountdown= 4 + 4;
	}
	m_bDockLayoutInitialized= true;

	ImGui::DockSpace(dockspaceId, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);

	// A dock node fronts the tab of the window that holds the focus, and
	// only one window holds it per frame, so the default layout fronts its
	// tabs one frame apart once the windows exist, ending on the preview
	if (m_dockFrontTabsCountdown > 0)
	{
		static const char* k_frontTabs[]= {"windows.videoPreview", "windows.stageInspector", "windows.tracking",
										   "windows.timeline"};
		--m_dockFrontTabsCountdown;
		if (m_dockFrontTabsCountdown < IM_ARRAYSIZE(k_frontTabs))
			ImGui::SetWindowFocus(locWindowTitle(k_frontTabs[m_dockFrontTabsCountdown]));
	}

	if (ImGui::BeginMenuBar())
	{
		if (ImGui::BeginMenu(locLabel("mainWindow.fileMenu")))
		{
			const bool bWizardActive= m_wizardHost->isAnyActive() || m_setupFlow->isActive();
			if (ImGui::MenuItem(locLabel("mainWindow.saveProject")))
				m_app->getConfig()->save();
			if (ImGui::MenuItem(locLabel("mainWindow.loadProject"), nullptr, false, !bWizardActive))
			{
				const std::string defaultDir=
					(ProjectManager::getProjectsRootDirectory() / "").string();
				const char* filterPatterns[]= {"project.json"};
				const char* selectedPath= tinyfd_openFileDialog(
					locText("mainWindow.loadProjectDialogTitle"), defaultDir.c_str(), 1, filterPatterns,
					locText("mainWindow.loadProjectDialogFilterDescription"), 0);
				if (selectedPath != nullptr)
					m_app->requestLoadProject(PathUtils::utf8ToPath(selectedPath));
			}
			if (ImGui::MenuItem(locLabel("mainWindow.closeProject"), nullptr, false, !bWizardActive))
				m_app->requestCloseProject();
			ImGui::Separator();
			// The Retarget stage's load controls run the same two actions
			if (ImGui::MenuItem(locLabel("mainWindow.loadAvatar")))
				m_avatarPanel->loadAvatarFromDialog();
			const bool bAvatarLoaded= m_app->getAvatarModel() != nullptr;
			if (ImGui::MenuItem(locLabel("mainWindow.unloadAvatar"), nullptr, false, bAvatarLoaded))
				m_avatarPanel->unloadAvatar();
			ImGui::Separator();
			if (ImGui::MenuItem(locLabel("mainWindow.quit"), "Alt+F4"))
				m_app->requestShutdown();
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu(locLabel("mainWindow.calibrationMenu")))
		{
			const bool bWizardActive= m_wizardHost->isAnyActive() || m_setupFlow->isActive();
			AppConfig* config= m_app->getConfig();

			for (int cameraIndex= 0; cameraIndex < (int)config->cameraCount(); ++cameraIndex)
			{
				ImGui::PushID(cameraIndex);
				const std::string label= locFormat("mainWindow.cameraIntrinsicsFmt", cameraIndex + 1);
				if (ImGui::MenuItem(label.c_str(), nullptr, false, !bWizardActive))
					launchWizard(eWizardKind::Intrinsics, cameraIndex);
				ImGui::PopID();
			}

			// One shared session: every camera calibrates against the same
			// marker placement (separate sessions = disagreeing world frames)
			bool bAllIntrinsics= true;
			for (size_t i= 0; i < config->cameraCount(); ++i)
				bAllIntrinsics&= config->camera(i).intrinsics.present;
			if (ImGui::MenuItem(locLabel("mainWindow.extrinsicsAllCameras"), nullptr, false,
								!bWizardActive && bAllIntrinsics))
				launchWizard(eWizardKind::Extrinsics);
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu(locLabel("mainWindow.viewMenu")))
		{
			ImGui::MenuItem(locLabel("mainWindow.viewPipelinePanel"), nullptr, &m_bShowPipelinePanel);
			ImGui::MenuItem(locLabel("mainWindow.viewInspectorPanel"), nullptr, &m_bShowInspectorPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewVideoPreviewPanel"), nullptr, &m_bShowVideoPreviewPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewScene3dPanel"), nullptr, &m_bShowScene3dPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewTrackingPanel"), nullptr, &m_bShowTrackingPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewOscPanel"), nullptr, &m_bShowOscPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewTimelinePanel"), nullptr, &m_bShowTimelinePanel);
			ImGui::MenuItem(locLabel("mainWindow.viewSettingsPanel"), nullptr, &m_bShowSettingsPanel);
			ImGui::MenuItem(locLabel("mainWindow.logPanel"), nullptr, &m_bShowLogPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewPerformancePanel"), nullptr, &m_bShowPerformancePanel);
			ImGui::Separator();
			// Every panel shown again in the default arrangement
			if (ImGui::MenuItem(locLabel("mainWindow.resetLayout")))
			{
				m_bResetDockLayout= true;
				m_bShowVideoPreviewPanel= m_bShowScene3dPanel= m_bShowTrackingPanel= true;
				m_bShowOscPanel= m_bShowTimelinePanel= m_bShowLogPanel= true;
				m_bShowSettingsPanel= true;
				m_bShowPipelinePanel= m_bShowInspectorPanel= m_bShowPerformancePanel= true;
			}
			ImGui::EndMenu();
		}
		ImGui::EndMenuBar();
	}

	ImGui::End();
}

// -- Global hotkeys ----------------------------------------------------------

void MainWindow::handleHotkeys()
{
	// Every key the tracking UI binds, in one place. Repeat is off: each
	// press fires once.
	struct HotkeyBinding
	{
		ImGuiKey key;
		bool bShift;
		void (MainWindow::*action)();
	};
	static const HotkeyBinding k_hotkeys[]= {
		{ImGuiKey_F9, false, &MainWindow::dumpDiagnostics},
		{ImGuiKey_F9, true, &MainWindow::dumpDiagnosticsDelayed},
		{ImGuiKey_F10, false, &MainWindow::toggleRecording},
	};

	const bool bShift= ImGui::GetIO().KeyShift;
	for (const HotkeyBinding& binding : k_hotkeys)
	{
		if (binding.bShift == bShift && ImGui::IsKeyPressed(binding.key, false))
			(this->*binding.action)();
	}
}

// Diagnostic dump: state history + camera frames + config, and in direct
// mode the posed avatar
void MainWindow::dumpDiagnostics()
{
	m_bDumpPending= true;
}

void MainWindow::dumpDiagnosticsDelayed()
{
	constexpr float kDumpCountdownSeconds= 3.f;
	if (!m_trackingPanelState.dumpCountdown.isRunning())
		m_trackingPanelState.dumpCountdown.start(kDumpCountdownSeconds);
}

void MainWindow::performDiagnosticDump(const std::vector<SceneCameraView>& cameras,
									   const std::vector<const TrackingFrameResult*>& perCameraResults)
{
	// The avatar as the scene sees it from the tracking camera, rendered
	// here where the GL context is, saved by the vision thread beside the
	// frame it should match
	cv::Mat avatarRender;
	if (m_scene3dPanel->hasCameraMatch())
		m_scene3dPanel->captureFromCamera(m_latestFused, cameras, perCameraResults, avatarRender);
	m_app->getVisionThread()->requestDiagnosticDump(m_app->getConfig()->makeDumpDirectoryPath(), avatarRender);
}

// Toggles the tracking recording (deterministic replay input capture;
// starting resets transient tracking state - brief blip)
void MainWindow::toggleRecording()
{
	VisionThread* visionThread= m_app->getVisionThread();
	if (visionThread->isRecording())
		visionThread->requestRecordingStop();
	else
		visionThread->requestRecordingStart(m_app->getConfig()->makeRecordingFilePath());
}

// -- Frame ------------------------------------------------------------------

void MainWindow::update(float deltaSeconds)
{
	if (m_app->getAppState() == App::eAppState::MainMenu)
	{
		drawMainMenu();
		return;
	}

	// A freshly created project starts the guided setup chain
	if (m_app->consumeStartSetupFlowFlag())
		m_setupFlow->begin();

	AppConfig* config= m_app->getConfig();
	VisionThread* visionThread= m_app->getVisionThread();
	const int cameraCount= (int)config->cameraCount();

	// Keep per-camera containers in sync with the config
	m_latestPreviews.resize(cameraCount);
	m_videoPreviewPanel->setCameraCount(cameraCount);

	// Pull the newest per-camera previews + the fused result
	for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
	{
		VisionPreviewFrame freshFrame;
		if (visionThread->fetchPreviewFrame(cameraIndex, freshFrame))
		{
			m_latestPreviews[cameraIndex]= std::move(freshFrame);
			m_videoPreviewPanel->setFrame(cameraIndex, m_latestPreviews[cameraIndex].bgr);
		}
	}
	visionThread->fetchFusedResult(m_latestFused, &m_latestDirectPose, &m_latestDirectState, &m_latestFusionDiagnostics);

	// Newest per-camera results, for the preview overlays and the 3D scene
	std::vector<const TrackingFrameResult*> perCameraResults;
	for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
	{
		perCameraResults.push_back(
			m_latestPreviews[cameraIndex].valid ? &m_latestPreviews[cameraIndex].result : nullptr);
	}

	// The Timeline advances before anything reads the shown frame, whether
	// or not its panel is drawn this frame
	m_timelinePanel->setFaceBinding(m_app->getFaceBinding());
	m_timelinePanel->update(deltaSeconds);
	updateShownFrame(perCameraResults);

	handleHotkeys();

	drawDockspaceAndMenuBar();

	const bool bWizardActive= m_wizardHost->isAnyActive() || m_setupFlow->isActive();

	// Panels
	TrackingSectionContext trackingContext;
	trackingContext.config= config;
	trackingContext.visionThread= visionThread;
	trackingContext.videoCapture= m_app->getVideoCapture();
	trackingContext.previewPanel= m_videoPreviewPanel.get();
	trackingContext.scene3dPanel= m_scene3dPanel.get();
	trackingContext.state= &m_trackingPanelState;
	trackingContext.latestPreviews= &m_latestPreviews;
	trackingContext.fusedResult= &m_latestFused;
	trackingContext.fusion= &m_fusionReadout;
	trackingContext.imu= &m_imuReadout;
	// The tracking panel's countdowns, captures and readouts run whether or
	// not any of its sections is drawn this frame
	SettingsPanels::updateTrackingState(trackingContext, deltaSeconds);
	OscSectionContext oscContext;
	oscContext.config= config;
	oscContext.visionThread= visionThread;
	oscContext.fusedResult= &m_latestFused;
	// The stage sections the Inspector draws read this frame's contexts
	m_stageContent->beginFrame(trackingContext, oscContext, m_fusionReadout, m_imuReadout, m_faceReadout,
							   m_microphoneReadout, m_faceExprEditor->getTarget(), bWizardActive);
	if (m_bShowTrackingPanel)
		SettingsPanels::drawTrackingPanel(&m_bShowTrackingPanel, trackingContext);
	if (m_trackingPanelState.bDumpRequested)
	{
		m_trackingPanelState.bDumpRequested= false;
		dumpDiagnostics();
	}
	if (m_trackingPanelState.bDelayedDumpRequested)
	{
		m_trackingPanelState.bDelayedDumpRequested= false;
		dumpDiagnosticsDelayed();
	}
	if (m_bShowOscPanel)
		SettingsPanels::drawOscPanel(&m_bShowOscPanel, oscContext);
	if (m_bShowSettingsPanel)
		SettingsPanels::drawAppSettingsPanel(&m_bShowSettingsPanel);
	if (m_bShowTimelinePanel)
	{
		m_timelinePanel->draw(&m_bShowTimelinePanel, config, visionThread);
		// A scrub or a view switch in the panel changes the shown frame
		updateShownFrame(perCameraResults);
	}

	// Launch requests raised by the panels this frame
	if (m_trackingPanelState.bLaunchMountingWizard)
		launchWizard(eWizardKind::Mounting);
	if (m_trackingPanelState.bLaunchBodyCalibrationWizard)
		launchWizard(eWizardKind::Body);
	if (m_trackingPanelState.bLaunchHandCalibrationWizard)
		launchWizard(eWizardKind::Hand);
	m_trackingPanelState.bLaunchMountingWizard= false;
	m_trackingPanelState.bLaunchBodyCalibrationWizard= false;
	m_trackingPanelState.bLaunchHandCalibrationWizard= false;

	// Guided setup chain: draws its prompt modals on top of the panels, and
	// launches/watches the wizards updated below
	m_setupFlow->update();

	// Bring the Video Preview tab forward when a camera calibration wizard
	// starts: the pattern feed is the wizard's whole UI, and the 3D Scene tab
	// may be the selected one in the shared center dock, or the panel closed
	if (m_wizardHost->consumeCameraWizardStarted())
	{
		m_bShowVideoPreviewPanel= true;
		ImGui::SetWindowFocus(locWindowTitle("windows.videoPreview"));
	}

	// Pin the preview highlight to the intrinsics wizard's camera while it is
	// active (the extrinsics wizard uses ALL cameras, so no pinning there)
	const int intrinsicsCamera= m_wizardHost->getIntrinsicsCameraIndex();
	if (intrinsicsCamera >= 0)
		m_videoPreviewPanel->setActiveCamera(intrinsicsCamera);

	// Central: side-by-side previews with per-camera overlays
	if (!m_bShowVideoPreviewPanel)
	{
		m_videoPreviewPanel->clearLastDrawList();
	}
	else
	{
		std::vector<const char*> executionProviders;
		std::vector<ForearmOverlay> forearmOverlays;
		for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
		{
			executionProviders.push_back(visionThread->getActiveExecutionProvider(cameraIndex));
			// The fused (world-space) forearm projected back into this camera:
			// the UI is what holds both the fused result and every camera's
			// calibration
			forearmOverlays.push_back(HandOverlay::makeForearmOverlay(
				config->camera(cameraIndex), m_latestFused, config->body.forearmLengthMeters));
		}
		// Direct mode: which of the direct camera's hand slots the solver took
		m_videoPreviewPanel->setHandUse(config->direct.enabled ? config->direct.cameraIndex : -1,
										{m_latestDirectState.arms[0].bHandSeen, m_latestDirectState.arms[1].bHandSeen});
		m_videoPreviewPanel->draw(&m_bShowVideoPreviewPanel, perCameraResults, executionProviders, &forearmOverlays);
	}

	// 3D scene: fused skeleton + all calibrated camera frustums. In replay
	// view the whole feed comes from the timeline's scrub position instead,
	// with the frustums built from the RECORDING's config snapshot.
	m_scene3dPanel->setForearmLength(config->body.forearmLengthMeters);
	m_scene3dPanel->setAvatar(m_app->getAvatarModel(), m_app->getAvatarSkeleton(), m_app->getAvatarGeneration());
	m_scene3dPanel->setFaceBinding(m_app->getFaceBinding());
	m_scene3dPanel->setShowAvatar(config->avatar.showInScene);
	SceneGizmos gizmos;
	updateDisplayAvatar(gizmos);
	const std::vector<SceneCameraView> sceneCameras= makeSceneCameraViews(*config);
	if (m_bShowScene3dPanel)
	{
		// The frustums come from the config the shown frame was made under
		m_scene3dPanel->draw(&m_bShowScene3dPanel, *m_shown.fused,
							 m_shown.bReplay ? m_timelinePanel->getSceneCameras() : sceneCameras, m_shown.perCamera,
							 gizmos);
	}
	applyGizmoDrags(gizmos);

	// The pipeline, once the avatar is posed for the shown frame: the strip,
	// the inspector on the strip's selection, and the frame loop's timings
	gatherPipelineInputs();
	if (m_bShowPipelinePanel)
		m_pipelinePanel->draw(&m_bShowPipelinePanel, m_pipelineInputs);
	if (m_bShowInspectorPanel)
	{
		const PipelineStageId requested= m_stageInspector->draw(&m_bShowInspectorPanel, m_pipelineInputs,
																m_pipelinePanel->getSelected(), m_stageContent.get());
		if (requested.isValid())
			m_pipelinePanel->setSelected(requested);
	}
	// After the menu bar and the Inspector, the draws that can unload the
	// avatar, so the preview pose is back to Live before the next frame's
	// scene reads it. This frame's scene was posed before the Inspector drew.
	m_avatarPanel->update();
	// The calibration wizards the Inspector's stage sections asked for
	const CalibrationPanel::DrawResult& calibrationAction= m_stageContent->getCalibrationAction();
	if (calibrationAction.bLaunchIntrinsicsWizard)
		launchWizard(eWizardKind::Intrinsics, calibrationAction.cameraIndex);
	if (calibrationAction.bLaunchExtrinsicsWizard)
		launchWizard(eWizardKind::Extrinsics);
	if (m_stageContent->getAudioAction().bLaunchAudioWizard)
		launchWizard(eWizardKind::AudioCalibration);
	// A face profile row's Edit button opens the expression editor on the
	// row. The editor runs every frame, after the Inspector so the open lands
	// this frame, and reads the frame the scene shows.
	if (m_stageContent->getFaceEditRequest().has_value())
		m_faceExprEditor->open(*m_stageContent->getFaceEditRequest());
	m_faceExprEditor->update(*m_shown.fused);
	if (m_bShowPerformancePanel)
		PerformancePanel::draw(&m_bShowPerformancePanel, *visionThread);

	// The dump, once the avatar is posed for this frame
	const bool bCountdownDone= m_trackingPanelState.dumpCountdown.update(deltaSeconds);
	if (m_bDumpPending || bCountdownDone)
	{
		m_bDumpPending= false;
		performDiagnosticDump(sceneCameras, perCameraResults);
	}

	// The active wizard, drawn last, on top
	m_wizardHost->update(deltaSeconds, m_latestPreviews, m_latestFused, m_videoPreviewPanel.get());

	if (m_bShowLogPanel)
		LogPanel::getInstance().draw(&m_bShowLogPanel);
}

void MainWindow::updateShownFrame(const std::vector<const TrackingFrameResult*>& livePerCamera)
{
	m_shown.bReplay= m_timelinePanel->isReplayViewActive();
	if (m_shown.bReplay)
	{
		m_shown.fused= &m_timelinePanel->getDisplayFused();
		m_shown.perCamera= m_timelinePanel->getPerCameraResults();
		m_shown.config= &m_timelinePanel->getRecordedConfig();
	}
	else
	{
		m_shown.fused= &m_latestFused;
		m_shown.perCamera= livePerCamera;
		m_shown.config= m_app->getConfig();
	}
	updateFusionReadout();
	updateImuReadout();
	updateFaceReadouts();
}

void MainWindow::updateFusionReadout()
{
	FusionReadout readout;
	readout.bReplay= m_shown.bReplay;
	readout.cameraCount= (int)m_shown.config->cameraCount();
	readout.fused= m_shown.fused;
	if (m_shown.bReplay)
	{
		// A take made before stage state was recorded has no readouts, but
		// the replay regenerates its diagnostics
		readout.diagnostics= m_timelinePanel->getDisplayFusionDiagnostics();
		const RecordedTelemetry* recorded= m_timelinePanel->getDisplayTelemetry();
		if (recorded != nullptr)
		{
			readout.bAvailable= true;
			readout.dominantCamera[0]= recorded->dominantCamera[0];
			readout.dominantCamera[1]= recorded->dominantCamera[1];
			for (int cameraIndex= 0; cameraIndex < FusionReadout::k_maxCameras; ++cameraIndex)
			{
				const bool bRecordedSlot= cameraIndex < RecordedTelemetry::k_maxCameras;
				for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
				{
					readout.observationConfidence[cameraIndex][sideIndex]=
						bRecordedSlot ? recorded->observationConfidence[cameraIndex][sideIndex] : -1.f;
				}
			}
			readout.autoHandScaleFactor= recorded->autoHandScaleFactor;
		}
	}
	else
	{
		const VisionThread* visionThread= m_app->getVisionThread();
		readout.bAvailable= true;
		readout.diagnostics= &m_latestFusionDiagnostics;
		readout.dominantCamera[0]= visionThread->getDominantCamera(eHandSide::Left);
		readout.dominantCamera[1]= visionThread->getDominantCamera(eHandSide::Right);
		for (int cameraIndex= 0; cameraIndex < FusionReadout::k_maxCameras; ++cameraIndex)
		{
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				readout.observationConfidence[cameraIndex][sideIndex]=
					visionThread->getObservationConfidence(cameraIndex, (eHandSide)sideIndex);
			}
		}
		readout.autoHandScaleFactor= visionThread->getAutoHandScaleFactor();
	}
	m_fusionReadout= readout;
}

void MainWindow::updateImuReadout()
{
	ImuReadout readout;
	readout.bReplay= m_shown.bReplay;
	if (m_shown.bReplay)
	{
		// A recording carries the status the device table reads, but not the
		// device name or the time since the last sample
		const RecordedTelemetry* recorded= m_timelinePanel->getDisplayTelemetry();
		if (recorded != nullptr)
		{
			readout.bAvailable= true;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const RecordedImuStatus& recordedSide= recorded->imu[sideIndex];
				ImuSideStatus& status= readout.sides[sideIndex];
				status.deviceConnected= recordedSide.deviceConnected;
				status.streaming= recordedSide.streaming;
				status.calibrated= recordedSide.calibrated;
				status.orientationValid= recordedSide.orientationValid;
				status.sampleRateHz= recordedSide.sampleRateHz;
				status.batteryLevel= recordedSide.batteryLevel;
				status.gyroBiasDegreesPerSecond.z= recordedSide.yawDriftDegPerSecond;
				status.wristAxialTwistDegrees= recordedSide.rollErrorDegrees;
			}
		}
	}
	else
	{
		const VisionThread* visionThread= m_app->getVisionThread();
		readout.bAvailable= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			readout.sides[sideIndex]= visionThread->getImuSideStatus((eHandSide)sideIndex);
	}
	m_imuReadout= readout;
}

void MainWindow::updateFaceReadouts()
{
	FaceReadout face;
	MicrophoneReadout microphone;
	face.bReplay= m_shown.bReplay;
	microphone.bReplay= m_shown.bReplay;
	if (m_shown.bReplay)
	{
		// A recording carries the phone stream's and the microphone's state,
		// but not the parser's counts, the head, the device name or the
		// backend's texts
		const RecordedTelemetry* recorded= m_timelinePanel->getDisplayTelemetry();
		if (recorded != nullptr)
		{
			face.bAvailable= true;
			face.bEnabled= recorded->face.enabled;
			face.bBound= recorded->face.bound;
			face.bStreaming= recorded->face.streaming;
			face.datagramsPerSecond= recorded->face.datagramsPerSecond;

			microphone.bAvailable= true;
			microphone.bEnabled= recorded->audio.enabled;
			microphone.bDeviceOpen= recorded->audio.deviceOpen;
			microphone.bBackendReady= recorded->audio.backendReady;
			microphone.bGateOpen= recorded->audio.gateOpen;
			microphone.levelDbfs= recorded->audio.levelDbfs;
			microphone.bPhoneOwnsMouth=
				recorded->face.enabled && recorded->face.streaming && recorded->face.faceTracked;
		}
	}
	else
	{
		const AppConfig* config= m_app->getConfig();
		face.bAvailable= true;
		// The stream's switch as the config holds it
		face.bEnabled= config->face.enabled;
		if (const FaceService* faceService= m_app->getFaceService())
		{
			const FaceServiceStatus status= faceService->getStatus();
			face.bBound= status.bound;
			face.bStreaming= status.streaming;
			face.datagramsPerSecond= status.datagramsPerSecond;
			face.bSampleValid= status.latest.valid;
			face.unknownNameCount= status.unknownNameCount;
			face.parseFailures= status.parseFailures;
			face.lastUnknownName= status.lastUnknownName;
			face.bHasHead= status.latest.hasHead;
			face.headEulerDegrees= status.latest.headEulerDegrees;

			FaceSample sample;
			microphone.bPhoneOwnsMouth= config->face.enabled && faceService->isStreaming(steadyNowMs()) &&
										 faceService->getLatestSample(sample) && sample.faceTracked;
		}

		microphone.bAvailable= true;
		AudioService* audioService= m_app->getAudioService();
		microphone.bEnabled= audioService != nullptr && config->audio.enabled;
		if (audioService != nullptr)
		{
			const AudioServiceStatus status= audioService->getStatus();
			microphone.bDeviceOpen= status.deviceOpen;
			microphone.bBackendReady= status.backendReady;
			microphone.bGateOpen= status.gateOpen;
			microphone.levelDbfs= status.levelDbfs;
			microphone.deviceName= status.deviceName;
			microphone.deviceSampleRate= status.deviceSampleRate;
			microphone.backendInfo= status.backendInfo;
			microphone.backendReason= status.backendReason;
			microphone.inferMsEma= status.inferMsEma;
			microphone.droppedBlocks= status.droppedBlocks;
		}
	}
	m_faceReadout= face;
	m_microphoneReadout= microphone;
}

void MainWindow::updateDisplayAvatar(SceneGizmos& outGizmos)
{
	m_shownDirectState= nullptr;
	const std::shared_ptr<const AvatarSkeleton> skeleton= m_app->getAvatarSkeleton();
	if (skeleton == nullptr)
	{
		m_scene3dPanel->setAvatarPose(nullptr);
		m_scene3dPanel->setAvatarFace(nullptr);
		updateSceneCameraMatch(nullptr, nullptr);
		return;
	}
	const AppConfig* config= m_app->getConfig();
	const AvatarRigSettings& rig= m_app->getAvatarRig();

	// The feed: a preview pose from the Retarget stage, else replay frames
	// (with the recording's body lengths), else live ones (with the
	// project's). Switching feeds resets the root follow so the filter never
	// bridges two timelines.
	const AvatarPanel::ePreviewPose preview= m_avatarPanel->getPreviewPose();
	const bool bReplay= m_shown.bReplay;
	const int feed= preview != AvatarPanel::ePreviewPose::Live ? 1 + (int)preview : (bReplay ? 1 : 0);
	if (feed != m_avatarPoseFeed)
	{
		m_avatarRetarget.reset();
		m_directRetarget.reset();
		m_avatarPoseFeed= feed;
	}

	const AvatarRetargetConfig retargetConfig= makeAvatarRetargetConfig(*config, rig);
	const DirectRetarget::SolveState* directState= nullptr;
	if (preview != AvatarPanel::ePreviewPose::Live)
	{
		if (preview == AvatarPanel::ePreviewPose::Rest && config->direct.enabled)
		{
			// Direct mode: the same rest pose the stream falls back to
			DirectRetarget::makeRestPose(*skeleton, makeDirectRetargetConfig(*config, rig), m_avatarPose);
		}
		else
		{
			TrackingFrameResult frame;
			BodyDimensions user;
			if (preview == AvatarPanel::ePreviewPose::Rest)
				AvatarPreviewPoses::makeRestFrame(*skeleton, frame, user);
			else
				AvatarPreviewPoses::makeDemoFrame(*skeleton, frame, user);
			m_avatarRetarget.solve(frame, user, *skeleton, retargetConfig, m_avatarPose);
		}
		m_scene3dPanel->setAvatarFace(nullptr);
	}
	else
	{
		const AppConfig& lengthsConfig= *m_shown.config;
		const TrackingFrameResult& shownFused= *m_shown.fused;
		// Direct mode: live, the vision thread's pose arrives with the result;
		// in replay the window's own solver runs the recorded camera's frame
		// under the recorded config. Either way the position retarget is
		// skipped, so the scene shows exactly what the stream carries.
		bool bDirectShown= false;
		if (bReplay)
		{
			bDirectShown= solveReplayDirectPose(lengthsConfig, *skeleton, rig);
			if (bDirectShown)
				directState= &m_directRetarget.getSolveState();
		}
		else if (m_latestDirectPose.valid)
		{
			m_avatarPose= m_latestDirectPose;
			bDirectShown= true;
			directState= &m_latestDirectState;
		}
		if (!bDirectShown)
		{
			m_avatarRetarget.solve(shownFused, makeBodyDimensions(lengthsConfig), *skeleton, retargetConfig,
								   m_avatarPose);
		}
		m_scene3dPanel->setAvatarFace(&shownFused.face);
	}
	m_scene3dPanel->setAvatarPose(&m_avatarPose);
	m_shownDirectState= directState;
	updateSceneCameraMatch(directState, skeleton.get());

	// The elbow hints where the solve put them: off the posed upper-arm
	// joint, turned with the root
	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
	computePosedBones(*skeleton, m_avatarPose, posed);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		SceneGizmos::ElbowHint& hint= outGizmos.elbowHints[sideIndex];
		hint.bEnabled= config->avatar.showInScene && m_avatarPanel->getShowElbowGizmo(sideIndex);
		const eHumanoidBone upperArm= sideIndex == 0 ? eHumanoidBone::LeftUpperArm : eHumanoidBone::RightUpperArm;
		hint.shoulderWorld= posed[(int)upperArm].positionWorld;
		hint.hintWorld= hint.shoulderWorld + m_avatarPose.rootRotationWorld * rig.sides[sideIndex].elbowHintOffset;
	}
}

void MainWindow::updateSceneCameraMatch(const DirectRetarget::SolveState* directState, const AvatarSkeleton* skeleton)
{
	SceneCameraMatch match;
	if (directState != nullptr && skeleton != nullptr && directState->torsoValid && directState->camera.width > 0 &&
		directState->camera.height > 0)
	{
		match.valid= true;
		match.camera= directState->camera;
		match.cameraToWorld= DirectRetarget::makeCameraToWorld(*skeleton, m_avatarPose, *directState);
		// The chains live in the solve's world, the camera at the origin; the
		// camera's translation carries them to the avatar's
		m_scene3dPanel->setDirectSolve(directState,
									   glm::translate(glm::mat4(1.f), glm::vec3(match.cameraToWorld[3])));
	}
	else
	{
		m_scene3dPanel->setDirectSolve(nullptr, glm::mat4(1.f));
		const AppConfig* config= m_app->getConfig();
		if (config->cameraCount() > 0 && !m_latestPreviews.empty() && m_latestPreviews[0].valid &&
			!m_latestPreviews[0].bgr.empty())
		{
			const CameraProfile& profile= config->camera(0);
			if (profile.extrinsics.present && profile.intrinsics.present)
			{
				const MikanMatrix3d& cameraMatrix= profile.intrinsics.intrinsics.undistorted_camera_matrix;
				match.valid= true;
				match.camera.fx= (float)cameraMatrix.x0;
				match.camera.fy= (float)cameraMatrix.y1;
				match.camera.cx= (float)cameraMatrix.z0;
				match.camera.cy= (float)cameraMatrix.z1;
				match.camera.width= m_latestPreviews[0].bgr.cols;
				match.camera.height= m_latestPreviews[0].bgr.rows;
				match.cameraToWorld= glm::mat4(profile.extrinsics.markerFromCamera);
			}
		}
	}
	m_scene3dPanel->setCameraMatch(match);
}

bool MainWindow::solveReplayDirectPose(const AppConfig& recordedConfig, const AvatarSkeleton& skeleton,
									   const AvatarRigSettings& rig)
{
	if (!recordedConfig.direct.enabled)
		return false;
	const int cameraIndex= recordedConfig.direct.cameraIndex;
	const std::vector<const TrackingFrameResult*> perCamera= m_timelinePanel->getPerCameraResults();
	if (cameraIndex < 0 || cameraIndex >= (int)perCamera.size() || perCamera[cameraIndex] == nullptr ||
		cameraIndex >= (int)recordedConfig.cameraCount() || !recordedConfig.camera(cameraIndex).intrinsics.present)
		return false;

	// The recording's undistorted pinhole, the space its landmarks live in
	const TrackingFrameResult& frame= *perCamera[cameraIndex];
	const MikanMatrix3d& cameraMatrix=
		recordedConfig.camera(cameraIndex).intrinsics.intrinsics.undistorted_camera_matrix;
	DirectCameraModel cameraModel;
	cameraModel.fx= (float)cameraMatrix.x0;
	cameraModel.fy= (float)cameraMatrix.y1;
	cameraModel.cx= (float)cameraMatrix.z0;
	cameraModel.cy= (float)cameraMatrix.z1;
	cameraModel.width= frame.frameWidth;
	cameraModel.height= frame.frameHeight;
	// No phone head in a replay: the recording carries the camera inputs
	const TrackingFrameResult::HeadPose noPhoneHead;
	m_directRetarget.solve(frame, cameraModel, noPhoneHead, frame.timestampMs, skeleton,
						   makeDirectRetargetConfig(recordedConfig, rig), m_avatarPose);
	return m_avatarPose.valid;
}

void MainWindow::applyGizmoDrags(const SceneGizmos& gizmos)
{
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SceneGizmos::ElbowHint& hint= gizmos.elbowHints[sideIndex];
		if (!hint.bDragging || hint.draggedHintWorld == hint.hintWorld)
			continue;
		// Back into the rooted torso frame the hint is stored in
		AvatarRigSettings rig= m_app->getAvatarRig();
		rig.sides[sideIndex].elbowHintOffset=
			glm::inverse(m_avatarPose.rootRotationWorld) * (hint.draggedHintWorld - hint.shoulderWorld);
		m_app->setAvatarRig(rig);
	}
}

// -- Pipeline ---------------------------------------------------------------

void MainWindow::gatherPipelineInputs()
{
	// The project's shape and calibration are the shown frame's: a replayed
	// take keeps the cameras and the pipeline it was recorded with
	const AppConfig* config= m_shown.config;
	VisionThread* visionThread= m_app->getVisionThread();
	VideoCaptureSystem* videoCapture= m_app->getVideoCapture();
	const TrackingFrameResult& fused= *m_shown.fused;
	const bool bReplay= m_shown.bReplay;
	// The stage state of the shown frame: the vision thread's snapshot and
	// the services live, the frame's own record in a replay. A take made
	// before stage state was recorded has none.
	const VisionThread::Telemetry telemetry= visionThread->getTelemetry();
	const RecordedTelemetry* recorded= bReplay ? m_timelinePanel->getDisplayTelemetry() : nullptr;

	PipelineStatusInputs in;
	in.bDirect= config->direct.enabled;
	in.cameraCount= (int)config->cameraCount();
	in.directCameraIndex= config->direct.cameraIndex;
	in.bReplay= bReplay;
	in.bStageStateRecorded= !bReplay || recorded != nullptr;

	for (int cameraIndex= 0; cameraIndex < std::min(in.cameraCount, PipelineStatusInputs::k_maxCameras); ++cameraIndex)
	{
		PipelineStatusInputs::Camera& camera= in.cameras[cameraIndex];
		const CameraProfile& profile= config->camera(cameraIndex);
		camera.bIntrinsics= profile.intrinsics.present;
		camera.bExtrinsics= profile.extrinsics.present;
		camera.bBodyEnabled= profile.bodyPose.enabled;
		if (!bReplay)
		{
			camera.bDeviceOpen= videoCapture->getIsDeviceOpen(cameraIndex);
			camera.bStreaming= videoCapture->isStreaming(cameraIndex);
			camera.bHandPipelineReady= std::string(visionThread->getActiveExecutionProvider(cameraIndex)) != "none";
			camera.bTrackingPaused= !visionThread->isTrackingEnabled(cameraIndex);
			camera.bBodyModelLoaded=
				cameraIndex < VisionThread::k_maxReportedCameras && telemetry.bBodyModelLoaded[cameraIndex];
		}
		else if (recorded != nullptr && cameraIndex < recorded->cameraCount)
		{
			const RecordedTelemetry::Camera& state= recorded->cameras[cameraIndex];
			camera.bDeviceOpen= state.deviceOpen;
			camera.bStreaming= state.streaming;
			camera.bHandPipelineReady= state.handPipelineReady;
			camera.bTrackingPaused= !state.trackingEnabled;
			camera.bBodyModelLoaded= state.bodyModelLoaded;
		}
		else
		{
			// The take holds this camera's frames, so it was open and its
			// models ran; nothing says otherwise
			camera.bDeviceOpen= true;
			camera.bStreaming= true;
			camera.bHandPipelineReady= true;
			camera.bBodyModelLoaded= profile.bodyPose.enabled;
		}
		const TrackingFrameResult* result=
			cameraIndex < (int)m_shown.perCamera.size() ? m_shown.perCamera[cameraIndex] : nullptr;
		if (result != nullptr)
		{
			camera.captureFps= result->captureFps;
			camera.handsTracked= (result->hands[0].tracked ? 1 : 0) + (result->hands[1].tracked ? 1 : 0);
			camera.bBodyValid= result->body.valid;
			camera.bodyConfidence= result->body.confidence;
			camera.bBodyHasDepth= result->body.hasDepth;
		}
	}

	// What the iteration behind the shown frame did
	using ePath= PipelineStatusInputs::ePath;
	auto toPath= [](VisionThread::Telemetry::ePath path) {
		switch (path)
		{
		case VisionThread::Telemetry::ePath::Idle:
			return ePath::Idle;
		case VisionThread::Telemetry::ePath::FaceOnly:
			return ePath::FaceOnly;
		case VisionThread::Telemetry::ePath::Fused:
			return ePath::Fused;
		case VisionThread::Telemetry::ePath::Passthrough:
			return ePath::Passthrough;
		case VisionThread::Telemetry::ePath::Direct:
			return ePath::Direct;
		}
		return ePath::Unknown;
	};
	if (recorded != nullptr)
	{
		in.path= toPath((VisionThread::Telemetry::ePath)recorded->path);
	}
	else if (bReplay)
	{
		// Unrecorded: the frame's own world pose says which way it went
		const bool bWorld= fused.poses[0].hasWorldPose || fused.poses[1].hasWorldPose;
		in.path= in.bDirect ? ePath::Direct : (bWorld ? ePath::Fused : ePath::Unknown);
	}
	else if (telemetry.bValid)
	{
		in.path= toPath(telemetry.path);
	}

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= fused.poses[sideIndex];
		PipelineStatusInputs::Hand& hand= in.hands[sideIndex];
		hand.bTracked= pose.tracked;
		hand.bWorldPose= pose.hasWorldPose;
		hand.confidence= pose.confidence;
		hand.bForearmPose= pose.hasForearmPose;
		hand.forearmConfidence= pose.forearmConfidence;
		hand.bShoulder= pose.hasShoulder;

		PipelineStatusInputs::Imu& imu= in.imu[sideIndex];
		if (!bReplay)
		{
			hand.dominantCamera= visionThread->getDominantCamera((eHandSide)sideIndex);
			const ImuSideStatus imuStatus= visionThread->getImuSideStatus((eHandSide)sideIndex);
			imu.bConnected= imuStatus.deviceConnected;
			imu.bStreaming= imuStatus.streaming;
			imu.bCalibrated= imuStatus.calibrated;
			imu.bOrientationValid= imuStatus.orientationValid;
			imu.sampleRateHz= imuStatus.sampleRateHz;
		}
		else if (recorded != nullptr)
		{
			hand.dominantCamera= recorded->dominantCamera[sideIndex];
			const RecordedImuStatus& imuStatus= recorded->imu[sideIndex];
			imu.bConnected= imuStatus.deviceConnected;
			imu.bStreaming= imuStatus.streaming;
			imu.bCalibrated= imuStatus.calibrated;
			imu.bOrientationValid= imuStatus.orientationValid;
			imu.sampleRateHz= imuStatus.sampleRateHz;
		}
	}
	in.bHeadValid= fused.head.valid;
	in.bImuEnabled= config->imu.enabled;

	// The direct solve behind the shown pose
	if (in.bDirect && in.directCameraIndex >= 0 && in.directCameraIndex < in.cameraCount)
		in.bDirectIntrinsics= config->camera(in.directCameraIndex).intrinsics.present;
	in.bDirectNeutral= config->direct.neutralPresent;
	if (m_shownDirectState != nullptr)
	{
		const DirectRetarget::SolveState& state= *m_shownDirectState;
		in.bDirectTorso= state.torsoValid;
		in.bDirectHeadMeasured= state.headMeasured;
		in.directShoulderDepthMeters= glm::length(state.shoulderMidpoint);
		in.directLeanDegrees= glm::degrees(glm::angle(glm::normalize(state.spineDelta)));
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::SolveState::Arm& arm= state.arms[sideIndex];
			PipelineStatusInputs::DirectArm& out= in.directArms[sideIndex];
			out.bChain= arm.chain.valid;
			out.bHandSeen= arm.bHandSeen;
			out.bElbowSeen= arm.bElbowSeen;
			out.bHandMeasured= arm.bHandMeasured;
			out.bResting= arm.bResting;
			out.restBlend= arm.restBlend;
			out.elbowMissPx= arm.chosenElbowPx;
		}
	}

	// The avatar is the one loaded now, whichever frame is shown
	in.bAvatarLoaded= m_app->getAvatarSkeleton() != nullptr;
	if (in.bAvatarLoaded)
	{
		in.bArmPlaced[0]= m_avatarPose.bones[(int)eHumanoidBone::LeftUpperArm].present;
		in.bArmPlaced[1]= m_avatarPose.bones[(int)eHumanoidBone::RightUpperArm].present;
	}

	in.bOscEnabled= config->osc.enabled;
	in.oscMinConfidence= config->osc.minConfidence;
	in.oscPort= config->osc.port;
	in.bFaceEnabled= config->face.enabled;
	in.bAudioEnabled= config->audio.enabled;
	if (!bReplay)
	{
		in.bOscRunning= telemetry.bValid && telemetry.bOscRunning;
		in.oscSendRateHz= telemetry.oscSendRateHz;
		if (FaceService* faceService= m_app->getFaceService())
		{
			const FaceServiceStatus faceStatus= faceService->getStatus();
			in.bFaceBound= faceStatus.bound;
			in.bFaceStreaming= faceStatus.streaming;
			in.bFaceTracked= faceStatus.latest.valid && faceStatus.latest.faceTracked;
			in.faceDatagramsPerSecond= faceStatus.datagramsPerSecond;
		}
		if (AudioService* audioService= m_app->getAudioService())
		{
			const AudioServiceStatus audioStatus= audioService->getStatus();
			in.bAudioDeviceOpen= audioStatus.deviceOpen;
			in.bAudioBackendReady= audioStatus.backendReady;
			in.bAudioGateOpen= audioStatus.gateOpen;
			in.audioLevelDbfs= audioStatus.levelDbfs;
		}
	}
	else if (recorded != nullptr)
	{
		in.bOscRunning= recorded->oscRunning;
		in.oscSendRateHz= recorded->oscSendRateHz;
		in.bFaceBound= recorded->face.bound;
		in.bFaceStreaming= recorded->face.streaming;
		in.bFaceTracked= recorded->face.faceTracked;
		in.faceDatagramsPerSecond= recorded->face.datagramsPerSecond;
		in.bAudioDeviceOpen= recorded->audio.deviceOpen;
		in.bAudioBackendReady= recorded->audio.backendReady;
		in.bAudioGateOpen= recorded->audio.gateOpen;
		in.audioLevelDbfs= recorded->audio.levelDbfs;
	}

	// The face profile is the live one, which a replay is evaluated through
	in.bProfileAuto= m_app->isActiveFaceProfileAuto();
	if (const std::shared_ptr<const FaceProfileBinding>& binding= m_app->getFaceBinding())
	{
		in.profileTargetCount= (int)binding->getWireNames().size();
		for (const eFaceTargetProblem problem : binding->getOutputProblemCodes())
			in.profileProblemCount+= problem != eFaceTargetProblem::None ? 1 : 0;
	}

	m_pipelineInputs= in;
}

bool MainWindow::handlePipelineAutomation(const std::vector<std::string>& args, std::vector<std::string>& outLines,
										  std::string& outError)
{
	if (args.empty())
	{
		outError= "missing verb";
		return false;
	}
	const std::string& verb= args[0];
	const PipelineStatusInputs& inputs= m_pipelineInputs;
	// One stage as "<id> <state> | <value> | <reason>", the texts as shown
	auto describe= [&](const PipelineStageId& stage) {
		const StageStatus status= PipelineModel::computeStatus(inputs, stage);
		static const char* k_states[]= {"running", "degraded", "inactive", "not-configured"};
		return PipelineModel::toString(stage) + " " + k_states[(int)status.state] + " | " +
			PipelineText::format(status.value) + " | " + PipelineText::format(status.reason);
	};

	if (verb == "stages")
	{
		outLines.push_back(std::string("shape ") + (inputs.bDirect ? "direct" : "multi") + " cameras " +
						   std::to_string(inputs.cameraCount) + " shown " + (inputs.bReplay ? "replay" : "live"));
		for (const PipelineStageId& stage : PipelineModel::getAllStages(inputs.bDirect, inputs.cameraCount))
			outLines.push_back(describe(stage));
		return true;
	}
	if (verb == "selected")
	{
		const PipelineStageId& selected= m_pipelinePanel->getSelected();
		outLines.push_back(selected.isValid() ? describe(selected) : std::string("none"));
		return true;
	}
	if (verb == "status" || verb == "select")
	{
		if (args.size() < 2)
		{
			outError= "missing stage";
			return false;
		}
		if (verb == "select" && args[1] == "none")
		{
			m_pipelinePanel->setSelected(PipelineStageId());
			outLines.push_back("none");
			return true;
		}
		PipelineStageId stage;
		const std::vector<PipelineStageId> all= PipelineModel::getAllStages(inputs.bDirect, inputs.cameraCount);
		if (!PipelineModel::fromString(args[1], stage) || std::find(all.begin(), all.end(), stage) == all.end())
		{
			outError= "no such stage in this project: " + args[1];
			return false;
		}
		if (verb == "select")
			m_pipelinePanel->setSelected(stage);
		outLines.push_back(describe(stage));
		return true;
	}
	outError= "unknown verb: " + verb;
	return false;
}
