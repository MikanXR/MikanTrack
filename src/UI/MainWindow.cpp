#include "MainWindow.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
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
#include "PerformancePanel.h"
#include "PinnedStageWindows.h"
#include "PipelinePanel.h"
#include "PipelineText.h"
#include "ProjectManager.h"
#include "RetargetChains.h"
#include "Scene3dPanel.h"
#include "SourcePoseInputs.h"
#include "SettingsPanels.h"
#include "SetupFlow.h"
#include "StageContent.h"
#include "StageInspector.h"
#include "StageView.h"
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
	, m_stageView(std::make_unique<StageView>(m_videoPreviewPanel.get(), m_scene3dPanel.get()))
	, m_pinnedWindows(std::make_unique<PinnedStageWindows>(m_videoPreviewPanel.get(), m_scene3dPanel.get()))
	, m_performancePanel(std::make_unique<PerformancePanel>())
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
		{"pipeline stages", "pipeline status <stage>", "pipeline selected", "pipeline select <stage>|none",
		 "pipeline view [follow|cameras|scene|overlay|sidebyside]", "pipeline pin <stage>",
		 "pipeline unpin <stage>|all",
		 "pipeline pinned", "pipeline chain [torso|head|leftArm|rightArm|leftHand|rightHand]",
		 "pipeline fx <leftArm|rightArm> elbowWeight [value <number>|expr <text...>]",
		 "pipeline limit <head|spine|leftWrist|rightWrist> [<degrees>]"},
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
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.stageView"), centerId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.tracking"), leftId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.settings"), leftId);
		ImGui::DockBuilderDockWindow(locWindowTitle("windows.stageInspector"), rightId);
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
	// tabs one frame apart once the windows exist, ending on the Stage View
	if (m_dockFrontTabsCountdown > 0)
	{
		static const char* k_frontTabs[]= {"windows.stageView", "windows.stageInspector", "windows.tracking",
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
			ImGui::MenuItem(locLabel("mainWindow.viewStageViewPanel"), nullptr, &m_bShowStageViewPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewTrackingPanel"), nullptr, &m_bShowTrackingPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewTimelinePanel"), nullptr, &m_bShowTimelinePanel);
			ImGui::MenuItem(locLabel("mainWindow.viewSettingsPanel"), nullptr, &m_bShowSettingsPanel);
			ImGui::MenuItem(locLabel("mainWindow.logPanel"), nullptr, &m_bShowLogPanel);
			ImGui::MenuItem(locLabel("mainWindow.viewPerformancePanel"), nullptr, &m_bShowPerformancePanel);
			ImGui::Separator();
			// Every panel shown again in the default arrangement
			if (ImGui::MenuItem(locLabel("mainWindow.resetLayout")))
			{
				m_bResetDockLayout= true;
				m_bShowStageViewPanel= m_bShowTrackingPanel= true;
				m_bShowTimelinePanel= m_bShowLogPanel= m_bShowSettingsPanel= true;
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
		// A closed project's pinned stages close with it
		m_pinnedWindows->unpinAll();
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
	visionThread->fetchFusedResult(m_latestFused, &m_latestRetarget, &m_latestFusionDiagnostics);

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
	// The stage sections the Inspector draws read this frame's contexts
	m_stageContent->beginFrame(trackingContext, oscContext, m_fusionReadout, m_imuReadout, m_faceReadout,
							   m_microphoneReadout, m_oscReadout, m_directReadout, m_sourcePoseReadout,
							   m_faceExprEditor->getTarget(), bWizardActive);
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

	// Show and front the Stage View when a camera calibration wizard starts:
	// the pattern feed is the wizard's whole UI, and the view may be closed or
	// behind another tab of its dock node
	if (m_wizardHost->consumeCameraWizardStarted())
	{
		m_bShowStageViewPanel= true;
		ImGui::SetWindowFocus(locWindowTitle("windows.stageView"));
	}

	// Pin the preview highlight to the intrinsics wizard's camera while it is
	// active (the extrinsics wizard uses ALL cameras, so no pinning there)
	const int intrinsicsCamera= m_wizardHost->getIntrinsicsCameraIndex();
	if (intrinsicsCamera >= 0)
		m_videoPreviewPanel->setActiveCamera(intrinsicsCamera);

	// The camera view's per-camera inputs, from the live output
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
									{m_latestRetarget.directState.arms[0].bHandSeen,
									 m_latestRetarget.directState.arms[1].bHandSeen});

	// The 3D scene's inputs. In replay view the whole feed comes from the
	// timeline's scrub position instead, with the frustums built from the
	// RECORDING's config snapshot.
	m_scene3dPanel->setForearmLength(config->body.forearmLengthMeters);
	m_scene3dPanel->setAvatar(m_app->getAvatarModel(), m_app->getAvatarSkeleton(), m_app->getAvatarGeneration());
	m_scene3dPanel->setFaceBinding(m_app->getFaceBinding());
	m_scene3dPanel->setShowAvatar(config->avatar.showInScene);
	SceneGizmos gizmos;
	updateDisplayAvatar(gizmos);
	const std::vector<SceneCameraView> sceneCameras= makeSceneCameraViews(*config);

	// The Stage View, on the strip's selection as the strip last drew it, once
	// the avatar is posed for the shown frame: the camera view or the scene
	m_stageView->update(m_pipelinePanel->getSelected(), m_pipelineInputs.bDirect, m_pipelineInputs.directCameraIndex,
						m_wizardHost->isCameraWizardActive());
	// The frame both the Stage View and the pinned stages' windows draw
	const std::vector<SceneCameraView> replayCameras=
		m_shown.bReplay ? m_timelinePanel->getSceneCameras() : std::vector<SceneCameraView>();
	StageView::Frame stageFrame;
	stageFrame.cameraResults= &perCameraResults;
	stageFrame.executionProviders= &executionProviders;
	stageFrame.forearms= &forearmOverlays;
	stageFrame.fused= m_shown.fused;
	// The frustums come from the config the shown frame was made under
	stageFrame.sceneCameras= m_shown.bReplay ? &replayCameras : &sceneCameras;
	stageFrame.scenePerCamera= &m_shown.perCamera;
	stageFrame.gizmos= &gizmos;
	if (!m_bShowStageViewPanel)
		m_videoPreviewPanel->clearLastDrawList();
	else
		m_stageView->draw(&m_bShowStageViewPanel, stageFrame);
	applyGizmoDrags(gizmos);
	// A click on the Retarget view's source pose selects that chain, for the
	// Inspector drawn below
	eRetargetChain pickedChain;
	if (m_scene3dPanel->consumePickedChain(pickedChain))
		m_avatarPanel->setSelectedChain(pickedChain);

	// The pipeline, once the avatar is posed for the shown frame: the strip,
	// the inspector on the strip's selection, and the frame loop's timings
	gatherPipelineInputs();
	if (m_bShowPipelinePanel)
		m_pipelinePanel->draw(&m_bShowPipelinePanel, m_pipelineInputs);
	const PipelineStageId pinRequest= m_pipelinePanel->consumePinRequest();
	if (pinRequest.isValid())
		m_pinnedWindows->pin(pinRequest);
	if (m_bShowInspectorPanel)
	{
		const PipelineStageId requested= m_stageInspector->draw(&m_bShowInspectorPanel, m_pipelineInputs,
																m_pipelinePanel->getSelected(), m_stageContent.get());
		if (requested.isValid())
			m_pipelinePanel->setSelected(requested);
	}
	// The pinned stages' windows, after the Inspector on the same inputs and
	// sections, and before the launch requests their sections raise are read
	// below. Their scenes render from the pose the Stage View's did.
	{
		const PipelineStageId requested= m_pinnedWindows->draw(m_pipelineInputs, stageFrame, m_stageContent.get());
		if (requested.isValid())
			m_pipelinePanel->setSelected(requested);
	}
	// After the menu bar, the Inspector and the pinned windows, the draws
	// that can unload the avatar, so the preview pose is back to Live before
	// the next frame's scene reads it. This frame's scenes were posed before
	// the Inspector drew.
	m_avatarPanel->update();
	// The calibration wizards the stage sections drawn this frame asked for
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
		m_performancePanel->draw(&m_bShowPerformancePanel, *visionThread, m_shown.bReplay);

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
	updateOscReadout();
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

void MainWindow::updateOscReadout()
{
	OscReadout readout;
	readout.bReplay= m_shown.bReplay;
	if (m_shown.bReplay)
	{
		const RecordedTelemetry* recorded= m_timelinePanel->getDisplayTelemetry();
		if (recorded != nullptr)
		{
			readout.bAvailable= true;
			readout.bRunning= recorded->oscRunning;
			readout.framesPerSecond= recorded->oscSendRateHz;
			readout.lastPacketCount= recorded->oscLastPacketCount;
			readout.lastFrameBytes= recorded->oscLastFrameBytes;
		}
	}
	else
	{
		// Before the vision thread's first iteration the stream reads as not
		// running
		const VisionThread::Telemetry telemetry= m_app->getVisionThread()->getTelemetry();
		readout.bAvailable= true;
		readout.bRunning= telemetry.bValid && telemetry.bOscRunning;
		readout.framesPerSecond= telemetry.oscSendRateHz;
		readout.lastPacketCount= telemetry.oscLastPacketCount;
		readout.lastFrameBytes= telemetry.oscLastFrameBytes;
	}
	m_oscReadout= readout;
}

void MainWindow::updateDisplayAvatar(SceneGizmos& outGizmos)
{
	updateDirectReadout(nullptr);
	updateSourcePoseReadout(nullptr);
	updateSceneSourcePose(nullptr, nullptr, nullptr);
	m_avatarPanel->setShownElbowWeights(nullptr);
	const std::shared_ptr<const AvatarSkeleton> skeleton= m_app->getAvatarSkeleton();
	if (skeleton == nullptr)
	{
		// Live, the vision thread's stage still fills the source the stream
		// is built from. A replay runs the window's stage only to pose an
		// avatar.
		if (!m_shown.bReplay)
		{
			updateSourcePoseReadout(&m_latestRetarget);
			updateSceneSourcePose(&m_latestRetarget, &m_shown.fused->head, nullptr);
		}
		m_scene3dPanel->setAvatarPose(nullptr);
		m_scene3dPanel->setAvatarFace(nullptr);
		updateSceneCameraMatch(nullptr, nullptr);
		return;
	}
	const AppConfig* config= m_app->getConfig();
	const AvatarRigSettings& rig= m_app->getAvatarRig();

	// The feed: a preview pose from the Retarget stage, else replay frames,
	// else the live output. Switching feeds starts the window's stage over,
	// so nothing it holds or filters bridges two timelines.
	const AvatarPanel::ePreviewPose preview= m_avatarPanel->getPreviewPose();
	const bool bReplay= m_shown.bReplay;
	const int feed= preview != AvatarPanel::ePreviewPose::Live ? 1 + (int)preview : (bReplay ? 1 : 0);
	if (feed != m_avatarPoseFeed)
	{
		m_displayStage.reset();
		m_replayPlayhead.invalidate();
		m_avatarPoseFeed= feed;
	}

	const DirectRetarget::SolveState* directState= nullptr;
	// The direct Rest preview is posed from no source
	const RetargetResult* shownResult= nullptr;
	// The head the shown result's frame measured
	TrackingFrameResult::HeadPose shownHead= m_shown.fused->head;
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
			RetargetStageConfig stageConfig= makeRetargetStageConfig(*config, rig, skeleton);
			stageConfig.user= user;
			m_displayStage.setConfig(stageConfig);
			m_displayStage.solvePosition(frame, m_displayResult);
			m_avatarPose= m_displayResult.pose;
			shownResult= &m_displayResult;
			shownHead= frame.head;
		}
		m_scene3dPanel->setAvatarFace(nullptr);
	}
	else if (bReplay)
	{
		directState= updateReplayRetarget(*config, rig, skeleton);
		m_avatarPose= m_displayResult.pose;
		shownResult= &m_displayResult;
		m_scene3dPanel->setAvatarFace(&m_shown.fused->face);
	}
	else
	{
		// Live: the vision thread's retarget stage posed the avatar for the
		// stream, through the same loss rules, so the scene shows exactly
		// what the stream carries
		m_avatarPose= m_latestRetarget.pose;
		shownResult= &m_latestRetarget;
		if (m_latestRetarget.bDirectState)
			directState= &m_latestRetarget.directState;
		m_scene3dPanel->setAvatarFace(&m_shown.fused->face);
	}
	m_scene3dPanel->setAvatarPose(&m_avatarPose);
	m_avatarPanel->setShownElbowWeights(shownResult != nullptr ? &shownResult->elbowWeights : nullptr);
	updateDirectReadout(directState);
	updateSourcePoseReadout(shownResult);
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

	const glm::vec3 posedHeadWorld= posed[(int)eHumanoidBone::Head].positionWorld;
	updateSceneSourcePose(shownResult, &shownHead, &posedHeadWorld);
}

void MainWindow::updateSceneSourcePose(const RetargetResult* result, const TrackingFrameResult::HeadPose* measuredHead,
									   const glm::vec3* posedHeadWorld)
{
	m_scene3dPanel->setSelectedChain(m_avatarPanel->getSelectedChain());
	if (result == nullptr || result->source.pipeline == SourcePose::ePipeline::None)
	{
		m_scene3dPanel->setSourcePose(nullptr, nullptr);
		return;
	}
	const glm::vec3* headAnchor= nullptr;
	if (result->source.pipeline == SourcePose::ePipeline::Position)
		headAnchor= measuredHead != nullptr && measuredHead->valid ? &measuredHead->positionWorld : nullptr;
	else
		headAnchor= posedHeadWorld;
	m_scene3dPanel->setSourcePose(result, headAnchor);
}

void MainWindow::updateDirectReadout(const DirectRetarget::SolveState* state)
{
	DirectReadout readout;
	readout.state= state;
	readout.bNeutralPresent= m_shown.config->direct.neutralPresent;
	if (state != nullptr)
	{
		readout.shoulderDepth= glm::length(state->shoulderMidpoint);
		readout.leanDegrees= glm::degrees(glm::angle(glm::normalize(state->spineDelta)));
	}
	m_directReadout= readout;
}

void MainWindow::updateSourcePoseReadout(const RetargetResult* result)
{
	// A result no pipeline has filled yet carries no source
	SourcePoseReadout readout;
	if (result != nullptr && result->source.pipeline != SourcePose::ePipeline::None)
		readout.source= &result->source;
	m_sourcePoseReadout= readout;
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

const DirectRetarget::SolveState* MainWindow::updateReplayRetarget(
	const AppConfig& liveConfig, const AvatarRigSettings& rig, const std::shared_ptr<const AvatarSkeleton>& skeleton)
{
	// The recording's lengths, direct settings and retarget chains (its loss
	// rules and elbow weights), the project's avatar placement, and the live
	// rig. A take from before chain settings migrates its chains from its own
	// legacy settings and the live rig.
	const AppConfig& recordedConfig= m_timelinePanel->getRecordedConfig();
	const RetargetChains recordedChains= resolveRetargetChains(recordedConfig, rig);
	RetargetStageConfig stageConfig= makeRetargetStageConfig(recordedConfig, rig, recordedChains, skeleton);
	stageConfig.position= makeAvatarRetargetConfig(liveConfig, rig);
	m_displayStage.setConfig(stageConfig);

	// A direct-mode recording's camera and its undistorted pinhole, the
	// space its landmarks live in
	const int cameraIndex= recordedConfig.direct.cameraIndex;
	const bool bDirect= recordedConfig.direct.enabled && cameraIndex >= 0 &&
		cameraIndex < (int)recordedConfig.cameraCount() && recordedConfig.camera(cameraIndex).intrinsics.present;
	DirectCameraModel pinhole;
	if (bDirect)
	{
		const MikanMatrix3d& cameraMatrix=
			recordedConfig.camera(cameraIndex).intrinsics.intrinsics.undistorted_camera_matrix;
		pinhole.fx= (float)cameraMatrix.x0;
		pinhole.fy= (float)cameraMatrix.y1;
		pinhole.cx= (float)cameraMatrix.z0;
		pinhole.cy= (float)cameraMatrix.z1;
	}

	// One recorded frame through the stage the way the vision thread ran it.
	// No phone head in a replay: the recording carries the camera inputs.
	const TimelinePanel& timeline= *m_timelinePanel;
	auto solveFrame= [&](int frameIndex, RetargetStage& stage, RetargetResult& ioResult) {
		TrackingFrameResult fused;
		std::vector<const TrackingFrameResult*> perCamera;
		timeline.getRetargetInputs(frameIndex, fused, perCamera);
		if (bDirect && cameraIndex < (int)perCamera.size() && perCamera[cameraIndex] != nullptr)
		{
			const TrackingFrameResult& frame= *perCamera[cameraIndex];
			DirectCameraModel cameraModel= pinhole;
			cameraModel.width= frame.frameWidth;
			cameraModel.height= frame.frameHeight;
			const TrackingFrameResult::HeadPose noPhoneHead;
			if (stage.solveDirect(frame, cameraModel, noPhoneHead, frame.timestampMs, ioResult))
				return;
		}
		stage.solvePosition(fused, ioResult);
	};
	auto frameTimeMs= [&timeline](int frameIndex) { return timeline.getFrameTimeMs(frameIndex); };

	const bool bSolved= m_replayPlayhead.seek(m_displayStage, timeline.getReplayTimelineId(),
											  timeline.getCurrentFrameIndex(), frameTimeMs, solveFrame, m_displayResult);
	if (!bSolved)
		m_displayStage.resolveTarget(m_displayResult);
	return m_displayResult.bDirectState ? &m_displayResult.directState : nullptr;
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
	if (m_directReadout.state != nullptr)
	{
		const DirectRetarget::SolveState& state= *m_directReadout.state;
		in.bDirectTorso= state.torsoValid;
		in.bDirectHeadMeasured= state.headMeasured;
		in.directShoulderDepthMeters= m_directReadout.shoulderDepth;
		in.directLeanDegrees= m_directReadout.leanDegrees;
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
	if (verb == "view")
	{
		// A mode, or the Retarget view's layout, which is kept whatever the
		// view shows
		if (args.size() >= 2)
		{
			eStageViewMode mode;
			eRetargetLayout layout;
			if (StageView::parseMode(args[1], mode))
				m_stageView->setMode(mode);
			else if (StageView::parseRetargetLayout(args[1], layout))
				m_stageView->setRetargetLayout(layout);
			else
			{
				outError= "unknown view mode or layout: " + args[1];
				return false;
			}
		}
		outLines.push_back(m_stageView->describe());
		return true;
	}
	if (verb == "chain")
	{
		// The Retarget stage's selected chain, by the id of its block in the
		// project's chain settings. Without an avatar only an arm selects.
		if (args.size() >= 2)
		{
			eRetargetChain chain;
			if (!AvatarPanel::parseChainId(args[1], chain))
			{
				outError= "unknown chain: " + args[1];
				return false;
			}
			if (!m_avatarPanel->setSelectedChain(chain))
			{
				outError= "no avatar loaded, only leftArm and rightArm select: " + args[1];
				return false;
			}
		}
		outLines.push_back(AvatarPanel::getChainId(m_avatarPanel->getSelectedChain()));
		return true;
	}
	if (verb == "fx")
		return handleFxAutomation(args, outLines, outError);
	if (verb == "limit")
		return handleLimitAutomation(args, outLines, outError);
	if (verb == "pinned")
	{
		const std::vector<PipelineStageId> pinned= m_pinnedWindows->getPinned();
		for (const PipelineStageId& stage : pinned)
			outLines.push_back(PipelineModel::toString(stage));
		if (pinned.empty())
			outLines.push_back("none");
		return true;
	}
	if (verb == "unpin")
	{
		if (args.size() < 2)
		{
			outError= "missing stage";
			return false;
		}
		if (args[1] == "all")
		{
			m_pinnedWindows->unpinAll();
			outLines.push_back("none");
			return true;
		}
		PipelineStageId stage;
		if (!PipelineModel::fromString(args[1], stage) || !m_pinnedWindows->unpin(stage))
		{
			outError= "not pinned: " + args[1];
			return false;
		}
		outLines.push_back(PipelineModel::toString(stage));
		return true;
	}
	if (verb == "status" || verb == "select" || verb == "pin")
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
		if (verb == "pin")
		{
			// The window opens on the next draw; the reply is the id the gui
			// namespace addresses it by
			m_pinnedWindows->pin(stage);
			outLines.push_back(PinnedStageWindows::getWindowId(stage));
			return true;
		}
		if (verb == "select")
			m_pipelinePanel->setSelected(stage);
		outLines.push_back(describe(stage));
		return true;
	}
	outError= "unknown verb: " + verb;
	return false;
}

bool MainWindow::handleLimitAutomation(const std::vector<std::string>& args, std::vector<std::string>& outLines,
									   std::string& outError)
{
	// limit <head|spine|leftWrist|rightWrist> [<degrees>]
	if (args.size() < 2 || args.size() > 3)
	{
		outError= "usage: limit <head|spine|leftWrist|rightWrist> [<degrees>]";
		return false;
	}
	if (m_app->getAvatarModel() == nullptr)
	{
		outError= "no avatar loaded";
		return false;
	}
	AvatarRigSettings rig= m_app->getAvatarRig();
	float* degrees= nullptr;
	if (args[1] == "head")
		degrees= &rig.headLimitDegrees;
	else if (args[1] == "spine")
		degrees= &rig.spineLimitDegrees;
	else if (args[1] == "leftWrist")
		degrees= &rig.sides[0].wristLimitDegrees;
	else if (args[1] == "rightWrist")
		degrees= &rig.sides[1].wristLimitDegrees;
	if (degrees == nullptr)
	{
		outError= "unknown limit: " + args[1];
		return false;
	}

	if (args.size() == 3)
	{
		// The slider's range: zero locks the joint, 180 never clamps
		char* end= nullptr;
		const double parsed= std::strtod(args[2].c_str(), &end);
		if (end == args[2].c_str() || *end != '\0' || !(parsed >= 0.0 && parsed <= 180.0))
		{
			outError= "not a number of degrees in [0, 180]: " + args[2];
			return false;
		}
		*degrees= (float)parsed;
		// The rig's one edit path, the panel's: the vision thread takes it
		// between frames and the sidecar is written by the rig's auto-save
		m_app->setAvatarRig(rig);
	}

	char text[64];
	std::snprintf(text, sizeof(text), "%s %.6g", args[1].c_str(), *degrees);
	std::string line= text;
	// Only the direct placement applies the limits
	if (!m_pipelineInputs.bDirect)
		line+= " (unused: the position pipeline applies no joint limit)";
	outLines.push_back(line);
	return true;
}

bool MainWindow::handleFxAutomation(const std::vector<std::string>& args, std::vector<std::string>& outLines,
									std::string& outError)
{
	// fx <arm> <parameter> [value <number> | expr <text...>]
	if (args.size() < 3)
	{
		outError= "usage: fx <leftArm|rightArm> elbowWeight [value <number>|expr <text...>]";
		return false;
	}
	int sideIndex= -1;
	if (args[1] == "leftArm")
		sideIndex= 0;
	else if (args[1] == "rightArm")
		sideIndex= 1;
	if (sideIndex < 0)
	{
		outError= "unknown arm: " + args[1];
		return false;
	}
	if (args[2] != "elbowWeight")
	{
		outError= "unknown parameter: " + args[2];
		return false;
	}

	RetargetChains chains= m_app->getRetargetChains();
	ExprParameter& weight= chains.arms[sideIndex].elbowWeight;
	if (args.size() >= 4)
	{
		const std::string& form= args[3];
		if (form == "value")
		{
			if (args.size() != 5)
			{
				outError= "value takes one number";
				return false;
			}
			char* end= nullptr;
			const double parsed= std::strtod(args[4].c_str(), &end);
			if (end == args[4].c_str() || *end != '\0' || !(parsed >= 0.0 && parsed <= 1.0))
			{
				outError= "not a number in [0, 1]: " + args[4];
				return false;
			}
			weight.value= (float)parsed;
			weight.bUseExpression= false;
		}
		else if (form == "expr")
		{
			if (args.size() < 5)
			{
				outError= "missing expression text";
				return false;
			}
			// The rest of the line is the text
			std::string text= args[4];
			for (size_t index= 5; index < args.size(); ++index)
				text+= " " + args[index];
			weight.expression= text;
			weight.bUseExpression= true;
		}
		else
		{
			outError= "unknown form: " + form;
			return false;
		}
		m_app->setRetargetChains(chains);
	}

	auto format= [](float value) {
		char text[32];
		std::snprintf(text, sizeof(text), "%.6g", value);
		return std::string(text);
	};
	outLines.push_back(std::string("setting ") + (weight.bUseExpression ? "function" : "value") + " value " +
					   format(weight.value) + " expr " + weight.expression);
	const FaceExprCompileResult compiled= compileExprParameter(weight.expression, SourcePoseInputs::table());
	if (compiled.errors.empty())
	{
		outLines.push_back("compile ok");
	}
	else
	{
		const FaceExprCompileError& error= compiled.errors.front();
		outLines.push_back("compile error at " + std::to_string(error.span.begin) + ".." +
						   std::to_string(error.span.end) + ": " + error.message);
	}
	// The frame the scene shows, solved before this setting reached it
	const RetargetElbowWeight& shown= m_avatarPanel->getShownElbowWeights()[sideIndex];
	if (!shown.bEvaluated)
	{
		outLines.push_back("shown none");
	}
	else
	{
		outLines.push_back("shown weight " + format(shown.weight) + " raw " + format(shown.rawValue) + " from " +
						   (shown.bExpression && shown.bCompiled ? "function" : "value") + " compiled " +
						   (shown.bCompiled ? "yes" : "no"));
	}
	return true;
}
