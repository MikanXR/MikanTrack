#pragma once

#include <string>
#include <vector>

#include "IVideoDevice.h" // eVideoSettingType, VideoSettingConstraint

class App;
class AppConfig;
class VideoCaptureSystem;

// Camera selection: one section per configured camera (device combo,
// resolution/frame-rate/format combos with best-match mode selection, stream
// start/stop), plus add/remove camera controls, drawn by the host window that
// shows the Capture stage. Subscribes to the capture system's hotplug,
// mode-change, and disconnect notifications so its cached lists follow the
// devices.
class DevicePanel
{
public:
	DevicePanel(App* app, VideoCaptureSystem* videoCapture, AppConfig* config);

	// The sections, each drawable in a host window. A camera section sizes
	// the per-camera state from the config itself, and draws nothing for a
	// camera index the config does not have.
	void drawRescanControl();
	// bWithHeader false leaves out the "Camera N" collapsing header, for a
	// host that names the camera; bWithSettings false leaves out the Camera
	// Settings tree, for a host that draws it elsewhere
	void drawCameraSection(int cameraIndex, bool bWithHeader= true, bool bWithSettings= true);
	// bDefaultOpen opens the tree the first time it is drawn
	void drawCameraSettings(int cameraIndex, bool bDefaultOpen= false);
	// Whether drawCameraSettings has anything to draw: the camera's device is
	// open and reports adjustable settings
	bool hasCameraSettings(int cameraIndex) const;
	void drawAddCameraControl();

	// Re-reads the device list (call on hotplug events)
	void refreshDeviceList();
	// Re-reads the mode option lists from one camera's open device
	void refreshModeOptions(int cameraIndex);

private:
	struct CameraSettingUi
	{
		eVideoSettingType type= eVideoSettingType::INVALID;
		const char* name= "";
		VideoSettingConstraint constraint;
		int value= 0;
	};

	struct CameraUiState
	{
		int selectedDeviceIndex= -1;
		std::vector<std::string> resolutionOptions;
		std::vector<std::string> frameRateOptions;
		std::vector<std::string> formatOptions;
		std::string selectedResolution;
		std::string selectedFrameRate;
		std::string selectedFormat;
		// Supported ProcAmp/CameraControl settings (cached on open/refresh)
		std::vector<CameraSettingUi> settings;
	};

	void syncCameraStateCount();
	void applyModeSelection(int cameraIndex);
	void refreshCameraSettings(int cameraIndex);

	App* m_app;
	VideoCaptureSystem* m_videoCapture;
	AppConfig* m_config;

	// Global device enumeration (shared by all camera sections)
	std::vector<std::string> m_devicePaths;
	std::vector<std::string> m_deviceNames;

	std::vector<CameraUiState> m_cameraStates;
};
