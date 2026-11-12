#pragma once

class AppConfig;

// Calibration status: per-camera intrinsics/extrinsics state plus the global
// hand scale, drawn by the host window that shows the stages they feed, and
// launches the wizards (launch requests are returned to the host with the
// camera they apply to).
class CalibrationPanel
{
public:
	explicit CalibrationPanel(AppConfig* config);

	struct DrawResult
	{
		bool bLaunchIntrinsicsWizard= false;
		bool bLaunchExtrinsicsWizard= false;
		int cameraIndex= 0;
	};

	// The sections, each drawable in a host window and each returning the
	// launches it raised. A camera section draws nothing for a camera index
	// the config does not have, and bWithHeader false leaves out its
	// "Camera N" collapsing header, for a host that names the camera.
	DrawResult drawCameraCalibration(int cameraIndex, bool bWizardActive, bool bWithHeader= true);
	DrawResult drawExtrinsicsSection(bool bWizardActive);
	void drawHandScaleSection();

private:
	AppConfig* m_config;
};
