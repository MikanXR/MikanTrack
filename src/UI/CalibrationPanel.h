#pragma once

class AppConfig;

// Calibration status panel: per-camera intrinsics/extrinsics state plus the
// global hand scale, and launches the wizards (launch requests are returned
// to MainWindow with the camera they apply to).
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

	DrawResult draw(bool* pOpen, bool bWizardActive);
	// The panel's contents without its window, for a host window
	DrawResult drawBody(bool bWizardActive);

	// The panel's sections, each drawable in a host window and each
	// returning the launches it raised. A camera section draws nothing for a
	// camera index the config does not have.
	DrawResult drawCameraCalibration(int cameraIndex, bool bWizardActive);
	DrawResult drawExtrinsicsSection(bool bWizardActive);
	void drawHandScaleSection();

private:
	AppConfig* m_config;
};
