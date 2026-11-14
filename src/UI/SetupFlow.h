#pragma once

#include <memory>
#include <string>
#include <vector>

#include "NeutralCaptureControl.h"
#include "WizardResult.h"

class App;
class BodyCalibrationWizard;
class ExtrinsicsWizard;
class HandCalibrationWizard;
class IntrinsicsWizard;
class LensPatternPrompt;
class MainWindow;
class MountingWizard;
class VideoPreviewPanel;
struct WebcamTableMatch;
enum class eIntrinsicsPattern;

// Guided new-project setup: a chain of modal prompts and the existing
// calibration wizards in dependency order (cameras, intrinsics per camera,
// extrinsics, hands, then IMU mounting or body measurement by variant). The
// two single-camera variants load their VRM avatar first. The single-front
// variant is the direct retarget: one webcam facing the user, intrinsics
// for undistortion, the left and right hand check, then the neutral
// capture, with no marker and no calibration wizards. The single-desk
// variant runs one camera that sees
// the desk marker through the extrinsics, the desk-plane hand measurement
// and the body wizard.
// Runs inside the normal tracking UI so the wizards keep
// their preview-panel plumbing; the prompts are modals drawn on top, and
// manual wizard launches are suppressed while the flow is active.
//
// The flow only ever runs on a project created moments earlier, so
// cancelling anywhere (always through a confirmation, since it is
// destructive) deletes that project from disk and returns to the main menu,
// with Resume pointing back at whatever project preceded it.
class SetupFlow
{
public:
	enum class eTrackingSetup
	{
		DualOverhead,
		DualOverheadJoyCons,
		TriCameraFront,
		SingleFront, // the direct retarget: a webcam facing the user
		SingleDesk,  // one camera that sees the desk marker
	};

	enum class eStep
	{
		Inactive,
		TrackingSetup,
		AvatarLoad, // single-camera variants only
		CameraSelection,
		CharucoPrint,
		IntrinsicsRunning, // loops over every camera
		ArucoPrint,
		ExtrinsicsRunning,
		HandCalibRunning,
		MountingRunning,  // JoyCon variant only
		BodyCalibRunning, // tri-camera and single-desk variants
		HandednessCheck,  // single-front variant only
		NeutralCapture,   // single-front variant only
		ConfirmCancel,
	};

	struct WizardSet
	{
		IntrinsicsWizard* intrinsics= nullptr;
		ExtrinsicsWizard* extrinsics= nullptr;
		HandCalibrationWizard* hand= nullptr;
		MountingWizard* mounting= nullptr;
		BodyCalibrationWizard* body= nullptr;
	};

	SetupFlow(App* app, MainWindow* mainWindow, VideoPreviewPanel* previewPanel,
			  const WizardSet& wizards);
	~SetupFlow();

	void begin();
	bool isActive() const { return m_step != eStep::Inactive; }

	// Draws the current prompt, or watches the running wizard and advances on
	// its result. Called once per frame from MainWindow, after the panels and
	// before the wizard update dispatch.
	void update();

	// The chain: the step after a completed one for a variant, Inactive when
	// the flow finishes there. Pure, so the chains are self-tested.
	static eStep nextStep(eTrackingSetup setup, eStep completed);

	// How the single-front variant's lens step settles the camera's lens
	enum class eLensChoice
	{
		Table,   // the webcam table's values for this camera model
		Phone,   // calibrate on a chessboard shown by a phone
		Printed, // calibrate on the printed charuco board
		Skip,    // a pinhole from a typed field of view
	};
	// The step a lens choice completes: a calibration goes on to the
	// intrinsics wizard, and a table or skipped lens passes over it
	static eStep stepCompletedByLensChoice(eLensChoice choice);
	static int getRequiredCameraCount(eTrackingSetup setup);

private:
	void transitionTo(eStep step);
	// Moves to the chain's next step, or finishes
	void advanceFrom(eStep completed);
	void enterConfirmCancel();
	// Saves the configured project and ends the flow
	void finish();
	// Ends the flow and asks App to delete the project and return to the menu
	// (applied between frames, so nothing here runs on a torn-down project)
	void requestDiscardProject();

	void updateTrackingSetupPrompt();
	void updateAvatarLoadPrompt();
	void updateCameraSelectionPrompt();
	void updateCharucoPrintPrompt();
	// The single-front variant's form of that step
	void updateLensPrompt();
	void updateArucoPrintPrompt();
	void updateHandednessCheckPrompt();
	void updateNeutralCapturePrompt();
	void updateConfirmCancelPrompt();
	// All five *Running steps: launch the step's wizard once, then wait for it
	// to close and branch on Completed vs Cancelled
	void updateRunningWizardStep();

	// Opens the picked device in a camera slot and starts its stream, so the
	// selection prompt can show a live preview of it
	void openSelectedDevice(int slotIndex);
	// Writes the open devices into the config as the camera selections, and
	// the variant's tracking settings
	void applyCameraSelection();

	App* m_app= nullptr;
	MainWindow* m_mainWindow= nullptr;
	VideoPreviewPanel* m_previewPanel= nullptr;
	WizardSet m_wizards;

	eStep m_step= eStep::Inactive;
	eStep m_stepBeforeConfirm= eStep::Inactive;
	eTrackingSetup m_trackingSetup= eTrackingSetup::DualOverhead;

	bool m_bWizardLaunched= false;
	int m_intrinsicsCameraIndex= 0;
	NeutralCaptureControl m_neutralCapture;
	// HandednessCheck state: whether this visit to the step has switched the
	// flip setting, which it does once, and how long ago
	bool m_bHandednessFlipApplied= false;
	// Both hands have been seen on their own sides this visit. It stays set
	// once they drop, since a hand has to come down to press Next.
	bool m_bHandednessMatched= false;
	float m_handednessFlipAgeSeconds= 0.f;

	// CameraSelection state: chosen enumeration index per camera slot, -1 =
	// nothing picked yet
	std::vector<int> m_selectedDeviceIndices;

	// Print-prompt edit fields (seeded from the config on step entry)
	std::unique_ptr<LensPatternPrompt> m_patternPrompt;
	eIntrinsicsPattern m_intrinsicsPattern;
	// Lens step state: the choice, the camera's frame size and table match
	// (looked up once its first frame is in), and the typed field of view
	eLensChoice m_lensChoice= eLensChoice::Phone;
	bool m_bLensTableChecked= false;
	bool m_bLensTableMatch= false;
	std::unique_ptr<WebcamTableMatch> m_lensTableMatch;
	int m_lensFrameWidth= 0;
	int m_lensFrameHeight= 0;
	float m_nominalDiagonalFovDegrees= 78.f;
	int m_boardCols= 11;
	int m_boardRows= 8;
	float m_boardSquareLengthMM= 16.f;
	float m_boardMarkerLengthMM= 12.f;
	float m_arucoMarkerLengthMM= 100.f;
};
