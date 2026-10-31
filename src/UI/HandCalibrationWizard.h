#pragma once

#include <vector>

#include "DeskHandCalibrator.h"
#include "VisionThread.h" // BoneCalibrationCapture / RestPoseCapture / VisionPreviewFrame
#include "WizardResult.h"

class AppConfig;
struct TrackingFrameResult;

// One guided flow for both hand calibrations, in the only order that works:
// bones first, rest pose second. The order is the point of the wizard -
// saving measured bones moves the THUMB's angle zero (its neutral direction
// follows the metacarpal, unlike the four fingers, which are pinned to palm
// forward), so a rest pose captured before a bone save is captured against a
// zero that no longer exists. Two separate buttons made that mistake easy;
// one flow makes it impossible.
//
// Both stages need a measurement the landmark model cannot bias. With two or
// more cameras that is stereo: bones are medians over triangulated landmarks
// (a monocular pose carries the landmark model's shape, the thing being
// replaced), and the rest capture reads the raw multi-view angles of the fuse
// that services it. With ONE camera the desk stands in for the second view:
// the bones come from hands laid flat on the desk, lifted onto its known
// plane (DeskHandCalibrator, sampled here on the main thread from the
// camera's previews), which also measures the camera's lift depth scale; and
// the rest zero is the fit's own angles, so the view's bias cancels at rest.
//
// Leaves tracking running, like the mounting and body wizards: the tracked
// hands ARE the measurement.
class HandCalibrationWizard
{
public:
	enum class eState
	{
		BonesIntro,     // instructions + live stereo or desk status
		BonesCountdown,
		BonesSampling,  // vision thread (stereo) or this wizard (desk) samples
		BonesReview,    // measured numbers; Accept writes skeleton + scale
		RestIntro,      // instructions (flat hands)
		RestCountdown,
		RestResult,     // per-side captured/missed; Retry or Done
	};

	HandCalibrationWizard(AppConfig* config, VisionThread* visionThread);

	void enter();
	void exit();
	bool isActive() const { return m_bActive; }
	eWizardResult getResult() const { return m_result; }
	// True when this run measures from the desk plane (one-camera project)
	bool isDeskMode() const { return m_bDeskMode; }

	// Returns false once the wizard wants to close
	bool update(float deltaSeconds, const std::vector<VisionPreviewFrame>& previews,
				const TrackingFrameResult& fusedResult);

private:
	void drawBonesIntro(const TrackingFrameResult& fusedResult);
	void drawBonesCountdown(float deltaSeconds);
	void drawBonesSampling(float deltaSeconds);
	void drawBonesReview();
	void drawRestIntro(const TrackingFrameResult& fusedResult);
	void drawRestCountdown(float deltaSeconds);
	void drawRestResult();

	// Desk mode: the one camera's calibration around its latest preview, the
	// same shape the vision thread builds for fusion
	bool makeDeskCameraFrame(CameraFrameResult& outCamera) const;
	void drawDeskStatusLines() const;

	void saveBones();
	void saveRest();

	AppConfig* m_config= nullptr;
	VisionThread* m_visionThread= nullptr;

	bool m_bActive= false;
	bool m_bWantsClose= false;
	eWizardResult m_result= eWizardResult::None;
	eState m_state= eState::BonesIntro;
	float m_countdown= 0.f;

	VisionThread::BoneCalibrationCapture m_boneCapture;
	VisionThread::RestPoseCapture m_restCapture;

	bool m_bDeskMode= false;
	const std::vector<VisionPreviewFrame>* m_previews= nullptr;
	DeskHandCalibrator m_deskCalibrator;
	DeskHandCalibrator::SampleStatus m_deskStatus;
	DeskHandCalibrator::Result m_deskResult;
	float m_deskSamplingRemaining= 0.f;
};
