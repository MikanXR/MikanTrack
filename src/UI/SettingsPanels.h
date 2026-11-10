#pragma once

#include <array>
#include <memory>
#include <vector>

#include "glm/ext/vector_double3.hpp"

#include "Countdown.h"
#include "NeutralCaptureControl.h"
#include "TrackingTypes.h"

class AppConfig;
class AvatarMouthReadout;
class FaceProfileBinding;
class VisionThread;
class VideoCaptureSystem;
class VideoPreviewPanel;
class Scene3dPanel;
struct AvatarModel;
struct VisionPreviewFrame;

// Frame-to-frame state for the tracking panel (owned by MainWindow, since the
// panel itself is a set of free functions)
struct TrackingPanelState
{
	// Set when the panel wants a guided multi-step wizard opened; the owner
	// consumes and clears these (a wizard is not something a panel button can
	// drive by itself)
	bool bLaunchMountingWizard= false;
	bool bLaunchBodyCalibrationWizard= false;
	bool bLaunchHandCalibrationWizard= false;

	// UI smoothing for the image-quality readout: the per-frame values are
	// twitchy at camera rate, so the panel shows a ~1s EMA (the dump keeps the
	// raw per-frame series). Indexed [camera][metric row].
	static constexpr int kQualityMetricCount= 8;
	static constexpr int kQualityMaxCameras= 8;
	float qualityEma[kQualityMaxCameras][kQualityMetricCount]= {};
	bool bQualityEmaValid[kQualityMaxCameras][kQualityMetricCount]= {};

	// The same EMA for the capture-timing readout. Indexed [camera][row].
	static constexpr int kTimingRowCount= 13;
	float timingEma[kQualityMaxCameras][kTimingRowCount]= {};
	bool bTimingEmaValid[kQualityMaxCameras][kTimingRowCount]= {};

	// Hold-still jitter test: countdown, then a sampling window over the
	// fused output. The result is THE number for A/B-ing camera settings and
	// lighting changes - one repeatable measurement per configuration.
	float holdStillCountdown= 0.f;   // seconds before sampling starts; <= 0 idle
	float holdStillSecondsLeft= 0.f; // seconds left in the sampling window
	double holdStillLastTimestampMs= 0.0; // dedupe: fused results arrive slower than UI frames
	struct HoldStillAccum
	{
		int sampleCount= 0;
		std::array<glm::dvec3, HAND_LANDMARK_COUNT> sum{};
		std::array<glm::dvec3, HAND_LANDMARK_COUNT> sumSq{};
	};
	HoldStillAccum holdStillAccum[2];
	bool bHoldStillHasResult= false;
	// Mean per-landmark deviation over the window, mm; -1 = too few samples
	float holdStillDeviationMm[2]= {-1.f, -1.f};

	// Direct retarget neutral capture
	NeutralCaptureControl directNeutral;

	// The diagnostic dump buttons: the owner runs the dump (it needs the
	// posed avatar, which the panel does not hold), at once or after the
	// settle countdown the panel draws
	bool bDumpRequested= false;
	bool bDelayedDumpRequested= false;
	Countdown dumpCountdown;
};

// What the Tracking panel's sections read and write. latestPreviews: the
// newest per-camera results (image-quality and timing readouts); fusedResult:
// the latest live fused tracking output (hold-still jitter test).
struct TrackingSectionContext
{
	AppConfig* config= nullptr;
	VisionThread* visionThread= nullptr;
	VideoCaptureSystem* videoCapture= nullptr;
	VideoPreviewPanel* previewPanel= nullptr;
	Scene3dPanel* scene3dPanel= nullptr;
	TrackingPanelState* state= nullptr;
	const std::vector<VisionPreviewFrame>* latestPreviews= nullptr;
	const TrackingFrameResult* fusedResult= nullptr;
};

// What the OSC panel's sections read and write. fusedResult: the latest
// fused tracking output (the live angle readout and the mouth section, which
// mouthReadout draws from the loaded avatar and its face profile binding).
struct OscSectionContext
{
	AppConfig* config= nullptr;
	VisionThread* visionThread= nullptr;
	const TrackingFrameResult* fusedResult= nullptr;
	AvatarMouthReadout* mouthReadout= nullptr;
	std::shared_ptr<const AvatarModel> avatar;
	std::shared_ptr<const FaceProfileBinding> faceBinding;
};

// Small side panels: tracking/fusion options and OSC output settings.
// Config edits mark the config dirty and request a vision-thread refresh.
// Each panel is a sequence of sections, and each section can be drawn on its
// own in any window, more than once a frame: a section's widgets only read
// the panel state and raise requests, and a section commits its own config
// edits. bWithHeader false leaves out the section's separator heading, for a
// host that supplies its own.
namespace SettingsPanels
{
// The Tracking panel's per-frame work, run once per frame whether or not any
// of its sections is drawn, before they are: the neutral capture countdown
// and its write-back, the face head-anchor fetch, the hold-still jitter test,
// and the image-quality and capture-timing readout smoothing
void updateTrackingState(const TrackingSectionContext& context, float deltaSeconds);
// Marks the config dirty and requests a vision-thread refresh
void commitTrackingConfigChange(const TrackingSectionContext& context);
void commitOscConfigChange(const OscSectionContext& context);

// The Tracking panel's sections, in the panel's order
void drawFlipHandednessSection(const TrackingSectionContext& context);
// Fusion and Hand Scale draw only with more than one camera
void drawFusionSection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawHandScaleSection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawImageQualitySection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawWristImuSection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawFaceSection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawBodyPoseSection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawDirectSection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawHandCalibrationSection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawOverlaySection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawInferenceSection(const TrackingSectionContext& context, bool bWithHeader= true);
// Draws nothing without a camera
void drawCaptureTimingSection(const TrackingSectionContext& context, bool bWithHeader= true);
// The dump buttons, then the hold-still jitter test
void drawDiagnosticsSection(const TrackingSectionContext& context, bool bWithHeader= true);
void drawHoldStillSection(const TrackingSectionContext& context);

// The OSC panel's sections, in the panel's order
void drawOscConnectionSection(const OscSectionContext& context);
void drawOscVmcSection(const OscSectionContext& context, bool bWithHeader= true);
void drawOscMouthSection(const OscSectionContext& context, bool bWithHeader= true);
void drawOscSolverSection(const OscSectionContext& context, bool bWithHeader= true);

// latestPreviews: newest per-camera results (image-quality and timing readouts);
// fusedResult: the latest fused tracking output (hold-still jitter test).
// Draws the sections only; updateTrackingState runs the panel's state.
void drawTrackingPanel(bool* pOpen, AppConfig* config, VisionThread* visionThread, VideoCaptureSystem* videoCapture,
					   VideoPreviewPanel* previewPanel, Scene3dPanel* scene3dPanel, TrackingPanelState& panelState,
					   const std::vector<VisionPreviewFrame>& latestPreviews,
					   const TrackingFrameResult& fusedResult);
// fusedResult: the latest fused tracking output (for the live angle readout
// and the mouth section, which mouthReadout draws from the loaded avatar and
// its face profile binding)
void drawOscPanel(bool* pOpen, AppConfig* config, VisionThread* visionThread, const TrackingFrameResult& fusedResult,
				  AvatarMouthReadout& mouthReadout, const std::shared_ptr<const AvatarModel>& avatar,
				  const std::shared_ptr<const FaceProfileBinding>& faceBinding);
// App-level settings dock window (language selection, app-global options)
void drawAppSettingsPanel(bool* pOpen);
// The language combo alone, shared between the Settings panel and the main
// menu. Draws nothing when no LocalizationManager exists.
void drawLanguageCombo();
}
