#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "glm/ext/vector_double3.hpp"

#include "Countdown.h"
#include "DirectRetarget.h" // DirectRetarget::SolveState
#include "ImuService.h"     // ImuSideStatus
#include "NeutralCaptureControl.h"
#include "SourcePose.h"
#include "TrackingTypes.h"

class AppConfig;
class VisionThread;
class VideoCaptureSystem;
class VideoPreviewPanel;
class Scene3dPanel;
struct FusionDiagnostics;
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

// What fusion reported for the shown frame
struct FusionReadout
{
	static constexpr int k_maxCameras= 8;

	FusionReadout()
	{
		for (int cameraIndex= 0; cameraIndex < k_maxCameras; ++cameraIndex)
		{
			observationConfidence[cameraIndex][0]= -1.f;
			observationConfidence[cameraIndex][1]= -1.f;
		}
	}

	bool bAvailable= false; // false: a replayed frame from a take that recorded no stage state
	// The cameras of the shown frame's config
	int cameraCount= 0;
	int dominantCamera[2]= {-1, -1};
	float observationConfidence[k_maxCameras][2]; // -1 where a camera gave no observation
	float autoHandScaleFactor= 1.f;
	const FusionDiagnostics* diagnostics= nullptr; // null when none is available
	const TrackingFrameResult* fused= nullptr;     // the shown fused result
	// The shown frame is a replayed one
	bool bReplay= false;
};

// What the wrist IMUs reported for the shown frame
struct ImuReadout
{
	bool bAvailable= false; // false: a replayed frame from a take that recorded no stage state
	bool bReplay= false;
	ImuSideStatus sides[2];
};

// What the phone's face stream reported for the shown frame. A recording
// carries the switch, the bind, streaming and the rate; the rest is live only
// and unset in a replay.
struct FaceReadout
{
	bool bAvailable= false; // false: a replayed frame from a take that recorded no stage state
	bool bReplay= false;
	bool bEnabled= false;
	bool bBound= false;
	bool bStreaming= false;
	float datagramsPerSecond= 0.f;
	// Live only: a sample has arrived (quiet rather than waiting), the
	// parser's counts, and the newest sample's head rotation
	bool bSampleValid= false;
	uint64_t unknownNameCount= 0;
	uint64_t parseFailures= 0;
	std::string lastUnknownName;
	bool bHasHead= false;
	glm::vec3 headEulerDegrees{0.f};
};

// The direct solve behind the shown avatar pose
struct DirectReadout
{
	// Null when the shown frame has no direct solve: no avatar, no
	// intrinsics, a preview pose, or a frame that is not a direct one
	const DirectRetarget::SolveState* state= nullptr;
	// The shoulder midpoint's distance from the camera, avatar units, and the
	// torso's lean against the neutral, degrees. Zero without a solve.
	float shoulderDepth= 0.f;
	float leanDegrees= 0.f;
	// The shown frame's config holds a neutral pose
	bool bNeutralPresent= false;
};

// The source pose behind the shown avatar pose, as the loss stage handed it to
// the target solve
struct SourcePoseReadout
{
	// Null when the shown frame has none: a replay with no avatar loaded, the
	// direct Rest preview, or live before the vision thread's first frame
	const SourcePose* source= nullptr;
};

// What the Tracking panel's sections read and write. latestPreviews: the
// newest per-camera results (image-quality and timing readouts); fusedResult:
// the latest live fused tracking output (hold-still jitter test); fusion: the
// shown frame's fusion readouts (the Fusion stage's diagnostics); imu: the
// shown frame's wrist IMU status (the IMU stage's device table); face: the
// shown frame's phone stream status (the Phone stage's diagnostics); direct:
// the shown frame's direct solve (the Direct solver stages' diagnostics);
// sourcePose: the shown frame's source pose (the Source pose stage's
// diagnostics).
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
	const FusionReadout* fusion= nullptr;
	const ImuReadout* imu= nullptr;
	const FaceReadout* face= nullptr;
	const DirectReadout* direct= nullptr;
	const SourcePoseReadout* sourcePose= nullptr;
};

// What the OSC stream reported for the shown frame
struct OscReadout
{
	bool bAvailable= false; // false: a replayed frame from a take that recorded no stage state
	bool bReplay= false;
	bool bRunning= false; // the streamer exists and its socket is open
	float framesPerSecond= 0.f;
	// The datagrams and total bytes of the last frame sent
	int lastPacketCount= 0;
	int lastFrameBytes= 0;
};

// What the OSC sections read and write. readout: the shown frame's stream
// status (the Output stage's diagnostics).
struct OscSectionContext
{
	AppConfig* config= nullptr;
	VisionThread* visionThread= nullptr;
	const OscReadout* readout= nullptr;
};

// Small side panels and hostable sections: tracking/fusion options and the
// OSC output's settings and readouts. Config edits mark the config dirty and
// request a vision-thread refresh.
// Each panel is a sequence of sections, and each section can be drawn on its
// own in any window, more than once a frame: a section's widgets only read
// the panel state and raise requests, and a section commits its own config
// edits.
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

// The stages' sections and pieces. Flip handedness, image quality, inference
// and capture timing are the Capture stage's; fusion, hand scale, the fused
// hands, the fusion detail, the per-camera skeletons toggle and the
// hold-still test are the Fusion stage's; the wrist IMU pieces are the IMU
// stage's; the body pieces are the Body solver stage's; the face stream
// pieces are the Phone stage's; and the direct pieces, with the hand
// calibration block, Measure My Body, the body proportions and the
// hold-still test again, are the Direct solver stages'. The Tracking panel
// itself draws only the direct mode switch while direct mode is off and the
// dump buttons.
void drawFlipHandednessSection(const TrackingSectionContext& context);
// Fusion and Hand Scale read context.fusion and draw only when the shown
// frame has more than one camera
void drawFusionSection(const TrackingSectionContext& context);
void drawHandScaleSection(const TrackingSectionContext& context);
// Each hand's palm position in the shown fused result
void drawFusedPalmReadout(const TrackingSectionContext& context);
// One row per cluster fusion assigned a side on the shown frame
void drawFusionDiagnosticsSection(const TrackingSectionContext& context);
void drawImageQualitySection(const TrackingSectionContext& context);
// The wrist IMU pieces, no headers. The controls edit the live config and
// all but the enable checkbox are disabled while the IMU is off. The device
// table reads context.imu.
void drawWristImuEnableControl(const TrackingSectionContext& context);
void drawWristImuArmControls(const TrackingSectionContext& context);
void drawWristImuDeviceTable(const TrackingSectionContext& context);
void drawMountingCalibrationControl(const TrackingSectionContext& context);
// The forearm length slider, which the wrist controls draw and the direct
// solve reads too, with the host's tooltip
void drawForearmLengthControl(const TrackingSectionContext& context, const char* tooltipKey);
// Each hand's wrist bend and forearm and wrist rotations in the shown fused
// result (context.fusion->fused)
void drawWristReadoutSection(const TrackingSectionContext& context);
// The phone face stream pieces, the Phone stage's, no headers. The stream
// controls (enable, port, phone address) and the head alignment edit the
// live config; the alignment draws only while the stream is enabled. The
// status reads context.face and draws only while the shown frame's stream is
// enabled.
void drawFaceStreamControls(const TrackingSectionContext& context);
void drawFaceHeadAlignControls(const TrackingSectionContext& context);
void drawFaceStreamStatus(const TrackingSectionContext& context);
// The body pieces, no headers: the proportions intro with Measure My Body,
// and the proportion controls, which edit the live config. The per-camera
// enables are drawBodyPoseCameraControls.
void drawBodyCalibrationControl(const TrackingSectionContext& context);
void drawBodyDimensionControls(const TrackingSectionContext& context);
// Each side's shoulder, elbow and forearm confidence, and the head, in the
// shown fused result (context.fusion->fused), no header
void drawBodySolveReadout(const TrackingSectionContext& context);
// What body tracking is for and which cameras should run it
void drawBodyPoseIntro();
// One camera's body pose enable and, when enabled, its model. Draws nothing
// for a camera index the config does not have.
void drawBodyPoseCameraControls(const TrackingSectionContext& context, int cameraIndex);
// The direct mode switch as the Tracking panel shows it while direct mode is
// off: the intro and the enable, under the Direct Retarget heading. A
// multi-camera project has no Direct solver stage to turn the mode on from.
void drawDirectEnableSection(const TrackingSectionContext& context);
// The direct pieces, no headers, editing the live config. Enable: the mode
// switch, which also cancels a neutral countdown when it turns the mode off.
// Camera: keeps the direct camera index in range and draws the camera combo
// only with more than one camera. Neutral: Capture neutral and Clear, the
// countdown updateTrackingState runs, and the neutral's state line. Tilt: the
// camera's downward pitch.
void drawDirectEnableControl(const TrackingSectionContext& context);
void drawDirectCameraControl(const TrackingSectionContext& context);
void drawDirectNeutralControl(const TrackingSectionContext& context);
void drawDirectTiltControl(const TrackingSectionContext& context);
// The direct readouts, reading the shown frame's solve (context.direct), one
// dim line when the shown frame has none. Summary: whether a torso is seen
// and the camera model and tilt the solve used, under its own heading.
// Filters: each side's hand slot agreement and elbow visibility gate. Torso:
// the person, the shoulder depth, the lean, the head and the neutral. Arm
// chain: each side's reach interval, chosen and filtered depth, elbow miss,
// model depth prior and hand lift. Palm: each side's hand measurement and
// forearm twist. Hold and rest: each side live or resting and how far into
// the rest pose.
void drawDirectSolveSummary(const TrackingSectionContext& context);
void drawDirectFiltersReadout(const TrackingSectionContext& context);
void drawDirectTorsoReadout(const TrackingSectionContext& context);
void drawDirectArmChainReadout(const TrackingSectionContext& context);
void drawDirectPalmReadout(const TrackingSectionContext& context);
void drawDirectHoldRestReadout(const TrackingSectionContext& context);
// The hand calibration state, the wizard button (the direct mode note in a
// direct project) and Clear, under its heading
void drawHandCalibrationSection(const TrackingSectionContext& context);
// The view toggles, no headers. Hand: the preview's skeleton overlay,
// detection boxes and, in direct mode, every hand slot. Body: the preview's
// body landmarks. Per-camera skeletons: the 3D scene's unfused skeletons,
// drawn only with more than one camera.
void drawHandOverlayControls(const TrackingSectionContext& context);
void drawBodyOverlayControls(const TrackingSectionContext& context);
void drawPerCameraSkeletonsControl(const TrackingSectionContext& context);
void drawInferenceSection(const TrackingSectionContext& context);
// Draws nothing without a camera
void drawCaptureTimingSection(const TrackingSectionContext& context);
// The dump buttons
void drawDiagnosticsSection(const TrackingSectionContext& context);
// The hold-still jitter test: its button, countdown, progress and result
void drawHoldStillSection(const TrackingSectionContext& context);

// The OSC stream pieces, no headers. The controls edit the live config.
// The Output stage's enable, the VMC stream stage's target and head offset,
// and the Delivery rules stage's rate and confidence gate.
void drawOscEnableControl(const OscSectionContext& context);
void drawOscTargetControls(const OscSectionContext& context);
// The head offset and the notes on the streamed bones
void drawOscVmcControls(const OscSectionContext& context);
void drawOscDeliveryControls(const OscSectionContext& context);
// Whether the stream runs, its frame rate and the last frame's size, from
// context.readout
void drawOscStreamReadout(const OscSectionContext& context);
// Each hand's confidence against the live confidence gate, red while under
// it. With a source pose, its hand's confidence (which decays across a hold)
// beside its arm's state after the loss stage; without one (null), fused's
// hand confidence, red while the hand is withheld.
void drawOscDeliveryReadout(const OscSectionContext& context, const TrackingFrameResult& fused,
							const SourcePose* source);
// The Source pose stage's diagnostics, reading the shown frame's source pose
// (context.sourcePose), one dim line when the shown frame has none: a
// summary (the pipeline and its units, the finger zero, the clock), one row
// per group with its state, confidence and what it carries, and each hand's
// finger angles in degrees as the target solve takes them
void drawSourcePoseReadout(const TrackingSectionContext& context);
// The Retarget stage's diagnostics: one row per chain with its state after
// the loss stage, from the shown frame's source pose (context.sourcePose),
// and for the arms the IK weight the target solve runs at, shown only with
// an avatar (bAvatar). One dim line when the shown frame has no source pose.
void drawRetargetChainReadout(const TrackingSectionContext& context, bool bAvatar);

// Draws the sections only; updateTrackingState runs the panel's state
void drawTrackingPanel(bool* pOpen, const TrackingSectionContext& context);
// App-level settings dock window (language selection, app-global options)
void drawAppSettingsPanel(bool* pOpen);
// The language combo alone, shared between the Settings panel and the main
// menu. Draws nothing when no LocalizationManager exists.
void drawLanguageCombo();
}
