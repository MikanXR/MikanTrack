#pragma once

#include <string>
#include <vector>

// The pipeline as the Pipeline panel shows it: the stages, which of them a
// project has, and what each one is doing on the shown frame. No ImGui, no
// app state, no thread: the layout is a function of the project's shape, and
// a stage's status is a function of PipelineStatusInputs, a plain struct the
// window fills from what it already holds. Text leaves as localization keys
// with numeric arguments, so the model never formats a string.

enum class ePipelineStage : int
{
	// Per-camera capture and the models that run on its frames
	Capture= 0,
	Camera, // one camera's lane; the four below are its sub-stages
	Video,
	HandModels,
	BodyModel,
	Lift,
	// The multi-camera solvers
	Fusion,
	Imu,
	BodySolver,
	// The single-camera solver that stands in for the three above
	DirectSolver,
	Filters,
	Torso,
	ArmChain,
	Palm,
	HoldRest,
	// The hand-off both pipelines share (data, drawn as a pill)
	SourcePose,
	Retarget,
	Output,
	DeliveryRules,
	VmcStream,
	// Joins at Output
	Face,
	Phone,
	Microphone,
	Profile,
	Count
};

struct PipelineStageId
{
	ePipelineStage stage= ePipelineStage::Count;
	// The camera a Camera, Video, HandModels, BodyModel or Lift stage belongs
	// to, -1 for every other stage
	int cameraIndex= -1;

	bool isValid() const { return stage != ePipelineStage::Count; }
	bool operator==(const PipelineStageId& other) const
	{
		return stage == other.stage && cameraIndex == other.cameraIndex;
	}
	bool operator!=(const PipelineStageId& other) const { return !(*this == other); }
};

enum class eStageState : int
{
	Running= 0,
	// Running, but part of what it produces is missing or held
	Degraded,
	// Not part of this project (no IMUs, no body camera), or nothing to do
	Inactive,
	// The project wants it and something it needs is missing
	NotConfigured,
};

// A localization key and the numbers its format takes. A key ending in "Fmt"
// is a printf format whose specifiers are all floating point, taken in order
// from args.
struct StageText
{
	std::string key;
	double args[3]= {0.0, 0.0, 0.0};

	bool isEmpty() const { return key.empty(); }
};

struct StageStatus
{
	eStageState state= eStageState::Inactive;
	// The one number or word the stage's box shows
	StageText value;
	// Why the stage is not simply running; empty when there is nothing to say
	StageText reason;
};

// Everything the status rules read, as plain values. The window gathers it
// once per frame from the config, the shown frame and the services.
struct PipelineStatusInputs
{
	static constexpr int k_maxCameras= 8;

	// The project's shape
	bool bDirect= false;
	int cameraCount= 0;
	int directCameraIndex= 0;
	// The shown frame is a recording's, so live-only state describes now
	bool bReplay= false;

	struct Camera
	{
		bool bDeviceOpen= false;
		bool bStreaming= false;
		float captureFps= 0.f;
		// The hand pipeline started (an execution provider is active)
		bool bHandPipelineReady= false;
		// A wizard has paused tracking on this camera
		bool bTrackingPaused= false;
		bool bIntrinsics= false;
		bool bExtrinsics= false;
		int handsTracked= 0;
		bool bBodyEnabled= false;
		bool bBodyModelLoaded= false;
		bool bBodyValid= false;
		float bodyConfidence= 0.f;
		// The body backend reports depth (the whole-body model)
		bool bBodyHasDepth= false;
	};
	Camera cameras[k_maxCameras];

	// What the vision thread's last iteration did
	enum class ePath : int
	{
		Unknown= 0,
		Idle,
		FaceOnly,
		Fused,
		Passthrough,
		Direct,
	};
	ePath path= ePath::Unknown;

	// The output frame's hands, left then right
	struct Hand
	{
		bool bTracked= false;
		bool bWorldPose= false;
		float confidence= 0.f;
		int dominantCamera= -1;
		bool bForearmPose= false;
		float forearmConfidence= 0.f;
		bool bShoulder= false;
	};
	Hand hands[2];
	bool bHeadValid= false;

	// Wrist IMUs
	bool bImuEnabled= false;
	struct Imu
	{
		bool bConnected= false;
		bool bStreaming= false;
		bool bCalibrated= false;
		bool bOrientationValid= false;
		float sampleRateHz= 0.f;
	};
	Imu imu[2];

	// The direct solver's last solve
	bool bDirectIntrinsics= false;
	bool bDirectNeutral= false;
	bool bDirectTorso= false;
	bool bDirectHeadMeasured= false;
	float directShoulderDepthMeters= 0.f;
	float directLeanDegrees= 0.f;
	struct DirectArm
	{
		bool bChain= false;
		bool bHandSeen= false;
		bool bElbowSeen= false;
		bool bHandMeasured= false;
		bool bResting= false;
		float restBlend= 0.f;
		float elbowMissPx= 0.f;
	};
	DirectArm directArms[2];

	// Retarget
	bool bAvatarLoaded= false;
	// The avatar pose placed this arm
	bool bArmPlaced[2]= {false, false};

	// Output
	bool bOscEnabled= false;
	bool bOscRunning= false;
	float oscSendRateHz= 0.f;
	float oscMinConfidence= 0.f;
	int oscPort= 0;

	// Face and mouth
	bool bFaceEnabled= false;
	bool bFaceBound= false;
	bool bFaceStreaming= false;
	bool bFaceTracked= false;
	float faceDatagramsPerSecond= 0.f;
	bool bAudioEnabled= false;
	bool bAudioDeviceOpen= false;
	bool bAudioBackendReady= false;
	bool bAudioGateOpen= false;
	float audioLevelDbfs= -120.f;
	bool bProfileAuto= true;
	int profileTargetCount= 0;
	int profileProblemCount= 0;
};

namespace PipelineModel
{
// The top row, left to right, for a project's shape. Face joins at Output
// and is not in the row (see getSideStage).
std::vector<PipelineStageId> getTopRow(bool bDirect);
PipelineStageId getSideStage();

// The lanes a stage opens into in the second row: one lane per camera for
// Capture (the camera, then its four sub-stages), one lane for the others,
// none for a stage without sub-stages
std::vector<std::vector<PipelineStageId>> getSubStageLanes(const PipelineStageId& stage, int cameraCount);

// The top-row (or side) stage a stage belongs to; itself for a top-level one
PipelineStageId getTopLevel(const PipelineStageId& stage);

// Every stage of a project, top row first, each followed by its sub-stages
std::vector<PipelineStageId> getAllStages(bool bDirect, int cameraCount);

// Whether the lanes join their boxes with arrows (a flow) or not (a set)
bool isLaneFlow(ePipelineStage topLevel);

// "pipeline.stage<Name>" and "pipeline.desc<Name>"
const char* getNameKey(ePipelineStage stage);
const char* getDescriptionKey(ePipelineStage stage);

// The stable text id automation and the dock layout use: "capture",
// "capture.cam0", "capture.cam0.hands", "direct.arm", "source", "face.mic"
std::string toString(const PipelineStageId& stage);
bool fromString(const std::string& text, PipelineStageId& outStage);

StageStatus computeStatus(const PipelineStatusInputs& inputs, const PipelineStageId& stage);

// "pipeline.state<Name>"
const char* getStateKey(eStageState state);
} // namespace PipelineModel
