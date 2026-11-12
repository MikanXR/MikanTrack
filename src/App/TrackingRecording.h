#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "AudioTypes.h"
#include "TrackingTypes.h"

// Tracking recording format: everything the post-MediaPipe stages consume,
// captured per fusion iteration, plus the fused output and a checksum, so the
// whole LandmarkTo3D -> HandFusion -> smoothing pipeline can be re-run offline
// bit-exactly. JSONL on disk: one header line, one line per frame, one footer
// line. nlohmann's float round-trip is lossless, which is what makes replayed
// checksums comparable to recorded ones.
//
// Non-finite floats serialize as null (nlohmann behavior) and load back as 0;
// a NaN that reached the recorded values means live consumed it too, so the
// resulting checksum mismatch is the honest outcome, not corruption.

// Pipeline-output fields for one hand, exactly what LandmarkTo3D::process
// consumes (plus imageQuality, carried for timeline display/analysis only)
struct RecordedHandInput
{
	bool tracked= false;
	int side= 0;
	int slotId= -1;
	float presence= 0.f;
	float handednessScore= 0.f;
	float rightProb= 0.5f;
	std::array<glm::vec3, HAND_LANDMARK_COUNT> imagePoints{};
	std::array<glm::vec3, HAND_LANDMARK_COUNT> modelPoints{};
	HandImageQuality imageQuality;
};

// One camera that popped a frame this iteration (stale cameras record
// nothing; replay carries their previous mirror forward)
struct RecordedCameraInput
{
	int cameraIndex= -1;
	double timestampMs= 0.0;
	// bProducedTracking: false = the camera ticked but tracking was disabled
	// or the pipeline was down; replay updates the mirror's timestamp only
	bool valid= false;
	// The hand scale actually in effect for this camera's LandmarkTo3D call
	// (records the auto-scale EMA feedback loop as a plain input)
	float refLengthMeters= 0.f;

	int64_t frameIndex= -1;
	int frameWidth= 0;
	int frameHeight= 0;
	float captureFps= 0.f;
	float inferenceMs= 0.f;
	float lumaInstability= 0.f;
	float lumaFlickerHz= 0.f;
	// The capture stage's step times for this frame, display only like the
	// fields above (absent in recordings made before they were carried)
	bool bHaveCaptureTimings= false;
	TrackingFrameResult::CaptureTimings captureTimings;

	std::array<RecordedHandInput, 2> hands;

	// Body-pose observation from this camera's opt-in body-pose stage. The
	// stage runs upstream of this tap, so divider cadence (held re-emits) is
	// baked in and replay never re-runs the pose models.
	bool bHaveBodyPose= false;
	BodyPoseObservation body;
};

// Fused pose snapshot taken immediately after fuse(), BEFORE the IMU forearm
// fill. Full skeleton included so the recorded output renders via FK.
struct RecordedPoseOutput
{
	bool tracked= false;
	int side= 0;
	float presence= 0.f;
	float visibility= 0.f;
	float confidence= 0.f;
	float fkReprojectionPx= 0.f;
	bool stereoTriangulated= false;
	bool hasCameraPose= false;
	glm::vec3 palmPositionCamera{0.f};
	glm::quat palmOrientationCamera{1.f, 0.f, 0.f, 0.f};
	bool hasWorldPose= false;
	glm::vec3 palmPositionWorld{0.f};
	glm::quat palmOrientationWorld{1.f, 0.f, 0.f, 0.f};
	std::array<FingerAngles, FINGER_COUNT> fingers{};
	HandSkeleton skeleton;
};

// The IMU forearm output actually published this frame. Replay overlays it
// (fusion never reads IMU; the EKF is deliberately not re-run offline).
struct RecordedImuOutput
{
	bool hasForearmPose= false;
	glm::quat forearmOrientationWorld{1.f, 0.f, 0.f, 0.f};
	float forearmConfidence= 0.f;
};

// The microphone mouth stage's newest frame at the iteration. Recorded as
// the stage emitted it (outside the checksum, like the IMU output), so a
// backend change can be measured against old recordings through the audio
// sidecar.
struct RecordedMouth
{
	bool present= false;
	// The frame's index in the audio sidecar's stream, or -1 when the sidecar
	// does not hold its audio (no sidecar, or the device reopened)
	int64_t audioFrameIndex= -1;
	MouthFrame frame;
};

// Stage state of the iteration that produced a recorded frame, for
// display in a replay. No replayed stage reads it and the checksum does
// not cover it.
struct RecordedImuStatus
{
	bool deviceConnected= false;
	bool streaming= false;
	bool calibrated= false;
	bool orientationValid= false;
	float sampleRateHz= 0.f;
	float batteryLevel= 0.f;
	float yawDriftDegPerSecond= 0.f; // the gyro bias about the yaw axis, as the Tracking panel reads it
	float rollErrorDegrees= 0.f;     // the wrist axial twist, the mounting roll error
};

struct RecordedTelemetry
{
	static constexpr int k_maxCameras= 8;
	static constexpr int k_phaseCount= 8; // VisionThread::eVisionPhase::Count

	RecordedTelemetry()
	{
		for (int cameraIndex= 0; cameraIndex < k_maxCameras; ++cameraIndex)
		{
			observationConfidence[cameraIndex][0]= -1.f;
			observationConfidence[cameraIndex][1]= -1.f;
		}
	}

	bool present= false;
	int path= 0; // VisionThread::Telemetry::ePath
	float phaseMs[k_phaseCount]= {};
	float totalMs= 0.f;

	bool oscRunning= false;
	float oscSendRateHz= 0.f;
	int oscLastPacketCount= 0;
	int oscLastFrameBytes= 0;

	int dominantCamera[2]= {-1, -1};
	float observationConfidence[k_maxCameras][2]; // -1 where a camera gave no observation
	float autoHandScaleFactor= 1.f;

	struct Camera
	{
		bool deviceOpen= false;
		bool streaming= false;
		float deviceFrameRate= 0.f;
		int droppedFrames= 0;
		int queuedFrames= 0;
		float callbackCopyMs= 0.f;
		bool trackingEnabled= true;
		bool handPipelineReady= false; // an execution provider is active
		bool bodyModelLoaded= false;
	};
	int cameraCount= 0;
	Camera cameras[k_maxCameras];

	bool imuEnabled= false;
	RecordedImuStatus imu[2];

	struct Face
	{
		bool enabled= false;
		bool bound= false;
		bool streaming= false;
		bool faceTracked= false;
		float datagramsPerSecond= 0.f;
	};
	Face face;

	struct Audio
	{
		bool enabled= false;
		bool deviceOpen= false;
		bool backendReady= false;
		bool gateOpen= false;
		float levelDbfs= -100.f;
		float inferMsEma= 0.f;
	};
	Audio audio;
};

struct RecordedFrame
{
	int64_t seq= -1;
	// The newestTimestampMs passed to fuse() (recorded, never re-derived)
	double nowTimestampMs= 0.0;
	// false = the no-world passthrough path (camera 0's result passed through
	// without a fuse() call)
	bool bFused= false;
	// Cameras that popped a frame this iteration, ascending camera index (the
	// live loop's sequential processing order)
	std::vector<RecordedCameraInput> freshCameras;
	std::array<RecordedPoseOutput, 2> outPoses;
	std::array<RecordedImuOutput, 2> imu;
	RecordedMouth mouth;
	uint64_t checksum= 0;
	// Absent (present false) in recordings made before it was carried
	RecordedTelemetry telemetry;
};

// The audio sidecar beside the recording: the microphone as captured, 16 kHz
// mono PCM, named after the recording. Its first sample is stream sample
// startSampleIndex, which is what places a frame's audioFrameIndex in it.
struct RecordedAudioSidecar
{
	bool present= false;
	// The file name alone, resolved beside the recording
	std::string fileName;
	int sampleRate= 0;
	int64_t startSampleIndex= 0;
	// steadyNowMs() when the sidecar began
	double startTimestampMs= 0.0;
};

// The face sidecar beside the recording: the phone's face samples as the
// vision thread saw them, one line each at the phone's rate, named after the
// recording. The header names it for every recording, and the file is deleted
// when the phone sent no sample, so an absent file means no phone.
struct RecordedFaceSidecar
{
	bool present= false;
	// The file name alone, resolved beside the recording
	std::string fileName;
	// steadyNowMs() when the sidecar began
	double startTimestampMs= 0.0;
};

struct RecordingHeader
{
	int formatVersion= 0;
	// The AppConfig snapshot (same schema config.json uses), reparsable via
	// AppConfig::loadFromJsonString
	std::string appConfigJsonText;
	int cameraCount= 0;
	RecordedAudioSidecar audio;
	RecordedFaceSidecar face;
};

struct RecordingFooter
{
	int64_t frameCount= 0;
	bool bAborted= false;
	std::string reason;
};

namespace TrackingRecording
{
constexpr int k_formatVersion= 1;

// FNV-1a 64 over the fused output's raw little-endian float bytes, per side
// Left then Right: 1 byte tracked; if tracked, 1 byte hasWorldPose, then 28
// floats in fixed order: palm position xyz, palm orientation xyzw (world when
// hasWorldPose, else camera), the 20 finger angles, confidence. Computed on
// the PRE-IMU fused result by both the live tap and replay.
uint64_t computeFusedChecksum(const TrackingFrameResult& preImuFused);

// Fills the pose snapshots from a (pre-IMU) fused result
void snapshotFusedOutput(const TrackingFrameResult& fused,
						 std::array<RecordedPoseOutput, 2>& outPoses);

// Raw frames live in a sibling directory named after the recording, so a
// recording and its frames travel together and either can be deleted alone:
// <dir>/2026-01-01_12-00-00.jsonl -> <dir>/2026-01-01_12-00-00_frames
std::string makeFrameDirectoryPath(const std::string& recordingFilePath);
// The audio sidecar sits beside the recording under its name:
// <dir>/2026-01-01_12-00-00.jsonl -> <dir>/2026-01-01_12-00-00.wav
std::string makeAudioSidecarPath(const std::string& recordingFilePath);
// The face sidecar likewise:
// <dir>/2026-01-01_12-00-00.jsonl -> <dir>/2026-01-01_12-00-00.face.jsonl
std::string makeFaceSidecarPath(const std::string& recordingFilePath);
// Whether a path names one of the files above that the recorder writes
// beside a take (the face or the audio sidecar), by its name alone
bool isRecordingSidecarFile(const std::string& path);
// Whether a path names a take: a .jsonl file that is not a sidecar
bool isRecordingFile(const std::string& path);

nlohmann::json headerToJson(const RecordingHeader& header);
bool headerFromJson(const nlohmann::json& j, RecordingHeader& outHeader);
nlohmann::json frameToJson(const RecordedFrame& frame);
bool frameFromJson(const nlohmann::json& j, RecordedFrame& outFrame);
nlohmann::json footerToJson(const RecordingFooter& footer);
bool footerFromJson(const nlohmann::json& j, RecordingFooter& outFooter);
} // namespace TrackingRecording
