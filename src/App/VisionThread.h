#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "AvatarRig.h"
#include "BodyPoseSolver.h"
#include "CameraContext.h"
#include "DiagnosticDump.h"
#include "DirectRetarget.h"
#include "FaceProfileRunner.h"
#include "FaceRecorder.h"
#include "FrameRecorder.h"
#include "ImuService.h"
#include "HandBoneCalibrator.h"
#include "HandPoseModel.h"
#include "HandFusion.h"
#include "IterationHistory.h"
#include "TrackingRecorder.h"
#include "TrackingTypes.h"

class AppConfig;
class AvatarSkeleton;
class AudioService;
class FaceService;
class VideoCaptureSystem;
class OscStreamer;

// Owns the inference thread: drains each camera's frames, converts to BGR,
// optionally undistorts, runs that camera's ML pipeline and 3D projection,
// fuses all cameras' world-space results (visibility-weighted softmax blend),
// smooths the fused output, streams it over OSC and publishes previews.
//
// Cameras are processed sequentially on one thread: DirectML serializes on a
// single GPU queue anyway, and the per-camera CameraContext boundary keeps a
// later thread-per-camera upgrade possible.
class VisionThread
{
public:
	VisionThread(VideoCaptureSystem* videoCapture, ImuService* imuService, FaceService* faceService,
				 AudioService* audioService, AppConfig* config);
	~VisionThread();

	// Sizes the per-camera contexts from the config's camera count and spawns
	// the thread. Changing the camera COUNT at runtime requires stop()+start()
	// (the context list is never resized while the thread runs - that keeps
	// main-thread accessors race-free); per-camera settings changes only need
	// requestConfigRefresh().
	void start();
	void stop();

	// Pauses ML tracking for one camera (calibration wizards run their
	// detection on the main thread from that camera's preview frames).
	// Preview frames keep flowing; the other cameras keep tracking.
	void setTrackingEnabled(int cameraIndex, bool bEnabled);
	bool isTrackingEnabled(int cameraIndex) const;

	// Disable to receive raw distorted preview frames for one camera (needed
	// while capturing intrinsics calibration samples)
	void setUndistortEnabled(int cameraIndex, bool bEnabled);
	bool isUndistortEnabled(int cameraIndex) const;

	// Copies the newest preview frame + per-camera result for a camera.
	// Returns false if nothing new arrived since the last call.
	bool fetchPreviewFrame(int cameraIndex, VisionPreviewFrame& outFrame);

	// Copies the newest fused (world-space, smoothed) tracking result, and
	// in direct mode the avatar pose the direct retarget solved for it (an
	// invalid pose otherwise), and the fusion diagnostics behind it (empty
	// for a passthrough or direct frame). Returns false if nothing new
	// arrived since the last call.
	bool fetchFusedResult(TrackingFrameResult& outResult, AvatarPose* outDirectPose= nullptr,
						  DirectRetarget::SolveState* outDirectState= nullptr,
						  FusionDiagnostics* outFusionDiagnostics= nullptr);

	// Per-side camera index that dominated the last fusion (-1 = untracked)
	int getDominantCamera(eHandSide side) const { return m_dominantCamera[(int)side].load(); }

	// One camera's observation confidence for a fused side in the last fusion
	// (presence x measured stability), or -1 if that camera contributed no
	// observation to that side. For the UI's per-camera quality readout.
	float getObservationConfidence(int cameraIndex, eHandSide side) const;

	// Stereo auto hand-scale: current correction factor over the configured
	// hand scale (1 = unchanged), refined from cross-camera wrist
	// triangulation when enabled and both cameras see the same hand
	float getAutoHandScaleFactor() const { return m_autoScaleFactor.load(); }

	// Re-reads config (cameras/intrinsics/extrinsics/hand scale/tracking/
	// fusion/osc) on the vision thread before the next frame
	void requestConfigRefresh() { m_bConfigRefreshRequested= true; }

	// The loaded avatar's skeleton (null = none) and its rig settings, for
	// the avatar-driven VMC output. Applied before the next frame without a
	// config refresh, which would end a recording. Survives a stop/start.
	void setAvatarSkeleton(std::shared_ptr<const AvatarSkeleton> skeleton, const AvatarRigSettings& rig);

	// The loaded avatar's identity and face profile binding, for the VMC
	// output's /VMC/Ext/VRM announcement and the face evaluation. Applied like
	// the skeleton; an empty path clears the identity. A null binding keeps
	// the no-avatar Auto profile.
	struct AvatarIdentity
	{
		std::string path;   // UTF-8
		std::string title;
		std::string sha256; // lowercase hex of the file bytes
		std::shared_ptr<const FaceProfileBinding> faceBinding;
	};
	void setAvatarIdentity(const AvatarIdentity& identity);

	// Phone face stream: aligns the phone's head to the camera head at the
	// next frame where both are measured. The anchor it solves is fetched by
	// the UI and persisted in the face config.
	void requestFaceAnchorCapture() { m_bFaceAnchorCaptureRequested= true; }
	bool fetchFaceAnchorCapture(glm::quat& outAnchor);

	// Rest-pose calibration: captures what EVERY camera currently reports for
	// each tracked hand. Per camera because the model landmarks are
	// view-dependent - one camera's rest angles do not zero another's.
	void requestRestPoseCapture() { m_bRestPoseCaptureRequested= true; }

	// Wrist IMU: solve the mounting rotation for both wrists from the two
	// recorded calibration motions (see imuSolveMountingFromMotions). Record a
	// twist and a curl first; there is no pose to hold.
	void requestImuMountingCapture() { m_bImuMountingCaptureRequested= true; }
	struct ImuMountingCapture
	{
		MountingCaptureResult sides[2];
	};
	bool fetchImuMountingCapture(ImuMountingCapture& outCapture);

	// Live per-wrist IMU status for the UI (main thread safe: plain copies)
	ImuSideStatus getImuSideStatus(eHandSide side) const;
	// Starts recording one of the two calibration motions, discarding whatever
	// that window held before and clearing the live twist history so the
	// progress readout describes the stage being asked for. Pass
	// eMountingMotion::None to stop recording.
	void requestImuMotionRecording(eMountingMotion motion)
	{
		m_requestedImuMotionRecording= (int)motion;
		m_bImuMotionRecordingRequested= true;
	}
	// Static gyro bias calibration: measures each resting controller's bias
	// directly. Progress is reported through getImuSideStatus().
	void requestImuBiasCalibration() { m_bImuBiasCalibrationRequested= true; }
	void cancelImuBiasCalibration() { m_bImuBiasCancelRequested= true; }

	// The captured rest angles for both sides (RAW multi-view angles from the
	// fuse that serviced the capture; a side is only captured when it was
	// stereo-quality at that moment, or fitted at all on a one-camera rig)
	struct RestPoseCapture
	{
		std::array<std::array<FingerAngles, FINGER_COUNT>, 2> angles{};
		bool bCaptured[2]= {false, false};
	};
	// Fetches a completed capture; returns false if none is pending
	bool fetchRestPoseCapture(RestPoseCapture& outFused);

	// Direct retarget: captures the solver's current filtered measurements
	// as the neutral pose (the fingers need no zero there), and while the
	// phone streams captures the face anchor with it (fetched separately,
	// above). The UI fetches the capture and persists it in the direct
	// config. False from fetch until a capture lands; a request with no
	// solved torso is dropped.
	struct NeutralCapture
	{
		DirectNeutralPose neutral;
	};
	void requestNeutralCapture() { m_bNeutralCaptureRequested= true; }
	bool fetchNeutralCapture(NeutralCapture& outCapture);

	// Hand bone calibration: measures the user's own skeleton from the
	// stereo-triangulated landmarks over a sampling window. Unlike the rest
	// pose this needs TIME rather than an instant - it is a median over many
	// frames, and one pose only measures that pose's triangulation luck.
	void requestBoneCalibration(float durationSeconds)
	{
		m_boneCalibrationSeconds= durationSeconds;
		m_bBoneCalibrationRequested= true;
	}
	void cancelBoneCalibration() { m_bBoneCalibrationCancelRequested= true; }
	// Live sample counts while a window is open, so the UI can show progress
	// and refuse a capture that never saw a hand
	void getBoneCalibrationProgress(bool& outActive, int& outLeftSamples, int& outRightSamples) const
	{
		outActive= m_bBoneCalibrationActive.load();
		outLeftSamples= m_boneCalibrationSamples[0].load();
		outRightSamples= m_boneCalibrationSamples[1].load();
	}

	struct BoneCalibrationCapture
	{
		std::array<HandSkeleton, 2> skeleton{};
		std::array<HandBoneCalibrator::Quality, 2> quality{};
		bool bCaptured[2]= {false, false};
	};
	// Fetches a completed window; returns false if none is pending
	bool fetchBoneCalibration(BoneCalibrationCapture& outCapture);

	// Tracking recording: captures every input to the post-MediaPipe stages
	// (per-camera landmarks, depth measurements, timestamps, IMU forearm
	// output) so the tracking pipeline can be re-run offline bit-exactly.
	// Starting a recording RESETS the transient tracking state (filters,
	// jitter EMAs, PnP warm starts, palmar memories) - replay runs on fresh
	// instances, so live must start from the same zero state. A config
	// refresh while recording finalizes the file (the header's config
	// snapshot must describe every frame).
	void requestRecordingStart(const std::string& filePath);
	void requestRecordingStop();
	bool isRecording() const { return m_recorder != nullptr && m_recorder->isRecording(); }
	int64_t getRecordingFrameCount() const { return m_recorder != nullptr ? m_recorder->getFrameCount() : 0; }
	uint64_t getRecordingBytes() const { return m_recorder != nullptr ? m_recorder->getBytesWritten() : 0; }
	bool didLastRecordingAbort() const { return m_recorder != nullptr && m_recorder->wasAborted(); }
	bool isRecordingRawFrames() const { return m_frameRecorder != nullptr && m_frameRecorder->isRecording(); }
	// Seconds of audio the recording's sidecar holds, or -1 while none is written
	double getAudioRecordingSeconds() const;
	int64_t getRecordedFrameCount() const
	{
		return m_frameRecorder != nullptr ? m_frameRecorder->getFramesWritten() : 0;
	}
	int64_t getDroppedFrameCount() const
	{
		return m_frameRecorder != nullptr ? m_frameRecorder->getFramesDropped() : 0;
	}
	uint64_t getRecordedFrameBytes() const
	{
		return m_frameRecorder != nullptr ? m_frameRecorder->getBytesWritten() : 0;
	}
	// Path of the last finalized recording ("" until one completes)
	std::string getLastRecordingPath();

	// Diagnostic dump (F9): the vision thread writes its rolling state history,
	// the current camera frames (raw + annotated PNGs), the live config, the
	// frame as a one-frame recording and, in direct mode, the posed avatar
	// to dumpDir on its next loop iteration. avatarRender: the avatar as the
	// UI rendered it from the direct camera this frame, saved beside the
	// frame it should match (empty for none).
	void requestDiagnosticDump(const std::string& dumpDir, const cv::Mat& avatarRender= cv::Mat());
	// Directory of the last completed dump ("" until one succeeds)
	std::string getLastDumpPath();

	// Introspection for the UI
	const char* getActiveExecutionProvider(int cameraIndex= 0) const;
	float getLastInferenceMs() const { return m_lastInferenceMs; } // summed across cameras

	// -- Frame-loop hitch watchdog ------------------------------------------
	// Every camera shares this one thread, so any phase that overruns the
	// frame budget starves ALL of them at once: frames keep arriving from the
	// driver, find no free block, and are dropped. The symptom downstream is a
	// synchronized multi-frame tracking gap that looks like a camera or USB
	// fault, so the loop measures itself and names the phase responsible.
	enum class eVisionPhase : int
	{
		None= 0,
		ConfigRefresh, // config refresh / recording start-stop
		Capture,       // frame pop, undistort, ML inference, 3D projection
		Imu,           // wrist IMU sample drain, discovery, forearm publish
		Fusion,        // cross-camera fusion + smoothing
		Face,          // phone face stream drain
		Osc,
		Diagnostics, // dump history, recording enqueue, dump writes
		Count
	};
	static const char* getPhaseName(eVisionPhase phase);

	int getHitchCount() const { return m_hitchCount; }
	float getLastHitchMs() const { return m_lastHitchMs; }
	eVisionPhase getLastHitchPhase() const { return (eVisionPhase)m_lastHitchPhase.load(); }

	// Per-camera readouts are fixed size so they need no locking; cameras
	// beyond this are simply not reported.
	static constexpr int k_maxReportedCameras= 8;

	// What the last loop iteration did, for the pipeline status display.
	// Fixed size, copied under one mutex.
	struct Telemetry
	{
		enum class ePath : int
		{
			Idle= 0,
			FaceOnly,
			Fused,
			Passthrough,
			Direct
		};
		bool bValid= false; // false until the first iteration publishes
		ePath path= ePath::Idle;
		double timestampMs= 0.0;                     // the output frame's timestamp, 0 on an idle iteration
		float phaseMs[(int)eVisionPhase::Count]= {}; // the iteration's time per phase
		float totalMs= 0.f;
		uint64_t iteration= 0;     // counts published iterations
		bool bOscRunning= false;   // the streamer exists and its socket is open
		float oscSendRateHz= 0.f;  // OscStreamer::getMessagesPerSecond
		int oscLastPacketCount= 0; // datagrams in the last frame sent
		int oscLastFrameBytes= 0;  // bytes in the last frame sent, all datagrams
		bool bBodyModelLoaded[k_maxReportedCameras]= {}; // per camera: body tracker allocated and loaded
	};
	Telemetry getTelemetry() const;

	// The frame loop's recent iterations, for the Performance chart (display
	// only: no recording and no replayed stage reads it). Appends every held
	// entry numbered after afterSequence, oldest first, and returns the
	// newest entry's number. An entry is an iteration whose own timings the
	// telemetry published: a camera frame, or a face-only frame while no
	// camera frame is recent. The idle spins between frames are left out.
	uint64_t fetchIterationHistory(uint64_t afterSequence, std::vector<IterationHistory::Entry>& outEntries) const;
	// An iteration at least this long is a hitch
	static double getHitchThresholdMs();

private:
	// Scratch for one loop iteration, threaded through the stages below
	struct IterationState
	{
		bool bAnyNewResult= false;
		double newestTimestampMs= 0.0;
		std::vector<const CameraFrameResult*> fusionCandidates;
		bool bAnyWorldCandidate= false;
		// The frame's output: fused world space, or camera 0's camera-space
		// passthrough when no calibrated camera contributed
		TrackingFrameResult outputResult;
		// Direct mode this iteration: the output is the direct camera's result
		// and the direct retarget poses the avatar from it
		bool bDirect= false;
		// The recording record under assembly, live only while recording
		bool bRecordingThisFrame= false;
		RecordedFrame recordFrame;
		// The loop's per-phase times so far this iteration, for the recorded
		// telemetry
		const double* phaseMs= nullptr;
	};

	// Which path the iteration took: the direct camera, the fusion, or the
	// camera-space passthrough
	static Telemetry::ePath getCameraIterationPath(const IterationState& iteration);
	// The stage state a recorded frame carries for display. nowMs places the
	// cameras' last frames in time (a camera whose newest frame is older than
	// the telemetry hold window reads as not streaming).
	void gatherRecordedTelemetry(Telemetry::ePath path, const double* phaseMs, double nowMs,
								 RecordedTelemetry& outTelemetry);

	// The loop and its stages, in iteration order. Each stage works on the
	// iteration scratch and the members; the main-thread handoffs stay here.
	void threadLoop();
	// Config refresh, recording stop, recording start
	void servicePendingRequests();
	// Every camera with a new frame, through its CameraContext
	void runCaptureStage(IterationState& iteration);
	// Cross-camera fusion (or the camera-space passthrough) and the fused
	// checksum tap
	void runFusionStage(IterationState& iteration);
	// Direct mode: the direct camera's result as the output (the camera is
	// the world), recorded as a passthrough of that camera, in place of the
	// fusion, IMU and body-solve stages
	bool isDirectModeActive() const;
	void runDirectStage(IterationState& iteration);
	// The direct retarget over the direct camera's result, after the face
	// stage so the phone head reaches it; an invalid pose without a skeleton
	void solveDirectPose(IterationState& iteration);
	// Wrist IMU: sample drain, yaw anchor, forearm fill, mounting capture
	void runImuStage(IterationState& iteration);
	// The mounting wizard's motion recording and bias calibration requests,
	// serviced before every IMU service update whether or not a palm is in
	// view
	void serviceImuRequests();
	// A requested mounting capture, published for fetchImuMountingCapture.
	// Needs no palm this iteration: the capture consumes averaged windows.
	void serviceImuMountingCapture();
	// Copies the IMU service's per-side status into the UI snapshot
	void publishImuStatus();
	// Vision body solve, then the fusion bookkeeping (search-hint seed,
	// dominant camera, per-camera confidence, auto hand-scale)
	void runBodySolveStage(IterationState& iteration);
	// The face profile evaluation, the OSC send and the fused-result publish
	void runOutputStage(IterationState& iteration);
	// The microphone's visemes onto the output's viseme source
	void applyMouthToOutput(TrackingFrameResult& output, double nowMs);
	// The face stream's blendshapes onto the output's phone source, and its
	// head, which wins over the camera head while the stream is live and
	// tracked. Fills the viseme source beside it.
	void applyFaceToOutput(TrackingFrameResult& output, double nowMs, bool bCameraHead);
	// A new face or mouth sample with no camera frame this iteration still
	// goes out: the sources run at their own rates, faster than the cameras,
	// and must keep flowing with the cameras stopped. Returns true when it
	// produced output.
	bool runFaceOnlyOutput(IterationState& iteration, double nowMs);
	// Bone calibration window and rest-pose capture
	void runCalibrationCaptures();
	// History ring, recording assembly, dump requests
	void runDiagnosticsStage(IterationState& iteration);

	void refreshConfigOnThread();
	// (Re)creates the OSC streamer and pushes the current config, body
	// dimensions and avatar skeleton into it
	void applyOscConfigOnThread();
	// Services a pending requestDiagnosticDump on the vision thread. path and
	// phaseMs describe the iteration that writes the dump, for the telemetry
	// its one-frame recording carries.
	void performDiagnosticDump(const TrackingFrameResult& latestOutput, Telemetry::ePath path, const double* phaseMs);

	VideoCaptureSystem* m_videoCapture= nullptr;
	AppConfig* m_config= nullptr;

	std::vector<std::unique_ptr<CameraContext>> m_cameras;
	HandFusion m_fusion;
	BodyPoseSolver m_bodyPoseSolver;

	// Carried between iterations (vision thread only). The previous fused
	// world result seeds the cross-camera search hints; the latest output
	// (world OR camera space) feeds diagnostic dumps.
	TrackingFrameResult m_lastFusedForHints;
	TrackingFrameResult m_lastOutputResult;

	// Wrist IMU service (devices + per-device orientation filters). Owned by
	// App; this thread is its only caller while running, and the UI reads
	// snapshots through m_imuMutex below.
	ImuService* m_imuService= nullptr;
	std::atomic_bool m_bImuMountingCaptureRequested{false};
	std::atomic_bool m_bImuMotionRecordingRequested{false};
	std::atomic_int m_requestedImuMotionRecording{(int)eMountingMotion::None};
	std::atomic_bool m_bImuBiasCalibrationRequested{false};
	std::atomic_bool m_bImuBiasCancelRequested{false};
	mutable std::mutex m_imuMutex;
	ImuMountingCapture m_capturedImuMounting;
	bool m_bImuMountingReady= false;
	ImuSideStatus m_imuStatus[2];
	// Phone face stream. Owned by App; this thread drains it.
	FaceService* m_faceService= nullptr;
	uint64_t m_lastFaceSequence= 0;
	// Microphone mouth stage. Owned by App; this thread reads its sample.
	AudioService* m_audioService= nullptr;
	uint64_t m_lastMouthSequence= 0;
	// The timestamp of the last output a camera produced, so a face-only
	// frame can tell when the hands it repeats have gone stale
	double m_lastCameraOutputMs= -1.0;
	glm::quat m_faceAnchor{1.f, 0.f, 0.f, 0.f};
	std::atomic_bool m_bFaceAnchorCaptureRequested{false};
	std::mutex m_faceAnchorMutex;
	glm::quat m_capturedFaceAnchor{1.f, 0.f, 0.f, 0.f};
	bool m_bFaceAnchorReady= false;

	std::unique_ptr<OscStreamer> m_oscStreamer;
	// Avatar handoff: the main thread drops the skeleton and the rig settings
	// in the pending slot, the vision thread adopts them between frames
	struct AvatarHandoff
	{
		std::shared_ptr<const AvatarSkeleton> skeleton;
		AvatarRigSettings rig;
	};
	std::mutex m_avatarMutex;
	AvatarHandoff m_pendingAvatar;
	std::atomic<bool> m_bAvatarSkeletonChanged{false};
	AvatarHandoff m_avatar;
	AvatarIdentity m_pendingAvatarIdentity;
	std::atomic<bool> m_bAvatarIdentityChanged{false};
	AvatarIdentity m_avatarIdentity;
	// The face profile over the output frames (vision thread only)
	FaceProfileRunner m_faceRunner;

	std::thread m_thread;
	std::atomic_bool m_bRunning{false};
	std::atomic_bool m_bConfigRefreshRequested{true};
	std::atomic<float> m_lastInferenceMs{0.f};
	std::atomic_int m_hitchCount{0};
	std::atomic<float> m_lastHitchMs{0.f};
	std::atomic_int m_lastHitchPhase{(int)eVisionPhase::None};
	std::atomic<int> m_dominantCamera[2]= {-1, -1};
	std::atomic<float> m_autoScaleFactor{1.f};

	// Per (camera, side) confidence from the last fusion, published for the
	// UI readout, k_maxReportedCameras cameras
	std::atomic<float> m_observationConfidence[k_maxReportedCameras * 2];

	// A mouth sample older than this is a stage that stopped, not a mouth
	static constexpr double k_mouthStaleMs= 1000.0;

	// Fused result handoff (mutex-guarded, latest-wins)
	std::mutex m_fusedMutex;
	TrackingFrameResult m_fusedResult;
	AvatarPose m_fusedDirectPose;
	DirectRetarget::SolveState m_fusedDirectState;
	// A display copy of the fuse behind m_fusedResult; a face-only frame,
	// which repeats the last output, keeps the last one
	FusionDiagnostics m_fusedDiagnostics;
	bool m_bFusedFresh= false;

	// Direct retarget (single camera, no world calibration): the solver and
	// its pose for the frame, and the neutral capture handoff
	DirectRetarget m_directRetarget;
	DirectRetargetConfig m_directConfig;
	AvatarPose m_directPose;
	std::atomic_bool m_bNeutralCaptureRequested{false};
	std::mutex m_neutralCaptureMutex;
	NeutralCapture m_capturedNeutral;
	bool m_bNeutralCaptureReady= false;

	// Rest-pose capture handoff
	std::atomic_bool m_bRestPoseCaptureRequested{false};
	std::mutex m_restPoseMutex;
	RestPoseCapture m_capturedFusedRest;
	bool m_bRestPoseReady= false;

	std::atomic_bool m_bBoneCalibrationRequested{false};
	std::atomic_bool m_bBoneCalibrationCancelRequested{false};
	std::atomic_bool m_bBoneCalibrationActive{false};
	std::atomic<float> m_boneCalibrationSeconds{0.f};
	std::atomic_int m_boneCalibrationSamples[2]{};
	// Vision-thread only
	HandBoneCalibrator m_boneCalibrator;
	double m_boneCalibrationEndMs= 0.0;
	std::mutex m_boneCalibrationMutex;
	BoneCalibrationCapture m_capturedBones;
	bool m_bBoneCalibrationReady= false;

	// Logs and publishes a loop iteration that overran the hitch threshold,
	// attributing it to the phase that consumed the most time
	void reportHitchIfSlow(double totalMs, const double* phaseMs);

	// Publishes the iteration's telemetry snapshot (display only: nothing on
	// the vision thread reads it back)
	void publishTelemetry(Telemetry::ePath path, double timestampMs, double totalMs, const double* phaseMs);
	mutable std::mutex m_telemetryMutex;
	Telemetry m_telemetry;
	// Written once per published iteration, after the telemetry, under a
	// mutex of its own so a reader copying the history never holds up the
	// telemetry
	mutable std::mutex m_iterationHistoryMutex;
	IterationHistory m_iterationHistory;
	// When the last camera-frame iteration published (vision thread only),
	// and how long its path stands in for the idle iterations after it
	double m_lastCameraTelemetryMs= -1.0e9;
	static constexpr double k_cameraTelemetryHoldMs= 500.0;
	// The last camera-frame iteration's Diagnostics phase time (vision thread
	// only): a recorded frame is assembled inside that phase, so it carries
	// the previous one's
	double m_lastDiagnosticsPhaseMs= 0.0;

	// Consumes a pending recording start request on the vision thread (state
	// reset + header snapshot + recorder start)
	void handleRecordingStartOnThread();
	// Finalizes an active recording on the vision thread
	void finalizeRecordingOnThread(bool bAborted, const std::string& reason);

	// Tracking recorder (vision thread owns it; UI reads atomics through the
	// accessors above)
	std::unique_ptr<TrackingRecorder> m_recorder;
	// Only allocated when raw frame capture is turned on
	std::unique_ptr<FrameRecorder> m_frameRecorder;
	std::atomic_bool m_bRecordingStartRequested{false};
	std::atomic_bool m_bRecordingStopRequested{false};
	std::mutex m_recordingMutex;
	std::string m_requestedRecordingPath;
	std::string m_lastRecordingPath;
	int64_t m_recordingSeq= 0;
	// The audio sidecar's stream, so a mouth frame from another stream (the
	// device reopened) records without an index into it
	bool m_bRecordingAudio= false;
	uint32_t m_recordingAudioEpoch= 0;
	// The face sidecar's writer, and the last phone sample it took
	FaceRecorder m_faceRecorder;
	uint64_t m_lastRecordedFaceSequence= 0;

	// Diagnostic dump: history lives on the vision thread; the request path
	// and completion path are the only cross-thread strings (mutex-guarded)
	DiagnosticDump m_diagnostics;
	std::atomic_bool m_bDumpRequested{false};
	std::mutex m_dumpMutex;
	std::string m_requestedDumpDir;
	cv::Mat m_requestedAvatarRender;
	std::string m_lastDumpPath;
};
