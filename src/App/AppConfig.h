#pragma once

#include <array>
#include <cassert>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "glm/ext/matrix_double4x4.hpp"

#include "AudioTypes.h"
#include "MikanVideoSourceTypes.h"
#include "TrackingTypes.h"

// Persisted per-project tracking settings, stored as JSON at the loaded
// project file path (legacy location: %APPDATA%/MikanTrack/config.json)

struct VideoConfig
{
	std::string deviceName;
	std::string devicePath;
	std::string modeName;
};

struct IntrinsicsConfig
{
	bool present= false;
	MikanMonoIntrinsics intrinsics;
	double reprojectionError= 0.0;
};

struct ExtrinsicsConfig
{
	bool present= false;
	glm::dmat4 markerFromCamera{1.0};
	// Quality of this camera's own pose solve: mean reprojection error of the
	// averaged board corners it was solved from. Low here only means the pose
	// fits THIS camera's detections - see ExtrinsicsQualityConfig for whether
	// the cameras agree with each other.
	double patternReprojectionErrorPx= 0.0;
	int patternCornerCount= 0;
	// Origin aruco marker this camera was calibrated against. A charuco board
	// was tried and rejected: at the 60-70 cm working distance its 24 mm
	// squares (the largest that fits on letter paper) do not resolve enough
	// corner detail to detect, while one large aruco square does.
	int markerId= 0;
	double markerLengthMM= 100.0;
};

// Cross-camera extrinsics quality from the last calibration, measured by
// triangulating the calibration board's corners and comparing the
// reconstruction against the board's known geometry. Reports the WORST camera
// pair, because that pair is what limits fused tracking.
struct ExtrinsicsQualityConfig
{
	bool present= false;
	int pairCount= 0;
	int worstPairCameraA= -1;
	int worstPairCameraB= -1;
	int worstPairSharedCorners= 0;
	// Comparable to the hand pipeline's triResidualRmsPx
	double worstPairReprojectionRmsPx= 0.0;
	// Mean absolute error of triangulated inter-corner distances
	double worstPairSpacingErrorMm= 0.0;
	// Reconstruction scale (1.0 = correct); catches a wrong baseline
	double worstPairSpacingScale= 1.0;
	double worstPairPlanarityRmsMm= 0.0;
};

struct HandScaleConfig
{
	bool present= false;
	double refLengthMeters= 0.08; // wrist -> middle-MCP distance
};

// The user's own hand geometry, measured from stereo triangulation (see
// HandBoneCalibrator). NOT per camera: bones are a physical property, unlike
// the rest angles above, which correct each camera's view-dependent model
// bias. When present it supersedes the landmark model's proportions both as
// the PnP object model and as the skeleton streamed to clients.
//
// neutralDirInPalm is deliberately not stored - it is derived from the base
// positions on load, so the streamed skeleton's zero keeps meaning one thing.
struct HandSkeletonConfig
{
	bool present[2]= {false, false}; // indexed by eHandSide
	std::array<HandSkeleton, 2> skeleton{};
};

struct CharucoBoardConfig
{
	int cols= 11;
	int rows= 8;
	double squareLengthMM= 16.0;
	double markerLengthMM= 12.0;
};

struct TrackingConfig
{
	bool flipHandedness= true;
	int detectorIntervalFrames= 30;
	// Relaxed palm-detection cutoff used while reacquiring a recently-lost
	// hand (a slot is free + a slot died within ~1.5s). Recall when it
	// matters, strict precision otherwise; fusion's gates absorb the extra
	// false positives.
	float palmScoreThresholdRelaxed= 0.25f;
	// When one camera tracks a hand another camera lost, project it into the
	// lost camera's image and try the landmark model there directly
	std::string onnxEp= "directml"; // "directml" | "cpu"
};

struct OscConfig
{
	bool enabled= true;
	std::string targetIp= "127.0.0.1";
	// VMC's conventional Performer -> Marionette port
	int port= 39539;
	int maxRateHz= 60;
	// Treat a hand below this fused confidence as lost, so the avatar holds
	// instead of jittering
	float minConfidence= 0.f;
	// Keep streaming the last good pose for this long after a dropout before
	// the loss rule (freeze or silence) applies - bridges brief losses.
	// 0 = apply it immediately. Legacy: a project with retarget chain settings
	// neither reads nor writes it (nor vmcFreezeOnLoss below), and one without
	// migrates its chains from both.
	float holdOnDropoutMs= 250.f;

	// Neck -> head bone offset. A VMC receiver replaces the translation of
	// every bone it is sent, so the head needs one, and nothing on this rig
	// measures a neck.
	float vmcHeadOffsetMeters= 0.08f;
	// Keep streaming a lost hand's last bones instead of going silent, which
	// would drop that arm back to the avatar's rest T-pose
	bool vmcFreezeOnLoss= true;
};

// Everything specific to one physical camera. Each camera calibrates
// intrinsics + extrinsics against the same printed marker, so all cameras
// share one world frame (which is what makes multi-camera fusion possible).
// Angles this camera reports for a hand held in its rest pose. Subtracted
// from later measurements so the rest pose reads zeros. PER CAMERA because
// MediaPipe's model landmarks are view-dependent - two cameras watching one
// physical hand disagree about articulation by tens of degrees.
struct RestAnglesConfig
{
	bool present[2]= {false, false}; // indexed by eHandSide
	std::array<std::array<FingerAngles, FINGER_COUNT>, 2> angles{};
};

// Fitted per-side Gaussian prior over the 20 RAW finger angles (mean +
// row-major precision matrix), produced by --fit-angle-prior from the user's
// own stereo-quality recordings. Consumed by the hand state estimator as a
// weak Mahalanobis pull toward the user's real pose distribution.
struct AnglePriorConfig
{
	static constexpr int k_angleCount= FINGER_COUNT * 4;
	bool present[2]= {false, false}; // indexed by eHandSide
	std::array<std::array<float, k_angleCount>, 2> mean{};
	std::array<std::array<float, k_angleCount * k_angleCount>, 2> precision{};
};

// Body-pose stage, opt-in per camera. The person detector only fires on
// cameras that see the user upright, so this stays off for overhead cameras
// and costs them nothing.
struct BodyPoseCameraConfig
{
	bool enabled= false;
	// Which landmark model runs on this camera: "rtmpose" (RTMPose-m body,
	// 17 planar keypoints) or "rtmw3d" (RTMW3D-X whole body with a depth
	// per keypoint). See eBodyPoseBackend.
	std::string backend= "rtmpose";
	// Pose models run every Nth frame on this camera
	int poseFrameDivider= 2;
	// Rebuild the region of interest from the image every Nth model frame,
	// regardless of model confidence
	int detectorIntervalFrames= 20;
};

struct CameraProfile
{
	VideoConfig video;
	IntrinsicsConfig intrinsics;
	ExtrinsicsConfig extrinsics;
	BodyPoseCameraConfig bodyPose;
	// Per side, the factor this camera's monocular lift distance is
	// multiplied by. The landmark model's depth from one viewpoint carries a
	// bias that depends on the view and the hand (measured 4 to 8 percent
	// deep on a left hand from a front camera, within 3 percent either way
	// on the right), and the scale lands the lift on a depth measured
	// another way, stereo or the desk plane. 1 = the lift as solved.
	// Indexed by eHandSide.
	std::array<float, 2> liftDepthScale{1.f, 1.f};
};

// Wrist-strapped inertial trackers (Joy-Cons today, SlimeVR later).
// The mounting rotation is captured per side with the wrist held straight,
// which defines the forearm frame as "the palm frame at neutral wrist".
struct ImuConfig
{
	bool enabled= true;
	// Vision yaw-anchor strength, radians. Loose on purpose: vision sees the
	// palm, the sensor rides the forearm, and the wrist joint between them is
	// unknown - so vision should correct slow yaw drift without fighting real
	// wrist motion.
	float visionYawSigma= 0.35f;
	// Swap which controller drives which wrist
	bool swapSides= false;
	// Captured mounting rotation (forearm -> sensor), indexed by eHandSide
	bool mountingPresent[2]= {false, false};
	std::array<glm::quat, 2> forearmToSensor{glm::quat(1.f, 0.f, 0.f, 0.f), glm::quat(1.f, 0.f, 0.f, 0.f)};
};

// iFacialMocap face stream (an iPhone app sending ARKit blendshapes and head
// rotation over UDP)
struct FaceConfig
{
	bool enabled= false;
	// The UDP port the phone app streams to, and is asked to stream on
	int port= 49983;
	// Where the streaming handshake goes (the broadcast address reaches a
	// phone anywhere on the LAN)
	std::string phoneAddress= "255.255.255.255";
	// World orientation of the head while it looks straight at the phone.
	// Absent, that is world +X, the avatar's rest facing. Captured against the
	// camera head, so a phone mounted off to one side still turns the head
	// about the right axis.
	bool headAnchorPresent= false;
	glm::quat headAnchor{1.f, 0.f, 0.f, 0.f};
};

// The microphone mouth stage
struct AudioConfig
{
	bool enabled= false;
	// The capture device by name, re-resolved at open; empty or a name no
	// device carries any more takes the system default
	std::string deviceName;
	// Applied to the samples before the gate and the backend
	float gainDb= 0.f;
	// Hop level under which the mouth is neutral and no backend runs
	float gateDbfs= -45.f;
	// The mouth model: the level alone, the MFCC profile, or Oculus LipSync
	eMouthModel model= eMouthModel::Mfcc;
	// The calibration profile the MFCC model loads; empty is volume-only
	std::string profilePath;
	// The emission filter's settling time per model (eMouthModel order), ms,
	// 0 for no filter
	std::array<float, MOUTH_MODEL_COUNT> smoothingMs= {
		MOUTH_SMOOTHING_DEFAULT_MS[0], MOUTH_SMOOTHING_DEFAULT_MS[1], MOUTH_SMOOTHING_DEFAULT_MS[2]};
	// The Oculus engine's own viseme smoothing, OVR_SMOOTHING_MIN..MAX
	int ovrSmoothing= OVR_SMOOTHING_DEFAULT;
};

// Body proportions shared by every elbow consumer (IMU-measured forearms and
// the vision body-pose solver alike)
struct BodyConfig
{
	// Forearm length, used to place the elbow estimate back along the forearm
	// direction from the wrist (visualization + IK hint). The direction is
	// measured; only this length is assumed. The IMU mounting wizard's curl
	// stage overwrites it with a measured value.
	float forearmLengthMeters= 0.25f;
	// Elbow-to-shoulder length. Only disambiguates which of the two
	// ray-sphere elbow solutions is real; it never places a joint by itself.
	float upperArmLengthMeters= 0.30f;
	// Derive the upper arm from the shoulder width instead of measuring it.
	// Measuring it needs the arm held straight and square to the camera,
	// which is hard to hit at a desk and wrong by 20% when missed - and a
	// wrong upper arm makes the elbow solve pick the wrong bend. Body
	// proportions are stable enough that a multiple of a width that IS easy
	// to measure beats a length that is not.
	bool bDeriveUpperArmFromShoulderWidth= true;
	// Upper arm as a multiple of the LANDMARK shoulder width. Not the
	// anatomical ratio: the model's shoulder points sit inside the real
	// joints (measured at ~0.74 of a biacromial breadth), so the familiar
	// "arm is about 1.5 shoulder widths" becomes ~2.0 of THIS width, and the
	// upper arm alone lands near 1.05.
	float upperArmPerShoulderWidth= 1.05f;
	// Distance between the shoulder joints. With both shoulder rays known,
	// this fixes their depth without the pose model's own metric guess, which
	// measured unusable per frame (its forearm length swung nearly 3x within
	// one session).
	float shoulderWidthMeters= 0.40f;
	// Ear-to-ear width, which fixes head depth the same way
	float headWidthMeters= 0.15f;
	// Ear-axis midpoint to nose tip, which fixes head yaw and pitch
	float noseForwardMeters= 0.11f;
	// Per side (left, right): where the elbow sits when the solver has no
	// history, relative to the shoulder, as a multiple of the upper-arm
	// length, in the torso frame (+X facing, +Y the user's left, +Z up). That
	// coincides with the world frame for a user facing the marker's FORWARD
	// label. Written from the loaded avatar's rig (its elbowHintOffset over its
	// upper-arm length), so every project that loads that avatar seeds the
	// same way and a recording replays with it. Zero = no seed.
	std::array<glm::vec3, 2> elbowSeedPerUpperArm{};
};

// Tracking recordings store landmarks, which are abstract enough to share.
// Raw frames are video of a room, so storing them is OPT-IN and local: this
// defaults off and stays off unless someone deliberately turns it on.
struct RecordingConfig
{
	// Store the frames the models consumed alongside the landmark recording.
	// The reason to want it: a landmark recording cannot answer "would a
	// different pose model have done better", because the model's input is
	// gone. Costs roughly 3-6 MB/s per camera.
	bool recordRawFrames= false;
	int jpegQuality= 85;
};

// The VRM avatar shown in the 3D scene and retargeted onto. Only the path
// and placement persist: the loaded model lives on App, and loading is never
// a side effect of reading this config (replay reconstructs configs
// headlessly, with no GL context).
struct AvatarConfig
{
	// UTF-8. Absolute, or relative to the project folder, then to the exe
	// folder (so "models/avatars/x.vrm" finds a shipped sample). Empty = none.
	std::string modelPath;
	bool showInScene= true;
	// Slide the avatar so its shoulders sit on the measured ones when the
	// body-pose stage tracks both; otherwise the fixed root below places it
	bool followShoulders= true;
	// Where the avatar's origin (between its feet) sits in the world frame
	// when the shoulders are not followed, meters, plus its yaw about world
	// +Z in degrees (0 = facing world +X, the rest-pose facing)
	glm::vec3 rootPositionWorld{0.f, 0.f, 0.f};
	float rootYawDegrees= 0.f;
};

// The single-camera direct retarget: one camera's landmarks posed straight
// onto the loaded avatar with no world calibration, the camera being the
// world. The neutral capture is the user's rest (sitting square, hands
// resting), the zero every relative rotation is measured from, stored in the
// camera-fixed frame at zero tilt.
struct DirectConfig
{
	bool enabled= false;
	// The camera whose landmarks drive the avatar
	int cameraIndex= 0;
	// The camera's downward pitch, degrees, positive looking down, set by
	// the user alone (a webcam on top of the monitor by default; nothing
	// measures it)
	float cameraTiltDegrees= 10.f;
	bool neutralPresent= false;
	glm::quat neutralTorso{1.f, 0.f, 0.f, 0.f};
	glm::vec3 neutralShoulderMidpoint{0.f};
	glm::quat neutralHead{1.f, 0.f, 0.f, 0.f};
};

struct FusionConfig
{
	// A camera's last result older than this is excluded from fusion
	double stalenessWindowMs= 66.0;
	// Two cameras' world wrists further apart than this can't be the same
	// physical hand (cross-camera handedness-conflict gate)
	float wristMatchMaxDistM= 0.25f;
	// Palm jitter (mm) at which an observation counts as half as trustworthy
	float jitterReferenceMm= 15.f;
	// RMS reprojection residual above which a triangulated pairing is
	// rejected as two different physical hands
	float triangulationMaxResidualPx= 25.f;
	// Residual (px) at which a triangulated pose counts as half as
	// trustworthy - a TRUE pose-quality signal (unlike presence, which stays
	// high on ill-conditioned views), folded into the fused confidence
	float residualReferencePx= 8.f;

	// Angle-space multi-view state estimator: one temporally continuous state
	// per hand fit to all fresh cameras' 2D landmarks (the output pose path).
	// Temporal-prior process sigmas (how far each block may move per second)
	float estimatorPalmPosSigmaMPerS= 0.3f;
	float estimatorPalmRotSigmaRadPerS= 2.f;
	float estimatorAngleSigmaRadPerS= 2.5f;
	// Measurement weighting
	float estimatorPixelSigmaPx= 2.f;
	float estimatorHuberDeltaPx= 10.f;
	int estimatorMaxIterations= 4;
	// Post-fit mean residual above this holds the previous state; a streak
	// of them drops the state for a reseed (divergence guard)
	float estimatorMaxResidualPx= 25.f;
	// Palm steps implying more than this speed are correspondence errors,
	// not motion - they take the bad-fit hold path (physical innovation gate)
	float estimatorMaxStepMetersPerS= 4.f;
	// Anatomical joint-limit prior inside the estimator fit (one-sided soft
	// penalties beyond the per-DoF ranges; zero cost inside them)
	bool estimatorJointLimitsEnabled= true;
	float estimatorJointLimitSigmaRad= 0.05f;
	// Scale on the fitted angle prior's precision (see AnglePriorConfig);
	// 1 = trust the fitted distribution as-is, smaller = softer
	float estimatorAnglePriorWeight= 0.3f;
	// Sigma (m) of the single-view depth anchor on a side that has not yet
	// been fitted from two views (see HandStateEstimatorConfig); 0 disables
	float estimatorMonoDepthAnchorSigmaM= 0.01f;
};

// Declared here (rather than each caller assembling its own) so the live
// solve and a replayed one cannot drift apart
struct BodyDimensions;
BodyDimensions makeBodyDimensions(const class AppConfig& config);

// Same drift-proofing for the fusion config: the vision thread, the replay
// engine and the replay self-test all build it from an AppConfig through this
// one mapping
struct HandFusionConfig;
HandFusionConfig makeHandFusionConfig(const class AppConfig& config);

// The retarget chain settings a project runs on: its own chain block, or for
// a project without one the chains migrated from its shape (direct or not),
// its legacy stream settings (holdOnDropoutMs, vmcFreezeOnLoss) and the
// avatar's sidecar (the rest blend, each arm's elbow hint confidence). The
// second form takes the block in place of the config's, so a caller holding
// its own snapshot never reads the config's.
struct RetargetChains;
struct AvatarRigSettings;
RetargetChains resolveRetargetChains(const class AppConfig& config, const AvatarRigSettings& rig);
RetargetChains resolveRetargetChains(const RetargetChains* block, const class AppConfig& config,
									 const AvatarRigSettings& rig);

// And for the avatar retarget: the OSC streamer and the 3D view both solve
// from this one mapping of the project's avatar settings, the loaded
// avatar's rig settings. The chains' elbow weights reach the solve through
// the retarget stage, which evaluates them per frame.
struct AvatarRetargetConfig;
AvatarRetargetConfig makeAvatarRetargetConfig(const class AppConfig& config, const AvatarRigSettings& rig);

// And for the direct retarget: the vision thread, the 3D view's replay feed
// and the gate tool all solve from this one mapping
struct DirectRetargetConfig;
DirectRetargetConfig makeDirectRetargetConfig(const class AppConfig& config, const AvatarRigSettings& rig);
DirectRetargetConfig makeDirectRetargetConfig(const class AppConfig& config, const AvatarRigSettings& rig,
											  const RetargetChains& chains);

// And for the retarget stage: both pipelines' solves through the mappings
// above, each pipeline's loss rules from the chains (makeChainLossPolicy)
// with the delivery rules' minimum confidence, and each arm's mapping, IK
// weight, elbow weight, reach scale and goal offset from its chain as stored,
// which both pipelines read
struct RetargetStageConfig;
class AvatarSkeleton;
RetargetStageConfig makeRetargetStageConfig(const class AppConfig& config, const AvatarRigSettings& rig,
											std::shared_ptr<const AvatarSkeleton> skeleton);
RetargetStageConfig makeRetargetStageConfig(const class AppConfig& config, const AvatarRigSettings& rig,
											const RetargetChains& chains,
											std::shared_ptr<const AvatarSkeleton> skeleton);

class AppConfig
{
public:
	// invariant: never empty (default-constructed with one camera)
	std::vector<CameraProfile> cameras= std::vector<CameraProfile>(1);
	HandScaleConfig handScale;
	HandSkeletonConfig handSkeleton;
	CharucoBoardConfig charucoBoard;
	TrackingConfig tracking;
	OscConfig osc;
	FusionConfig fusion;
	ImuConfig imu;
	FaceConfig face;
	AudioConfig audio;
	BodyConfig body;
	RecordingConfig recording;
	AvatarConfig avatar;
	DirectConfig direct;
	// The retarget chain settings under the project's "retarget" section, an
	// immutable snapshot replaced whole on an edit; null for a project that
	// has none yet, which runs on chains migrated from its legacy settings
	// (resolveRetargetChains) and keeps writing those
	std::shared_ptr<const RetargetChains> retargetChains;
	// Rest-pose zero for the stereo-TRIANGULATED path (one set, not per
	// camera: triangulated geometry has no per-camera model bias to fold in).
	// Captured alongside the per-camera rest angles.
	RestAnglesConfig fusedRestAngles;
	// Fitted angle prior for the hand state estimator (--fit-angle-prior)
	AnglePriorConfig anglePrior;

	// Cross-camera extrinsics quality from the last calibration session
	ExtrinsicsQualityConfig extrinsicsQuality;

	CameraProfile& camera(size_t index)
	{
		assert(index < cameras.size());
		return cameras[index];
	}
	const CameraProfile& camera(size_t index) const
	{
		assert(index < cameras.size());
		return cameras[index];
	}
	size_t cameraCount() const { return cameras.size(); }

	// Loads from the given file path; returns false (and keeps defaults) if
	// missing/corrupt. Migrates v1 single-camera configs into cameras[0].
	// Guarantees cameras.size() >= 1 afterwards. Remembers the path so save()
	// writes back to it.
	bool load(const std::filesystem::path& path);
	bool save() const;
	// Parses the same schema load() reads from an in-memory JSON string.
	// Tracking recordings embed a config snapshot in their header; replay
	// reconstructs the recorded AppConfig through this.
	bool loadFromJsonString(const std::string& jsonText);
	// Serialized live config (same schema save() writes) - diagnostic dumps
	// embed this so a dump always reflects unsaved in-UI changes too
	std::string toJsonString() const;
	// The same with the given chain block in place of retargetChains (null
	// writes none, and the legacy stream keys instead), which this form never
	// reads: what a thread holding its own snapshot of the chains writes
	std::string toJsonString(const std::shared_ptr<const RetargetChains>& chains) const;

	void markDirty() { m_bDirty= true; }
	// Saves at most once per cooldown period when dirty; call from the main loop
	void updateAutoSave(float deltaSeconds);

	void setProjectFilePath(const std::filesystem::path& path) { m_projectFilePath= path; }
	const std::filesystem::path& getProjectFilePath() const { return m_projectFilePath; }
	// Directory the dump/recording paths derive from: the project folder when
	// a project file path is set, the legacy appdata folder otherwise
	std::filesystem::path getProjectDirectory() const;

	// Legacy global config location (%APPDATA%/MikanTrack/config.json)
	static std::string getLegacyConfigFilePath();
	// Fresh timestamped folder path for a diagnostic dump:
	// <project dir>/dumps/<yyyy-mm-dd_hh-mm-ss> (not created here)
	std::string makeDumpDirectoryPath() const;
	// Fresh timestamped file path for a tracking recording:
	// <project dir>/recordings/<yyyy-mm-dd_hh-mm-ss>.jsonl (directory created)
	std::string makeRecordingFilePath() const;
	std::string getRecordingsDirectoryPath() const;

private:
	std::filesystem::path m_projectFilePath;
	bool m_bDirty= false;
	float m_secondsSinceDirty= 0.f;
};
