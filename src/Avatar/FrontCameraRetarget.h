#pragma once

#include <array>

#include "glm/ext/matrix_float4x4.hpp"
#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/scalar_constants.hpp"
#include "glm/ext/vector_float3.hpp"

#include "AvatarRetarget.h"
#include "HandPoseModel.h"
#include "OneEuroFilter.h"
#include "SourcePose.h"
#include "TrackingTypes.h"
#include "WholeBodyFaceRatios.h"

class AvatarSkeleton;

// The single-camera front-camera retarget: one camera's hand landmarks and
// whole-body observation become avatar bone rotations with no world
// calibration. The camera is the world, the avatar's own bone lengths are
// the scale, and every joint sits on the ray through its landmark, so the
// posed avatar reprojects onto what the camera saw. The one thing a single
// view cannot see, the wrist's distance along its ray, is not estimated: the
// wrist sits where its ray crosses one of two depth planes ahead of the
// shoulder, the near one with the hand at the head. Pixels are filtered
// lightly and the torso's and head's depths heavily, so pixel noise never
// becomes a depth jump. Pure math over
// the frame it is given plus filter and hold state advanced on frame
// timestamps, so a replayed recording poses identically.

// The undistorted pinhole the landmarks live in
struct FrontCameraModel
{
	float fx= 0.f;
	float fy= 0.f;
	float cx= 0.f;
	float cy= 0.f;
	int width= 0;
	int height= 0;
};

// The user's rest as the solver measured it at the neutral capture (sitting
// square, hands resting), the zero every relative rotation is measured from.
// Stored in the camera-fixed world (the conversion at zero tilt), so editing
// the tilt afterwards moves the references with the measurements.
struct FrontCameraNeutralPose
{
	bool present= false;
	// The shoulder line's frame (no lean: the lean is measured against this
	// capture's own midpoint) and the shoulder midpoint, the lean's pivot
	// hangs one torso length below it
	glm::quat torso{1.f, 0.f, 0.f, 0.f};
	glm::vec3 shoulderMidpoint{0.f};
	// The head frame while looking at the screen; identity when the capture
	// could not see the face (looking straight at the camera). The hands
	// carry no reference: a straight wrist is the avatar's rest wrist.
	glm::quat head{1.f, 0.f, 0.f, 0.f};
	// The face shape ratios at rest, the zero of the webcam face source.
	// Absent when the capture did not see the face. The solver reads none
	// of it.
	bool facePresent= false;
	WholeBodyFaceRatios face;
};

struct FrontCameraRetargetConfig
{
	// The camera's downward pitch, radians, positive looking down
	float cameraTiltRadians= 0.f;
	FrontCameraNeutralPose neutral;
	// The user's head geometry and the shoulder width it was measured
	// against; the solver scales the head onto the avatar by the ratio of the
	// avatar's shoulder width to this one, so every length is in the avatar's
	// units
	float userShoulderWidthMeters= 0.40f;
	float headWidthMeters= 0.15f;
	float noseForwardMeters= 0.11f;
	// The user's arm lengths in the body model's landmark space (the
	// shoulder landmark sits inboard of the joint, so the upper arm runs
	// longer than the anatomical one), the same lengths the body solver
	// uses; the arm chain is solved with these through the ratio, so the
	// reach and the depth planes follow the user's proportions, and the
	// avatar's bones take the chain's directions
	float userUpperArmLengthMeters= 0.42f;
	float userForearmLengthMeters= 0.25f;
	// The wrist's distance along its ray: where the ray crosses a plane
	// parallel to the shoulders, a fraction of the arm's reach toward the
	// camera from the arm's shoulder, the near fraction with the palm at the
	// head and the far one otherwise, capped at the reach, with the elbow on
	// its bone circle where the forearm runs along the image line from the
	// wrist to the body model's elbow, or nearest the rig's hint without one.
	float nearPlaneReachFraction= 0.25f;
	float farPlaneReachFraction= 0.667f;
	// The rig's head trim, applied under the measured head the way the
	// position retarget applies it
	glm::vec3 headTrimRadians{0.f};
	// The rig's per-side adjustments, the same fields the position retarget
	// reads (the elbow hint is the IK fallback's target, the hand trim
	// orients the wrist's axes, the gains and enables shape the fingers,
	// the rest pose is where a lost arm goes)
	std::array<AvatarRetargetConfig::Side, 2> sides{};
	// How long an arm takes to ease into its rest pose, milliseconds; zero
	// snaps. The left arm chain's, carried with the front-camera pipeline's settings
	// for a caller that builds the loss stage's arm policy from this config
	// alone (makeFrontCameraLossPolicy): the solver does not read it, and the
	// retarget stage takes each arm's from its chain.
	float restBlendMs= 300.f;
	// The lifted palm's mirror along the view ray is taken on how much better
	// its finger axis continues the forearm than the lifted one's does, the
	// gain, which runs to 2 and is about 1 for a hand pointing along the
	// forearm at the camera. The gain is twice the finger axis's lean along
	// the view ray times the forearm's, so with the fingers upright it is
	// near zero and its sign is noise, while the mirror turns an edge-on
	// palm's normal right round. So the mirror is taken above one bar and
	// kept above a lower one, and under that the lifted palm stands. Upright
	// hands turned about the wrist, which need no mirror, reach a gain of
	// 0.3. Both bars at zero is the bare rule, the mirror wherever it
	// continues the forearm better at all.
	float palmMirrorEnterGain= 0.5f;
	float palmMirrorLeaveGain= 0.3f;
	// A hand with this many landmarks at the frame's edge or past it is cut
	// by the frame. The hand model lays the rest of a cut hand out by guess,
	// and the guess can be the other hand's layout, which turns the palm
	// right round. Such a hand keeps the wrist it gives the arm, and its
	// palm and fingers are held as through a dropout. Zero reads every hand.
	int handCutLandmarks= 3;
	// The rig's joint limits on the relative rotations, radians: the head
	// against the neutral head, the spine (the lean and the shoulder line's
	// turn) against the neutral torso, and each wrist (the palm in the
	// forearm frame), left then right
	float headLimitRadians= 70.f * glm::pi<float>() / 180.f;
	float spineLimitRadians= 45.f * glm::pi<float>() / 180.f;
	std::array<float, 2> wristLimitRadians{80.f * glm::pi<float>() / 180.f, 80.f * glm::pi<float>() / 180.f};
};

class FrontCameraRetarget
{
public:
	// The one camera-to-world conversion of the front-camera mode: OpenCV camera
	// axes (+X right, +Y down, +Z forward) to the app world (+X forward, +Y
	// left, +Z up) for a user FACING the camera, so the user faces world +X
	// like the avatar's rest and their left lands on +Y, then the camera's
	// downward tilt undone about world +Y. The camera is the origin.
	static glm::quat worldFromCamera(float tiltRadians);

	// The finger angles a single view reads with no zero captured from the
	// user: the hand model's own 3D landmarks against each finger's
	// metacarpal, less the bend the model reads on a straight finger facing
	// the camera (kFingerOpenBias, kThumbOpenBias), each bend floored at
	// zero. The lateral is the splay past the hand's own fan, unbiased. One
	// palmar memory per hand, reset when the hand is lost. A hand whose side
	// is decided takes the palmar side from it, as the lift does.
	static void measureMonoFingers(const TrackedHand& hand, HandPoseModel::PalmarSideMemory& ioMemory,
								   std::array<FingerAngles, FINGER_COUNT>& outAngles);

	// Forgets every filter and hold, and starts a new epoch on the source
	// pose clock
	void reset();

	// The measurement half: one camera's frame into the source pose, in the
	// avatar's units, and the solve state (getSolveState) but for what the
	// placement decides. camera: the per-camera result (hands with image
	// points and camera-space palm poses, the body observation). phoneHead:
	// valid while the phone face stream drives the head, already in the
	// canonical head frame. Owns the clock: a timestamp regression or a gap
	// past the hold restarts the solver and advances the epoch. Owns the
	// holds: a torso, a head or an arm within its hold reads as held, with
	// its last values.
	void measure(const TrackingFrameResult& camera, const FrontCameraModel& cameraModel,
				 const TrackingFrameResult::HeadPose& phoneHead, double timestampMs, const AvatarSkeleton& skeleton,
				 const FrontCameraRetargetConfig& config, SourcePose& outSource);

	// The placement half: a front-camera source pose into the avatar's bones. The
	// head and the wrist take their joint limits and the rig's trims here.
	// Every arm the source carries is placed from its placement, a resting
	// one (the loss stage's ease into the rig's rest pose) like a measured
	// one; a lost arm is left out. A head at rest is the identity, one easing
	// there the rotation the source carries. A lost torso leaves the spine
	// bones out and a lost head the head bone, each then at the avatar's
	// rest. Writes the solve state's head delta. Holds no state across frames.
	// Every arm by its bone angles alone.
	void place(const SourcePose& source, const AvatarSkeleton& skeleton, const FrontCameraRetargetConfig& config,
			   AvatarPose& outPose);

	// The same with each arm's IK weight, reach scale and goal offset (by
	// eHandSide; the elbow weight is not read, the source's elbow is the
	// pole). An arm at an IK weight of exactly 0 is placed by its bone angles
	// as above. Above it, the arm's wrist is pulled from where its bone
	// angles put it toward a goal in the posed avatar's world: the avatar's
	// shoulder plus the source's shoulder-to-wrist offset (the chain's, or
	// for a resting arm its directions at the source's bone lengths) scaled
	// by the reach scale's factor on the avatar-to-source arm length ratio,
	// plus the goal offset, clamped to the avatar's reach, the arm bending
	// toward the source's elbow scaled the same way. The forearm's twist, the
	// wrist and the fingers ride on the posed forearm as the bone angles
	// place them. outGoals, when given, receives each arm's goal (above
	// weight 0) and the weight it was solved at.
	void place(const SourcePose& source, const AvatarSkeleton& skeleton, const FrontCameraRetargetConfig& config,
			   const std::array<AvatarRetargetArm, 2>& arms, AvatarPose& outPose,
			   AvatarRetargetGoals* outGoals= nullptr);

	// The current filtered measurements as a neutral pose. False when no
	// torso has been solved yet.
	bool captureNeutral(FrontCameraNeutralPose& outNeutral) const;

	// The solved arm chain of the last frame, world space, for overlays and
	// tools
	struct ArmChain
	{
		bool valid= false;
		glm::vec3 shoulder{0.f};
		glm::vec3 elbow{0.f};
		glm::vec3 wrist{0.f};
	};
	const ArmChain& getArmChain(int sideIndex) const { return m_arms[sideIndex].chain; }

	// What the last solve measured and chose, world space (the camera at
	// the origin), for the diagnostic dump and the pose report: the pose
	// says where the bones went, this says why
	struct SolveState
	{
		struct Arm
		{
			ArmChain chain;
			bool bHandSeen= false;  // the hand slot agreed with the body model's wrist
			bool bElbowSeen= false; // the body model's elbow is past the visibility bar
			float depthLow= 0.f;    // the wrist distances the arm reaches along its ray
			float depthHigh= 0.f;
			float wristDepth= 0.f; // the chain's wrist distance along its ray
			// The arm is on the near depth plane, and the plane's distance
			// ahead of the shoulder as eased, avatar units
			bool bNearPlane= false;
			float planeDistance= 0.f;
			// How far the elbow has eased from the rig's hint onto the body
			// model's elbow direction, 0 to 1
			float elbowDirectionWeight= 0.f;
			// The chain was solved on a coasted wrist this frame, not a
			// measured one
			bool bWristCoasting= false;
			bool bHandMeasured= false;
			float twist= 0.f;
			glm::quat wristRotation{1.f, 0.f, 0.f, 0.f};
			glm::quat filteredPalm{1.f, 0.f, 0.f, 0.f};
			// The palm read this frame is the lifted palm's mirror along the
			// view ray, and how much better the mirror's finger axis
			// continues the forearm than the lifted one's (zero with no hand
			// read)
			bool bPalmMirrored= false;
			float palmMirrorGain= 0.f;
			// The hand is on the arm and cut by the frame's edge, so its palm
			// and fingers are held
			bool bHandCut= false;
			std::array<FingerAngles, FINGER_COUNT> measuredFingers{}; // this frame, before the filters
			std::array<FingerAngles, FINGER_COUNT> fingers{};         // filtered, what the bones took
			// The loss stage's readout, which the retarget stage writes from
			// the source pose it hands on (the solver leaves both at rest
			// values): no chain to place, the arm is easing into, or at, the
			// rest pose, and how far into it, 1 at rest
			bool bResting= false;
			float restBlend= 0.f;
		};
		FrontCameraModel camera;
		float tiltRadians= 0.f;
		bool torsoValid= false;
		glm::vec3 shoulder[2]{};
		glm::vec3 shoulderMidpoint{0.f};
		glm::quat torsoFrame{1.f, 0.f, 0.f, 0.f}; // the shoulder line's frame, no lean
		glm::quat spineDelta{1.f, 0.f, 0.f, 0.f};
		bool headMeasured= false;
		glm::quat headFrame{1.f, 0.f, 0.f, 0.f};
		glm::quat headDelta{1.f, 0.f, 0.f, 0.f};
		Arm arms[2];
		// The hand slots have been the opposite side's hands, and none its
		// own side's, for a second: the hand model's left and right are the
		// wrong way round for this camera. A slot's side is read from its
		// shape against the whole-body hands, or from the body wrists where
		// the body model gives no scored hands.
		bool bHandednessSwapped= false;
		// Per slot, the frames since the solver's reset on which the entry
		// gate turned the slot away while it sat on the other arm's wrist
		int oppositeWristRejections[2]= {0, 0};
		// Per arm, the frames since the solver's reset on which it took a
		// hand the camera had moved to its side from the other label
		int sideCorrections[2]= {0, 0};
	};
	const SolveState& getSolveState() const { return m_state; }

	// The camera's pose in the posed avatar's world (OpenCV camera axes to
	// world): the solve's world has the camera at the origin and the avatar's
	// has the root there, so the two differ by the measured shoulder
	// midpoint against the posed one. Identity before a torso is measured.
	static glm::mat4 makeCameraToWorld(const AvatarSkeleton& skeleton, const AvatarPose& pose,
									   const SolveState& state);

	// What places an arm's bones: the chain's directions (world, unit), the
	// forearm's roll against the avatar's rest palm, the palm in the forearm
	// frame, and the finger angles. A solved chain fills one, the rest pose
	// fills one, and the ease into rest blends between them.
	struct ArmPlacement
	{
		glm::vec3 upperDirection{0.f};
		glm::vec3 forearmDirection{0.f};
		float twist= 0.f;
		glm::quat wristRotation{1.f, 0.f, 0.f, 0.f};
		std::array<FingerAngles, FINGER_COUNT> fingers{};
	};

	// The rig's rest placement of one arm under the lean: the chain's
	// directions from the rest offsets, the palm facing the rest normal
	// through the same hand math as a measured palm, the four fingers at the
	// rest curl and the thumb at the avatar's rest
	static ArmPlacement makeRestPlacement(const AvatarSkeleton& skeleton, int sideIndex, const glm::quat& spineDelta,
										  const AvatarRetargetConfig::Side& sideConfig);

	// A placement for a chain posed by hand rather than measured: the chain's
	// directions through the given elbow, or through the elbow on the bone
	// circle nearest elbowHint when elbow is null (straight toward the wrist
	// when it is out of reach), and a world palm frame through the same hand
	// math as a measured palm. outElbow and outWrist receive the chain's
	// joints at the avatar's bone lengths, the wrist being wristTarget when
	// it is within reach.
	static ArmPlacement makePosedPlacement(const AvatarSkeleton& skeleton, int sideIndex, const glm::quat& spineDelta,
										   const glm::vec3& shoulder, const glm::vec3& wristTarget,
										   const glm::vec3* elbow, const glm::vec3& elbowHint, const glm::quat& palm,
										   const std::array<FingerAngles, FINGER_COUNT>& fingers, glm::vec3& outElbow,
										   glm::vec3& outWrist);

	// The placement part way from one to another: the directions and the
	// finger angles linearly, the twist by the shorter way round, the wrist
	// along the arc
	static ArmPlacement blendPlacements(const ArmPlacement& from, const ArmPlacement& to, float t);

	// Both arms at the rig's rest pose with the spine straight, the rest of
	// the pose absent: what a lost arm has eased into, for the preview and
	// the tests
	static void makeRestPose(const AvatarSkeleton& skeleton, const FrontCameraRetargetConfig& config,
							 AvatarPose& outPose);

	// The hand model's bend on a straight finger facing the camera, radians
	// (proximal, intermediate, distal): 10, 25 and 16 degrees for the four
	// fingers, 21, 10 and 0.5 for the thumb, the medians over held open hands
	static constexpr float kFingerOpenBias[3]= {0.1745f, 0.4363f, 0.2793f};
	static constexpr float kThumbOpenBias[3]= {0.3665f, 0.1745f, 0.0087f};
	static constexpr float kMinBendRadians= 0.f;

private:
	void configureFilters();
	void resetMeasure();

	struct ArmState
	{
		ArmChain chain;
		double solvedTimestampMs= -1.0;
		// When the chain's current unbroken run of placements began: the
		// hold it has earned is the run's length, up to the maximum
		double sightingStartMs= -1.0;
		// When the arm last had evidence, a hand slot on the body wrist or a
		// placed arm's wrist clear of the frame's edge: the clock the hold
		// runs on
		double evidenceTimestampMs= -1.0;
		// The near depth plane is the one in force, and the plane's distance
		// ahead of the shoulder as it eases between the two
		bool bNearPlane= false;
		bool bPlaneDistance= false;
		float planeDistance= 0.f;
		// and how far the elbow has eased from the rig's hint onto the body
		// model's elbow direction, 0 to 1
		float elbowDirectionWeight= 0.f;
		// The elbow's turn about the shoulder to wrist line away from the
		// rig's hint, radians: the one the direction search chose last, raw,
		// which the search's continuity term and a frame without a direction
		// read, and the eased turn unwrapped for its filter
		bool bSwivelAngle= false;
		float swivelAngle= 0.f;
		bool bSwivelUnwrapped= false;
		float swivelUnwrapped= 0.f;
		OneEuroFilter swivelFilter;
		// The wrist's pixel as last measured with its speed in the image, the
		// pixel the chain was last solved on (measured, glided or coasted),
		// the coast of a wrist that is not measured, and the glide back onto
		// one that is: the offset it started with and when (negative outside
		// one). And whether the arm's last evidence was in the band along the
		// frame's edge, which shortens its hold.
		bool bMeasuredWristPx= false;
		glm::vec2 measuredWristPx{0.f};
		double measuredWristMs= 0.0;
		glm::vec2 wristVelocityPx{0.f};
		bool bSolvedWristPx= false;
		glm::vec2 solvedWristPx{0.f};
		bool bSolvedByCoast= false;
		bool bCoasting= false;
		glm::vec2 coastOriginPx{0.f};
		glm::vec2 glideOffsetPx{0.f};
		double glideStartMs= -1.0;
		bool bEvidenceAtEdge= false;
		// The entry gates on the body wrist and on the hand slot's agreement
		// with it: consecutive frames of evidence, and whether in
		int wristGateFrames= 0;
		bool bWristAccepted= false;
		int handGateFrames= 0;
		bool bHandAccepted= false;
		// The hand as last measured, held while the hand model loses it
		bool bHandMeasured= false;
		float twist= 0.f;                            // palm normal roll about the forearm vs the rest palm
		glm::quat wristRotation{1.f, 0.f, 0.f, 0.f}; // palm in the measured forearm frame
		std::array<FingerAngles, FINGER_COUNT> fingers{};
		// Which side of the hand model's palm plane is the palmar one,
		// carried between frames of one tracked hand
		HandPoseModel::PalmarSideMemory palmarMemory;
		// The filtered palm frame the hold compares against, and how many
		// consecutive frames the raw palm has jumped away from it
		bool bPalmFiltered= false;
		glm::quat filteredPalm{1.f, 0.f, 0.f, 0.f};
		int palmJumpFrames= 0;
		// The lifted palm's mirror along the view ray is the one in force
		bool bPalmMirrored= false;
		// Pixels lightly, the palm axes as directions
		OneEuroFilterVec2 elbowPixelFilter;
		OneEuroFilterVec2 wristPixelFilter;
		OneEuroFilterVec3 palmXFilter;
		OneEuroFilterVec3 palmZFilter;
		std::array<OneEuroFilter, FINGER_COUNT * 4> fingerFilters;
	};
	struct TorsoState
	{
		bool valid= false;
		double solvedTimestampMs= -1.0;
		glm::vec3 shoulder[2]{};
		glm::vec3 midpoint{0.f};
		glm::quat lineFrame{1.f, 0.f, 0.f, 0.f}; // the shoulder line's frame, world, no lean
		OneEuroFilterVec2 shoulderPixelFilter[2];
		OneEuroFilter depthFilter;
	};
	struct HeadState
	{
		bool bMeasured= false; // by the camera; the phone head never lands here
		double solvedTimestampMs= -1.0;
		glm::quat frame{1.f, 0.f, 0.f, 0.f};
		// The head's centre and width in the image, from the ear pixels
		glm::vec2 earMidPx{0.f};
		float earSeparationPx= 0.f;
		OneEuroFilterVec2 nosePixelFilter;
		OneEuroFilterVec2 earPixelFilter[2];
		OneEuroFilter depthFilter;
	};

	// The source clock, owned by measure; the epoch survives a reset
	uint32_t m_resetEpoch= 0;
	bool m_bHasTime= false;
	double m_lastTimestampMs= 0.0;
	bool m_bFiltersConfigured= false;
	// When the current run of frames with swapped hand labels began,
	// negative outside one
	double m_handednessSwapStartMs= -1.0;
	int m_oppositeWristRejections[2]= {0, 0};
	int m_sideCorrections[2]= {0, 0};
	float m_tiltRadians= 0.f;
	TorsoState m_torso;
	HeadState m_head;
	ArmState m_arms[2];
	SolveState m_state;
};
