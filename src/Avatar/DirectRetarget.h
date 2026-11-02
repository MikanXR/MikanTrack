#pragma once

#include <array>

#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/vector_float3.hpp"

#include "AvatarRetarget.h"
#include "HandPoseModel.h"
#include "OneEuroFilter.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

// The single-camera direct retarget: one camera's hand landmarks and
// whole-body observation become avatar bone rotations with no world
// calibration. The camera is the world, the avatar's own bone lengths are
// the scale, and every joint sits on the ray through its landmark, so the
// posed avatar reprojects onto what the camera saw. The one thing a single
// view cannot see, the wrist's distance along its ray, is scored over the
// reach the arm allows: the elbow's reprojection, the model's relative
// depth and the last frame's chain each pull on it, and the hand lift's
// depth is measured against the result. Pixels are filtered lightly and
// depths heavily, so pixel noise never becomes a depth jump. Pure math over
// the frame it is given plus filter and hold state advanced on frame
// timestamps, so a replayed recording poses identically.

// The undistorted pinhole the landmarks live in
struct DirectCameraModel
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
struct DirectNeutralPose
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
};

struct DirectRetargetConfig
{
	// The camera's downward pitch, radians, positive looking down
	float cameraTiltRadians= 0.f;
	DirectNeutralPose neutral;
	// The user's head geometry and the shoulder width it was measured
	// against; the solver scales the head onto the avatar by the ratio of the
	// avatar's shoulder width to this one, so every length is in the avatar's
	// units
	float userShoulderWidthMeters= 0.40f;
	float headWidthMeters= 0.15f;
	float noseForwardMeters= 0.11f;
	// The rig's head trim, applied under the measured head the way the
	// position retarget applies it
	glm::vec3 headTrimRadians{0.f};
	// The rig's per-side adjustments, the same fields the position retarget
	// reads (the elbow hint is the IK fallback's target, the hand trim
	// orients the wrist's axes, the gains and enables shape the fingers)
	std::array<AvatarRetargetConfig::Side, 2> sides{};
};

class DirectRetarget
{
public:
	// The one camera-to-world conversion of the direct mode: OpenCV camera
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
	// palmar memory per hand, reset when the hand is lost.
	static void measureMonoFingers(const TrackedHand& hand, HandPoseModel::PalmarSideMemory& ioMemory,
								   std::array<FingerAngles, FINGER_COUNT>& outAngles);

	void reset();

	// camera: the per-camera result (hands with image points and camera-space
	// palm poses, the body observation). phoneHead: valid while the phone
	// face stream drives the head, already in the canonical head frame.
	void solve(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
			   const TrackingFrameResult::HeadPose& phoneHead, double timestampMs, const AvatarSkeleton& skeleton,
			   const DirectRetargetConfig& config, AvatarPose& outPose);

	// The current filtered measurements as a neutral pose. False when no
	// torso has been solved yet.
	bool captureNeutral(DirectNeutralPose& outNeutral) const;

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

	// The hand lift's depth against the chain's, as measured so far: the
	// factor the lift's wrist distance would be multiplied by to match the
	// chain, and how many frames with a seen elbow set it. Before any sample
	// it is the avatar-to-user length ratio. A measurement the gate tool
	// reports; the score does not read it.
	struct PnpScale
	{
		float scale= 1.f;
		int sampleCount= 0;
	};
	PnpScale getPnpScale(int sideIndex) const;

	// The running median's window, frames with a seen elbow
	static constexpr int kPnpScaleSamples= 90;

	// The hand model's bend on a straight finger facing the camera, radians
	// (proximal, intermediate, distal): 10, 25 and 16 degrees for the four
	// fingers, 21, 10 and 0.5 for the thumb, the medians over held open hands
	static constexpr float kFingerOpenBias[3]= {0.1745f, 0.4363f, 0.2793f};
	static constexpr float kThumbOpenBias[3]= {0.3665f, 0.1745f, 0.0087f};
	static constexpr float kMinBendRadians= 0.f;

private:
	void configureFilters();

	struct ArmState
	{
		ArmChain chain;
		double solvedTimestampMs= -1.0;
		// The wrist's distance along its ray as the scorer chose it last
		// frame, before the depth filter: the smoothness term's anchor and a
		// candidate of its own
		float lastChosenDepth= 0.f;
		// The hand lift's depth scale (see PnpScale): the ring of samples,
		// and the median over them
		std::array<float, kPnpScaleSamples> pnpScaleSamples{};
		int pnpScaleSampleCount= 0;
		int pnpScaleNextSample= 0;
		float pnpScale= 1.f;
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
		// Pixels lightly, the depth heavily, the palm axes as directions
		OneEuroFilterVec2 elbowPixelFilter;
		OneEuroFilterVec2 wristPixelFilter;
		OneEuroFilter depthFilter;
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
		OneEuroFilterVec2 nosePixelFilter;
		OneEuroFilterVec2 earPixelFilter[2];
		OneEuroFilter depthFilter;
	};

	bool m_bHasTime= false;
	double m_lastTimestampMs= 0.0;
	bool m_bFiltersConfigured= false;
	float m_tiltRadians= 0.f;
	TorsoState m_torso;
	HeadState m_head;
	ArmState m_arms[2];
};
