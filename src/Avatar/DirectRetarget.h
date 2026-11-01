#pragma once

#include <array>

#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/vector_float3.hpp"

#include "AvatarRetarget.h"
#include "OneEuroFilter.h"
#include "TrackingTypes.h"

class AvatarSkeleton;

// The single-camera direct retarget: one camera's hand landmarks and
// whole-body observation become avatar bone rotations with no world
// calibration. The camera is the world, the avatar's own bone lengths are
// the scale, and every segment is placed where its known length meets the
// ray through its landmark, with the model's relative depth deciding only
// which of the two meeting points is real. Pure math over the frame it is
// given plus filter and hold state advanced on frame timestamps, so a
// replayed recording poses identically.

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
	// could not see the face (looking straight at the camera)
	glm::quat head{1.f, 0.f, 0.f, 0.f};
	// Per side: the palm in the measured forearm frame (identity = a straight
	// wrist) and the palm normal's roll about the forearm against the
	// avatar's rest palm, radians
	std::array<glm::quat, 2> wrist{glm::quat(1.f, 0.f, 0.f, 0.f), glm::quat(1.f, 0.f, 0.f, 0.f)};
	std::array<float, 2> forearmTwist{0.f, 0.f};
};

// What one capture measured: the neutral pose plus the camera tilt the torso
// implied, when the hips were in frame to imply one
struct DirectNeutralCapture
{
	DirectNeutralPose neutral;
	bool bTiltMeasured= false;
	float cameraTiltRadians= 0.f;
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
	// The hand model's rest zero (the fused rest angles), subtracted from the
	// raw per-camera angles the way fusion does on the stereo path
	bool bHasRestAngles[2]= {false, false};
	std::array<std::array<FingerAngles, FINGER_COUNT>, 2> restAngles{};
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

	void reset();

	// camera: the per-camera result (hands with image points and camera-space
	// palm poses, the body observation). phoneHead: valid while the phone
	// face stream drives the head, already in the canonical head frame.
	void solve(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
			   const TrackingFrameResult::HeadPose& phoneHead, double timestampMs, const AvatarSkeleton& skeleton,
			   const DirectRetargetConfig& config, AvatarPose& outPose);

	// The current filtered measurements as a neutral pose. False when no
	// torso has been solved yet.
	bool captureNeutral(DirectNeutralCapture& outCapture) const;

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

private:
	void configureFilters();

	struct ArmState
	{
		ArmChain chain;
		double solvedTimestampMs= -1.0;
		// The hand as last measured, held while the hand model loses it
		bool bHandMeasured= false;
		float twist= 0.f;                            // palm normal roll about the forearm vs the rest palm
		glm::quat wristRotation{1.f, 0.f, 0.f, 0.f}; // palm in the measured forearm frame
		std::array<FingerAngles, FINGER_COUNT> fingers{};
		OneEuroFilterVec3 elbowFilter;
		OneEuroFilterVec3 wristFilter;
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
		// The torso's up direction when the hips were in frame, for the tilt
		// estimate
		bool bHasUpAxis= false;
		glm::vec3 upAxis{0.f, 0.f, 1.f};
		OneEuroFilterVec3 shoulderFilter[2];
	};
	struct HeadState
	{
		bool bMeasured= false; // by the camera; the phone head never lands here
		double solvedTimestampMs= -1.0;
		glm::quat frame{1.f, 0.f, 0.f, 0.f};
		OneEuroFilterVec3 leftFilter;
		OneEuroFilterVec3 forwardFilter;
	};

	bool m_bHasTime= false;
	double m_lastTimestampMs= 0.0;
	bool m_bFiltersConfigured= false;
	float m_tiltRadians= 0.f;
	TorsoState m_torso;
	HeadState m_head;
	ArmState m_arms[2];
};
