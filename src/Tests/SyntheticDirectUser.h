#pragma once

#include "glm/gtc/quaternion.hpp"

#include "AvatarSkeleton.h"
#include "DirectRetarget.h"
#include "HandPoseModel.h"
#include "SyntheticAvatar.h"
#include "TrackingTypes.h"

// A synthetic pinhole pitched down at a synthetic user built from the
// synthetic avatar's own lengths, and the per-camera frame the direct
// retarget reads from them. Shared by the direct retarget's self test and
// the retarget golden test, so both drive the solver with the same input.

namespace SyntheticDirectUser
{
using namespace SyntheticAvatar;

// The model's relative depth arrives compressed (measured 0.6 of truth on
// the webcam take); the solver may read only its sign
constexpr float kModelDepthScale= 0.6f;

// The torso length the synthetic rig is authored with (shoulder height
// above the hips), which the user below is built to
constexpr float kTorsoLength= 0.4f;

struct Camera
{
	DirectCameraModel model;
	glm::quat worldFromCamera{1.f, 0.f, 0.f, 0.f};
	glm::quat cameraFromWorld{1.f, 0.f, 0.f, 0.f};
};

inline Camera makeCamera(float tiltDegrees)
{
	Camera camera;
	camera.model.fx= 300.f;
	camera.model.fy= 300.f;
	camera.model.cx= 320.f;
	camera.model.cy= 240.f;
	camera.model.width= 640;
	camera.model.height= 480;
	camera.worldFromCamera= DirectRetarget::worldFromCamera(glm::radians(tiltDegrees));
	camera.cameraFromWorld= glm::inverse(camera.worldFromCamera);
	return camera;
}

inline glm::vec3 toCamera(const Camera& camera, const glm::vec3& world)
{
	return camera.cameraFromWorld * world;
}

inline glm::vec2 project(const Camera& camera, const glm::vec3& world)
{
	const glm::vec3 c= toCamera(camera, world);
	return glm::vec2(camera.model.fx * c.x / c.z + camera.model.cx, camera.model.fy * c.y / c.z + camera.model.cy);
}

// A palm frame from its finger direction and its palmar normal
inline glm::quat makePalm(const glm::vec3& x, const glm::vec3& z)
{
	const glm::vec3 xUnit= glm::normalize(x);
	const glm::vec3 zUnit= glm::normalize(z - xUnit * glm::dot(z, xUnit));
	return glm::quat_cast(glm::mat3(xUnit, glm::cross(zUnit, xUnit), zUnit));
}

// The palm a single view lifts when it guesses the hand's depth direction
// wrong: mirrored along the view ray through it, the normal flipped so the
// frame stays right-handed
inline glm::quat mirrorPalmAlongRay(const glm::quat& palm, const glm::vec3& rayDirection)
{
	const glm::vec3 ray= glm::normalize(rayDirection);
	const glm::vec3 x= palm * glm::vec3(1.f, 0.f, 0.f);
	const glm::vec3 z= palm * glm::vec3(0.f, 0.f, 1.f);
	return makePalm(x - 2.f * glm::dot(x, ray) * ray, -(z - 2.f * glm::dot(z, ray) * ray));
}

// The user in the direct world (the camera at the origin, the user facing
// +X from 0.8 m away, the camera a quarter meter above the shoulders),
// built from the avatar's lengths
struct User
{
	glm::vec3 shoulder[2];
	glm::vec3 elbow[2];
	glm::vec3 wrist[2];
	glm::vec3 hip[2];
	glm::vec3 ear[2];
	glm::vec3 nose;
	glm::quat palm[2];
	bool bHandTracked[2]= {true, true};
	// Whether a tracked slot's lift solved: a slot without it is a hand on
	// the wrist with no distance to offer
	bool bHandLifted[2]= {true, true};
	bool bElbowSeen[2]= {true, true};
	bool bWristSeen[2]= {true, true};
	// The body model's visibility on a seen wrist
	float wristVisibility[2]= {1.f, 1.f};
	// Whether the body model reports its relative depth
	bool bModelDepth= true;
	// The hand lift's wrist distance as a multiple of the truth
	float pnpDepthScale[2]= {1.f, 1.f};
	// Where the hand slot sits relative to the wrist's true pixel: a slot
	// on something else in the scene
	glm::vec2 handPixelOffset[2]{};
	std::array<std::array<FingerAngles, FINGER_COUNT>, 2> fingers{};
};

inline User makeNeutralUser()
{
	User user;
	const float x= -0.8f;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const float sy= sideIndex == 0 ? 1.f : -1.f;
		user.shoulder[sideIndex]= glm::vec3(x, sy * 0.5f * kShoulderWidth, -0.25f);
		// Arms hanging, forearms forward onto the desk, palms down
		user.elbow[sideIndex]= user.shoulder[sideIndex] + glm::vec3(0.f, 0.f, -kUpperArmLength);
		user.wrist[sideIndex]= user.elbow[sideIndex] + glm::vec3(kForearmLength, 0.f, 0.f);
		user.hip[sideIndex]= glm::vec3(x, sy * 0.08f, -0.25f - kTorsoLength);
		user.ear[sideIndex]= glm::vec3(x, sy * 0.075f, 0.05f);
		user.palm[sideIndex]= makePalm(glm::vec3(1.f, 0.f, 0.f), glm::vec3(0.f, 0.f, -1.f));
	}
	user.nose= glm::vec3(x + 0.11f, 0.f, 0.05f);
	return user;
}

inline void setLandmark(BodyPoseObservation& body, const Camera& camera, int index, const glm::vec3& world,
				 float hipDepth, bool bSeen)
{
	const glm::vec2 px= bSeen ? project(camera, world) : glm::vec2(-100.f, -100.f);
	const float depth= toCamera(camera, world).z;
	body.imagePoints[index]= glm::vec3(px.x, px.y, (depth - hipDepth) * kModelDepthScale);
	body.visibility[index]= bSeen ? 1.f : 0.1f;
	body.providedMask|= 1ull << index;
}

inline TrackingFrameResult makeFrame(const Camera& camera, const User& user, const AvatarSkeleton& skeleton,
							  double timestampMs, int64_t frameIndex)
{
	TrackingFrameResult frame;
	frame.frameIndex= frameIndex;
	frame.timestampMs= timestampMs;
	frame.frameWidth= camera.model.width;
	frame.frameHeight= camera.model.height;

	BodyPoseObservation& body= frame.body;
	body.valid= true;
	body.boxSource= eBodyBoxSource::Detector;
	body.modelFrameIndex= frameIndex;
	body.confidence= 1.f;
	body.hasDepth= user.bModelDepth;
	const float hipDepth= toCamera(camera, 0.5f * (user.hip[0] + user.hip[1])).z;
	setLandmark(body, camera, (int)ePoseLandmark::NOSE, user.nose, hipDepth, true);
	setLandmark(body, camera, (int)ePoseLandmark::LEFT_EAR, user.ear[0], hipDepth, true);
	setLandmark(body, camera, (int)ePoseLandmark::RIGHT_EAR, user.ear[1], hipDepth, true);
	setLandmark(body, camera, (int)ePoseLandmark::LEFT_SHOULDER, user.shoulder[0], hipDepth, true);
	setLandmark(body, camera, (int)ePoseLandmark::RIGHT_SHOULDER, user.shoulder[1], hipDepth, true);
	setLandmark(body, camera, (int)ePoseLandmark::LEFT_ELBOW, user.elbow[0], hipDepth, user.bElbowSeen[0]);
	setLandmark(body, camera, (int)ePoseLandmark::RIGHT_ELBOW, user.elbow[1], hipDepth, user.bElbowSeen[1]);
	setLandmark(body, camera, (int)ePoseLandmark::LEFT_WRIST, user.wrist[0], hipDepth, user.bWristSeen[0]);
	setLandmark(body, camera, (int)ePoseLandmark::RIGHT_WRIST, user.wrist[1], hipDepth, user.bWristSeen[1]);
	if (user.bWristSeen[0])
		body.visibility[(int)ePoseLandmark::LEFT_WRIST]= user.wristVisibility[0];
	if (user.bWristSeen[1])
		body.visibility[(int)ePoseLandmark::RIGHT_WRIST]= user.wristVisibility[1];
	setLandmark(body, camera, (int)ePoseLandmark::LEFT_HIP, user.hip[0], hipDepth, true);
	setLandmark(body, camera, (int)ePoseLandmark::RIGHT_HIP, user.hip[1], hipDepth, true);

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		TrackedHand& hand= frame.hands[sideIndex];
		HandPose& pose= frame.poses[sideIndex];
		hand.side= (eHandSide)sideIndex;
		pose.side= (eHandSide)sideIndex;
		if (!user.bHandTracked[sideIndex])
			continue;
		hand.tracked= true;
		hand.presence= 1.f;
		const glm::vec2 wristPx= project(camera, user.wrist[sideIndex]) + user.handPixelOffset[sideIndex];
		// Every landmark at the wrist: the solver reads the wrist's pixel, and
		// counts the landmarks at the frame's edge
		hand.imagePoints.fill(glm::vec3(wristPx.x, wristPx.y, 0.f));
		pose.tracked= true;
		pose.presence= 1.f;
		pose.confidence= 1.f;
		pose.hasCameraPose= user.bHandLifted[sideIndex];
		pose.palmOrientationCamera= camera.cameraFromWorld * user.palm[sideIndex];
		pose.palmPositionCamera= toCamera(camera, user.wrist[sideIndex]) * user.pnpDepthScale[sideIndex];
		pose.fingers= user.fingers[sideIndex];
		pose.skeleton= skeleton.getHand((eHandSide)sideIndex).skeleton;

		// The hand model's own 3D landmarks, what the solver measures the
		// fingers from: the avatar's hand skeleton posed by the user's
		// angles against each finger's metacarpal, with the bend the model
		// reads on a straight finger added, so the measurement recovers the
		// user's angles. In the palm frame, the wrist on -X of the middle base.
		HandSkeleton synth= pose.skeleton;
		const glm::vec3 wristInPalm(-synth.baseInPalm[(int)eFinger::Middle].x, 0.f, 0.f);
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			const glm::vec3 metacarpal= synth.baseInPalm[finger] - wristInPalm;
			synth.neutralDirInPalm[finger]= glm::normalize(glm::vec3(metacarpal.x, metacarpal.y, 0.f));
		}
		std::array<FingerAngles, FINGER_COUNT> biased= user.fingers[sideIndex];
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			const float* bias=
				finger == (int)eFinger::Thumb ? DirectRetarget::kThumbOpenBias : DirectRetarget::kFingerOpenBias;
			biased[finger].proximal+= bias[0];
			biased[finger].intermediate+= bias[1];
			biased[finger].distal+= bias[2];
		}
		std::array<std::array<glm::vec3, 4>, FINGER_COUNT> joints;
		HandPoseModel::buildFingerJoints(glm::mat4(1.f), synth, biased, joints);
		hand.modelPoints[(int)eHandLandmark::WRIST]= wristInPalm;
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
			for (int joint= 0; joint < 4; ++joint)
				hand.modelPoints[FINGER_JOINTS[finger][joint]]= joints[finger][joint];
	}
	return frame;
}
} // namespace SyntheticDirectUser
