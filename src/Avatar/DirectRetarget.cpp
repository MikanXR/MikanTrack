#include "DirectRetarget.h"

#include <algorithm>
#include <cmath>

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/quaternion.hpp"

#include "AvatarRetargetMath.h"
#include "AvatarSkeleton.h"
#include "BodyPoseSolver.h"
#include "BodyPoseTypes.h"
#include "MathGLM.h"

namespace
{
using B= eHumanoidBone;
using namespace AvatarRetargetMath;
using RaySphereHit= BodyPoseSolver::RaySphereHit;

// Landmark visibility below this is treated as unseen
constexpr float kMinVisibility= 0.5f;
// A landmark this close to the frame edge is treated as out of frame: the
// model places a joint it cannot see somewhere plausible inside the image,
// and that somewhere is not where the joint is
constexpr float kFrameMarginPx= 16.f;
// The model's relative depth decides a segment's near-or-far choice only
// when the two landmarks differ by at least this much; below it the choice
// carries over from the last frame
constexpr float kDepthSignMinMeters= 0.1f;
// A group (torso, head, an arm) unseen for longer than this returns to rest,
// and a gap longer than this restarts the filters
constexpr double kMaxHoldMs= 500.0;
// A pair of landmarks whose known separation places it outside this range
// is not on the person
constexpr float kMinBodyDepthMeters= 0.15f;
constexpr float kMaxBodyDepthMeters= 2.0f;
// Joint limits on the relative rotations, radians
constexpr float kHeadLimitRadians= 70.f * glm::pi<float>() / 180.f;
constexpr float kWristLimitRadians= 80.f * glm::pi<float>() / 180.f;
constexpr float kSpineLimitRadians= 45.f * glm::pi<float>() / 180.f;
// One-euro parameters, the body solver's
constexpr float kPositionMinCutoffHz= 1.2f;
constexpr float kPositionBeta= 0.05f;
constexpr float kDirectionMinCutoffHz= 1.5f;
constexpr float kDirectionBeta= 0.08f;
constexpr float kDerivativeCutoffHz= 1.0f;

const glm::vec3 k_up(0.f, 0.f, 1.f);
const glm::vec3 k_left(0.f, 1.f, 0.f);
const glm::vec3 k_origin(0.f);

// Which of a ray's two sphere crossings a segment takes: the one whose depth
// relation to the segment's start matches the model's sign when it has one,
// else the one that changes depth least. An undecided sign means the model
// saw little depth change, and the crossing with the least is the one that
// agrees with it.
glm::vec3 pickCrossing(const RaySphereHit& hit, const glm::vec3& cameraForward, const glm::vec3& start, int depthSign)
{
	if (!hit.bNearValid)
		return hit.farPoint;
	if (!hit.bFarValid || hit.bClamped)
		return hit.nearPoint;
	const float nearDelta= glm::dot(hit.nearPoint - start, cameraForward);
	const float farDelta= glm::dot(hit.farPoint - start, cameraForward);
	if (depthSign != 0 && (nearDelta > 0.f) != (farDelta > 0.f))
		return ((depthSign > 0) == (farDelta > 0.f)) ? hit.farPoint : hit.nearPoint;
	return std::fabs(nearDelta) <= std::fabs(farDelta) ? hit.nearPoint : hit.farPoint;
}

glm::vec3 restDirection(const AvatarSkeleton& skeleton, B bone)
{
	return safeNormalize(skeleton.getBone(bone).restOffsetFromParentWorld);
}

void placeBone(AvatarPose& pose, const AvatarSkeleton& skeleton, B bone, const glm::quat& delta)
{
	if (!skeleton.getBone(bone).present)
		return;
	AvatarPose::Bone& out= pose.bones[(int)bone];
	out.present= true;
	out.deltaWorld= delta;
}

// An arm at rest: every bone of the chain present at identity, so a receiver
// is told the arm rests rather than left holding its last pose
void restArm(AvatarPose& pose, const AvatarSkeleton& skeleton, int sideIndex)
{
	const glm::quat identity(1.f, 0.f, 0.f, 0.f);
	placeBone(pose, skeleton, sideBone(sideIndex, B::LeftUpperArm, B::RightUpperArm), identity);
	placeBone(pose, skeleton, sideBone(sideIndex, B::LeftLowerArm, B::RightLowerArm), identity);
	placeBone(pose, skeleton, sideBone(sideIndex, B::LeftHand, B::RightHand), identity);
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
		for (int phalanx= 0; phalanx < 3; ++phalanx)
			placeBone(pose, skeleton, fingerBone(sideIndex, finger, phalanx), identity);
}

// A rotation from three unit axes that may have drifted apart under
// independent filtering: forward survives, up is rebuilt from it and left,
// left from the two
bool frameFromForwardAndLeft(const glm::vec3& forward, const glm::vec3& left, glm::quat& outFrame)
{
	const glm::vec3 forwardUnit= safeNormalize(forward);
	const glm::vec3 up= glm::cross(forwardUnit, left);
	if (glm::dot(forwardUnit, forwardUnit) <= 0.f || glm::dot(up, up) < 1e-8f)
		return false;
	const glm::vec3 upUnit= glm::normalize(up);
	const glm::vec3 leftUnit= glm::cross(upUnit, forwardUnit);
	outFrame= glm::quat_cast(glm::mat3(forwardUnit, leftUnit, upUnit));
	return true;
}
} // namespace

glm::quat DirectRetarget::worldFromCamera(float tiltRadians)
{
	// Columns: the world images of camera +X, +Y and +Z for a level camera.
	// A camera pitched down by the tilt sees the world rolled up about its
	// own right axis (world +Y), so the tilt is undone by the opposite turn.
	static const glm::mat3 k_levelWorldFromCamera(glm::vec3(0.f, 1.f, 0.f), glm::vec3(0.f, 0.f, -1.f),
												  glm::vec3(-1.f, 0.f, 0.f));
	return glm::angleAxis(-tiltRadians, glm::vec3(0.f, 1.f, 0.f)) * glm::quat_cast(k_levelWorldFromCamera);
}

void DirectRetarget::reset()
{
	m_bHasTime= false;
	m_lastTimestampMs= 0.0;
	m_torso= TorsoState();
	m_head= HeadState();
	for (ArmState& arm : m_arms)
		arm= ArmState();
	m_bFiltersConfigured= false;
}

void DirectRetarget::configureFilters()
{
	for (OneEuroFilterVec3& filter : m_torso.shoulderFilter)
		filter.configure(kPositionMinCutoffHz, kPositionBeta, kDerivativeCutoffHz);
	m_head.leftFilter.configure(kDirectionMinCutoffHz, kDirectionBeta, kDerivativeCutoffHz);
	m_head.forwardFilter.configure(kDirectionMinCutoffHz, kDirectionBeta, kDerivativeCutoffHz);
	for (ArmState& arm : m_arms)
	{
		arm.elbowFilter.configure(kPositionMinCutoffHz, kPositionBeta, kDerivativeCutoffHz);
		arm.wristFilter.configure(kPositionMinCutoffHz, kPositionBeta, kDerivativeCutoffHz);
		arm.palmXFilter.configure(kDirectionMinCutoffHz, kDirectionBeta, kDerivativeCutoffHz);
		arm.palmZFilter.configure(kDirectionMinCutoffHz, kDirectionBeta, kDerivativeCutoffHz);
		for (OneEuroFilter& filter : arm.fingerFilters)
			filter.configure(kDirectionMinCutoffHz, kDirectionBeta, kDerivativeCutoffHz);
	}
	m_bFiltersConfigured= true;
}

void DirectRetarget::solve(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
						   const TrackingFrameResult::HeadPose& phoneHead, double timestampMs,
						   const AvatarSkeleton& skeleton, const DirectRetargetConfig& config, AvatarPose& outPose)
{
	// The frame clock. A regression (a replay scrub) or a gap past the hold
	// starts over: nothing measured before it should shape what comes after.
	if (m_bHasTime)
	{
		const double gapMs= timestampMs - m_lastTimestampMs;
		if (gapMs < 0.0 || gapMs > kMaxHoldMs)
			reset();
	}
	if (!m_bFiltersConfigured)
		configureFilters();
	const float dtSeconds= m_bHasTime ? (float)((timestampMs - m_lastTimestampMs) * 0.001) : 0.f;
	m_bHasTime= true;
	m_lastTimestampMs= timestampMs;

	outPose.clear();
	outPose.valid= true;
	outPose.timestampMs= timestampMs;
	// The root is fixed: the camera is the world and the user faces it
	placeBone(outPose, skeleton, B::Hips, glm::quat(1.f, 0.f, 0.f, 0.f));

	m_tiltRadians= config.cameraTiltRadians;
	const glm::quat worldFromCam= worldFromCamera(config.cameraTiltRadians);
	// The stored neutral references live in the level frame; this carries
	// them to the current tilt
	const glm::quat tilt= worldFromCam * glm::inverse(worldFromCamera(0.f));
	const glm::vec3 cameraForward= worldFromCam * glm::vec3(0.f, 0.f, 1.f);

	// Every length in the avatar's units: the user's head geometry is scaled
	// by the ratio of the avatar's shoulder width to the width it was
	// measured against
	const float avatarShoulderWidth= skeleton.getShoulderWidth();
	const float lengthScale= (config.userShoulderWidthMeters > 1e-3f && avatarShoulderWidth > 1e-3f)
		? avatarShoulderWidth / config.userShoulderWidthMeters
		: 1.f;
	const float headWidth= config.headWidthMeters * lengthScale;
	const float noseForward= config.noseForwardMeters * lengthScale;
	const float torsoLength= std::max(
		skeleton.getRestShoulderMidpointWorld().z - skeleton.getBone(B::Hips).restPositionWorld.z, 0.05f);

	const BodyPoseObservation& body= camera.body;
	const bool bFrameKnown= cameraModel.width > 0 && cameraModel.height > 0;
	auto usable= [&](int index) {
		if (!body.valid || !body.isProvided(index) || body.visibility[index] < kMinVisibility)
			return false;
		if (!bFrameKnown)
			return true;
		const glm::vec3& point= body.imagePoints[index];
		return point.x >= kFrameMarginPx && point.y >= kFrameMarginPx &&
			point.x <= (float)cameraModel.width - kFrameMarginPx && point.y <= (float)cameraModel.height - kFrameMarginPx;
	};
	auto rayThrough= [&](float u, float v) {
		return worldFromCam *
			glm::normalize(glm::vec3((u - cameraModel.cx) / cameraModel.fx, (v - cameraModel.cy) / cameraModel.fy, 1.f));
	};
	auto landmarkRay= [&](int index) { return rayThrough(body.imagePoints[index].x, body.imagePoints[index].y); };
	// +1 when the model puts `to` farther from the camera than `from`, -1
	// nearer, 0 undecided
	auto depthSignBetween= [&](float fromZ, float toZ) -> int {
		if (!body.hasDepth)
			return 0;
		const float delta= toZ - fromZ;
		if (std::fabs(delta) < kDepthSignMinMeters)
			return 0;
		return delta > 0.f ? 1 : -1;
	};
	auto depthSign= [&](int fromIndex, int toIndex) {
		return depthSignBetween(body.imagePoints[fromIndex].z, body.imagePoints[toIndex].z);
	};
	auto depthInRange= [](float depth) { return depth >= kMinBodyDepthMeters && depth <= kMaxBodyDepthMeters; };

	const int leftShoulderIndex= (int)ePoseLandmark::LEFT_SHOULDER;
	const int rightShoulderIndex= (int)ePoseLandmark::RIGHT_SHOULDER;
	const int leftEarIndex= (int)ePoseLandmark::LEFT_EAR;
	const int rightEarIndex= (int)ePoseLandmark::RIGHT_EAR;
	const int noseIndex= (int)ePoseLandmark::NOSE;

	// -- Torso: both shoulders at the depth a known width implies --
	bool bTorsoFresh= false;
	if (usable(leftShoulderIndex) && usable(rightShoulderIndex))
	{
		const glm::vec3 leftRay= landmarkRay(leftShoulderIndex);
		const glm::vec3 rightRay= landmarkRay(rightShoulderIndex);

		// The depth at which the rays sit a shoulder width apart. The torso
		// reads as square to the camera: the model's relative depth is too
		// compressed to measure a yaw, and a head-anchored depth moves with a
		// lean. The line still carries a raised shoulder as roll.
		const float depth= BodyPoseSolver::depthFromKnownSeparation(leftRay, rightRay, avatarShoulderWidth);
		if (depthInRange(depth))
		{
			const glm::vec3 left= leftRay * depth;
			const glm::vec3 right= rightRay * depth;

			m_torso.shoulder[0]= m_torso.shoulderFilter[0].filter(left, dtSeconds);
			m_torso.shoulder[1]= m_torso.shoulderFilter[1].filter(right, dtSeconds);
			m_torso.midpoint= 0.5f * (m_torso.shoulder[0] + m_torso.shoulder[1]);
			const glm::vec3 leftDirection= safeNormalize(m_torso.shoulder[0] - m_torso.shoulder[1]);
			m_torso.lineFrame= glm::dot(leftDirection, leftDirection) > 0.f ? glm_shortest_arc(k_left, leftDirection)
																			  : glm::quat(1.f, 0.f, 0.f, 0.f);
			m_torso.valid= true;
			m_torso.solvedTimestampMs= timestampMs;
			bTorsoFresh= true;

			// The torso's up direction for the tilt estimate, from the hips
			// one torso length below the shoulder midpoint on their ray
			m_torso.bHasUpAxis= false;
			const int leftHipIndex= (int)ePoseLandmark::LEFT_HIP;
			const int rightHipIndex= (int)ePoseLandmark::RIGHT_HIP;
			if (usable(leftHipIndex) && usable(rightHipIndex))
			{
				const glm::vec3 hipPx= 0.5f * (body.imagePoints[leftHipIndex] + body.imagePoints[rightHipIndex]);
				const float hipZ= 0.5f * (body.imagePoints[leftHipIndex].z + body.imagePoints[rightHipIndex].z);
				const float shoulderZ=
					0.5f * (body.imagePoints[leftShoulderIndex].z + body.imagePoints[rightShoulderIndex].z);
				RaySphereHit hit;
				if (BodyPoseSolver::intersectRaySphere(k_origin, rayThrough(hipPx.x, hipPx.y), m_torso.midpoint,
													   torsoLength, hit))
				{
					// Undecided, the hips sit farther: a camera above the
					// screen looks down on the torso
					const int hipSign= depthSignBetween(shoulderZ, hipZ);
					const glm::vec3 hips= pickCrossing(hit, cameraForward, m_torso.midpoint, hipSign == 0 ? 1 : hipSign);
					const glm::vec3 up= safeNormalize(m_torso.midpoint - hips);
					if (glm::dot(up, up) > 0.f)
					{
						m_torso.upAxis= up;
						m_torso.bHasUpAxis= true;
					}
				}
			}
		}
	}
	const bool bTorsoUsable= m_torso.valid && (timestampMs - m_torso.solvedTimestampMs) <= kMaxHoldMs;
	if (!bTorsoUsable)
	{
		m_torso.valid= false;
		for (OneEuroFilterVec3& filter : m_torso.shoulderFilter)
			filter.reset();
	}

	// -- Spine: the torso frame against the neutral one --
	glm::quat spineDelta(1.f, 0.f, 0.f, 0.f);
	if (bTorsoUsable && config.neutral.present)
	{
		// The lean: the shoulder midpoint seen from a pivot one torso length
		// below where it rested, so shifting the shoulders reads as bending
		// at the hips
		const glm::vec3 pivot= tilt * config.neutral.shoulderMidpoint - k_up * torsoLength;
		const glm::vec3 leanDirection= safeNormalize(m_torso.midpoint - pivot);
		const glm::quat lean= glm::dot(leanDirection, leanDirection) > 0.f ? glm_shortest_arc(k_up, leanDirection)
																		   : glm::quat(1.f, 0.f, 0.f, 0.f);
		const glm::quat torsoNow= lean * m_torso.lineFrame;
		const glm::quat neutralTorso= tilt * config.neutral.torso;
		spineDelta= limitRotationAngle(torsoNow * glm::inverse(neutralTorso), kSpineLimitRadians);
	}
	placeBone(outPose, skeleton, B::Spine, spineDelta);
	placeBone(outPose, skeleton, B::Chest, spineDelta);
	placeBone(outPose, skeleton, B::UpperChest, spineDelta);
	placeBone(outPose, skeleton, B::Neck, spineDelta);

	// -- Head: the two ear rays plus the known head width, the nose a known
	// distance forward of their midpoint --
	if (usable(noseIndex) && usable(leftEarIndex) && usable(rightEarIndex))
	{
		const glm::vec3 leftRay= landmarkRay(leftEarIndex);
		const glm::vec3 rightRay= landmarkRay(rightEarIndex);
		const float depth= BodyPoseSolver::depthFromKnownSeparation(leftRay, rightRay, headWidth);
		RaySphereHit noseHit;
		if (depthInRange(depth))
		{
			const glm::vec3 leftEar= leftRay * depth;
			const glm::vec3 rightEar= rightRay * depth;
			const glm::vec3 earMid= 0.5f * (leftEar + rightEar);
			// The near crossing: a face points toward the camera that sees it
			if (BodyPoseSolver::intersectRaySphere(k_origin, landmarkRay(noseIndex), earMid, noseForward, noseHit) &&
				noseHit.bNearValid)
			{
				const glm::vec3 leftRaw= safeNormalize(leftEar - rightEar);
				const glm::vec3 forwardRaw= safeNormalize(noseHit.nearPoint - earMid);
				if (glm::dot(leftRaw, leftRaw) > 0.f && glm::dot(forwardRaw, forwardRaw) > 0.f)
				{
					const glm::vec3 left= m_head.leftFilter.filter(leftRaw, dtSeconds);
					const glm::vec3 forward= m_head.forwardFilter.filter(forwardRaw, dtSeconds);
					glm::quat frame;
					if (frameFromForwardAndLeft(forward, left, frame))
					{
						m_head.frame= frame;
						m_head.bMeasured= true;
						m_head.solvedTimestampMs= timestampMs;
					}
				}
			}
		}
	}
	const bool bHeadUsable= m_head.bMeasured && (timestampMs - m_head.solvedTimestampMs) <= kMaxHoldMs;
	if (!bHeadUsable)
	{
		m_head.bMeasured= false;
		m_head.leftFilter.reset();
		m_head.forwardFilter.reset();
	}
	{
		glm::quat headDelta(1.f, 0.f, 0.f, 0.f);
		if (phoneHead.valid)
		{
			// Already canonical: the face anchor captured at the neutral
			// pose makes looking at the screen the identity
			headDelta= limitRotationAngle(phoneHead.orientationWorld, kHeadLimitRadians);
		}
		else if (bHeadUsable)
		{
			const glm::quat relative=
				config.neutral.present ? m_head.frame * glm::inverse(tilt * config.neutral.head) : m_head.frame;
			headDelta= limitRotationAngle(relative, kHeadLimitRadians);
		}
		headDelta= headDelta * glm::inverse(glm::quat(config.headTrimRadians));
		placeBone(outPose, skeleton, B::Head, headDelta);
	}

	// -- Arms --
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		ArmState& arm= m_arms[sideIndex];
		const eHandSide side= (eHandSide)sideIndex;
		const AvatarRetargetConfig::Side& sideConfig= config.sides[sideIndex];
		const TrackedHand& hand= camera.hands[sideIndex];
		const HandPose& pose= camera.poses[sideIndex];
		const int shoulderIndex= sideIndex == 0 ? leftShoulderIndex : rightShoulderIndex;
		const int elbowIndex= sideIndex == 0 ? (int)ePoseLandmark::LEFT_ELBOW : (int)ePoseLandmark::RIGHT_ELBOW;
		const int wristIndex= sideIndex == 0 ? (int)ePoseLandmark::LEFT_WRIST : (int)ePoseLandmark::RIGHT_WRIST;
		const B upperArmBone= sideBone(sideIndex, B::LeftUpperArm, B::RightUpperArm);
		const B lowerArmBone= sideBone(sideIndex, B::LeftLowerArm, B::RightLowerArm);
		const B handBone= sideBone(sideIndex, B::LeftHand, B::RightHand);
		const float upperArm= skeleton.getUpperArmLength(side);
		const float forearm= skeleton.getForearmLength(side);
		const float reach= upperArm + forearm;

		// The wrist pixel: the hand model's wrist when it tracks the hand,
		// else the body model's, which keeps the arm placed through a hand
		// dropout
		bool bWristPixel= false;
		glm::vec2 wristPx(0.f);
		if (hand.tracked)
		{
			wristPx= glm::vec2(hand.imagePoints[0].x, hand.imagePoints[0].y);
			bWristPixel= true;
		}
		else if (usable(wristIndex))
		{
			wristPx= glm::vec2(body.imagePoints[wristIndex].x, body.imagePoints[wristIndex].y);
			bWristPixel= true;
		}

		const bool bHadChain= arm.chain.valid && (timestampMs - arm.solvedTimestampMs) <= kMaxHoldMs;
		if (bTorsoFresh && bWristPixel && upperArm > 1e-3f && forearm > 1e-3f)
		{
			const glm::vec3 shoulder= m_torso.shoulder[sideIndex];
			const glm::vec3 wristRay= rayThrough(wristPx.x, wristPx.y);
			bool bSolved= false;
			glm::vec3 elbow(0.f);
			glm::vec3 wrist(0.f);
			if (usable(elbowIndex))
			{
				// Each segment where its ray meets the sphere of its length
				// about the joint before it, the model's depth picking the
				// crossing
				RaySphereHit elbowHit;
				RaySphereHit wristHit;
				if (BodyPoseSolver::intersectRaySphere(k_origin, landmarkRay(elbowIndex), shoulder, upperArm, elbowHit))
				{
					elbow= pickCrossing(elbowHit, cameraForward, shoulder, depthSign(shoulderIndex, elbowIndex));
					if (BodyPoseSolver::intersectRaySphere(k_origin, wristRay, elbow, forearm, wristHit))
					{
						wrist= pickCrossing(wristHit, cameraForward, elbow, depthSign(elbowIndex, wristIndex));
						bSolved= true;
					}
				}
			}
			else
			{
				// The elbow is not seen (out of frame, where the model
				// invents one): the wrist keeps its last distance along its
				// ray within reach, and the elbow stays on the bone circle
				// nearest where it was, or the rig's hint before any was seen
				const float wristDistance= bHadChain ? glm::length(arm.chain.wrist) : glm::length(shoulder);
				wrist= wristRay * wristDistance;
				const float maxReach= reach * 0.999f;
				if (glm::length(wrist - shoulder) > maxReach)
				{
					// Beyond reach: back along the ray to where it crosses
					// the reach sphere, the crossing nearer the distance it had
					RaySphereHit reachHit;
					if (BodyPoseSolver::intersectRaySphere(k_origin, wristRay, shoulder, maxReach, reachHit))
					{
						const float nearError= std::fabs(glm::length(reachHit.nearPoint) - wristDistance);
						const float farError= std::fabs(glm::length(reachHit.farPoint) - wristDistance);
						wrist= (reachHit.bNearValid && (!reachHit.bFarValid || nearError <= farError))
							? reachHit.nearPoint
							: reachHit.farPoint;
					}
					else
					{
						wrist= shoulder + safeNormalize(wrist - shoulder) * maxReach;
					}
				}
				const glm::vec3 target= bHadChain ? arm.chain.elbow : shoulder + sideConfig.elbowHintOffset;
				if (!BodyPoseSolver::solveElbowNearestTo(shoulder, wrist, upperArm, forearm, target, elbow))
				{
					const glm::vec3 axis= safeNormalize(wrist - shoulder);
					elbow= shoulder + (glm::dot(axis, axis) > 0.f ? axis : -k_up) * upperArm;
				}
				bSolved= true;
			}

			if (bSolved)
			{
				arm.chain.shoulder= shoulder;
				arm.chain.elbow= arm.elbowFilter.filter(elbow, dtSeconds);
				arm.chain.wrist= arm.wristFilter.filter(wrist, dtSeconds);
				arm.chain.valid= true;
				arm.solvedTimestampMs= timestampMs;
			}
		}

		const bool bChainUsable= arm.chain.valid && (timestampMs - arm.solvedTimestampMs) <= kMaxHoldMs;
		if (!bChainUsable)
		{
			arm.chain.valid= false;
			arm.elbowFilter.reset();
			arm.wristFilter.reset();
			restArm(outPose, skeleton, sideIndex);
			continue;
		}

		// Upper arm and forearm: swings from rest onto the chain's directions
		const glm::vec3 upperDirection= safeNormalize(arm.chain.elbow - arm.chain.shoulder);
		glm::vec3 forearmDirection= safeNormalize(arm.chain.wrist - arm.chain.elbow);
		if (glm::dot(forearmDirection, forearmDirection) <= 0.f)
			forearmDirection= upperDirection;
		glm::quat chainDelta= spineDelta;
		if (glm::dot(upperDirection, upperDirection) > 0.f)
			chainDelta= swingTo(chainDelta, restDirection(skeleton, lowerArmBone), upperDirection);
		placeBone(outPose, skeleton, upperArmBone, chainDelta);
		if (glm::dot(forearmDirection, forearmDirection) > 0.f)
			chainDelta= swingTo(chainDelta, restDirection(skeleton, handBone), forearmDirection);
		else
			forearmDirection= chainDelta * restDirection(skeleton, handBone);

		// The hand: the palm frame in the world, its normal's roll about the
		// forearm against the avatar's rest palm, and the palm in the
		// measured forearm frame (the wrist). Held through a hand dropout.
		const AvatarSkeleton::HandRest& handRest= skeleton.getHand(side);
		const glm::vec3 restPalmNormal=
			safeNormalize(projectPerpendicular(chainDelta * glm::vec3(handRest.palmFrameWorld[2]), forearmDirection));
		if (pose.tracked && pose.hasCameraPose)
		{
			const glm::quat palmRaw= glm::normalize(worldFromCam * pose.palmOrientationCamera);
			glm::vec3 palmXRaw= palmRaw * glm::vec3(1.f, 0.f, 0.f);
			glm::vec3 palmZRaw= palmRaw * glm::vec3(0.f, 0.f, 1.f);
			// A single view cannot tell a hand pointing at the camera from one
			// pointing away, and on the wrong guess the lift's palm comes back
			// mirrored along the view ray: the finger axis reflected, the
			// normal reflected and flipped to keep the frame right-handed. The
			// forearm decides: the mirror whose finger axis continues it is
			// the hand.
			const glm::vec3 viewRay= safeNormalize(arm.chain.wrist);
			if (glm::dot(viewRay, viewRay) > 0.f)
			{
				const glm::vec3 mirroredX= palmXRaw - 2.f * glm::dot(palmXRaw, viewRay) * viewRay;
				const glm::vec3 mirroredZ= -(palmZRaw - 2.f * glm::dot(palmZRaw, viewRay) * viewRay);
				if (glm::dot(mirroredX, forearmDirection) > glm::dot(palmXRaw, forearmDirection))
				{
					palmXRaw= mirroredX;
					palmZRaw= mirroredZ;
				}
			}
			const glm::vec3 palmXFiltered= arm.palmXFilter.filter(palmXRaw, dtSeconds);
			const glm::vec3 palmZFiltered= arm.palmZFilter.filter(palmZRaw, dtSeconds);
			glm::vec3 palmX= safeNormalize(palmXFiltered);
			if (glm::dot(palmX, palmX) <= 0.f)
				palmX= palmXRaw;
			glm::vec3 palmZ= safeNormalize(projectPerpendicular(palmZFiltered, palmX));
			if (glm::dot(palmZ, palmZ) <= 0.f)
				palmZ= safeNormalize(projectPerpendicular(palmZRaw, palmX));
			if (glm::dot(palmZ, palmZ) > 0.f)
			{
				const glm::vec3 palmY= glm::cross(palmZ, palmX);
				const glm::quat palm= glm::quat_cast(glm::mat3(palmX, palmY, palmZ));

				// The measured forearm frame: +X along the forearm, +Z the
				// palm normal projected clear of it (the palm's +Y when the
				// normal lies along the forearm)
				glm::vec3 forearmZ= safeNormalize(projectPerpendicular(palmZ, forearmDirection));
				if (glm::dot(forearmZ, forearmZ) <= 0.f)
					forearmZ= safeNormalize(projectPerpendicular(palmY, forearmDirection));
				if (glm::dot(forearmZ, forearmZ) > 0.f)
				{
					if (glm::dot(restPalmNormal, restPalmNormal) > 0.f)
						arm.twist= signedAngle(restPalmNormal, forearmZ, forearmDirection);
					const glm::vec3 forearmY= glm::cross(forearmZ, forearmDirection);
					const glm::quat forearmFrame= glm::quat_cast(glm::mat3(forearmDirection, forearmY, forearmZ));
					arm.wristRotation= glm::inverse(forearmFrame) * palm;
					arm.bHandMeasured= true;
				}
			}

			// Fingers: the raw angles less the rest zero, filtered
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				FingerAngles angles= pose.fingers[finger];
				if (config.bHasRestAngles[sideIndex])
				{
					const FingerAngles& rest= config.restAngles[sideIndex][finger];
					angles.lateral-= rest.lateral;
					angles.proximal-= rest.proximal;
					angles.intermediate-= rest.intermediate;
					angles.distal-= rest.distal;
				}
				OneEuroFilter* filters= &arm.fingerFilters[finger * 4];
				arm.fingers[finger].lateral= filters[0].filter(angles.lateral, dtSeconds);
				arm.fingers[finger].proximal= filters[1].filter(angles.proximal, dtSeconds);
				arm.fingers[finger].intermediate= filters[2].filter(angles.intermediate, dtSeconds);
				arm.fingers[finger].distal= filters[3].filter(angles.distal, dtSeconds);
			}
		}
		else
		{
			// The held values stay; the filters restart on reacquire rather
			// than dragging across the gap
			arm.palmXFilter.reset();
			arm.palmZFilter.reset();
			for (OneEuroFilter& filter : arm.fingerFilters)
				filter.reset();
		}

		// Forearm roll: the measured twist less the neutral one, plus the
		// rig's trim; an arm whose hand was never seen rolls only by the trim
		float twist= sideConfig.forearmRollTrimRadians;
		if (arm.bHandMeasured)
			twist+= wrapAngle(arm.twist - (config.neutral.present ? config.neutral.forearmTwist[sideIndex] : 0.f));
		chainDelta= glm::angleAxis(twist, forearmDirection) * chainDelta;
		placeBone(outPose, skeleton, lowerArmBone, chainDelta);

		// The hand bone: the wrist's rotation away from neutral, applied in
		// the avatar's trimmed rest palm frame under the posed forearm, so
		// the neutral wrist is the avatar's rest wrist
		glm::quat relative(1.f, 0.f, 0.f, 0.f);
		if (arm.bHandMeasured)
		{
			relative= config.neutral.present ? glm::inverse(config.neutral.wrist[sideIndex]) * arm.wristRotation
											 : arm.wristRotation;
			relative= limitRotationAngle(relative, kWristLimitRadians);
		}
		const glm::quat restLowerArm= skeleton.getBone(lowerArmBone).restRotationWorld;
		const glm::quat trimmedRestPalm=
			glm::quat_cast(glm::mat3(handRest.palmFrameWorld)) * glm::quat(sideConfig.handTrimRadians);
		const glm::quat posedForearm= chainDelta * restLowerArm;
		const glm::quat posedPalm= posedForearm * (glm::inverse(restLowerArm) * trimmedRestPalm) * relative;
		const glm::quat handDelta= posedPalm * glm::inverse(trimmedRestPalm);
		placeBone(outPose, skeleton, handBone, handDelta);

		solveFingerBones(skeleton, sideIndex, glm::quat(1.f, 0.f, 0.f, 0.f), handDelta,
						 skeleton.getBone(handBone).restPositionWorld, arm.fingers, sideConfig, outPose);
	}
}

bool DirectRetarget::captureNeutral(DirectNeutralCapture& outCapture) const
{
	outCapture= DirectNeutralCapture();
	if (!m_torso.valid)
		return false;

	// Back to the level frame, so the references survive a tilt edit
	const glm::quat levelFromTilted= glm::inverse(worldFromCamera(m_tiltRadians) * glm::inverse(worldFromCamera(0.f)));
	DirectNeutralPose& neutral= outCapture.neutral;
	neutral.present= true;
	neutral.torso= levelFromTilted * m_torso.lineFrame;
	neutral.shoulderMidpoint= levelFromTilted * m_torso.midpoint;
	neutral.head= m_head.bMeasured ? levelFromTilted * m_head.frame : glm::quat(1.f, 0.f, 0.f, 0.f);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const ArmState& arm= m_arms[sideIndex];
		neutral.wrist[sideIndex]= arm.bHandMeasured ? arm.wristRotation : glm::quat(1.f, 0.f, 0.f, 0.f);
		neutral.forearmTwist[sideIndex]= arm.bHandMeasured ? arm.twist : 0.f;
	}

	if (m_torso.bHasUpAxis)
	{
		// An upright torso measured as leaning forward or back by some angle
		// means the tilt is off by that angle
		outCapture.bTiltMeasured= true;
		outCapture.cameraTiltRadians= m_tiltRadians + atan2f(m_torso.upAxis.x, m_torso.upAxis.z);
	}
	return true;
}
