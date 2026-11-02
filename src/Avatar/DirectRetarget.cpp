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
// A hand slot farther than this from the body model's wrist, in shoulder
// widths, is not the hand: the palm detector runs over the whole frame and
// fires on hand-shaped things in the scene
constexpr float kHandAgreementShoulderWidths= 0.5f;
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

// One-euro bands. Pixels are accurate and fast, so their band is light.
// Depths are the single view's guess and the quantity that jumps, so their
// band is heavy. The palm axes keep the body solver's direction band.
constexpr float kPixelMinCutoffHz= 2.5f;
constexpr float kPixelBeta= 0.1f;
constexpr float kDepthMinCutoffHz= 0.6f;
constexpr float kDepthBeta= 0.02f;
constexpr float kDirectionMinCutoffHz= 1.5f;
constexpr float kDirectionBeta= 0.08f;
constexpr float kDerivativeCutoffHz= 1.0f;

// The wrist-depth scorer: candidates evenly across the reach the ray
// allows, the best refined by halving the spacing around it
constexpr int kDepthCandidates= 24;
constexpr int kDepthRefineSteps= 4;
// The cost terms, each a squared ratio to its sigma. The elbow's reprojection
// against its ray in pixels, the dominant term while the elbow is seen; the
// same sigma sets the elbow's place around its bone circle against the ray.
constexpr float kElbowPixelSigma= 20.f;
// The distance a joint may move per second (the wrist along its ray, the
// elbow from where it was), in reach, and the floor on the per-frame sigma
// so a long frame does not void the term
constexpr float kMaxJointSpeedReachPerS= 2.5f;
constexpr float kMinMotionSigmaReach= 0.02f;
// The model's shoulder-to-wrist relative depth arrives compressed by this
// (the gate tool's "scale p50" on the webcam take); divided out, it is a
// wide prior on the candidate's own shoulder-to-wrist depth
constexpr float kModelDepthScale= 0.6f;
constexpr float kModelDepthSigmaReach= 0.3f;
// Before any chain, with the elbow unseen, the candidate whose elbow stays
// nearest the rig's hint wins ties: the squared distance over the reach,
// times this
constexpr float kElbowDriftPenalty= 0.5f;
// The hand lift's depth against the chain's is measured, not yet scored:
// the scale stays within these multiples of the avatar-to-user length
// ratio, and is sampled only while the arm is bent enough that the elbow's
// reprojection fixed the wrist's distance
constexpr float kPnpScaleMin= 0.5f;
constexpr float kPnpScaleMax= 2.0f;
constexpr float kPnpSampleMaxReach= 0.98f;
// A raw palm this far from the filtered one is held out as the lift's
// mirror until it persists this many consecutive frames
constexpr float kPalmJumpRadians= 60.f * glm::pi<float>() / 180.f;
constexpr int kPalmJumpAcceptFrames= 2;

const glm::vec3 k_up(0.f, 0.f, 1.f);
const glm::vec3 k_left(0.f, 1.f, 0.f);
const glm::vec3 k_origin(0.f);

glm::vec3 restDirection(const AvatarSkeleton& skeleton, B bone)
{
	return safeNormalize(skeleton.getBone(bone).restOffsetFromParentWorld);
}

float rotationAngle(const glm::quat& rotation)
{
	const glm::quat unit= glm::normalize(rotation);
	return 2.f * acosf(std::min(std::fabs(unit.w), 1.f));
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

// A rotation from a forward and a left that may not be exactly
// perpendicular: forward survives, up is rebuilt from it and left, left from
// the two
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

// A palm frame from its finger axis and a normal that may not be exactly
// perpendicular to it
glm::quat palmFrame(const glm::vec3& palmX, const glm::vec3& palmZ)
{
	const glm::vec3 palmY= glm::cross(palmZ, palmX);
	return glm::quat_cast(glm::mat3(palmX, palmY, palmZ));
}

// The elbow for a wrist, both bone lengths held by construction: on the bone
// circle against the elbow ray and near the target while the elbow is seen,
// nearest the target while it is not, straight toward the wrist when the
// circle fails
glm::vec3 solveElbow(const glm::vec3& shoulder, const glm::vec3& wrist, float upperArm, float forearm, bool bElbowSeen,
					 const glm::vec3& elbowRay, float raySigma, const glm::vec3& target, float targetSigma)
{
	glm::vec3 elbow;
	const bool bSolved= bElbowSeen
		? BodyPoseSolver::solveElbowOnBoneCircleNear(shoulder, wrist, upperArm, forearm, k_origin, elbowRay, raySigma,
													target, targetSigma, elbow)
		: BodyPoseSolver::solveElbowNearestTo(shoulder, wrist, upperArm, forearm, target, elbow);
	if (bSolved)
		return elbow;
	const glm::vec3 axis= safeNormalize(wrist - shoulder);
	return shoulder + (glm::dot(axis, axis) > 0.f ? axis : -k_up) * upperArm;
}

// The median of the filled samples, a sorted copy so the ring stays in
// insertion order
float medianOf(const float* samples, int count)
{
	std::array<float, DirectRetarget::kPnpScaleSamples> sorted;
	std::copy(samples, samples + count, sorted.begin());
	std::sort(sorted.begin(), sorted.begin() + count);
	return (count % 2) == 1 ? sorted[count / 2] : 0.5f * (sorted[count / 2 - 1] + sorted[count / 2]);
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

void DirectRetarget::measureMonoFingers(const TrackedHand& hand, HandPoseModel::PalmarSideMemory& ioMemory,
										std::array<FingerAngles, FINGER_COUNT>& outAngles)
{
	HandPoseModel::computeMetacarpalFingerAngles(hand.modelPoints, hand.side, outAngles, &ioMemory);
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		const float* bias= finger == (int)eFinger::Thumb ? kThumbOpenBias : kFingerOpenBias;
		FingerAngles& angles= outAngles[finger];
		angles.proximal= std::max(angles.proximal - bias[0], kMinBendRadians);
		angles.intermediate= std::max(angles.intermediate - bias[1], kMinBendRadians);
		angles.distal= std::max(angles.distal - bias[2], kMinBendRadians);
	}
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
	for (OneEuroFilterVec2& filter : m_torso.shoulderPixelFilter)
		filter.configure(kPixelMinCutoffHz, kPixelBeta, kDerivativeCutoffHz);
	m_torso.depthFilter.configure(kDepthMinCutoffHz, kDepthBeta, kDerivativeCutoffHz);
	m_head.nosePixelFilter.configure(kPixelMinCutoffHz, kPixelBeta, kDerivativeCutoffHz);
	for (OneEuroFilterVec2& filter : m_head.earPixelFilter)
		filter.configure(kPixelMinCutoffHz, kPixelBeta, kDerivativeCutoffHz);
	m_head.depthFilter.configure(kDepthMinCutoffHz, kDepthBeta, kDerivativeCutoffHz);
	for (ArmState& arm : m_arms)
	{
		arm.elbowPixelFilter.configure(kPixelMinCutoffHz, kPixelBeta, kDerivativeCutoffHz);
		arm.wristPixelFilter.configure(kPixelMinCutoffHz, kPixelBeta, kDerivativeCutoffHz);
		arm.depthFilter.configure(kDepthMinCutoffHz, kDepthBeta, kDerivativeCutoffHz);
		arm.palmXFilter.configure(kDirectionMinCutoffHz, kDirectionBeta, kDerivativeCutoffHz);
		arm.palmZFilter.configure(kDirectionMinCutoffHz, kDirectionBeta, kDerivativeCutoffHz);
		for (OneEuroFilter& filter : arm.fingerFilters)
			filter.configure(kDirectionMinCutoffHz, kDirectionBeta, kDerivativeCutoffHz);
	}
	m_bFiltersConfigured= true;
}

DirectRetarget::PnpScale DirectRetarget::getPnpScale(int sideIndex) const
{
	PnpScale out;
	out.scale= m_arms[sideIndex].pnpScale;
	out.sampleCount= m_arms[sideIndex].pnpScaleSampleCount;
	return out;
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
	auto rayThrough= [&](const glm::vec2& px) {
		return worldFromCam *
			glm::normalize(glm::vec3((px.x - cameraModel.cx) / cameraModel.fx, (px.y - cameraModel.cy) / cameraModel.fy, 1.f));
	};
	auto landmarkPx= [&](int index) { return glm::vec2(body.imagePoints[index].x, body.imagePoints[index].y); };
	auto depthInRange= [](float depth) { return depth >= kMinBodyDepthMeters && depth <= kMaxBodyDepthMeters; };
	// A world point's angular miss from a ray through the camera, in pixels
	// at the focal length
	auto pixelsOffRay= [&](const glm::vec3& point, const glm::vec3& ray) {
		const glm::vec3 direction= safeNormalize(point);
		if (glm::dot(direction, direction) <= 0.f)
			return 0.f;
		return acosf(std::clamp(glm::dot(direction, ray), -1.f, 1.f)) * cameraModel.fx;
	};

	const int leftShoulderIndex= (int)ePoseLandmark::LEFT_SHOULDER;
	const int rightShoulderIndex= (int)ePoseLandmark::RIGHT_SHOULDER;
	const int leftEarIndex= (int)ePoseLandmark::LEFT_EAR;
	const int rightEarIndex= (int)ePoseLandmark::RIGHT_EAR;
	const int noseIndex= (int)ePoseLandmark::NOSE;

	// -- Torso: both shoulders at the depth a known width implies, the
	// pixels filtered lightly and the depth heavily --
	bool bTorsoFresh= false;
	float shoulderWidthPx= 0.f;
	if (usable(leftShoulderIndex) && usable(rightShoulderIndex))
	{
		const glm::vec2 leftPx= m_torso.shoulderPixelFilter[0].filter(landmarkPx(leftShoulderIndex), dtSeconds);
		const glm::vec2 rightPx= m_torso.shoulderPixelFilter[1].filter(landmarkPx(rightShoulderIndex), dtSeconds);
		const glm::vec3 leftRay= rayThrough(leftPx);
		const glm::vec3 rightRay= rayThrough(rightPx);
		shoulderWidthPx= glm::distance(landmarkPx(leftShoulderIndex), landmarkPx(rightShoulderIndex));

		// The depth at which the rays sit a shoulder width apart. The torso
		// reads as square to the camera: the model's relative depth is too
		// compressed to measure a yaw, and a head-anchored depth moves with a
		// lean. The line still carries a raised shoulder as roll.
		const float depthRaw= BodyPoseSolver::depthFromKnownSeparation(leftRay, rightRay, avatarShoulderWidth);
		if (depthInRange(depthRaw))
		{
			const float depth= m_torso.depthFilter.filter(depthRaw, dtSeconds);
			m_torso.shoulder[0]= leftRay * depth;
			m_torso.shoulder[1]= rightRay * depth;
			m_torso.midpoint= 0.5f * (m_torso.shoulder[0] + m_torso.shoulder[1]);
			const glm::vec3 leftDirection= safeNormalize(m_torso.shoulder[0] - m_torso.shoulder[1]);
			m_torso.lineFrame= glm::dot(leftDirection, leftDirection) > 0.f ? glm_shortest_arc(k_left, leftDirection)
																			  : glm::quat(1.f, 0.f, 0.f, 0.f);
			m_torso.valid= true;
			m_torso.solvedTimestampMs= timestampMs;
			bTorsoFresh= true;
		}
	}
	const bool bTorsoUsable= m_torso.valid && (timestampMs - m_torso.solvedTimestampMs) <= kMaxHoldMs;
	if (!bTorsoUsable)
	{
		m_torso.valid= false;
		for (OneEuroFilterVec2& filter : m_torso.shoulderPixelFilter)
			filter.reset();
		m_torso.depthFilter.reset();
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
	// distance forward of their midpoint, the same filtering as the torso --
	if (usable(noseIndex) && usable(leftEarIndex) && usable(rightEarIndex))
	{
		const glm::vec2 nosePx= m_head.nosePixelFilter.filter(landmarkPx(noseIndex), dtSeconds);
		const glm::vec2 leftEarPx= m_head.earPixelFilter[0].filter(landmarkPx(leftEarIndex), dtSeconds);
		const glm::vec2 rightEarPx= m_head.earPixelFilter[1].filter(landmarkPx(rightEarIndex), dtSeconds);
		const glm::vec3 leftRay= rayThrough(leftEarPx);
		const glm::vec3 rightRay= rayThrough(rightEarPx);
		const float depthRaw= BodyPoseSolver::depthFromKnownSeparation(leftRay, rightRay, headWidth);
		RaySphereHit noseHit;
		if (depthInRange(depthRaw))
		{
			const float depth= m_head.depthFilter.filter(depthRaw, dtSeconds);
			const glm::vec3 leftEar= leftRay * depth;
			const glm::vec3 rightEar= rightRay * depth;
			const glm::vec3 earMid= 0.5f * (leftEar + rightEar);
			// The near crossing: a face points toward the camera that sees it
			if (BodyPoseSolver::intersectRaySphere(k_origin, rayThrough(nosePx), earMid, noseForward, noseHit) &&
				noseHit.bNearValid)
			{
				const glm::vec3 left= safeNormalize(leftEar - rightEar);
				const glm::vec3 forward= safeNormalize(noseHit.nearPoint - earMid);
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
	const bool bHeadUsable= m_head.bMeasured && (timestampMs - m_head.solvedTimestampMs) <= kMaxHoldMs;
	if (!bHeadUsable)
	{
		m_head.bMeasured= false;
		m_head.nosePixelFilter.reset();
		for (OneEuroFilterVec2& filter : m_head.earPixelFilter)
			filter.reset();
		m_head.depthFilter.reset();
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
		// Before any sample the hand lift's depth is trusted at the length
		// ratio, the scale a correct lift would measure
		if (arm.pnpScaleSampleCount == 0)
			arm.pnpScale= lengthScale;

		// The hand slot is the hand only where the body model agrees: the
		// palm detector runs over the whole frame and fires on hand-shaped
		// things in the scene, while the body model places the wrist by the
		// whole figure and reports one it cannot see as unseen. Agreement is
		// measured in shoulder widths, so it scales with the user's distance.
		const glm::vec2 handWristPx(hand.imagePoints[0].x, hand.imagePoints[0].y);
		const glm::vec2 bodyWristPx= landmarkPx(wristIndex);
		const bool bHandSeen= hand.tracked && usable(wristIndex) && shoulderWidthPx > 0.f &&
			glm::distance(handWristPx, bodyWristPx) <= kHandAgreementShoulderWidths * shoulderWidthPx;

		// The wrist pixel: the hand model's wrist while the hand is seen, else
		// the body model's, which keeps the arm placed through a hand dropout;
		// either through the same light filter. The elbow pixel likewise.
		bool bWristPixel= false;
		glm::vec2 wristPx(0.f);
		if (bHandSeen || usable(wristIndex))
		{
			wristPx= arm.wristPixelFilter.filter(bHandSeen ? handWristPx : bodyWristPx, dtSeconds);
			bWristPixel= true;
		}
		else
		{
			arm.wristPixelFilter.reset();
		}
		const bool bElbowSeen= usable(elbowIndex);
		glm::vec2 elbowPx(0.f);
		if (bElbowSeen)
			elbowPx= arm.elbowPixelFilter.filter(landmarkPx(elbowIndex), dtSeconds);
		else
			arm.elbowPixelFilter.reset();

		const bool bHadChain= arm.chain.valid && (timestampMs - arm.solvedTimestampMs) <= kMaxHoldMs;
		if (bTorsoFresh && bWristPixel && upperArm > 1e-3f && forearm > 1e-3f)
		{
			const glm::vec3 shoulder= m_torso.shoulder[sideIndex];
			const glm::vec3 wristRay= rayThrough(wristPx);
			const glm::vec3 elbowRay= bElbowSeen ? rayThrough(elbowPx) : glm::vec3(0.f);
			// The elbow's target: where it was, or the rig's hint before any
			// chain. A seen elbow sits on its circle against its ray, with the
			// ray's pixel sigma carried to the elbow's distance, and near the
			// target at the joint-speed sigma; an unseen one nearest the target.
			const glm::vec3 elbowTarget= bHadChain ? arm.chain.elbow : shoulder + sideConfig.elbowHintOffset;
			const float motionSigma=
				std::max(kMaxJointSpeedReachPerS * reach * dtSeconds, kMinMotionSigmaReach * reach);
			const float elbowRaySigma= kElbowPixelSigma / cameraModel.fx * glm::length(shoulder);
			const float elbowTargetSigma= bHadChain ? motionSigma : 0.f;
			auto elbowFor= [&](const glm::vec3& wrist, float targetSigma) {
				return solveElbow(shoulder, wrist, upperArm, forearm, bElbowSeen, elbowRay, elbowRaySigma, elbowTarget,
								  targetSigma);
			};
			const float maxReach= reach * 0.999f;
			const float minReach= std::fabs(upperArm - forearm) * 1.001f;

			// The distances along the ray the arm can reach: between the
			// ray's crossings of the reach sphere about the shoulder, or the
			// one nearest point when the ray misses it
			RaySphereHit reachHit;
			float depthLow= 0.f;
			float depthHigh= 0.f;
			bool bInterval= false;
			if (BodyPoseSolver::intersectRaySphere(k_origin, wristRay, shoulder, maxReach, reachHit))
			{
				if (reachHit.bClamped)
				{
					depthLow= depthHigh= glm::length(reachHit.nearPoint);
				}
				else
				{
					depthLow= reachHit.bNearValid ? glm::length(reachHit.nearPoint) : 1e-2f;
					depthHigh= glm::length(reachHit.farPoint);
				}
				bInterval= depthHigh >= depthLow;
			}

			if (bInterval)
			{
				// The hand lift's wrist distance, measured against the chain
				const float pnpDistanceRaw= (bHandSeen && pose.tracked && pose.hasCameraPose)
					? glm::length(pose.palmPositionCamera)
					: 0.f;
				const bool bPnp= pnpDistanceRaw > 1e-3f;
				// The model's shoulder-to-wrist depth, decompressed and in the
				// avatar's units
				const bool bModelDepth= body.hasDepth;
				const float modelDepthDelta= bModelDepth
					? (body.imagePoints[wristIndex].z - body.imagePoints[shoulderIndex].z) / kModelDepthScale * lengthScale
					: 0.f;
				auto feasible= [&](float depth) {
					return depth >= depthLow && depth <= depthHigh && glm::length(wristRay * depth - shoulder) >= minReach;
				};
				struct Scored
				{
					float depth= 0.f;
					float cost= 0.f;
					float elbowPx= 0.f;
					bool valid= false;
				};
				auto score= [&](float depth, bool bWithMotion) {
					Scored out;
					if (!feasible(depth))
						return out;
					const glm::vec3 wrist= wristRay * depth;
					const glm::vec3 elbow= elbowFor(wrist, bWithMotion ? elbowTargetSigma : 0.f);
					out.depth= depth;
					out.valid= true;
					if (bElbowSeen)
					{
						out.elbowPx= pixelsOffRay(elbow, elbowRay);
						const float ratio= out.elbowPx / kElbowPixelSigma;
						out.cost+= ratio * ratio;
					}
					const glm::vec3 drift= elbow - elbowTarget;
					if (bWithMotion && bHadChain)
					{
						out.cost+= glm::dot(drift, drift) / (motionSigma * motionSigma);
						const float ratio= (depth - arm.lastChosenDepth) / motionSigma;
						out.cost+= ratio * ratio;
					}
					else if (!bElbowSeen && !bHadChain)
					{
						out.cost+= kElbowDriftPenalty * glm::dot(drift, drift) / (reach * reach);
					}
					if (bModelDepth)
					{
						const float ratio= (glm::dot(wrist - shoulder, cameraForward) - modelDepthDelta) /
							(kModelDepthSigmaReach * reach);
						out.cost+= ratio * ratio;
					}
					return out;
				};
				// The best of the even candidates and the last distance, refined
				// by halving the spacing around it
				auto choose= [&](bool bWithMotion) {
					Scored best;
					auto consider= [&](float depth) {
						const Scored scored= score(depth, bWithMotion);
						if (scored.valid && (!best.valid || scored.cost < best.cost))
							best= scored;
					};
					const float spacing= (depthHigh - depthLow) / (float)(kDepthCandidates - 1);
					for (int i= 0; i < kDepthCandidates; ++i)
						consider(depthLow + spacing * (float)i);
					if (bWithMotion && bHadChain)
						consider(arm.lastChosenDepth);
					float step= spacing;
					for (int refine= 0; refine < kDepthRefineSteps && best.valid; ++refine)
					{
						step*= 0.5f;
						const float center= best.depth;
						consider(center - step);
						consider(center + step);
					}
					return best;
				};

				// The lift's scale is measured where the elbow fixes the wrist's
				// distance on its own: this frame's geometry and the model's
				// prior, without the frame before (neither the chain's motion
				// nor the elbow's pull toward where it was). A nearly straight
				// arm fixes nothing: its bone circle has collapsed, and the one
				// elbow it allows reprojects wherever the line of sight puts it.
				if (bElbowSeen && bPnp)
				{
					const Scored geometry= choose(false);
					if (geometry.valid && geometry.elbowPx < kElbowPixelSigma &&
						glm::length(wristRay * geometry.depth - shoulder) < kPnpSampleMaxReach * reach)
					{
						arm.pnpScaleSamples[arm.pnpScaleNextSample]= geometry.depth / pnpDistanceRaw;
						arm.pnpScaleNextSample= (arm.pnpScaleNextSample + 1) % kPnpScaleSamples;
						arm.pnpScaleSampleCount= std::min(arm.pnpScaleSampleCount + 1, kPnpScaleSamples);
						arm.pnpScale= std::clamp(medianOf(arm.pnpScaleSamples.data(), arm.pnpScaleSampleCount),
												 kPnpScaleMin * lengthScale, kPnpScaleMax * lengthScale);
					}
				}
				// The choice: every term
				const Scored chosen= choose(true);

				if (chosen.valid)
				{
					// The chosen distance through the heavy filter, kept within
					// what the arm reaches, and the elbow re-solved for it
					arm.lastChosenDepth= chosen.depth;
					float depth= std::clamp(arm.depthFilter.filter(chosen.depth, dtSeconds), depthLow, depthHigh);
					if (!feasible(depth))
						depth= chosen.depth;
					const glm::vec3 wrist= wristRay * depth;
					arm.chain.shoulder= shoulder;
					arm.chain.wrist= wrist;
					arm.chain.elbow= elbowFor(wrist, elbowTargetSigma);
					arm.chain.valid= true;
					arm.solvedTimestampMs= timestampMs;
				}
			}
		}

		const bool bChainUsable= arm.chain.valid && (timestampMs - arm.solvedTimestampMs) <= kMaxHoldMs;
		if (!bChainUsable)
		{
			arm.chain.valid= false;
			arm.wristPixelFilter.reset();
			arm.elbowPixelFilter.reset();
			arm.depthFilter.reset();
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
		// measured forearm frame (the wrist). Held through a hand dropout, a
		// slot the body disagrees with counting as one.
		const AvatarSkeleton::HandRest& handRest= skeleton.getHand(side);
		const glm::vec3 restPalmNormal=
			safeNormalize(projectPerpendicular(chainDelta * glm::vec3(handRest.palmFrameWorld[2]), forearmDirection));
		if (bHandSeen && pose.tracked && pose.hasCameraPose)
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

			// The hold: a raw palm that jumps away from the filtered one is
			// the lift's remaining flip until it persists, then it is a turn
			// and the filters restart on it rather than dragging across
			bool bHoldPalm= false;
			if (arm.bPalmFiltered)
			{
				const glm::vec3 rawZ= safeNormalize(projectPerpendicular(palmZRaw, safeNormalize(palmXRaw)));
				const float jump= glm::dot(rawZ, rawZ) > 0.f
					? rotationAngle(glm::inverse(arm.filteredPalm) * palmFrame(safeNormalize(palmXRaw), rawZ))
					: 0.f;
				if (jump > kPalmJumpRadians)
				{
					arm.palmJumpFrames++;
					if (arm.palmJumpFrames < kPalmJumpAcceptFrames)
					{
						bHoldPalm= true;
					}
					else
					{
						arm.palmXFilter.reset();
						arm.palmZFilter.reset();
						arm.palmJumpFrames= 0;
					}
				}
				else
				{
					arm.palmJumpFrames= 0;
				}
			}

			if (!bHoldPalm)
			{
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
					const glm::quat palm= palmFrame(palmX, palmZ);
					arm.filteredPalm= palm;
					arm.bPalmFiltered= true;

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
			}

			// Fingers: the single view's angles, filtered
			std::array<FingerAngles, FINGER_COUNT> measured;
			measureMonoFingers(hand, arm.palmarMemory, measured);
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				const FingerAngles& angles= measured[finger];
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
			arm.bPalmFiltered= false;
			arm.palmJumpFrames= 0;
			arm.palmarMemory.reset();
			for (OneEuroFilter& filter : arm.fingerFilters)
				filter.reset();
		}

		// Forearm roll: the measured twist against the avatar's rest palm,
		// plus the rig's trim; an arm whose hand was never seen rolls only
		// by the trim
		float twist= sideConfig.forearmRollTrimRadians;
		if (arm.bHandMeasured)
			twist+= arm.twist;
		chainDelta= glm::angleAxis(twist, forearmDirection) * chainDelta;
		placeBone(outPose, skeleton, lowerArmBone, chainDelta);

		// The hand bone: the measured wrist, applied in the avatar's trimmed
		// rest palm frame under the posed forearm. A straight wrist is the
		// avatar's rest wrist: the user's hands carry no reference, since a
		// single view's palm at one instant is too unsure to be subtracted
		// from every later frame.
		glm::quat relative(1.f, 0.f, 0.f, 0.f);
		if (arm.bHandMeasured)
			relative= limitRotationAngle(arm.wristRotation, kWristLimitRadians);
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

bool DirectRetarget::captureNeutral(DirectNeutralPose& outNeutral) const
{
	outNeutral= DirectNeutralPose();
	if (!m_torso.valid)
		return false;

	// Back to the level frame, so the references survive a tilt edit
	const glm::quat levelFromTilted= glm::inverse(worldFromCamera(m_tiltRadians) * glm::inverse(worldFromCamera(0.f)));
	outNeutral.present= true;
	outNeutral.torso= levelFromTilted * m_torso.lineFrame;
	outNeutral.shoulderMidpoint= levelFromTilted * m_torso.midpoint;
	outNeutral.head= m_head.bMeasured ? levelFromTilted * m_head.frame : glm::quat(1.f, 0.f, 0.f, 0.f);
	return true;
}
