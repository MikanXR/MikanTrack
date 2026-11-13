#include "DirectRetarget.h"

#include <algorithm>
#include <cmath>

#include "glm/ext/matrix_transform.hpp"
#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/quaternion.hpp"

#include "AvatarRetargetMath.h"
#include "AvatarSkeleton.h"
#include "BodyPoseSolver.h"
#include "BodyPoseTypes.h"
#include "HandSideMatch.h"
#include "MathGLM.h"

namespace
{
using B= eHumanoidBone;
using namespace AvatarRetargetMath;
using RaySphereHit= BodyPoseSolver::RaySphereHit;

// Landmark visibility below this is treated as unseen. A wrist, and a hand
// slot on it, gets in only after this many consecutive frames of evidence:
// the model's visibility flickers across the bar on a joint it has placed
// by guesswork (a wrist below the frame put beside a hand-shaped thing on
// the wall), and one frame across it used to be enough to pose the arm and
// start the hold. The bar itself stays where it is: the model scores a
// plainly visible wrist at 0.5 to 0.7 on the seated takes.
constexpr float kMinVisibility= 0.5f;
constexpr int kEnterFrames= 3;
// A landmark this close to the frame edge is treated as out of frame: the
// model places a joint it cannot see somewhere plausible inside the image,
// and that somewhere is not where the joint is
constexpr float kFrameMarginPx= 16.f;
// A hand slot farther than this from the body model's wrist, in shoulder
// widths, is not the hand: the palm detector runs over the whole frame and
// fires on hand-shaped things in the scene
constexpr float kHandAgreementShoulderWidths= 0.5f;
// Hand slots that sit on the opposite side's body wrist for this long, with
// none on its own, are labelled the wrong way round
constexpr double kHandednessSwapMs= 1000.0;
// A group (torso, head, an arm) unseen for longer than this returns to rest,
// and a gap longer than this restarts the filters. An arm earns its hold:
// it is held for as long as it was continuously placed plus one frame, up
// to this, so a sighting of a few frames does not leave the arm up for half
// a second.
constexpr double kMaxHoldMs= 500.0;
// A pair of landmarks whose known separation places it outside this range
// is not on the person
constexpr float kMinBodyDepthMeters= 0.15f;
constexpr float kMaxBodyDepthMeters= 2.0f;

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
// The hand lift's wrist distance, in the avatar's units through the length
// ratio, is the one metric cue the single view has: the candidate's
// distance from it, over this fraction of it
constexpr float kLiftDepthSigmaFraction= 0.15f;
// The lift's depth against the chain's is also measured, for the report:
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

// A gate over a per-frame reading: in after kEnterFrames consecutive frames
// of evidence, out on the first frame without
bool advanceGate(bool bEvidence, int& frames, bool& bAccepted)
{
	frames= bEvidence ? std::min(frames + 1, kEnterFrames) : 0;
	bAccepted= frames >= kEnterFrames;
	return bAccepted;
}

// The hold an arm has earned by the time it was last placed: the length of
// its unbroken run of placements plus one frame, so a run of one frame
// carries to the next frame and no further
double earnedHoldMs(double sightingStartMs, double solvedTimestampMs, double frameMs)
{
	return std::min(std::max(solvedTimestampMs - sightingStartMs, 0.0) + frameMs, kMaxHoldMs);
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

// The chain's swings: the upper arm and the forearm from rest onto the
// directions, chained from the spine. A forearm without a direction takes
// the upper arm's, and without either it keeps the swung rest direction.
struct ChainSwing
{
	glm::quat upperDelta{1.f, 0.f, 0.f, 0.f};
	glm::quat forearmDelta{1.f, 0.f, 0.f, 0.f};
	glm::vec3 forearmDirection{0.f};
};
ChainSwing swingChain(const AvatarSkeleton& skeleton, int sideIndex, const glm::quat& spineDelta,
					  const glm::vec3& upperDirection, glm::vec3 forearmDirection)
{
	const B lowerArmBone= sideBone(sideIndex, B::LeftLowerArm, B::RightLowerArm);
	const B handBone= sideBone(sideIndex, B::LeftHand, B::RightHand);
	if (glm::dot(forearmDirection, forearmDirection) <= 0.f)
		forearmDirection= upperDirection;
	ChainSwing swing;
	swing.upperDelta= spineDelta;
	if (glm::dot(upperDirection, upperDirection) > 0.f)
		swing.upperDelta= swingTo(spineDelta, restDirection(skeleton, lowerArmBone), upperDirection);
	swing.forearmDelta= swing.upperDelta;
	if (glm::dot(forearmDirection, forearmDirection) > 0.f)
		swing.forearmDelta= swingTo(swing.upperDelta, restDirection(skeleton, handBone), forearmDirection);
	else
		forearmDirection= swing.upperDelta * restDirection(skeleton, handBone);
	swing.forearmDirection= forearmDirection;
	return swing;
}

// The avatar's rest palm normal under the chain's swings, clear of the
// forearm: the zero of the forearm's twist
glm::vec3 restPalmNormalUnder(const AvatarSkeleton& skeleton, eHandSide side, const ChainSwing& swing)
{
	const AvatarSkeleton::HandRest& handRest= skeleton.getHand(side);
	return safeNormalize(
		projectPerpendicular(swing.forearmDelta * glm::vec3(handRest.palmFrameWorld[2]), swing.forearmDirection));
}

// A palm frame from its finger axis and a normal that may not be exactly
// perpendicular to it
glm::quat palmFrame(const glm::vec3& palmX, const glm::vec3& palmZ)
{
	const glm::vec3 palmY= glm::cross(palmZ, palmX);
	return glm::quat_cast(glm::mat3(palmX, palmY, palmZ));
}

// The hand from a palm frame (+X the finger axis, +Z the palm normal): the
// normal's roll about the forearm against the avatar's rest palm, and the
// palm in the forearm's frame (the wrist). The forearm frame takes the
// normal projected clear of the forearm, or the palm's +Y when the normal
// lies along it; false when both do. A zero rest normal leaves the twist.
bool handFromPalm(const glm::vec3& forearmDirection, const glm::vec3& restPalmNormal, const glm::quat& palm,
				  float& ioTwist, glm::quat& outWristRotation)
{
	const glm::vec3 palmY= palm * glm::vec3(0.f, 1.f, 0.f);
	const glm::vec3 palmZ= palm * glm::vec3(0.f, 0.f, 1.f);
	glm::vec3 forearmZ= safeNormalize(projectPerpendicular(palmZ, forearmDirection));
	if (glm::dot(forearmZ, forearmZ) <= 0.f)
		forearmZ= safeNormalize(projectPerpendicular(palmY, forearmDirection));
	if (glm::dot(forearmZ, forearmZ) <= 0.f)
		return false;
	if (glm::dot(restPalmNormal, restPalmNormal) > 0.f)
		ioTwist= signedAngle(restPalmNormal, forearmZ, forearmDirection);
	const glm::vec3 forearmY= glm::cross(forearmZ, forearmDirection);
	const glm::quat forearmFrame= glm::quat_cast(glm::mat3(forearmDirection, forearmY, forearmZ));
	outWristRotation= glm::inverse(forearmFrame) * palm;
	return true;
}

// The arm's bones from a placement: the chain's swings, the forearm's roll
// (the rig's trim plus the twist), the hand bone (the wrist applied in the
// avatar's trimmed rest palm frame under the posed forearm: a straight
// wrist is the avatar's rest wrist, since the user's hands carry no
// reference, a single view's palm at one instant being too unsure to be
// subtracted from every later frame, within wristLimitRadians) and the
// fingers
void placeArm(const AvatarSkeleton& skeleton, int sideIndex, const glm::quat& spineDelta,
			  const DirectRetarget::ArmPlacement& placement, const AvatarRetargetConfig::Side& sideConfig,
			  float wristLimitRadians, AvatarPose& outPose)
{
	const eHandSide side= (eHandSide)sideIndex;
	const B upperArmBone= sideBone(sideIndex, B::LeftUpperArm, B::RightUpperArm);
	const B lowerArmBone= sideBone(sideIndex, B::LeftLowerArm, B::RightLowerArm);
	const B handBone= sideBone(sideIndex, B::LeftHand, B::RightHand);
	const ChainSwing swing=
		swingChain(skeleton, sideIndex, spineDelta, placement.upperDirection, placement.forearmDirection);
	placeBone(outPose, skeleton, upperArmBone, swing.upperDelta);
	const glm::quat chainDelta=
		glm::angleAxis(sideConfig.forearmRollTrimRadians + placement.twist, swing.forearmDirection) *
		swing.forearmDelta;
	placeBone(outPose, skeleton, lowerArmBone, chainDelta);

	const AvatarSkeleton::HandRest& handRest= skeleton.getHand(side);
	const glm::quat relative= limitRotationAngle(placement.wristRotation, wristLimitRadians);
	const glm::quat restLowerArm= skeleton.getBone(lowerArmBone).restRotationWorld;
	const glm::quat trimmedRestPalm=
		glm::quat_cast(glm::mat3(handRest.palmFrameWorld)) * glm::quat(sideConfig.handTrimRadians);
	const glm::quat posedForearm= chainDelta * restLowerArm;
	const glm::quat posedPalm= posedForearm * (glm::inverse(restLowerArm) * trimmedRestPalm) * relative;
	const glm::quat handDelta= posedPalm * glm::inverse(trimmedRestPalm);
	placeBone(outPose, skeleton, handBone, handDelta);

	solveFingerBones(skeleton, sideIndex, glm::quat(1.f, 0.f, 0.f, 0.f), handDelta,
					 skeleton.getBone(handBone).restPositionWorld, placement.fingers, sideConfig, outPose);
}

// An arm's placement with its chain's directions pulled from its bone angles
// toward a goal by the arm's IK weight, the twist, the wrist and the fingers
// kept, so they ride on the posed forearm as the bone angles place them. The
// world is the posed avatar's, whose axes are the solve's (the two differ by
// a translation), so the source's offsets carry across as they are, and in
// the avatar's units. avatarShoulder is the posed upper-arm joint.
void pullPlacementTowardGoal(const SourcePose& source, const AvatarSkeleton& skeleton, int sideIndex,
							 const glm::quat& spineDelta, const glm::vec3& avatarShoulder,
							 const AvatarRetargetArm& armSettings, const AvatarRetargetConfig::Side& sideConfig,
							 DirectRetarget::ArmPlacement& ioPlacement, AvatarRetargetGoals* outGoals)
{
	const eHandSide side= (eHandSide)sideIndex;
	const SourcePose::Arm& sourceArm= source.arms[sideIndex];
	const B lowerArmBone= sideBone(sideIndex, B::LeftLowerArm, B::RightLowerArm);
	const float upperArm= skeleton.getUpperArmLength(side);
	const float forearm= skeleton.getForearmLength(side);
	const float avatarReach= upperArm + forearm;

	// The arm as its bone angles place it: the directions the swings take
	const ChainSwing swing=
		swingChain(skeleton, sideIndex, spineDelta, ioPlacement.upperDirection, ioPlacement.forearmDirection);
	const glm::vec3 angleUpper= swing.upperDelta * restDirection(skeleton, lowerArmBone);
	const glm::vec3 angleElbow= avatarShoulder + angleUpper * upperArm;
	const glm::vec3 angleWrist= angleElbow + swing.forearmDirection * forearm;

	// The source's offsets from its shoulder: the chain's, or for an arm with
	// no chain (one easing to rest) its directions at the source's lengths
	glm::vec3 wristOffset;
	glm::vec3 elbowOffset;
	if (sourceArm.bHasShoulder && sourceArm.bHasElbow)
	{
		wristOffset= sourceArm.wristPosition - sourceArm.shoulderPosition;
		elbowOffset= sourceArm.elbowPosition - sourceArm.shoulderPosition;
	}
	else
	{
		elbowOffset= angleUpper * source.upperArmLength;
		wristOffset= elbowOffset + swing.forearmDirection * source.forearmLength;
	}

	// The source's arm is the user's proportions in the avatar's units, so
	// the reach scale's ratio is the avatar's arm over the source's
	const float sourceReach= source.upperArmLength + source.forearmLength;
	const float ratio= sourceReach > 1e-3f ? avatarReach / sourceReach : 1.f;
	const float factor= reachFactor(armSettings.reachScale, ratio);
	glm::vec3 goal= avatarShoulder + wristOffset * factor;
	// The root is the identity, so the rooted torso frame is the world's
	if (armSettings.goalOffset != glm::vec3(0.f))
		goal+= armSettings.goalOffset;
	if (outGoals != nullptr)
	{
		outGoals->bHasWristGoal[sideIndex]= true;
		outGoals->wristGoalWorld[sideIndex]= goal;
	}
	const glm::vec3 toGoal= goal - avatarShoulder;
	const float reach= glm::length(toGoal);
	if (reach > avatarReach && reach > 1e-6f)
		goal= avatarShoulder + toGoal * (avatarReach / reach);
	const glm::vec3 polePoint= avatarShoulder + elbowOffset * factor;

	const float ikWeight= std::min(armSettings.ikWeight, 1.f);
	glm::vec3 elbow;
	glm::vec3 wrist;
	pullArmTowardGoal(avatarShoulder, angleElbow, angleWrist, goal, polePoint, ikWeight, upperArm, forearm,
					  safeNormalize(sideConfig.elbowHintOffset), angleUpper, elbow, wrist);
	const glm::vec3 upperDirection= safeNormalize(elbow - avatarShoulder);
	const glm::vec3 forearmDirection= safeNormalize(wrist - elbow);
	if (glm::dot(upperDirection, upperDirection) > 0.f)
		ioPlacement.upperDirection= upperDirection;
	if (glm::dot(forearmDirection, forearmDirection) > 0.f)
		ioPlacement.forearmDirection= forearmDirection;
}

// A unit direction between two, falling to the target where they cancel
glm::vec3 mixDirection(const glm::vec3& from, const glm::vec3& to, float t)
{
	const glm::vec3 mixed= safeNormalize(glm::mix(from, to, t));
	return glm::dot(mixed, mixed) > 0.f ? mixed : to;
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

DirectRetarget::ArmPlacement DirectRetarget::makeRestPlacement(const AvatarSkeleton& skeleton, int sideIndex,
															   const glm::quat& spineDelta,
															   const AvatarRetargetConfig::Side& sideConfig)
{
	// A normal along the forearm leaves the wrist straight
	const eHandSide side= (eHandSide)sideIndex;
	ArmPlacement rest;
	rest.upperDirection= safeNormalize(spineDelta * sideConfig.restElbowOffset);
	rest.forearmDirection= safeNormalize(spineDelta * (sideConfig.restWristOffset - sideConfig.restElbowOffset));
	const ChainSwing swing= swingChain(skeleton, sideIndex, spineDelta, rest.upperDirection, rest.forearmDirection);
	const glm::vec3 palmZ=
		safeNormalize(projectPerpendicular(spineDelta * sideConfig.restPalmNormal, swing.forearmDirection));
	if (glm::dot(palmZ, palmZ) > 0.f)
	{
		const glm::quat palm= palmFrame(swing.forearmDirection, palmZ);
		handFromPalm(swing.forearmDirection, restPalmNormalUnder(skeleton, side, swing), palm, rest.twist,
					 rest.wristRotation);
	}
	for (int finger= (int)eFinger::Index; finger < FINGER_COUNT; ++finger)
	{
		rest.fingers[finger].proximal= sideConfig.restFingerCurlRadians;
		rest.fingers[finger].intermediate= sideConfig.restFingerCurlRadians;
		rest.fingers[finger].distal= 0.5f * sideConfig.restFingerCurlRadians;
	}
	return rest;
}

DirectRetarget::ArmPlacement DirectRetarget::blendPlacements(const ArmPlacement& from, const ArmPlacement& to, float t)
{
	ArmPlacement out;
	out.upperDirection= mixDirection(from.upperDirection, to.upperDirection, t);
	out.forearmDirection= mixDirection(from.forearmDirection, to.forearmDirection, t);
	out.twist= from.twist + wrapAngle(to.twist - from.twist) * t;
	out.wristRotation= glm::slerp(from.wristRotation, to.wristRotation, t);
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		out.fingers[finger].lateral= glm::mix(from.fingers[finger].lateral, to.fingers[finger].lateral, t);
		out.fingers[finger].proximal= glm::mix(from.fingers[finger].proximal, to.fingers[finger].proximal, t);
		out.fingers[finger].intermediate=
			glm::mix(from.fingers[finger].intermediate, to.fingers[finger].intermediate, t);
		out.fingers[finger].distal= glm::mix(from.fingers[finger].distal, to.fingers[finger].distal, t);
	}
	return out;
}

void DirectRetarget::reset()
{
	resetMeasure();
	m_resetEpoch++;
}

void DirectRetarget::resetMeasure()
{
	m_state= SolveState();
	m_bHasTime= false;
	m_lastTimestampMs= 0.0;
	m_handednessSwapStartMs= -1.0;
	m_oppositeWristRejections[0]= 0;
	m_oppositeWristRejections[1]= 0;
	m_sideCorrections[0]= 0;
	m_sideCorrections[1]= 0;
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

void DirectRetarget::measure(const TrackingFrameResult& camera, const DirectCameraModel& cameraModel,
							 const TrackingFrameResult::HeadPose& phoneHead, double timestampMs,
							 const AvatarSkeleton& skeleton, const DirectRetargetConfig& config, SourcePose& outSource)
{
	using eGroupState= SourcePose::eGroupState;

	// The frame clock. A regression (a replay scrub) or a gap past the hold
	// starts over: nothing measured before it should shape what comes after.
	if (m_bHasTime)
	{
		const double gapMs= timestampMs - m_lastTimestampMs;
		if (gapMs < 0.0 || gapMs > kMaxHoldMs)
		{
			resetMeasure();
			m_resetEpoch++;
		}
	}
	if (!m_bFiltersConfigured)
		configureFilters();
	const float dtSeconds= m_bHasTime ? (float)((timestampMs - m_lastTimestampMs) * 0.001) : 0.f;
	const double frameMs= m_bHasTime ? timestampMs - m_lastTimestampMs : 0.0;
	m_bHasTime= true;
	m_lastTimestampMs= timestampMs;

	outSource= SourcePose();
	outSource.pipeline= SourcePose::ePipeline::Direct;
	// Any backward step restarts above, so the step clock is the frame's
	outSource.clock.timestampMs= timestampMs;
	outSource.clock.stepMs= timestampMs;
	outSource.clock.frameIntervalMs= frameMs;
	outSource.clock.resetEpoch= m_resetEpoch;

	m_state= SolveState();
	m_state.camera= cameraModel;
	m_state.tiltRadians= config.cameraTiltRadians;
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
	outSource.upperArmLength= config.userUpperArmLengthMeters * lengthScale;
	outSource.forearmLength= config.userForearmLengthMeters * lengthScale;

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
	m_state.torsoValid= bTorsoUsable;
	if (bTorsoUsable)
	{
		m_state.shoulder[0]= m_torso.shoulder[0];
		m_state.shoulder[1]= m_torso.shoulder[1];
		m_state.shoulderMidpoint= m_torso.midpoint;
		m_state.torsoFrame= m_torso.lineFrame;
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
		spineDelta= limitRotationAngle(torsoNow * glm::inverse(neutralTorso), config.spineLimitRadians);
	}
	m_state.spineDelta= spineDelta;
	outSource.torso.state= bTorsoFresh ? eGroupState::Measured : bTorsoUsable ? eGroupState::Held : eGroupState::Lost;
	outSource.torso.spineDelta= spineDelta;

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
	if (phoneHead.valid)
	{
		// Already canonical: the face anchor captured at the neutral pose
		// makes looking at the screen the identity
		outSource.head.state= eGroupState::Measured;
		outSource.head.confidence.bPresent= true;
		outSource.head.confidence.value= phoneHead.confidence;
		outSource.head.orientation= phoneHead.orientationWorld;
	}
	else if (bHeadUsable)
	{
		outSource.head.state=
			m_head.solvedTimestampMs == timestampMs ? eGroupState::Measured : eGroupState::Held;
		outSource.head.orientation=
			config.neutral.present ? m_head.frame * glm::inverse(tilt * config.neutral.head) : m_head.frame;
	}
	m_state.headMeasured= bHeadUsable;
	if (bHeadUsable)
		m_state.headFrame= m_head.frame;

	// -- Arms --
	// This frame's evidence on the hand labels: a slot on its own side's
	// body wrist, and a slot on the other side's
	bool bAnyHandOnOwnWrist= false;
	bool bAnyHandOnOppositeWrist= false;
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
		// The chain in the user's proportions, in the avatar's units
		const float upperArm= config.userUpperArmLengthMeters * lengthScale;
		const float forearm= config.userForearmLengthMeters * lengthScale;
		const float reach= upperArm + forearm;
		// Before any sample the hand lift's depth is trusted at the length
		// ratio, the scale a correct lift would measure
		if (arm.pnpScaleSampleCount == 0)
			arm.pnpScale= lengthScale;

		// The entry gates apply to an arm coming from rest: a visibility
		// flicker on a wrist the model cannot see neither poses the arm nor
		// starts the hold. An arm already placed and within its hold takes
		// the wrist and the hand back the frame they return, since a dip of
		// one frame is the model's score wavering on a wrist it has, and the
		// hand's lift is what keeps the wrist's distance from flipping.
		const bool bHadChain= arm.chain.valid &&
			(timestampMs - arm.solvedTimestampMs) <=
				earnedHoldMs(arm.sightingStartMs, arm.solvedTimestampMs, frameMs);
		const bool bWristVisible= usable(wristIndex);
		const bool bWristGate= advanceGate(bWristVisible, arm.wristGateFrames, arm.bWristAccepted);
		const bool bWristUsable= bHadChain ? bWristVisible : bWristGate;

		// The hand slot is the hand only where the body model agrees: the
		// palm detector runs over the whole frame and fires on hand-shaped
		// things in the scene, while the body model places the wrist by the
		// whole figure and reports one it cannot see as unseen. Agreement is
		// measured in shoulder widths, so it scales with the user's distance,
		// and it has to hold for the entry frames before the slot is the hand.
		const glm::vec2 handWristPx(hand.imagePoints[0].x, hand.imagePoints[0].y);
		const glm::vec2 bodyWristPx= landmarkPx(wristIndex);
		const bool bHandNear= hand.tracked && bWristVisible && shoulderWidthPx > 0.f &&
			glm::distance(handWristPx, bodyWristPx) <= kHandAgreementShoulderWidths * shoulderWidthPx;
		const bool bHandGate= advanceGate(bHandNear, arm.handGateFrames, arm.bHandAccepted);
		// The same slot against the other arm's wrist. The body model labels
		// a wrist by the arm it ends, so a slot there and not here carries
		// the wrong side, whatever the arms are doing.
		const int oppositeWristIndex=
			sideIndex == 0 ? (int)ePoseLandmark::RIGHT_WRIST : (int)ePoseLandmark::LEFT_WRIST;
		const bool bHandOpposite= hand.tracked && usable(oppositeWristIndex) && shoulderWidthPx > 0.f &&
			glm::distance(handWristPx, landmarkPx(oppositeWristIndex)) <=
				kHandAgreementShoulderWidths * shoulderWidthPx;
		// The label report takes the slot's side from the hand's shape
		// against the two whole-body hands where both score, which still
		// tells the hands apart with the wrists together, and from the wrists
		// otherwise. The entry gate above keeps the wrist alone. The report
		// is about the hand model's labels, so a slot the camera moved to
		// this side is read against the side it was published under.
		const bool bSideCorrected= hand.tracked && hand.bSideCorrected;
		const int labelSide= bSideCorrected ? 1 - sideIndex : sideIndex;
		const bool bNearLabelWrist= bSideCorrected ? bHandOpposite : bHandNear;
		const bool bNearOtherLabelWrist= bSideCorrected ? bHandNear : bHandOpposite;
		const HandShapeMatch shape=
			hand.tracked ? matchHandSideByShape(hand.imagePoints, body.wholeBody) : HandShapeMatch();
		bAnyHandOnOwnWrist|= shape.side >= 0 ? shape.side == labelSide : bNearLabelWrist;
		bAnyHandOnOppositeWrist|= shape.side >= 0 ? shape.side != labelSide : bNearOtherLabelWrist;
		// A slot the gate turned away while it sat on the other arm's wrist:
		// what a wrong label costs
		if (hand.tracked && !bHandNear && bHandOpposite)
			m_oppositeWristRejections[sideIndex]++;
		const bool bHandSeen= (bHadChain ? bHandNear : bHandGate) && bWristUsable;
		// A hand this arm has only because the camera moved the slot to it
		if (bHandSeen && bSideCorrected)
			m_sideCorrections[sideIndex]++;

		// The wrist pixel: the hand model's wrist while the hand is seen, else
		// the body model's, which keeps the arm placed through a hand dropout;
		// either through the same light filter. The elbow pixel likewise.
		bool bWristPixel= false;
		glm::vec2 wristPx(0.f);
		if (bHandSeen || bWristUsable)
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
		SolveState::Arm& armState= m_state.arms[sideIndex];
		armState.bHandSeen= bHandSeen;
		armState.bElbowSeen= bElbowSeen;

		if (bWristPixel && !bHadChain)
			arm.sightingStartMs= timestampMs;
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
				// The hand lift's wrist distance, scored in the avatar's units
				// and measured against the chain
				const float pnpDistanceRaw= (bHandSeen && pose.tracked && pose.hasCameraPose)
					? glm::length(pose.palmPositionCamera)
					: 0.f;
				const bool bPnp= pnpDistanceRaw > 1e-3f;
				const float liftDepth= pnpDistanceRaw * lengthScale;
				// The model's shoulder-to-wrist depth, decompressed and in the
				// avatar's units
				const bool bModelDepth= body.hasDepth;
				const float modelDepthDelta= bModelDepth
					? (body.imagePoints[wristIndex].z - body.imagePoints[shoulderIndex].z) / kModelDepthScale * lengthScale
					: 0.f;
				armState.depthLow= depthLow;
				armState.depthHigh= depthHigh;
				armState.modelDepthDelta= modelDepthDelta;
				armState.pnpDistance= pnpDistanceRaw;
				armState.liftDepth= liftDepth;
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
				auto score= [&](float depth, bool bWithMotion, bool bWithLift) {
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
					if (bWithLift && bPnp)
					{
						const float ratio= (depth - liftDepth) / (kLiftDepthSigmaFraction * liftDepth);
						out.cost+= ratio * ratio;
					}
					return out;
				};
				// The best of the even candidates and the last distance, refined
				// by halving the spacing around it
				auto choose= [&](bool bWithMotion, bool bWithLift) {
					Scored best;
					auto consider= [&](float depth) {
						const Scored scored= score(depth, bWithMotion, bWithLift);
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
				// prior, without the lift itself and without the frame before
				// (neither the chain's motion nor the elbow's pull toward where
				// it was). A nearly straight arm fixes nothing: its bone circle
				// has collapsed, and the one elbow it allows reprojects wherever
				// the line of sight puts it.
				if (bElbowSeen && bPnp)
				{
					const Scored geometry= choose(false, false);
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
				const Scored chosen= choose(true, true);

				if (chosen.valid)
				{
					// The chosen distance through the heavy filter, kept within
					// what the arm reaches, and the elbow re-solved for it
					arm.lastChosenDepth= chosen.depth;
					float depth= std::clamp(arm.depthFilter.filter(chosen.depth, dtSeconds), depthLow, depthHigh);
					if (!feasible(depth))
						depth= chosen.depth;
					armState.bDepthChosen= true;
					armState.chosenDepth= chosen.depth;
					armState.chosenCost= chosen.cost;
					armState.chosenElbowPx= chosen.elbowPx;
					armState.filteredDepth= depth;
					const glm::vec3 wrist= wristRay * depth;
					arm.chain.shoulder= shoulder;
					arm.chain.wrist= wrist;
					arm.chain.elbow= elbowFor(wrist, elbowTargetSigma);
					arm.chain.valid= true;
					arm.solvedTimestampMs= timestampMs;
				}
			}
		}

		const bool bChainUsable= arm.chain.valid &&
			(timestampMs - arm.solvedTimestampMs) <=
				earnedHoldMs(arm.sightingStartMs, arm.solvedTimestampMs, frameMs);
		if (!bChainUsable)
		{
			// Nothing places the arm: the source says so, and the loss stage
			// decides what a lost arm does
			arm.chain.valid= false;
			arm.wristPixelFilter.reset();
			arm.elbowPixelFilter.reset();
			arm.depthFilter.reset();
			continue;
		}
		armState.chain= arm.chain;
		armState.pnpScale= getPnpScale(sideIndex);

		// The chain's directions, and the swings the placement will make onto
		// them, which the hand is measured under
		const glm::vec3 upperDirection= safeNormalize(arm.chain.elbow - arm.chain.shoulder);
		const glm::vec3 chainForearmDirection= safeNormalize(arm.chain.wrist - arm.chain.elbow);
		const ChainSwing swing= swingChain(skeleton, sideIndex, spineDelta, upperDirection, chainForearmDirection);
		const glm::vec3 forearmDirection= swing.forearmDirection;

		// The hand: the palm frame in the world, its normal's roll about the
		// forearm against the avatar's rest palm, and the palm in the
		// measured forearm frame (the wrist). Held through a hand dropout, a
		// slot the body disagrees with counting as one.
		const glm::vec3 restPalmNormal= restPalmNormalUnder(skeleton, side, swing);
		const bool bHandRead= bHandSeen && pose.tracked && pose.hasCameraPose;
		if (bHandRead)
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
				const float mirrorGain= glm::dot(mirroredX, forearmDirection) - glm::dot(palmXRaw, forearmDirection);
				arm.bPalmMirrored=
					mirrorGain > (arm.bPalmMirrored ? config.palmMirrorLeaveGain : config.palmMirrorEnterGain);
				armState.palmMirrorGain= mirrorGain;
				if (arm.bPalmMirrored)
				{
					palmXRaw= mirroredX;
					palmZRaw= mirroredZ;
				}
			}
			else
			{
				arm.bPalmMirrored= false;
			}
			armState.bPalmMirrored= arm.bPalmMirrored;

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
					const glm::quat palm= palmFrame(palmX, palmZ);
					arm.filteredPalm= palm;
					arm.bPalmFiltered= true;
					if (handFromPalm(forearmDirection, restPalmNormal, palm, arm.twist, arm.wristRotation))
						arm.bHandMeasured= true;
				}
			}

			// Fingers: the single view's angles, filtered
			std::array<FingerAngles, FINGER_COUNT> measured;
			measureMonoFingers(hand, arm.palmarMemory, measured);
			armState.measuredFingers= measured;
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
			arm.bPalmMirrored= false;
			arm.palmarMemory.reset();
			for (OneEuroFilter& filter : arm.fingerFilters)
				filter.reset();
		}

		armState.bHandMeasured= arm.bHandMeasured;
		armState.twist= arm.twist;
		armState.wristRotation= arm.wristRotation;
		armState.filteredPalm= arm.filteredPalm;
		armState.fingers= arm.fingers;

		// The source: the chain, and what places the arm's bones. An arm
		// whose hand was never seen rolls only by the rig's trim and keeps
		// the avatar's rest wrist.
		SourcePose::Arm& sourceArm= outSource.arms[sideIndex];
		sourceArm.state= arm.solvedTimestampMs == timestampMs ? eGroupState::Measured : eGroupState::Held;
		sourceArm.bHasShoulder= true;
		sourceArm.shoulderPosition= arm.chain.shoulder;
		sourceArm.bHasElbow= true;
		sourceArm.elbowPosition= arm.chain.elbow;
		sourceArm.wristPosition= arm.chain.wrist;
		sourceArm.upperDirection= upperDirection;
		sourceArm.forearmDirection= chainForearmDirection;
		SourcePose::Hand& sourceHand= outSource.hands[sideIndex];
		sourceHand.state= bHandRead ? eGroupState::Measured
			: arm.bHandMeasured     ? eGroupState::Held
									: eGroupState::Lost;
		if (bHandRead)
		{
			sourceHand.confidence.bPresent= true;
			sourceHand.confidence.value= pose.confidence;
		}
		if (arm.bHandMeasured)
		{
			sourceArm.twist= arm.twist;
			sourceHand.wristRotation= arm.wristRotation;
		}
		sourceHand.fingerZero= SourcePose::eFingerZero::Metacarpal;
		sourceHand.fingers= arm.fingers;
	}

	// Swapped hand labels: a run of frames with a slot on the opposite wrist
	// and none on its own. A slot on its own wrist ends the run, and a frame
	// with no evidence either way leaves it as it is. Reported only, nothing
	// in the solve reads it.
	if (bAnyHandOnOwnWrist)
		m_handednessSwapStartMs= -1.0;
	else if (bAnyHandOnOppositeWrist && m_handednessSwapStartMs < 0.0)
		m_handednessSwapStartMs= timestampMs;
	m_state.bHandednessSwapped=
		m_handednessSwapStartMs >= 0.0 && (timestampMs - m_handednessSwapStartMs) >= kHandednessSwapMs;
	m_state.oppositeWristRejections[0]= m_oppositeWristRejections[0];
	m_state.oppositeWristRejections[1]= m_oppositeWristRejections[1];
	m_state.sideCorrections[0]= m_sideCorrections[0];
	m_state.sideCorrections[1]= m_sideCorrections[1];
}

void DirectRetarget::place(const SourcePose& source, const AvatarSkeleton& skeleton,
						   const DirectRetargetConfig& config, AvatarPose& outPose)
{
	std::array<AvatarRetargetArm, 2> boneAngles;
	for (AvatarRetargetArm& arm : boneAngles)
		arm.ikWeight= 0.f;
	place(source, skeleton, config, boneAngles, outPose);
}

void DirectRetarget::place(const SourcePose& source, const AvatarSkeleton& skeleton,
						   const DirectRetargetConfig& config, const std::array<AvatarRetargetArm, 2>& arms,
						   AvatarPose& outPose, AvatarRetargetGoals* outGoals)
{
	using eGroupState= SourcePose::eGroupState;
	if (outGoals != nullptr)
		*outGoals= AvatarRetargetGoals();

	outPose.clear();
	outPose.valid= true;
	outPose.timestampMs= source.clock.timestampMs;
	// The root is fixed: the camera is the world and the user faces it
	placeBone(outPose, skeleton, B::Hips, glm::quat(1.f, 0.f, 0.f, 0.f));

	// A released torso leaves the spine bones out, at the avatar's rest, which
	// is the identity the lost torso's spineDelta carries, so the arms hang
	// off the spine as it is drawn
	const glm::quat spineDelta= source.torso.spineDelta;
	if (source.torso.state != eGroupState::Lost)
	{
		placeBone(outPose, skeleton, B::Spine, spineDelta);
		placeBone(outPose, skeleton, B::Chest, spineDelta);
		placeBone(outPose, skeleton, B::UpperChest, spineDelta);
		placeBone(outPose, skeleton, B::Neck, spineDelta);
	}

	// The head within its limit, under the rig's trim; at rest, the identity,
	// and on the way there the eased rotation the source carries. A released
	// head is left out.
	m_state.headDelta= glm::quat(1.f, 0.f, 0.f, 0.f);
	if (source.head.state != eGroupState::Lost)
	{
		glm::quat headDelta(1.f, 0.f, 0.f, 0.f);
		if (source.head.state != eGroupState::Resting || source.head.restBlend < 1.f)
			headDelta= limitRotationAngle(source.head.orientation, config.headLimitRadians);
		headDelta= headDelta * glm::inverse(glm::quat(config.headTrimRadians));
		placeBone(outPose, skeleton, B::Head, headDelta);
		m_state.headDelta= headDelta;
	}

	// The avatar's shoulders under the placed spine, which an arm pulled
	// toward a goal reaches from; the arms do not move them
	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posedTorso;
	bool bTorsoPosed= false;

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SourcePose::Arm& sourceArm= source.arms[sideIndex];
		const SourcePose::Hand& sourceHand= source.hands[sideIndex];
		if (sourceArm.state == eGroupState::Lost)
			continue;

		// Upper arm and forearm: swings from rest onto the chain's
		// directions; the forearm's roll and the hand as the source measured
		// them against the avatar's rest palm
		ArmPlacement placement;
		placement.upperDirection= sourceArm.upperDirection;
		placement.forearmDirection= sourceArm.forearmDirection;
		placement.twist= sourceArm.twist;
		placement.wristRotation= sourceHand.wristRotation;
		placement.fingers= sourceHand.fingers;
		const AvatarRetargetArm& armSettings= arms[sideIndex];
		if (outGoals != nullptr)
		{
			outGoals->bPosed[sideIndex]= true;
			outGoals->ikWeight[sideIndex]= armSettings.ikWeight > 0.f ? armSettings.ikWeight : 0.f;
		}
		if (armSettings.ikWeight > 0.f)
		{
			if (!bTorsoPosed)
			{
				computePosedBones(skeleton, outPose, posedTorso);
				bTorsoPosed= true;
			}
			pullPlacementTowardGoal(source, skeleton, sideIndex, spineDelta,
									posedTorso[(int)sideBone(sideIndex, B::LeftUpperArm, B::RightUpperArm)].positionWorld,
									armSettings, config.sides[sideIndex], placement, outGoals);
		}
		placeArm(skeleton, sideIndex, spineDelta, placement, config.sides[sideIndex],
				 config.wristLimitRadians[sideIndex], outPose);
	}
}

void DirectRetarget::makeRestPose(const AvatarSkeleton& skeleton, const DirectRetargetConfig& config,
								  AvatarPose& outPose)
{
	outPose.clear();
	outPose.valid= true;
	const glm::quat identity(1.f, 0.f, 0.f, 0.f);
	placeBone(outPose, skeleton, B::Hips, identity);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		placeArm(skeleton, sideIndex, identity,
				 makeRestPlacement(skeleton, sideIndex, identity, config.sides[sideIndex]), config.sides[sideIndex],
				 config.wristLimitRadians[sideIndex], outPose);
}

glm::mat4 DirectRetarget::makeCameraToWorld(const AvatarSkeleton& skeleton, const AvatarPose& pose,
											const SolveState& state)
{
	const glm::mat4 rotation= glm::mat4_cast(worldFromCamera(state.tiltRadians));
	if (!state.torsoValid || !pose.valid)
		return rotation;
	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
	computePosedBones(skeleton, pose, posed);
	const glm::vec3 posedMidpoint=
		0.5f * (posed[(int)B::LeftUpperArm].positionWorld + posed[(int)B::RightUpperArm].positionWorld);
	return glm::translate(glm::mat4(1.f), posedMidpoint - state.shoulderMidpoint) * rotation;
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
