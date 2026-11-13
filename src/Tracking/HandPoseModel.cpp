#include "HandPoseModel.h"

#include <algorithm>
#include <cmath>

#include "glm/ext/vector_float2.hpp"
#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/quaternion.hpp"

namespace
{
// The thumb rests pronated (twisted about its own axis) relative to the
// fingers, so its MCP/IP flexion sweeps ACROSS the palm toward the pinky
// instead of curling toward the palm plane. Its flexion hinge is the
// finger-style hinge rotated about the thumb bone by this fixed anatomical
// offset - applied identically in extraction and FK so the 4-angle schema
// round-trips exactly. Without it, across-palm thumb flexion projects to
// nearly nothing on the finger-style hinge (thumb opposition was lost).
constexpr float kThumbPronationRad= 1.2f; // ~69 degrees

// Signed angle from vector a to vector b about the given axis (all normalized)
float signedAngle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& axis)
{
	const float cosAngle= std::clamp(glm::dot(a, b), -1.f, 1.f);
	const float sinAngle= glm::dot(glm::cross(a, b), axis);
	return atan2f(sinAngle, cosAngle);
}

glm::vec3 safeNormalize(const glm::vec3& v)
{
	const float length= glm::length(v);
	return length > 1e-6f ? v / length : glm::vec3(1.f, 0.f, 0.f);
}

// Thumb flexion hinge: the standard hinge pronated about the (post-bend)
// thumb metacarpal direction. chiralitySign: +1 when the thumb sits on the
// palm frame's +Y side, which is a LEFT hand (palm +Y runs toward the thumb
// on a left hand and toward the pinky on a right), -1 for a right hand.
glm::vec3 pronatedThumbHinge(const glm::vec3& standardHinge, const glm::vec3& boneDirection, float chiralitySign)
{
	const glm::quat pronation= glm::angleAxis(chiralitySign * kThumbPronationRad, safeNormalize(boneDirection));
	return pronation * standardHinge;
}

// The same with the signed pronation angle given outright
glm::vec3 pronatedThumbHingeByAngle(const glm::vec3& standardHinge, const glm::vec3& boneDirection, float pronationRad)
{
	return glm::angleAxis(pronationRad, safeNormalize(boneDirection)) * standardHinge;
}
} // namespace

glm::mat4 HandPoseModel::computePalmFrame(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& points,
										  eHandSide side, PalmarSideMemory* ioMemory, bool bSideDecided)
{
	const glm::vec3& wrist= points[(int)eHandLandmark::WRIST];
	const glm::vec3& indexMcp= points[(int)eHandLandmark::INDEX_MCP];
	const glm::vec3& middleMcp= points[(int)eHandLandmark::MIDDLE_MCP];
	const glm::vec3& pinkyMcp= points[(int)eHandLandmark::PINKY_MCP];

	// +X toward the fingers
	const glm::vec3 xAxis= safeNormalize(middleMcp - wrist);

	// Palm plane normal, sign undetermined
	const glm::vec3 normal= safeNormalize(glm::cross(indexMcp - wrist, pinkyMcp - wrist));

	// Which sign is the PALMAR side? Do NOT trust the handedness label for
	// this: MediaPipe's classifier is view-dependent (a right hand seen from
	// the back looks like a left hand seen from the palm), so the label
	// routinely flips when the palm rotates away from the camera - which
	// would mirror every extracted angle. Disambiguate geometrically instead,
	// from two anatomical invariants:
	//  (1) flexed finger joints rotate about their hinge toward the palmar
	//      side. The joint ROTATION AXIS (cross of successive bones) stays
	//      aligned with the finger hinge at any curl DEPTH - unlike bone tilt,
	//      which reverses past 180 degrees of total curl (a fist). It needs
	//      actual flexion though: fingers hyperextend ~10-20 degrees, so a
	//      flat hand rotates its joints the WRONG way by a small amount, and
	//      the sign of a small sum means nothing. Measured over recording
	//      2026-08-14_19-29-22: with |curlEvidence| under 0.5 the sign is
	//      right on 39-44% of frames (worse than a coin), and at 1.0 or more
	//      on 86-92%. Only the strong band may argue with continuity.
	//  (2) the thumb metacarpal sits palmar of the wrist-index-pinky plane.
	//      Independent of flexion, so this is the one that still means
	//      something on a flat hand (right 76%, left 93% in the same band) -
	//      but it is weak, a tenth of curl's magnitude, so it seeds a new
	//      hand rather than overturning one already being tracked.
	float curlEvidence= 0.f;
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		const int* joints= FINGER_JOINTS[finger];

		// Hinge reference for "curl toward n": metacarpal is always in-plane,
		// so this never degenerates however deep the curl is
		const glm::vec3 metacarpal= points[joints[0]] - wrist;
		const glm::vec3 metaInPlane= safeNormalize(metacarpal - normal * glm::dot(metacarpal, normal));
		const glm::vec3 hingeRef= glm::cross(metaInPlane, normal);

		const glm::vec3 bone0= safeNormalize(points[joints[1]] - points[joints[0]]);
		const glm::vec3 bone1= safeNormalize(points[joints[2]] - points[joints[1]]);
		const glm::vec3 bone2= safeNormalize(points[joints[3]] - points[joints[2]]);
		curlEvidence+= glm::dot(glm::cross(bone0, bone1), hingeRef);
		curlEvidence+= glm::dot(glm::cross(bone1, bone2), hingeRef);
	}
	const float thumbEvidence=
		glm::dot(safeNormalize(points[(int)eHandLandmark::THUMB_MCP] - wrist), normal);

	const float palmarScore= curlEvidence + 0.5f * thumbEvidence;

	// A hand's palmar side cannot invert between frames, so CONTINUITY is a
	// far stronger prior than weak geometry - and much stronger than the
	// handedness label, which flips whenever the palm turns away from a
	// camera. Deciding this independently every frame is what let the left
	// palm frame flip mid-capture and poison the mounting average.
	// Curl strong enough to be worth 86-92% rather than a coin flip; only
	// this may contradict the remembered side
	constexpr float kCurlOverride= 1.0f;
	constexpr float kWeakEvidence= 0.05f; // better than nothing when new
	// Decisive evidence AGAINST the remembered side has to persist. Turning a
	// hand over rotates the remembered normal with it, so following real
	// motion never needs continuity broken - a contradiction is a bad
	// reconstruction until proven otherwise. Sustained contradiction still
	// wins, so a memory that started out wrong recovers within a few frames.
	constexpr int kFlipEvidenceCount= 5;

	const bool bRemembered= ioMemory != nullptr && glm::dot(ioMemory->palmarNormal, ioMemory->palmarNormal) > 0.25f;
	const float rememberedSign=
		bRemembered ? (glm::dot(normal, ioMemory->palmarNormal) >= 0.f ? 1.f : -1.f) : 0.f;
	const float evidenceSign= palmarScore > 0.f ? 1.f : -1.f;

	bool bContradicting= false;
	float palmarSign;
	if (bSideDecided)
	{
		// The side is known, so the palmar side is anatomy and no evidence
		// or memory is asked. With the index knuckle toward the thumb, the
		// cross of wrist-to-index with wrist-to-pinky points out of the back
		// of a left hand and out of a right hand's palm, in a right-handed
		// frame, which the camera's and the hand model's both are.
		palmarSign= side == eHandSide::Left ? -1.f : 1.f;
	}
	else if (!bRemembered)
	{
		// Nothing to be continuous with, so take the best signal available:
		// curl when the hand is properly flexed, otherwise the thumb, which
		// is the only one that survives a flat hand
		if (fabsf(curlEvidence) >= kCurlOverride)
		{
			palmarSign= curlEvidence > 0.f ? 1.f : -1.f;
		}
		else if (fabsf(thumbEvidence) > kWeakEvidence)
		{
			palmarSign= thumbEvidence > 0.f ? 1.f : -1.f;
		}
		else
		{
			// No usable geometry at all (flat hand, thumb in-plane): the
			// label is the last resort. For a RIGHT hand the raw cross points
			// out of the BACK of the hand; for a LEFT hand out of the palm.
			palmarSign= side == eHandSide::Right ? -1.f : 1.f;
		}
	}
	else if (fabsf(curlEvidence) >= kCurlOverride && evidenceSign != rememberedSign)
	{
		bContradicting= true;
		if (palmarScore != ioMemory->countedScore)
		{
			ioMemory->contradictionCount++;
			ioMemory->countedScore= palmarScore;
		}
		palmarSign= ioMemory->contradictionCount >= kFlipEvidenceCount ? evidenceSign : rememberedSign;
	}
	else
	{
		// Agreeing, or too weak to argue with what we already know
		palmarSign= rememberedSign;
	}

	if (ioMemory != nullptr)
	{
		// The streak has to be CONSECUTIVE: one agreeing or inconclusive
		// observation spends it, and so does actually flipping
		if (!bContradicting || palmarSign != rememberedSign)
		{
			ioMemory->contradictionCount= 0;
			ioMemory->countedScore= 0.f;
		}
		ioMemory->palmarNormal= normal * palmarSign;
	}

	// Orthonormalize: Z out of the palmar surface, Y completes right-handed
	glm::vec3 zAxis= safeNormalize(normal * palmarSign - xAxis * glm::dot(normal * palmarSign, xAxis));
	const glm::vec3 yAxis= glm::cross(zAxis, xAxis);

	const glm::vec3 palmCenter= (wrist + middleMcp) * 0.5f;

	glm::mat4 frame(1.f);
	frame[0]= glm::vec4(xAxis, 0.f);
	frame[1]= glm::vec4(yAxis, 0.f);
	frame[2]= glm::vec4(zAxis, 0.f);
	frame[3]= glm::vec4(palmCenter, 1.f);
	return frame;
}

HandPoseModel::NeutralDirections HandPoseModel::makeDefaultNeutralDirections(const HandSkeleton& skeleton)
{
	NeutralDirections neutralDirs;

	// A flat hand holds its four fingers parallel to the middle metacarpal,
	// which IS palm +X by construction - so that is the honest zero for them.
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
		neutralDirs[finger]= glm::vec3(1.f, 0.f, 0.f);

	// The thumb rests well off that axis, so its own metacarpal (wrist ->
	// thumb CMC, in the palm plane) is the better zero
	const glm::vec3& middleBase= skeleton.baseInPalm[(int)eFinger::Middle];
	const glm::vec3 wristInPalm(-middleBase.x, 0.f, 0.f);
	const glm::vec3 thumbMeta= skeleton.baseInPalm[(int)eFinger::Thumb] - wristInPalm;
	neutralDirs[(int)eFinger::Thumb]= safeNormalize(glm::vec3(thumbMeta.x, thumbMeta.y, 0.f));

	return neutralDirs;
}

namespace
{
// The angle extraction over a palm frame already built and each finger's
// rest direction already in the points' space (unit, any tilt out of the
// palm plane allowed): the shared body of the two public conventions
void computeFingerAnglesAgainst(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& points,
								const glm::mat4& palmFrame,
								const std::array<glm::vec3, FINGER_COUNT>& neutralWorlds,
								std::array<FingerAngles, FINGER_COUNT>& outAngles)
{
	const glm::vec3 palmY= glm::vec3(palmFrame[1]);
	const glm::vec3 palmZ= glm::vec3(palmFrame[2]);
	const glm::vec3& wrist= points[(int)eHandLandmark::WRIST];

	// The thumb's anatomical pronation direction is opposite between hands;
	// detect it GEOMETRICALLY (which side of the palm frame the index sits on)
	// rather than trusting the handedness label, which is view-dependent
	const bool bThumbOnMinusY=
		glm::dot(points[(int)eHandLandmark::INDEX_MCP] - wrist, palmY) < 0.f;

	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		const int* joints= FINGER_JOINTS[finger];
		const glm::vec3& base= points[joints[0]];

		// Rest direction for this finger in the points' space. Its palm-plane
		// projection is the zero for lateral; the full direction is the zero
		// for proximal.
		const glm::vec3& neutralWorld= neutralWorlds[finger];
		const glm::vec3 neutralInPlane=
			safeNormalize(neutralWorld - palmZ * glm::dot(neutralWorld, palmZ));

		const glm::vec3 proximalBone= safeNormalize(points[joints[1]] - base);
		const glm::vec3 intermediateBone= safeNormalize(points[joints[2]] - points[joints[1]]);
		const glm::vec3 distalBone= safeNormalize(points[joints[3]] - points[joints[2]]);

		// Lateral: signed splay about palm +Z, positive counter-clockwise
		// about +Z (toward palm +Y), from two readings of the same angle
		// blended by how well each is conditioned. The first is the proximal
		// bone's palm-plane projection against the rest direction, sharp on
		// a straight finger and gone on one curled to the palm normal, where
		// the projection is landmark noise. The second is the finger's own
		// bend plane: the intermediate bone turns about the lateral hinge,
		// so the normal of the proximal and intermediate bones is that hinge
		// (negated for a curl), sharp on a curled finger and gone on a
		// straight one. Each reading's weight is the square of the length
		// that made it (the projection's, the cross product's), the inverse
		// of its angular variance, and the blend is the vector mean. The
		// thumb bends about a pronated hinge, so it reads the projection
		// alone. Either reading pointing back toward the wrist is a curl past
		// 90 degrees (a fist seen from the front), never a splay, so it folds
		// by a half turn to stay within a quarter turn: the hinge stays on
		// the finger's side and the proximal bend reads past 90 instead of
		// the later joints reading negative about a reversed hinge.
		const glm::vec3 hingeAtZero= safeNormalize(glm::cross(neutralInPlane, -palmZ));
		auto foldToQuarterTurn= [](float angle) {
			if (angle > glm::half_pi<float>())
				return angle - glm::pi<float>();
			if (angle < -glm::half_pi<float>())
				return angle + glm::pi<float>();
			return angle;
		};
		const glm::vec3 proximalInPlaneRaw= proximalBone - palmZ * glm::dot(proximalBone, palmZ);
		const float projectionLength= glm::length(proximalInPlaneRaw);
		const float lateralFromProjection=
			foldToQuarterTurn(signedAngle(neutralInPlane, safeNormalize(proximalInPlaneRaw), palmZ));
		glm::vec2 lateralVector= glm::vec2(std::cos(lateralFromProjection), std::sin(lateralFromProjection)) *
			(projectionLength * projectionLength);
		if (finger != (int)eFinger::Thumb)
		{
			const glm::vec3 bendNormalRaw= -glm::cross(proximalBone, intermediateBone);
			const glm::vec3 hingeInPlaneRaw= bendNormalRaw - palmZ * glm::dot(bendNormalRaw, palmZ);
			const float hingeLength= glm::length(hingeInPlaneRaw);
			const float lateralFromBendPlane=
				foldToQuarterTurn(signedAngle(hingeAtZero, safeNormalize(hingeInPlaneRaw), palmZ));
			lateralVector+= glm::vec2(std::cos(lateralFromBendPlane), std::sin(lateralFromBendPlane)) *
				(hingeLength * hingeLength);
		}
		const float lateral= glm::length(lateralVector) > 1e-8f ? std::atan2(lateralVector.y, lateralVector.x) : 0.f;

		// ONE fixed hinge axis per finger, exactly as the FK side builds it:
		// from the post-lateral direction. All three bend angles are measured
		// as signed rotations about this axis - measuring each joint's sign
		// against cross(bone, palmZ) (the old approach) breaks down when a
		// curled bone points along the palm normal and that cross degenerates,
		// which flipped distal signs mid-curl (Z-shaped fingers).
		const glm::quat lateralRotation= glm::angleAxis(lateral, palmZ);
		const glm::vec3 directionLat= lateralRotation * neutralWorld;
		const glm::vec3 hingeAxis= safeNormalize(glm::cross(directionLat, -palmZ));

		// FK rotates by angleAxis(-bend, hinge), so the extracted bend is the
		// NEGATED signed angle about the hinge - which makes positive bend
		// curl toward the palmar side (+Z). The thumb's MCP/IP flexion hinge
		// is pronated about the metacarpal (see kThumbPronationRad);
		// lateral/proximal stay on the standard hinge (together they
		// spherically parameterize the proximal bone direction).
		const glm::vec3 flexHinge=
			finger == (int)eFinger::Thumb
				? pronatedThumbHinge(hingeAxis, proximalBone, bThumbOnMinusY ? -1.f : 1.f)
				: hingeAxis;

		outAngles[finger].lateral= lateral;
		outAngles[finger].proximal= -signedAngle(directionLat, proximalBone, hingeAxis);
		outAngles[finger].intermediate= -signedAngle(proximalBone, intermediateBone, flexHinge);
		outAngles[finger].distal= -signedAngle(intermediateBone, distalBone, flexHinge);
	}
}
} // namespace

void HandPoseModel::computeFingerAngles(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& points, eHandSide side,
										const NeutralDirections& neutralDirs,
										std::array<FingerAngles, FINGER_COUNT>& outAngles,
										PalmarSideMemory* ioMemory, bool bSideDecided)
{
	const glm::mat4 palmFrame= computePalmFrame(points, side, ioMemory, bSideDecided);
	const glm::mat3 palmRotation= glm::mat3(palmFrame);
	std::array<glm::vec3, FINGER_COUNT> neutralWorlds;
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
		neutralWorlds[finger]= safeNormalize(palmRotation * neutralDirs[finger]);
	computeFingerAnglesAgainst(points, palmFrame, neutralWorlds, outAngles);
}

void HandPoseModel::computeMetacarpalFingerAngles(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& points,
												  eHandSide side, std::array<FingerAngles, FINGER_COUNT>& outAngles,
												  PalmarSideMemory* ioMemory)
{
	const glm::mat4 palmFrame= computePalmFrame(points, side, ioMemory);
	const glm::vec3 palmZ= glm::vec3(palmFrame[2]);
	const glm::vec3& wrist= points[(int)eHandLandmark::WRIST];
	std::array<glm::vec3, FINGER_COUNT> neutralWorlds;
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		const glm::vec3 metacarpal= points[FINGER_JOINTS[finger][0]] - wrist;
		neutralWorlds[finger]= safeNormalize(metacarpal - palmZ * glm::dot(metacarpal, palmZ));
	}
	computeFingerAnglesAgainst(points, palmFrame, neutralWorlds, outAngles);
}

void HandPoseModel::computeSkeleton(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& points, eHandSide side,
									HandSkeleton& outSkeleton, PalmarSideMemory* ioMemory, bool bSideDecided)
{
	const glm::mat4 palmFrame= computePalmFrame(points, side, ioMemory, bSideDecided);
	const glm::mat4 palmInverse= glm::inverse(palmFrame);

	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		const int* joints= FINGER_JOINTS[finger];

		outSkeleton.baseInPalm[finger]= glm::vec3(palmInverse * glm::vec4(points[joints[0]], 1.f));
		for (int phalanx= 0; phalanx < 3; ++phalanx)
		{
			outSkeleton.phalanxLengths[finger][phalanx]=
				glm::length(points[joints[phalanx + 1]] - points[joints[phalanx]]);
		}
	}

	outSkeleton.neutralDirInPalm= makeDefaultNeutralDirections(outSkeleton);
}

void HandPoseModel::buildFingerJoints(const glm::mat4& palmTransform, const HandSkeleton& skeleton,
									  const std::array<FingerAngles, FINGER_COUNT>& angles,
									  std::array<std::array<glm::vec3, 4>, FINGER_COUNT>& outJoints,
									  const float* thumbPronationRad)
{
	const float pronationRad= thumbPronationRad != nullptr ? *thumbPronationRad : getThumbPronationRad(skeleton);
	const glm::vec3 palmZLocal(0.f, 0.f, 1.f);

	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		const FingerAngles& fingerAngles= angles[finger];
		const glm::vec3& base= skeleton.baseInPalm[finger];

		// Rest direction for this finger (the pose all-zero angles rebuild).
		// Lateral rotates the FULL rest direction about palm +Z, so the
		// out-of-plane part of a captured rest pose is preserved.
		const glm::vec3 neutralDir= safeNormalize(skeleton.neutralDirInPalm[finger]);

		const glm::quat lateralRotation= glm::angleAxis(fingerAngles.lateral, palmZLocal);
		glm::vec3 direction= lateralRotation * neutralDir;

		const glm::vec3 hingeAxis= safeNormalize(glm::cross(direction, -palmZLocal));
		const glm::quat proximalRotation= glm::angleAxis(-fingerAngles.proximal, hingeAxis);
		direction= proximalRotation * direction;

		// Thumb MCP/IP flexion happens about the pronated hinge (mirrors
		// computeFingerAngles); chirality from the skeleton's index y sign,
		// the FK-side equivalent of that function's bThumbOnMinusY test,
		// unless the caller gave the angle
		const glm::vec3 flexHinge=
			finger == (int)eFinger::Thumb ? pronatedThumbHingeByAngle(hingeAxis, direction, pronationRad) : hingeAxis;

		std::array<glm::vec3, 4>& joints= outJoints[finger];
		joints[0]= glm::vec3(palmTransform * glm::vec4(base, 1.f));

		glm::vec3 position= base + direction * skeleton.phalanxLengths[finger][0];
		joints[1]= glm::vec3(palmTransform * glm::vec4(position, 1.f));

		const glm::quat intermediateRotation= glm::angleAxis(-fingerAngles.intermediate, flexHinge);
		direction= intermediateRotation * direction;
		position+= direction * skeleton.phalanxLengths[finger][1];
		joints[2]= glm::vec3(palmTransform * glm::vec4(position, 1.f));

		const glm::quat distalRotation= glm::angleAxis(-fingerAngles.distal, flexHinge);
		direction= distalRotation * direction;
		position+= direction * skeleton.phalanxLengths[finger][2];
		joints[3]= glm::vec3(palmTransform * glm::vec4(position, 1.f));
	}
}

float HandPoseModel::getThumbPronationRad(const HandSkeleton& skeleton)
{
	return (skeleton.baseInPalm[(int)eFinger::Index].y < 0.f ? -1.f : 1.f) * kThumbPronationRad;
}
