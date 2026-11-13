#include "WholeBodyFaceRatios.h"

#include <algorithm>

#include "glm/ext/vector_float2.hpp"
#include "glm/geometric.hpp"

namespace
{
// iBUG 68 indices. Sides are the person's own.
constexpr int kRightBrowFirst= 17;
constexpr int kRightBrowMid= 19;
constexpr int kRightBrowInner= 21;
constexpr int kLeftBrowInner= 22;
constexpr int kLeftBrowMid= 24;
constexpr int kLeftBrowLast= 26;
constexpr int kRightEyeOuter= 36;
constexpr int kRightEyeInner= 39;
constexpr int kLeftEyeInner= 42;
constexpr int kLeftEyeOuter= 45;
constexpr int kMouthRight= 48;
constexpr int kOuterLipTop= 51;
constexpr int kMouthLeft= 54;
constexpr int kOuterLipBottom= 57;
constexpr int kInnerLipTop= 62;
constexpr int kInnerLipBottom= 66;

// The six points of an eye, in the order the aspect ratio reads them: the two
// corners, then the two upper lid points each with the lower lid point under
// it
struct EyePoints
{
	int cornerA;
	int cornerB;
	int upperA;
	int lowerA;
	int upperB;
	int lowerB;
};
constexpr EyePoints kEyes[2]= {
	{42, 45, 43, 47, 44, 46}, // left
	{36, 39, 37, 41, 38, 40}, // right
};
} // namespace

WholeBodyFaceRatios computeWholeBodyFaceRatios(const WholeBodyExtras& extras)
{
	WholeBodyFaceRatios ratios;
	if (!extras.present)
		return ratios;

	auto point= [&](int index) { return glm::vec2(extras.face[index]); };

	const glm::vec2 eyeLine= point(kLeftEyeOuter) - point(kRightEyeOuter);
	const float faceWidth= glm::length(eyeLine);
	if (faceWidth < 1e-3f)
		return ratios;
	const glm::vec2 leftAxis= eyeLine / faceWidth;

	// The up axis is the eye line's perpendicular on the side away from the
	// mouth, whichever way the image's y runs
	const glm::vec2 eyeCenter= (point(kLeftEyeOuter) + point(kRightEyeOuter)) * 0.5f;
	const glm::vec2 lipCenter= (point(kOuterLipTop) + point(kOuterLipBottom)) * 0.5f;
	glm::vec2 upAxis(-leftAxis.y, leftAxis.x);
	if (glm::dot(upAxis, eyeCenter - lipCenter) < 0.f)
		upAxis= -upAxis;
	auto heightAbove= [&](const glm::vec2& p, const glm::vec2& reference) {
		return glm::dot(p - reference, upAxis) / faceWidth;
	};

	ratios.valid= true;
	ratios.faceWidthPx= faceWidth;

	float minScore= 1.f;
	auto read= [&](int index) { minScore= std::min(minScore, extras.faceScores[index]); };

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const EyePoints& eye= kEyes[sideIndex];
		const float width= glm::length(point(eye.cornerA) - point(eye.cornerB));
		const float gap= glm::length(point(eye.upperA) - point(eye.lowerA)) +
			glm::length(point(eye.upperB) - point(eye.lowerB));
		ratios.eyeOpenness[sideIndex]= width > 1e-3f ? gap / (2.f * width) : 0.f;
		for (int index : {eye.cornerA, eye.cornerB, eye.upperA, eye.lowerA, eye.upperB, eye.lowerB})
			read(index);
	}

	ratios.mouthOpen= glm::length(point(kInnerLipTop) - point(kInnerLipBottom)) / faceWidth;
	ratios.mouthWidth= glm::length(point(kMouthLeft) - point(kMouthRight)) / faceWidth;
	ratios.mouthCornerLift[0]= heightAbove(point(kMouthLeft), lipCenter);
	ratios.mouthCornerLift[1]= heightAbove(point(kMouthRight), lipCenter);
	for (int index : {kInnerLipTop, kInnerLipBottom, kMouthLeft, kMouthRight, kOuterLipTop, kOuterLipBottom})
		read(index);

	const glm::vec2 eyeCenters[2]= {
		(point(kLeftEyeInner) + point(kLeftEyeOuter)) * 0.5f,
		(point(kRightEyeInner) + point(kRightEyeOuter)) * 0.5f,
	};
	ratios.browHeight[0]= heightAbove(point(kLeftBrowMid), eyeCenters[0]);
	ratios.browHeight[1]= heightAbove(point(kRightBrowMid), eyeCenters[1]);
	ratios.browInnerHeight= 0.5f * (heightAbove(point(kLeftBrowInner), point(kLeftEyeInner)) +
									heightAbove(point(kRightBrowInner), point(kRightEyeInner)));
	for (int index= kRightBrowFirst; index <= kLeftBrowLast; ++index)
		read(index);

	ratios.minScore= minScore;
	return ratios;
}
