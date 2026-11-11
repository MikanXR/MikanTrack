#include "SourceSkeleton.h"

#include <algorithm>
#include <cmath>

#include "glm/geometric.hpp"
#include "glm/gtc/quaternion.hpp"

#include "HandPoseModel.h"
#include "LossStage.h" // LossHands
#include "Scene3dPanel.h" // projectToViewImage

namespace
{
using eGroupState= SourcePose::eGroupState;

// The line of a group whose input carries no confidence
const glm::vec3 k_neutralColor(0.82f, 0.85f, 0.9f);
const glm::vec3 k_highConfidenceColor(0.35f, 0.9f, 0.4f);
const glm::vec3 k_lowConfidenceColor(1.f, 0.7f, 0.15f);
// The head's facing and up lines, meters
constexpr float kHeadFacingLength= 0.08f;
constexpr float kHeadUpLength= 0.05f;

eRetargetChain armChain(int sideIndex)
{
	return sideIndex == 0 ? eRetargetChain::LeftArm : eRetargetChain::RightArm;
}

eRetargetChain handChain(int sideIndex)
{
	return sideIndex == 0 ? eRetargetChain::LeftHand : eRetargetChain::RightHand;
}

float markerSize(eGroupState state)
{
	return isSourceGroupDashed(state) ? SourceSkeletonStyle::kCarriedMarkerSize
									  : SourceSkeletonStyle::kMeasuredMarkerSize;
}

void addSegment(SourceSkeleton& skeleton, const glm::vec3& start, const glm::vec3& end, eRetargetChain chain,
				eGroupState state, const glm::vec3& color)
{
	SourceSkeleton::Segment segment;
	segment.start= start;
	segment.end= end;
	segment.chain= chain;
	segment.bDashed= isSourceGroupDashed(state);
	segment.color= color;
	skeleton.segments.push_back(segment);
}

void addMarker(SourceSkeleton& skeleton, const glm::vec3& position, eRetargetChain chain, eGroupState state,
			   const glm::vec3& color)
{
	SourceSkeleton::Marker marker;
	marker.position= position;
	marker.chain= chain;
	marker.color= color;
	marker.size= markerSize(state);
	skeleton.markers.push_back(marker);
}

// One arm's chain: the joints it carries joined in order, the segment from
// the shoulder colored by the shoulder's confidence and the rest by the
// forearm's
void addArm(SourceSkeleton& skeleton, const SourcePose::Arm& arm, int sideIndex, const glm::mat4& toWorld)
{
	const eRetargetChain chain= armChain(sideIndex);
	const glm::vec3 shoulderColor= getSourceConfidenceColor(arm.shoulderConfidence);
	const glm::vec3 forearmColor= getSourceConfidenceColor(arm.forearmConfidence);
	auto place= [&](const glm::vec3& point) { return glm::vec3(toWorld * glm::vec4(point, 1.f)); };

	const glm::vec3 wrist= place(arm.wristPosition);
	glm::vec3 previous= wrist;
	bool bPrevious= false;
	if (arm.bHasShoulder)
	{
		previous= place(arm.shoulderPosition);
		bPrevious= true;
		addMarker(skeleton, previous, chain, arm.state, shoulderColor);
	}
	if (arm.bHasElbow)
	{
		const glm::vec3 elbow= place(arm.elbowPosition);
		if (bPrevious)
			addSegment(skeleton, previous, elbow, chain, arm.state, shoulderColor);
		addMarker(skeleton, elbow, chain, arm.state, forearmColor);
		previous= elbow;
		bPrevious= true;
	}
	if (bPrevious)
		addSegment(skeleton, previous, wrist, chain, arm.state, forearmColor);
	addMarker(skeleton, wrist, chain, arm.state, forearmColor);
}

// One hand from its resolved pose, through the forward kinematics the
// stream's finger bones are built from: the wrist to each finger's base,
// then each finger's chain
void addHand(SourceSkeleton& skeleton, const HandPose& pose, const SourcePose::Hand& hand, int sideIndex)
{
	const eRetargetChain chain= handChain(sideIndex);
	const glm::vec3 color= getSourceConfidenceColor(hand.confidence);

	glm::mat4 palmTransform= glm::mat4_cast(pose.palmOrientationWorld);
	palmTransform[3]= glm::vec4(pose.palmPositionWorld, 1.f);
	const glm::vec3& middleBase= pose.skeleton.baseInPalm[(int)eFinger::Middle];
	const glm::vec3 wristWorld= glm::vec3(palmTransform * glm::vec4(-middleBase.x, 0.f, 0.f, 1.f));

	std::array<std::array<glm::vec3, 4>, FINGER_COUNT> joints;
	HandPoseModel::buildFingerJoints(palmTransform, pose.skeleton, pose.fingers, joints);
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		addSegment(skeleton, wristWorld, joints[finger][0], chain, hand.state, color);
		for (int joint= 0; joint < 3; ++joint)
			addSegment(skeleton, joints[finger][joint], joints[finger][joint + 1], chain, hand.state, color);
	}
}
} // namespace

void SourceSkeleton::clear()
{
	segments.clear();
	markers.clear();
}

bool isSourceGroupDrawn(SourcePose::eGroupState state)
{
	return state != eGroupState::Lost;
}

bool isSourceGroupDashed(SourcePose::eGroupState state)
{
	return state == eGroupState::Held || state == eGroupState::Frozen || state == eGroupState::Resting;
}

glm::vec3 getSourceConfidenceColor(const SourcePose::Confidence& confidence)
{
	if (!confidence.bPresent)
		return k_neutralColor;
	const float t= std::clamp(confidence.value, 0.f, 1.f);
	return k_lowConfidenceColor + (k_highConfidenceColor - k_lowConfidenceColor) * t;
}

void buildSourceSkeleton(const SourceSkeletonInputs& inputs, SourceSkeleton& outSkeleton)
{
	outSkeleton.clear();
	if (inputs.source == nullptr)
		return;
	const SourcePose& source= *inputs.source;
	const bool bPosition= source.pipeline == SourcePose::ePipeline::Position;
	const bool bDirect= source.pipeline == SourcePose::ePipeline::Direct;
	if (!bPosition && !(bDirect && inputs.directToWorld != nullptr))
		return;
	const glm::mat4 toWorld= bDirect ? *inputs.directToWorld : glm::mat4(1.f);

	// The arms that carry positions: a resting arm carries its placement
	// alone, so it has nothing to draw
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SourcePose::Arm& arm= source.arms[sideIndex];
		if (isSourceGroupDrawn(arm.state) && arm.state != eGroupState::Resting)
			addArm(outSkeleton, arm, sideIndex, toWorld);
	}

	// The hands, from the poses the loss stage resolved beside the source
	if (bPosition && inputs.hands != nullptr)
	{
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const SourcePose::Hand& hand= source.hands[sideIndex];
			const HandPose& pose= inputs.hands->poses[sideIndex];
			if (isSourceGroupDrawn(hand.state) && inputs.hands->bCarried[sideIndex] && pose.hasWorldPose)
				addHand(outSkeleton, pose, hand, sideIndex);
		}
	}

	// The torso: the shoulder line, and the neck to the head where it is
	// drawn
	const SourcePose::Arm& left= source.arms[0];
	const SourcePose::Arm& right= source.arms[1];
	const bool bShoulderLine= isSourceGroupDrawn(source.torso.state) && left.bHasShoulder && right.bHasShoulder &&
		isSourceGroupDrawn(left.state) && isSourceGroupDrawn(right.state) && left.state != eGroupState::Resting &&
		right.state != eGroupState::Resting;
	glm::vec3 shoulderMidpoint(0.f);
	if (bShoulderLine)
	{
		const glm::vec3 leftShoulder= glm::vec3(toWorld * glm::vec4(left.shoulderPosition, 1.f));
		const glm::vec3 rightShoulder= glm::vec3(toWorld * glm::vec4(right.shoulderPosition, 1.f));
		shoulderMidpoint= (leftShoulder + rightShoulder) * 0.5f;
		addSegment(outSkeleton, leftShoulder, rightShoulder, eRetargetChain::Torso, source.torso.state, k_neutralColor);
	}

	// The head: its marker at the anchor, and in the position pipeline,
	// whose head is a world frame (+X facing, +Z up), its facing and up
	if (isSourceGroupDrawn(source.head.state) && inputs.headAnchorWorld != nullptr)
	{
		const glm::vec3& anchor= *inputs.headAnchorWorld;
		const glm::vec3 color= getSourceConfidenceColor(source.head.confidence);
		if (bShoulderLine)
			addSegment(outSkeleton, shoulderMidpoint, anchor, eRetargetChain::Torso, source.torso.state, k_neutralColor);
		addMarker(outSkeleton, anchor, eRetargetChain::Head, source.head.state, color);
		if (bPosition)
		{
			addSegment(outSkeleton, anchor, anchor + source.head.orientation * glm::vec3(kHeadFacingLength, 0.f, 0.f),
					   eRetargetChain::Head, source.head.state, color);
			addSegment(outSkeleton, anchor, anchor + source.head.orientation * glm::vec3(0.f, 0.f, kHeadUpLength),
					   eRetargetChain::Head, source.head.state, color);
		}
	}
}

void appendDashes(const glm::vec3& start, const glm::vec3& end, float dashLength, float gapLength,
				  std::vector<std::array<glm::vec3, 2>>& outDashes)
{
	const glm::vec3 delta= end - start;
	const float length= glm::length(delta);
	if (length <= dashLength || dashLength <= 0.f || gapLength < 0.f)
	{
		outDashes.push_back({start, end});
		return;
	}
	// Counted rather than accumulated, so rounding never starts a dash of no
	// length at the end
	const glm::vec3 direction= delta / length;
	const float period= dashLength + gapLength;
	const int dashCount= (int)std::ceil(length / period - 1e-4f);
	for (int dashIndex= 0; dashIndex < dashCount; ++dashIndex)
	{
		const float along= period * (float)dashIndex;
		const float dashEnd= std::min(along + dashLength, length);
		outDashes.push_back({start + direction * along, start + direction * dashEnd});
	}
}

float distanceToSegment2d(const glm::vec2& point, const glm::vec2& a, const glm::vec2& b)
{
	const glm::vec2 ab= b - a;
	const float lengthSquared= glm::dot(ab, ab);
	const float t= lengthSquared > 1e-12f ? std::clamp(glm::dot(point - a, ab) / lengthSquared, 0.f, 1.f) : 0.f;
	return glm::length(point - (a + ab * t));
}

bool pickSourceSkeleton(const SourceSkeleton& skeleton, const glm::mat4& viewProjection, const glm::ivec4& viewport,
						const glm::vec2& imageMin, float imageHeight, const glm::vec2& mouse, float tolerancePixels,
						eRetargetChain& outChain)
{
	float bestDistance= tolerancePixels;
	bool bFound= false;
	for (const SourceSkeleton::Segment& segment : skeleton.segments)
	{
		glm::vec2 start, end;
		if (!projectToViewImage(viewProjection, viewport, imageMin, imageHeight, segment.start, start) ||
			!projectToViewImage(viewProjection, viewport, imageMin, imageHeight, segment.end, end))
			continue;
		const float distance= distanceToSegment2d(mouse, start, end);
		if (distance <= bestDistance)
		{
			bestDistance= distance;
			outChain= segment.chain;
			bFound= true;
		}
	}
	for (const SourceSkeleton::Marker& marker : skeleton.markers)
	{
		glm::vec2 pixel;
		if (!projectToViewImage(viewProjection, viewport, imageMin, imageHeight, marker.position, pixel))
			continue;
		const float distance= glm::length(mouse - pixel);
		if (distance <= bestDistance)
		{
			bestDistance= distance;
			outChain= marker.chain;
			bFound= true;
		}
	}
	return bFound;
}
