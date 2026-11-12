#pragma once

#include <array>
#include <vector>

#include "glm/ext/matrix_float4x4.hpp"
#include "glm/ext/vector_float2.hpp"
#include "glm/ext/vector_float3.hpp"
#include "glm/ext/vector_int4.hpp"

#include "AvatarPanel.h" // eRetargetChain
#include "SourcePose.h"

struct LossHands;

// The source pose as the target solve receives it on a frame, laid out as
// world-space lines and markers for the scene, each tagged with the chain it
// belongs to so a click on it can select that chain. Pure geometry: building
// it, dashing it and hit testing it need no GL, and the scene draws it.
struct SourceSkeleton
{
	// A bone: solid while its group is measured, dashed while the group is
	// carried over (held, frozen or resting)
	struct Segment
	{
		glm::vec3 start{0.f};
		glm::vec3 end{0.f};
		eRetargetChain chain= eRetargetChain::Torso;
		bool bDashed= false;
		glm::vec3 color{1.f};
	};
	// A joint or the head: a point of a size in pixels
	struct Marker
	{
		glm::vec3 position{0.f};
		eRetargetChain chain= eRetargetChain::Torso;
		glm::vec3 color{1.f};
		float size= 7.f;
	};
	std::vector<Segment> segments;
	std::vector<Marker> markers;

	void clear();
};

// What the source skeleton is built from
struct SourceSkeletonInputs
{
	// The source pose after the loss stage; null builds nothing
	const SourcePose* source= nullptr;
	// The position pipeline's hands as the loss stage resolved them; null
	// draws no hands
	const LossHands* hands= nullptr;
	// The direct pipeline's positions live in the solve's world, the camera
	// at the origin: this carries them into the scene's. Null leaves the
	// direct arms and shoulder line out.
	const glm::mat4* directToWorld= nullptr;
	// Where the head is drawn, in the scene's world; the source pose carries
	// the head's orientation alone. Null leaves the head out.
	const glm::vec3* headAnchorWorld= nullptr;
};

// The source skeleton's style: a dash and the gap after it along a carried
// over bone, meters, and the marker sizes, pixels
namespace SourceSkeletonStyle
{
constexpr float kDashLength= 0.015f;
constexpr float kGapLength= 0.01f;
constexpr float kMeasuredMarkerSize= 7.f;
constexpr float kCarriedMarkerSize= 5.f;
} // namespace SourceSkeletonStyle

// Lays out the source skeleton. Position pipeline: each carried arm's
// shoulder, elbow and wrist joined as a chain (the joints it has), the hand
// through the finger forward kinematics from its resolved hand pose, the
// shoulder line and a neck line from its midpoint to the head while the
// torso is carried, and the head's marker with its facing and up lines.
// Direct pipeline: each carried arm's solved chain and the shoulder line,
// carried by directToWorld, and the head's marker. A lost group draws
// nothing, and so does a resting arm, which carries a placement and no
// positions.
void buildSourceSkeleton(const SourceSkeletonInputs& inputs, SourceSkeleton& outSkeleton);

// Whether a group in this state is drawn, and whether it is drawn dashed
bool isSourceGroupDrawn(SourcePose::eGroupState state);
bool isSourceGroupDashed(SourcePose::eGroupState state);

// Green at confidence 1 to amber at 0, or the neutral color for a group
// whose input carries no confidence
glm::vec3 getSourceConfidenceColor(const SourcePose::Confidence& confidence);

// The dashes along start to end: a dash, a gap, and so on from start, the
// last dash cut at end. A segment shorter than one dash is one dash.
void appendDashes(const glm::vec3& start, const glm::vec3& end, float dashLength, float gapLength,
				  std::vector<std::array<glm::vec3, 2>>& outDashes);

// The distance from point to the segment a to b, in the plane
float distanceToSegment2d(const glm::vec2& point, const glm::vec2& a, const glm::vec2& b);

// The chain of the segment or marker nearest the mouse within
// tolerancePixels, through the view-projection and the GL viewport the
// skeleton was drawn with, onto an image of imageHeight pixels whose top
// left is imageMin. A segment with an end behind the camera is skipped.
// False when nothing is that near.
bool pickSourceSkeleton(const SourceSkeleton& skeleton, const glm::mat4& viewProjection, const glm::ivec4& viewport,
						const glm::vec2& imageMin, float imageHeight, const glm::vec2& mouse, float tolerancePixels,
						eRetargetChain& outChain);
