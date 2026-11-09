#pragma once

#include "glm/ext/matrix_float4x4.hpp"
#include "opencv2/core/mat.hpp"

#include "DirectRetarget.h" // DirectCameraModel

class AvatarSkeleton;
struct AvatarPose;

// The posed avatar's skeleton over a camera frame: the one picture where the
// avatar and the landmarks it was solved from can be compared pixel for
// pixel. cameraToWorld maps OpenCV camera axes to the avatar's world, so a
// posed joint lands where the camera would see it.
namespace AvatarOverlay
{
// Projects a world point through the pinhole; false behind the camera
bool projectToPixel(const glm::vec3& world, const glm::mat4& cameraToWorld, const DirectCameraModel& camera,
					glm::vec2& outPixel);

// Draws the arms (shoulder, elbow, wrist, hand), the hands' finger chains
// and the head's neck-to-head segment, white with dark outlines, over the
// image in place
void drawPosedSkeleton(cv::Mat& image, const AvatarSkeleton& skeleton, const AvatarPose& pose,
					   const glm::mat4& cameraToWorld, const DirectCameraModel& camera);

// The frame and the avatar render side by side, the render scaled to the
// frame's height when the two differ
cv::Mat makeCompareImage(const cv::Mat& frame, const cv::Mat& avatarRender);
} // namespace AvatarOverlay
