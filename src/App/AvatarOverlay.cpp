#include "AvatarOverlay.h"

#include "opencv2/imgproc.hpp"

#include "AvatarRetarget.h"
#include "AvatarSkeleton.h"
#include "AvatarTypes.h"

namespace
{
using B= eHumanoidBone;

void drawBoneSegment(cv::Mat& image, const std::array<AvatarPosedBone, HUMANOID_BONE_COUNT>& posed,
					 const AvatarSkeleton& skeleton, const AvatarPose& pose, B from, B to,
					 const glm::mat4& cameraToWorld, const DirectCameraModel& camera)
{
	if (!skeleton.getBone(from).present || !skeleton.getBone(to).present || !pose.bones[(int)from].present ||
		!pose.bones[(int)to].present)
		return;
	glm::vec2 a, b;
	if (!AvatarOverlay::projectToPixel(posed[(int)from].positionWorld, cameraToWorld, camera, a) ||
		!AvatarOverlay::projectToPixel(posed[(int)to].positionWorld, cameraToWorld, camera, b))
		return;
	cv::line(image, cv::Point2f(a.x, a.y), cv::Point2f(b.x, b.y), cv::Scalar(0, 0, 0), 4);
	cv::line(image, cv::Point2f(a.x, a.y), cv::Point2f(b.x, b.y), cv::Scalar(255, 255, 255), 2);
	cv::circle(image, cv::Point2f(a.x, a.y), 3, cv::Scalar(255, 255, 255), cv::FILLED);
	cv::circle(image, cv::Point2f(b.x, b.y), 3, cv::Scalar(255, 255, 255), cv::FILLED);
}
} // namespace

namespace AvatarOverlay
{
bool projectToPixel(const glm::vec3& world, const glm::mat4& cameraToWorld, const DirectCameraModel& camera,
					glm::vec2& outPixel)
{
	const glm::vec3 inCamera= glm::vec3(glm::inverse(cameraToWorld) * glm::vec4(world, 1.f));
	if (inCamera.z <= 1e-4f)
		return false;
	outPixel= glm::vec2(camera.fx * inCamera.x / inCamera.z + camera.cx, camera.fy * inCamera.y / inCamera.z + camera.cy);
	return true;
}

void drawPosedSkeleton(cv::Mat& image, const AvatarSkeleton& skeleton, const AvatarPose& pose,
					   const glm::mat4& cameraToWorld, const DirectCameraModel& camera)
{
	if (!pose.valid)
		return;
	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
	computePosedBones(skeleton, pose, posed);
	auto segment= [&](B from, B to) {
		drawBoneSegment(image, posed, skeleton, pose, from, to, cameraToWorld, camera);
	};

	segment(B::LeftUpperArm, B::RightUpperArm);
	segment(B::Neck, B::Head);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const B upperArm= sideIndex == 0 ? B::LeftUpperArm : B::RightUpperArm;
		const B lowerArm= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
		const B hand= sideIndex == 0 ? B::LeftHand : B::RightHand;
		segment(upperArm, lowerArm);
		segment(lowerArm, hand);
		// Each finger: the hand to its proximal bone, then along the chain.
		// The last phalanx has no child bone, so its tip is not drawn.
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			const B proximal= (B)((int)firstHumanoidFingerBone(sideIndex) + finger * 3);
			segment(hand, proximal);
			segment(proximal, (B)((int)proximal + 1));
			segment((B)((int)proximal + 1), (B)((int)proximal + 2));
		}
	}
}

cv::Mat makeCompareImage(const cv::Mat& frame, const cv::Mat& avatarRender)
{
	if (frame.empty())
		return avatarRender.clone();
	if (avatarRender.empty())
		return frame.clone();
	cv::Mat render= avatarRender;
	if (render.rows != frame.rows)
	{
		const double scale= (double)frame.rows / (double)render.rows;
		cv::resize(avatarRender, render, cv::Size(), scale, scale, cv::INTER_AREA);
	}
	if (render.type() != frame.type())
		render.convertTo(render, frame.type());
	cv::Mat compare;
	cv::hconcat(frame, render, compare);
	return compare;
}
} // namespace AvatarOverlay
