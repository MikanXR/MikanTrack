#include "TestCommon.h"

#include "glm/ext/matrix_transform.hpp"

#include "AvatarOverlay.h"
#include "DirectRetarget.h"
#include "Scene3dPanel.h"

// The camera-matched scene view, no GL: a point the pinhole puts on a pixel
// lands on that pixel through the view and projection the scene renders
// with, and through the overlay's projection, under a tilted and
// translated camera; a point behind the camera is refused by both.
static int runSceneViewTest(const TestArgs& args)
{
	int result= 0;
	auto check= [&](bool bPass, const char* name) {
		if (bPass)
			MIKAN_LOG_INFO("test-scene-view") << "PASS " << name;
		else
		{
			MIKAN_LOG_ERROR("test-scene-view") << "FAIL " << name;
			result= 1;
		}
	};

	DirectCameraModel camera;
	camera.fx= 610.f;
	camera.fy= 590.f;
	camera.cx= 335.f;
	camera.cy= 228.f;
	camera.width= 640;
	camera.height= 480;
	const glm::mat4 cameraToWorld= glm::translate(glm::mat4(1.f), glm::vec3(0.1f, 0.2f, 0.3f)) *
		glm::mat4_cast(DirectRetarget::worldFromCamera(0.2f));

	glm::mat4 view, projection;
	makeCameraMatchedView(camera, cameraToWorld, 0.01f, 100.f, view, projection);
	const glm::mat4 viewProjection= projection * view;

	const glm::vec3 pointsInCamera[]= {
		glm::vec3(0.12f, -0.07f, 1.3f),
		glm::vec3(-0.3f, 0.2f, 0.8f),
		glm::vec3(0.f, 0.f, 2.f),
	};
	float maxMiss= 0.f;
	float maxOverlayMiss= 0.f;
	for (const glm::vec3& inCamera : pointsInCamera)
	{
		const glm::vec2 expected(camera.fx * inCamera.x / inCamera.z + camera.cx,
								 camera.fy * inCamera.y / inCamera.z + camera.cy);
		const glm::vec3 world= glm::vec3(cameraToWorld * glm::vec4(inCamera, 1.f));
		glm::vec2 pixel;
		if (projectCameraMatched(viewProjection, world, camera.width, camera.height, pixel))
			maxMiss= std::max(maxMiss, glm::length(pixel - expected));
		else
			maxMiss= 1e9f;
		glm::vec2 overlayPixel;
		if (AvatarOverlay::projectToPixel(world, cameraToWorld, camera, overlayPixel))
			maxOverlayMiss= std::max(maxOverlayMiss, glm::length(overlayPixel - expected));
		else
			maxOverlayMiss= 1e9f;
	}
	MIKAN_LOG_INFO("test-scene-view") << "max miss: view " << maxMiss << " px, overlay " << maxOverlayMiss << " px";
	check(maxMiss < 1e-2f, "(a) the camera-matched view lands a point on its pinhole pixel");
	check(maxOverlayMiss < 1e-3f, "(b) the overlay projection lands a point on its pinhole pixel");

	// The pixel of a point on the optical axis is the principal point, so
	// the off-center terms carry the right sign
	{
		const glm::vec3 world= glm::vec3(cameraToWorld * glm::vec4(0.f, 0.f, 1.f, 1.f));
		glm::vec2 pixel;
		const bool bOk= projectCameraMatched(viewProjection, world, camera.width, camera.height, pixel) &&
			glm::length(pixel - glm::vec2(camera.cx, camera.cy)) < 1e-2f;
		check(bOk, "(c) the optical axis lands on the principal point");
	}

	{
		const glm::vec3 behind= glm::vec3(cameraToWorld * glm::vec4(0.f, 0.f, -1.f, 1.f));
		glm::vec2 pixel;
		const bool bViewRefused= !projectCameraMatched(viewProjection, behind, camera.width, camera.height, pixel);
		const bool bOverlayRefused= !AvatarOverlay::projectToPixel(behind, cameraToWorld, camera, pixel);
		check(bViewRefused && bOverlayRefused, "(d) a point behind the camera is refused");
	}

	MIKAN_LOG_INFO("test-scene-view") << (result == 0 ? "ALL PASS" : "FAILURES");
	return result;
}

MIKAN_REGISTER_TEST("--test-scene-view", "Camera-matched scene view and overlay projection", eTestCategory::SelfTest,
					runSceneViewTest);
