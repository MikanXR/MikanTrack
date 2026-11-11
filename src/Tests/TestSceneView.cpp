#include "TestCommon.h"

#include "glm/ext/matrix_transform.hpp"

#include "AvatarOverlay.h"
#include "DirectRetarget.h"
#include "PipelineModel.h"
#include "Scene3dPanel.h"
#include "SourceSkeleton.h"
#include "StageView.h"

// The camera-matched scene view, no GL: a point the pinhole puts on a pixel
// lands on that pixel through the view and projection the scene renders
// with, and through the overlay's projection, under a tilted and
// translated camera; a point behind the camera is refused by both. Then
// the Retarget view's pieces: a viewport inside the image, the layers each
// stage draws and their side by side split, the source skeleton's layout
// per pipeline and state, its dashes, and the click's hit test.
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

	// A viewport inside a larger image (a half of a side by side view, a
	// letterbox) moves the pixel by its offset, the image's rows counted
	// from the top and the viewport's from the bottom
	{
		const glm::ivec4 viewport(100, 20, camera.width, camera.height);
		const glm::vec2 imageMin(7.f, 9.f);
		const float imageHeight= 600.f;
		float maxViewportMiss= 0.f;
		for (const glm::vec3& inCamera : pointsInCamera)
		{
			const glm::vec3 world= glm::vec3(cameraToWorld * glm::vec4(inCamera, 1.f));
			glm::vec2 full, inImage;
			if (!projectCameraMatched(viewProjection, world, camera.width, camera.height, full) ||
				!projectToViewImage(viewProjection, viewport, imageMin, imageHeight, world, inImage))
			{
				maxViewportMiss= 1e9f;
				continue;
			}
			const glm::vec2 expected= imageMin + glm::vec2(100.f, imageHeight - 20.f - (float)camera.height) + full;
			maxViewportMiss= std::max(maxViewportMiss, glm::length(inImage - expected));
		}
		check(maxViewportMiss < 1e-2f, "(e) a viewport inside the image offsets the projected pixel");
	}

	// The layers: Retarget draws the source pose with the avatar, its gizmos,
	// the direct chains and the IK goals in place of the fused skeleton;
	// the Source pose stage the source pose alone; the IK goals go with the
	// avatar; the whole scene and Fusion keep the fused skeleton and no
	// source pose
	{
		PipelineStageId retarget;
		retarget.stage= ePipelineStage::Retarget;
		PipelineStageId sourcePose;
		sourcePose.stage= ePipelineStage::SourcePose;
		PipelineStageId fusion;
		fusion.stage= ePipelineStage::Fusion;
		const SceneLayers r= makeStageSceneLayers(retarget, true, true);
		const SceneLayers rHidden= makeStageSceneLayers(retarget, false, true);
		const SceneLayers s= makeStageSceneLayers(sourcePose, true, true);
		const SceneLayers f= makeStageSceneLayers(fusion, true, false);
		const SceneLayers whole= makeStageSceneLayers(PipelineStageId(), true, false);
		const bool bRetarget= r.bGrid && r.bSourcePose && r.bAvatar && r.bGizmos && r.bDirectSolve && r.bIkGoals &&
			!r.bFusedSkeleton && !r.bPerCameraSkeletons && !r.bCameraFrustums;
		const bool bHidden= rHidden.bSourcePose && !rHidden.bAvatar && !rHidden.bGizmos && !rHidden.bIkGoals;
		const bool bSource= s.bGrid && s.bSourcePose && !s.bAvatar && !s.bFusedSkeleton && !s.bDirectSolve &&
			!s.bIkGoals && !s.bGizmos && !s.bCameraFrustums;
		const bool bOthers= f.bFusedSkeleton && !f.bSourcePose && whole.bFusedSkeleton && !whole.bSourcePose &&
			!whole.bIkGoals && whole.bAvatar && whole.bCameraFrustums;
		const SceneLayers defaults;
		check(bRetarget && bHidden && bSource && bOthers && !defaults.bSourcePose && !defaults.bIkGoals,
			  "(f) the stages' layers and the defaults the capture draws");

		SceneLayers sourceHalf, avatarHalf;
		splitSideBySideLayers(r, sourceHalf, avatarHalf);
		const bool bSplit= sourceHalf.bGrid && sourceHalf.bSourcePose && !sourceHalf.bAvatar && !sourceHalf.bGizmos &&
			!sourceHalf.bIkGoals && !sourceHalf.bDirectSolve && avatarHalf.bGrid && avatarHalf.bAvatar &&
			avatarHalf.bGizmos && avatarHalf.bIkGoals && avatarHalf.bDirectSolve && !avatarHalf.bSourcePose;
		check(bSplit, "(g) side by side puts the source pose in one half and the avatar with its gizmos in the other");
	}

	// The source skeleton, position pipeline: a measured left arm is a solid
	// chain of two bones, a held right arm a dashed one, the torso's
	// shoulder line and neck, and the head a marker with its facing and up
	// lines; a lost arm, a lost torso and an absent head anchor draw nothing
	auto countSegments= [](const SourceSkeleton& skeleton, eRetargetChain chain, bool bDashed) {
		int count= 0;
		for (const SourceSkeleton::Segment& segment : skeleton.segments)
			count+= segment.chain == chain && segment.bDashed == bDashed ? 1 : 0;
		return count;
	};
	auto countMarkers= [](const SourceSkeleton& skeleton, eRetargetChain chain) {
		int count= 0;
		for (const SourceSkeleton::Marker& marker : skeleton.markers)
			count+= marker.chain == chain ? 1 : 0;
		return count;
	};
	SourcePose position;
	position.pipeline= SourcePose::ePipeline::Position;
	position.torso.state= SourcePose::eGroupState::Measured;
	position.head.state= SourcePose::eGroupState::Measured;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const float side= sideIndex == 0 ? 1.f : -1.f;
		SourcePose::Arm& arm= position.arms[sideIndex];
		arm.state= sideIndex == 0 ? SourcePose::eGroupState::Measured : SourcePose::eGroupState::Held;
		arm.bHasShoulder= true;
		arm.shoulderPosition= glm::vec3(0.f, 0.18f * side, 1.4f);
		arm.bHasElbow= true;
		arm.elbowPosition= glm::vec3(0.05f, 0.22f * side, 1.12f);
		arm.wristPosition= glm::vec3(0.3f, 0.2f * side, 1.0f);
		arm.forearmConfidence.bPresent= true;
		arm.forearmConfidence.value= 1.f;
	}
	const glm::vec3 headAnchor(0.f, 0.f, 1.65f);
	{
		SourceSkeletonInputs inputs;
		inputs.source= &position;
		inputs.headAnchorWorld= &headAnchor;
		SourceSkeleton skeleton;
		buildSourceSkeleton(inputs, skeleton);
		const bool bArms= countSegments(skeleton, eRetargetChain::LeftArm, false) == 2 &&
			countSegments(skeleton, eRetargetChain::LeftArm, true) == 0 &&
			countSegments(skeleton, eRetargetChain::RightArm, true) == 2 &&
			countSegments(skeleton, eRetargetChain::RightArm, false) == 0 &&
			countMarkers(skeleton, eRetargetChain::LeftArm) == 3;
		const bool bTorsoHead= countSegments(skeleton, eRetargetChain::Torso, false) == 2 &&
			countSegments(skeleton, eRetargetChain::Head, false) == 2 && countMarkers(skeleton, eRetargetChain::Head) == 1;
		check(bArms && bTorsoHead, "(h) position source: measured solid, held dashed, torso and head drawn");

		SourcePose lost= position;
		lost.arms[1]= SourcePose::Arm();
		lost.torso.state= SourcePose::eGroupState::Lost;
		inputs.source= &lost;
		inputs.headAnchorWorld= nullptr;
		buildSourceSkeleton(inputs, skeleton);
		const int rightArmPieces= countSegments(skeleton, eRetargetChain::RightArm, false) +
			countSegments(skeleton, eRetargetChain::RightArm, true) + countMarkers(skeleton, eRetargetChain::RightArm);
		const bool bLost= rightArmPieces == 0 && countSegments(skeleton, eRetargetChain::Torso, false) == 0 &&
			countMarkers(skeleton, eRetargetChain::Head) == 0;
		check(bLost, "(i) a lost arm, a lost torso and a head without an anchor draw nothing");
	}

	// The confidence colors: green at 1, amber at 0, a third color without
	// one
	{
		SourcePose::Confidence high, low, none;
		high.bPresent= true;
		high.value= 1.f;
		low.bPresent= true;
		low.value= 0.f;
		const glm::vec3 highColor= getSourceConfidenceColor(high);
		const glm::vec3 lowColor= getSourceConfidenceColor(low);
		const glm::vec3 noneColor= getSourceConfidenceColor(none);
		const bool bOk= highColor.g > highColor.r && lowColor.r > lowColor.g && lowColor.r > lowColor.b &&
			glm::length(noneColor - highColor) > 0.1f && glm::length(noneColor - lowColor) > 0.1f;
		check(bOk, "(j) confidence runs green to amber, a group without one is neutral");
	}

	// The direct pipeline: the chain is carried by the solve's transform, a
	// resting arm carries no positions and draws nothing, the head is a
	// marker alone, and without the transform nothing draws
	{
		SourcePose direct= position;
		direct.pipeline= SourcePose::ePipeline::Direct;
		direct.arms[0].forearmConfidence= SourcePose::Confidence();
		direct.arms[1]= SourcePose::Arm();
		direct.arms[1].state= SourcePose::eGroupState::Resting;
		direct.torso.state= SourcePose::eGroupState::Held;
		const glm::vec3 offset(0.f, 0.f, 2.f);
		const glm::mat4 toWorld= glm::translate(glm::mat4(1.f), offset);
		SourceSkeletonInputs inputs;
		inputs.source= &direct;
		inputs.directToWorld= &toWorld;
		inputs.headAnchorWorld= &headAnchor;
		SourceSkeleton skeleton;
		buildSourceSkeleton(inputs, skeleton);
		bool bCarried= false;
		for (const SourceSkeleton::Segment& segment : skeleton.segments)
		{
			if (segment.chain == eRetargetChain::LeftArm &&
				glm::length(segment.start - (direct.arms[0].shoulderPosition + offset)) < 1e-5f)
				bCarried= true;
		}
		const int restingPieces= countSegments(skeleton, eRetargetChain::RightArm, false) +
			countSegments(skeleton, eRetargetChain::RightArm, true) + countMarkers(skeleton, eRetargetChain::RightArm);
		const bool bHead= countMarkers(skeleton, eRetargetChain::Head) == 1 &&
			countSegments(skeleton, eRetargetChain::Head, false) == 0;
		inputs.directToWorld= nullptr;
		SourceSkeleton unplaced;
		buildSourceSkeleton(inputs, unplaced);
		check(bCarried && restingPieces == 0 && bHead && unplaced.segments.empty() && unplaced.markers.empty(),
			  "(k) direct source: carried into the avatar's world, a resting arm and an unplaced solve draw nothing");
	}

	// The dashes: a bone of 10 cm in dashes of 1.5 cm with gaps of 1 cm is
	// four dashes from its start, none past its end; a bone shorter than a
	// dash is one
	{
		std::vector<std::array<glm::vec3, 2>> dashes;
		const glm::vec3 start(0.f, 0.f, 1.f);
		const glm::vec3 end(0.1f, 0.f, 1.f);
		appendDashes(start, end, 0.015f, 0.01f, dashes);
		bool bOk= dashes.size() == 4 && glm::length(dashes[0][0] - start) < 1e-6f;
		for (const std::array<glm::vec3, 2>& dash : dashes)
			bOk= bOk && dash[1].x <= end.x + 1e-6f && std::fabs(glm::length(dash[1] - dash[0]) - 0.015f) < 1e-5f;
		std::vector<std::array<glm::vec3, 2>> shortDashes;
		appendDashes(start, start + glm::vec3(0.01f, 0.f, 0.f), 0.015f, 0.01f, shortDashes);
		bOk= bOk && shortDashes.size() == 1;
		check(bOk, "(l) the dashes along a carried bone");
	}

	// The hit test: the distance to a segment past either end is to that
	// end; a click within the tolerance of a bone's projection picks its
	// chain, one farther off picks nothing
	{
		const float perpendicular= distanceToSegment2d(glm::vec2(5.f, 3.f), glm::vec2(0.f, 0.f), glm::vec2(10.f, 0.f));
		const float pastEnd= distanceToSegment2d(glm::vec2(13.f, 4.f), glm::vec2(0.f, 0.f), glm::vec2(10.f, 0.f));
		const float degenerate= distanceToSegment2d(glm::vec2(3.f, 4.f), glm::vec2(0.f, 0.f), glm::vec2(0.f, 0.f));
		const bool bDistances= std::fabs(perpendicular - 3.f) < 1e-5f && std::fabs(pastEnd - 5.f) < 1e-5f &&
			std::fabs(degenerate - 5.f) < 1e-5f;

		// A bone and the head's marker in front of the camera
		SourceSkeleton skeleton;
		SourceSkeleton::Segment bone;
		bone.start= glm::vec3(cameraToWorld * glm::vec4(-0.2f, 0.1f, 1.5f, 1.f));
		bone.end= glm::vec3(cameraToWorld * glm::vec4(0.2f, 0.1f, 1.5f, 1.f));
		bone.chain= eRetargetChain::RightArm;
		skeleton.segments.push_back(bone);
		SourceSkeleton::Marker head;
		head.position= glm::vec3(cameraToWorld * glm::vec4(0.f, -0.3f, 1.5f, 1.f));
		head.chain= eRetargetChain::Head;
		skeleton.markers.push_back(head);

		const glm::ivec4 viewport(0, 0, camera.width, camera.height);
		const glm::vec2 imageMin(0.f, 0.f);
		const float imageHeight= (float)camera.height;
		glm::vec2 middle(0.f), headPixel(0.f);
		const bool bProjected=
			projectToViewImage(viewProjection, viewport, imageMin, imageHeight, (bone.start + bone.end) * 0.5f, middle) &&
			projectToViewImage(viewProjection, viewport, imageMin, imageHeight, head.position, headPixel);
		eRetargetChain picked= eRetargetChain::Torso;
		const bool bBone= pickSourceSkeleton(skeleton, viewProjection, viewport, imageMin, imageHeight,
											 middle + glm::vec2(0.f, 4.f), 6.f, picked) &&
			picked == eRetargetChain::RightArm;
		const bool bHead= pickSourceSkeleton(skeleton, viewProjection, viewport, imageMin, imageHeight,
											 headPixel + glm::vec2(3.f, 0.f), 6.f, picked) &&
			picked == eRetargetChain::Head;
		const bool bMiss= !pickSourceSkeleton(skeleton, viewProjection, viewport, imageMin, imageHeight,
											  middle + glm::vec2(0.f, 25.f), 6.f, picked);
		check(bDistances && bProjected && bBone && bHead && bMiss,
			  "(m) a click near a bone or a marker picks its chain, one off it nothing");
	}

	MIKAN_LOG_INFO("test-scene-view") << (result == 0 ? "ALL PASS" : "FAILURES");
	return result;
}

MIKAN_REGISTER_TEST("--test-scene-view",
					"Camera-matched scene view, overlay projection, the Retarget view's layers, source skeleton, "
					"dashes and pick",
					eTestCategory::SelfTest, runSceneViewTest);
