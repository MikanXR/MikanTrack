#include "TestCommon.h"

#include "StageView.h"

// What the Stage View shows for each mode, through the pure functions the
// view decides by, and the mode names automation speaks

namespace
{
const char* const kTag= "test-stage-view";

bool sameContent(const StageViewContent& a, const StageViewContent& b)
{
	return a.bCameras == b.bCameras && a.bScene == b.bScene && a.cameraIndex == b.cameraIndex &&
		a.sceneStage == b.sceneStage && a.bWizard == b.bWizard;
}
} // namespace

static int runStageViewTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO(kTag) << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR(kTag) << "FAIL " << name;
			failures++;
		}
	};

	PipelineStageId cameraLane;
	cameraLane.stage= ePipelineStage::HandModels;
	cameraLane.cameraIndex= 1;
	PipelineStageId retarget;
	retarget.stage= ePipelineStage::Retarget;

	// What following a camera lane's stage, a scene stage and nothing shows
	const StageViewContent followCamera= decideStageViewContent(cameraLane, false, -1, 3);
	const StageViewContent followScene= decideStageViewContent(retarget, false, -1, 3);
	const StageViewContent followNone= decideStageViewContent(PipelineStageId(), false, -1, 3);
	check(followCamera.bCameras && !followCamera.bScene && followCamera.cameraIndex == 1,
		  "(a) following a camera lane's stage shows that camera alone");
	check(!followScene.bCameras && followScene.bScene && followScene.sceneStage == retarget,
		  "(a) following a scene stage shows the scene with that stage");
	check(!followNone.bCameras && followNone.bScene && !followNone.sceneStage.isValid(),
		  "(a) following nothing shows the whole scene");

	// Follow is the follow rule itself
	check(sameContent(resolveStageViewContent(eStageViewMode::Follow, followCamera, false), followCamera) &&
			  sameContent(resolveStageViewContent(eStageViewMode::Follow, followScene, false), followScene),
		  "(b) the follow mode shows what following the selection shows");

	// Cameras and Scene hold one body whatever the selection
	{
		const StageViewContent camerasOnCamera= resolveStageViewContent(eStageViewMode::Cameras, followCamera, false);
		const StageViewContent camerasOnScene= resolveStageViewContent(eStageViewMode::Cameras, followScene, false);
		check(camerasOnCamera.bCameras && !camerasOnCamera.bScene && camerasOnCamera.cameraIndex == 1 &&
				  camerasOnScene.bCameras && !camerasOnScene.bScene && camerasOnScene.cameraIndex == -1,
			  "(c) the cameras mode shows the selection's camera, or every camera on a scene stage");
		const StageViewContent sceneOnCamera= resolveStageViewContent(eStageViewMode::Scene, followCamera, false);
		const StageViewContent sceneOnScene= resolveStageViewContent(eStageViewMode::Scene, followScene, false);
		check(!sceneOnCamera.bCameras && sceneOnCamera.bScene && !sceneOnCamera.sceneStage.isValid() &&
				  !sceneOnScene.bCameras && sceneOnScene.bScene && sceneOnScene.sceneStage == retarget,
			  "(c) the scene mode shows the selection's stage, or the whole scene on a camera stage");
	}

	// Split draws both, each half following the selection where it can
	{
		const StageViewContent splitOnCamera= resolveStageViewContent(eStageViewMode::Split, followCamera, false);
		const StageViewContent splitOnScene= resolveStageViewContent(eStageViewMode::Split, followScene, false);
		const StageViewContent splitOnNone= resolveStageViewContent(eStageViewMode::Split, followNone, false);
		check(splitOnCamera.bCameras && splitOnCamera.bScene && splitOnCamera.cameraIndex == 1 &&
				  !splitOnCamera.sceneStage.isValid(),
			  "(d) the split mode on a camera stage shows that camera beside the whole scene");
		check(splitOnScene.bCameras && splitOnScene.bScene && splitOnScene.cameraIndex == -1 &&
				  splitOnScene.sceneStage == retarget,
			  "(d) the split mode on a scene stage shows every camera beside that stage's scene");
		check(splitOnNone.bCameras && splitOnNone.bScene && splitOnNone.cameraIndex == -1 &&
				  !splitOnNone.sceneStage.isValid(),
			  "(d) the split mode with nothing selected shows every camera beside the whole scene");
	}

	// A camera calibration wizard takes the view whatever the mode
	{
		bool bWizard= true;
		for (const eStageViewMode mode :
			 {eStageViewMode::Follow, eStageViewMode::Cameras, eStageViewMode::Scene, eStageViewMode::Split})
		{
			const StageViewContent content= resolveStageViewContent(mode, followScene, true);
			bWizard&= content.bWizard && content.bCameras && !content.bScene && content.cameraIndex == -1;
		}
		check(bWizard, "(e) a camera wizard shows every camera alone in every mode");
	}

	// The mode names
	{
		bool bRoundTrip= true;
		for (const eStageViewMode mode :
			 {eStageViewMode::Follow, eStageViewMode::Cameras, eStageViewMode::Scene, eStageViewMode::Split})
		{
			eStageViewMode parsed= eStageViewMode::Follow;
			bRoundTrip&= StageView::parseMode(StageView::getModeName(mode), parsed) && parsed == mode;
		}
		eStageViewMode parsed= eStageViewMode::Follow;
		check(bRoundTrip, "(f) every mode survives getModeName and parseMode");
		check(std::string(StageView::getModeName(eStageViewMode::Split)) == "split" &&
				  !StageView::parseMode("nope", parsed) && !StageView::parseMode("", parsed),
			  "(f) the split mode is named split, and an unknown word is refused");
	}

	MIKAN_LOG_INFO(kTag) << (failures == 0 ? "ALL PASS" : "FAILURES") << " (" << failures << ")";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-stage-view", "Stage View: the content of every view mode and the mode names",
					eTestCategory::SelfTest, runStageViewTest);
