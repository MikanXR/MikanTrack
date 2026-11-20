#include "TestCommon.h"

#include "AvatarRetarget.h"
#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "BodyPoseTracker.h"
#include "DirectRetarget.h"
#include "FrameRecorder.h"
#include "HeadlessGlContext.h"
#include "PathUtils.h"
#include "RetargetChains.h"
#include "RetargetStage.h"
#include "Scene3dPanel.h"
#include "TrackingRecording.h"
#include "TrackingReplay.h"
#include "VrmLoader.h"

// Renders the avatar a one-camera direct project would show for every frame
// of a take with raw frames: the whole-body model run over the camera's raw
// frames, the recorded hands, the retarget stage as the app builds it for a
// direct project on that camera, and the avatar drawn from the camera at the
// frame's size. One image per raw frame, named by the camera's frame index,
// so the sequence lines up with the raw frames for a side-by-side video.
//
//   --render-direct <recording.jsonl> <outDir> [cameraIndex] [vrm=<file.vrm>] [tilt=<degrees>]
//       [forearm=<meters>] [view=front] [near=<fraction>] [far=<fraction>]
//
// The take need not be a direct-mode recording: the camera given (the first
// with body pose by default) becomes the direct camera. vrm names the avatar
// in place of the recorded one. tilt and forearm stand in for the recorded
// direct tilt and forearm length. view=front draws the avatar from a camera
// in front of it that rides with its hips, in place of the tracking camera,
// so the arms stay in the picture when the user's hands leave the frame.
// near and far stand in for the depth planes' fractions of the reach.
namespace
{
constexpr const char* kTag= "render-direct";
// The front view: a 35 degree lens this far ahead of the hips and this far
// above them, the upper body with room for the arms
constexpr float kFrontViewFovDegrees= 35.f;
constexpr float kFrontViewDistance= 1.4f;
constexpr float kFrontViewAboveHips= 0.35f;

int runRenderDirectTool(const TestArgs& args)
{
	std::string recordingPath;
	std::string outDirArg;
	std::string vrmArg;
	int requestedCamera= -1;
	bool bTilt= false;
	float tiltDegrees= 0.f;
	float forearmMeters= 0.f;
	bool bFrontView= false;
	float nearFraction= -1.f;
	float farFraction= -1.f;
	for (const std::string& arg : args)
	{
		if (arg.rfind("near=", 0) == 0)
		{
			nearFraction= (float)atof(arg.substr(5).c_str());
			continue;
		}
		if (arg.rfind("far=", 0) == 0)
		{
			farFraction= (float)atof(arg.substr(4).c_str());
			continue;
		}
		if (arg == "view=front")
		{
			bFrontView= true;
			continue;
		}
		if (arg.rfind("vrm=", 0) == 0)
			vrmArg= arg.substr(4);
		else if (arg.rfind("tilt=", 0) == 0)
		{
			bTilt= true;
			tiltDegrees= (float)atof(arg.substr(5).c_str());
		}
		else if (arg.rfind("forearm=", 0) == 0)
			forearmMeters= (float)atof(arg.substr(8).c_str());
		else if (recordingPath.empty())
			recordingPath= arg;
		else if (outDirArg.empty())
			outDirArg= arg;
		else
			requestedCamera= atoi(arg.c_str());
	}
	if (recordingPath.empty() || outDirArg.empty())
	{
		MIKAN_LOG_ERROR(kTag) << "Usage: --render-direct <recording.jsonl> <outDir> [cameraIndex] [vrm=<file.vrm>] "
								 "[tilt=<degrees>] [forearm=<meters>] [view=front] [near=<fraction>] [far=<fraction>]";
		return 1;
	}
	const std::filesystem::path outDir= PathUtils::utf8ToPath(outDirArg);

	TrackingReplay replay;
	std::string error;
	if (!replay.load(recordingPath, error))
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": load failed: " << error;
		return 1;
	}
	// The recorded project as a direct project on the camera
	AppConfig config= replay.getRecordedConfig();
	int cameraIndex= requestedCamera;
	for (int index= 0; cameraIndex < 0 && index < (int)config.cameraCount(); ++index)
	{
		if (getBodyPoseBackend(config, index).has_value())
			cameraIndex= index;
	}
	if (cameraIndex < 0 || cameraIndex >= (int)config.cameraCount() || !config.camera(cameraIndex).intrinsics.present)
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": no camera with intrinsics to render from; pass a cameraIndex";
		return 1;
	}
	config.direct.enabled= true;
	config.direct.cameraIndex= cameraIndex;
	if (bTilt)
		config.direct.cameraTiltDegrees= tiltDegrees;
	if (forearmMeters > 0.f)
		config.body.forearmLengthMeters= forearmMeters;
	if (nearFraction >= 0.f)
		config.direct.nearPlaneReachFraction= nearFraction;
	if (farFraction >= 0.f)
		config.direct.farPlaneReachFraction= farFraction;
	if (!vrmArg.empty())
		config.avatar.modelPath= vrmArg;
	if (config.avatar.modelPath.empty())
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": the recorded config names no avatar; pass vrm=<file.vrm>";
		return 1;
	}
	const std::string frameDirectory= TrackingRecording::makeFrameDirectoryPath(recordingPath);
	if (!std::filesystem::exists(frameDirectory))
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": no raw frames beside the recording (" << frameDirectory << ")";
		return 1;
	}

	// The avatar through its rig sidecar
	const std::filesystem::path modelPath= PathUtils::utf8ToPath(config.avatar.modelPath);
	const VrmLoader::LoadResult loaded= VrmLoader::loadFile(modelPath);
	if (loaded.model == nullptr)
	{
		MIKAN_LOG_ERROR(kTag) << "Avatar load failed (" << modelPath << "): " << loaded.error;
		return 1;
	}
	const AvatarRigSettings rig= loadAvatarRig(modelPath);
	std::vector<std::string> rigWarnings;
	applyRigToModel(rig, *loaded.model, rigWarnings);
	for (const std::string& warning : rigWarnings)
		MIKAN_LOG_WARNING(kTag) << warning;
	const std::shared_ptr<const AvatarSkeleton> skeleton= std::make_shared<AvatarSkeleton>(*loaded.model);
	for (const std::string& warning : skeleton->getWarnings())
		MIKAN_LOG_WARNING(kTag) << warning;

	const MikanMatrix3d& cameraMatrix= config.camera(cameraIndex).intrinsics.intrinsics.undistorted_camera_matrix;
	DirectCameraModel cameraModel;
	cameraModel.fx= (float)cameraMatrix.x0;
	cameraModel.fy= (float)cameraMatrix.y1;
	cameraModel.cx= (float)cameraMatrix.z0;
	cameraModel.cy= (float)cameraMatrix.z1;

	const RetargetChains chains= resolveRetargetChains(config, rig);
	RetargetStage stage;
	stage.setConfig(makeRetargetStageConfig(config, rig, chains, skeleton));

	// The whole-body model on every raw frame, as the live direct camera
	// runs it at a divider of 1
	BodyPoseTrackerConfig trackerConfig;
	trackerConfig.backend= eBodyPoseBackend::Rtmw3dWholeBody;
	trackerConfig.frameDivider= 1;
	trackerConfig.detectorIntervalFrames= config.camera(cameraIndex).bodyPose.detectorIntervalFrames;
	BodyPoseTracker tracker;
	if (!tracker.load("models", "directml", trackerConfig))
	{
		MIKAN_LOG_ERROR(kTag) << "Failed to load the whole-body model (models/rtmw3d_wholebody.onnx)";
		return 1;
	}

	replay.runAll();

	std::error_code ec;
	std::filesystem::create_directories(outDir, ec);
	HeadlessGlContext gl;
	if (!gl.create(kTag))
		return 1;

	int framesRendered= 0;
	int framesMissing= 0;
	int framesUnposed= 0;
	// On the depth planes: the raw frames each arm spent on the near plane,
	// and the first of them, by the count of raw frames rendered
	int nearFrames[2]= {0, 0};
	int firstNearFrame[2]= {-1, -1};
	{
		Scene3dPanel scene;
		scene.setAvatar(loaded.model, skeleton, 1);
		scene.setShowAvatar(true);
		RetargetResult result;
		const TrackingFrameResult::HeadPose noPhoneHead;
		cv::Mat lastRender;
		for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
		{
			const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIndex);
			const RecordedCameraInput* fresh= nullptr;
			for (const RecordedCameraInput& candidate : frame.recorded->freshCameras)
			{
				if (candidate.cameraIndex == cameraIndex)
					fresh= &candidate;
			}
			if (fresh == nullptr)
				continue;
			const cv::Mat image= cv::imread(
				FrameRecorder::makeFramePath(frameDirectory, cameraIndex, fresh->frameIndex), cv::IMREAD_COLOR);
			if (image.empty())
			{
				framesMissing++;
				continue;
			}

			BodyPoseObservation observation;
			tracker.process(image, observation);
			cv::Mat render;
			if (cameraIndex < (int)frame.perCamera.size() && frame.perCameraValid[cameraIndex])
			{
				TrackingFrameResult cameraInput= frame.perCamera[cameraIndex];
				cameraInput.body= observation;
				cameraModel.width= cameraInput.frameWidth > 0 ? cameraInput.frameWidth : image.cols;
				cameraModel.height= cameraInput.frameHeight > 0 ? cameraInput.frameHeight : image.rows;
				stage.solveDirect(cameraInput, cameraModel, noPhoneHead, cameraInput.timestampMs, result);

				for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
				{
					if (!result.directState.arms[sideIndex].bNearPlane)
						continue;
					if (nearFrames[sideIndex]++ == 0)
						firstNearFrame[sideIndex]= framesRendered;
				}
				const glm::mat4 cameraToWorld=
					DirectRetarget::makeCameraToWorld(*skeleton, result.pose, result.directState);
				scene.setAvatarPose(&result.pose);
				SceneCameraMatch match;
				match.valid= true;
				match.camera= cameraModel;
				match.cameraToWorld= cameraToWorld;
				if (bFrontView)
				{
					// Ahead of the avatar on its +X, looking back along it: the
					// camera's right is the avatar's left and its down is -Z
					std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
					computePosedBones(*skeleton, result.pose, posed);
					const glm::vec3 eye= posed[(int)eHumanoidBone::Hips].positionWorld +
						glm::vec3(kFrontViewDistance, 0.f, kFrontViewAboveHips);
					match.cameraToWorld= glm::mat4(glm::vec4(0.f, 1.f, 0.f, 0.f), glm::vec4(0.f, 0.f, -1.f, 0.f),
												   glm::vec4(-1.f, 0.f, 0.f, 0.f), glm::vec4(eye, 1.f));
					const float focal=
						0.5f * (float)cameraModel.height / tanf(glm::radians(kFrontViewFovDegrees) * 0.5f);
					match.camera.fx= match.camera.fy= focal;
					match.camera.cx= 0.5f * (float)cameraModel.width;
					match.camera.cy= 0.5f * (float)cameraModel.height;
				}
				scene.setCaptureMatch(match);
				if (!scene.captureFromCamera(cameraInput, {}, {}, render))
					render= cv::Mat();
			}
			// A frame the camera had no result for repeats the image before
			// it, so the sequence keeps one image per raw frame
			if (render.empty())
			{
				framesUnposed++;
				render= lastRender.empty() ? cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0)) : lastRender;
			}
			lastRender= render;

			char name[64];
			snprintf(name, sizeof(name), "frame_%lld.jpg", (long long)fresh->frameIndex);
			if (!cv::imwrite((outDir / name).string(), render))
			{
				MIKAN_LOG_ERROR(kTag) << "Could not write " << (outDir / name);
				return 1;
			}
			framesRendered++;
		}
	}
	MIKAN_LOG_INFO(kTag) << recordingPath << ": camera " << cameraIndex << ", " << framesRendered << " frames to "
						 << outDir << " (" << framesUnposed << " with no camera result, " << framesMissing
						 << " raw frames missing)";
	MIKAN_LOG_INFO(kTag) << "near plane: left on " << nearFrames[0] << " frames from frame " << firstNearFrame[0]
						 << ", right on " << nearFrames[1] << " from frame " << firstNearFrame[1];
	return framesRendered > 0 ? 0 : 1;
}
} // namespace

MIKAN_REGISTER_TEST("--render-direct",
					"Render the avatar a direct project would show for every raw frame of a take, from the camera "
					"(<recording.jsonl> <outDir> [cameraIndex] [vrm=<file.vrm>] [tilt=<degrees>] [forearm=<meters>] [view=front] [near=<fraction>] [far=<fraction>])",
					eTestCategory::Tool, runRenderDirectTool);
