#include "TestCommon.h"

#include "glm/ext/matrix_transform.hpp"

#include "AvatarJson.h"
#include "AvatarOverlay.h"
#include "AvatarRetarget.h"
#include "AvatarRetargetMath.h"
#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "BodyPoseTypes.h"
#include "DiagnosticDump.h"
#include "DirectRetarget.h"
#include "FrameRecorder.h"
#include "HeadlessGlContext.h"
#include "PathUtils.h"
#include "Scene3dPanel.h"
#include "TrackingRecording.h"
#include "TrackingReplay.h"
#include "VrmLoader.h"

// Re-solves one recorded frame of a direct-mode take or dump through the
// direct retarget and reports how the posed avatar sits on the camera's
// landmarks: the loop after a solver change, with no recapture.
//
//   --pose-report <recording.jsonl> [frameIndex] [outDir]
//
// The frame (the last by default) is held for kIterations solves at the
// recorded frame interval so the filters settle on the static pose, under
// the recorded config and the avatar at its modelPath. Writes
// pose-report.json (the solve state and the posed avatar, the dump's own
// serializers) and pose-report.png (the frame with the landmarks and the
// posed skeleton beside the avatar rendered from the camera), and logs the
// per-joint reprojection, the arm angles and the finger angles.
namespace
{
constexpr int kIterations= 60;
constexpr const char* kTag= "pose-report";
using B= eHumanoidBone;

float degrees(float radians)
{
	return radians * 180.f / glm::pi<float>();
}

struct JointMatch
{
	const char* name;
	B bone;
	glm::vec2 landmarkPx;
	bool bLandmark;
};

// The posed bone's miss from its landmark, logged and collected
void reportJoint(const JointMatch& match, const std::array<AvatarPosedBone, HUMANOID_BONE_COUNT>& posed,
				 const AvatarSkeleton& skeleton, const AvatarPose& pose, const glm::mat4& cameraToWorld,
				 const DirectCameraModel& camera, nlohmann::json& outJoints)
{
	if (!skeleton.getBone(match.bone).present || !pose.bones[(int)match.bone].present)
		return;
	glm::vec2 pixel;
	if (!AvatarOverlay::projectToPixel(posed[(int)match.bone].positionWorld, cameraToWorld, camera, pixel))
		return;
	nlohmann::json joint= {{"bone", humanoidBoneName(match.bone)}, {"avatarPx", {pixel.x, pixel.y}}};
	if (match.bLandmark)
	{
		const float missPx= glm::length(pixel - match.landmarkPx);
		joint["landmarkPx"]= {match.landmarkPx.x, match.landmarkPx.y};
		joint["missPx"]= missPx;
		MIKAN_LOG_INFO(kTag) << "  " << match.name << ": avatar (" << pixel.x << ", " << pixel.y << ") landmark ("
							 << match.landmarkPx.x << ", " << match.landmarkPx.y << ") miss " << missPx << " px";
	}
	else
	{
		MIKAN_LOG_INFO(kTag) << "  " << match.name << ": avatar (" << pixel.x << ", " << pixel.y
							 << ") no landmark";
	}
	outJoints.push_back(joint);
}

int runPoseReportTool(const TestArgs& args)
{
	if (args.empty())
	{
		MIKAN_LOG_ERROR(kTag) << "Usage: --pose-report <recording.jsonl> [frameIndex] [outDir]";
		return 1;
	}
	const std::string recordingPath= args[0];
	const int requestedFrame= args.size() > 1 ? std::atoi(args[1].c_str()) : -1;
	const std::filesystem::path outDir= args.size() > 2
		? PathUtils::utf8ToPath(args[2])
		: PathUtils::utf8ToPath(recordingPath).parent_path();

	TrackingReplay replay;
	std::string error;
	if (!replay.load(recordingPath, error))
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": load failed: " << error;
		return 1;
	}
	const AppConfig& config= replay.getRecordedConfig();
	if (!config.direct.enabled)
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": not a direct-mode recording";
		return 1;
	}
	const int cameraIndex= config.direct.cameraIndex;
	if (cameraIndex < 0 || cameraIndex >= (int)config.cameraCount() ||
		!config.camera(cameraIndex).intrinsics.present)
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": the direct camera " << cameraIndex << " has no intrinsics";
		return 1;
	}
	if (replay.getFrameCount() == 0)
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": no frames";
		return 1;
	}
	replay.runAll();
	const int frameIndex= requestedFrame < 0 ? replay.getFrameCount() - 1
											 : std::min(requestedFrame, replay.getFrameCount() - 1);
	const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIndex);
	if (cameraIndex >= (int)frame.perCamera.size() || !frame.perCameraValid[cameraIndex])
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": frame " << frameIndex << " has no result from camera "
							  << cameraIndex;
		return 1;
	}
	const TrackingFrameResult& cameraFrame= frame.perCamera[cameraIndex];

	// The avatar the recording was made with, through its rig sidecar
	if (config.avatar.modelPath.empty())
	{
		MIKAN_LOG_ERROR(kTag) << recordingPath << ": the recorded config names no avatar";
		return 1;
	}
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

	// The recording's undistorted pinhole at the frame's size
	const MikanMatrix3d& cameraMatrix= config.camera(cameraIndex).intrinsics.intrinsics.undistorted_camera_matrix;
	DirectCameraModel cameraModel;
	cameraModel.fx= (float)cameraMatrix.x0;
	cameraModel.fy= (float)cameraMatrix.y1;
	cameraModel.cx= (float)cameraMatrix.z0;
	cameraModel.cy= (float)cameraMatrix.z1;
	cameraModel.width= cameraFrame.frameWidth;
	cameraModel.height= cameraFrame.frameHeight;
	const DirectRetargetConfig directConfig= makeDirectRetargetConfig(config, rig);

	// The frame held still: the same frame solved at the recorded interval
	// until the filters have settled on it
	const double intervalMs= cameraFrame.captureFps > 1.f ? 1000.0 / (double)cameraFrame.captureFps : 33.3;
	DirectRetarget solver;
	AvatarPose pose;
	const TrackingFrameResult::HeadPose noPhoneHead;
	for (int iteration= 0; iteration < kIterations; ++iteration)
	{
		solver.solve(cameraFrame, cameraModel, noPhoneHead, cameraFrame.timestampMs + intervalMs * iteration,
					 *skeleton, directConfig, pose);
	}
	const DirectRetarget::SolveState& state= solver.getSolveState();
	const glm::mat4 cameraToWorld= DirectRetarget::makeCameraToWorld(*skeleton, pose, state);
	MIKAN_LOG_INFO(kTag) << recordingPath << " frame " << frameIndex << " of " << replay.getFrameCount() << ", camera "
						 << cameraIndex << " " << cameraModel.width << "x" << cameraModel.height << ", " << kIterations
						 << " solves at " << intervalMs << " ms, torso " << (state.torsoValid ? "solved" : "unsolved");

	// Reprojection: the posed bones against the body and hand landmarks
	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
	computePosedBones(*skeleton, pose, posed);
	const BodyPoseObservation& body= cameraFrame.body;
	auto bodyPx= [&](ePoseLandmark landmark, JointMatch& match) {
		match.bLandmark= body.valid && body.isProvided((int)landmark);
		if (match.bLandmark)
			match.landmarkPx= glm::vec2(body.imagePoints[(int)landmark]);
	};
	nlohmann::json joints= nlohmann::json::array();
	nlohmann::json arms= nlohmann::json::array();
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const char* sideName= sideIndex == 0 ? "L" : "R";
		MIKAN_LOG_INFO(kTag) << sideName << " arm reprojection:";
		JointMatch shoulder{sideIndex == 0 ? "L shoulder" : "R shoulder",
							sideIndex == 0 ? B::LeftUpperArm : B::RightUpperArm};
		JointMatch elbow{sideIndex == 0 ? "L elbow" : "R elbow", sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm};
		JointMatch wrist{sideIndex == 0 ? "L wrist" : "R wrist", sideIndex == 0 ? B::LeftHand : B::RightHand};
		bodyPx(sideIndex == 0 ? ePoseLandmark::LEFT_SHOULDER : ePoseLandmark::RIGHT_SHOULDER, shoulder);
		bodyPx(sideIndex == 0 ? ePoseLandmark::LEFT_ELBOW : ePoseLandmark::RIGHT_ELBOW, elbow);
		bodyPx(sideIndex == 0 ? ePoseLandmark::LEFT_WRIST : ePoseLandmark::RIGHT_WRIST, wrist);
		const TrackedHand& hand= cameraFrame.hands[sideIndex];
		// The hand model's wrist is the one the solver placed the wrist on
		if (hand.tracked)
		{
			wrist.bLandmark= true;
			wrist.landmarkPx= glm::vec2(hand.imagePoints[(int)eHandLandmark::WRIST]);
		}
		reportJoint(shoulder, posed, *skeleton, pose, cameraToWorld, cameraModel, joints);
		reportJoint(elbow, posed, *skeleton, pose, cameraToWorld, cameraModel, joints);
		reportJoint(wrist, posed, *skeleton, pose, cameraToWorld, cameraModel, joints);
		if (hand.tracked)
		{
			static const char* kFingerNames[FINGER_COUNT]= {"thumb", "index", "middle", "ring", "pinky"};
			static const char* kPhalanxNames[3]= {"proximal", "intermediate", "distal"};
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				for (int phalanx= 0; phalanx < 3; ++phalanx)
				{
					char name[48];
					snprintf(name, sizeof(name), "%s %s %s", sideName, kFingerNames[finger], kPhalanxNames[phalanx]);
					JointMatch joint{name, (B)((int)firstHumanoidFingerBone(sideIndex) + finger * 3 + phalanx)};
					joint.bLandmark= true;
					joint.landmarkPx= glm::vec2(hand.imagePoints[FINGER_JOINTS[finger][phalanx]]);
					reportJoint(joint, posed, *skeleton, pose, cameraToWorld, cameraModel, joints);
				}
			}
		}

		// The arm's angles and the wrist's distance, what the depth choice
		// came to
		const DirectRetarget::SolveState::Arm& arm= state.arms[sideIndex];
		nlohmann::json armJson= {{"side", sideName}, {"chainValid", arm.chain.valid}};
		if (arm.chain.valid)
		{
			const glm::vec3 upper= arm.chain.shoulder - arm.chain.elbow;
			const glm::vec3 lower= arm.chain.wrist - arm.chain.elbow;
			const float elbowBend= degrees(acosf(std::clamp(
				glm::dot(glm::normalize(upper), glm::normalize(lower)), -1.f, 1.f)));
			const float wristAngle= glm::angle(glm::normalize(arm.wristRotation));
			const float wristBend= degrees(std::min(wristAngle, glm::two_pi<float>() - wristAngle));
			armJson["elbowBendDegrees"]= elbowBend;
			armJson["wristBendDegrees"]= wristBend;
			armJson["wristDistance"]= arm.filteredDepth;
			armJson["chosenDepth"]= arm.chosenDepth;
			armJson["depthInterval"]= {arm.depthLow, arm.depthHigh};
			armJson["modelDepthDelta"]= arm.modelDepthDelta;
			armJson["pnpDistance"]= arm.pnpDistance;
			armJson["liftDepth"]= arm.liftDepth;
			MIKAN_LOG_INFO(kTag) << sideName << " arm: elbow bend " << elbowBend << " deg (180 straight), wrist bend "
								 << wristBend << " deg, wrist distance " << arm.filteredDepth << " (chosen "
								 << arm.chosenDepth << ", reach " << arm.depthLow << " to " << arm.depthHigh
								 << ", model depth delta " << arm.modelDepthDelta << ", lift distance "
								 << arm.pnpDistance << " scored at " << arm.liftDepth << ", measured scale "
								 << arm.pnpScale.scale << "), elbow "
								 << (arm.bElbowSeen ? "seen" : "unseen") << ", hand "
								 << (arm.bHandSeen ? "seen" : "unseen") << ", chosen elbow miss "
								 << arm.chosenElbowPx << " px";
		}
		if (arm.bHandMeasured)
		{
			MIKAN_LOG_INFO(kTag) << sideName << " fingers (lateral/proximal/intermediate/distal, deg), measured -> filtered:";
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				const FingerAngles& m= arm.measuredFingers[finger];
				const FingerAngles& f= arm.fingers[finger];
				MIKAN_LOG_INFO(kTag) << "  " << finger << ": " << degrees(m.lateral) << "/" << degrees(m.proximal)
									 << "/" << degrees(m.intermediate) << "/" << degrees(m.distal) << " -> "
									 << degrees(f.lateral) << "/" << degrees(f.proximal) << "/"
									 << degrees(f.intermediate) << "/" << degrees(f.distal);
			}
		}
		arms.push_back(armJson);
	}

	// The report
	std::error_code ec;
	std::filesystem::create_directories(outDir, ec);
	nlohmann::json report= {
		{"recording", recordingPath},
		{"frameIndex", frameIndex},
		{"cameraIndex", cameraIndex},
		{"iterations", kIterations},
		{"intervalMs", intervalMs},
		{"modelPath", config.avatar.modelPath},
		{"pose", AvatarJson::avatarPoseToJson(*skeleton, pose)},
		{"solveState", AvatarJson::directSolveStateToJson(state)},
		{"reprojection", joints},
		{"arms", arms},
	};
	{
		std::ofstream file(outDir / "pose-report.json");
		if (!file.is_open())
		{
			MIKAN_LOG_ERROR(kTag) << "Could not write " << (outDir / "pose-report.json");
			return 1;
		}
		file << report.dump(1, '\t');
	}

	// The picture: the frame from the recording's frame directory or the
	// dump's raw PNG, else black, with the landmarks and the posed skeleton,
	// beside the avatar rendered from the camera
	cv::Mat raw;
	const std::string framePath= FrameRecorder::makeFramePath(TrackingRecording::makeFrameDirectoryPath(recordingPath),
															 cameraIndex, cameraFrame.frameIndex);
	if (std::filesystem::exists(framePath))
		raw= cv::imread(framePath, cv::IMREAD_COLOR);
	if (raw.empty())
	{
		char rawName[64];
		snprintf(rawName, sizeof(rawName), "cam%d_raw.png", cameraIndex);
		const std::filesystem::path dumpRaw= PathUtils::utf8ToPath(recordingPath).parent_path() / rawName;
		if (std::filesystem::exists(dumpRaw))
			raw= cv::imread(dumpRaw.string(), cv::IMREAD_COLOR);
	}
	if (raw.empty())
		raw= cv::Mat(std::max(cameraModel.height, 1), std::max(cameraModel.width, 1), CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat annotated;
	DiagnosticDump::annotateFrame(raw, cameraFrame, annotated);
	AvatarOverlay::drawPosedSkeleton(annotated, *skeleton, pose, cameraToWorld, cameraModel);

	cv::Mat render;
	{
		HeadlessGlContext gl;
		if (gl.create("pose-report"))
		{
			Scene3dPanel scene;
			scene.setAvatar(loaded.model, skeleton, 1);
			scene.setAvatarPose(&pose);
			scene.setDirectSolve(&state, glm::translate(glm::mat4(1.f), glm::vec3(cameraToWorld[3])));
			SceneCameraMatch match;
			match.valid= true;
			match.camera= cameraModel;
			match.cameraToWorld= cameraToWorld;
			scene.setCameraMatch(match);
			if (!scene.captureFromCamera(cameraFrame, {}, {}, render))
				MIKAN_LOG_ERROR(kTag) << "The avatar render failed; the report shows the frame alone";
		}
	}
	const std::filesystem::path imagePath= outDir / "pose-report.png";
	if (!cv::imwrite(imagePath.string(), AvatarOverlay::makeCompareImage(annotated, render)))
	{
		MIKAN_LOG_ERROR(kTag) << "Could not write " << imagePath;
		return 1;
	}
	MIKAN_LOG_INFO(kTag) << "Wrote " << (outDir / "pose-report.json") << " and " << imagePath;
	return 0;
}
} // namespace

MIKAN_REGISTER_TEST("--pose-report",
					"Re-solve one frame of a direct-mode recording or dump and report the posed avatar against "
					"the landmarks (<recording.jsonl> [frameIndex] [outDir])",
					eTestCategory::Tool, runPoseReportTool);
