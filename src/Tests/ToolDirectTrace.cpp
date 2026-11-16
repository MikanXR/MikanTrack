#include "TestCommon.h"

#include <algorithm>
#include <fstream>

#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "DirectRetarget.h"
#include "PathUtils.h"
#include "RetargetChains.h"
#include "RetargetStage.h"
#include "TrackingReplay.h"
#include "VrmLoader.h"

// Solves every frame of a direct-mode take through the retarget stage, as the
// app solved it, and writes the solve state per frame: the instrument for what
// a still frame cannot show, the shoulders' jitter, an elbow that pops between
// frames, and how an arm leaves and returns to rest.
//
//   --direct-trace <recording.jsonl> [out.csv]
//
// The CSV carries per frame the shoulders, the spine's turn, and per arm the
// chain, the elbow's direction weight and the rest blend. The log carries the
// take's summary: the shoulder midpoint's step per frame, the frames an elbow
// sits inboard of its shoulder and how many of those have the wrist outboard
// of the elbow (an arm across the body has the elbow inboard too, with the
// wrist further in), and the elbow's turns about the shoulder to wrist line
// past kSwivelJumpDegrees between consecutive frames. It also counts the
// placed frames whose chain stands exactly as the frame before, an arm the
// frame could not solve and held.
namespace
{
constexpr const char* kTag= "direct-trace";
constexpr float kInboardMeters= 0.05f;
constexpr float kSwivelJumpDegrees= 30.f;

float percentile(std::vector<float> values, float fraction)
{
	if (values.empty())
		return 0.f;
	std::sort(values.begin(), values.end());
	const size_t index= std::min(values.size() - 1, (size_t)(fraction * (float)(values.size() - 1) + 0.5f));
	return values[index];
}

// The elbow's offset from the shoulder to wrist line, unit, zero on a
// straight arm
glm::vec3 swivelDirection(const DirectRetarget::ArmChain& chain)
{
	const glm::vec3 span= chain.wrist - chain.shoulder;
	const float spanSquared= glm::dot(span, span);
	if (spanSquared < 1e-8f)
		return glm::vec3(0.f);
	const glm::vec3 fromShoulder= chain.elbow - chain.shoulder;
	const glm::vec3 offset= fromShoulder - span * (glm::dot(fromShoulder, span) / spanSquared);
	const float offsetLength= glm::length(offset);
	return offsetLength > 1e-4f ? offset / offsetLength : glm::vec3(0.f);
}

int runDirectTraceTool(const TestArgs& args)
{
	if (args.empty())
	{
		MIKAN_LOG_ERROR(kTag) << "Usage: --direct-trace <recording.jsonl> [out.csv]";
		return 1;
	}
	const std::string recordingPath= args[0];
	std::filesystem::path outPath= PathUtils::utf8ToPath(recordingPath);
	outPath.replace_extension(".direct-trace.csv");
	if (args.size() > 1)
		outPath= PathUtils::utf8ToPath(args[1]);

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
	const std::shared_ptr<const AvatarSkeleton> skeleton= std::make_shared<AvatarSkeleton>(*loaded.model);

	const RetargetChains chains= resolveRetargetChains(config, rig);
	RetargetStage stage;
	stage.setConfig(makeRetargetStageConfig(config, rig, chains, skeleton));
	replay.runAll();

	std::ofstream csv(outPath);
	if (!csv)
	{
		MIKAN_LOG_ERROR(kTag) << "Cannot write " << outPath;
		return 1;
	}
	csv << "frame,tMs,torso,shLx,shLy,shLz,shRx,shRy,shRz,spineDeg";
	for (const char* side : {"L", "R"})
	{
		csv << "," << side << "chain," << side << "elx," << side << "ely," << side << "elz," << side << "wrx," << side
			<< "wry," << side << "wrz," << side << "elbowSeen," << side << "dirWeight," << side << "resting," << side
			<< "restBlend," << side << "swivelStepDeg";
	}
	csv << "\n";

	const MikanMatrix3d& cameraMatrix= config.camera(cameraIndex).intrinsics.intrinsics.undistorted_camera_matrix;
	const TrackingFrameResult::HeadPose noPhoneHead;
	RetargetResult result;
	std::vector<float> midpointStepsMm;
	bool bPreviousTorso= false;
	glm::vec3 previousMidpoint(0.f);
	glm::vec3 previousSwivel[2]= {glm::vec3(0.f), glm::vec3(0.f)};
	int armFrames[2]= {0, 0};
	int inboardFrames[2]= {0, 0};
	int inwardFrames[2]= {0, 0};
	int stalledFrames[2]= {0, 0};
	DirectRetarget::ArmChain previousChain[2];
	int swivelJumps[2]= {0, 0};
	int solvedFrames= 0;
	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIndex);
		if (cameraIndex >= (int)frame.perCamera.size() || !frame.perCameraValid[cameraIndex])
			continue;
		const TrackingFrameResult& cameraFrame= frame.perCamera[cameraIndex];
		DirectCameraModel cameraModel;
		cameraModel.fx= (float)cameraMatrix.x0;
		cameraModel.fy= (float)cameraMatrix.y1;
		cameraModel.cx= (float)cameraMatrix.z0;
		cameraModel.cy= (float)cameraMatrix.z1;
		cameraModel.width= cameraFrame.frameWidth;
		cameraModel.height= cameraFrame.frameHeight;
		stage.solveDirect(cameraFrame, cameraModel, noPhoneHead, cameraFrame.timestampMs, result);
		const DirectRetarget::SolveState& state= result.directState;
		solvedFrames++;

		const float spineDegrees=
			glm::degrees(2.f * acosf(std::clamp(std::fabs(state.spineDelta.w), 0.f, 1.f)));
		csv << frameIndex << "," << std::fixed << cameraFrame.timestampMs << std::defaultfloat << ","
			<< (state.torsoValid ? 1 : 0);
		for (const glm::vec3& shoulder : state.shoulder)
			csv << "," << shoulder.x << "," << shoulder.y << "," << shoulder.z;
		csv << "," << spineDegrees;
		if (state.torsoValid)
		{
			if (bPreviousTorso)
				midpointStepsMm.push_back(glm::distance(state.shoulderMidpoint, previousMidpoint) * 1000.f);
			previousMidpoint= state.shoulderMidpoint;
		}
		bPreviousTorso= state.torsoValid;

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::SolveState::Arm& arm= state.arms[sideIndex];
			const DirectRetarget::ArmChain& chain= arm.chain;
			float swivelStepDegrees= 0.f;
			glm::vec3 swivel(0.f);
			if (chain.valid)
			{
				armFrames[sideIndex]++;
				// A chain that stands exactly as it was: the frame did not
				// solve the arm, and it is held
				const DirectRetarget::ArmChain& previous= previousChain[sideIndex];
				if (previous.valid && chain.elbow == previous.elbow && chain.wrist == previous.wrist)
					stalledFrames[sideIndex]++;
				// Toward the body's midline: the left arm's inboard is world -Y
				const float inboard= (chain.elbow.y - chain.shoulder.y) * (sideIndex == 0 ? -1.f : 1.f);
				if (inboard > kInboardMeters)
				{
					inboardFrames[sideIndex]++;
					const float wristOutboard= (chain.wrist.y - chain.elbow.y) * (sideIndex == 0 ? 1.f : -1.f);
					if (wristOutboard > 0.f)
						inwardFrames[sideIndex]++;
				}
				swivel= swivelDirection(chain);
				if (glm::dot(swivel, swivel) > 0.f && glm::dot(previousSwivel[sideIndex], previousSwivel[sideIndex]) > 0.f)
				{
					swivelStepDegrees= glm::degrees(
						acosf(std::clamp(glm::dot(swivel, previousSwivel[sideIndex]), -1.f, 1.f)));
					if (swivelStepDegrees > kSwivelJumpDegrees)
						swivelJumps[sideIndex]++;
				}
			}
			previousSwivel[sideIndex]= swivel;
			previousChain[sideIndex]= chain;
			csv << "," << (chain.valid ? 1 : 0) << "," << chain.elbow.x << "," << chain.elbow.y << "," << chain.elbow.z
				<< "," << chain.wrist.x << "," << chain.wrist.y << "," << chain.wrist.z << ","
				<< (arm.bElbowSeen ? 1 : 0) << "," << arm.elbowDirectionWeight << "," << (arm.bResting ? 1 : 0) << ","
				<< arm.restBlend << "," << swivelStepDegrees;
		}
		csv << "\n";
	}

	MIKAN_LOG_INFO(kTag) << recordingPath << ": " << solvedFrames << " frames solved of " << replay.getFrameCount();
	MIKAN_LOG_INFO(kTag) << "shoulder midpoint step per frame: p50 " << percentile(midpointStepsMm, 0.5f) << " mm, p95 "
						 << percentile(midpointStepsMm, 0.95f) << " mm, max " << percentile(midpointStepsMm, 1.f)
						 << " mm";
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		MIKAN_LOG_INFO(kTag) << (sideIndex == 0 ? "L" : "R") << " arm: " << armFrames[sideIndex] << " frames placed, "
							 << stalledFrames[sideIndex] << " of them standing as the frame before, "
							 << inboardFrames[sideIndex] << " with the elbow over " << kInboardMeters * 100.f
							 << " cm inboard of the shoulder (" << inwardFrames[sideIndex]
							 << " with the wrist outboard of the elbow), " << swivelJumps[sideIndex] << " elbow turns over "
							 << kSwivelJumpDegrees << " degrees in a frame";
	}
	MIKAN_LOG_INFO(kTag) << "Wrote " << outPath;
	return 0;
}
} // namespace

MIKAN_REGISTER_TEST("--direct-trace",
					"Solve every frame of a direct-mode take and write the solve state per frame, with the shoulder "
					"jitter and elbow pop summary (<recording.jsonl> [out.csv])",
					eTestCategory::Tool, runDirectTraceTool);
