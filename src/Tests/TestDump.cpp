#include "TestCommon.h"

#include "AvatarSkeleton.h"
#include "DirectRetarget.h"
#include "SyntheticAvatar.h"
#include "TrackingReplay.h"

static int runDumpTest(const TestArgs& args)
{
	int result= 0;

	// Synthetic camera result: one tracked left hand + a detection box
	CameraFrameResult cameraResult;
	cameraResult.cameraIndex= 0;
	cameraResult.valid= true;
	cameraResult.timestampMs= 1000.0;
	cameraResult.hasExtrinsics= true;

	TrackingFrameResult& frame= cameraResult.result;
	frame.frameIndex= 42;
	frame.timestampMs= 1000.0;
	frame.frameWidth= 128;
	frame.frameHeight= 128;

	TrackedHand& hand= frame.hands[(int)eHandSide::Left];
	hand.tracked= true;
	hand.side= eHandSide::Left;
	hand.presence= 0.9f;
	hand.handednessScore= 0.1f;
	hand.hasWorldSpace= true;
	for (int landmark= 0; landmark < HAND_LANDMARK_COUNT; ++landmark)
	{
		hand.imagePoints[landmark]= glm::vec3(20.f + landmark * 4.f, 30.f + landmark * 3.f, 0.f);
		hand.worldPoints[landmark]= glm::vec3(0.1f, 0.05f, 0.1f + landmark * 0.001f);
	}

	HandPose& pose= frame.poses[(int)eHandSide::Left];
	pose.tracked= true;
	pose.side= eHandSide::Left;
	pose.presence= 0.9f;
	pose.hasWorldPose= true;
	pose.palmPositionWorld= glm::vec3(0.1f, 0.05f, 0.1f);
	pose.fingers[1].proximal= 0.5f;

	DetectionBox box;
	box.corners= {glm::vec2(10, 10), glm::vec2(60, 10), glm::vec2(60, 60), glm::vec2(10, 60)};
	frame.palmDetections.push_back(box);

	FusionDiagnostics diagnostics;
	diagnostics.totalObservations= 1;
	{
		FusionDiagnostics::Cluster cluster;
		cluster.palmWorld= pose.palmPositionWorld;
		cluster.assignedSide= (int)eHandSide::Left;
		FusionDiagnostics::Observation observation;
		observation.cameraIndex= 0;
		observation.labeledSide= (int)eHandSide::Left;
		observation.weight= 0.9f;
		cluster.observations.push_back(observation);
		diagnostics.clusters.push_back(cluster);
	}

	DiagnosticDump dump;
	const int dominant[2]= {0, -1};
	// One side with a live IMU, one without - the writer must emit both
	DiagImuState imuStates[2];
	imuStates[0].deviceConnected= true;
	imuStates[0].streaming= true;
	imuStates[0].calibrated= true;
	imuStates[0].orientationValid= true;
	imuStates[0].sampleRateHz= 200.f;
	imuStates[0].millisecondsSinceLastSample= 4.0;
	imuStates[0].forearmAxisConsistency= 0.88f;
	imuStates[0].armAxisDominance= 0.94f;
	imuStates[0].gyroBiasDegreesPerSecond= glm::vec3(0.1f, -0.2f, 0.3f);
	imuStates[0].yawSigmaRadians= 0.02f;
	for (int record= 0; record < 3; ++record)
		dump.record({&cameraResult}, frame, diagnostics, dominant, 1.02f, imuStates);

	cv::Mat testFrame(128, 128, CV_8UC3, cv::Scalar(40, 40, 40));
	DiagCameraSnapshot snapshot;
	snapshot.lastResult= &cameraResult;
	snapshot.frame= &testFrame;
	snapshot.deviceFps= 30.f;
	snapshot.droppedFrames= 1;
	snapshot.activeEp= "test";
	snapshot.trackingEnabled= true;

	const std::filesystem::path dumpDir=
		std::filesystem::temp_directory_path() / "mikanmediapipe_test_dump";
	std::filesystem::remove_all(dumpDir);

	AppConfig config;
	std::vector<DiagImuRawSample> rawImu[2];
	DiagImuRawSample rawSample;
	rawSample.timestampMs= 12.0;
	rawSample.acceleration= glm::vec3(0.f, 0.f, 9.8f);
	rawSample.angularVelocity= glm::vec3(0.1f, -0.2f, 0.3f);
	rawImu[0].push_back(rawSample);

	DiagImuCapture lastCapture[2];
	lastCapture[0].present= true;
	lastCapture[0].poseSamples= 240;
	lastCapture[0].poseSpreadDegrees= 12.5f;
	lastCapture[0].motionUsable= true;
	lastCapture[0].axisDominance= 0.93f;

	bool bOk= dump.write(dumpDir.string(), {snapshot}, frame, config.toJsonString(), rawImu,
						 lastCapture);
	bOk&= std::filesystem::exists(dumpDir / "dump.json");
	bOk&= std::filesystem::exists(dumpDir / "cam0_raw.png");
	bOk&= std::filesystem::exists(dumpDir / "cam0_annotated.png");

	// Sanity-check the JSON payload: all top-level sections present,
	// history depth matches, affinity table serialized
	if (bOk)
	{
		std::ifstream jsonFile(dumpDir / "dump.json");
		const std::string content(
			(std::istreambuf_iterator<char>(jsonFile)), std::istreambuf_iterator<char>());
		for (const char* needle :
			 {"\"config\"", "\"cameras\"", "\"fusedSnapshot\"", "\"history\"", "\"affinity\"",
			  "\"imagePoints\"", "\"assignedSide\"", "\"imu\"", "\"imuRaw\"", "\"imuLastCapture\"", "\"filterOrientation\"",
			  "\"gravityAcceptRatio\"", "\"armAxisDominance\"",
			  "\"forearmAxisConsistency\""})
		{
			if (content.find(needle) == std::string::npos)
			{
				MIKAN_LOG_ERROR("test-dump") << "dump.json missing section " << needle;
				bOk= false;
			}
		}
	}

	// Outside direct mode the avatar block is null and no avatar file is written
	if (bOk)
	{
		std::ifstream jsonFile(dumpDir / "dump.json");
		const std::string content(
			(std::istreambuf_iterator<char>(jsonFile)), std::istreambuf_iterator<char>());
		if (content.find("\"avatar\": null") == std::string::npos)
		{
			MIKAN_LOG_ERROR("test-dump") << "dump.json without a direct pose should carry \"avatar\": null";
			bOk= false;
		}
		if (std::filesystem::exists(dumpDir / "cam0_avatar.png") || std::filesystem::exists(dumpDir / "dump.jsonl"))
		{
			MIKAN_LOG_ERROR("test-dump") << "no avatar render or record frame was given, yet their files exist";
			bOk= false;
		}
	}

	// A direct-mode dump: the posed avatar over its camera, its render and
	// the compare image, the solve state, and the frame as a one-frame
	// recording that replay loads
	if (bOk)
	{
		const std::filesystem::path directDir=
			std::filesystem::temp_directory_path() / "mikanmediapipe_test_dump_direct";
		std::filesystem::remove_all(directDir);

		const AvatarModel model= SyntheticAvatar::makeModel(eVrmVersion::Vrm1, false);
		const AvatarSkeleton skeleton(model);
		AvatarPose avatarPose;
		avatarPose.valid= true;
		avatarPose.timestampMs= 1000.0;
		for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
			avatarPose.bones[index].present= skeleton.getBone((eHumanoidBone)index).present;
		DirectRetarget::SolveState state;
		state.camera.fx= 100.f;
		state.camera.fy= 100.f;
		state.camera.cx= 64.f;
		state.camera.cy= 64.f;
		state.camera.width= 128;
		state.camera.height= 128;
		state.torsoValid= true;
		state.shoulder[0]= glm::vec3(1.2f, 0.2f, 0.f);
		state.shoulder[1]= glm::vec3(1.2f, -0.2f, 0.f);
		state.shoulderMidpoint= glm::vec3(1.2f, 0.f, 0.f);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			DirectRetarget::SolveState::Arm& arm= state.arms[sideIndex];
			arm.chain.valid= true;
			arm.chain.shoulder= state.shoulder[sideIndex];
			arm.chain.elbow= arm.chain.shoulder - glm::vec3(0.f, 0.f, 0.25f);
			arm.chain.wrist= arm.chain.elbow - glm::vec3(0.f, 0.f, 0.25f);
			arm.bDepthChosen= true;
			arm.chosenDepth= 1.3f;
			arm.filteredDepth= 1.31f;
		}

		DiagAvatarSnapshot avatar;
		avatar.skeleton= &skeleton;
		avatar.pose= &avatarPose;
		avatar.solveState= &state;
		avatar.modelPath= "synthetic.vrm";
		avatar.cameraIndex= 0;
		avatar.render= cv::Mat(128, 128, CV_8UC3, cv::Scalar(20, 60, 20));

		RecordedFrame recordFrame;
		recordFrame.seq= 0;
		recordFrame.nowTimestampMs= 1000.0;
		RecordedCameraInput input;
		input.cameraIndex= 0;
		input.timestampMs= 1000.0;
		input.valid= true;
		input.frameIndex= 42;
		input.frameWidth= 128;
		input.frameHeight= 128;
		input.hands[0].tracked= true;
		input.hands[0].imagePoints= hand.imagePoints;
		recordFrame.freshCameras.push_back(input);

		AppConfig directConfig;
		directConfig.direct.enabled= true;
		directConfig.direct.cameraIndex= 0;
		bool bDirectOk= dump.write(directDir.string(), {snapshot}, frame, directConfig.toJsonString(), rawImu,
								   lastCapture, &recordFrame, &avatar);
		for (const char* name : {"dump.json", "cam0_raw.png", "cam0_annotated.png", "cam0_avatar.png",
								 "cam0_compare.png", "dump.jsonl"})
		{
			if (!std::filesystem::exists(directDir / name))
			{
				MIKAN_LOG_ERROR("test-dump") << "direct dump missing " << name;
				bDirectOk= false;
			}
		}
		if (bDirectOk)
		{
			std::ifstream jsonFile(directDir / "dump.json");
			const std::string content(
				(std::istreambuf_iterator<char>(jsonFile)), std::istreambuf_iterator<char>());
			for (const char* needle : {"\"avatar\"", "\"solveState\"", "\"cameraToWorld\"", "\"LeftUpperArm\"",
									   "\"chosenDepth\"", "\"compare\""})
			{
				if (content.find(needle) == std::string::npos)
				{
					MIKAN_LOG_ERROR("test-dump") << "direct dump.json missing " << needle;
					bDirectOk= false;
				}
			}
			// The compare image is the frame beside the render
			const cv::Mat compare= cv::imread((directDir / "cam0_compare.png").string());
			if (compare.cols != 256 || compare.rows != 128)
			{
				MIKAN_LOG_ERROR("test-dump") << "cam0_compare.png is " << compare.cols << "x" << compare.rows
											 << ", expected 256x128";
				bDirectOk= false;
			}
			// The one-frame recording replays as a direct-mode take
			TrackingReplay replay;
			std::string error;
			if (!replay.load((directDir / "dump.jsonl").string(), error))
			{
				MIKAN_LOG_ERROR("test-dump") << "dump.jsonl failed to load: " << error;
				bDirectOk= false;
			}
			else if (replay.getFrameCount() != 1 || !replay.getRecordedConfig().direct.enabled ||
					 replay.getFrame(0).recorded->freshCameras.size() != 1 ||
					 replay.getFrame(0).recorded->freshCameras[0].frameIndex != 42)
			{
				MIKAN_LOG_ERROR("test-dump") << "dump.jsonl did not round-trip the frame";
				bDirectOk= false;
			}
		}
		MIKAN_LOG_INFO("test-dump") << "direct dump dir: " << directDir.string() << " ok=" << bDirectOk;
		if (bDirectOk)
			std::filesystem::remove_all(directDir);
		bOk&= bDirectOk;
	}

	MIKAN_LOG_INFO("test-dump") << "dump dir: " << dumpDir.string() << " ok=" << bOk;
	if (!bOk)
	{
		MIKAN_LOG_ERROR("test-dump") << "FAILED";
		result= 1;
	}
	else
	{
		std::filesystem::remove_all(dumpDir);
		MIKAN_LOG_INFO("test-dump") << "All dump checks passed";
	}

	return result;
}

MIKAN_REGISTER_TEST("--test-dump", "Diagnostic dump writer + schema", eTestCategory::SelfTest, runDumpTest);
