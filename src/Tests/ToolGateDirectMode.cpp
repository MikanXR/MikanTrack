#include "TestCommon.h"

#include "FrameRecorder.h"

// Direct-mode gate: re-runs the whole-body 3D model over a recording's raw
// frames from one camera and measures what the single-camera direct mode
// would retarget from - the arm segment directions - against the stereo
// rig's output of the same frames. The truth is the faithful replay (the
// triangulated wrists, the IMU forearms where recorded, the body solver's
// shoulders), carried into the camera's own frame, so every comparison is
// one model's view against the rig's view of the same arm.
//
//   --gate-direct <recording.jsonl>... [cameraIndex] [frames.csv]
//
// cameraIndex defaults to the recording's body-pose camera. The raw frames
// beside the recording are required (raw frame recording is opt-in).
//
// The model's depth is relative to the hips, so lifting its pixels into
// camera space needs a root depth. Three lifts are measured for each arm:
//   oracle   the root depth that best fits the truth joints, so the number
//            is the model's own direction error with the depth scale solved
//            away
//   live     the root depth from the pixel shoulder width against the
//            configured shoulder width, what the live mode would have
//   planar   the oracle depth with every relative depth zeroed, what a 2D
//            model gives, so the depth axis's contribution is visible
// An elbow flip is a bend vector (elbow off the shoulder-wrist line) pointing
// against the truth's, with both bends past kBendMinMeters.
//
// The gates cover what the truth measures independently of the model: the
// stereo wrist and the shoulder solved from its own rays and width. The
// truth elbow is not that: without an IMU it comes from the body solver,
// which picks a point on the bone circle by the body-pose camera's own 2D
// elbow, the same landmark under test, so the direction numbers and bend
// flips are reported but not gated until a take with the IMUs worn sets
// them. The limits are from the 2026-10-31 webcam-position take (head in
// frame, shallow tilt, hands raised toward the camera at 0.45 m): shoulders
// 9 px, wrists 58 px, live root depth 7 cm, depth sign 80 percent pooled.

namespace
{
constexpr float kScoreGate= 0.3f;
constexpr float kBendMinMeters= 0.05f;
// A wrist this much nearer or farther than the shoulder, by the truth, is a
// depth sign worth asking the model for
constexpr float kDepthSignMinMeters= 0.1f;

constexpr float kShoulderPxP50Gate= 15.f;
constexpr float kWristPxP50Gate= 80.f;
constexpr float kRootDepthErrCmP50Gate= 10.f;
constexpr float kDepthSignAgreePctGate= 75.f;

struct Stats
{
	std::vector<float> values;
	void add(float value) { values.push_back(value); }
	void append(const Stats& other) { values.insert(values.end(), other.values.begin(), other.values.end()); }
	int count() const { return (int)values.size(); }
	float percentile(float fraction) const
	{
		if (values.empty())
			return 0.f;
		std::vector<float> sorted= values;
		std::sort(sorted.begin(), sorted.end());
		const size_t index= std::min(sorted.size() - 1, (size_t)((float)sorted.size() * fraction));
		return sorted[index];
	}
	float p50() const { return percentile(0.5f); }
	float p90() const { return percentile(0.9f); }
};

struct LiftStats
{
	Stats upperArmDeg;
	Stats forearmDeg;
	Stats elbowErrCm;
	Stats wristErrCm;
	int bendFrames= 0;
	int bendFlips= 0;

	void append(const LiftStats& other)
	{
		upperArmDeg.append(other.upperArmDeg);
		forearmDeg.append(other.forearmDeg);
		elbowErrCm.append(other.elbowErrCm);
		wristErrCm.append(other.wristErrCm);
		bendFrames+= other.bendFrames;
		bendFlips+= other.bendFlips;
	}
};

struct SideStats
{
	// Model frames where the truth had a full arm, and how many of those the
	// model provided all three joints on
	int truthArmFrames= 0;
	int modelArmFrames= 0;

	LiftStats oracle;
	LiftStats live;
	LiftStats planar;

	// The model's pixels against the truth joints projected into the
	// camera: the planar agreement, which separates a depth disagreement
	// from a landmark one. The wrist's is the 2D touch-up's residual.
	Stats shoulderPx;
	Stats elbowPx;
	Stats wristPx;
	// Root depth: the live estimate against the oracle
	Stats rootDepthErrCm;
	// The model's wrist depth relative to its shoulder against the truth's,
	// on frames where the truth separates them by kDepthSignMinMeters: does
	// the sign agree, and how much of the difference does the model report
	int depthSignFrames= 0;
	int depthSignAgree= 0;
	Stats wristDepthScale;
	// Direction steps between consecutive model frames, the oracle lift, for
	// the model and for the truth
	Stats modelUpperArmStepDeg;
	Stats modelForearmStepDeg;
	Stats truthUpperArmStepDeg;
	Stats truthForearmStepDeg;

	void append(const SideStats& other)
	{
		truthArmFrames+= other.truthArmFrames;
		modelArmFrames+= other.modelArmFrames;
		oracle.append(other.oracle);
		live.append(other.live);
		planar.append(other.planar);
		shoulderPx.append(other.shoulderPx);
		elbowPx.append(other.elbowPx);
		wristPx.append(other.wristPx);
		rootDepthErrCm.append(other.rootDepthErrCm);
		depthSignFrames+= other.depthSignFrames;
		depthSignAgree+= other.depthSignAgree;
		wristDepthScale.append(other.wristDepthScale);
		modelUpperArmStepDeg.append(other.modelUpperArmStepDeg);
		modelForearmStepDeg.append(other.modelForearmStepDeg);
		truthUpperArmStepDeg.append(other.truthUpperArmStepDeg);
		truthForearmStepDeg.append(other.truthForearmStepDeg);
	}
};

struct CameraGeometry
{
	glm::dmat4 cameraFromWorld{1.0};
	float fx= 0.f, fy= 0.f, cx= 0.f, cy= 0.f;

	glm::vec3 toCamera(const glm::vec3& world) const
	{
		return glm::vec3(cameraFromWorld * glm::dvec4(glm::dvec3(world), 1.0));
	}
	bool project(const glm::vec3& camera, glm::vec2& outPx) const
	{
		if (camera.z < 1e-3f)
			return false;
		outPx= glm::vec2(fx * camera.x / camera.z + cx, fy * camera.y / camera.z + cy);
		return true;
	}
	// The ray through a pixel, scaled so z = 1
	glm::vec3 rayAtUnitDepth(const glm::vec2& px) const
	{
		return glm::vec3((px.x - cx) / fx, (px.y - cy) / fy, 1.f);
	}
};

// One arm's joints, shoulder, elbow, wrist
struct Arm
{
	glm::vec3 shoulder{0.f};
	glm::vec3 elbow{0.f};
	glm::vec3 wrist{0.f};

	glm::vec3 upperArm() const { return elbow - shoulder; }
	glm::vec3 forearm() const { return wrist - elbow; }
	// The elbow's offset from the shoulder-wrist line
	glm::vec3 bend() const
	{
		const glm::vec3 axis= wrist - shoulder;
		const float axisLength2= glm::dot(axis, axis);
		if (axisLength2 < 1e-9f)
			return glm::vec3(0.f);
		const float t= glm::dot(elbow - shoulder, axis) / axisLength2;
		return elbow - (shoulder + axis * t);
	}
};

// The model's arm: pixel rays and hip-relative depths
struct ModelArm
{
	glm::vec3 ray[3]; // shoulder, elbow, wrist at unit depth
	float relativeDepth[3];
	glm::vec2 px[3];

	Arm lift(float rootDepth, bool bUseDepth) const
	{
		Arm arm;
		glm::vec3* joints[3]= {&arm.shoulder, &arm.elbow, &arm.wrist};
		for (int i= 0; i < 3; ++i)
			*joints[i]= ray[i] * (rootDepth + (bUseDepth ? relativeDepth[i] : 0.f));
		return arm;
	}

	// The root depth that best fits the truth joints, least squares over the
	// three joints: each lifted joint is ray * (root + relative), linear in
	// the root
	float solveRootDepth(const Arm& truth) const
	{
		const glm::vec3* truthJoints[3]= {&truth.shoulder, &truth.elbow, &truth.wrist};
		float numerator= 0.f;
		float denominator= 0.f;
		for (int i= 0; i < 3; ++i)
		{
			numerator+= glm::dot(ray[i], *truthJoints[i]) - glm::dot(ray[i], ray[i]) * relativeDepth[i];
			denominator+= glm::dot(ray[i], ray[i]);
		}
		return denominator > 1e-9f ? numerator / denominator : 0.f;
	}
};

float angleBetweenDeg(const glm::vec3& a, const glm::vec3& b)
{
	const float lengths= glm::length(a) * glm::length(b);
	if (lengths < 1e-9f)
		return 0.f;
	return glm::degrees(acosf(std::max(-1.f, std::min(1.f, glm::dot(a, b) / lengths))));
}

void accumulateLift(const Arm& model, const Arm& truth, LiftStats& stats)
{
	stats.upperArmDeg.add(angleBetweenDeg(model.upperArm(), truth.upperArm()));
	stats.forearmDeg.add(angleBetweenDeg(model.forearm(), truth.forearm()));
	stats.elbowErrCm.add(glm::length(model.elbow - truth.elbow) * 100.f);
	stats.wristErrCm.add(glm::length(model.wrist - truth.wrist) * 100.f);

	const glm::vec3 modelBend= model.bend();
	const glm::vec3 truthBend= truth.bend();
	if (glm::length(modelBend) >= kBendMinMeters && glm::length(truthBend) >= kBendMinMeters)
	{
		stats.bendFrames++;
		if (glm::dot(modelBend, truthBend) < 0.f)
			stats.bendFlips++;
	}
}

// The root depth at which the lifted shoulders sit the configured width
// apart: |a * root + b| = width, the positive root of the quadratic
bool solveRootDepthFromShoulders(const glm::vec3& leftRay, float leftDepth, const glm::vec3& rightRay,
								 float rightDepth, float shoulderWidthMeters, float& outRootDepth)
{
	const glm::vec3 a= leftRay - rightRay;
	const glm::vec3 b= leftRay * leftDepth - rightRay * rightDepth;
	const float qa= glm::dot(a, a);
	const float qb= 2.f * glm::dot(a, b);
	const float qc= glm::dot(b, b) - shoulderWidthMeters * shoulderWidthMeters;
	if (qa < 1e-12f)
		return false;
	const float discriminant= qb * qb - 4.f * qa * qc;
	if (discriminant < 0.f)
		return false;
	outRootDepth= (-qb + sqrtf(discriminant)) / (2.f * qa);
	return outRootDepth > 0.f;
}

int defaultCameraIndex(const AppConfig& config)
{
	for (int cameraIndex= 0; cameraIndex < (int)config.cameraCount(); ++cameraIndex)
	{
		if (config.camera(cameraIndex).bodyPose.enabled)
			return cameraIndex;
	}
	return -1;
}

void logLift(const char* tag, const char* name, const LiftStats& stats)
{
	const float flipPct= stats.bendFrames > 0 ? 100.f * (float)stats.bendFlips / (float)stats.bendFrames : 0.f;
	MIKAN_LOG_INFO(tag) << "    " << name << ": upper arm p50 " << stats.upperArmDeg.p50() << " p90 "
						<< stats.upperArmDeg.p90() << " deg, forearm p50 " << stats.forearmDeg.p50() << " p90 "
						<< stats.forearmDeg.p90() << " deg, elbow p50 " << stats.elbowErrCm.p50() << " cm, wrist p50 "
						<< stats.wristErrCm.p50() << " cm, bend flips " << stats.bendFlips << "/" << stats.bendFrames
						<< " (" << flipPct << "%)";
}

void logSide(const char* tag, const char* name, const SideStats& side)
{
	MIKAN_LOG_INFO(tag) << "  " << name << ": truth arm on " << side.truthArmFrames
						<< " model frames, model arm on " << side.modelArmFrames;
	if (side.modelArmFrames == 0)
		return;
	logLift(tag, "oracle", side.oracle);
	logLift(tag, "live  ", side.live);
	logLift(tag, "planar", side.planar);
	MIKAN_LOG_INFO(tag) << "    planar px p50: shoulder " << side.shoulderPx.p50() << ", elbow " << side.elbowPx.p50()
						<< ", wrist " << side.wristPx.p50() << " (p90 " << side.wristPx.p90()
						<< "), live root depth err p50 " << side.rootDepthErrCm.p50() << " p90 "
						<< side.rootDepthErrCm.p90() << " cm (" << side.rootDepthErrCm.count() << " frames)";
	const float signPct=
		side.depthSignFrames > 0 ? 100.f * (float)side.depthSignAgree / (float)side.depthSignFrames : 0.f;
	MIKAN_LOG_INFO(tag) << "    wrist depth against the shoulder: sign agrees on " << side.depthSignAgree << "/"
						<< side.depthSignFrames << " (" << signPct << "%), scale p50 " << side.wristDepthScale.p50();
	MIKAN_LOG_INFO(tag) << "    step between model frames, upper arm model p50 " << side.modelUpperArmStepDeg.p50()
						<< " truth " << side.truthUpperArmStepDeg.p50() << " deg, forearm model "
						<< side.modelForearmStepDeg.p50() << " truth " << side.truthForearmStepDeg.p50() << " deg";
}

void writeCsvHeader(std::ofstream& csv)
{
	csv << "recording,frame,side,rootOracleM,rootLiveM,upperOracleDeg,forearmOracleDeg,upperLiveDeg,forearmLiveDeg,"
		   "upperPlanarDeg,forearmPlanarDeg,elbowOracleCm,shoulderPx,elbowPx,wristPx";
	const char* joints[3]= {"Shoulder", "Elbow", "Wrist"};
	for (const char* source : {"truth", "model"})
		for (const char* joint : joints)
			csv << "," << source << joint << "X," << source << joint << "Y," << source << joint << "Z";
	csv << "\n";
}
} // namespace

static int runGateDirectModeTool(const TestArgs& args)
{
	const char* tag= "gate-direct";

	std::vector<std::string> recordings;
	std::string csvPath;
	int requestedCamera= -1;
	for (const std::string& arg : args)
	{
		if (arg.size() > 6 && arg.substr(arg.size() - 6) == ".jsonl")
			recordings.push_back(arg);
		else if (arg.size() > 4 && arg.substr(arg.size() - 4) == ".csv")
			csvPath= arg;
		else
			requestedCamera= atoi(arg.c_str());
	}
	if (recordings.empty())
	{
		MIKAN_LOG_ERROR(tag) << "Usage: --gate-direct <recording.jsonl>... [cameraIndex] [frames.csv]";
		return 1;
	}
	std::ofstream csv;
	if (!csvPath.empty())
	{
		csv.open(csvPath, std::ios::trunc);
		if (!csv.is_open())
		{
			MIKAN_LOG_ERROR(tag) << "Cannot write " << csvPath;
			return 1;
		}
		writeCsvHeader(csv);
	}

	int result= 0;
	std::array<SideStats, 2> pooled{};
	Stats pooledInferenceMs;
	for (const std::string& path : recordings)
	{
		TrackingReplay replay;
		std::string error;
		if (!replay.load(path, error))
		{
			MIKAN_LOG_ERROR(tag) << path << ": load failed: " << error;
			result= 1;
			continue;
		}
		const AppConfig& config= replay.getRecordedConfig();
		const std::string recordingName= std::filesystem::path(path).filename().string();

		const int cameraIndex= requestedCamera >= 0 ? requestedCamera : defaultCameraIndex(config);
		if (cameraIndex < 0 || cameraIndex >= (int)config.cameraCount())
		{
			MIKAN_LOG_ERROR(tag) << path << ": no body-pose camera to default to; pass a cameraIndex";
			result= 1;
			continue;
		}
		const CameraProfile& profile= config.camera(cameraIndex);
		if (!profile.intrinsics.present || !profile.extrinsics.present)
		{
			MIKAN_LOG_ERROR(tag) << path << ": camera " << cameraIndex
								 << " has no intrinsics or extrinsics, nothing to compare in";
			result= 1;
			continue;
		}
		const std::string frameDirectory= TrackingRecording::makeFrameDirectoryPath(path);
		if (!std::filesystem::exists(frameDirectory))
		{
			MIKAN_LOG_ERROR(tag) << path << ": no raw frames beside the recording (" << frameDirectory << ")";
			result= 1;
			continue;
		}

		CameraGeometry geometry;
		geometry.cameraFromWorld= glm::inverse(profile.extrinsics.markerFromCamera);
		{
			const MikanMatrix3d& cameraMatrix= profile.intrinsics.intrinsics.undistorted_camera_matrix;
			geometry.fx= (float)cameraMatrix.x0;
			geometry.fy= (float)cameraMatrix.y1;
			geometry.cx= (float)cameraMatrix.z0;
			geometry.cy= (float)cameraMatrix.z1;
		}
		const BodyDimensions dimensions= makeBodyDimensions(config);

		BodyPoseTrackerConfig trackerConfig;
		trackerConfig.backend= eBodyPoseBackend::Rtmw3dWholeBody;
		trackerConfig.frameDivider= 1;
		trackerConfig.detectorIntervalFrames= profile.bodyPose.detectorIntervalFrames;
		BodyPoseTracker tracker;
		if (!tracker.load("models", "directml", trackerConfig))
		{
			MIKAN_LOG_ERROR(tag) << "Failed to load the whole-body model (models/rtmw3d_wholebody.onnx)";
			return 1;
		}

		replay.runAll();

		std::array<SideStats, 2> sides{};
		Stats inferenceMs;
		int framesRun= 0;
		int framesMissing= 0;
		// Previous model frame's oracle-lifted arm and truth arm per side,
		// for the step statistics
		bool bHaveLast[2]= {false, false};
		Arm lastModel[2];
		Arm lastTruth[2];

		for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
		{
			const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIndex);
			const RecordedFrame& recorded= *frame.recorded;
			const RecordedCameraInput* fresh= nullptr;
			for (const RecordedCameraInput& candidate : recorded.freshCameras)
			{
				if (candidate.cameraIndex == cameraIndex)
					fresh= &candidate;
			}
			if (fresh == nullptr)
				continue;

			const cv::Mat image=
				cv::imread(FrameRecorder::makeFramePath(frameDirectory, cameraIndex, fresh->frameIndex),
						   cv::IMREAD_COLOR);
			if (image.empty())
			{
				framesMissing++;
				continue;
			}
			framesRun++;

			BodyPoseObservation observation;
			const auto start= std::chrono::steady_clock::now();
			tracker.process(image, observation);
			inferenceMs.add(
				(float)std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
			if (!observation.valid || !observation.hasDepth)
				continue;

			// The live root depth from the shoulders, shared by both arms
			bool bHaveLiveRoot= false;
			float liveRootDepth= 0.f;
			{
				const int leftSlot= (int)ePoseLandmark::LEFT_SHOULDER;
				const int rightSlot= (int)ePoseLandmark::RIGHT_SHOULDER;
				if (observation.isProvided(leftSlot) && observation.isProvided(rightSlot) &&
					observation.visibility[leftSlot] >= kScoreGate && observation.visibility[rightSlot] >= kScoreGate)
				{
					bHaveLiveRoot= solveRootDepthFromShoulders(
						geometry.rayAtUnitDepth(glm::vec2(observation.imagePoints[leftSlot])),
						observation.imagePoints[leftSlot].z,
						geometry.rayAtUnitDepth(glm::vec2(observation.imagePoints[rightSlot])),
						observation.imagePoints[rightSlot].z, dimensions.shoulderWidthMeters, liveRootDepth);
				}
			}

			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				SideStats& side= sides[sideIndex];
				const HandPose& truthPose= frame.replayedFused.poses[sideIndex];
				if (!truthPose.tracked || !truthPose.hasWorldPose || !truthPose.hasForearmPose ||
					!truthPose.hasShoulder)
					continue;
				side.truthArmFrames++;

				const int slots[3]= {
					sideIndex == 0 ? (int)ePoseLandmark::LEFT_SHOULDER : (int)ePoseLandmark::RIGHT_SHOULDER,
					sideIndex == 0 ? (int)ePoseLandmark::LEFT_ELBOW : (int)ePoseLandmark::RIGHT_ELBOW,
					sideIndex == 0 ? (int)ePoseLandmark::LEFT_WRIST : (int)ePoseLandmark::RIGHT_WRIST,
				};
				bool bProvided= true;
				ModelArm modelArm;
				for (int i= 0; i < 3; ++i)
				{
					if (!observation.isProvided(slots[i]) || observation.visibility[slots[i]] < kScoreGate)
					{
						bProvided= false;
						break;
					}
					modelArm.ray[i]= geometry.rayAtUnitDepth(glm::vec2(observation.imagePoints[slots[i]]));
					modelArm.relativeDepth[i]= observation.imagePoints[slots[i]].z;
				}
				if (!bProvided)
					continue;
				for (int i= 0; i < 3; ++i)
					modelArm.px[i]= glm::vec2(observation.imagePoints[slots[i]]);
				side.modelArmFrames++;

				Arm truth;
				truth.shoulder= geometry.toCamera(truthPose.shoulderPositionWorld);
				truth.elbow= geometry.toCamera(truthPose.getElbowPositionWorld(dimensions.forearmLengthMeters));
				truth.wrist= geometry.toCamera(truthPose.getWristPositionWorld());

				const float oracleRootDepth= modelArm.solveRootDepth(truth);
				const Arm oracle= modelArm.lift(oracleRootDepth, true);
				const Arm planar= modelArm.lift(oracleRootDepth, false);
				accumulateLift(oracle, truth, side.oracle);
				accumulateLift(planar, truth, side.planar);
				Arm live;
				if (bHaveLiveRoot)
				{
					live= modelArm.lift(liveRootDepth, true);
					accumulateLift(live, truth, side.live);
					side.rootDepthErrCm.add(std::fabs(liveRootDepth - oracleRootDepth) * 100.f);
				}

				// Relative depths lift as root + relative on unit-z rays, so
				// the model's wrist-to-shoulder depth difference is its own,
				// whatever root depth is chosen
				{
					const float truthDepthDelta= truth.wrist.z - truth.shoulder.z;
					const float modelDepthDelta= modelArm.relativeDepth[2] - modelArm.relativeDepth[0];
					if (std::fabs(truthDepthDelta) >= kDepthSignMinMeters)
					{
						side.depthSignFrames++;
						if ((modelDepthDelta < 0.f) == (truthDepthDelta < 0.f))
							side.depthSignAgree++;
						side.wristDepthScale.add(modelDepthDelta / truthDepthDelta);
					}
				}

				float planarPx[3]= {0.f, 0.f, 0.f};
				{
					const glm::vec3* truthJoints[3]= {&truth.shoulder, &truth.elbow, &truth.wrist};
					Stats* planarStats[3]= {&side.shoulderPx, &side.elbowPx, &side.wristPx};
					for (int i= 0; i < 3; ++i)
					{
						glm::vec2 truthPx;
						if (geometry.project(*truthJoints[i], truthPx))
						{
							planarPx[i]= glm::length(modelArm.px[i] - truthPx);
							planarStats[i]->add(planarPx[i]);
						}
					}
				}

				if (bHaveLast[sideIndex])
				{
					side.modelUpperArmStepDeg.add(angleBetweenDeg(oracle.upperArm(), lastModel[sideIndex].upperArm()));
					side.modelForearmStepDeg.add(angleBetweenDeg(oracle.forearm(), lastModel[sideIndex].forearm()));
					side.truthUpperArmStepDeg.add(angleBetweenDeg(truth.upperArm(), lastTruth[sideIndex].upperArm()));
					side.truthForearmStepDeg.add(angleBetweenDeg(truth.forearm(), lastTruth[sideIndex].forearm()));
				}
				bHaveLast[sideIndex]= true;
				lastModel[sideIndex]= oracle;
				lastTruth[sideIndex]= truth;

				if (csv.is_open())
				{
					csv << recordingName << "," << frameIndex << "," << sideIndex << "," << oracleRootDepth << ","
						<< (bHaveLiveRoot ? liveRootDepth : 0.f) << ","
						<< angleBetweenDeg(oracle.upperArm(), truth.upperArm()) << ","
						<< angleBetweenDeg(oracle.forearm(), truth.forearm()) << ","
						<< (bHaveLiveRoot ? angleBetweenDeg(live.upperArm(), truth.upperArm()) : 0.f) << ","
						<< (bHaveLiveRoot ? angleBetweenDeg(live.forearm(), truth.forearm()) : 0.f) << ","
						<< angleBetweenDeg(planar.upperArm(), truth.upperArm()) << ","
						<< angleBetweenDeg(planar.forearm(), truth.forearm()) << ","
						<< glm::length(oracle.elbow - truth.elbow) * 100.f << "," << planarPx[0] << "," << planarPx[1]
						<< "," << planarPx[2];
					const Arm* arms[2]= {&truth, &oracle};
					for (const Arm* arm : arms)
					{
						const glm::vec3* armJoints[3]= {&arm->shoulder, &arm->elbow, &arm->wrist};
						for (const glm::vec3* joint : armJoints)
							csv << "," << joint->x << "," << joint->y << "," << joint->z;
					}
					csv << "\n";
				}
			}
		}

		MIKAN_LOG_INFO(tag) << recordingName << " (" << replay.getFrameCount() << " frames, camera " << cameraIndex
							<< ", " << framesRun << " raw frames run, " << framesMissing << " missing, ep "
							<< tracker.activeEp() << ", inference p50 " << inferenceMs.p50() << " ms):";
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			logSide(tag, sideIndex == 0 ? "Left" : "Right", sides[sideIndex]);
			pooled[sideIndex].append(sides[sideIndex]);
		}
		pooledInferenceMs.append(inferenceMs);
	}

	MIKAN_LOG_INFO(tag) << "Pooled over " << recordings.size() << " recording(s), inference p50 "
						<< pooledInferenceMs.p50() << " ms:";
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		logSide(tag, sideIndex == 0 ? "Left" : "Right", pooled[sideIndex]);

	SideStats all;
	all.append(pooled[0]);
	all.append(pooled[1]);
	logSide(tag, "Both", all);

	// Gates over everything, both sides pooled
	if (all.modelArmFrames == 0)
	{
		MIKAN_LOG_ERROR(tag) << "No arm compared on any frame; nothing to gate";
		return 1;
	}
	const float signPct=
		all.depthSignFrames > 0 ? 100.f * (float)all.depthSignAgree / (float)all.depthSignFrames : 0.f;
	struct Gate
	{
		const char* name;
		float value;
		float limit;
		bool bAtMost;
	};
	const Gate gates[]= {
		{"shoulder px p50", all.shoulderPx.p50(), kShoulderPxP50Gate, true},
		{"wrist px p50", all.wristPx.p50(), kWristPxP50Gate, true},
		{"live root depth err cm p50", all.rootDepthErrCm.p50(), kRootDepthErrCmP50Gate, true},
		{"wrist depth sign agreement %", signPct, kDepthSignAgreePctGate, false},
	};
	bool bAllPassed= true;
	for (const Gate& gate : gates)
	{
		const bool bPassed= gate.bAtMost ? gate.value <= gate.limit : gate.value >= gate.limit;
		bAllPassed= bAllPassed && bPassed;
		MIKAN_LOG_INFO(tag) << "  gate " << gate.name << ": " << gate.value << " (" << (gate.bAtMost ? "at most " : "at least ")
							<< gate.limit << ") " << (bPassed ? "pass" : "FAIL");
	}
	if (!bAllPassed)
	{
		MIKAN_LOG_ERROR(tag) << "Gate(s) failed";
		return 1;
	}
	return result;
}

MIKAN_REGISTER_TEST("--gate-direct", "Whole-body 3D arm directions on one camera against the stereo rig's output",
					eTestCategory::Tool, runGateDirectModeTool);
