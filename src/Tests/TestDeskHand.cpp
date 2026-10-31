#include "TestCommon.h"

#include "DeskHandCalibrator.h"

// A flat synthetic hand on a synthetic desk seen by one calibrated camera:
// the plane lift has to measure the hand's bones back, the flatness gates
// have to refuse a tilted hand, and the lift depth scale has to read the
// disagreement between the landmark model's articulation and the pixels.

namespace
{
constexpr float kFx= 700.f;
constexpr float kFy= 700.f;
constexpr float kCx= 640.f;
constexpr float kCy= 360.f;

// Proportions measured from stereo on a real hand, not the landmark model's
HandSkeleton makeTruthSkeleton()
{
	HandSkeleton skeleton;
	const float baseY[FINGER_COUNT]= {0.045f, 0.03f, 0.f, -0.01f, -0.03f};
	const float baseX[FINGER_COUNT]= {-0.01f, 0.035f, 0.0425f, 0.035f, 0.03f};
	const float proximal[FINGER_COUNT]= {0.038f, 0.0415f, 0.0441f, 0.0401f, 0.032f};
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		skeleton.baseInPalm[finger]= glm::vec3(baseX[finger], baseY[finger], 0.f);
		skeleton.phalanxLengths[finger]= {proximal[finger], 0.025f, 0.020f};
	}
	skeleton.neutralDirInPalm= HandPoseModel::makeDefaultNeutralDirections(skeleton);
	return skeleton;
}

std::array<glm::vec3, HAND_LANDMARK_COUNT> buildLandmarks(
	const glm::mat4& palmTransform,
	const HandSkeleton& skeleton,
	const std::array<FingerAngles, FINGER_COUNT>& angles)
{
	std::array<std::array<glm::vec3, 4>, FINGER_COUNT> joints;
	HandPoseModel::buildFingerJoints(palmTransform, skeleton, angles, joints);

	std::array<glm::vec3, HAND_LANDMARK_COUNT> points{};
	const glm::vec3 wristInPalm(-skeleton.baseInPalm[(int)eFinger::Middle].x, 0.f, 0.f);
	points[(int)eHandLandmark::WRIST]= glm::vec3(palmTransform * glm::vec4(wristInPalm, 1.f));
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
		for (int joint= 0; joint < 4; ++joint)
			points[FINGER_JOINTS[finger][joint]]= joints[finger][joint];
	return points;
}

// A webcam on the monitor in front of the user: above the desk at +X,
// looking back down at the marker. OpenCV camera axes: +Z forward, +Y down.
glm::dmat4 makeMarkerFromCamera()
{
	const glm::dvec3 position(0.7, 0.05, 0.4);
	const glm::dvec3 forward= glm::normalize(glm::dvec3(0.0, 0.0, 0.0) - position);
	const glm::dvec3 right= glm::normalize(glm::cross(forward, glm::dvec3(0.0, 0.0, 1.0)));
	const glm::dvec3 down= glm::cross(forward, right);
	glm::dmat4 markerFromCamera(1.0);
	markerFromCamera[0]= glm::dvec4(right, 0.0);
	markerFromCamera[1]= glm::dvec4(down, 0.0);
	markerFromCamera[2]= glm::dvec4(forward, 0.0);
	markerFromCamera[3]= glm::dvec4(position, 1.0);
	return markerFromCamera;
}

MikanMonoIntrinsics makeIntrinsics()
{
	MikanMonoIntrinsics intrinsics;
	intrinsics.pixel_width= 1280;
	intrinsics.pixel_height= 720;
	intrinsics.undistorted_camera_matrix.x0= kFx;
	intrinsics.undistorted_camera_matrix.y1= kFy;
	intrinsics.undistorted_camera_matrix.z0= kCx;
	intrinsics.undistorted_camera_matrix.z1= kCy;
	return intrinsics;
}

// The right hand palm down on the desk beside the marker, fingers toward the
// camera, walked a little per frame so the medians see more than one frame.
// Palm +Z (out of the palmar surface) points into the desk.
glm::mat4 flatPalmForSample(int sampleIndex, float tiltRadians)
{
	const float t= 0.2f * (float)sampleIndex;
	const glm::quat flat= glm::angleAxis(glm::pi<float>(), glm::vec3(1.f, 0.f, 0.f));
	const glm::quat tilt= glm::angleAxis(tiltRadians, glm::vec3(1.f, 0.f, 0.f));
	const glm::quat yaw= glm::angleAxis(0.15f * sinf(t), glm::vec3(0.f, 0.f, 1.f));
	glm::mat4 transform= glm::mat4_cast(yaw * tilt * flat);
	transform[3]= glm::vec4(0.1f + 0.02f * sinf(t), -0.15f + 0.02f * cosf(t),
							DeskHandCalibrator::k_jointHeightMeters, 1.f);
	return transform;
}

// Projects a world hand into the camera's pixels and fills the per-camera
// frame the way the vision thread publishes it: 2D landmarks, the landmark
// model's own articulated points, and the camera's lift of them
struct SyntheticDesk
{
	glm::dmat4 markerFromCamera= makeMarkerFromCamera();
	glm::dmat4 cameraFromMarker= glm::inverse(markerFromCamera);
	MikanMonoIntrinsics intrinsics= makeIntrinsics();
	LandmarkTo3D lift;

	SyntheticDesk() { lift.configure(intrinsics, 0.085); }

	CameraFrameResult makeFrame(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& worldPoints,
								const std::array<glm::vec3, HAND_LANDMARK_COUNT>& modelPoints)
	{
		CameraFrameResult camera;
		camera.valid= true;
		camera.hasExtrinsics= true;
		camera.markerFromCamera= markerFromCamera;
		camera.hasIntrinsics= true;
		camera.fx= kFx;
		camera.fy= kFy;
		camera.cx= kCx;
		camera.cy= kCy;

		TrackedHand& hand= camera.result.hands[(int)eHandSide::Right];
		hand.tracked= true;
		hand.side= eHandSide::Right;
		hand.presence= 0.95f;
		hand.rightProb= 0.95f;
		for (int landmark= 0; landmark < HAND_LANDMARK_COUNT; ++landmark)
		{
			const glm::dvec4 cameraPoint= cameraFromMarker * glm::dvec4(glm::dvec3(worldPoints[landmark]), 1.0);
			hand.imagePoints[landmark]= glm::vec3((float)(kFx * cameraPoint.x / cameraPoint.z + kCx),
												  (float)(kFy * cameraPoint.y / cameraPoint.z + kCy), 0.f);
			hand.modelPoints[landmark]= modelPoints[landmark];
		}
		camera.result.timestampMs= 1000.0;
		lift.process(camera.result);
		return camera;
	}
};

float maxPhalanxError(const HandSkeleton& recovered, const HandSkeleton& truth)
{
	float worst= 0.f;
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
		for (int phalanx= 0; phalanx < 3; ++phalanx)
			worst= std::max(worst, fabsf(recovered.phalanxLengths[finger][phalanx] -
										 truth.phalanxLengths[finger][phalanx]));
	return worst;
}
} // namespace

static int runDeskHandTest(const TestArgs& args)
{
	int result= 0;
	const HandSkeleton truth= makeTruthSkeleton();
	const std::array<FingerAngles, FINGER_COUNT> flatAngles{};
	constexpr int kSampleCount= 60;
	const int rightIndex= (int)eHandSide::Right;

	// The landmark model's articulated hand in its own frame: the truth hand
	// about its wrist, which carries exactly the truth's articulation
	auto modelPointsFor= [&](const std::array<FingerAngles, FINGER_COUNT>& angles) {
		std::array<glm::vec3, HAND_LANDMARK_COUNT> points= buildLandmarks(glm::mat4(1.f), truth, angles);
		const glm::vec3 wrist= points[(int)eHandLandmark::WRIST];
		for (glm::vec3& point : points)
			point-= wrist;
		return points;
	};

	// -- (a) A flat hand measures its bones back, and the lift scale reads 1
	// when the model's articulation matches the pixels -----
	{
		SyntheticDesk desk;
		DeskHandCalibrator calibrator;
		int accepted= 0;
		float worstTilt= 0.f;
		for (int sample= 0; sample < kSampleCount; ++sample)
		{
			const CameraFrameResult camera= desk.makeFrame(
				buildLandmarks(flatPalmForSample(sample, 0.f), truth, flatAngles), modelPointsFor(flatAngles));
			DeskHandCalibrator::SampleStatus status;
			calibrator.addSample(camera, status);
			accepted+= status.bAccepted[rightIndex] ? 1 : 0;
			worstTilt= std::max(worstTilt, status.tiltDegrees[rightIndex]);
		}

		DeskHandCalibrator::Result measured;
		calibrator.finish(desk.intrinsics, measured);
		const float boneError= measured.bCaptured[rightIndex]
			? maxPhalanxError(measured.skeleton[rightIndex], truth) : 1e9f;
		MIKAN_LOG_INFO("test-deskhand")
			<< "(a) flat hand: " << accepted << "/" << kSampleCount << " accepted, worst tilt " << worstTilt
			<< " deg, max phalanx error " << boneError * 1000.f << " mm, lift scale "
			<< measured.liftDepthScale[rightIndex] << " (spread " << measured.liftDepthScaleSpread[rightIndex]
			<< ", " << measured.liftSampleCount[rightIndex] << " samples)";
		if (accepted < kSampleCount || !measured.bCaptured[rightIndex] || boneError > 0.0005f)
		{
			MIKAN_LOG_ERROR("test-deskhand") << "FAILED: (a) the plane lift must measure the flat hand's bones";
			result= 1;
		}
		if (!measured.bLiftScaleMeasured[rightIndex] || fabsf(measured.liftDepthScale[rightIndex] - 1.f) > 0.01f)
		{
			MIKAN_LOG_ERROR("test-deskhand") << "FAILED: (a) a consistent model must measure a lift scale of 1";
			result= 1;
		}
		if (measured.bCaptured[(int)eHandSide::Left])
		{
			MIKAN_LOG_ERROR("test-deskhand") << "FAILED: (a) the untracked left hand must not be captured";
			result= 1;
		}
	}

	// -- (b) A hand tilted off the desk is refused by the flatness gate -----
	{
		SyntheticDesk desk;
		DeskHandCalibrator calibrator;
		int accepted= 0;
		float minTilt= 1e9f;
		for (int sample= 0; sample < kSampleCount; ++sample)
		{
			const CameraFrameResult camera= desk.makeFrame(
				buildLandmarks(flatPalmForSample(sample, glm::radians(40.f)), truth, flatAngles),
				modelPointsFor(flatAngles));
			DeskHandCalibrator::SampleStatus status;
			calibrator.addSample(camera, status);
			accepted+= status.bAccepted[rightIndex] ? 1 : 0;
			if (status.bTracked[rightIndex])
				minTilt= std::min(minTilt, status.tiltDegrees[rightIndex]);
		}
		MIKAN_LOG_INFO("test-deskhand") << "(b) tilted 40 deg: " << accepted << " accepted, smallest tilt read "
										<< minTilt << " deg";
		if (accepted != 0 || calibrator.getSampleCount(eHandSide::Right) != 0)
		{
			MIKAN_LOG_ERROR("test-deskhand") << "FAILED: (b) a tilted hand must not be sampled";
			result= 1;
		}
	}

	// -- (c) The model's articulation disagreeing with the pixels is what the
	// lift scale measures: curled model fingers under flat pixels -----
	{
		SyntheticDesk desk;
		DeskHandCalibrator calibrator;
		std::array<FingerAngles, FINGER_COUNT> curledAngles{};
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			curledAngles[finger].proximal= 0.5f;
			curledAngles[finger].intermediate= 0.4f;
		}
		const std::array<glm::vec3, HAND_LANDMARK_COUNT> curledModel= modelPointsFor(curledAngles);
		int accepted= 0;
		for (int sample= 0; sample < kSampleCount; ++sample)
		{
			// The frame's own lift and pose come from the consistent model,
			// so the flatness gates pass; only the stored model points carry
			// the curl, which is what the re-lift in finish() reads
			CameraFrameResult camera= desk.makeFrame(
				buildLandmarks(flatPalmForSample(sample, 0.f), truth, flatAngles), modelPointsFor(flatAngles));
			camera.result.hands[rightIndex].modelPoints= curledModel;
			DeskHandCalibrator::SampleStatus status;
			calibrator.addSample(camera, status);
			accepted+= status.bAccepted[rightIndex] ? 1 : 0;
		}

		DeskHandCalibrator::Result measured;
		calibrator.finish(desk.intrinsics, measured);
		MIKAN_LOG_INFO("test-deskhand")
			<< "(c) curled model under flat pixels: " << accepted << " accepted, lift scale "
			<< measured.liftDepthScale[rightIndex]
			<< " (spread " << measured.liftDepthScaleSpread[rightIndex] << ", "
			<< measured.liftSampleCount[rightIndex] << " samples), bones captured "
			<< measured.bCaptured[rightIndex];
		if (!measured.bLiftScaleMeasured[rightIndex] || fabsf(measured.liftDepthScale[rightIndex] - 1.f) < 0.01f ||
			measured.liftDepthScaleSpread[rightIndex] > 0.05f)
		{
			MIKAN_LOG_ERROR("test-deskhand")
				<< "FAILED: (c) a biased model must measure a lift scale away from 1 with a small spread";
			result= 1;
		}
	}

	if (result == 0)
		MIKAN_LOG_INFO("test-deskhand") << "All desk hand calibration checks passed";
	return result;
}

MIKAN_REGISTER_TEST("--test-deskhand", "Desk-plane hand calibration: bones from one camera, flatness gates, lift depth scale",
					eTestCategory::SelfTest, runDeskHandTest);
