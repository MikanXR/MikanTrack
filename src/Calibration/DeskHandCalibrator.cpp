#include "DeskHandCalibrator.h"

#include <algorithm>
#include <cmath>

#include "glm/geometric.hpp"
#include "glm/trigonometric.hpp"

#include "LandmarkTo3D.h"
#include "SpaceTransforms.h"

namespace
{
float medianOf(std::vector<float> values)
{
	if (values.empty())
		return 0.f;
	const size_t middle= values.size() / 2;
	std::nth_element(values.begin(), values.begin() + middle, values.end());
	return values[middle];
}

float medianAbsoluteDeviation(const std::vector<float>& values, float median)
{
	std::vector<float> deviations;
	deviations.reserve(values.size());
	for (float value : values)
		deviations.push_back(std::fabs(value - median));
	return medianOf(deviations);
}
} // namespace

void DeskHandCalibrator::reset()
{
	m_bones.reset();
	for (SideFrames& side : m_sides)
	{
		side.hands.clear();
		side.planeWristDistance.clear();
	}
}

void DeskHandCalibrator::assess(const CameraFrameResult& camera, SampleStatus& outStatus)
{
	outStatus= SampleStatus();
	if (!camera.valid || !camera.hasExtrinsics || !camera.hasIntrinsics)
		return;

	const glm::vec3 deskNormal(0.f, 0.f, 1.f);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const TrackedHand& hand= camera.result.hands[sideIndex];
		const HandPose& pose= camera.result.poses[sideIndex];
		if (!hand.tracked || !hand.hasCameraSpace)
			continue;
		outStatus.bTracked[sideIndex]= true;

		// Flatness from the camera's own lift: the palm triangle's normal
		// against the desk's, in world space. Chirality drops out of the
		// absolute value.
		auto liftWorld= [&](eHandLandmark landmark) {
			const glm::vec3& point= hand.cameraPoints[(int)landmark];
			return glm::vec3(camera.markerFromCamera * glm::dvec4(glm::dvec3(point), 1.0));
		};
		const glm::vec3 wrist= liftWorld(eHandLandmark::WRIST);
		const glm::vec3 palmNormal= glm::cross(liftWorld(eHandLandmark::INDEX_MCP) - wrist,
											   liftWorld(eHandLandmark::PINKY_MCP) - wrist);
		const float normalLength= glm::length(palmNormal);
		if (normalLength < 1e-9f)
			continue;
		const float cosTilt= std::fabs(glm::dot(palmNormal / normalLength, deskNormal));
		outStatus.tiltDegrees[sideIndex]= glm::degrees(acosf(std::clamp(cosTilt, 0.f, 1.f)));

		float curl= 0.f;
		for (int finger= (int)eFinger::Index; finger < FINGER_COUNT; ++finger)
			curl= std::max(curl, std::fabs(pose.fingers[finger].proximal));
		outStatus.curlDegrees[sideIndex]= glm::degrees(curl);

		outStatus.bFlat[sideIndex]= outStatus.tiltDegrees[sideIndex] <= k_maxTiltDegrees &&
									outStatus.curlDegrees[sideIndex] <= k_maxCurlDegrees;
	}
}

void DeskHandCalibrator::addSample(const CameraFrameResult& camera, SampleStatus& outStatus)
{
	assess(camera, outStatus);
	const glm::vec3 cameraPos= cameraPositionWorld(camera.markerFromCamera);

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const TrackedHand& hand= camera.result.hands[sideIndex];
		if (!outStatus.bFlat[sideIndex])
			continue;

		// Every landmark's ray onto the raised desk plane. A ray that does
		// not descend never reaches the desk, and the frame is dropped whole:
		// a partial hand would measure nothing useful.
		std::array<glm::vec3, HAND_LANDMARK_COUNT> planePoints;
		bool bAllOnPlane= true;
		for (int landmark= 0; landmark < HAND_LANDMARK_COUNT; ++landmark)
		{
			const glm::vec3 rayDir= pixelRayDirWorld(camera.markerFromCamera, camera.fx, camera.fy,
													 camera.cx, camera.cy,
													 glm::vec2(hand.imagePoints[landmark]));
			if (rayDir.z >= -1e-4f)
			{
				bAllOnPlane= false;
				break;
			}
			const float t= (k_jointHeightMeters - cameraPos.z) / rayDir.z;
			if (t <= 0.f)
			{
				bAllOnPlane= false;
				break;
			}
			planePoints[landmark]= cameraPos + rayDir * t;
		}
		if (!bAllOnPlane)
			continue;

		const int before= m_bones.getSampleCount((eHandSide)sideIndex);
		m_bones.addSample((eHandSide)sideIndex, planePoints);
		if (m_bones.getSampleCount((eHandSide)sideIndex) == before)
			continue; // degenerate palm frame, refused by the bone calibrator

		SideFrames& frames= m_sides[sideIndex];
		frames.hands.push_back(hand);
		frames.planeWristDistance.push_back(
			glm::length(planePoints[(int)eHandLandmark::WRIST] - cameraPos));
		outStatus.bAccepted[sideIndex]= true;
	}
}

void DeskHandCalibrator::finish(const MikanMonoIntrinsics& intrinsics, Result& outResult) const
{
	outResult= Result();
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const eHandSide side= (eHandSide)sideIndex;
		outResult.bCaptured[sideIndex]=
			m_bones.finish(side, outResult.skeleton[sideIndex], outResult.quality[sideIndex]);
		if (!outResult.bCaptured[sideIndex])
			continue;

		// The lift as live tracking will run it after this skeleton saves:
		// the measured skeleton as the PnP object model, reference bone from
		// the measurement. Frames replay in order so the warm start behaves
		// as it does live.
		const HandSkeleton& skeleton= outResult.skeleton[sideIndex];
		LandmarkTo3D lift;
		lift.configure(intrinsics, skeleton.baseInPalm[(int)eFinger::Middle].x * 2.0);
		lift.setCalibratedSkeleton(side, skeleton);

		const SideFrames& frames= m_sides[sideIndex];
		std::vector<float> ratios;
		ratios.reserve(frames.hands.size());
		for (size_t frameIndex= 0; frameIndex < frames.hands.size(); ++frameIndex)
		{
			TrackingFrameResult frame;
			frame.timestampMs= (double)frameIndex * (1000.0 / 30.0);
			TrackedHand& hand= frame.hands[sideIndex];
			hand= frames.hands[frameIndex];
			hand.hasCameraSpace= false;
			hand.hasWorldSpace= false;
			lift.process(frame);
			if (!hand.hasCameraSpace)
				continue;

			const float liftDistance= glm::length(hand.cameraPoints[(int)eHandLandmark::WRIST]);
			if (liftDistance > 1e-3f)
				ratios.push_back(frames.planeWristDistance[frameIndex] / liftDistance);
		}

		outResult.liftSampleCount[sideIndex]= (int)ratios.size();
		if ((int)ratios.size() < HandBoneCalibrator::k_minSamples)
			continue;

		const float median= medianOf(ratios);
		if (!(median > 1e-3f))
			continue;
		outResult.bLiftScaleMeasured[sideIndex]= true;
		outResult.liftDepthScale[sideIndex]= median;
		outResult.liftDepthScaleSpread[sideIndex]= medianAbsoluteDeviation(ratios, median) / median;
	}
}
