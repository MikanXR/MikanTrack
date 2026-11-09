#pragma once

#include <array>
#include <vector>

#include "HandBoneCalibrator.h"
#include "HandFusion.h" // CameraFrameResult
#include "MikanVideoSourceTypes.h"

// Measures the user's hand geometry from ONE camera, with the desk standing
// in for the second view.
//
// Bone calibration needs landmarks measured in meters, and a single camera
// only has rays. A hand laid flat on the desk beside the origin marker puts
// every landmark on a known plane: the marker's plane, raised by the height
// of a flat hand's joints above the table. Each landmark's ray meets that
// plane at a metric point, and those points feed the same HandBoneCalibrator
// the stereo path uses. The thumb rests on its side and sits a little higher
// than the plane assumes, which the medians tolerate.
//
// The same frames measure the camera's lift depth scale for free. Once the
// skeleton is known, the monocular PnP lift of each sampled frame is re-run
// with that skeleton as its object model, exactly as live tracking will run
// it, and the plane's wrist distance over the lift's wrist distance is the
// factor the lift is short or long by in this view. The median over the
// window is what CameraProfile::liftDepthScale stores.
//
// A frame is accepted only when the hand reads flat: the camera's own lift
// says the palm lies within k_maxTiltDegrees of the desk and the fingers are
// within k_maxCurlDegrees of straight. A tilted or curled hand lifted onto
// the plane measures foreshortened bones, consistently, so the medians would
// not catch it.
//
// THREAD AFFINITY: none of its own. The wizard owns it on the main thread
// and feeds it the per-camera preview results.
class DeskHandCalibrator
{
public:
	struct SampleStatus
	{
		bool bTracked[2]= {false, false};
		bool bFlat[2]= {false, false};
		bool bAccepted[2]= {false, false};
		float tiltDegrees[2]= {0.f, 0.f};
		float curlDegrees[2]= {0.f, 0.f};
	};

	struct Result
	{
		bool bCaptured[2]= {false, false};
		std::array<HandSkeleton, 2> skeleton{};
		std::array<HandBoneCalibrator::Quality, 2> quality{};
		// The camera's lift depth scale per side, 1 when not measured
		bool bLiftScaleMeasured[2]= {false, false};
		float liftDepthScale[2]= {1.f, 1.f};
		// Median absolute deviation of the per-frame ratio over its median
		float liftDepthScaleSpread[2]= {0.f, 0.f};
		int liftSampleCount[2]= {0, 0};
	};

	// Height of a flat hand's joint centers above the table, meters
	static constexpr float k_jointHeightMeters= 0.012f;
	// The palm normal within this of the desk normal counts as flat
	static constexpr float k_maxTiltDegrees= 25.f;
	// The largest proximal bend of the four fingers that still counts as
	// straight
	static constexpr float k_maxCurlDegrees= 25.f;

	void reset();

	// The flatness read of each hand in one camera frame, without sampling:
	// what the wizard shows while the user lays the hands down
	static void assess(const CameraFrameResult& camera, SampleStatus& outStatus);

	// One camera frame: every tracked hand that reads flat is lifted onto the
	// desk plane and sampled. The camera needs intrinsics and extrinsics.
	void addSample(const CameraFrameResult& camera, SampleStatus& outStatus);

	int getSampleCount(eHandSide side) const { return m_bones.getSampleCount(side); }

	// Medians over the window, then the lift re-run with the measured
	// skeleton for the depth scale. intrinsics are the sampled camera's.
	void finish(const MikanMonoIntrinsics& intrinsics, Result& outResult) const;

private:
	struct SideFrames
	{
		// The accepted frames' 2D hands, kept for the lift re-run
		std::vector<TrackedHand> hands;
		// The plane-lifted wrist's distance from the camera per frame
		std::vector<float> planeWristDistance;
	};

	HandBoneCalibrator m_bones;
	std::array<SideFrames, 2> m_sides;
};
