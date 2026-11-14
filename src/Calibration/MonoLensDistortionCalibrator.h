#pragma once

#include "CalibrationPatternFinder.h"
#include "MikanVideoSourceTypes.h"

#include <cstdint>
#include <memory>

// Helper use to implement OpenCV camera lens intrinsic/distortion calibration method.
// See https://docs.opencv.org/3.3.0/dc/dbb/tutorial_py_calibration.html for details.
class MonoLensDistortionCalibrator
{
public:
	// A sample is captured from a pattern that is being held still, since a
	// moving one is blurred and its corners land off. Still means the corners
	// moved no more than k_steadyMaxMovement on average against the detection
	// k_steadyBaselineSeconds earlier. A hand-held pattern is only that still
	// for a frame or two at a time, so one such comparison is the whole test.
	// The baseline sits just under three frames at 30 fps and one frame at
	// 10 fps, the rate a webcam drops to in low light.
	static constexpr float k_steadyBaselineSeconds= 0.09f;
	static constexpr float k_steadyMaxMovement= 3.f; // pixels
	// Two detections further apart than this are not compared
	static constexpr float k_steadyMaxFrameGapSeconds= 0.5f;
	// Minimum average pixel distance the pattern corners must move from the last
	// captured sample to be considered a new board location
	static constexpr float k_defaultMinSeperationDist= 10.f; // pixels

	// Focal length is only observable from PERSPECTIVE (tilted boards). A
	// capture set of fronto-parallel boards fits distortion to sub-pixel
	// error while fx sits wherever the initializer left it (seen live: a
	// solved hfov of exactly 90 deg). At least this many samples must show
	// real keystone before the set is complete; flat captures stop being
	// accepted once their quota (desired - tilted) is full.
	static constexpr int k_minTiltedSampleCount= 4;
	// A board tilted ~20 deg at arm's length shows ~1.12; fronto-parallel ~1.0
	static constexpr float k_tiltKeystoneThreshold= 1.10f;

	MonoLensDistortionCalibrator(int frameWidth, int frameHeight, const CalibrationPatternDesc& pattern,
								 int desiredSampleCount);
	virtual ~MonoLensDistortionCalibrator();

	inline CalibrationPatternFinder* getPatternFinder() const { return m_patternFinder; }

	// Feed in the latest grayscale video frame: search it for the calibration
	// pattern, and auto-capture a sample once the pattern has been held still
	// in a new location. frameSequence identifies the video frame, so a caller
	// that runs faster than the camera can pass the same frame again and have
	// it ignored. frameTimeSeconds is that frame's capture time.
	void update(int64_t frameSequence, double frameTimeSeconds, const cv::Mat* grayscaleFrame,
				const float minSeperationDist= k_defaultMinSeperationDist);

	void findNewCalibrationPattern(const float minSeperationDist);
	bool captureLastFoundCalibrationPattern();

	bool hasSampledAllCalibrationPatterns() const;
	bool areCurrentImagePointsValid() const;
	// UI hint: the flat-board quota is full - only tilted boards are accepted now
	bool wantsTiltedSample() const;
	int getCapturedSampleCount() const;
	int getTiltedSampleCount() const;
	// 0..1 measure of how still the pattern is being held: the movement
	// allowance over the movement seen. Capture fires at 1.
	float getStabilityFraction() const;
	float computeCalibrationProgress() const;
	void resetCalibrationState();

	void computeCameraCalibration();
	bool getIsCameraCalibrationComplete() const;
	int getDesiredPatternCount() const;
	bool getCameraCalibration(MikanMonoIntrinsics* out_mono_intrinsics);
	float getReprojectionError() const;

protected:
	// Keystone factor (>= 1) of the current detection: max length ratio
	// between opposite outer edges of the board. Requires a FULL board
	// detection (corner index == grid position), which the finders enforce.
	float computeCurrentPatternKeystone() const;

	float frameWidth;
	float frameHeight;
	int m_cornerCols= 0;
	int m_cornerRows= 0;

	// Internal Calibration State
	struct MonoLensDistortionCalibrationState* m_calibrationState;

	// Calibration pattern being used
	CalibrationPatternFinder* m_patternFinder;
};
