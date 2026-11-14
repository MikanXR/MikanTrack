#include "CalibrationPatternFinder.h"
#include "Logger.h"
#include "MathUtility.h"
#include "MonoLensDistortionCalibrator.h"
#include "MathTypeConversion.h"

#include <algorithm>
#include <atomic>
#include <deque>
#include <thread>

#include "opencv2/opencv.hpp"

struct MonoLensDistortionCalibrationState
{
	enum AsyncComputeStatus
	{
		NotStarted,
		Running,
		Succeded,
		Failed
	};

	// One whole-pattern detection and the capture time of its video frame
	struct DetectionSample
	{
		double timeSeconds;
		t_opencv_point2d_list points;
	};

	// Static Input
	OpenCVCalibrationGeometry calibrationGeometry;
	MikanMonoIntrinsics originalCameraIntrinsics;
	int frameWidth, frameHeight;
	int desiredPatternCount;

	// Capture State
	int capturedPatternCount;
	int tiltedPatternCount;
	bool bWantsTiltedSample;
	t_opencv_point2d_list quadList;
	std::vector<t_opencv_point2d_list> imagePointsList;
	std::vector<t_opencv_pointID_list> imagePointIDList;

	// Steadiness state
	int64_t lastFrameSequence;
	std::deque<DetectionSample> recentDetections;
	float steadyFraction;

	// Async Calibration Compute
	std::thread* asyncComputeTask;
	std::atomic_int asyncComputeTaskStatus;

	// Result
	MikanMonoIntrinsics outputCameraIntrinsics;
	double reprojectionError;

	void init(CalibrationPatternFinder* patternFinder, int width, int height, int patternCount)
	{
		frameWidth= width;
		frameHeight= height;
		desiredPatternCount= patternCount;

		createDefautMonoIntrinsics(frameWidth, frameHeight, originalCameraIntrinsics);

		patternFinder->getOpenCVLensCalibrationGeometry(&calibrationGeometry);

		resetCalibration();
	}

	void resetSteadiness()
	{
		recentDetections.clear();
		steadyFraction= 0.f;
	}

	void resetCalibration()
	{
		// Reset the capture state
		capturedPatternCount= 0;
		tiltedPatternCount= 0;
		bWantsTiltedSample= false;
		quadList.clear();
		imagePointsList.clear();
		imagePointIDList.clear();

		// Reset the steadiness state
		lastFrameSequence= -1;
		resetSteadiness();

		// Reset the async task state
		asyncComputeTask= nullptr;
		asyncComputeTaskStatus= NotStarted;

		// Reset the output
		outputCameraIntrinsics= originalCameraIntrinsics;
		reprojectionError= 0.0;
	}
};

//-- MonoDistortionCalibrator ----
MonoLensDistortionCalibrator::MonoLensDistortionCalibrator(int frameWidth, int frameHeight,
														   const CalibrationPatternDesc& pattern,
														   int desiredSampleCount)
	: m_calibrationState(new MonoLensDistortionCalibrationState)
{
	m_patternFinder= CalibrationPatternFinder::createLensPatternFinder(pattern, frameWidth, frameHeight);

	this->frameWidth= (float)frameWidth;
	this->frameHeight= (float)frameHeight;
	m_cornerCols= pattern.cols - 1;
	m_cornerRows= pattern.rows - 1;

	m_calibrationState->init(m_patternFinder, frameWidth, frameHeight, desiredSampleCount);
}

MonoLensDistortionCalibrator::~MonoLensDistortionCalibrator()
{
	if (m_calibrationState->asyncComputeTask != nullptr)
	{
		m_calibrationState->asyncComputeTask->join();
		delete m_calibrationState->asyncComputeTask;
	}

	delete m_patternFinder;
	delete m_calibrationState;
}

void MonoLensDistortionCalibrator::update(int64_t frameSequence, double frameTimeSeconds,
										  const cv::Mat* grayscaleFrame, const float minSeperationDist)
{
	MonoLensDistortionCalibrationState* state= m_calibrationState;

	// Feed the latest grayscale frame to the pattern finder
	m_patternFinder->setGrayscaleFrame(grayscaleFrame);

	if (hasSampledAllCalibrationPatterns())
		return;

	// The same video frame again says nothing new about how still the pattern is
	if (frameSequence == state->lastFrameSequence)
		return;
	state->lastFrameSequence= frameSequence;

	// Update the pattern capture state
	findNewCalibrationPattern(minSeperationDist);

	t_opencv_point2d_list imagePoints;
	cv::Point2f unusedQuad[4];
	if (!m_patternFinder->getCurrentCalibrationPattern(imagePoints, unusedQuad))
	{
		state->resetSteadiness();
		return;
	}

	// A frame time that runs backwards is a restarted stream, and one that
	// jumps ahead leaves nothing recent enough to compare against
	if (!state->recentDetections.empty())
	{
		const double sinceLastDetection= frameTimeSeconds - state->recentDetections.back().timeSeconds;
		if (sinceLastDetection < 0.0 || sinceLastDetection > (double)k_steadyMaxFrameGapSeconds)
			state->resetSteadiness();
	}

	// Compare against the newest earlier detection that is at least a baseline old
	state->steadyFraction= 0.f;
	for (auto it= state->recentDetections.rbegin(); it != state->recentDetections.rend(); ++it)
	{
		if (frameTimeSeconds - it->timeSeconds >= (double)k_steadyBaselineSeconds)
		{
			const float movement= m_patternFinder->computeMeanPointDistance(imagePoints, it->points);
			state->steadyFraction= movement > k_steadyMaxMovement ? k_steadyMaxMovement / movement : 1.f;
			break;
		}
	}

	state->recentDetections.push_back({frameTimeSeconds, imagePoints});
	// Only one detection older than the baseline is ever read
	while (state->recentDetections.size() > 1
		   && frameTimeSeconds - state->recentDetections[1].timeSeconds >= (double)k_steadyBaselineSeconds)
	{
		state->recentDetections.pop_front();
	}

	if (state->steadyFraction < 1.f)
		return;

	// Tilt quota: focal length is only observable from perspective, so
	// once the flat-board quota is full, only tilted boards are
	// accepted. A rejected flat board does NOT touch the finder's
	// last-valid points, so tilting the board in place re-arms the
	// hold cycle and captures as soon as it is still again.
	const bool bTilted= computeCurrentPatternKeystone() >= k_tiltKeystoneThreshold;
	const int flatQuota= int_max(state->desiredPatternCount - k_minTiltedSampleCount, 0);
	const int flatCount= state->capturedPatternCount - state->tiltedPatternCount;

	if (!bTilted && flatCount >= flatQuota)
	{
		state->bWantsTiltedSample= true;
	}
	else if (captureLastFoundCalibrationPattern())
	{
		if (bTilted)
			++state->tiltedPatternCount;
		state->bWantsTiltedSample= false;
	}

	// The comparison MUST start over on every attempt, captured or not: a
	// fast board move can produce a valid detection on the very next frame
	// after a capture, and that frame has to be judged against detections of
	// its own pose, not the captured one.
	state->resetSteadiness();
}

float MonoLensDistortionCalibrator::computeCurrentPatternKeystone() const
{
	t_opencv_point2d_list imagePoints;
	cv::Point2f unusedQuad[4];
	if (!m_patternFinder->getCurrentCalibrationPattern(imagePoints, unusedQuad))
		return 1.f;

	// Full-board detection is enforced by the finder, so the corner list is
	// the complete grid in row-major order
	const int cornerCount= m_cornerCols * m_cornerRows;
	if ((int)imagePoints.size() != cornerCount || m_cornerCols < 2 || m_cornerRows < 2)
		return 1.f;

	const cv::Point2f& topLeft= imagePoints[0];
	const cv::Point2f& topRight= imagePoints[m_cornerCols - 1];
	const cv::Point2f& bottomLeft= imagePoints[cornerCount - m_cornerCols];
	const cv::Point2f& bottomRight= imagePoints[cornerCount - 1];

	// Keystone = perspective foreshortening: the ratio of opposite outer edge
	// lengths. Fronto-parallel ~1 in both axes; tilt shows up in one of them.
	const float top= (float)cv::norm(topRight - topLeft);
	const float bottom= (float)cv::norm(bottomRight - bottomLeft);
	const float left= (float)cv::norm(bottomLeft - topLeft);
	const float right= (float)cv::norm(bottomRight - topRight);
	if (top < 1.f || bottom < 1.f || left < 1.f || right < 1.f)
		return 1.f;

	const float verticalKeystone= top > bottom ? top / bottom : bottom / top;
	const float horizontalKeystone= left > right ? left / right : right / left;
	return fmaxf(verticalKeystone, horizontalKeystone);
}

bool MonoLensDistortionCalibrator::wantsTiltedSample() const
{
	return m_calibrationState->bWantsTiltedSample && !hasSampledAllCalibrationPatterns();
}

int MonoLensDistortionCalibrator::getCapturedSampleCount() const { return m_calibrationState->capturedPatternCount; }

int MonoLensDistortionCalibrator::getTiltedSampleCount() const
{
	return m_calibrationState->tiltedPatternCount;
}

void MonoLensDistortionCalibrator::findNewCalibrationPattern(const float minSeperationDist)
{
	m_patternFinder->findNewCalibrationPattern(minSeperationDist);
}

bool MonoLensDistortionCalibrator::captureLastFoundCalibrationPattern()
{
	bool bCaptured= false;
	t_opencv_point2d_list imagePoints;
	t_opencv_pointID_list imagePointIDs;
	cv::Point2f boundingQuad[4];
	if (m_patternFinder->fetchLastFoundCalibrationPattern(imagePoints, imagePointIDs, boundingQuad))
	{
		m_calibrationState->imagePointsList.push_back(imagePoints);
		m_calibrationState->imagePointIDList.push_back(imagePointIDs);

		for (int i= 0; i < 4; ++i)
		{
			m_calibrationState->quadList.push_back(boundingQuad[i]);
		}

		++m_calibrationState->capturedPatternCount;
		bCaptured= true;
	}

	return bCaptured;
}

bool MonoLensDistortionCalibrator::hasSampledAllCalibrationPatterns() const
{
	return m_calibrationState->capturedPatternCount >= m_calibrationState->desiredPatternCount;
}

bool MonoLensDistortionCalibrator::areCurrentImagePointsValid() const
{
	return m_patternFinder->areCurrentImagePointsValid();
}

float MonoLensDistortionCalibrator::getStabilityFraction() const { return m_calibrationState->steadyFraction; }

float MonoLensDistortionCalibrator::computeCalibrationProgress() const
{
	const float samplePercentage=
		(float)m_calibrationState->capturedPatternCount / (float)m_calibrationState->desiredPatternCount;

	return samplePercentage;
}

void MonoLensDistortionCalibrator::resetCalibrationState() { m_calibrationState->resetCalibration(); }

void MonoLensDistortionCalibrator::computeCameraCalibration()
{
	assert(m_calibrationState->asyncComputeTask == nullptr);

	MonoLensDistortionCalibrationState* calibrationState= m_calibrationState;

	// Spin up a worker thread to compute the camera calibration.
	// The result of this is polled by getCameraCalibration().
	calibrationState->asyncComputeTaskStatus= MonoLensDistortionCalibrationState::Running;
	calibrationState->asyncComputeTask= new std::thread(
		[calibrationState]
		{
			bool bSuccess= computeMonoLensCameraCalibration(
				calibrationState->frameWidth, calibrationState->frameHeight, calibrationState->calibrationGeometry,
				calibrationState->imagePointsList, calibrationState->imagePointIDList,
				calibrationState->outputCameraIntrinsics, calibrationState->reprojectionError);

			// Signal the main thread that the task is complete
			calibrationState->asyncComputeTaskStatus=
				bSuccess ? MonoLensDistortionCalibrationState::Succeded : MonoLensDistortionCalibrationState::Failed;
		});
}

bool MonoLensDistortionCalibrator::getIsCameraCalibrationComplete() const
{
	auto status= m_calibrationState->asyncComputeTaskStatus.load();

	return status == MonoLensDistortionCalibrationState::Succeded
		   || status == MonoLensDistortionCalibrationState::Failed;
}

int MonoLensDistortionCalibrator::getDesiredPatternCount() const { return m_calibrationState->desiredPatternCount; }

bool MonoLensDistortionCalibrator::getCameraCalibration(MikanMonoIntrinsics* out_mono_intrinsics)
{
	bool bFetchSuccess= false;

	if (m_calibrationState->asyncComputeTaskStatus.load() == MonoLensDistortionCalibrationState::Succeded)
	{
		if (m_calibrationState->asyncComputeTask != nullptr)
		{
			m_calibrationState->asyncComputeTask->join();

			delete m_calibrationState->asyncComputeTask;
			m_calibrationState->asyncComputeTask= nullptr;
		}

		*out_mono_intrinsics= m_calibrationState->outputCameraIntrinsics;
		bFetchSuccess= true;
	}

	return bFetchSuccess;
}

float MonoLensDistortionCalibrator::getReprojectionError() const { return m_calibrationState->reprojectionError; }
