#pragma once

#include "OpenCVFwd.h"
#include "CameraMath.h"
#include "MikanVideoSourceTypes.h"

#include <memory>
#include "opencv2/core/types.hpp"

#include "glm/ext/vector_float3.hpp"

// Calibration pattern enums, inlined from MikanXR's ProjectConfigConstants.h
enum class eCalibrationPatternType : int
{
	INVALID= -1,

	mode_chessboard,
	mode_charuco,
	mode_aruco,

	COUNT
};

enum class eCharucoDictionaryType : int
{
	INVALID= -1,

	DICT_4X4,
	DICT_5X5,
	DICT_6X6,
	DICT_7X7,

	COUNT
};

// What a lens calibration pattern is, enough to build its finder
struct CalibrationPatternDesc
{
	eCalibrationPatternType type= eCalibrationPatternType::mode_charuco;
	// Squares across and down, so the inner corner grid is one less each way
	int cols= 11;
	int rows= 8;
	float squareLengthMM= 16.f;
	// Charuco only
	float markerLengthMM= 12.f;
	eCharucoDictionaryType dictionary= eCharucoDictionaryType::DICT_6X6;
};

class CalibrationPatternFinder;
typedef std::shared_ptr<CalibrationPatternFinder> CalibrationPatternFinderPtr;

// Helper use to implement OpenCV camera lens intrinsic/distortion calibration method.
// See https://docs.opencv.org/3.3.0/dc/dbb/tutorial_py_calibration.html for details.
class CalibrationPatternFinder
{
public:
	CalibrationPatternFinder(int frameWidth, int frameHeight);
	virtual ~CalibrationPatternFinder();

	// The caller owns the grayscale frame buffer and updates the pointer each frame
	inline void setGrayscaleFrame(const cv::Mat* frame) { m_grayscaleFrame= frame; }
	inline const cv::Mat* getGrayscaleVideoFrameInput() const { return m_grayscaleFrame; }

	// Mono camera intrinsics used by estimateNewCalibrationPatternPose
	void setCameraIntrinsics(const struct MikanMonoIntrinsics& intrinsics);
	inline bool hasCameraIntrinsics() const { return m_hasCameraIntrinsics; }

	virtual eCalibrationPatternType getCalibrationPatternType() const= 0;
	virtual bool findNewCalibrationPattern(const float minSeperationDist= 0.f)= 0;
	virtual bool estimateNewCalibrationPatternPose(glm::dmat4& outCameraToPatternXform);

	// Side-effect-free read of the currently detected pattern (for UI overlays).
	// Does NOT commit the points as the last-valid capture, so it never
	// interferes with the min-separation check in findNewCalibrationPattern.
	virtual bool getCurrentCalibrationPattern(t_opencv_point2d_list& outImagePoints,
											  cv::Point2f outBoundingQuad[4]) const= 0;

	// Fetches the current pattern AND commits it as the last-valid capture:
	// the pattern must then move at least minSeperationDist away before
	// findNewCalibrationPattern reports valid points again. Only call this
	// when actually capturing a calibration sample.
	virtual bool fetchLastFoundCalibrationPattern(t_opencv_point2d_list& outImagePoints,
												  t_opencv_pointID_list& outImagePointIDs,
												  cv::Point2f outBoundingQuad[4])= 0;

	// Average pixel distance between two detections of the whole pattern,
	// corner for corner. How far the pattern moved between them.
	virtual float computeMeanPointDistance(const t_opencv_point2d_list& pointsA,
										   const t_opencv_point2d_list& pointsB) const;

	bool areCurrentImagePointsValid() const;
	inline float getFrameWidth() const { return m_frameWidth; }
	inline float getFrameHeight() const { return m_frameHeight; }
	inline void getOpenCVLensCalibrationGeometry(OpenCVCalibrationGeometry* outGeometry) const
	{
		*outGeometry= m_opencvLensCalibrationGeometry;
	};
	inline void getOpenCVSolvePnPGeometry(OpenCVCalibrationGeometry* outGeometry) const
	{
		*outGeometry= m_opencvSolvePnPGeometry;
	};
	inline void getOpenGLSolvePnPGeometry(OpenGLCalibrationGeometry* outGeometry) const
	{
		*outGeometry= m_openglSolvePnPGeometry;
	};

	// The finder for a lens calibration pattern (chessboard or charuco), owned
	// by the caller. Throws cv::Exception on board geometry OpenCV rejects.
	static CalibrationPatternFinder* createLensPatternFinder(const CalibrationPatternDesc& pattern, int frameWidth,
															 int frameHeight);

	static ArucoDictionaryPtr getArucoDictionary(eCharucoDictionaryType dictionaryType);

protected:
	// Video buffer state (owned by the caller, updated per frame)
	const cv::Mat* m_grayscaleFrame;

	float m_frameWidth;
	float m_frameHeight;

	// Camera intrinsics used for pattern pose estimation
	MikanMonoIntrinsics m_cameraIntrinsics;
	bool m_hasCameraIntrinsics;

	// Internal Calibration State
	OpenCVCalibrationGeometry m_opencvLensCalibrationGeometry;
	OpenCVCalibrationGeometry m_opencvSolvePnPGeometry;
	OpenGLCalibrationGeometry m_openglSolvePnPGeometry;
	t_opencv_point2d_list m_lastValidQuad;
	t_opencv_point2d_list m_lastValidImagePoints;
	t_opencv_point2d_list m_currentImagePoints;
};
