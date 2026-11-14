#include "CalibrationPatternFinder_Chessboard.h"
#include "MathTypeConversion.h"

#include <algorithm>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

CalibrationPatternFinder_Chessboard::CalibrationPatternFinder_Chessboard(int frameWidth, int frameHeight,
																		 int squareCols, int squareRows,
																		 float squareLengthMM)
	: CalibrationPatternFinder(frameWidth, frameHeight)
	, m_cornerCols(squareCols - 1)
	, m_cornerRows(squareRows - 1)
{
	if (m_cornerCols < 2 || m_cornerRows < 2 || squareLengthMM <= 0.f)
		CV_Error(cv::Error::StsBadArg, "A chessboard needs at least 3 by 3 squares of a positive size");

	m_opencvLensCalibrationGeometry.points.clear();
	m_opencvSolvePnPGeometry.points.clear();
	m_openglSolvePnPGeometry.points.clear();

	for (int row= 0; row < m_cornerRows; ++row)
	{
		for (int col= 0; col < m_cornerCols; ++col)
		{
			// Solve PnP points are on the XZ Plane
			cv::Point3f openCVSolvePnPPoint(float(col) * squareLengthMM, 0.f, -float(row) * squareLengthMM);
			// Lens calibration points are on the XY Plane
			cv::Point3f openCVLensCalibrationPoint(float(col) * squareLengthMM, float(row) * squareLengthMM, 0.f);

			// OpenCV -> OpenGL coordinate system transform
			// Rendering world units in meters, not mm
			glm::vec3 openGLPoint(openCVSolvePnPPoint.x * k_millimeters_to_meters,
								  -openCVSolvePnPPoint.y * k_millimeters_to_meters,
								  -openCVSolvePnPPoint.z * k_millimeters_to_meters);

			m_opencvLensCalibrationGeometry.points.push_back(openCVLensCalibrationPoint);
			m_opencvSolvePnPGeometry.points.push_back(openCVSolvePnPPoint);
			m_openglSolvePnPGeometry.points.push_back(openGLPoint);
		}
	}
}

CalibrationPatternDesc CalibrationPatternFinder_Chessboard::makePhoneBoardDesc()
{
	CalibrationPatternDesc pattern;
	pattern.type= eCalibrationPatternType::mode_chessboard;
	pattern.cols= k_phoneBoardCols;
	pattern.rows= k_phoneBoardRows;
	// The solve does not depend on the board's physical size, only its
	// proportions, and a phone screen's size is not known
	pattern.squareLengthMM= 10.f;
	return pattern;
}

bool CalibrationPatternFinder_Chessboard::findNewCalibrationPattern(const float minSeperationDist)
{
	m_currentImagePoints.clear();

	const cv::Mat* gsSourceBuffer= getGrayscaleVideoFrameInput();
	if (gsSourceBuffer == nullptr)
		return false;

	// The fast detector, run on every frame. Fast check gives up early on a
	// frame with no board in it.
	const cv::Size cornerGrid(m_cornerCols, m_cornerRows);
	t_opencv_point2d_list corners;
	const int flags= cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
	if (!cv::findChessboardCorners(*gsSourceBuffer, cornerGrid, corners, flags))
		return false;

	cv::cornerSubPix(*gsSourceBuffer, corners, cv::Size(5, 5), cv::Size(-1, -1),
					 cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));

	// It's a new location if the board moved far enough from the last capture
	if (!m_lastValidImagePoints.empty() && minSeperationDist > 0.f
		&& computeMeanPointDistance(corners, m_lastValidImagePoints) < minSeperationDist)
	{
		return false;
	}

	m_currentImagePoints= corners;
	return true;
}

bool CalibrationPatternFinder_Chessboard::getCurrentCalibrationPattern(t_opencv_point2d_list& outImagePoints,
																	   cv::Point2f outBoundingQuad[4]) const
{
	if (!areCurrentImagePointsValid())
		return false;

	const cv::RotatedRect boundingRect= cv::minAreaRect(m_currentImagePoints);
	boundingRect.points(outBoundingQuad);

	outImagePoints= m_currentImagePoints;
	return true;
}

bool CalibrationPatternFinder_Chessboard::fetchLastFoundCalibrationPattern(t_opencv_point2d_list& outImagePoints,
																		   t_opencv_pointID_list& outImagePointIDs,
																		   cv::Point2f outBoundingQuad[4])
{
	const cv::Mat* gsSourceBuffer= getGrayscaleVideoFrameInput();
	if (gsSourceBuffer == nullptr || !areCurrentImagePointsValid())
		return false;

	// The sector based detector places corners more accurately on a soft or
	// bloomed image, at several times the cost, so it runs once per sample
	t_opencv_point2d_list corners;
	if (!cv::findChessboardCornersSB(*gsSourceBuffer, cv::Size(m_cornerCols, m_cornerRows), corners,
									 cv::CALIB_CB_NORMALIZE_IMAGE))
	{
		return false;
	}

	const cv::RotatedRect boundingRect= cv::minAreaRect(corners);
	boundingRect.points(outBoundingQuad);

	outImagePoints= corners;
	outImagePointIDs.clear();
	for (int cornerIndex= 0; cornerIndex < (int)corners.size(); ++cornerIndex)
		outImagePointIDs.push_back(cornerIndex);

	// The min-separation check compares against the fast detector's corners,
	// the ones findNewCalibrationPattern sees on later frames
	m_lastValidImagePoints= m_currentImagePoints;

	return true;
}

float CalibrationPatternFinder_Chessboard::computeMeanPointDistance(const t_opencv_point2d_list& pointsA,
																	const t_opencv_point2d_list& pointsB) const
{
	const float forwardDistance= CalibrationPatternFinder::computeMeanPointDistance(pointsA, pointsB);
	if (pointsA.empty() || pointsA.size() != pointsB.size())
		return forwardDistance;

	const t_opencv_point2d_list reversedB(pointsB.rbegin(), pointsB.rend());
	return std::min(forwardDistance, CalibrationPatternFinder::computeMeanPointDistance(pointsA, reversedB));
}
