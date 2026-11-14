#pragma once

#include "CalibrationPatternFinder.h"

// A plain chessboard, the lens calibration pattern for a phone screen: with
// no marker bits to decode it survives the bloom of an emissive display.
// The whole board must be in view, and its corners carry no identity, so a
// detection is the full inner corner grid in row-major order.
class CalibrationPatternFinder_Chessboard : public CalibrationPatternFinder
{
public:
	// The board a phone shows: squares across and down, in landscape
	static constexpr int k_phoneBoardCols= 10;
	static constexpr int k_phoneBoardRows= 5;

	CalibrationPatternFinder_Chessboard(int frameWidth, int frameHeight, int squareCols, int squareRows,
										float squareLengthMM);

	static CalibrationPatternDesc makePhoneBoardDesc();

	virtual eCalibrationPatternType getCalibrationPatternType() const override
	{
		return eCalibrationPatternType::mode_chessboard;
	}
	virtual bool findNewCalibrationPattern(const float minSeperationDist= 0.f) override;
	virtual bool getCurrentCalibrationPattern(t_opencv_point2d_list& outImagePoints,
											  cv::Point2f outBoundingQuad[4]) const override;
	// Re-detects the current frame with the slower sector based detector and
	// hands back its corners, the ones a sample is solved from. Fails, with
	// nothing committed, when that detector does not find the board.
	virtual bool fetchLastFoundCalibrationPattern(t_opencv_point2d_list& outImagePoints,
												  t_opencv_pointID_list& outImagePointIDs,
												  cv::Point2f outBoundingQuad[4]) override;
	// A board turned half a turn detects with its corners in reverse order,
	// so the distance is the smaller of the two pairings
	virtual float computeMeanPointDistance(const t_opencv_point2d_list& pointsA,
										   const t_opencv_point2d_list& pointsB) const override;

protected:
	int m_cornerCols;
	int m_cornerRows;
};
