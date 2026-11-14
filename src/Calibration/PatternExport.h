#pragma once

#include "CalibrationPatternFinder.h"

#include <filesystem>
#include <string>

#include "opencv2/core/mat.hpp"

// Renders a printable ChArUco board image and writes it to a PNG file.
// Adapted from MikanXR's MarkerObjectSystem::printMarker (minus the libharu PDF output).
// pixelsPerMM controls the output resolution (e.g. 11.811f ~= 300 DPI).
bool generateCharucoBoardPng(const std::filesystem::path& pngPath, int charucoCols, int charucoRows,
							 float charucoSquareLengthMM, float charucoMarkerLengthMM,
							 eCharucoDictionaryType charucoDictionaryType, float pixelsPerMM);

// A plain chessboard, squares across and down, with a white margin half a
// square wide: the pattern a phone screen shows for lens calibration. The top
// left square is black.
cv::Mat renderChessboardImage(int squareCols, int squareRows, int squareSizePx);
bool generateChessboardPng(const std::filesystem::path& pngPath, int squareCols, int squareRows, int squareSizePx);

// The page that shows the phone chessboard, and that address as a QR code
// image: black on white, CV_8UC3, each module modulePx across, with a quiet
// margin. Empty when the text cannot be encoded.
extern const char* const k_phonePatternUrl;
cv::Mat renderQrCodeImage(const std::string& text, int modulePx);

// Writes a printable pattern to the standard calibration resource path and
// opens it in the OS's default image viewer, ready to print. Returns the
// written path, or an empty path on failure (logged).
//
// Wrapped rather than left to each caller because "wrote a file, logged a
// path" is not a usable prompt to go print something.
std::filesystem::path exportAndOpenCharucoBoard(int charucoCols, int charucoRows,
												float charucoSquareLengthMM, float charucoMarkerLengthMM,
												eCharucoDictionaryType charucoDictionaryType);
// The phone chessboard as an image file, for a phone with no connection to
// load the page: sent to the phone and shown full screen it is the same board
std::filesystem::path exportAndOpenPhoneChessboard();
std::filesystem::path exportAndOpenArucoMarker(int arucoId, eCharucoDictionaryType arucoDictionaryType,
											   float markerLengthMM);

// Renders a printable ArUco origin marker and writes it to a PNG file.
// Adapted from MikanXR's MarkerComponent::printMarker (minus the libharu PDF output).
//
// The sheet carries direction labels, because the marker defines the tracking
// world's axes and a marker laid down rotated produces a world frame rotated
// with it - tracking that is entirely self-consistent and points the wrong way.
//
// The detector recovers a marker frame whose +X is the marker pattern's own
// right and whose +Y is its own top (asserted by --test-extrinsics). The
// tracking world is X forward, Y left, Z up, so the pattern is printed rotated
// a quarter turn: that way the SHEET is laid down the way it reads, top edge
// away from the user, and the axes land where the convention says.
//
// markerLengthMM is only used for the printed reminder of what the square has
// to measure, since a page scaled by the print dialog silently rescales the
// whole tracking world.
bool generateArucoMarkerPng(const std::filesystem::path& pngPath, int arucoId,
							eCharucoDictionaryType arucoDictionaryType, int markerSizePx,
							float markerLengthMM);
