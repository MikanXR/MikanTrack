#pragma once

#include "MikanVideoSourceTypes.h"

#include <filesystem>
#include <string>
#include <vector>

// A lens found in the webcam table for a camera and frame size
struct WebcamTableMatch
{
	std::string cameraName;
	// Where the table's values came from
	std::string origin;
	MikanMonoIntrinsics intrinsics;
	// The values were scaled from a calibration at another frame size
	bool bScaled= false;
};

// Shipped lens calibrations of known webcams, keyed by USB vendor id, product
// id and frame size, so a listed camera needs no calibration step. A webcam
// model's lenses differ from unit to unit by a percent or two, so an entry
// stands in for a calibration without being one.
class WebcamIntrinsicsTable
{
public:
	// resources/calibration/webcam_table.json beside the executable
	static std::filesystem::path getDefaultPath();

	// False, with the table left empty, for a missing or malformed file
	bool load(const std::filesystem::path& path);
	bool loadFromText(const std::string& jsonText);
	size_t getCameraCount() const { return m_cameras.size(); }

	// The camera's calibration at the frame size. With none at that size, the
	// largest one of the same aspect ratio is scaled by the width ratio. A
	// size of another aspect ratio is no match, since a webcam often crops
	// its sensor for such a mode and the lens then reads differently.
	bool find(unsigned int vendorId, unsigned int productId, int frameWidth, int frameHeight,
			  WebcamTableMatch& outMatch) const;
	// The same from a device path, false for a path that names no USB ids
	bool findForDevicePath(const std::string& devicePath, int frameWidth, int frameHeight,
						   WebcamTableMatch& outMatch) const;

private:
	struct Calibration
	{
		int width= 0;
		int height= 0;
		double focalLengthX= 0.0;
		double focalLengthY= 0.0;
		double principalPointX= 0.0;
		double principalPointY= 0.0;
		MikanDistortionCoefficients distortion;
	};

	struct Camera
	{
		unsigned int vendorId= 0;
		unsigned int productId= 0;
		std::string name;
		std::string origin;
		std::vector<Calibration> calibrations;
	};

	std::vector<Camera> m_cameras;
};
