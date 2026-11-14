#include "WebcamIntrinsicsTable.h"

#include "CameraMath.h"
#include "Logger.h"
#include "PathUtils.h"
#include "UsbDeviceIds.h"

#include <fstream>
#include <sstream>

#include "nlohmann/json.hpp"

std::filesystem::path WebcamIntrinsicsTable::getDefaultPath()
{
	return PathUtils::getResourceDirectory() / "calibration" / "webcam_table.json";
}

bool WebcamIntrinsicsTable::load(const std::filesystem::path& path)
{
	m_cameras.clear();

	std::ifstream file(path);
	if (!file.is_open())
	{
		MIKAN_LOG_WARNING("WebcamIntrinsicsTable::load") << "No webcam table at " << path.string();
		return false;
	}

	std::stringstream text;
	text << file.rdbuf();
	return loadFromText(text.str());
}

bool WebcamIntrinsicsTable::loadFromText(const std::string& jsonText)
{
	m_cameras.clear();

	const nlohmann::json table= nlohmann::json::parse(jsonText, nullptr, false);
	if (!table.is_object() || !table.contains("cameras") || !table["cameras"].is_array())
	{
		MIKAN_LOG_ERROR("WebcamIntrinsicsTable::loadFromText") << "The webcam table is not valid JSON";
		return false;
	}

	try
	{
		for (const nlohmann::json& cameraJson : table["cameras"])
		{
			Camera camera;
			// The ids are written the way a device path spells them, in hex
			camera.vendorId= (unsigned int)std::stoul(cameraJson.at("vendorId").get<std::string>(), nullptr, 16);
			camera.productId= (unsigned int)std::stoul(cameraJson.at("productId").get<std::string>(), nullptr, 16);
			camera.name= cameraJson.value("name", "");
			camera.origin= cameraJson.value("origin", "");

			for (const nlohmann::json& calibrationJson : cameraJson.at("calibrations"))
			{
				Calibration calibration;
				calibration.width= calibrationJson.at("width").get<int>();
				calibration.height= calibrationJson.at("height").get<int>();
				calibration.focalLengthX= calibrationJson.at("focalLength").at(0).get<double>();
				calibration.focalLengthY= calibrationJson.at("focalLength").at(1).get<double>();
				calibration.principalPointX= calibrationJson.at("principalPoint").at(0).get<double>();
				calibration.principalPointY= calibrationJson.at("principalPoint").at(1).get<double>();

				// In OpenCV's order: k1 k2 p1 p2 k3 k4 k5 k6
				const nlohmann::json& distortion= calibrationJson.at("distortion");
				calibration.distortion.k1= distortion.at(0).get<double>();
				calibration.distortion.k2= distortion.at(1).get<double>();
				calibration.distortion.p1= distortion.at(2).get<double>();
				calibration.distortion.p2= distortion.at(3).get<double>();
				calibration.distortion.k3= distortion.at(4).get<double>();
				calibration.distortion.k4= distortion.at(5).get<double>();
				calibration.distortion.k5= distortion.at(6).get<double>();
				calibration.distortion.k6= distortion.at(7).get<double>();

				if (calibration.width > 0 && calibration.height > 0 && calibration.focalLengthX > 0.0)
					camera.calibrations.push_back(calibration);
			}

			if (!camera.calibrations.empty())
				m_cameras.push_back(camera);
		}
	}
	catch (const std::exception& e)
	{
		MIKAN_LOG_ERROR("WebcamIntrinsicsTable::loadFromText") << "Malformed webcam table entry: " << e.what();
		m_cameras.clear();
		return false;
	}

	return true;
}

bool WebcamIntrinsicsTable::find(unsigned int vendorId, unsigned int productId, int frameWidth, int frameHeight,
								 WebcamTableMatch& outMatch) const
{
	if (frameWidth <= 0 || frameHeight <= 0)
		return false;

	for (const Camera& camera : m_cameras)
	{
		if (camera.vendorId != vendorId || camera.productId != productId)
			continue;

		// The calibration at this size, else the largest of the same aspect ratio
		const Calibration* best= nullptr;
		for (const Calibration& calibration : camera.calibrations)
		{
			if (calibration.width == frameWidth && calibration.height == frameHeight)
			{
				best= &calibration;
				break;
			}

			const bool bSameAspect= calibration.width * frameHeight == calibration.height * frameWidth;
			if (bSameAspect && (best == nullptr || calibration.width > best->width))
				best= &calibration;
		}
		if (best == nullptr)
			return false;

		const double scale= (double)frameWidth / (double)best->width;
		if (!createMonoIntrinsics(frameWidth, frameHeight, best->focalLengthX * scale, best->focalLengthY * scale,
								  best->principalPointX * scale, best->principalPointY * scale, best->distortion,
								  outMatch.intrinsics))
		{
			return false;
		}

		outMatch.cameraName= camera.name;
		outMatch.origin= camera.origin;
		outMatch.bScaled= best->width != frameWidth;
		return true;
	}

	return false;
}

bool WebcamIntrinsicsTable::findForDevicePath(const std::string& devicePath, int frameWidth, int frameHeight,
											  WebcamTableMatch& outMatch) const
{
	unsigned int vendorId= 0;
	unsigned int productId= 0;
	return parseUsbDeviceIds(devicePath, vendorId, productId)
		   && find(vendorId, productId, frameWidth, frameHeight, outMatch);
}
