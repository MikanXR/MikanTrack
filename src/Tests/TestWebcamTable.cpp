#include "TestCommon.h"

#include "CameraMath.h"
#include "UsbDeviceIds.h"
#include "WebcamIntrinsicsTable.h"

// The lens a camera gets without a calibration: the shipped webcam table's
// lookup rules, the pinhole from a typed field of view, and the intrinsics
// source through a config save and load

static int runWebcamTableTest(const TestArgs& args)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO("test-webcam-table") << "PASS: " << name;
		}
		else
		{
			MIKAN_LOG_ERROR("test-webcam-table") << "FAIL: " << name;
			++failures;
		}
	};

	// (a) USB ids out of a device path
	{
		unsigned int vendorId= 0, productId= 0;
		const bool bParsed= parseUsbDeviceIds(
			"\\\\?\\usb#vid_046d&pid_082d&mi_00#7&1a2b3c4d&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\\global",
			vendorId, productId);
		check(bParsed && vendorId == 0x046d && productId == 0x082d, "(a) the ids are read from a USB device path");

		const bool bUpperParsed= parseUsbDeviceIds("\\\\?\\USB#VID_07CA&PID_315A#123", vendorId, productId);
		check(bUpperParsed && vendorId == 0x07ca && productId == 0x315a, "(a) an upper case path reads the same");

		check(!parseUsbDeviceIds("\\\\?\\root#image#0000#{e5323777}\\global", vendorId, productId),
			  "(a) a path that is not USB has no ids");
	}

	// (b) The shipped table
	WebcamIntrinsicsTable table;
	const bool bLoaded= table.load(WebcamIntrinsicsTable::getDefaultPath());
	check(bLoaded && table.getCameraCount() >= 7, "(b) the shipped table loads");
	{
		WebcamTableMatch match;

		// The C920 at its own calibrated size, with the measured principal point kept
		const bool bExact= table.find(0x046d, 0x082d, 1920, 1080, match);
		check(bExact && !match.bScaled && fabs(match.intrinsics.distorted_camera_matrix.x0 - 1385.92) < 1e-6
				  && fabs(match.intrinsics.distortion_coefficients.k1 - 0.117627) < 1e-9
				  && fabs(match.intrinsics.distortion_coefficients.k3 - 0.107441) < 1e-9
				  && match.intrinsics.hfov > 60.0 && match.intrinsics.hfov < 80.0,
			  "(b) a calibrated size is returned as stored");

		// A size the table lacks scales from the largest calibration of its aspect ratio
		const bool bScaled= table.find(0x046d, 0x082d, 1280, 720, match);
		check(bScaled && match.bScaled
				  && fabs(match.intrinsics.distorted_camera_matrix.x0 - 1385.92 * 1280.0 / 1920.0) < 1e-6
				  && fabs(match.intrinsics.distorted_camera_matrix.z0 - 951.982 * 1280.0 / 1920.0) < 1e-6
				  && match.intrinsics.pixel_width == 1280.0 && match.intrinsics.pixel_height == 720.0,
			  "(b) a 16:9 size scales the 1920x1080 calibration by the width ratio");
		const bool bScaledNarrow= table.find(0x046d, 0x082d, 1024, 768, match);
		check(bScaledNarrow && fabs(match.intrinsics.distorted_camera_matrix.x0 - 775.79 * 1024.0 / 800.0) < 1e-6,
			  "(b) a 4:3 size scales the largest 4:3 calibration, not a 16:9 one");

		// Every camera's lens passes the plausibility window at a size it lists
		const struct
		{
			unsigned int vendorId, productId;
			int width, height;
		} listed[]= {{0x045e, 0x0779, 640, 480}, {0x045e, 0x0810, 640, 480}, {0x046d, 0x081b, 640, 480},
					 {0x046d, 0x081b, 640, 360}, {0x046d, 0x0825, 640, 480}, {0x07ca, 0x315a, 1280, 720},
					 {0x328f, 0x0121, 1280, 720}};
		bool bAllListed= true;
		for (const auto& entry : listed)
			bAllListed&= table.find(entry.vendorId, entry.productId, entry.width, entry.height, match);
		check(bAllListed, "(b) every listed camera resolves at a size it was calibrated at");

		check(!table.find(0x046d, 0x0825, 1280, 720, match),
			  "(b) a size of an aspect ratio the camera was not calibrated at is no match");
		check(!table.find(0x1234, 0x5678, 1280, 720, match), "(b) an unlisted camera is no match");
		check(table.findForDevicePath("\\\\?\\usb#vid_328f&pid_0121&mi_00#abc", 1280, 720, match)
				  && match.cameraName.find("C960") != std::string::npos,
			  "(b) a device path finds its camera");
		check(!table.findForDevicePath("\\\\?\\root#image#0000", 1280, 720, match),
			  "(b) a device path that is not USB is no match");
	}

	// (c) A table that does not parse leaves nothing loaded
	{
		WebcamIntrinsicsTable broken;
		check(!broken.loadFromText("{\"cameras\": [{\"vendorId\": \"046d\"}]}") && broken.getCameraCount() == 0,
			  "(c) a malformed entry fails the load");
		check(!broken.loadFromText("not json") && broken.getCameraCount() == 0, "(c) text that is not JSON fails");
	}

	// (d) The pinhole from a field of view across the diagonal
	{
		MikanMonoIntrinsics nominal;
		const bool bCreated= createNominalMonoIntrinsics(1280, 720, 78.0, nominal);
		const double focalLength= nominal.distorted_camera_matrix.x0;
		const double halfDiagonal= 0.5 * sqrt(1280.0 * 1280.0 + 720.0 * 720.0);
		const double diagonalFov= 2.0 * glm::degrees(atan(halfDiagonal / focalLength));
		check(bCreated && fabs(diagonalFov - 78.0) < 1e-6 && fabs(nominal.hfov - 70.43) < 0.05
				  && nominal.distorted_camera_matrix.z0 == 640.0 && nominal.distorted_camera_matrix.z1 == 360.0
				  && nominal.distortion_coefficients.k1 == 0.0
				  && fabs(nominal.undistorted_camera_matrix.x0 - focalLength) < 1e-3,
			  "(d) a 78 degree diagonal gives a centred pinhole with that field of view");
		check(!createNominalMonoIntrinsics(1280, 720, 0.0, nominal) && !createNominalMonoIntrinsics(1280, 720, 180.0, nominal),
			  "(d) a field of view outside 0 to 180 is refused");
	}

	// (e) The source through a save and a load
	{
		const std::filesystem::path tempDir= std::filesystem::temp_directory_path() / "mikan_webcam_table_test";
		std::error_code ec;
		std::filesystem::create_directories(tempDir, ec);
		const std::filesystem::path projectPath= tempDir / "project.json";

		AppConfig saved;
		saved.setProjectFilePath(projectPath);
		saved.camera(0).intrinsics.present= true;
		saved.camera(0).intrinsics.source= eIntrinsicsSource::Nominal;
		createNominalMonoIntrinsics(1280, 720, 78.0, saved.camera(0).intrinsics.intrinsics);
		const bool bSaved= saved.save();

		AppConfig loaded;
		const bool bLoadedConfig= loaded.load(projectPath);
		check(bSaved && bLoadedConfig && loaded.camera(0).intrinsics.present
				  && loaded.camera(0).intrinsics.source == eIntrinsicsSource::Nominal
				  && !loaded.camera(0).intrinsics.isCalibrated(),
			  "(e) a nominal source survives a save and a load, and is not calibrated");

		saved.camera(0).intrinsics.source= eIntrinsicsSource::Table;
		saved.save();
		loaded.load(projectPath);
		check(loaded.camera(0).intrinsics.source == eIntrinsicsSource::Table, "(e) a table source survives too");

		// A config from before the source existed reads as calibrated
		nlohmann::json legacy= nlohmann::json::parse(std::ifstream(projectPath), nullptr, false);
		legacy["cameras"][0]["intrinsics"].erase("source");
		std::ofstream(projectPath) << legacy.dump();
		loaded.load(projectPath);
		check(loaded.camera(0).intrinsics.isCalibrated(), "(e) a config without the key reads as calibrated");
	}

	MIKAN_LOG_INFO("test-webcam-table") << (failures == 0 ? "ALL PASS" : "FAILURES") << " (" << failures << ")";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-webcam-table",
					"Webcam table: USB ids, table lookup and scaling, the nominal pinhole, the intrinsics source",
					eTestCategory::SelfTest, runWebcamTableTest);
