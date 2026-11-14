#include "TestCommon.h"

#include "CalibrationPatternFinder_Chessboard.h"

// Runs the intrinsics wizard's capture and solve over a folder of recorded
// frames: the pattern finder, the hold-still gate, and the lens solve. The
// check for a change to the gate or a finder without someone holding a
// pattern up to a camera.
//
// The folder holds the frames and a frames.csv naming each with its capture
// time ("file,seconds" under a header line). Every sample the gate takes is
// kept, with no cap, so the count says how the gate behaved on the footage.
//
// A trailing .json path receives the captured views, the file the wizard
// writes after its own solve.
//
// usage: --replay-intrinsics <frames_dir> chessboard|charuco [cols rows] [capture_set.json]

static int runReplayIntrinsics(const TestArgs& allArgs)
{
	TestArgs args= allArgs;
	std::filesystem::path captureSetPath;
	if (!args.empty() && std::filesystem::path(args.back()).extension() == ".json")
	{
		captureSetPath= args.back();
		args.pop_back();
	}

	const char* tag= "replay-intrinsics";
	if (args.size() < 2 || (args.size() != 2 && args.size() != 4))
	{
		MIKAN_LOG_ERROR(tag)
			<< "usage: --replay-intrinsics <frames_dir> chessboard|charuco [cols rows] [capture_set.json]";
		return 1;
	}

	const std::filesystem::path framesDirectory= args[0];
	CalibrationPatternDesc pattern;
	if (args[1] == "chessboard")
	{
		pattern= CalibrationPatternFinder_Chessboard::makePhoneBoardDesc();
	}
	else if (args[1] != "charuco")
	{
		MIKAN_LOG_ERROR(tag) << "Unknown pattern '" << args[1] << "'";
		return 1;
	}
	if (args.size() == 4)
	{
		pattern.cols= atoi(args[2].c_str());
		pattern.rows= atoi(args[3].c_str());
	}

	std::ifstream frameList(framesDirectory / "frames.csv");
	if (!frameList.is_open())
	{
		MIKAN_LOG_ERROR(tag) << "No frames.csv in " << framesDirectory.string();
		return 1;
	}

	std::unique_ptr<MonoLensDistortionCalibrator> calibrator;
	int frameCount= 0;
	int foundCount= 0;
	double lastFrameSeconds= 0.0;
	std::string line;
	std::getline(frameList, line); // header
	while (std::getline(frameList, line))
	{
		if (!line.empty() && line.back() == '\r')
			line.pop_back();
		const size_t comma= line.find(',');
		if (comma == std::string::npos)
			continue;

		const cv::Mat grayFrame=
			cv::imread((framesDirectory / line.substr(0, comma)).string(), cv::IMREAD_GRAYSCALE);
		if (grayFrame.empty())
			continue;
		lastFrameSeconds= atof(line.substr(comma + 1).c_str());

		if (calibrator == nullptr)
		{
			// No cap on the sample count, which also leaves the tilt quota open
			calibrator= std::make_unique<MonoLensDistortionCalibrator>(grayFrame.cols, grayFrame.rows, pattern,
																		 100000);
		}

		calibrator->update(frameCount, lastFrameSeconds, &grayFrame);
		if (calibrator->areCurrentImagePointsValid())
			++foundCount;
		++frameCount;
	}

	if (calibrator == nullptr)
	{
		MIKAN_LOG_ERROR(tag) << "No readable frames in " << framesDirectory.string();
		return 1;
	}

	const int capturedCount= calibrator->getCapturedSampleCount();
	MIKAN_LOG_INFO(tag) << frameCount << " frames over " << lastFrameSeconds << " s, pattern in view on "
						<< foundCount << ", captured " << capturedCount << " samples ("
						<< calibrator->getTiltedSampleCount() << " tilted)";
	if (capturedCount < 4)
	{
		MIKAN_LOG_ERROR(tag) << "Too few samples to solve";
		return 1;
	}

	// The footage is over, so views the solve drops are not replaced: the
	// solve runs again on what is left
	while (true)
	{
		calibrator->computeCameraCalibration();
		while (!calibrator->getIsCameraCalibrationComplete())
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		if (!calibrator->wantsReplacementSamples())
			break;
		calibrator->resumeCapture();
	}
	MIKAN_LOG_INFO(tag) << "dropped " << calibrator->getDroppedSampleCount() << " outlier views, solved from "
						<< calibrator->getCapturedSampleCount();

	if (!captureSetPath.empty() && calibrator->writeCaptureSet(captureSetPath))
		MIKAN_LOG_INFO(tag) << "Capture set written to " << captureSetPath.string();

	MikanMonoIntrinsics intrinsics;
	if (!calibrator->getCameraCalibration(&intrinsics))
	{
		MIKAN_LOG_ERROR(tag) << "The solve did not converge";
		return 1;
	}

	MIKAN_LOG_INFO(tag) << "fx=" << intrinsics.distorted_camera_matrix.x0 << " hfov=" << intrinsics.hfov
						<< " k1=" << intrinsics.distortion_coefficients.k1
						<< " k2=" << intrinsics.distortion_coefficients.k2
						<< " reprojection=" << calibrator->getReprojectionError() << "px";
	return 0;
}

MIKAN_REGISTER_TEST("--replay-intrinsics",
					"Runs the intrinsics capture gate and lens solve over a folder of recorded frames",
					eTestCategory::Tool, runReplayIntrinsics);
