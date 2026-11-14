#include "TestCommon.h"

#include "CalibrationPatternFinder_Chessboard.h"

static int runCharucoTest(const TestArgs& args)
{
	int result= 0;
	try
	{
		MIKAN_LOG_INFO("test-charuco") << "1: bare finder ctor";
		{
			CalibrationPatternFinder_Charuco finder(1280, 720, 11, 8, 16.f, 12.f,
													eCharucoDictionaryType::DICT_6X6);
		}

		MIKAN_LOG_INFO("test-charuco") << "2: ORT pipeline startup (mirrors app state)";
		HandTrackingPipeline pipeline;
		HandTrackingPipelineConfig pipelineConfig;
		pipeline.startup(pipelineConfig);

		MIKAN_LOG_INFO("test-charuco") << "3: MonoLensDistortionCalibrator ctor (wizard path)";
		// The default pattern is the 11x8 charuco board of the printed sheet
		const CalibrationPatternDesc charucoPattern;
		MonoLensDistortionCalibrator calibrator(1280, 720, charucoPattern, 12);

		MIKAN_LOG_INFO("test-charuco") << "4: update() with synthetic frames";
		cv::Mat grayFrame(720, 1280, CV_8UC1, cv::Scalar(128));
		for (int frame= 0; frame < 5; ++frame)
			calibrator.update(frame, frame * 0.033, &grayFrame);

		// 5: Regression test for the capture stall: render a synthetic
		// charuco board and run the wizard loop (update + overlay read
		// each frame). The overlay read must not commit capture state,
		// so the hold-still gate must fire and progress must advance.
		MIKAN_LOG_INFO("test-charuco") << "5: capture progress with a rendered board + overlay reads";
		{
			const cv::aruco::Dictionary dictionary=
				cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			const cv::aruco::CharucoBoard board(cv::Size(11, 8), 0.016f, 0.012f, dictionary);
			cv::Mat boardImage;
			// Generous margin: step 6 shifts the board 150px and detection
			// requires the ENTIRE board visible (no partial captures)
			board.generateImage(cv::Size(1100, 800), boardImage, 180, 1);

			MonoLensDistortionCalibrator boardCalibrator(1100, 800, charucoPattern, 12);

			// The same video frame handed over on several UI frames must
			// not count as a pattern being held still
			for (int repeat= 0; repeat < 40; ++repeat)
				boardCalibrator.update(0, repeat * 0.05, &boardImage);
			if (boardCalibrator.computeCalibrationProgress() > 0.f)
			{
				MIKAN_LOG_ERROR("test-charuco") << "REGRESSION: a repeated video frame was captured";
				result= 1;
			}

			t_opencv_point2d_list overlayPoints;
			cv::Point2f overlayQuad[4];
			int frameSequence= 1;
			double frameSeconds= 10.0;
			for (int frame= 0; frame < 40; ++frame, ++frameSequence, frameSeconds+= 0.05) // 2s, several holds long
			{
				boardCalibrator.update(frameSequence, frameSeconds, &boardImage);
				// Mimic the wizard overlay read every frame
				boardCalibrator.getPatternFinder()->getCurrentCalibrationPattern(overlayPoints, overlayQuad);
			}

			if (boardCalibrator.computeCalibrationProgress() <= 0.f)
			{
				MIKAN_LOG_ERROR("test-charuco")
					<< "REGRESSION: capture progress stuck at 0 with overlay reads active";
				result= 1;
			}
			else
			{
				MIKAN_LOG_INFO("test-charuco")
					<< "Captured " << boardCalibrator.computeCalibrationProgress() * 12.f
					<< " samples from static board (expected 1)";
			}

			// 6: Regression test for the stuck-green stall: right after a
			// capture, present the board >= 100px away with NO invalid
			// frame in between (fast board move). The hold cycle must
			// restart and produce a second capture.
			MIKAN_LOG_INFO("test-charuco") << "6: second capture after an immediate fast board move";
			{
				cv::Mat shiftedImage;
				const cv::Mat translation= (cv::Mat_<double>(2, 3) << 1, 0, 150, 0, 1, 0);
				cv::warpAffine(boardImage, shiftedImage, translation, boardImage.size(),
							   cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));

				for (int frame= 0; frame < 40; ++frame, ++frameSequence, frameSeconds+= 0.05)
				{
					boardCalibrator.update(frameSequence, frameSeconds, &shiftedImage);
					boardCalibrator.getPatternFinder()->getCurrentCalibrationPattern(overlayPoints,
																					 overlayQuad);
				}

				const float capturedSamples= boardCalibrator.computeCalibrationProgress() * 12.f;
				if (capturedSamples < 2.f)
				{
					MIKAN_LOG_ERROR("test-charuco")
						<< "REGRESSION: no capture after fast board move (stuck hold state), samples="
						<< capturedSamples;
					result= 1;
				}
				else
				{
					MIKAN_LOG_INFO("test-charuco")
						<< "Captured " << capturedSamples << " samples after fast move (expected 2)";
				}
			}
		}

		// A board that keeps moving is never held still, so nothing is
		// captured however long it stays in view
		MIKAN_LOG_INFO("test-charuco") << "6b: no capture from a moving board";
		{
			const cv::aruco::Dictionary dictionary= cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			const cv::aruco::CharucoBoard board(cv::Size(11, 8), 0.016f, 0.012f, dictionary);
			cv::Mat boardImage;
			board.generateImage(cv::Size(1100, 800), boardImage, 180, 1);

			MonoLensDistortionCalibrator movingCalibrator(1100, 800, charucoPattern, 12);
			for (int frame= 0; frame < 40; ++frame)
			{
				// 3 px a frame at 20 fps, twice the hold-still allowance over its baseline
				cv::Mat movedImage;
				const cv::Mat translation= (cv::Mat_<double>(2, 3) << 1, 0, -60.0 + 3.0 * frame, 0, 1, 0);
				cv::warpAffine(boardImage, movedImage, translation, boardImage.size(), cv::INTER_LINEAR,
							   cv::BORDER_CONSTANT, cv::Scalar(255));
				movingCalibrator.update(frame, frame * 0.05, &movedImage);
			}

			if (movingCalibrator.computeCalibrationProgress() > 0.f)
			{
				MIKAN_LOG_ERROR("test-charuco") << "REGRESSION: a moving board was captured";
				result= 1;
			}
		}

		// The phone's plain chessboard through the same calibrator: found
		// whole, captured once held still, and captured again after a move
		MIKAN_LOG_INFO("test-charuco") << "6c: chessboard capture";
		{
			const CalibrationPatternDesc chessboardPattern= CalibrationPatternFinder_Chessboard::makePhoneBoardDesc();
			const cv::Mat boardImage=
				renderChessboardImage(chessboardPattern.cols, chessboardPattern.rows, 60);
			// Room around the board for the shifted pose
			cv::Mat frameImage(boardImage.rows + 200, boardImage.cols + 400, CV_8UC1, cv::Scalar(255));
			boardImage.copyTo(frameImage(cv::Rect(100, 100, boardImage.cols, boardImage.rows)));
			cv::Mat shiftedImage(frameImage.size(), CV_8UC1, cv::Scalar(255));
			boardImage.copyTo(shiftedImage(cv::Rect(250, 100, boardImage.cols, boardImage.rows)));

			MonoLensDistortionCalibrator chessCalibrator(frameImage.cols, frameImage.rows, chessboardPattern, 12);
			int frameSequence= 0;
			for (int frame= 0; frame < 20; ++frame, ++frameSequence)
				chessCalibrator.update(frameSequence, frameSequence * 0.05, &frameImage);
			const int firstCount= chessCalibrator.getCapturedSampleCount();
			for (int frame= 0; frame < 20; ++frame, ++frameSequence)
				chessCalibrator.update(frameSequence, frameSequence * 0.05, &shiftedImage);

			if (firstCount != 1 || chessCalibrator.getCapturedSampleCount() != 2)
			{
				MIKAN_LOG_ERROR("test-charuco")
					<< "REGRESSION: chessboard captures " << firstCount << " then "
					<< chessCalibrator.getCapturedSampleCount() << " (expected 1 then 2)";
				result= 1;
			}
		}

		// 7: End-to-end intrinsics recovery through a KNOWN synthetic
		// camera: project the board's corner geometry through ground-truth
		// intrinsics at a spread of tilted poses and verify the solve
		// recovers the true hfov. Regression guard for the free-k6
		// rational-model bug (hfov of 180/8.6 deg with sub-pixel
		// reprojection error) and the new plausibility gate.
		MIKAN_LOG_INFO("test-charuco") << "7: synthetic intrinsics recovery";
		{
			const int width= 1280, height= 720;
			const double fx= 800.0;
			const double truthHfov= 2.0 * glm::degrees(atan((width * 0.5) / fx)); // 77.32 deg

			// 10x7 corner grid, 16mm squares (matches an 11x8 board)
			OpenCVCalibrationGeometry geometry;
			for (int row= 0; row < 7; ++row)
				for (int col= 0; col < 10; ++col)
					geometry.points.push_back(cv::Point3f(col * 16.f, row * 16.f, 0.f));

			const cv::Matx33d cameraMatrix(fx, 0, width * 0.5, 0, fx, height * 0.5, 0, 0, 1);
			const cv::Mat noDistortion= cv::Mat::zeros(1, 8, CV_64F);

			std::vector<t_opencv_point2d_list> imagePointsList;
			std::vector<t_opencv_pointID_list> imagePointIDList;
			// 12 poses: varied tilt about X and Y, varied depth + offset
			for (int sample= 0; sample < 12; ++sample)
			{
				const double tiltX= glm::radians(-25.0 + 10.0 * (sample % 3));
				const double tiltY= glm::radians(-20.0 + 8.0 * (sample % 5));
				const cv::Vec3d rvecX(tiltX, 0, 0), rvecY(0, tiltY, 0);
				cv::Matx33d rotX, rotY;
				cv::Rodrigues(rvecX, rotX);
				cv::Rodrigues(rvecY, rotY);
				cv::Vec3d rvec;
				cv::Rodrigues(rotY * rotX, rvec);
				const cv::Vec3d tvec(-80.0 + 30.0 * (sample % 4), -60.0 + 25.0 * (sample % 3),
									 350.0 + 40.0 * sample);

				t_opencv_point2d_list imagePoints;
				cv::projectPoints(geometry.points, rvec, tvec, cameraMatrix, noDistortion, imagePoints);

				t_opencv_pointID_list pointIDs;
				for (int id= 0; id < (int)geometry.points.size(); ++id)
					pointIDs.push_back(id);

				imagePointsList.push_back(imagePoints);
				imagePointIDList.push_back(pointIDs);
			}

			MikanMonoIntrinsics recovered;
			double reprojectionError= 0.0;
			const bool bSolved= computeMonoLensCameraCalibration(
				width, height, geometry, imagePointsList, imagePointIDList, recovered, reprojectionError);

			MIKAN_LOG_INFO("test-charuco")
				<< "recovered hfov=" << recovered.hfov << " (truth " << truthHfov
				<< ") reprojection=" << reprojectionError << "px";
			if (!bSolved || fabs(recovered.hfov - truthHfov) > 2.0 || reprojectionError > 0.5)
			{
				MIKAN_LOG_ERROR("test-charuco")
					<< "REGRESSION: synthetic intrinsics recovery failed";
				result= 1;
			}
		}

		MIKAN_LOG_INFO("test-charuco") << "All steps passed";
	}
	catch (const cv::Exception& e)
	{
		MIKAN_LOG_ERROR("test-charuco") << "cv::Exception: " << e.what();
		result= 1;
	}
	catch (const std::exception& e)
	{
		MIKAN_LOG_ERROR("test-charuco") << "std::exception: " << e.what();
		result= 1;
	}

	return result;
}

MIKAN_REGISTER_TEST("--test-charuco", "Charuco board finder + synthetic intrinsics recovery", eTestCategory::SelfTest, runCharucoTest);
