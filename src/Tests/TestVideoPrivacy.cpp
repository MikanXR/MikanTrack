#include "TestCommon.h"

#include "StreamingSoftware.h"
#include "VideoPrivacy.h"

// The camera video rule without a window: which cameras are shown under the
// user's choices and the calibration grants and what clears a choice, the
// pixelated preview's blocks, and the streaming program names.
static int runVideoPrivacyTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO("test-video-privacy") << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR("test-video-privacy") << "FAIL " << name;
			failures++;
		}
	};

	// (a) Hidden until the user says otherwise, one camera at a time
	{
		VideoPrivacyState state;
		check(!state.isAnyVisible() && !state.isVisible(0), "(a) a state with no cameras shows none");
		state.setCameras({"Front", "Desk"});
		check(!state.isVisible(0) && !state.isVisible(1) && !state.isAnyVisible(), "(a) every camera starts hidden");
		state.setUserShown(1, true);
		check(!state.isVisible(0) && state.isVisible(1) && state.isAnyVisible(),
			  "(a) showing one camera leaves the other hidden");
		state.setCameras({"Front", "Desk"});
		check(state.isVisible(1), "(a) the same devices on a later frame keep the choice");
		state.setUserShown(1, false);
		check(!state.isAnyVisible(), "(a) hiding it hides it");
		state.setUserShown(5, true);
		state.setUserShown(-1, true);
		check(!state.isAnyVisible() && !state.isVisible(5) && !state.isVisible(-1),
			  "(a) a camera the state does not have is never shown");
	}

	// (b) A calibration's grant shows its cameras without becoming a choice
	{
		VideoPrivacyState state;
		state.setCameras({"Front", "Desk", "Side"});
		state.setGrants(1, false);
		check(!state.isVisible(0) && state.isVisible(1) && !state.isVisible(2) && state.isGranted(1),
			  "(b) one camera's grant shows that camera alone");
		state.setGrants(-1, true);
		check(state.isVisible(0) && state.isVisible(1) && state.isVisible(2), "(b) a grant for all shows every camera");
		check(!state.getUserShown(0) && !state.getUserShown(1) && !state.getUserShown(2),
			  "(b) a grant is not the user's choice");
		state.setGrants(-1, false);
		check(!state.isAnyVisible(), "(b) the cameras are hidden again when the grant ends");
		state.setGrants(7, false);
		check(!state.isAnyVisible() && !state.isGranted(7), "(b) a grant for a camera the state does not have shows none");
	}

	// (c) What clears a choice
	{
		VideoPrivacyState state;
		state.setCameras({"Front", "Desk"});
		state.setUserShown(0, true);
		state.setUserShown(1, true);
		state.setCameras({"Front", "Other"});
		check(state.isVisible(0) && !state.isVisible(1), "(c) another device in a camera's slot clears that camera alone");
		state.setCameras({"", "Other"});
		state.setCameras({"Front", "Other"});
		check(!state.isVisible(0), "(c) a device closed and opened again comes back hidden");

		state.setUserShown(0, true);
		state.setCameras({"Front", "Other", "Side"});
		check(!state.isAnyVisible(), "(c) a changed camera count clears every choice");

		state.setUserShown(2, true);
		state.setGrants(0, false);
		state.reset();
		check(!state.isAnyVisible() && !state.isGranted(0) && state.getCameraCount() == 3,
			  "(c) a reset clears the choices and the grants and keeps the cameras");
	}

	// (d) The pixelated preview: its size, and that each block is one color
	{
		cv::Mat frame(360, 640, CV_8UC3);
		cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
		const int blockColumns= 48;
		const cv::Mat preview= makePixelatedPreview(frame, blockColumns, 384);
		check(preview.cols == 384 && preview.rows == 216 && preview.type() == CV_8UC3,
			  "(d) a 16:9 frame makes a 384 by 216 preview");

		const int blockSize= preview.cols / blockColumns;
		bool bUniformBlocks= !preview.empty() && blockSize == 8;
		for (int row= 0; bUniformBlocks && row < preview.rows; ++row)
		{
			for (int column= 0; column < preview.cols; ++column)
			{
				const cv::Vec3b corner= preview.at<cv::Vec3b>(row - row % blockSize, column - column % blockSize);
				if (preview.at<cv::Vec3b>(row, column) != corner)
				{
					bUniformBlocks= false;
					break;
				}
			}
		}
		check(bUniformBlocks, "(d) every 8 pixel block of the preview is one color");

		// A block is the mean of the frame under it, so no source pixel
		// survives: a lone white pixel on black comes out dark
		cv::Mat dot(360, 640, CV_8UC3, cv::Scalar(0, 0, 0));
		dot.at<cv::Vec3b>(100, 200)= cv::Vec3b(255, 255, 255);
		const cv::Mat dotPreview= makePixelatedPreview(dot, blockColumns, 384);
		double maxValue= 0.0;
		cv::minMaxLoc(dotPreview.reshape(1), nullptr, &maxValue);
		check(maxValue < 8.0, "(d) a single bright pixel is averaged away");

		const cv::Mat odd= makePixelatedPreview(cv::Mat(480, 640, CV_8UC3, cv::Scalar(10, 20, 30)), blockColumns, 400);
		check(odd.cols == 384 && odd.rows == 288, "(d) the width is the widest whole number of blocks that fits");
		const cv::Mat narrow= makePixelatedPreview(cv::Mat(10, 20, CV_8UC3, cv::Scalar(1, 2, 3)), blockColumns, 384);
		check(narrow.cols == 380 && narrow.rows == 190, "(d) a frame narrower than the block count keeps its own columns");
		check(makePixelatedPreview(cv::Mat(), blockColumns, 384).empty() &&
				  makePixelatedPreview(frame, 0, 384).empty(),
			  "(d) an empty frame or no blocks makes no preview");
	}

	// (e) The streaming programs' executable names
	{
		auto matches= [](const char* exeName, const char* expected) {
			const char* name= matchStreamingProcessName(exeName);
			return name != nullptr && std::string(name) == expected;
		};
		check(matches("obs64.exe", "OBS Studio") && matches("obs32.exe", "OBS Studio") &&
				  matches("obs.exe", "OBS Studio"),
			  "(e) the OBS executables are OBS Studio");
		check(matches("Streamlabs OBS.exe", "Streamlabs Desktop") &&
				  matches("Streamlabs Desktop.exe", "Streamlabs Desktop"),
			  "(e) the Streamlabs executables are Streamlabs Desktop");
		check(matches("OBS64.EXE", "OBS Studio") && matches("streamlabs obs.EXE", "Streamlabs Desktop"),
			  "(e) letter case is ignored");
		check(matchStreamingProcessName("obs64") == nullptr && matchStreamingProcessName("jobs.exe") == nullptr &&
				  matchStreamingProcessName("obs64.exe.bak") == nullptr &&
				  matchStreamingProcessName("MikanTrack.exe") == nullptr && matchStreamingProcessName("") == nullptr,
			  "(e) a name that only resembles one is no streaming program");

		// The process list reads without failing, whatever is running
		StreamingSoftwareWatch watch;
		const std::string first= watch.poll();
		check(watch.poll() == first, "(e) a second poll inside the rescan interval answers as the first did");
		MIKAN_LOG_INFO("test-video-privacy") << "  running streaming program: " << (first.empty() ? "none" : first);
	}

	MIKAN_LOG_INFO("test-video-privacy") << (failures == 0 ? "ALL PASS" : "FAILURES") << " (" << failures << ")";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-video-privacy",
					"Camera video: the shown cameras, the pixelated preview and the streaming program names",
					eTestCategory::SelfTest, runVideoPrivacyTest);
