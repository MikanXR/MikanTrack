#include "TestCommon.h"

#include <cstdio>
#include <iomanip>

#include "PathUtils.h"
#include "WebcamBlink.h"
#include "WebcamFace.h"

// Runs the Blink stage over the face points a recording holds for its front
// camera, in model frame order from the take's own neutral, so a change to
// the stage can be judged against old recordings without the model or the
// raw frames. The settings are the recorded project's unless --defaults asks
// for the stage's own.
//
//   --replay-blink <recording.jsonl> [out.csv] [--defaults]
//
// The CSV holds one row per model frame: the frame's index and time, then per
// eye by the person's own side the openness, the baseline, the floor, the
// state, the closure, whether a wink is held and the eyes' difference, then
// the two output columns.

static int runReplayBlink(const TestArgs& args)
{
	const char* tag= "replay-blink";
	std::filesystem::path recordingPath;
	std::filesystem::path outPath;
	bool bDefaults= false;
	int positional= 0;
	bool bUsage= false;
	for (const std::string& arg : args)
	{
		if (arg == "--defaults")
			bDefaults= true;
		else if (arg.rfind("--", 0) == 0)
			bUsage= true;
		else if (positional == 0)
			recordingPath= PathUtils::utf8ToPath(arg);
		else if (positional == 1)
			outPath= PathUtils::utf8ToPath(arg);
		else
			bUsage= true;
		if (arg.rfind("--", 0) != 0)
			++positional;
	}
	if (bUsage || recordingPath.empty())
	{
		MIKAN_LOG_ERROR(tag) << "usage: --replay-blink <recording.jsonl> [out.csv] [--defaults]";
		return 1;
	}

	TrackingReplay replay;
	std::string error;
	if (!replay.load(PathUtils::pathToUtf8(recordingPath), error))
	{
		MIKAN_LOG_ERROR(tag) << "Load failed: " << error;
		return 1;
	}
	const AppConfig& config= replay.getRecordedConfig();
	const FrontCameraConfig& frontCamera= config.frontCamera;
	if (!frontCamera.enabled || frontCamera.cameraIndex < 0)
	{
		MIKAN_LOG_ERROR(tag) << "the recording is not of a front-camera project, so it has no camera the stage reads";
		return 1;
	}
	const BlinkConfig blinkConfig= bDefaults ? BlinkConfig() : config.blink;
	const WholeBodyFaceRatios* neutral= frontCamera.neutralFacePresent ? &frontCamera.neutralFace : nullptr;

	std::ofstream csv;
	if (!outPath.empty())
	{
		csv.open(outPath);
		if (!csv)
		{
			MIKAN_LOG_ERROR(tag) << "cannot write " << outPath.string();
			return 1;
		}
		csv << "frameIndex,timestampMs,tracked";
		for (const char* eye : {"left", "right"})
		{
			csv << "," << eye << "Openness," << eye << "Baseline," << eye << "Floor," << eye << "State," << eye
				<< "Closure," << eye << "WinkHeld," << eye << "Difference";
		}
		csv << ",eyeBlinkLeft,eyeBlinkRight\n";
	}

	WebcamBlinkFilter filter;
	filter.reset(neutral);
	int modelFrames= 0;
	int trackedFrames= 0;
	// Per output column: the frames over a half, the runs of them, and the
	// frames a wink was held on the eye the column reads
	int overHalf[2]= {0, 0};
	int events[2]= {0, 0};
	int heldFrames[2]= {0, 0};
	bool bWasOver[2]= {false, false};
	int64_t lastModelFrameIndex= -1;
	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const RecordedFrame& recorded= *replay.getFrame(frameIndex).recorded;
		for (const RecordedCameraInput& fresh : recorded.freshCameras)
		{
			if (fresh.cameraIndex != frontCamera.cameraIndex || !fresh.valid || !fresh.bHaveBodyPose ||
				!fresh.body.valid || !fresh.body.wholeBody.present || fresh.body.modelFrameIndex == lastModelFrameIndex)
				continue;
			lastModelFrameIndex= fresh.body.modelFrameIndex;

			FaceSources::Blink blink;
			filter.update(fresh.body, true, fresh.timestampMs, blinkConfig, blink);
			++modelFrames;
			trackedFrames+= blink.tracked ? 1 : 0;
			const float outputs[2]= {blink.eyeBlinkLeft, blink.eyeBlinkRight};
			const int sideOfOutput[2]= {k_ratioSideOfArkitLeft, k_ratioSideOfArkitRight};
			for (int output= 0; output < 2; ++output)
			{
				const bool bOver= outputs[output] > 0.5f;
				overHalf[output]+= bOver ? 1 : 0;
				events[output]+= bOver && !bWasOver[output] ? 1 : 0;
				bWasOver[output]= bOver;
				heldFrames[output]+= blink.bWinkHeld[sideOfOutput[output]] ? 1 : 0;
			}

			if (csv.is_open())
			{
				csv << fresh.frameIndex << "," << std::fixed << std::setprecision(3) << fresh.timestampMs << ","
					<< (blink.tracked ? 1 : 0) << std::setprecision(5);
				for (int side= 0; side < 2; ++side)
				{
					csv << "," << blink.openness[side] << "," << blink.baseline[side] << "," << blink.floor[side] << ","
						<< (int)blink.state[side] << "," << blink.closure[side] << "," << (blink.bWinkHeld[side] ? 1 : 0)
						<< "," << blink.difference[side];
				}
				csv << "," << blink.eyeBlinkLeft << "," << blink.eyeBlinkRight << "\n";
			}
		}
	}
	if (modelFrames == 0)
	{
		MIKAN_LOG_ERROR(tag) << "the recording holds no whole-body face points for camera " << frontCamera.cameraIndex;
		return 1;
	}

	auto percent= [&](int count) {
		char text[16];
		snprintf(text, sizeof(text), "%.1f", 100.0 * (double)count / (double)modelFrames);
		return std::string(text);
	};
	MIKAN_LOG_INFO(tag) << recordingPath.filename().string() << ": " << modelFrames << " model frames of camera "
						<< frontCamera.cameraIndex << ", the face tracked on " << trackedFrames << ", "
						<< (bDefaults ? "default settings" : "the recorded settings") << " (sensitivity "
						<< blinkConfig.sensitivity << ", wink threshold " << blinkConfig.winkThreshold << ", clarity "
						<< blinkConfig.clarity << ")";
	static const char* const kOutputNames[2]= {"eyeBlinkLeft", "eyeBlinkRight"};
	for (int output= 0; output < 2; ++output)
	{
		MIKAN_LOG_INFO(tag) << "  " << kOutputNames[output] << ": over a half on " << percent(overHalf[output])
							<< "% of the frames in " << events[output] << " runs, a wink held on "
							<< percent(heldFrames[output]) << "%";
	}
	if (csv.is_open())
		MIKAN_LOG_INFO(tag) << "wrote " << outPath.string();
	return 0;
}

MIKAN_REGISTER_TEST("--replay-blink",
					"Runs the Blink stage over a recording's own face points and reports its output, with a per-frame CSV",
					eTestCategory::Tool, runReplayBlink)
