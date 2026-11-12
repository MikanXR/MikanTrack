#include "TestCommon.h"

#include <map>

#include "FrameRecorder.h"
#include "HandLandmarkModel.h"

// Re-runs the hand landmark model over the RAW FRAMES stored beside a
// recording, the A/B for a change to the landmark crop or model. Each frame's
// recorded hands are re-estimated from the ROI the live pipeline used for
// them: the previous fresh frame's landmarks of the same slot. Holding the
// ROI to the recorded one keeps the comparison per frame, where a stateful
// re-run would compound one frame's difference into the next frame's crop.
//
// The reference is the recorded landmarks, which the live pipeline computed
// from the uncompressed frame. The stored frames are JPEG, so a run on the
// unchanged model shows the compression's share of the difference, the
// baseline a crop change is read against.
//
// usage: --replay-hands <recording.jsonl> [cameraIndex] [out.csv]

namespace
{
float percentile(std::vector<float> values, float fraction)
{
	if (values.empty())
		return 0.f;
	std::sort(values.begin(), values.end());
	return values[std::min((size_t)(values.size() * fraction), values.size() - 1)];
}

struct SizeBucket
{
	std::vector<float> estimateMs;
	std::vector<float> meanErrorPx;
};
} // namespace

static int runReplayHands(const TestArgs& args)
{
	const char* tag= "replay-hands";
	if (args.empty())
	{
		MIKAN_LOG_ERROR(tag) << "usage: --replay-hands <recording.jsonl> [cameraIndex] [out.csv]";
		return 1;
	}

	const std::string recordingPath= args[0];
	int requestedCamera= -1;
	std::ofstream csv;
	for (size_t i= 1; i < args.size(); ++i)
	{
		if (args[i].size() > 4 && args[i].compare(args[i].size() - 4, 4, ".csv") == 0)
		{
			csv.open(args[i]);
			csv << "frameIndex,slotId,palmSidePx,wristX,wristY,middleMcpX,middleMcpY,meanErrorPx,maxErrorPx,"
				   "recordedPresence,presence,estimateMs";
			// The re-estimated landmarks, for overlays
			for (int landmark= 0; landmark < HAND_LANDMARK_COUNT; ++landmark)
				csv << ",x" << landmark << ",y" << landmark;
			csv << "\n";
		}
		else
		{
			requestedCamera= atoi(args[i].c_str());
		}
	}

	TrackingReplay replay;
	std::string error;
	if (!replay.load(recordingPath, error))
	{
		MIKAN_LOG_ERROR(tag) << "Load failed: " << error;
		return 1;
	}

	const std::string frameDirectory= TrackingRecording::makeFrameDirectoryPath(recordingPath);
	if (!std::filesystem::exists(frameDirectory))
	{
		MIKAN_LOG_ERROR(tag)
			<< "No frames beside this recording (" << frameDirectory
			<< "). Enable 'Also record raw camera frames' in the Timeline panel and record again.";
		return 1;
	}

	HandLandmarkModel model;
	if (!model.load("models/hand_landmark.onnx", replay.getRecordedConfig().tracking.onnxEp))
	{
		MIKAN_LOG_ERROR(tag) << "Failed to load models/hand_landmark.onnx";
		return 1;
	}
	MIKAN_LOG_INFO(tag) << "ep=" << model.activeEp() << ", frames from " << frameDirectory;

	// Previous fresh frame's hands per camera, the source of each ROI
	std::vector<std::array<RecordedHandInput, 2>> lastHands(replay.getRecordedConfig().cameraCount());
	std::vector<bool> bHaveLast(lastHands.size(), false);

	// The first runs carry the execution provider's warm-up
	constexpr int kWarmupEstimates= 10;
	int estimatesRun= 0;
	int framesMissing= 0;
	int handsInvalid= 0;
	std::vector<float> estimateMs;
	std::vector<float> meanErrorPx;
	std::vector<float> maxErrorPx;
	std::vector<float> relativeError;
	std::vector<float> presenceDelta;
	// Keyed by the ROI's palm box side in 50 px steps, the size the crop
	// cost and the crop resampling both depend on
	std::map<int, SizeBucket> buckets;

	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const RecordedFrame& recorded= *replay.getFrame(frameIndex).recorded;
		for (const RecordedCameraInput& fresh : recorded.freshCameras)
		{
			const int cameraIndex= fresh.cameraIndex;
			if (cameraIndex < 0 || cameraIndex >= (int)lastHands.size())
				continue;
			if (requestedCamera >= 0 && cameraIndex != requestedCamera)
				continue;

			if (bHaveLast[cameraIndex])
			{
				const cv::Mat image= cv::imread(
					FrameRecorder::makeFramePath(frameDirectory, cameraIndex, fresh.frameIndex), cv::IMREAD_COLOR);
				if (image.empty())
				{
					framesMissing++;
				}
				else
				{
					for (const RecordedHandInput& current : fresh.hands)
					{
						if (!current.tracked)
							continue;
						const RecordedHandInput* previous= nullptr;
						for (const RecordedHandInput& candidate : lastHands[cameraIndex])
						{
							if (candidate.tracked && candidate.slotId == current.slotId)
								previous= &candidate;
						}
						if (previous == nullptr)
							continue;

						const HandRoi roi= HandRoi::fromLandmarks(previous->imagePoints);
						HandLandmarkResult result;
						const auto start= std::chrono::steady_clock::now();
						model.estimate(image, roi, result);
						const float elapsedMs=
							(float)std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
								.count();
						estimatesRun++;
						if (!result.valid)
						{
							handsInvalid++;
							continue;
						}

						float errorSum= 0.f;
						float errorMax= 0.f;
						for (int i= 0; i < HAND_LANDMARK_COUNT; ++i)
						{
							const float error=
								glm::length(glm::vec2(result.imagePoints[i]) - glm::vec2(current.imagePoints[i]));
							errorSum+= error;
							errorMax= std::max(errorMax, error);
						}
						const float errorMean= errorSum / (float)HAND_LANDMARK_COUNT;
						const glm::vec2 palmExtent= roi.palmBoxMax - roi.palmBoxMin;
						const float palmSide= std::max(palmExtent.x, palmExtent.y);

						meanErrorPx.push_back(errorMean);
						maxErrorPx.push_back(errorMax);
						relativeError.push_back(palmSide > 0.f ? errorMean / palmSide : 0.f);
						presenceDelta.push_back(std::fabs(result.confidence - current.presence));
						if (csv.is_open())
						{
							csv << fresh.frameIndex << "," << current.slotId << "," << palmSide << ","
								<< roi.palmKeypoints[0].x << "," << roi.palmKeypoints[0].y << ","
								<< roi.palmKeypoints[2].x << "," << roi.palmKeypoints[2].y << "," << errorMean << ","
								<< errorMax << "," << current.presence << "," << result.confidence << "," << elapsedMs;
							for (const glm::vec3& point : result.imagePoints)
								csv << "," << point.x << "," << point.y;
							csv << "\n";
						}

						SizeBucket& bucket= buckets[(int)(palmSide / 50.f) * 50];
						bucket.meanErrorPx.push_back(errorMean);
						if (estimatesRun > kWarmupEstimates)
						{
							estimateMs.push_back(elapsedMs);
							bucket.estimateMs.push_back(elapsedMs);
						}
					}
				}
			}

			lastHands[cameraIndex]= fresh.hands;
			bHaveLast[cameraIndex]= true;
		}
	}

	if (meanErrorPx.empty())
	{
		MIKAN_LOG_ERROR(tag) << "No tracked hand had a previous-frame ROI and a stored frame (missing frames "
							 << framesMissing << ")";
		return 1;
	}

	MIKAN_LOG_INFO(tag) << "hands estimated " << estimatesRun << " (invalid crop " << handsInvalid
						<< "), missing frames " << framesMissing;
	MIKAN_LOG_INFO(tag) << "estimate ms: p50 " << percentile(estimateMs, 0.5f) << " p90 "
						<< percentile(estimateMs, 0.9f) << " max " << percentile(estimateMs, 1.f);
	MIKAN_LOG_INFO(tag) << "mean landmark error vs recorded px: p50 " << percentile(meanErrorPx, 0.5f) << " p90 "
						<< percentile(meanErrorPx, 0.9f) << " max " << percentile(meanErrorPx, 1.f);
	MIKAN_LOG_INFO(tag) << "worst landmark error vs recorded px: p50 " << percentile(maxErrorPx, 0.5f) << " p90 "
						<< percentile(maxErrorPx, 0.9f);
	MIKAN_LOG_INFO(tag) << "mean error / palm box side: p50 " << percentile(relativeError, 0.5f) << " p90 "
						<< percentile(relativeError, 0.9f);
	MIKAN_LOG_INFO(tag) << "presence delta vs recorded: p50 " << percentile(presenceDelta, 0.5f) << " p90 "
						<< percentile(presenceDelta, 0.9f);
	MIKAN_LOG_INFO(tag) << "by palm box side:";
	for (const auto& [side, bucket] : buckets)
	{
		MIKAN_LOG_INFO(tag) << "  " << side << "-" << side + 49 << " px: hands " << bucket.meanErrorPx.size()
							<< ", estimate ms p50 " << percentile(bucket.estimateMs, 0.5f) << ", mean error px p50 "
							<< percentile(bucket.meanErrorPx, 0.5f);
	}

	return 0;
}

MIKAN_REGISTER_TEST("--replay-hands", "Re-run the hand landmark model over a recording's raw frames",
					eTestCategory::Tool, runReplayHands);
