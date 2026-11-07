#include "TestCommon.h"

#include <map>

#include "IMouthBackend.h"
#include "MouthReplay.h"
#include "PathUtils.h"
#include "WavFile.h"

// Re-runs the microphone mouth stage over a recording's audio sidecar and
// measures the result against the mouth frames the live run recorded, so a
// backend change (or another backend, or other gain and gate settings) can be
// judged against old recordings. The sidecar is aligned to the live hop grid,
// so the same backend under the same settings replays to the live frames
// within the chunked drift once the window has filled.
//
//   --replay-audio <recording.jsonl> [out.json] [--backend wav2arkit|mfcc] [--model path] [--gain db] [--gate dbfs]

namespace
{
struct ReplayAudioOptions
{
	std::filesystem::path recordingPath;
	std::filesystem::path outPath;
	std::string backendName;
	std::string modelPath;
	bool bGainSet= false;
	float gainDb= 0.f;
	bool bGateSet= false;
	float gateDbfs= 0.f;
};

bool parseOptions(const TestArgs& args, ReplayAudioOptions& options)
{
	int positional= 0;
	for (size_t index= 0; index < args.size(); ++index)
	{
		const std::string& arg= args[index];
		const bool bHasValue= index + 1 < args.size();
		if (arg == "--backend" && bHasValue)
			options.backendName= args[++index];
		else if (arg == "--model" && bHasValue)
			options.modelPath= args[++index];
		else if (arg == "--gain" && bHasValue)
		{
			options.gainDb= std::stof(args[++index]);
			options.bGainSet= true;
		}
		else if (arg == "--gate" && bHasValue)
		{
			options.gateDbfs= std::stof(args[++index]);
			options.bGateSet= true;
		}
		else if (arg.rfind("--", 0) == 0)
			return false;
		else if (positional == 0)
			options.recordingPath= PathUtils::utf8ToPath(arg);
		else if (positional == 1)
			options.outPath= PathUtils::utf8ToPath(arg);
		else
			return false;
		if (arg.rfind("--", 0) != 0)
			++positional;
	}
	return !options.recordingPath.empty();
}

nlohmann::json framesToJson(const std::vector<MouthFrame>& frames)
{
	nlohmann::json rows= nlohmann::json::array();
	for (const MouthFrame& frame : frames)
	{
		nlohmann::json row= nlohmann::json::array();
		for (float weight : frame.weights)
			row.push_back(std::round(weight * 10000.f) / 10000.f);
		rows.push_back(std::move(row));
	}
	return rows;
}
} // namespace

static int runReplayAudio(const TestArgs& args)
{
	ReplayAudioOptions options;
	if (!parseOptions(args, options))
	{
		MIKAN_LOG_ERROR("replay-audio")
			<< "usage: --replay-audio <recording.jsonl> [out.json] [--backend wav2arkit|mfcc] [--model path] [--gain db] "
			   "[--gate dbfs]";
		return 1;
	}

	TrackingReplay replay;
	std::string error;
	if (!replay.load(PathUtils::pathToUtf8(options.recordingPath), error))
	{
		MIKAN_LOG_ERROR("replay-audio") << "Load failed: " << error;
		return 1;
	}
	const RecordedAudioSidecar& sidecar= replay.getHeader().audio;
	if (!sidecar.present)
	{
		MIKAN_LOG_ERROR("replay-audio") << "the recording has no audio sidecar (the mouth stage was off when it was made)";
		return 1;
	}

	const std::filesystem::path wavPath= options.recordingPath.parent_path() / PathUtils::utf8ToPath(sidecar.fileName);
	std::vector<float> clip;
	if (!WavFile::readMono16k(wavPath, clip, error))
	{
		MIKAN_LOG_ERROR("replay-audio") << "sidecar: " << error;
		return 1;
	}
	MIKAN_LOG_INFO("replay-audio") << wavPath.string() << ": " << clip.size() << " samples ("
								   << (double)clip.size() / AUDIO_SAMPLE_RATE << " s), starting at stream sample "
								   << sidecar.startSampleIndex << ", recorded with " << sidecar.backend;

	// The chain under the recorded settings unless overridden
	const AudioConfig& recordedAudio= replay.getRecordedConfig().audio;
	MouthChainConfig chainConfig;
	chainConfig.gainDb= options.bGainSet ? options.gainDb : recordedAudio.gainDb;
	chainConfig.gateDbfs= options.bGateSet ? options.gateDbfs : recordedAudio.gateDbfs;
	const std::string backendName= options.backendName.empty() ? sidecar.backend : options.backendName;
	if (backendName != "wav2arkit" && backendName != "mfcc")
	{
		MIKAN_LOG_ERROR("replay-audio") << "unknown backend " << backendName << " (wav2arkit or mfcc)";
		return 1;
	}
	std::unique_ptr<IMouthBackend> backend= createMouthBackend(backendName, options.modelPath);
	if (!backend->isReady())
	{
		MIKAN_LOG_ERROR("replay-audio") << backend->getUnavailableReason();
		return 1;
	}

	const MouthReplayAlignment alignment= mouthReplayAlignment(sidecar.startSampleIndex, chainConfig.window);
	MouthReplayOutput replayed;
	replayMouthChain(clip, alignment.padSamples, chainConfig, *backend, replayed);
	MIKAN_LOG_INFO("replay-audio") << backend->getName() << " at gain " << chainConfig.gainDb << " dB, gate "
								   << chainConfig.gateDbfs << " dBFS: " << replayed.frames.size() << " frames from "
								   << replayed.runs.size() << " runs, padded " << alignment.padSamples
								   << " samples onto the live hop grid, live frame = replayed frame + "
								   << alignment.frameOffset;

	// The live frames by their stream index: consecutive iterations share a
	// mouth sample, so each distinct frame counts once
	std::map<int64_t, MouthFrame> recordedByIndex;
	int recordedFrames= 0;
	int untiedFrames= 0;
	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const RecordedMouth& mouth= replay.getFrame(frameIndex).recorded->mouth;
		if (!mouth.present)
			continue;
		++recordedFrames;
		if (mouth.audioFrameIndex < 0)
			++untiedFrames;
		else
			recordedByIndex[mouth.audioFrameIndex]= mouth.frame;
	}

	// The first window's frames replay from padding where live had the
	// stream's history, so they are measured apart
	const int64_t warmupFrames= (int64_t)chainConfig.window.windowSamples * MOUTH_FRAME_RATE / AUDIO_SAMPLE_RATE;
	std::array<double, MOUTH_COLUMN_COUNT> columnDiff{};
	std::array<double, MOUTH_COLUMN_COUNT> columnActivity{};
	double warmupDiff= 0.0;
	int compared= 0;
	int warmupCompared= 0;
	int outside= 0;
	std::vector<MouthFrame> recordedSeries;
	std::vector<MouthFrame> replayedSeries;
	for (const auto& [liveIndex, recordedFrame] : recordedByIndex)
	{
		const int64_t replayIndex= liveIndex - alignment.frameOffset;
		if (replayIndex < 0 || replayIndex >= (int64_t)replayed.frames.size())
		{
			++outside;
			continue;
		}
		const MouthFrame& replayedFrame= replayed.frames[(size_t)replayIndex].frame;
		recordedSeries.push_back(recordedFrame);
		replayedSeries.push_back(replayedFrame);
		if (replayIndex < warmupFrames)
		{
			for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
				warmupDiff+= std::fabs(replayedFrame.weights[column] - recordedFrame.weights[column]);
			++warmupCompared;
			continue;
		}
		for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
		{
			columnDiff[column]+= std::fabs(replayedFrame.weights[column] - recordedFrame.weights[column]);
			columnActivity[column]+= std::fabs(recordedFrame.weights[column]);
		}
		++compared;
	}

	double meanDiff= 0.0;
	double meanActivity= 0.0;
	int worstColumn= 0;
	for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
	{
		columnDiff[column]/= std::max(compared, 1);
		columnActivity[column]/= std::max(compared, 1);
		meanDiff+= columnDiff[column];
		meanActivity+= columnActivity[column];
		if (columnDiff[column] > columnDiff[worstColumn])
			worstColumn= column;
	}
	meanDiff/= MOUTH_COLUMN_COUNT;
	meanActivity/= MOUTH_COLUMN_COUNT;
	warmupDiff/= std::max(warmupCompared, 1) * MOUTH_COLUMN_COUNT;

	std::vector<double> runMs;
	for (const MouthChainRun& run : replayed.runs)
	{
		if (run.bInferred)
			runMs.push_back(run.inferMs);
	}
	std::sort(runMs.begin(), runMs.end());
	const double p50= runMs.empty() ? 0.0 : runMs[runMs.size() / 2];
	const double p95= runMs.empty() ? 0.0 : runMs[std::min(runMs.size() - 1, (size_t)(0.95 * (double)runMs.size()))];

	MIKAN_LOG_INFO("replay-audio") << recordedFrames << " recorded iterations carried a mouth, " << recordedByIndex.size()
								   << " distinct frames tied to the sidecar, " << untiedFrames << " untied, " << outside
								   << " outside the replayed span";
	MIKAN_LOG_INFO("replay-audio") << "warm-up (the first " << warmupFrames << " frames): " << warmupCompared
								   << " compared, mean abs diff " << warmupDiff;
	MIKAN_LOG_INFO("replay-audio") << "AUDIO REPLAY: " << compared << " frames compared, mean abs diff " << meanDiff
								   << " over mean activity " << meanActivity << ", worst column "
								   << mouthColumnName(worstColumn) << " " << columnDiff[worstColumn] << ", run p50 " << p50
								   << " ms p95 " << p95 << " ms";
	for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
	{
		if (columnDiff[column] > 0.01 || columnActivity[column] > 0.05)
			MIKAN_LOG_INFO("replay-audio") << "  " << mouthColumnName(column) << ": diff " << columnDiff[column]
										   << " activity " << columnActivity[column];
	}

	if (!options.outPath.empty())
	{
		nlohmann::json columns= nlohmann::json::array();
		for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
			columns.push_back(mouthColumnName(column));
		nlohmann::json perColumnDiff= nlohmann::json::object();
		nlohmann::json perColumnActivity= nlohmann::json::object();
		for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
		{
			perColumnDiff[mouthColumnName(column)]= columnDiff[column];
			perColumnActivity[mouthColumnName(column)]= columnActivity[column];
		}
		nlohmann::json out;
		out["recording"]= PathUtils::pathToUtf8(options.recordingPath);
		out["sidecar"]= sidecar.fileName;
		out["recordedBackend"]= sidecar.backend;
		out["backend"]= backend->getName();
		out["gainDb"]= chainConfig.gainDb;
		out["gateDbfs"]= chainConfig.gateDbfs;
		out["sampleRate"]= AUDIO_SAMPLE_RATE;
		out["frameRate"]= MOUTH_FRAME_RATE;
		out["alignment"]= {{"startSample", sidecar.startSampleIndex},
						   {"padSamples", alignment.padSamples},
						   {"frameOffset", alignment.frameOffset},
						   {"warmupFrames", warmupFrames}};
		out["columns"]= columns;
		out["timings"]= {{"runs", runMs.size()}, {"p50Ms", p50}, {"p95Ms", p95}};
		out["diff"]= {{"compared", compared},
					  {"mean", meanDiff},
					  {"meanActivity", meanActivity},
					  {"worstColumn", mouthColumnName(worstColumn)},
					  {"warmupCompared", warmupCompared},
					  {"warmupMean", warmupDiff},
					  {"perColumn", perColumnDiff},
					  {"perColumnActivity", perColumnActivity}};
		out["recorded"]= framesToJson(recordedSeries);
		out["replayed"]= framesToJson(replayedSeries);

		std::ofstream file(options.outPath);
		if (!file)
		{
			MIKAN_LOG_ERROR("replay-audio") << "cannot write " << options.outPath.string();
			return 1;
		}
		file << out.dump();
		MIKAN_LOG_INFO("replay-audio") << "wrote " << options.outPath.string();
	}

	return compared > 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--replay-audio",
					"Re-runs the mouth stage over a recording's audio sidecar and measures it against the recorded mouth frames",
					eTestCategory::Tool, runReplayAudio)
