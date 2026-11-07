#include "TestCommon.h"

#include <iomanip>
#include <memory>
#include <sstream>

#include "IMouthBackend.h"
#include "MfccBackend.h"
#include "MouthFilter.h"
#include "MouthWindow.h"
#include "PathUtils.h"
#include "Wav2ArkitBackend.h"
#include "WavFile.h"

// The audio stage's bench: runs a backend over a wav clip under the live
// rolling-window policy, writes the emitted 30 fps mouth weights as JSON for
// playback on the avatar, and measures the two numbers that gate the live
// stage: how far the chunked tail frames drift from one whole-clip run, and
// the per-run inference time against the hop.
//
//   --viseme-bench <clip.wav> [out.json] [--backend wav2arkit|mfcc] [--hop ms] [--window ms]
//                  [--lookahead frames] [--model path] [--ep cpu|directml] [--threads n]
//                  [--cutoff hz] [--beta b]

namespace
{
struct BenchOptions
{
	std::filesystem::path clipPath;
	std::filesystem::path outPath;
	int hopMs= 100;
	int windowMs= 1000;
	int lookaheadFrames= 2;
	std::string backendName= "wav2arkit";
	// Empty for the backend's default
	std::string modelPath;
	std::string ep= "cpu";
	int threads= 4;
	MouthFilterConfig filter;
};

bool parseOptions(const TestArgs& args, BenchOptions& out)
{
	for (size_t index= 0; index < args.size(); ++index)
	{
		const std::string& arg= args[index];
		const bool bHasValue= index + 1 < args.size();
		if (arg == "--backend" && bHasValue)
			out.backendName= args[++index];
		else if (arg == "--hop" && bHasValue)
			out.hopMs= std::stoi(args[++index]);
		else if (arg == "--window" && bHasValue)
			out.windowMs= std::stoi(args[++index]);
		else if (arg == "--lookahead" && bHasValue)
			out.lookaheadFrames= std::stoi(args[++index]);
		else if (arg == "--model" && bHasValue)
			out.modelPath= args[++index];
		else if (arg == "--ep" && bHasValue)
			out.ep= args[++index];
		else if (arg == "--threads" && bHasValue)
			out.threads= std::stoi(args[++index]);
		else if (arg == "--cutoff" && bHasValue)
			out.filter.minCutoffHz= std::stof(args[++index]);
		else if (arg == "--beta" && bHasValue)
			out.filter.beta= std::stof(args[++index]);
		else if (arg.rfind("--", 0) == 0)
			return false;
		else if (out.clipPath.empty())
			out.clipPath= PathUtils::utf8ToPath(arg);
		else if (out.outPath.empty())
			out.outPath= PathUtils::utf8ToPath(arg);
		else
			return false;
	}
	return !out.clipPath.empty();
}

double percentile(std::vector<double> values, double fraction)
{
	if (values.empty())
		return 0.0;
	std::sort(values.begin(), values.end());
	const size_t index= std::min(values.size() - 1, (size_t)(fraction * (double)(values.size() - 1) + 0.5));
	return values[index];
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

static int runVisemeBench(const TestArgs& args)
{
	BenchOptions options;
	if (!parseOptions(args, options))
	{
		MIKAN_LOG_ERROR("viseme-bench")
			<< "Usage: --viseme-bench <clip.wav> [out.json] [--backend wav2arkit|mfcc] [--hop ms] [--window ms] "
			   "[--lookahead frames] [--model path] [--ep cpu|directml] [--threads n] [--cutoff hz] [--beta b]";
		return 1;
	}

	std::vector<float> clip;
	std::string error;
	if (!WavFile::readMono16k(options.clipPath, clip, error))
	{
		MIKAN_LOG_ERROR("viseme-bench") << "clip: " << error;
		return 1;
	}
	const double clipSeconds= (double)clip.size() / AUDIO_SAMPLE_RATE;
	MIKAN_LOG_INFO("viseme-bench") << options.clipPath.string() << ": " << clip.size() << " samples at 16 kHz, "
								   << clipSeconds << " s";

	std::unique_ptr<IMouthBackend> backend;
	std::string provider= "CPU";
	if (options.backendName == "wav2arkit")
	{
		auto wav2arkit= std::make_unique<Wav2ArkitBackend>();
		const std::string modelPath=
			options.modelPath.empty() ? std::string(Wav2ArkitBackend::k_defaultModelPath) : options.modelPath;
		if (!wav2arkit->create(modelPath, options.ep, options.threads))
		{
			MIKAN_LOG_ERROR("viseme-bench") << wav2arkit->getUnavailableReason();
			return 1;
		}
		provider= wav2arkit->getActiveEp();
		backend= std::move(wav2arkit);
	}
	else if (options.backendName == "mfcc")
	{
		auto mfcc= std::make_unique<MfccBackend>();
		const std::string modelPath=
			options.modelPath.empty() ? std::string(MfccBackend::k_defaultModelPath) : options.modelPath;
		if (!mfcc->create(modelPath))
		{
			MIKAN_LOG_ERROR("viseme-bench") << mfcc->getUnavailableReason();
			return 1;
		}
		backend= std::move(mfcc);
	}
	else
	{
		MIKAN_LOG_ERROR("viseme-bench") << "unknown backend " << options.backendName << " (wav2arkit or mfcc)";
		return 1;
	}
	MIKAN_LOG_INFO("viseme-bench") << backend->getName() << " on " << provider << ", " << options.threads
								   << " threads, hop " << options.hopMs << " ms, window " << options.windowMs << " ms, lookahead "
								   << options.lookaheadFrames << " frames";

	// One run over the whole clip: the reference the chunked tail frames are
	// measured against
	std::vector<MouthFrame> offlineFrames;
	double offlineMs= 0.0;
	if (backend->infer(clip.data(), clip.size(), offlineFrames))
	{
		offlineMs= backend->getLastInferMs();
		MIKAN_LOG_INFO("viseme-bench") << "whole clip: " << offlineFrames.size() << " frames in " << offlineMs << " ms";
		if (const MfccBackend* mfcc= dynamic_cast<const MfccBackend*>(backend.get()))
		{
			// How the clip's hops split across the visemes, the readout that
			// says whether the classifier is calling anything but silence
			std::array<int, VISEME_COUNT> counts{};
			for (uint8_t viseme : mfcc->getLastHopVisemes())
				counts[viseme]++;
			std::ostringstream shares;
			for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
			{
				if (counts[viseme] > 0)
					shares << " " << visemeName(viseme) << " " << std::fixed << std::setprecision(2)
						   << (double)counts[viseme] / (double)mfcc->getLastHopVisemes().size();
			}
			MIKAN_LOG_INFO("viseme-bench") << "viseme share of hops:" << shares.str();
		}
	}
	else
	{
		// A fixed-shape export cannot take the whole clip; the timing still
		// measures, the drift does not
		MIKAN_LOG_WARNING("viseme-bench") << "whole-clip run failed - drift against it is not measured";
	}

	// The live policy: the clip arrives in hops, each run sees the window
	// ending at the newest hop and emits its tail
	MouthWindowConfig windowConfig;
	windowConfig.hopSamples= options.hopMs * AUDIO_SAMPLE_RATE / 1000;
	windowConfig.windowSamples= options.windowMs * AUDIO_SAMPLE_RATE / 1000;
	windowConfig.lookaheadFrames= options.lookaheadFrames;
	MouthWindow window;
	window.configure(windowConfig);
	MouthFilter filter;
	filter.configure(options.filter);

	std::vector<MouthFrame> chunkedFrames;
	std::vector<MouthFrame> filteredFrames;
	std::vector<double> runMs;
	std::vector<float> windowSamples;
	std::vector<MouthFrame> windowFrames;
	std::vector<MouthFrame> tailFrames;
	size_t offset= 0;
	while (offset < clip.size())
	{
		const size_t count= std::min((size_t)windowConfig.hopSamples, clip.size() - offset);
		window.push(clip.data() + offset, count);
		offset+= count;

		while (window.isRunDue())
		{
			const int emitCount= window.beginRun(windowSamples);
			if (!backend->infer(windowSamples.data(), windowSamples.size(), windowFrames))
			{
				MIKAN_LOG_ERROR("viseme-bench") << "chunked run failed at sample " << window.getConsumedSamples();
				return 1;
			}
			runMs.push_back(backend->getLastInferMs());

			tailFrames.clear();
			MouthWindow::takeTail(windowFrames, emitCount, windowConfig.lookaheadFrames, tailFrames);
			for (const MouthFrame& frame : tailFrames)
			{
				chunkedFrames.push_back(frame);
				filteredFrames.push_back(filter.filter(frame));
			}
		}
	}

	// Drift of the chunked frames from the whole-clip run, per column and
	// overall, beside how much each column moves at all
	const size_t compareCount= std::min(chunkedFrames.size(), offlineFrames.size());
	std::array<double, MOUTH_COLUMN_COUNT> columnDiff{};
	std::array<double, MOUTH_COLUMN_COUNT> columnActivity{};
	for (size_t frameIndex= 0; frameIndex < compareCount; ++frameIndex)
	{
		for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
		{
			columnDiff[column]+= std::fabs(chunkedFrames[frameIndex].weights[column] - offlineFrames[frameIndex].weights[column]);
			columnActivity[column]+= std::fabs(offlineFrames[frameIndex].weights[column]);
		}
	}
	double meanDiff= 0.0;
	double meanActivity= 0.0;
	int worstColumn= 0;
	for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
	{
		if (compareCount > 0)
		{
			columnDiff[column]/= (double)compareCount;
			columnActivity[column]/= (double)compareCount;
		}
		meanDiff+= columnDiff[column];
		meanActivity+= columnActivity[column];
		if (columnDiff[column] > columnDiff[worstColumn])
			worstColumn= column;
	}
	meanDiff/= MOUTH_COLUMN_COUNT;
	meanActivity/= MOUTH_COLUMN_COUNT;

	double totalRunMs= 0.0;
	for (double ms : runMs)
		totalRunMs+= ms;
	const double p50= percentile(runMs, 0.5);
	const double p95= percentile(runMs, 0.95);
	const double maxMs= runMs.empty() ? 0.0 : *std::max_element(runMs.begin(), runMs.end());
	const double cpuShare= clipSeconds > 0.0 ? totalRunMs / (clipSeconds * 1000.0) : 0.0;

	MIKAN_LOG_INFO("viseme-bench") << "chunked: " << chunkedFrames.size() << " frames from " << runMs.size()
								   << " runs (offline " << offlineFrames.size() << ")";
	MIKAN_LOG_INFO("viseme-bench") << "inference ms per run: p50 " << p50 << ", p95 " << p95 << ", max " << maxMs
								   << ", share of real time " << cpuShare;
	if (!offlineFrames.empty())
	MIKAN_LOG_INFO("viseme-bench") << "chunked vs whole-clip mean abs diff " << meanDiff << " over mean activity "
								   << meanActivity << "; worst column " << mouthColumnName(worstColumn) << " at "
								   << columnDiff[worstColumn] << " (activity " << columnActivity[worstColumn] << ")";
	const int jawOpen= mouthColumnFromArkitIndex(arkitBlendshapeFromName("jawOpen"));
	MIKAN_LOG_INFO("viseme-bench") << "jawOpen: diff " << columnDiff[jawOpen] << ", activity " << columnActivity[jawOpen];
	MIKAN_LOG_INFO("viseme-bench") << (p95 < (double)options.hopMs ? "LATENCY PASS" : "LATENCY FAIL")
								   << ": p95 " << p95 << " ms against the " << options.hopMs << " ms hop; emitted frames trail the audio by about "
								   << options.hopMs + options.lookaheadFrames * 1000 / MOUTH_FRAME_RATE + (int)p50 << " ms";

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
		out["clip"]= PathUtils::pathToUtf8(options.clipPath);
		out["clipSeconds"]= clipSeconds;
		out["backend"]= backend->getName();
		out["executionProvider"]= provider;
		out["threads"]= options.threads;
		out["sampleRate"]= AUDIO_SAMPLE_RATE;
		out["frameRate"]= MOUTH_FRAME_RATE;
		out["hopMs"]= options.hopMs;
		out["windowMs"]= options.windowMs;
		out["lookaheadFrames"]= options.lookaheadFrames;
		out["filter"]= {{"minCutoffHz", options.filter.minCutoffHz},
						{"beta", options.filter.beta},
						{"derivativeCutoffHz", options.filter.derivativeCutoffHz}};
		out["columns"]= columns;
		out["timings"]= {{"runs", runMs.size()},
						 {"p50Ms", p50},
						 {"p95Ms", p95},
						 {"maxMs", maxMs},
						 {"totalMs", totalRunMs},
						 {"shareOfRealTime", cpuShare},
						 {"wholeClipMs", offlineMs}};
		out["diff"]= {{"mean", meanDiff},
					  {"meanActivity", meanActivity},
					  {"worstColumn", mouthColumnName(worstColumn)},
					  {"perColumn", perColumnDiff},
					  {"perColumnActivity", perColumnActivity}};
		out["offline"]= framesToJson(offlineFrames);
		out["chunked"]= framesToJson(chunkedFrames);
		out["filtered"]= framesToJson(filteredFrames);

		std::ofstream file(options.outPath);
		if (!file)
		{
			MIKAN_LOG_ERROR("viseme-bench") << "cannot write " << options.outPath.string();
			return 1;
		}
		file << out.dump();
		MIKAN_LOG_INFO("viseme-bench") << "wrote " << options.outPath.string();
	}

	return 0;
}

MIKAN_REGISTER_TEST("--viseme-bench",
					"Run a mouth backend over a wav clip under the live window policy: drift vs a whole-clip run, "
					"inference ms, mouth weights as JSON",
					eTestCategory::Tool, runVisemeBench)
