#include "TestCommon.h"

#include <iomanip>
#include <sstream>

#include "MfccBackend.h"
#include "MouthFilter.h"
#include "MouthWindow.h"
#include "PathUtils.h"
#include "WavFile.h"

// The audio stage's bench: runs the backend over a wav clip under the live
// rolling-window policy, writes the emitted 30 fps viseme weights as JSON for
// playback on the avatar, and measures the two numbers that gate the live
// stage: how far the chunked tail frames drift from one whole-clip run, and
// the per-run inference time against the hop.
//
//   --viseme-bench <clip.wav> [out.json] [--hop ms] [--window ms] [--lookahead frames]
//                  [--profile path] [--cutoff hz] [--beta b]

namespace
{
struct BenchOptions
{
	std::filesystem::path clipPath;
	std::filesystem::path outPath;
	int hopMs= 100;
	int windowMs= 200;
	int lookaheadFrames= 0;
	// Empty for the volume-only stage
	std::string profilePath;
	MouthFilterConfig filter;
};

bool parseOptions(const TestArgs& args, BenchOptions& out)
{
	for (size_t index= 0; index < args.size(); ++index)
	{
		const std::string& arg= args[index];
		const bool bHasValue= index + 1 < args.size();
		if (arg == "--hop" && bHasValue)
			out.hopMs= std::stoi(args[++index]);
		else if (arg == "--window" && bHasValue)
			out.windowMs= std::stoi(args[++index]);
		else if (arg == "--lookahead" && bHasValue)
			out.lookaheadFrames= std::stoi(args[++index]);
		else if (arg == "--profile" && bHasValue)
			out.profilePath= args[++index];
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
			<< "Usage: --viseme-bench <clip.wav> [out.json] [--hop ms] [--window ms] [--lookahead frames] "
			   "[--profile path] [--cutoff hz] [--beta b]";
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

	MfccBackend backend;
	if (!backend.create(options.profilePath) && !options.profilePath.empty())
	{
		MIKAN_LOG_ERROR("viseme-bench") << backend.getProfileError();
		return 1;
	}
	const std::string profileLabel= options.profilePath.empty() ? std::string("no profile, volume only") : options.profilePath;
	MIKAN_LOG_INFO("viseme-bench") << profileLabel << ", hop " << options.hopMs << " ms, window " << options.windowMs
								   << " ms, lookahead " << options.lookaheadFrames << " frames";

	// One run over the whole clip: the reference the chunked tail frames are
	// measured against
	std::vector<MouthFrame> offlineFrames;
	double offlineMs= 0.0;
	if (backend.infer(clip.data(), clip.size(), offlineFrames))
	{
		offlineMs= backend.getLastInferMs();
		MIKAN_LOG_INFO("viseme-bench") << "whole clip: " << offlineFrames.size() << " frames in " << offlineMs << " ms";

		// How the clip's frames split across the dominant visemes, the
		// readout that says whether the profile is calling anything but aa
		std::array<int, VISEME_COUNT> counts{};
		for (const MouthFrame& frame : offlineFrames)
			counts[dominantViseme(frame.weights)]++;
		std::ostringstream shares;
		for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		{
			if (counts[viseme] > 0)
				shares << " " << visemeName(viseme) << " " << std::fixed << std::setprecision(2)
					   << (double)counts[viseme] / (double)offlineFrames.size();
		}
		MIKAN_LOG_INFO("viseme-bench") << "viseme share of frames:" << shares.str();
	}
	else
	{
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
			if (!backend.infer(windowSamples.data(), windowSamples.size(), windowFrames))
			{
				MIKAN_LOG_ERROR("viseme-bench") << "chunked run failed at sample " << window.getConsumedSamples();
				return 1;
			}
			runMs.push_back(backend.getLastInferMs());

			tailFrames.clear();
			MouthWindow::takeTail(windowFrames, emitCount, windowConfig.lookaheadFrames, tailFrames);
			for (const MouthFrame& frame : tailFrames)
			{
				chunkedFrames.push_back(frame);
				filteredFrames.push_back(filter.filter(frame));
			}
		}
	}

	// Drift of the chunked frames from the whole-clip run, per viseme and
	// overall, beside how much each viseme moves at all
	const size_t compareCount= std::min(chunkedFrames.size(), offlineFrames.size());
	std::array<double, VISEME_COUNT> visemeDiff{};
	std::array<double, VISEME_COUNT> visemeActivity{};
	for (size_t frameIndex= 0; frameIndex < compareCount; ++frameIndex)
	{
		for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		{
			visemeDiff[viseme]+= std::fabs(chunkedFrames[frameIndex].weights[viseme] - offlineFrames[frameIndex].weights[viseme]);
			visemeActivity[viseme]+= std::fabs(offlineFrames[frameIndex].weights[viseme]);
		}
	}
	double meanDiff= 0.0;
	double meanActivity= 0.0;
	int worstViseme= 0;
	for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
	{
		if (compareCount > 0)
		{
			visemeDiff[viseme]/= (double)compareCount;
			visemeActivity[viseme]/= (double)compareCount;
		}
		meanDiff+= visemeDiff[viseme];
		meanActivity+= visemeActivity[viseme];
		if (visemeDiff[viseme] > visemeDiff[worstViseme])
			worstViseme= viseme;
	}
	meanDiff/= VISEME_COUNT;
	meanActivity/= VISEME_COUNT;

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
	{
		MIKAN_LOG_INFO("viseme-bench") << "chunked vs whole-clip mean abs diff " << meanDiff << " over mean activity "
									   << meanActivity << "; worst viseme " << visemeName(worstViseme) << " at "
									   << visemeDiff[worstViseme] << " (activity " << visemeActivity[worstViseme] << ")";
		std::ostringstream activity;
		for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		{
			if (visemeActivity[viseme] > 0.01)
				activity << " " << visemeName(viseme) << " " << std::fixed << std::setprecision(3) << visemeActivity[viseme];
		}
		MIKAN_LOG_INFO("viseme-bench") << "mean activity per viseme:" << activity.str();
	}
	MIKAN_LOG_INFO("viseme-bench") << (p95 < (double)options.hopMs ? "LATENCY PASS" : "LATENCY FAIL")
								   << ": p95 " << p95 << " ms against the " << options.hopMs << " ms hop; emitted frames trail the audio by about "
								   << options.hopMs + options.lookaheadFrames * 1000 / MOUTH_FRAME_RATE + (int)p50 << " ms";

	if (!options.outPath.empty())
	{
		nlohmann::json visemes= nlohmann::json::array();
		nlohmann::json perVisemeDiff= nlohmann::json::object();
		nlohmann::json perVisemeActivity= nlohmann::json::object();
		for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		{
			visemes.push_back(visemeName(viseme));
			perVisemeDiff[visemeName(viseme)]= visemeDiff[viseme];
			perVisemeActivity[visemeName(viseme)]= visemeActivity[viseme];
		}

		nlohmann::json out;
		out["clip"]= PathUtils::pathToUtf8(options.clipPath);
		out["clipSeconds"]= clipSeconds;
		out["profile"]= options.profilePath;
		out["sampleRate"]= AUDIO_SAMPLE_RATE;
		out["frameRate"]= MOUTH_FRAME_RATE;
		out["hopMs"]= options.hopMs;
		out["windowMs"]= options.windowMs;
		out["lookaheadFrames"]= options.lookaheadFrames;
		out["filter"]= {{"minCutoffHz", options.filter.minCutoffHz},
						{"beta", options.filter.beta},
						{"derivativeCutoffHz", options.filter.derivativeCutoffHz}};
		out["visemes"]= visemes;
		out["timings"]= {{"runs", runMs.size()},
						 {"p50Ms", p50},
						 {"p95Ms", p95},
						 {"maxMs", maxMs},
						 {"totalMs", totalRunMs},
						 {"shareOfRealTime", cpuShare},
						 {"wholeClipMs", offlineMs}};
		out["diff"]= {{"mean", meanDiff},
					  {"meanActivity", meanActivity},
					  {"worstViseme", visemeName(worstViseme)},
					  {"perViseme", perVisemeDiff},
					  {"perVisemeActivity", perVisemeActivity}};
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
					"Run the mouth backend over a wav clip under the live window policy: drift vs a whole-clip run, "
					"inference ms, viseme weights as JSON",
					eTestCategory::Tool, runVisemeBench)
