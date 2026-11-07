#include "MouthReplay.h"

#include <algorithm>

#include "MfccBackend.h"

namespace
{
// The capture callback's block, the granularity the live chain is fed at
constexpr size_t k_replayBlockSamples= 256;
} // namespace

MouthReplayAlignment mouthReplayAlignment(int64_t startSampleIndex, const MouthWindowConfig& window)
{
	MouthReplayAlignment alignment;
	const int64_t hop= std::max<int64_t>(window.hopSamples, 1);
	alignment.padSamples= startSampleIndex % hop;
	// The hop boundary at or before the start, and the frames due there
	const int64_t boundary= startSampleIndex - alignment.padSamples;
	alignment.frameOffset= boundary * MOUTH_FRAME_RATE / AUDIO_SAMPLE_RATE;
	return alignment;
}

void replayMouthChain(const std::vector<float>& clip, int64_t padSamples, const MouthChainConfig& config,
					  MfccBackend& backend, MouthReplayOutput& outReplay)
{
	outReplay= MouthReplayOutput();

	MouthChain chain;
	chain.configure(config);

	const std::vector<float> silence(k_replayBlockSamples, 0.f);
	int64_t padLeft= padSamples;
	while (padLeft > 0)
	{
		const size_t count= std::min((size_t)padLeft, k_replayBlockSamples);
		chain.push(silence.data(), count, &backend, outReplay.frames, outReplay.runs);
		padLeft-= (int64_t)count;
	}
	for (size_t offset= 0; offset < clip.size(); offset+= k_replayBlockSamples)
	{
		const size_t count= std::min(k_replayBlockSamples, clip.size() - offset);
		chain.push(clip.data() + offset, count, &backend, outReplay.frames, outReplay.runs);
	}
}
