#pragma once

#include <cstdint>
#include <vector>

#include "MouthChain.h"

class IMouthBackend;

// Where a recorded stretch of the stream sits on the live chain's grid. The
// live chain runs at every hop boundary and counts frames from the device's
// first sample, while a sidecar starts at startSampleIndex, somewhere inside
// a hop. Padding the sidecar with padSamples of silence puts its hop
// boundaries where live had them, and then a replayed frame j is the live
// frame j + frameOffset. Exact when a hop is a whole number of frames.
struct MouthReplayAlignment
{
	int64_t padSamples= 0;
	int64_t frameOffset= 0;
};
MouthReplayAlignment mouthReplayAlignment(int64_t startSampleIndex, const MouthWindowConfig& window);

struct MouthReplayOutput
{
	std::vector<MouthChainFrame> frames;
	std::vector<MouthChainRun> runs;
};

// Runs the chain over a clip the way the live stage would have: the backend
// reset for the new stream, padSamples of silence first, then the clip in
// capture-sized blocks
void replayMouthChain(const std::vector<float>& clip, int64_t padSamples, const MouthChainConfig& config,
					  IMouthBackend& backend, MouthReplayOutput& outReplay);
