#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "MouthChain.h"

class IMouthBackend;

// The backend a config names: "mfcc" is the HeadAudio port, anything else
// the learned wav2arkit model. An empty model path takes the backend's
// default. The backend comes back created, ready or carrying its reason.
std::unique_ptr<IMouthBackend> createMouthBackend(const std::string& backendName, const std::string& modelPath);

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

// Runs the chain over a clip the way the live stage would have: padSamples
// of silence first, then the clip in capture-sized blocks
void replayMouthChain(const std::vector<float>& clip, int64_t padSamples, const MouthChainConfig& config,
					  IMouthBackend& backend, MouthReplayOutput& outReplay);
