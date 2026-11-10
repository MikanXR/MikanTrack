#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "FaceTypes.h"
#include "VisemeTypes.h"

// The audio stage's sample rate: the backend consumes 16 kHz mono float
constexpr int AUDIO_SAMPLE_RATE= 16000;
// The rate the backend emits mouth frames at
constexpr int MOUTH_FRAME_RATE= 30;

// The mouth model the Microphone stage selects: the level alone, uLipSync's MFCC
// scoring against a calibrated profile, or the Oculus LipSync engine
enum class eMouthModel : int
{
	Loudness= 0,
	Mfcc= 1,
	OvrLipSync= 2,
};
constexpr int MOUTH_MODEL_COUNT= 3;
// The config's name for a model: loudness, mfcc, ovrlipsync
const char* mouthModelName(eMouthModel model);
// Parses a config name; false (and outModel untouched) for an unknown one
bool parseMouthModel(const std::string& name, eMouthModel& outModel);

// The emission filter's settling time per model, in ms: the one-euro
// minimum cutoff's time constant. The loudness mouth settles faster, since
// it has no vowel calls to steady, only a level to follow.
constexpr float MOUTH_SMOOTHING_DEFAULT_MS[MOUTH_MODEL_COUNT]= {20.f, 50.f, 50.f};
constexpr float MOUTH_SMOOTHING_MAX_MS= 200.f;
// The minimum cutoff in Hz for a settling time, 0 for no filter at 0 ms
float smoothingCutoffHz(float smoothingMs);

// The Oculus engine's own viseme smoothing, its VisemeSmoothing signal:
// 1 is the engine's raw prediction, 100 its smoothest response
constexpr int OVR_SMOOTHING_MIN= 1;
constexpr int OVR_SMOOTHING_MAX= 100;
constexpr int OVR_SMOOTHING_DEFAULT= 70;

// Whether an ARKit column is one the voice drives: the four jaw columns, the
// twenty-three mouth columns, cheekPuff, and tongueOut. Audio cannot see
// eyes or brows, so the viseme table never touches another column.
bool isMouthArkitIndex(int arkitIndex);

// The number of 30 fps frames the backend emits for a span of samples:
// ceil(30 * samples / 16000)
int mouthFramesForSamples(size_t sampleCount);
// The sample a 30 fps frame ends at, round((frameIndex + 1) * 16000 / 30):
// the block that scores the frame ends there
size_t mouthFrameEndSample(int frameIndex);

// One 30 fps frame of viseme weights, 0..1, in eViseme order
struct MouthFrame
{
	VisemeWeights weights{};
};

// The viseme with the largest weight, or silence when no viseme is above zero
int dominantViseme(const VisemeWeights& weights);

// The newest mouth frame the audio stage published
struct MouthSample
{
	// false until the first frame is emitted
	bool valid= false;
	// +1 per emitted frame
	uint64_t sequence= 0;
	// steadyNowMs() at emission, the same clock camera frames use
	double timestampMs= 0.0;
	// The frame's index in the capture stream since the device opened, what
	// ties it to a recording of that stream
	int64_t audioFrameIndex= -1;
	// +1 per device open; frame indices restart with it
	uint32_t captureEpoch= 0;
	MouthFrame frame;
};
