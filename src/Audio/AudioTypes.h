#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "FaceTypes.h"

// The audio stage's sample rate: every backend consumes 16 kHz mono float
constexpr int AUDIO_SAMPLE_RATE= 16000;
// The rate the backends emit mouth frames at
constexpr int MOUTH_FRAME_RATE= 30;
// The ARKit columns the voice drives: the four jaw columns, the twenty-three
// mouth columns, cheekPuff, and tongueOut. Everything else a backend outputs
// (eyes, brows, nose, cheek squints) is discarded, since audio cannot see it.
constexpr int MOUTH_COLUMN_COUNT= 29;

// The ARKit index of a mouth column; -1 when out of range
int mouthColumnArkitIndex(int mouthColumn);
// The mouth column of an ARKit index; -1 for a column the voice does not drive
int mouthColumnFromArkitIndex(int arkitIndex);
// The ARKit name of a mouth column; "" when out of range
const char* mouthColumnName(int mouthColumn);

// The number of 30 fps frames a backend emits for a span of samples, the
// formula the wav2arkit model uses: ceil(30 * samples / 16000)
int mouthFramesForSamples(size_t sampleCount);

// One 30 fps frame of mouth weights, 0..1, in mouth column order
struct MouthFrame
{
	std::array<float, MOUTH_COLUMN_COUNT> weights{};
};

// Writes the frame's columns onto a 52-wide ARKit face, leaving the columns
// the voice does not drive untouched
void applyMouthFrame(const MouthFrame& frame, std::array<float, ARKIT_BLENDSHAPE_COUNT>& face);
// Pulls the mouth columns out of a 52-wide ARKit face
MouthFrame mouthFrameFromFace(const std::array<float, ARKIT_BLENDSHAPE_COUNT>& face);

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
