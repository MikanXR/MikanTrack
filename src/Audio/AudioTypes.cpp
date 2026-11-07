#include "AudioTypes.h"

#include <cmath>

// The ARKit table lists the jaw and mouth columns contiguously, then
// cheekPuff and tongueOut further on
static constexpr int k_jawFirst= 14;
static constexpr int k_mouthLast= 40;
static constexpr int k_cheekPuff= 46;
static constexpr int k_tongueOut= 51;

bool isMouthArkitIndex(int arkitIndex)
{
	return (arkitIndex >= k_jawFirst && arkitIndex <= k_mouthLast) || arkitIndex == k_cheekPuff ||
		   arkitIndex == k_tongueOut;
}

int mouthFramesForSamples(size_t sampleCount)
{
	return (int)std::ceil((double)MOUTH_FRAME_RATE * (double)sampleCount / (double)AUDIO_SAMPLE_RATE);
}

size_t mouthFrameEndSample(int frameIndex)
{
	return (size_t)std::llround((double)(frameIndex + 1) * (double)AUDIO_SAMPLE_RATE / (double)MOUTH_FRAME_RATE);
}

int dominantViseme(const VisemeWeights& weights)
{
	int best= VISEME_SIL;
	float bestWeight= 0.f;
	for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
	{
		if (viseme != VISEME_SIL && weights[viseme] > bestWeight)
		{
			best= viseme;
			bestWeight= weights[viseme];
		}
	}
	return best;
}
