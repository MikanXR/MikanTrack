#include "AudioTypes.h"

#include <cmath>
#include <cstring>

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

namespace
{
constexpr const char* k_mouthModelNames[MOUTH_MODEL_COUNT]= {"loudness", "mfcc", "ovrlipsync"};
}

const char* mouthModelName(eMouthModel model)
{
	const int index= (int)model;
	return (index >= 0 && index < MOUTH_MODEL_COUNT) ? k_mouthModelNames[index] : k_mouthModelNames[(int)eMouthModel::Mfcc];
}

float smoothingCutoffHz(float smoothingMs)
{
	if (smoothingMs <= 0.f)
		return 0.f;
	return 1000.f / (2.f * 3.14159265f * smoothingMs);
}

bool parseMouthModel(const std::string& name, eMouthModel& outModel)
{
	for (int index= 0; index < MOUTH_MODEL_COUNT; ++index)
	{
		if (name == k_mouthModelNames[index])
		{
			outModel= (eMouthModel)index;
			return true;
		}
	}
	return false;
}
