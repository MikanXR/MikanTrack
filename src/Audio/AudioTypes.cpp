#include "AudioTypes.h"

#include <cmath>

// The ARKit table lists the jaw and mouth columns contiguously, then
// cheekPuff and tongueOut further on
static constexpr int k_jawFirst= 14;
static constexpr int k_mouthLast= 40;
static constexpr int k_cheekPuff= 46;
static constexpr int k_tongueOut= 51;

int mouthColumnArkitIndex(int mouthColumn)
{
	if (mouthColumn < 0 || mouthColumn >= MOUTH_COLUMN_COUNT)
		return -1;
	if (mouthColumn < MOUTH_COLUMN_COUNT - 2)
		return k_jawFirst + mouthColumn;
	return mouthColumn == MOUTH_COLUMN_COUNT - 2 ? k_cheekPuff : k_tongueOut;
}

int mouthColumnFromArkitIndex(int arkitIndex)
{
	if (arkitIndex >= k_jawFirst && arkitIndex <= k_mouthLast)
		return arkitIndex - k_jawFirst;
	if (arkitIndex == k_cheekPuff)
		return MOUTH_COLUMN_COUNT - 2;
	if (arkitIndex == k_tongueOut)
		return MOUTH_COLUMN_COUNT - 1;
	return -1;
}

const char* mouthColumnName(int mouthColumn)
{
	return arkitBlendshapeName(mouthColumnArkitIndex(mouthColumn));
}

int mouthFramesForSamples(size_t sampleCount)
{
	return (int)std::ceil((double)MOUTH_FRAME_RATE * (double)sampleCount / (double)AUDIO_SAMPLE_RATE);
}

void applyMouthFrame(const MouthFrame& frame, std::array<float, ARKIT_BLENDSHAPE_COUNT>& face)
{
	for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
		face[mouthColumnArkitIndex(column)]= frame.weights[column];
}

MouthFrame mouthFrameFromFace(const std::array<float, ARKIT_BLENDSHAPE_COUNT>& face)
{
	MouthFrame frame;
	for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
		frame.weights[column]= face[mouthColumnArkitIndex(column)];
	return frame;
}
