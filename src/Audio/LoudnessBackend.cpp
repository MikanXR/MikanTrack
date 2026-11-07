#include "LoudnessBackend.h"

#include <algorithm>
#include <chrono>

#include "MfccFeatures.h"
#include "MouthChain.h"

void LoudnessBackend::setVolumeWindow(float minDbfs, float maxDbfs)
{
	m_minDbfs= minDbfs;
	m_maxDbfs= std::max(maxDbfs, minDbfs + 1.f);
}

bool LoudnessBackend::infer(const float* samples, size_t sampleCount, int, std::vector<MouthFrame>& outFrames)
{
	outFrames.clear();
	if (sampleCount == 0)
		return false;

	const auto start= std::chrono::steady_clock::now();

	const int frameCount= mouthFramesForSamples(sampleCount);
	outFrames.resize((size_t)frameCount);
	for (int frameIndex= 0; frameIndex < frameCount; ++frameIndex)
	{
		// The block ends at the frame's end and reaches back a whole block,
		// the front zero-padded where the window has not reached one: the
		// padding counts as silence in the RMS, as it does for the MFCC model
		const size_t end= std::min(mouthFrameEndSample(frameIndex), sampleCount);
		const size_t available= std::min(end, (size_t)LIPSYNC_BLOCK_SAMPLES);
		const float levelDbfs= audioBlockLevelDbfs(samples + end - available, available, LIPSYNC_BLOCK_SAMPLES);
		const float volume= volumeForLevel(levelDbfs, m_minDbfs, m_maxDbfs);

		VisemeWeights& weights= outFrames[(size_t)frameIndex].weights;
		weights.fill(0.f);
		weights[(size_t)eViseme::aa]= volume;
		weights[(size_t)VISEME_SIL]= 1.f - volume;
	}

	m_lastInferMs= std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	return true;
}
