#include "MfccBackend.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "Logger.h"
#include "MouthChain.h"
#include "PathUtils.h"

namespace
{
constexpr std::array<int, 5> k_vowelVisemes= {(int)eViseme::aa, (int)eViseme::E, (int)eViseme::I, (int)eViseme::O,
											  (int)eViseme::U};
}

bool MfccBackend::create(const std::string& profilePath)
{
	m_lastInferMs= 0.0;
	m_profile= LipSyncProfile();
	m_profileError.clear();
	m_bCreated= true;
	if (profilePath.empty())
		return false;

	if (!m_profile.load(PathUtils::utf8ToPath(profilePath), m_profileError))
	{
		MIKAN_MT_LOG_WARNING("MfccBackend::create") << m_profileError;
		m_profile= LipSyncProfile();
		return false;
	}
	m_extractor.configure(m_profile.melFilterBankChannels);
	return true;
}

std::string MfccBackend::getInfo() const
{
	if (!hasProfile())
		return "no profile, volume only";
	return "uLipSync profile, " + std::to_string(m_profile.phonemes.size()) + " phonemes";
}

void MfccBackend::setVolumeWindow(float minDbfs, float maxDbfs)
{
	m_minDbfs= minDbfs;
	m_maxDbfs= std::max(maxDbfs, minDbfs + 1.f);
}

bool MfccBackend::infer(const float* samples, size_t sampleCount, int, std::vector<MouthFrame>& outFrames)
{
	outFrames.clear();
	if (!m_bCreated || sampleCount == 0)
		return false;

	const auto start= std::chrono::steady_clock::now();

	const int frameCount= mouthFramesForSamples(sampleCount);
	outFrames.resize((size_t)frameCount);
	m_frameScores.resize((size_t)frameCount);
	const bool bProfile= hasProfile();
	MfccVector mfcc;
	for (int frameIndex= 0; frameIndex < frameCount; ++frameIndex)
	{
		// The block ends at the frame's end, zero-padded at the front while
		// the window has not reached a whole block
		const size_t end= std::min(mouthFrameEndSample(frameIndex), sampleCount);
		const size_t available= std::min(end, (size_t)LIPSYNC_BLOCK_SAMPLES);
		const size_t padding= (size_t)LIPSYNC_BLOCK_SAMPLES - available;
		std::fill(m_block.begin(), m_block.begin() + (std::ptrdiff_t)padding, 0.f);
		std::copy(samples + end - available, samples + end, m_block.begin() + (std::ptrdiff_t)padding);

		FrameScores& frame= m_frameScores[(size_t)frameIndex];
		m_extractor.compute(m_block.data(), frame.levelDbfs, mfcc);
		frame.volume= volumeForLevel(frame.levelDbfs, m_minDbfs, m_maxDbfs);

		VisemeWeights& weights= outFrames[(size_t)frameIndex].weights;
		weights.fill(0.f);
		float vowelSum= 0.f;
		if (bProfile)
		{
			m_profile.score(mfcc, frame.scores);
			for (int viseme : k_vowelVisemes)
			{
				weights[(size_t)viseme]= frame.scores.perViseme[(size_t)viseme] * frame.volume;
				vowelSum+= weights[(size_t)viseme];
			}
		}
		else
		{
			frame.scores= LipSyncScores();
			frame.scores.mainViseme= frame.volume > 0.f ? (int)eViseme::aa : VISEME_SIL;
			weights[(size_t)eViseme::aa]= frame.volume;
			vowelSum= frame.volume;
		}
		weights[(size_t)VISEME_SIL]= std::clamp(1.f - vowelSum, 0.f, 1.f);
	}

	m_lastInferMs= std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	return true;
}
