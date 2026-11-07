#include "MfccBackend.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "Logger.h"
#include "PathUtils.h"

namespace
{
// HeadAudio's easing over a 0..1 ramp: a sigmoid of sharpness 5 rescaled to
// pass through 0 and 1 at the ends
float easeRamp(float t)
{
	constexpr float k= 5.f;
	const auto base= [](float x) { return 1.f / (1.f + std::exp(-k * x)) - 0.5f; };
	const float correction= 0.5f / base(1.f);
	return correction * base(2.f * std::min(std::max(t, 0.f), 1.f) - 1.f) + 0.5f;
}
} // namespace

int MfccBackend::hopsForSamples(size_t sampleCount)
{
	if (sampleCount < (size_t)MFCC_BLOCK_SAMPLES)
		return 0;
	return 1 + (int)((sampleCount - MFCC_BLOCK_SAMPLES) / MFCC_HOP_SAMPLES);
}

bool MfccBackend::create(const std::string& modelPath)
{
	m_lastInferMs= 0.0;
	m_classifier= VisemeClassifier();

	VisemeModel model;
	if (!model.load(PathUtils::utf8ToPath(modelPath), m_unavailableReason))
	{
		MIKAN_MT_LOG_WARNING("MfccBackend::create") << m_unavailableReason;
		return false;
	}
	m_classifier.setModel(model);
	m_unavailableReason.clear();
	return true;
}

bool MfccBackend::infer(const float* samples, size_t sampleCount, std::vector<MouthFrame>& outFrames)
{
	outFrames.clear();
	const int hopCount= hopsForSamples(sampleCount);
	if (!isReady() || hopCount == 0)
		return false;

	const auto start= std::chrono::steady_clock::now();

	m_preEmphasized.resize(sampleCount);
	MfccExtractor::preEmphasize(samples, sampleCount, m_preEmphasized.data());

	// One vote per hop the gate lets through (a gated hop is silence and
	// feeds no vote), then the crossfade: the winning viseme ramps toward one
	// and every other toward zero, a hop's worth per hop
	m_gate.reset();
	m_classifier.reset();
	m_hopVisemes.resize((size_t)hopCount);
	m_hopWeights.resize((size_t)hopCount);
	VisemeWeights alphas{};
	const float rampPerHop= rampMs > 0.f ? (1000.f * MFCC_HOP_SAMPLES / AUDIO_SAMPLE_RATE) / rampMs : 1.f;
	MfccVector vector;
	float logEnergy= 0.f;
	for (int hop= 0; hop < hopCount; ++hop)
	{
		const size_t blockEnd= (size_t)MFCC_BLOCK_SAMPLES + (size_t)hop * MFCC_HOP_SAMPLES;
		m_extractor.compute(m_preEmphasized.data() + blockEnd - MFCC_BLOCK_SAMPLES, logEnergy, vector);
		MfccExtractor::compress(vector);
		const int viseme= m_gate.process(logEnergy) ? m_classifier.predict(vector) : VISEME_SIL;
		m_hopVisemes[(size_t)hop]= (uint8_t)viseme;

		VisemeWeights& weights= m_hopWeights[(size_t)hop];
		for (int index= 0; index < VISEME_COUNT; ++index)
		{
			float& alpha= alphas[index];
			if (index == viseme && viseme != VISEME_SIL)
				alpha= std::min(alpha + rampPerHop, 1.f);
			else
				alpha= std::max(alpha - rampPerHop, 0.f);
			weights[index]= alpha <= 0.f ? 0.f : alpha >= 1.f ? 1.f : easeRamp(alpha);
		}
	}

	// Each 30 fps frame takes the weights of the latest hop centered at or
	// before the frame's center; the earliest frames take the first hop
	const int frameCount= mouthFramesForSamples(sampleCount);
	outFrames.resize((size_t)frameCount);
	for (int frameIndex= 0; frameIndex < frameCount; ++frameIndex)
	{
		const double frameCenter= (frameIndex + 0.5) * AUDIO_SAMPLE_RATE / MOUTH_FRAME_RATE;
		// Hop h is centered at (h + 1) * 256 samples
		const int hop= std::min(std::max((int)std::floor(frameCenter / MFCC_HOP_SAMPLES) - 1, 0), hopCount - 1);
		outFrames[(size_t)frameIndex].weights= m_hopWeights[(size_t)hop];
	}

	m_lastInferMs= std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	return true;
}
