#include "MfccFeatures.h"

#include <algorithm>
#include <cmath>

#include "AudioTypes.h"

namespace
{
constexpr double k_pi= 3.14159265358979323846;
// Unity.Mathematics EPSILON, under which uLipSync skips the normalization
constexpr float k_peakEpsilon= 1.1920929e-7f;

// uLipSync's mel scale, the 1127 ln form, in single precision as it computes
// it
float hzToMel(float hz)
{
	return 1127.f * std::log(hz / 700.f + 1.f);
}

float melToHz(float mel)
{
	return 700.f * (std::exp(mel / 1127.f) - 1.f);
}
} // namespace

MfccExtractor::MfccExtractor()
{
	constexpr int n= LIPSYNC_BLOCK_SAMPLES;

	const double theta= -2.0 * k_pi / n;
	for (int index= 0; index < n; ++index)
	{
		const double phase= theta * index;
		m_cosSin[2 * index]= (float)std::cos(phase);
		m_cosSin[2 * index + 1]= (float)std::sin(phase);
	}

	int bits= 0;
	while ((1 << bits) < n)
		++bits;
	for (int index= 0; index < n; ++index)
	{
		uint32_t reversed= 0;
		for (int bit= 0; bit < bits; ++bit)
			reversed= (reversed << 1) | ((index >> bit) & 1);
		m_bitReverse[index]= reversed;
	}

	for (int index= 0; index < n; ++index)
		m_hamming[index]= 0.54f - 0.46f * (float)std::cos(2.0 * k_pi * ((float)index / (n - 1)));

	configure(LIPSYNC_DEFAULT_MEL_CHANNELS);
}

void MfccExtractor::configure(int melChannels)
{
	m_melChannels= std::clamp(melChannels, MFCC_COEFF_COUNT + 1, LIPSYNC_BLOCK_SAMPLES / 2);
	buildMelFilterbank();
	buildDct();
	m_melDb.assign((size_t)m_melChannels, 0.f);
}

void MfccExtractor::buildMelFilterbank()
{
	// uLipSync's bank: channel edges evenly spaced on the mel scale from 0 to
	// half the sample rate, each triangle summed over the bins strictly
	// above its ceil'd start up to its floor'd end, the weights scaled by
	// half the triangle's width in Hz
	constexpr int nMax= LIPSYNC_BLOCK_SAMPLES / 2;
	const float fMax= (float)AUDIO_SAMPLE_RATE / 2.f;
	const float melMax= hzToMel(fMax);
	const float df= fMax / nMax;
	const float dMel= melMax / (float)(m_melChannels + 1);

	m_melBands.assign((size_t)m_melChannels, MelBand());
	for (int channel= 0; channel < m_melChannels; ++channel)
	{
		const float fBegin= melToHz(dMel * (float)channel);
		const float fCenter= melToHz(dMel * (float)(channel + 1));
		const float fEnd= melToHz(dMel * (float)(channel + 2));
		const int iBegin= (int)std::ceil(fBegin / df);
		const int iCenter= (int)std::round(fCenter / df);
		const int iEnd= std::min((int)std::floor(fEnd / df), nMax);

		MelBand& band= m_melBands[(size_t)channel];
		band.firstBin= iBegin + 1;
		for (int bin= iBegin + 1; bin <= iEnd; ++bin)
		{
			const float f= df * (float)bin;
			float weight= bin < iCenter ? (f - fBegin) / (fCenter - fBegin) : (fEnd - f) / (fEnd - fCenter);
			weight/= (fEnd - fBegin) * 0.5f;
			band.weights.push_back(weight);
		}
	}
}

void MfccExtractor::buildDct()
{
	// The direct DCT-II uLipSync runs, unnormalized, rows 1..12
	const double a= k_pi / m_melChannels;
	for (int row= 1; row <= MFCC_COEFF_COUNT; ++row)
	{
		std::vector<float>& coefficients= m_dct[(size_t)(row - 1)];
		coefficients.resize((size_t)m_melChannels);
		for (int channel= 0; channel < m_melChannels; ++channel)
			coefficients[(size_t)channel]= (float)std::cos((channel + 0.5) * row * a);
	}
}

void MfccExtractor::fftMagnitude()
{
	constexpr int n= LIPSYNC_BLOCK_SAMPLES;

	for (int index= 0; index < n; ++index)
	{
		const uint32_t target= m_bitReverse[index];
		m_spectrum[2 * target]= m_block[index];
		m_spectrum[2 * target + 1]= 0.f;
	}

	// Iterative radix-2 butterflies
	for (int half= 1; half < n; half<<= 1)
	{
		const int step= half << 1;
		const int delta= n / step;
		for (int k= 0; k < half; ++k)
		{
			const float twiddleRe= m_cosSin[2 * (k * delta)];
			const float twiddleIm= m_cosSin[2 * (k * delta) + 1];
			for (int j= k; j < n; j+= step)
			{
				const int i0= j << 1;
				const int i1= (j + half) << 1;
				const float s0Re= m_spectrum[i0];
				const float s0Im= m_spectrum[i0 + 1];
				const float s1Re= m_spectrum[i1];
				const float s1Im= m_spectrum[i1 + 1];
				const float v1Re= s1Re * twiddleRe - s1Im * twiddleIm;
				const float v1Im= s1Re * twiddleIm + s1Im * twiddleRe;
				m_spectrum[i0]= s0Re + v1Re;
				m_spectrum[i0 + 1]= s0Im + v1Im;
				m_spectrum[i1]= s0Re - v1Re;
				m_spectrum[i1 + 1]= s0Im - v1Im;
			}
		}
	}

	for (int k= 0; k <= n / 2; ++k)
	{
		const float re= m_spectrum[2 * k];
		const float im= m_spectrum[2 * k + 1];
		m_magnitude[k]= std::sqrt(re * re + im * im);
	}
}

void MfccExtractor::compute(const float* block, float& outLevelDbfs, MfccVector& outMfcc)
{
	constexpr int n= LIPSYNC_BLOCK_SAMPLES;

	double sum= 0.0;
	for (int index= 0; index < n; ++index)
		sum+= (double)block[index] * block[index];
	outLevelDbfs= std::max(-100.f, 20.f * (float)std::log10(std::sqrt(sum / n) + 1e-10));

	// Pre-emphasis with the first sample as it was, the Hamming window, then
	// the peak brought to 1 unless the block is all but silent
	m_block[0]= block[0];
	for (int index= 1; index < n; ++index)
		m_block[index]= block[index] - MFCC_PREEMPHASIS_ALPHA * block[index - 1];
	float peak= 0.f;
	for (int index= 0; index < n; ++index)
	{
		m_block[index]*= m_hamming[index];
		peak= std::max(peak, std::fabs(m_block[index]));
	}
	if (peak >= k_peakEpsilon)
	{
		const float scale= 1.f / peak;
		for (float& sample : m_block)
			sample*= scale;
	}

	fftMagnitude();

	for (int channel= 0; channel < m_melChannels; ++channel)
	{
		const MelBand& band= m_melBands[(size_t)channel];
		float bandSum= 0.f;
		for (size_t index= 0; index < band.weights.size(); ++index)
			bandSum+= band.weights[index] * m_magnitude[(size_t)band.firstBin + index];
		m_melDb[(size_t)channel]= 10.f * std::log10(bandSum + 1e-10f);
	}

	for (int row= 0; row < MFCC_COEFF_COUNT; ++row)
	{
		const std::vector<float>& coefficients= m_dct[(size_t)row];
		float dot= 0.f;
		for (int channel= 0; channel < m_melChannels; ++channel)
			dot+= coefficients[(size_t)channel] * m_melDb[(size_t)channel];
		outMfcc[(size_t)row]= dot;
	}
}
