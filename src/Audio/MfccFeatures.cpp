#include "MfccFeatures.h"

#include <algorithm>
#include <cmath>

#include "AudioTypes.h"

namespace
{
constexpr int k_lifter= 22;
constexpr float k_melLowHz= 30.f;
constexpr float k_melHighHz= 7800.f;
constexpr double k_pi= 3.14159265358979323846;

double hzToMel(double hz)
{
	return 2595.0 * std::log10(1.0 + hz / 700.0);
}

double melToHz(double mel)
{
	return 700.0 * (std::pow(10.0, mel / 2595.0) - 1.0);
}
} // namespace

MfccExtractor::MfccExtractor()
{
	constexpr int n= MFCC_BLOCK_SAMPLES;

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
		m_hamming[index]= (float)(0.54 - 0.46 * std::cos(2.0 * k_pi * index / (n - 1)));

	buildMelFilterbank();

	const double scale= std::sqrt(2.0 / MFCC_MEL_BANDS);
	for (int row= 0; row <= MFCC_COEFF_COUNT; ++row)
	{
		for (int band= 0; band < MFCC_MEL_BANDS; ++band)
			m_dct[row][band]= (float)(scale * std::cos(k_pi * row * (band + 0.5) / MFCC_MEL_BANDS));
	}

	for (int index= 1; index <= MFCC_COEFF_COUNT; ++index)
		m_lifter[index - 1]= (float)(1.0 + (k_lifter / 2.0) * std::sin(k_pi * index / k_lifter));
}

void MfccExtractor::buildMelFilterbank()
{
	// Band edges spaced evenly on the mel scale between the low and high
	// frequencies, then snapped to FFT bins
	const double melLow= hzToMel(k_melLowHz);
	const double melHigh= hzToMel(k_melHighHz);
	std::array<int, MFCC_MEL_BANDS + 2> bin{};
	for (int index= 0; index < MFCC_MEL_BANDS + 2; ++index)
	{
		const double mel= melLow + (melHigh - melLow) / (MFCC_MEL_BANDS + 1) * index;
		// The JS builds the edge in single precision before snapping it
		const float hz= (float)melToHz(mel);
		bin[index]= (int)std::floor((hz / AUDIO_SAMPLE_RATE) * MFCC_BLOCK_SAMPLES);
	}

	for (int band= 0; band < MFCC_MEL_BANDS; ++band)
	{
		std::array<float, MFCC_BLOCK_SAMPLES / 2>& filter= m_melFilters[band];
		filter.fill(0.f);
		for (int k= bin[band]; k < bin[band + 1]; ++k)
			filter[k]= (float)(k - bin[band]) / (float)(bin[band + 1] - bin[band]);
		for (int k= bin[band + 1]; k < bin[band + 2]; ++k)
			filter[k]= (float)(bin[band + 2] - k) / (float)(bin[band + 2] - bin[band + 1]);
	}
}

void MfccExtractor::preEmphasize(const float* in, size_t count, float* out)
{
	float previous= 0.f;
	for (size_t index= 0; index < count; ++index)
	{
		const float sample= in[index];
		out[index]= sample - MFCC_PREEMPHASIS_ALPHA * previous;
		previous= sample;
	}
}

void MfccExtractor::hammingAndFft(const float* block)
{
	constexpr int n= MFCC_BLOCK_SAMPLES;

	// Load windowed samples straight into bit-reversed order
	for (int index= 0; index < n; ++index)
	{
		const uint32_t target= m_bitReverse[index];
		m_spectrum[2 * target]= block[index] * m_hamming[index];
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
}

void MfccExtractor::compute(const float* block, float& outLogEnergy, MfccVector& outRaw)
{
	constexpr int halfN= MFCC_BLOCK_SAMPLES / 2;

	hammingAndFft(block);

	double totalEnergy= 0.0;
	for (int k= 0; k < halfN; ++k)
	{
		const float re= m_spectrum[2 * k];
		const float im= m_spectrum[2 * k + 1];
		m_power[k]= (re * re + im * im) / MFCC_BLOCK_SAMPLES;
		totalEnergy+= m_power[k];
	}
	outLogEnergy= (float)std::log10(totalEnergy + 1e-10);

	for (int band= 0; band < MFCC_MEL_BANDS; ++band)
	{
		const std::array<float, halfN>& filter= m_melFilters[band];
		double sum= 0.0;
		for (int k= 0; k < halfN; ++k)
			sum+= m_power[k] * filter[k];
		m_melEnergies[band]= (float)std::log10(sum + 1e-10);
	}

	for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
	{
		const std::array<float, MFCC_MEL_BANDS>& row= m_dct[index + 1];
		double sum= 0.0;
		for (int band= 0; band < MFCC_MEL_BANDS; ++band)
			sum+= row[band] * m_melEnergies[band];
		outRaw[index]= (float)sum * m_lifter[index];
	}
}

void MfccExtractor::compress(MfccVector& vector)
{
	for (float& value : vector)
		value= std::tanh(value);
}
