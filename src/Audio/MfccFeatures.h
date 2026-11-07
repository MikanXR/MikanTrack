#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// The MFCC feature chain of the mfcc backend, a port of HeadAudio's (MIT,
// Mika Suominen) stage for stage so its shipped model reads the same
// vectors: pre-emphasis, 512-sample blocks at a 256-sample hop, a Hamming
// window, a radix-2 FFT, the power spectrum, 40 triangular mel bands over
// 30..7800 Hz, a DCT keeping coefficients 1..12 under a sinusoidal lifter,
// then tanh compression. The input is already 16 kHz, so HeadAudio's
// polyphase downsampler has no counterpart here.

constexpr int MFCC_BLOCK_SAMPLES= 512;
constexpr int MFCC_HOP_SAMPLES= 256;
constexpr int MFCC_COEFF_COUNT= 12;
constexpr int MFCC_MEL_BANDS= 40;
constexpr float MFCC_PREEMPHASIS_ALPHA= 0.97f;

// One feature vector: the 12 coefficients, raw out of the DCT or compressed
using MfccVector= std::array<float, MFCC_COEFF_COUNT>;

class MfccExtractor
{
public:
	MfccExtractor();

	// out[i]= in[i] - 0.97 * in[i - 1], with in[-1] taken as zero
	static void preEmphasize(const float* in, size_t count, float* out);

	// One block of MFCC_BLOCK_SAMPLES to its log energy (log10 of the summed
	// power spectrum, HeadAudio's "le", a tenth of the level in dB) and the
	// 12 raw coefficients
	void compute(const float* block, float& outLogEnergy, MfccVector& outRaw);

	// The tanh compression the classifier's vectors carry, in place
	static void compress(MfccVector& vector);

private:
	void buildMelFilterbank();
	void hammingAndFft(const float* block);

	std::array<float, 2 * MFCC_BLOCK_SAMPLES> m_cosSin{};
	std::array<uint32_t, MFCC_BLOCK_SAMPLES> m_bitReverse{};
	std::array<float, MFCC_BLOCK_SAMPLES> m_hamming{};
	std::array<std::array<float, MFCC_BLOCK_SAMPLES / 2>, MFCC_MEL_BANDS> m_melFilters{};
	std::array<std::array<float, MFCC_MEL_BANDS>, MFCC_COEFF_COUNT + 1> m_dct{};
	std::array<float, MFCC_COEFF_COUNT> m_lifter{};

	// Scratch for one block: the complex spectrum, the power spectrum, the
	// log mel energies
	std::array<float, 2 * MFCC_BLOCK_SAMPLES> m_spectrum{};
	std::array<float, MFCC_BLOCK_SAMPLES / 2> m_power{};
	std::array<float, MFCC_MEL_BANDS> m_melEnergies{};
};
