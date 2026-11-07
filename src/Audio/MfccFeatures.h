#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// The MFCC feature chain of the mouth stage, a port of uLipSync's (MIT,
// hecomi) stage for stage so a profile calibrated there reads the same
// coefficients: over one 1024-sample block at 16 kHz, pre-emphasis, a Hamming
// window, a peak normalization to 1, a radix-2 FFT to the magnitude
// spectrum, a mel filterbank of triangles on the 1127 ln scale from 0 to
// 8 kHz, 10 log10 of each band, a direct DCT-II, and coefficients 1..12.
// uLipSync's windowed-sinc low-pass and its decimation to 16 kHz have no
// counterpart here: they anti-alias the host's audio before the downsample,
// and the capture is already 16 kHz. The block's RMS level comes out beside
// the coefficients, the volume uLipSync opens the mouth with.

constexpr int LIPSYNC_BLOCK_SAMPLES= 1024;
constexpr int MFCC_COEFF_COUNT= 12;
constexpr int LIPSYNC_DEFAULT_MEL_CHANNELS= 30;
constexpr float MFCC_PREEMPHASIS_ALPHA= 0.97f;

// One feature vector: the 12 coefficients
using MfccVector= std::array<float, MFCC_COEFF_COUNT>;

class MfccExtractor
{
public:
	MfccExtractor();

	// Rebuilds the mel filterbank and the DCT rows for a channel count; the
	// constructor builds the default
	void configure(int melChannels);
	int getMelChannels() const { return m_melChannels; }

	// One block of LIPSYNC_BLOCK_SAMPLES to its level in dBFS RMS (floored
	// at -100, read before the chain touches the samples) and its 12
	// coefficients. The block is copied, not modified.
	void compute(const float* block, float& outLevelDbfs, MfccVector& outMfcc);

private:
	void buildMelFilterbank();
	void buildDct();
	void fftMagnitude();

	int m_melChannels= LIPSYNC_DEFAULT_MEL_CHANNELS;

	std::array<float, 2 * LIPSYNC_BLOCK_SAMPLES> m_cosSin{};
	std::array<uint32_t, LIPSYNC_BLOCK_SAMPLES> m_bitReverse{};
	std::array<float, LIPSYNC_BLOCK_SAMPLES> m_hamming{};

	// A mel band's triangle: the weights over the FFT bins from firstBin up
	struct MelBand
	{
		int firstBin= 0;
		std::vector<float> weights;
	};
	std::vector<MelBand> m_melBands;
	// Rows 1..12 of the DCT over the mel channels
	std::array<std::vector<float>, MFCC_COEFF_COUNT> m_dct;

	// Scratch for one block: the processed samples, the complex spectrum,
	// the magnitudes of bins 0..N/2, the mel energies in dB
	std::array<float, LIPSYNC_BLOCK_SAMPLES> m_block{};
	std::array<float, 2 * LIPSYNC_BLOCK_SAMPLES> m_spectrum{};
	std::array<float, LIPSYNC_BLOCK_SAMPLES / 2 + 1> m_magnitude{};
	std::vector<float> m_melDb;
};
