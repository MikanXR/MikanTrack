#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

// A decoded wav file: interleaved float samples in -1..1
struct WavData
{
	int sampleRate= 0;
	int channels= 0;
	std::vector<float> samples;

	size_t frameCount() const { return channels > 0 ? samples.size() / (size_t)channels : 0; }
};

// RIFF wave reading for the bench clips and the recording sidecars. Reads
// 8, 16, 24, and 32-bit PCM and 32-bit float, plain or WAVE_FORMAT_EXTENSIBLE,
// any channel count.
namespace WavFile
{
bool read(const std::filesystem::path& path, WavData& outData, std::string& outError);

// Averages the channels down to one
std::vector<float> toMono(const WavData& data);

// Resamples a mono series between rates through SDL's converter. A matching
// rate copies.
bool resample(const std::vector<float>& mono, int fromRate, int toRate, std::vector<float>& outMono,
			  std::string& outError);

// A mono clip at the audio stage's rate, the three steps above in one
bool readMono16k(const std::filesystem::path& path, std::vector<float>& outMono, std::string& outError);
} // namespace WavFile

// Writes a 16-bit PCM wave file sample by sample, for the recording sidecar.
// The header's sizes are placeholders until close(), set to the maximum so a
// file cut short by a crash still reads to its end.
class WavWriter
{
public:
	~WavWriter() { close(); }

	bool open(const std::filesystem::path& path, int sampleRate, int channels, std::string& outError);
	bool isOpen() const { return m_file.is_open(); }

	// Converts floats in -1..1 (clamped) to PCM16 and appends them
	bool write(const float* samples, size_t sampleCount);
	bool writeSilence(size_t sampleCount);

	// Patches the sizes and closes the file
	bool close();

	uint64_t getSamplesWritten() const { return m_samplesWritten; }

private:
	std::ofstream m_file;
	int m_channels= 0;
	uint64_t m_samplesWritten= 0;
};
