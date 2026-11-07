#include "WavFile.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>

#include "SDL.h"

#include "AudioTypes.h"

namespace
{
constexpr uint16_t k_formatPcm= 1;
constexpr uint16_t k_formatFloat= 3;
constexpr uint16_t k_formatExtensible= 0xFFFE;

uint32_t readU32(const uint8_t* bytes)
{
	return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

uint16_t readU16(const uint8_t* bytes)
{
	return (uint16_t)(bytes[0] | (bytes[1] << 8));
}
} // namespace

bool WavFile::read(const std::filesystem::path& path, WavData& outData, std::string& outError)
{
	outData= WavData();

	std::ifstream file(path, std::ios::binary);
	if (!file)
	{
		outError= "cannot open " + path.string();
		return false;
	}
	std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

	if (bytes.size() < 12 || std::memcmp(bytes.data(), "RIFF", 4) != 0 || std::memcmp(bytes.data() + 8, "WAVE", 4) != 0)
	{
		outError= "not a RIFF wave file";
		return false;
	}

	uint16_t format= 0;
	uint16_t channels= 0;
	uint32_t sampleRate= 0;
	uint16_t bitsPerSample= 0;
	bool bHaveFormat= false;
	const uint8_t* data= nullptr;
	size_t dataSize= 0;

	size_t offset= 12;
	while (offset + 8 <= bytes.size())
	{
		const uint8_t* chunk= bytes.data() + offset;
		const uint32_t chunkSize= readU32(chunk + 4);
		const uint8_t* body= chunk + 8;
		const size_t bodyAvailable= std::min((size_t)chunkSize, bytes.size() - (offset + 8));

		if (std::memcmp(chunk, "fmt ", 4) == 0 && bodyAvailable >= 16)
		{
			format= readU16(body);
			channels= readU16(body + 2);
			sampleRate= readU32(body + 4);
			bitsPerSample= readU16(body + 14);
			if (format == k_formatExtensible && bodyAvailable >= 26)
			{
				// The real format tag is the first two bytes of the sub-format GUID
				format= readU16(body + 24);
			}
			bHaveFormat= true;
		}
		else if (std::memcmp(chunk, "data", 4) == 0)
		{
			data= body;
			dataSize= bodyAvailable;
			break;
		}

		// Chunks are word-aligned
		offset+= 8 + (size_t)chunkSize + (chunkSize & 1);
	}

	if (!bHaveFormat || data == nullptr)
	{
		outError= "missing fmt or data chunk";
		return false;
	}
	if (channels == 0 || sampleRate == 0)
	{
		outError= "zero channels or sample rate";
		return false;
	}

	const size_t bytesPerSample= bitsPerSample / 8;
	if (format == k_formatPcm && (bitsPerSample == 8 || bitsPerSample == 16 || bitsPerSample == 24 || bitsPerSample == 32))
	{
		const size_t count= dataSize / bytesPerSample;
		outData.samples.resize(count);
		for (size_t index= 0; index < count; ++index)
		{
			const uint8_t* sample= data + index * bytesPerSample;
			float value= 0.f;
			switch (bitsPerSample)
			{
				case 8:
					value= ((float)sample[0] - 128.f) / 128.f;
					break;
				case 16:
					value= (float)(int16_t)readU16(sample) / 32768.f;
					break;
				case 24:
				{
					const int32_t raw= (int32_t)(((uint32_t)sample[0] << 8) | ((uint32_t)sample[1] << 16) |
												 ((uint32_t)sample[2] << 24)) >> 8;
					value= (float)raw / 8388608.f;
					break;
				}
				case 32:
					value= (float)(int32_t)readU32(sample) / 2147483648.f;
					break;
			}
			outData.samples[index]= value;
		}
	}
	else if (format == k_formatFloat && bitsPerSample == 32)
	{
		const size_t count= dataSize / 4;
		outData.samples.resize(count);
		std::memcpy(outData.samples.data(), data, count * 4);
	}
	else
	{
		outError= "unsupported format tag " + std::to_string(format) + " at " + std::to_string(bitsPerSample) + " bits";
		return false;
	}

	outData.channels= channels;
	outData.sampleRate= (int)sampleRate;
	return true;
}

std::vector<float> WavFile::toMono(const WavData& data)
{
	std::vector<float> mono;
	if (data.channels <= 0)
		return mono;

	const size_t frameCount= data.frameCount();
	mono.resize(frameCount);
	const float scale= 1.f / (float)data.channels;
	for (size_t frame= 0; frame < frameCount; ++frame)
	{
		float sum= 0.f;
		for (int channel= 0; channel < data.channels; ++channel)
			sum+= data.samples[frame * (size_t)data.channels + (size_t)channel];
		mono[frame]= sum * scale;
	}
	return mono;
}

bool WavFile::resample(const std::vector<float>& mono, int fromRate, int toRate, std::vector<float>& outMono,
					   std::string& outError)
{
	if (fromRate == toRate)
	{
		outMono= mono;
		return true;
	}

	SDL_AudioCVT cvt;
	const int needed= SDL_BuildAudioCVT(&cvt, AUDIO_F32SYS, 1, fromRate, AUDIO_F32SYS, 1, toRate);
	if (needed < 0)
	{
		outError= std::string("SDL_BuildAudioCVT: ") + SDL_GetError();
		return false;
	}
	if (needed == 0)
	{
		outMono= mono;
		return true;
	}

	cvt.len= (int)(mono.size() * sizeof(float));
	std::vector<uint8_t> buffer((size_t)cvt.len * (size_t)cvt.len_mult);
	std::memcpy(buffer.data(), mono.data(), (size_t)cvt.len);
	cvt.buf= buffer.data();
	if (SDL_ConvertAudio(&cvt) != 0)
	{
		outError= std::string("SDL_ConvertAudio: ") + SDL_GetError();
		return false;
	}

	const size_t outCount= (size_t)cvt.len_cvt / sizeof(float);
	outMono.resize(outCount);
	std::memcpy(outMono.data(), buffer.data(), outCount * sizeof(float));
	return true;
}

bool WavFile::readMono16k(const std::filesystem::path& path, std::vector<float>& outMono, std::string& outError)
{
	WavData data;
	if (!read(path, data, outError))
		return false;
	return resample(toMono(data), data.sampleRate, AUDIO_SAMPLE_RATE, outMono, outError);
}

// -- WavWriter -------------------------------------------------------------------

namespace
{
constexpr uint32_t k_placeholderSize= 0xFFFFFFFFu;
constexpr std::streamoff k_riffSizeOffset= 4;
constexpr std::streamoff k_dataSizeOffset= 40;
constexpr std::streamoff k_headerBytes= 44;

void writeU16(std::ofstream& file, uint16_t value)
{
	const uint8_t bytes[2]= {(uint8_t)(value & 0xff), (uint8_t)(value >> 8)};
	file.write((const char*)bytes, 2);
}

void writeU32(std::ofstream& file, uint32_t value)
{
	const uint8_t bytes[4]= {(uint8_t)(value & 0xff), (uint8_t)((value >> 8) & 0xff), (uint8_t)((value >> 16) & 0xff),
							 (uint8_t)(value >> 24)};
	file.write((const char*)bytes, 4);
}
} // namespace

bool WavWriter::open(const std::filesystem::path& path, int sampleRate, int channels, std::string& outError)
{
	close();
	if (sampleRate <= 0 || channels <= 0)
	{
		outError= "zero channels or sample rate";
		return false;
	}
	m_file.open(path, std::ios::binary | std::ios::trunc);
	if (!m_file)
	{
		outError= "cannot open " + path.string();
		return false;
	}
	m_channels= channels;
	m_samplesWritten= 0;

	const uint16_t blockAlign= (uint16_t)(channels * 2);
	m_file.write("RIFF", 4);
	writeU32(m_file, k_placeholderSize);
	m_file.write("WAVE", 4);
	m_file.write("fmt ", 4);
	writeU32(m_file, 16);
	writeU16(m_file, k_formatPcm);
	writeU16(m_file, (uint16_t)channels);
	writeU32(m_file, (uint32_t)sampleRate);
	writeU32(m_file, (uint32_t)sampleRate * blockAlign);
	writeU16(m_file, blockAlign);
	writeU16(m_file, 16);
	m_file.write("data", 4);
	writeU32(m_file, k_placeholderSize);
	return m_file.good();
}

bool WavWriter::write(const float* samples, size_t sampleCount)
{
	if (!m_file.is_open())
		return false;
	std::vector<int16_t> pcm(sampleCount);
	for (size_t index= 0; index < sampleCount; ++index)
	{
		const float clamped= std::max(-1.f, std::min(1.f, samples[index]));
		pcm[index]= (int16_t)std::lrintf(clamped * 32767.f);
	}
	m_file.write((const char*)pcm.data(), (std::streamsize)(sampleCount * sizeof(int16_t)));
	m_samplesWritten+= sampleCount;
	return m_file.good();
}

bool WavWriter::writeSilence(size_t sampleCount)
{
	if (!m_file.is_open())
		return false;
	const std::vector<int16_t> zeros(sampleCount, 0);
	m_file.write((const char*)zeros.data(), (std::streamsize)(sampleCount * sizeof(int16_t)));
	m_samplesWritten+= sampleCount;
	return m_file.good();
}

bool WavWriter::close()
{
	if (!m_file.is_open())
		return false;
	const uint64_t dataBytes= m_samplesWritten * sizeof(int16_t);
	const uint32_t dataSize= (uint32_t)std::min<uint64_t>(dataBytes, k_placeholderSize);
	m_file.seekp(k_riffSizeOffset);
	writeU32(m_file, (uint32_t)std::min<uint64_t>(dataBytes + (uint64_t)(k_headerBytes - 8), k_placeholderSize));
	m_file.seekp(k_dataSizeOffset);
	writeU32(m_file, dataSize);
	const bool bOk= m_file.good();
	m_file.close();
	return bOk;
}
