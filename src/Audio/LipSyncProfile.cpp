#include "LipSyncProfile.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>

#include "nlohmann/json.hpp"

#include "AudioTypes.h"
#include "PathUtils.h"

namespace
{
constexpr float k_cosinePower= 100.f;

std::string lowered(const std::string& text)
{
	std::string result= text;
	for (char& c : result)
		c= (char)std::tolower((unsigned char)c);
	return result;
}
} // namespace

int visemeForPhonemeName(const std::string& name)
{
	const std::string key= lowered(name);
	if (key == "a" || key == "aa")
		return (int)eViseme::aa;
	if (key == "i" || key == "ih")
		return (int)eViseme::I;
	if (key == "u" || key == "ou" || key == "oo")
		return (int)eViseme::U;
	if (key == "e" || key == "eh")
		return (int)eViseme::E;
	if (key == "o" || key == "oh")
		return (int)eViseme::O;
	if (key == "-" || key == "n" || key == "sil" || key == "silence" || key == "noise")
		return VISEME_SIL;
	return -1;
}

LipSyncProfile::LipSyncProfile()
{
	stdDevs.fill(1.f);
}

bool LipSyncProfile::load(const std::filesystem::path& path, std::string& outError)
{
	*this= LipSyncProfile();

	std::ifstream file(path, std::ios::binary);
	if (!file)
	{
		outError= "profile missing: " + PathUtils::pathToUtf8(path);
		return false;
	}
	nlohmann::json root;
	try
	{
		root= nlohmann::json::parse(file);
	}
	catch (const std::exception& e)
	{
		outError= "profile is not JSON: " + std::string(e.what());
		return false;
	}
	if (!root.is_object())
	{
		outError= "profile is not a JSON object: " + PathUtils::pathToUtf8(path);
		return false;
	}

	jsonPath= root.value("jsonPath", std::string());
	mfccNum= root.value("mfccNum", MFCC_COEFF_COUNT);
	mfccDataCount= root.value("mfccDataCount", k_defaultMfccDataCount);
	melFilterBankChannels= root.value("melFilterBankChannels", LIPSYNC_DEFAULT_MEL_CHANNELS);
	targetSampleRate= root.value("targetSampleRate", AUDIO_SAMPLE_RATE);
	sampleCount= root.value("sampleCount", LIPSYNC_BLOCK_SAMPLES);
	useStandardization= root.value("useStandardization", false);
	compareMethod= (eCompareMethod)root.value("compareMethod", (int)eCompareMethod::L2Norm);

	if (mfccNum != MFCC_COEFF_COUNT)
	{
		outError= "profile has " + std::to_string(mfccNum) + " coefficients, the stage runs " +
				  std::to_string(MFCC_COEFF_COUNT);
		return false;
	}
	if (sampleCount != LIPSYNC_BLOCK_SAMPLES || targetSampleRate != AUDIO_SAMPLE_RATE)
	{
		outError= "profile analyzes " + std::to_string(sampleCount) + " samples at " + std::to_string(targetSampleRate) +
				  " Hz, the stage runs " + std::to_string(LIPSYNC_BLOCK_SAMPLES) + " at " +
				  std::to_string(AUDIO_SAMPLE_RATE);
		return false;
	}
	if (melFilterBankChannels <= MFCC_COEFF_COUNT || melFilterBankChannels > LIPSYNC_BLOCK_SAMPLES / 2)
	{
		outError= "profile has " + std::to_string(melFilterBankChannels) + " mel channels";
		return false;
	}
	if (compareMethod != eCompareMethod::L1Norm && compareMethod != eCompareMethod::L2Norm &&
		compareMethod != eCompareMethod::CosineSimilarity)
	{
		outError= "profile names compare method " + std::to_string((int)compareMethod);
		return false;
	}
	if (mfccDataCount < 1)
		mfccDataCount= 1;

	const nlohmann::json mfccs= root.value("mfccs", nlohmann::json::array());
	if (mfccs.is_array())
	{
		for (const nlohmann::json& entry : mfccs)
		{
			if (!entry.is_object())
				continue;
			std::vector<MfccVector> frames;
			const nlohmann::json list= entry.value("mfccCalibrationDataList", nlohmann::json::array());
			if (list.is_array())
			{
				for (const nlohmann::json& item : list)
				{
					const nlohmann::json array= item.is_object() ? item.value("array", nlohmann::json()) : item;
					if (!array.is_array() || (int)array.size() != MFCC_COEFF_COUNT)
						continue;
					MfccVector frame{};
					for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
						frame[(size_t)index]= array[(size_t)index].is_number() ? array[(size_t)index].get<float>() : 0.f;
					frames.push_back(frame);
				}
			}
			addPhoneme(entry.value("name", std::string()), frames);
		}
	}
	finalize();
	outError.clear();
	return true;
}

bool LipSyncProfile::save(const std::filesystem::path& path, std::string& outError) const
{
	nlohmann::json root;
	root["jsonPath"]= jsonPath;
	root["mfccNum"]= mfccNum;
	root["mfccDataCount"]= mfccDataCount;
	root["melFilterBankChannels"]= melFilterBankChannels;
	root["targetSampleRate"]= targetSampleRate;
	root["sampleCount"]= sampleCount;
	root["useStandardization"]= useStandardization;
	root["compareMethod"]= (int)compareMethod;
	nlohmann::json mfccs= nlohmann::json::array();
	for (const LipSyncPhoneme& phoneme : phonemes)
	{
		nlohmann::json list= nlohmann::json::array();
		for (const MfccVector& frame : phoneme.frames)
		{
			nlohmann::json item;
			item["array"]= frame;
			list.push_back(item);
		}
		nlohmann::json entry;
		entry["name"]= phoneme.name;
		entry["mfccCalibrationDataList"]= list;
		mfccs.push_back(entry);
	}
	root["mfccs"]= mfccs;

	std::ofstream file(path, std::ios::binary | std::ios::trunc);
	if (!file)
	{
		outError= "cannot write profile: " + PathUtils::pathToUtf8(path);
		return false;
	}
	file << root.dump(2);
	if (!file)
	{
		outError= "cannot write profile: " + PathUtils::pathToUtf8(path);
		return false;
	}
	outError.clear();
	return true;
}

void LipSyncProfile::addPhoneme(const std::string& name, const std::vector<MfccVector>& frames)
{
	LipSyncPhoneme phoneme;
	phoneme.name= name;
	phoneme.frames= frames;
	phoneme.viseme= visemeForPhonemeName(name);
	phonemes.push_back(std::move(phoneme));
}

void LipSyncProfile::finalize()
{
	const size_t keep= (size_t)std::max(mfccDataCount, 1);
	for (LipSyncPhoneme& phoneme : phonemes)
	{
		if (phoneme.frames.size() > keep)
			phoneme.frames.erase(phoneme.frames.begin(), phoneme.frames.end() - (std::ptrdiff_t)keep);
		phoneme.viseme= visemeForPhonemeName(phoneme.name);
		phoneme.mean.fill(0.f);
		if (phoneme.frames.empty())
			continue;
		for (const MfccVector& frame : phoneme.frames)
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
				phoneme.mean[(size_t)index]+= frame[(size_t)index];
		for (float& value : phoneme.mean)
			value/= (float)phoneme.frames.size();
	}

	// uLipSync pools every kept frame of every phoneme for the mean and the
	// population deviation; a coefficient with no spread (or no frames at
	// all) keeps a deviation of one rather than dividing by zero
	means.fill(0.f);
	stdDevs.fill(1.f);
	if (!useStandardization)
		return;
	size_t frameCount= 0;
	for (const LipSyncPhoneme& phoneme : phonemes)
	{
		for (const MfccVector& frame : phoneme.frames)
		{
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
				means[(size_t)index]+= frame[(size_t)index];
			++frameCount;
		}
	}
	if (frameCount == 0)
		return;
	for (float& value : means)
		value/= (float)frameCount;
	MfccVector variance{};
	for (const LipSyncPhoneme& phoneme : phonemes)
		for (const MfccVector& frame : phoneme.frames)
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
			{
				const float diff= frame[(size_t)index] - means[(size_t)index];
				variance[(size_t)index]+= diff * diff;
			}
	for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
	{
		const float deviation= std::sqrt(variance[(size_t)index] / (float)frameCount);
		stdDevs[(size_t)index]= deviation > 1e-6f ? deviation : 1.f;
	}
}

float LipSyncProfile::phonemeScore(const LipSyncPhoneme& phoneme, const MfccVector& mfcc) const
{
	constexpr int n= MFCC_COEFF_COUNT;
	switch (compareMethod)
	{
	case eCompareMethod::L1Norm:
	{
		float distance= 0.f;
		for (int index= 0; index < n; ++index)
		{
			const float x= (mfcc[(size_t)index] - means[(size_t)index]) / stdDevs[(size_t)index];
			const float y= (phoneme.mean[(size_t)index] - means[(size_t)index]) / stdDevs[(size_t)index];
			distance+= std::fabs(x - y);
		}
		return std::pow(10.f, -distance / n);
	}
	case eCompareMethod::CosineSimilarity:
	{
		float xNorm= 0.f;
		float yNorm= 0.f;
		float product= 0.f;
		for (int index= 0; index < n; ++index)
		{
			const float x= (mfcc[(size_t)index] - means[(size_t)index]) / stdDevs[(size_t)index];
			const float y= (phoneme.mean[(size_t)index] - means[(size_t)index]) / stdDevs[(size_t)index];
			xNorm+= x * x;
			yNorm+= y * y;
			product+= x * y;
		}
		const float norms= std::sqrt(xNorm) * std::sqrt(yNorm);
		const float similarity= norms > 0.f ? std::max(product / norms, 0.f) : 0.f;
		return std::pow(similarity, k_cosinePower);
	}
	case eCompareMethod::L2Norm:
	default:
	{
		float distance= 0.f;
		for (int index= 0; index < n; ++index)
		{
			const float x= (mfcc[(size_t)index] - means[(size_t)index]) / stdDevs[(size_t)index];
			const float y= (phoneme.mean[(size_t)index] - means[(size_t)index]) / stdDevs[(size_t)index];
			distance+= (x - y) * (x - y);
		}
		return std::pow(10.f, -std::sqrt(distance / n));
	}
	}
}

void LipSyncProfile::score(const MfccVector& mfcc, LipSyncScores& outScores) const
{
	outScores.perPhoneme.assign(phonemes.size(), 0.f);
	outScores.perViseme.fill(0.f);
	outScores.mainViseme= VISEME_SIL;

	float sum= 0.f;
	for (size_t index= 0; index < phonemes.size(); ++index)
	{
		const float value= phonemeScore(phonemes[index], mfcc);
		outScores.perPhoneme[index]= value;
		sum+= value;
	}
	if (sum <= 0.f)
		return;

	// The first of the best phonemes wins, as uLipSync's GetVowel picks it
	size_t best= 0;
	for (size_t index= 0; index < phonemes.size(); ++index)
	{
		float& value= outScores.perPhoneme[index];
		value/= sum;
		if (value > outScores.perPhoneme[best])
			best= index;
		const int viseme= phonemes[index].viseme >= 0 ? phonemes[index].viseme : VISEME_SIL;
		outScores.perViseme[(size_t)viseme]+= value;
	}
	outScores.mainViseme= phonemes[best].viseme >= 0 ? phonemes[best].viseme : VISEME_SIL;
}

void calibrationFrames(const float* samples, size_t sampleCount, float gateDbfs, MfccExtractor& extractor,
					   std::vector<MfccVector>& outFrames)
{
	outFrames.clear();
	const int frameCount= mouthFramesForSamples(sampleCount);
	std::array<float, LIPSYNC_BLOCK_SAMPLES> block{};
	MfccVector mfcc;
	float levelDbfs= -100.f;
	for (int frameIndex= 0; frameIndex < frameCount; ++frameIndex)
	{
		const size_t end= std::min(mouthFrameEndSample(frameIndex), sampleCount);
		if (end < (size_t)LIPSYNC_BLOCK_SAMPLES)
			continue;
		std::copy(samples + end - LIPSYNC_BLOCK_SAMPLES, samples + end, block.begin());
		extractor.compute(block.data(), levelDbfs, mfcc);
		if (gateDbfs > -100.f && levelDbfs < gateDbfs)
			continue;
		outFrames.push_back(mfcc);
	}
}
