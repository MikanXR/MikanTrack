#include "TestCommon.h"

#include <limits>

#include "AppConfig.h"
#include "AudioRecorder.h"
#include "AudioService.h"
#include "AudioTypes.h"
#include "LoudnessBackend.h"
#include "MfccBackend.h"
#include "MfccFeatures.h"
#include "MouthBackend.h"
#include "MouthChain.h"
#include "MouthFilter.h"
#include "MouthReplay.h"
#include "LipSyncProfile.h"
#include "MouthWindow.h"
#include "OvrLipSyncBackend.h"
#include "PathUtils.h"
#include "VisemeTable.h"
#include "VisemeTypes.h"
#include "WavFile.h"

// Tests for the audio stage's pieces: the mouth columns and the frame
// accounting, the rolling-window policy, the wav reader and resampler, the
// emission filter, the MFCC chain against uLipSync's, the profile, the
// viseme table, the backends and the model selector, the level gate, the
// config, the wav writer and the sidecar recorder, the pacer, the chain and
// its replay, the calibration frames, and the Oculus LipSync engine where
// the build carries it.

namespace
{
void appendU32(std::vector<uint8_t>& bytes, uint32_t value)
{
	for (int shift= 0; shift < 32; shift+= 8)
		bytes.push_back((uint8_t)(value >> shift));
}

void appendU16(std::vector<uint8_t>& bytes, uint16_t value)
{
	bytes.push_back((uint8_t)value);
	bytes.push_back((uint8_t)(value >> 8));
}

// A minimal RIFF wave with the given format tag and the payload bytes
std::vector<uint8_t> buildWav(uint16_t formatTag, uint16_t channels, uint32_t sampleRate, uint16_t bitsPerSample,
							  const std::vector<uint8_t>& payload)
{
	std::vector<uint8_t> bytes;
	const uint32_t dataSize= (uint32_t)payload.size();
	bytes.insert(bytes.end(), {'R', 'I', 'F', 'F'});
	appendU32(bytes, 36 + dataSize);
	bytes.insert(bytes.end(), {'W', 'A', 'V', 'E'});
	bytes.insert(bytes.end(), {'f', 'm', 't', ' '});
	appendU32(bytes, 16);
	appendU16(bytes, formatTag);
	appendU16(bytes, channels);
	appendU32(bytes, sampleRate);
	appendU32(bytes, sampleRate * channels * bitsPerSample / 8);
	appendU16(bytes, (uint16_t)(channels * bitsPerSample / 8));
	appendU16(bytes, bitsPerSample);
	bytes.insert(bytes.end(), {'d', 'a', 't', 'a'});
	appendU32(bytes, dataSize);
	bytes.insert(bytes.end(), payload.begin(), payload.end());
	return bytes;
}

bool writeBytes(const std::filesystem::path& path, const std::vector<uint8_t>& bytes)
{
	std::ofstream file(path, std::ios::binary);
	if (!file)
		return false;
	file.write((const char*)bytes.data(), (std::streamsize)bytes.size());
	return (bool)file;
}

float rms(const std::vector<float>& samples)
{
	double sum= 0.0;
	for (float sample : samples)
		sum+= (double)sample * sample;
	return samples.empty() ? 0.f : (float)std::sqrt(sum / (double)samples.size());
}

// A backend that opens aa fully on every frame and counts what the chain
// asks of it
struct CountingBackend : IMouthBackend
{
	bool bRunsWhileGated= false;
	int inferCalls= 0;
	std::vector<int> newFrameCounts;
	std::string reason;

	bool isReady() const override { return true; }
	std::string getInfo() const override { return "counting"; }
	const std::string& getReason() const override { return reason; }
	bool runsWhileGated() const override { return bRunsWhileGated; }
	bool infer(const float*, size_t sampleCount, int newFrameCount, std::vector<MouthFrame>& outFrames) override
	{
		++inferCalls;
		newFrameCounts.push_back(newFrameCount);
		outFrames.assign((size_t)mouthFramesForSamples(sampleCount), MouthFrame());
		for (MouthFrame& frame : outFrames)
			frame.weights[(int)eViseme::aa]= 1.f;
		return true;
	}
	double getLastInferMs() const override { return 0.0; }
};
} // namespace

static int runAudioTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO("test-audio") << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR("test-audio") << "FAIL " << name;
			failures++;
		}
	};

	// (a) The mouth columns and the frame accounting
	{
		int mouthCount= 0;
		bool bMouthOnly= true;
		for (int arkitIndex= 0; arkitIndex < ARKIT_BLENDSHAPE_COUNT; ++arkitIndex)
		{
			if (!isMouthArkitIndex(arkitIndex))
				continue;
			++mouthCount;
			const std::string name= arkitBlendshapeName(arkitIndex);
			bMouthOnly&= name.rfind("jaw", 0) == 0 || name.rfind("mouth", 0) == 0 || name == "cheekPuff" ||
						 name == "tongueOut";
		}
		check(mouthCount == 29 && bMouthOnly, "the 29 mouth columns are the jaw, mouth, cheekPuff, and tongueOut shapes");
		check(!isMouthArkitIndex(arkitBlendshapeFromName("eyeBlinkLeft")) &&
				  !isMouthArkitIndex(arkitBlendshapeFromName("browInnerUp")) &&
				  !isMouthArkitIndex(arkitBlendshapeFromName("cheekSquintLeft")) &&
				  !isMouthArkitIndex(arkitBlendshapeFromName("noseSneerRight")),
			  "eyes, brows, cheek squints, and nose are excluded");
		check(isMouthArkitIndex(arkitBlendshapeFromName("jawOpen")) &&
				  isMouthArkitIndex(arkitBlendshapeFromName("mouthPucker")) &&
				  isMouthArkitIndex(arkitBlendshapeFromName("cheekPuff")) &&
				  isMouthArkitIndex(arkitBlendshapeFromName("tongueOut")),
			  "jawOpen, mouthPucker, cheekPuff, and tongueOut are included");
		check(!isMouthArkitIndex(-1) && !isMouthArkitIndex(ARKIT_BLENDSHAPE_COUNT), "out-of-range indices are not mouth columns");

		VisemeWeights weights{};
		check(dominantViseme(weights) == VISEME_SIL, "no weight at all is silence");
		weights[(int)eViseme::U]= 0.4f;
		weights[(int)eViseme::O]= 0.6f;
		weights[VISEME_SIL]= 1.f;
		check(dominantViseme(weights) == (int)eViseme::O, "the dominant viseme is the heaviest one other than silence");

		check(mouthFramesForSamples(16000) == 30 && mouthFramesForSamples(32000) == 60 &&
				  mouthFramesForSamples(1600) == 3 && mouthFramesForSamples(1) == 1 && mouthFramesForSamples(0) == 0,
			  "frames per samples follow ceil(30 * n / 16000)");
	}

	// (c) The window policy
	{
		MouthWindowConfig config;
		config.windowSamples= 32000;
		config.hopSamples= 1600;
		config.lookaheadFrames= 0;
		MouthWindow window;
		window.configure(config);

		// A ramp so sample positions can be read back from values
		std::vector<float> ramp(48000);
		for (size_t index= 0; index < ramp.size(); ++index)
			ramp[index]= (float)index;

		check(!window.isRunDue(), "no run is due before any audio");
		window.push(ramp.data(), 1599);
		check(!window.isRunDue(), "a hop less one sample is not due");
		window.push(ramp.data() + 1599, 1);
		check(window.isRunDue(), "one hop is due");

		std::vector<float> windowSamples;
		int emitCount= window.beginRun(windowSamples);
		check(emitCount == 3, "the first hop emits three frames");
		check(windowSamples.size() == 32000, "the window is always windowSamples long");
		bool bPadded= true;
		for (size_t index= 0; index < 32000 - 1600; ++index)
			bPadded&= windowSamples[index] == 0.f;
		check(bPadded && windowSamples[32000 - 1600] == 0.f && windowSamples[31999] == 1599.f,
			  "the first window is zero-padded at the front with the hop at its tail");
		check(!window.isRunDue() && window.getConsumedSamples() == 1600 && window.getEmittedFrames() == 3,
			  "the hop is consumed and the counters advance");

		// Nine more hops: the clip's first second
		int totalEmitted= emitCount;
		for (int hop= 1; hop < 10; ++hop)
		{
			window.push(ramp.data() + hop * 1600, 1600);
			totalEmitted+= window.beginRun(windowSamples);
		}
		check(totalEmitted == 30 && window.getEmittedFrames() == 30, "ten hops emit thirty frames");

		// Past the window length the window is exactly the newest samples
		window.push(ramp.data() + 16000, 32000);
		int runs= 0;
		while (window.isRunDue())
		{
			window.beginRun(windowSamples);
			runs++;
		}
		check(runs == 20, "a push of twenty hops makes twenty runs due");
		check(windowSamples.front() == 16000.f && windowSamples.back() == 47999.f,
			  "after the window fills it holds exactly the newest windowSamples");
		check(window.getConsumedSamples() == 48000 && window.getEmittedFrames() == 90 && window.getPendingSamples() == 0,
			  "counters cover the whole push");

		// An odd hop never skips or repeats a frame
		MouthWindowConfig oddConfig;
		oddConfig.windowSamples= 8000;
		oddConfig.hopSamples= 1000;
		oddConfig.lookaheadFrames= 0;
		MouthWindow oddWindow;
		oddWindow.configure(oddConfig);
		std::vector<int> emits;
		for (int hop= 0; hop < 16; ++hop)
		{
			oddWindow.push(ramp.data() + hop * 1000, 1000);
			emits.push_back(oddWindow.beginRun(windowSamples));
		}
		int oddTotal= 0;
		bool bNonNegative= true;
		for (int count : emits)
		{
			oddTotal+= count;
			bNonNegative&= count >= 0;
		}
		check(bNonNegative && oddTotal == 30 && emits[0] == 1 && emits[1] == 2,
			  "a 1000-sample hop alternates one and two frames and sums to thirty per second");

		// takeTail
		std::vector<MouthFrame> windowFrames(60);
		for (size_t index= 0; index < windowFrames.size(); ++index)
			windowFrames[index].weights[0]= (float)index;
		std::vector<MouthFrame> tail;
		MouthWindow::takeTail(windowFrames, 3, 0, tail);
		check(tail.size() == 3 && tail[0].weights[0] == 57.f && tail[2].weights[0] == 59.f,
			  "takeTail copies the last frames in order");
		MouthWindow::takeTail(windowFrames, 100, 0, tail);
		check(tail.size() == 63, "takeTail clamps to the frames available and appends");
		MouthWindow::takeTail(windowFrames, -1, 0, tail);
		check(tail.size() == 63, "a negative count takes nothing");
		tail.clear();
		MouthWindow::takeTail(windowFrames, 3, 2, tail);
		check(tail.size() == 3 && tail[0].weights[0] == 55.f && tail[2].weights[0] == 57.f,
			  "a lookahead holds the last frames back");

		// With a lookahead the emitted frames trail the frames due and the
		// first run never emits the padding
		MouthWindowConfig lookConfig;
		lookConfig.windowSamples= 32000;
		lookConfig.hopSamples= 1600;
		lookConfig.lookaheadFrames= 2;
		MouthWindow lookWindow;
		lookWindow.configure(lookConfig);
		lookWindow.push(ramp.data(), 1600);
		const int firstEmit= lookWindow.beginRun(windowSamples);
		lookWindow.push(ramp.data() + 1600, 1600);
		const int secondEmit= lookWindow.beginRun(windowSamples);
		check(firstEmit == 1 && secondEmit == 3 && lookWindow.getEmittedFrames() == 4,
			  "a two-frame lookahead emits one frame on the first hop and three after");

		window.reset();
		check(window.getConsumedSamples() == 0 && window.getEmittedFrames() == 0 && !window.isRunDue(),
			  "reset clears the counters and the samples");

		const MouthWindowConfig defaults;
		check(defaults.windowSamples == 2 * (AUDIO_SAMPLE_RATE / 10) && defaults.hopSamples == AUDIO_SAMPLE_RATE / 10 &&
				  defaults.lookaheadFrames == 0 && defaults.windowSamples >= LIPSYNC_BLOCK_SAMPLES + defaults.hopSamples,
			  "the defaults are a two-hop window on the frame grid, a 100 ms hop, and no lookahead");
	}

	// (d) The wav reader and the resampler
	{
		const std::filesystem::path tempDir= std::filesystem::temp_directory_path() / "mikan_audio_test";
		std::filesystem::create_directories(tempDir);

		// A 1 kHz tone, stereo 16-bit at 48 kHz, half a second
		const int sourceRate= 48000;
		const size_t frameCount= 24000;
		std::vector<uint8_t> pcm;
		std::vector<float> reference(frameCount);
		for (size_t frame= 0; frame < frameCount; ++frame)
		{
			const float value= 0.5f * std::sin(2.f * glm::pi<float>() * 1000.f * (float)frame / (float)sourceRate);
			reference[frame]= value;
			const int16_t quantized= (int16_t)std::lround(value * 32767.f);
			for (int channel= 0; channel < 2; ++channel)
				appendU16(pcm, (uint16_t)quantized);
		}
		const std::filesystem::path pcmPath= tempDir / "tone_pcm16.wav";
		check(writeBytes(pcmPath, buildWav(1, 2, sourceRate, 16, pcm)), "test wav written");

		WavData data;
		std::string error;
		check(WavFile::read(pcmPath, data, error), "PCM16 stereo file reads");
		check(data.channels == 2 && data.sampleRate == sourceRate && data.frameCount() == frameCount,
			  "channels, rate, and frame count are read");
		const std::vector<float> mono= WavFile::toMono(data);
		check(mono.size() == frameCount && std::fabs(mono[6] - reference[6]) < 1e-3f, "mono mix keeps the values");

		std::vector<float> resampled;
		check(WavFile::resample(mono, sourceRate, AUDIO_SAMPLE_RATE, resampled, error), "resample to 16 kHz runs");
		check(std::llabs((long long)resampled.size() - (long long)(frameCount / 3)) <= 4,
			  "resampled length is a third of the source");
		const float sourceRms= rms(mono);
		const float resampledRms= rms(resampled);
		MIKAN_LOG_INFO("test-audio") << "tone rms source " << sourceRms << " resampled " << resampledRms;
		check(std::fabs(resampledRms - sourceRms) < 0.1f * sourceRms, "a 1 kHz tone survives resampling");

		std::vector<float> same;
		check(WavFile::resample(mono, sourceRate, sourceRate, same, error) && same == mono,
			  "a matching rate copies");

		// A float32 mono file at 16 kHz through the one-call path
		std::vector<uint8_t> floatBytes(resampled.size() * 4);
		std::memcpy(floatBytes.data(), resampled.data(), floatBytes.size());
		const std::filesystem::path floatPath= tempDir / "tone_float.wav";
		writeBytes(floatPath, buildWav(3, 1, AUDIO_SAMPLE_RATE, 32, floatBytes));
		std::vector<float> clip;
		check(WavFile::readMono16k(floatPath, clip, error) && clip.size() == resampled.size() &&
				  std::fabs(clip[100] - resampled[100]) < 1e-6f,
			  "float32 file reads through readMono16k unchanged");

		const std::filesystem::path badPath= tempDir / "bad.wav";
		writeBytes(badPath, {'R', 'I', 'F', 'X', 0, 0, 0, 0, 'W', 'A', 'V', 'E'});
		check(!WavFile::read(badPath, data, error) && !error.empty(), "a non-RIFF file errors");
		check(!WavFile::read(tempDir / "missing.wav", data, error), "a missing file errors");

		std::filesystem::remove_all(tempDir);
	}

	// (e) The emission filter
	{
		MouthFilter filter;
		filter.configure(MouthFilterConfig());
		MouthFrame constant;
		constant.weights.fill(0.4f);
		MouthFrame out= filter.filter(constant);
		bool bHeld= true;
		for (int step= 0; step < 10; ++step)
		{
			out= filter.filter(constant);
			for (float weight : out.weights)
				bHeld&= std::fabs(weight - 0.4f) < 1e-6f;
		}
		check(bHeld, "a constant input passes through the filter unchanged");

		MouthFrame stepUp;
		stepUp.weights.fill(1.f);
		out= filter.filter(stepUp);
		check(out.weights[0] > 0.4f && out.weights[0] < 1.f, "a step is approached, not jumped to");
		for (int step= 0; step < 30; ++step)
			out= filter.filter(stepUp);
		check(out.weights[0] > 0.98f, "a held step converges within a second");

		filter.reset();
		out= filter.filter(constant);
		check(std::fabs(out.weights[0] - 0.4f) < 1e-6f, "reset restarts from the next value");
	}

	// (g) The MFCC chain against uLipSync's Algorithm.cs run in Python on
	// the same signal: a 150 Hz buzz under LCG noise, one second at 16 kHz,
	// the 1024-sample block ending at a frame's end
	{
		std::vector<float> signal(16000);
		uint32_t seed= 12345;
		for (size_t index= 0; index < signal.size(); ++index)
		{
			seed= seed * 1664525u + 1013904223u;
			const double noise= seed / 4294967296.0 - 0.5;
			double buzz= 0.0;
			for (int harmonic= 1; harmonic <= 6; ++harmonic)
				buzz+= std::sin(2.0 * glm::pi<double>() * 150.0 * harmonic * (double)index / 16000.0 + harmonic) / harmonic;
			signal[index]= (float)(0.03 * buzz + 0.2 * noise);
		}

		struct Reference
		{
			int frame;
			float levelDbfs;
			std::array<float, MFCC_COEFF_COUNT> mfcc;
		};
		const Reference references[]= {
			{5, -23.9000f, {-86.0547f, -13.8630f, -12.0956f, -3.4764f, -7.3946f, -5.5722f, -1.2879f, -6.6664f, -5.9787f, -4.0328f, -6.8480f, -3.9221f}},
			{15, -23.9107f, {-81.8679f, -13.3102f, -12.2050f, -6.5073f, -6.9149f, -7.1728f, -3.5939f, -0.8741f, -2.3028f, -8.5764f, -5.9658f, -2.4682f}},
			{25, -23.8512f, {-83.3203f, -17.6725f, -16.8212f, -8.2013f, -7.9032f, -5.7109f, -3.3506f, -6.5332f, -4.8773f, -3.1051f, -3.1589f, -0.7612f}},
		};
		MfccExtractor extractor;
		check(extractor.getMelChannels() == LIPSYNC_DEFAULT_MEL_CHANNELS && LIPSYNC_DEFAULT_MEL_CHANNELS == 30,
			  "the default bank is uLipSync's 30 mel channels");
		for (const Reference& reference : references)
		{
			const size_t blockEnd= mouthFrameEndSample(reference.frame);
			float levelDbfs= 0.f;
			MfccVector mfcc;
			extractor.compute(signal.data() + blockEnd - LIPSYNC_BLOCK_SAMPLES, levelDbfs, mfcc);
			float worst= std::fabs(levelDbfs - reference.levelDbfs);
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
				worst= std::max(worst, std::fabs(mfcc[index] - reference.mfcc[index]));
			const std::string label= "frame " + std::to_string(reference.frame) +
									 " matches uLipSync's level and coefficients (worst " + std::to_string(worst) + ")";
			check(worst < 0.05f, label.c_str());
		}

		// The peak normalization: a block a tenth as loud gives the same
		// coefficients and a level 20 dB lower
		{
			const size_t blockEnd= mouthFrameEndSample(15);
			std::array<float, LIPSYNC_BLOCK_SAMPLES> quiet;
			for (int index= 0; index < LIPSYNC_BLOCK_SAMPLES; ++index)
				quiet[index]= 0.1f * signal[blockEnd - LIPSYNC_BLOCK_SAMPLES + index];
			float levelDbfs= 0.f;
			MfccVector mfcc;
			extractor.compute(quiet.data(), levelDbfs, mfcc);
			float worst= 0.f;
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
				worst= std::max(worst, std::fabs(mfcc[index] - references[1].mfcc[index]));
			check(worst < 0.05f && std::fabs(levelDbfs - (references[1].levelDbfs - 20.f)) < 0.01f,
				  "the coefficients are level-free and the level reads the block's RMS");
		}
		check(mouthFrameEndSample(0) == 533 && mouthFrameEndSample(2) == 1600 && mouthFrameEndSample(29) == 16000,
			  "a frame ends at round((f + 1) * 16000 / 30)");
	}

	// (h) The profile: uLipSync's JSON round-trips, a load trims each phoneme
	// to the data count and takes the means, the names map, the pooled
	// statistics standardize, and a block scores against the templates
	{
		const std::filesystem::path tempDir= std::filesystem::temp_directory_path() / "mikan_audio_test";
		std::filesystem::create_directories(tempDir);
		const auto ramp= [](float offset) {
			MfccVector frame;
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
				frame[index]= offset + (float)index;
			return frame;
		};

		LipSyncProfile profile;
		profile.mfccDataCount= 3;
		profile.addPhoneme("A", {ramp(1.f), ramp(2.f), ramp(3.f), ramp(4.f), ramp(5.f)});
		profile.addPhoneme("-", {ramp(-20.f), ramp(-22.f)});
		profile.addPhoneme("xx", {ramp(50.f)});
		profile.addPhoneme("e", {});
		profile.finalize();
		check(profile.phonemes.size() == 4 && profile.phonemes[0].frames.size() == 3 &&
				  std::fabs(profile.phonemes[0].mean[0] - 4.f) < 1e-6f && std::fabs(profile.phonemes[0].mean[11] - 15.f) < 1e-6f,
			  "finalize keeps the last mfccDataCount frames and means them");
		check(profile.phonemes[1].viseme == VISEME_SIL && profile.phonemes[0].viseme == (int)eViseme::aa &&
				  profile.phonemes[2].viseme == -1 && profile.phonemes[3].viseme == (int)eViseme::E &&
				  profile.phonemes[3].mean[5] == 0.f,
			  "the names map to visemes case-insensitively, an unknown name to none, and no frames mean zero");
		check(visemeForPhonemeName("U") == (int)eViseme::U && visemeForPhonemeName("oo") == (int)eViseme::U &&
				  visemeForPhonemeName("oh") == (int)eViseme::O && visemeForPhonemeName("ih") == (int)eViseme::I &&
				  visemeForPhonemeName("Silence") == VISEME_SIL && visemeForPhonemeName("N") == VISEME_SIL,
			  "uLipSync's names and the wizard's old names both map");

		const std::filesystem::path profilePath= tempDir / "profile.json";
		std::string error;
		check(profile.save(profilePath, error), "the profile writes");
		{
			std::ifstream file(profilePath);
			const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
			check(text.find("\"mfccCalibrationDataList\"") != std::string::npos && text.find("\"compareMethod\": 1") != std::string::npos &&
					  text.find("\"melFilterBankChannels\": 30") != std::string::npos && text.find("\"name\": \"A\"") != std::string::npos,
				  "the file carries uLipSync's field names");
		}
		LipSyncProfile loaded;
		check(loaded.load(profilePath, error) && loaded.phonemes.size() == 4 && loaded.mfccDataCount == 3 &&
				  loaded.compareMethod == eCompareMethod::L2Norm && !loaded.useStandardization &&
				  loaded.phonemes[0].frames.size() == 3 && loaded.phonemes[0].mean == profile.phonemes[0].mean &&
				  loaded.phonemes[1].mean == profile.phonemes[1].mean && loaded.phonemes[1].viseme == VISEME_SIL,
			  "the profile loads back with the same templates");

		// A uLipSync export by hand, compact as JsonUtility writes it, with
		// standardization and the cosine method on
		{
			const char* exported=
				"{\"jsonPath\":\"\",\"mfccNum\":12,\"mfccDataCount\":16,\"melFilterBankChannels\":30,\"targetSampleRate\":16000,"
				"\"sampleCount\":1024,\"useStandardization\":true,\"compareMethod\":2,\"mfccs\":["
				"{\"name\":\"A\",\"mfccCalibrationDataList\":[{\"array\":[1,2,3,4,5,6,7,8,9,10,11,12]},{\"array\":[3,4,5,6,7,8,9,10,11,12,13,14]}]},"
				"{\"name\":\"I\",\"mfccCalibrationDataList\":[{\"array\":[0,0,0,0,0,0,0,0,0,0,0,0]}]}]}";
			const std::filesystem::path exportPath= tempDir / "ulipsync_export.json";
			{
				std::ofstream file(exportPath);
				file << exported;
			}
			LipSyncProfile imported;
			check(imported.load(exportPath, error) && imported.useStandardization &&
					  imported.compareMethod == eCompareMethod::CosineSimilarity && imported.phonemes.size() == 2 &&
					  imported.phonemes[0].frames.size() == 2 && std::fabs(imported.phonemes[0].mean[0] - 2.f) < 1e-6f,
				  "a uLipSync export loads with its switches");
			// Pooled over the three frames: mean 4/3, population deviation
			// sqrt(42/27) on the first coefficient
			check(std::fabs(imported.means[0] - 4.f / 3.f) < 1e-5f && std::fabs(imported.stdDevs[0] - std::sqrt(42.f / 27.f)) < 1e-5f,
				  "standardization pools every kept frame for the mean and the population deviation");
		}

		// Scoring: a block on a template is that phoneme's, the scores sum
		// to one, and the fold lands on the template's viseme
		{
			LipSyncProfile two;
			two.addPhoneme("A", {ramp(0.f)});
			two.addPhoneme("I", {ramp(30.f)});
			two.addPhoneme("-", {ramp(-30.f)});
			two.finalize();
			LipSyncScores scores;
			two.score(ramp(0.f), scores);
			float sum= 0.f;
			for (float value : scores.perPhoneme)
				sum+= value;
			check(scores.perPhoneme.size() == 3 && std::fabs(sum - 1.f) < 1e-5f && scores.perPhoneme[0] > 0.99f &&
					  scores.mainViseme == (int)eViseme::aa && std::fabs(scores.perViseme[(int)eViseme::aa] - scores.perPhoneme[0]) < 1e-6f,
				  "a block on the A template scores as A, the shares summing to one");
			two.score(ramp(15.f), scores);
			check(std::fabs(scores.perPhoneme[0] - scores.perPhoneme[1]) < 1e-5f && scores.perPhoneme[2] < 1e-6f,
				  "a block halfway between two templates splits between them");
			two.compareMethod= eCompareMethod::L1Norm;
			two.score(ramp(29.f), scores);
			check(scores.mainViseme == (int)eViseme::I, "the L1 method calls the nearer template");
			two.compareMethod= eCompareMethod::CosineSimilarity;
			two.score(ramp(0.f), scores);
			check(scores.mainViseme == (int)eViseme::aa, "the cosine method calls the aligned template");
		}

		LipSyncProfile missing;
		check(!missing.load(tempDir / "does_not_exist.json", error) && error.find("missing") != std::string::npos,
			  "a missing profile reports the reason");
		{
			const std::filesystem::path brokenPath= tempDir / "broken.json";
			{
				std::ofstream file(brokenPath);
				file << "{\"mfccNum\": 13}";
			}
			check(!missing.load(brokenPath, error) && error.find("coefficients") != std::string::npos,
				  "a profile with another coefficient count is refused");
			{
				std::ofstream file(brokenPath);
				file << "not json";
			}
			check(!missing.load(brokenPath, error) && error.find("not JSON") != std::string::npos,
				  "a file that is not JSON reports the reason");
		}
	}

	// (i) The viseme table
	{
		bool bRowsValid= true;
		for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		{
			const std::vector<VisemeTerm>& row= visemeTableRow(viseme);
			if (viseme == VISEME_SIL)
				bRowsValid&= row.empty();
			else
				bRowsValid&= !row.empty();
			for (const VisemeTerm& term : row)
				bRowsValid&= isMouthArkitIndex(term.arkitIndex) && term.weight > 0.f && term.weight <= 1.f;
		}
		check(bRowsValid, "every viseme but silence drives at least one mouth column with a weight in 0..1");
		check(std::string(visemeName((int)eViseme::aa)) == "aa" && std::string(visemeName(VISEME_SIL)) == "sil" &&
				  std::string(visemeName(15)).empty(),
			  "the viseme names follow the id order");

		const int jawOpen= arkitBlendshapeFromName("jawOpen");
		const int mouthPucker= arkitBlendshapeFromName("mouthPucker");
		const int eyeBlinkLeft= arkitBlendshapeFromName("eyeBlinkLeft");
		VisemeWeights weights{};
		weights[(int)eViseme::aa]= 1.f;
		std::array<float, ARKIT_BLENDSHAPE_COUNT> face{};
		face[mouthPucker]= 0.7f;
		face[eyeBlinkLeft]= 0.9f;
		applyVisemeWeights(weights, face);
		check(std::fabs(face[jawOpen] - 0.85f) < 1e-6f, "a full aa opens the jaw to its row's weight");
		check(face[mouthPucker] == 0.f, "a column another viseme drives is overwritten, not accumulated");
		check(face[eyeBlinkLeft] == 0.9f, "a column the table does not drive is left alone");
		weights.fill(1.f);
		applyVisemeWeights(weights, face);
		bool bClamped= true;
		for (float weight : face)
			bClamped&= weight >= 0.f && weight <= 1.f;
		check(bClamped, "every viseme at once stays within 0..1");
		weights.fill(0.f);
		applyVisemeWeights(weights, face);
		check(face[jawOpen] == 0.f && face[mouthPucker] == 0.f, "no viseme is the neutral mouth");
	}

	// (j) The live stage's level and gate
	{
		std::vector<float> fullScale(1600, 1.f);
		check(std::fabs(audioLevelDbfs(fullScale.data(), fullScale.size())) < 1e-3f, "a full-scale constant is 0 dBFS");
		std::vector<float> tenth(1600);
		for (size_t index= 0; index < tenth.size(); ++index)
			tenth[index]= 0.1f * std::sin(2.f * glm::pi<float>() * 440.f * (float)index / AUDIO_SAMPLE_RATE);
		const float sineDbfs= audioLevelDbfs(tenth.data(), tenth.size());
		check(std::fabs(sineDbfs - (-23.01f)) < 0.2f, "a tenth-amplitude sine is -23 dBFS RMS");
		std::vector<float> silence(1600, 0.f);
		check(audioLevelDbfs(silence.data(), silence.size()) == -100.f && audioLevelDbfs(nullptr, 0) == -100.f,
			  "silence and nothing floor at -100 dBFS");

		LevelGate gate;
		gate.setThreshold(-45.f);
		check(!gate.isOpen() && !gate.update(-46.f), "the gate starts closed and stays under the threshold");
		check(gate.update(-45.f), "it opens at the threshold");
		check(gate.update(-50.f), "and stays open within the hysteresis under it");
		check(!gate.update(-51.5f), "and closes past the hysteresis");
		check(!gate.update(-46.f), "and does not reopen under the threshold");
	}

	// (k) The audio config round-trips through the project JSON
	{
		AppConfig config;
		config.audio.enabled= true;
		config.audio.deviceName= "USB Microphone";
		config.audio.gainDb= 6.f;
		config.audio.gateDbfs= -38.f;
		config.audio.profilePath= "audio/profile.json";
		AppConfig loaded;
		const bool bLoaded= loaded.loadFromJsonString(config.toJsonString());
		check(bLoaded && loaded.audio.enabled && loaded.audio.deviceName == "USB Microphone" &&
				  std::fabs(loaded.audio.gainDb - 6.f) < 1e-6f && std::fabs(loaded.audio.gateDbfs + 38.f) < 1e-6f &&
				  loaded.audio.profilePath == "audio/profile.json",
			  "the audio section round-trips through the project JSON");
		AppConfig fresh;
		check(fresh.loadFromJsonString("{}") && !fresh.audio.enabled &&
				  fresh.audio.deviceName.empty() && std::fabs(fresh.audio.gateDbfs + 45.f) < 1e-6f,
			  "a project without an audio section gets the defaults");
	}

	// (m) The backend on a window: volume-only without a profile, and a
	// profile of two phonemes taught on two signals calls each back
	{
		const std::filesystem::path tempDir= std::filesystem::temp_directory_path() / "mikan_audio_test";
		std::filesystem::create_directories(tempDir);
		const auto chord= [](float f0, float second, float third) {
			std::vector<float> tone(16000);
			for (size_t index= 0; index < tone.size(); ++index)
			{
				const float t= (float)index / AUDIO_SAMPLE_RATE;
				tone[index]= 0.3f * (std::sin(2.f * glm::pi<float>() * f0 * t) + 0.5f * std::sin(2.f * glm::pi<float>() * second * t) +
									 0.25f * std::sin(2.f * glm::pi<float>() * third * t));
			}
			return tone;
		};
		const std::vector<float> low= chord(150.f, 450.f, 900.f);
		const std::vector<float> high= chord(1200.f, 2900.f, 4100.f);
		const std::vector<float> silence(16000, 0.f);
		const auto vowelSum= [](const MouthFrame& frame) {
			return frame.weights[(int)eViseme::aa] + frame.weights[(int)eViseme::E] + frame.weights[(int)eViseme::I] +
				   frame.weights[(int)eViseme::O] + frame.weights[(int)eViseme::U];
		};

		MfccBackend backend;
		check(!backend.create("") && backend.isReady() && !backend.hasProfile() && backend.getReason().empty(),
			  "no profile is ready and volume-only");
		backend.setVolumeWindow(-45.f, -25.f);
		std::vector<MouthFrame> frames;
		check(backend.infer(low.data(), low.size(), mouthFramesForSamples(low.size()), frames) && frames.size() == 30 &&
				  backend.getLastFrameScores().size() == 30,
			  "a second of audio gives thirty frames");
		bool bInRange= true;
		bool bPartition= true;
		for (const MouthFrame& frame : frames)
		{
			for (float weight : frame.weights)
				bInRange&= weight >= 0.f && weight <= 1.f;
			bPartition&= std::fabs(vowelSum(frame) + frame.weights[VISEME_SIL] - 1.f) < 1e-5f;
		}
		check(bInRange, "every weight is within 0..1");
		check(bPartition, "the vowels and silence partition one");
		check(frames.back().weights[(int)eViseme::aa] == 1.f && frames.back().weights[VISEME_SIL] == 0.f &&
				  backend.getLastFrameScores().back().volume == 1.f,
			  "a loud tone over the window is all aa, volume-only");
		MIKAN_LOG_INFO("test-audio") << "volume-only inference " << backend.getLastInferMs() << " ms for one second";

		check(backend.infer(silence.data(), silence.size(), mouthFramesForSamples(silence.size()), frames) && frames.size() == 30 &&
				  frames.back().weights[(int)eViseme::aa] == 0.f && frames.back().weights[VISEME_SIL] == 1.f,
			  "digital silence is silence");
		check(!backend.infer(low.data(), 0, mouthFramesForSamples(0), frames) && frames.empty(), "nothing is refused");
		check(backend.infer(low.data(), 300, mouthFramesForSamples(300), frames) && frames.size() == 1, "less than a block is padded into its frame");

		// Two phonemes taught on the two chords and the noise phoneme on
		// silence, through the calibration frames and a saved profile
		{
			MfccExtractor extractor;
			std::vector<MfccVector> lowFrames;
			std::vector<MfccVector> highFrames;
			std::vector<MfccVector> silentFrames;
			calibrationFrames(low.data(), low.size(), -100.f, extractor, lowFrames);
			calibrationFrames(high.data(), high.size(), -100.f, extractor, highFrames);
			calibrationFrames(silence.data(), silence.size(), -100.f, extractor, silentFrames);
			LipSyncProfile profile;
			profile.mfccDataCount= (int)lowFrames.size();
			profile.addPhoneme("A", lowFrames);
			profile.addPhoneme("I", highFrames);
			profile.addPhoneme("-", silentFrames);
			profile.finalize();
			const std::filesystem::path profilePath= tempDir / "two_phonemes.json";
			std::string error;
			check(profile.save(profilePath, error), "the two-phoneme profile writes");

			MfccBackend taught;
			check(taught.create(PathUtils::pathToUtf8(profilePath)) && taught.hasProfile() && taught.getReason().empty() &&
					  taught.getProfile().phonemes.size() == 3,
				  "the backend loads the profile");
			taught.setVolumeWindow(-45.f, -25.f);
			check(taught.infer(low.data(), low.size(), mouthFramesForSamples(low.size()), frames) && frames.size() == 30, "the low chord runs");
			const MfccBackend::FrameScores& lowScore= taught.getLastFrameScores().back();
			check(lowScore.scores.mainViseme == (int)eViseme::aa && lowScore.scores.perViseme[(int)eViseme::aa] > 0.6f &&
					  std::fabs(frames.back().weights[(int)eViseme::aa] - lowScore.scores.perViseme[(int)eViseme::aa] * lowScore.volume) < 1e-6f,
				  "the low chord calls A, its weight the share times the volume");
			check(taught.infer(high.data(), high.size(), mouthFramesForSamples(high.size()), frames) && frames.size() == 30 &&
					  taught.getLastFrameScores().back().scores.mainViseme == (int)eViseme::I &&
					  frames.back().weights[(int)eViseme::I] > 0.6f,
				  "the high chord calls I");
			check(taught.infer(silence.data(), silence.size(), mouthFramesForSamples(silence.size()), frames) && frames.back().weights[VISEME_SIL] == 1.f,
				  "silence is silence with a profile too");
			MIKAN_LOG_INFO("test-audio") << "profile inference " << taught.getLastInferMs() << " ms for one second";
		}

		MfccBackend broken;
		check(!broken.create(PathUtils::pathToUtf8(tempDir / "does_not_exist.json")) && broken.isReady() && !broken.hasProfile() &&
				  broken.getReason().find("missing") != std::string::npos,
			  "a missing profile leaves the backend volume-only with the reason");

		// The loudness model: aa at the volume, silence the remainder, the
		// same level and window the volume-only MFCC model opens aa by
		{
			LoudnessBackend loudness;
			loudness.setVolumeWindow(-45.f, -25.f);
			check(loudness.isReady() && loudness.getInfo() == "loudness" && loudness.getReason().empty(),
				  "the loudness backend is ready");
			check(loudness.infer(low.data(), low.size(), mouthFramesForSamples(low.size()), frames) && frames.size() == 30,
				  "the loudness backend gives thirty frames for a second");
			bool bOpen= true;
			bool bRemainder= true;
			for (const MouthFrame& frame : frames)
			{
				const float aa= frame.weights[(int)eViseme::aa];
				float sum= 0.f;
				for (float weight : frame.weights)
					sum+= weight;
				bOpen&= aa > 0.f && aa <= 1.f;
				bRemainder&= std::fabs(frame.weights[VISEME_SIL] - (1.f - aa)) < 1e-6f && std::fabs(sum - 1.f) < 1e-6f;
			}
			check(bOpen, "the low chord opens aa on every frame");
			check(bRemainder, "silence is the remainder and the weights sum to one");

			MfccBackend volumeOnly;
			volumeOnly.create("");
			volumeOnly.setVolumeWindow(-45.f, -25.f);
			std::vector<MouthFrame> volumeFrames;
			volumeOnly.infer(low.data(), low.size(), mouthFramesForSamples(low.size()), volumeFrames);
			bool bSame= volumeFrames.size() == frames.size();
			for (size_t index= 0; bSame && index < frames.size(); ++index)
				bSame&= std::fabs(frames[index].weights[(int)eViseme::aa] - volumeFrames[index].weights[(int)eViseme::aa]) < 1e-5f;
			check(bSame, "frame for frame the loudness aa is the volume-only MFCC aa");

			check(loudness.infer(silence.data(), silence.size(), mouthFramesForSamples(silence.size()), frames) &&
					  frames.size() == 30 && frames.back().weights[VISEME_SIL] == 1.f && frames.back().weights[(int)eViseme::aa] == 0.f,
				  "digital silence is silence to the loudness backend");
		}

		// The model selector: the names, and the factory per model
		{
			bool bRoundTrip= true;
			for (int index= 0; index < MOUTH_MODEL_COUNT; ++index)
			{
				eMouthModel parsed= eMouthModel::Loudness;
				bRoundTrip&= parseMouthModel(mouthModelName((eMouthModel)index), parsed) && parsed == (eMouthModel)index;
			}
			check(bRoundTrip && std::string(mouthModelName(eMouthModel::Loudness)) == "loudness" &&
					  std::string(mouthModelName(eMouthModel::Mfcc)) == "mfcc" &&
					  std::string(mouthModelName(eMouthModel::OvrLipSync)) == "ovrlipsync",
				  "each model name round-trips");
			eMouthModel untouched= eMouthModel::OvrLipSync;
			check(!parseMouthModel("bogus", untouched) && untouched == eMouthModel::OvrLipSync,
				  "an unknown name is refused and leaves the model untouched");

			std::string reason;
			const std::unique_ptr<IMouthBackend> loudness= createMouthBackend(eMouthModel::Loudness, "", reason);
			check(loudness && loudness->isReady() && loudness->getInfo() == "loudness" && reason.empty(),
				  "the factory makes a ready loudness backend");
			const std::unique_ptr<IMouthBackend> mfcc= createMouthBackend(eMouthModel::Mfcc, "", reason);
			check(mfcc && mfcc->isReady() && mfcc->getInfo() == "no profile, volume only" && reason.empty(),
				  "the factory makes a ready volume-only mfcc backend");
#ifndef MIKANTRACK_OVRLIPSYNC
			const std::unique_ptr<IMouthBackend> ovr= createMouthBackend(eMouthModel::OvrLipSync, "", reason);
			check(!ovr && !reason.empty(), "a build without the Oculus SDK makes no Oculus backend, with the reason");
#endif
		}
	}

	// (n) The wav writer and the sidecar recorder
	{
		const std::filesystem::path tempDir= std::filesystem::temp_directory_path() / "mikan_audio_test";
		std::filesystem::create_directories(tempDir);

		std::vector<float> tone(4000);
		for (size_t index= 0; index < tone.size(); ++index)
			tone[index]= 0.6f * std::sin(2.f * glm::pi<float>() * 440.f * (float)index / AUDIO_SAMPLE_RATE);
		tone[10]= 2.f;
		tone[11]= -2.f;

		const std::filesystem::path writerPath= tempDir / "writer.wav";
		std::string error;
		{
			WavWriter writer;
			check(writer.open(writerPath, AUDIO_SAMPLE_RATE, 1, error), "the writer opens a file");
			check(writer.write(tone.data(), 1000) && writer.writeSilence(100) && writer.write(tone.data() + 1100, 2900) &&
					  writer.getSamplesWritten() == 4000,
				  "samples and silence append");
			check(writer.close(), "close patches the header");
		}
		std::vector<float> readBack;
		check(WavFile::readMono16k(writerPath, readBack, error) && readBack.size() == 4000, "the written file reads back whole");
		if (readBack.size() == 4000)
		{
			float maxError= 0.f;
			for (size_t index= 0; index < 1000; ++index)
			{
				const float expected= std::max(-1.f, std::min(1.f, tone[index]));
				maxError= std::max(maxError, std::fabs(readBack[index] - expected));
			}
			bool bSilent= true;
			for (size_t index= 1000; index < 1100; ++index)
				bSilent&= readBack[index] == 0.f;
			check(maxError < 1.f / 32767.f + 1e-5f && bSilent, "PCM16 keeps the samples to a quantum, clamped, with the silence in place");
		}

		// A file the writer never closed keeps the placeholder sizes and still
		// reads to its end
		{
			std::vector<uint8_t> bytes;
			std::ifstream file(writerPath, std::ios::binary);
			bytes.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
			file.close();
			for (int index= 0; index < 4; ++index)
			{
				bytes[4 + index]= 0xFF;
				bytes[40 + index]= 0xFF;
			}
			const std::filesystem::path tornPath= tempDir / "torn.wav";
			writeBytes(tornPath, bytes);
			std::vector<float> torn;
			check(WavFile::readMono16k(tornPath, torn, error) && torn.size() == 4000, "a file cut short of its close reads to its end");
		}

		// The recorder through its writer thread, in capture-sized blocks
		{
			const std::filesystem::path recorderPath= tempDir / "recorder.wav";
			AudioRecorder recorder;
			check(recorder.start(recorderPath.string()) && recorder.isRecording(), "the recorder starts");
			for (size_t offset= 0; offset < tone.size(); offset+= 256)
				recorder.enqueue(tone.data() + offset, (int)std::min<size_t>(256, tone.size() - offset));
			recorder.stop();
			check(!recorder.isRecording() && recorder.getSamplesQueued() == 4000 && recorder.getSamplesWritten() == 4000 &&
					  recorder.getDroppedBlocks() == 0,
				  "every block reaches the file");
			std::vector<float> recorded;
			check(WavFile::readMono16k(recorderPath, recorded, error) && recorded.size() == 4000 &&
					  std::fabs(recorded[500] - tone[500]) < 1e-4f,
				  "the recorder's file reads back");
			check(!recorder.start((tempDir / "no_such_dir" / "x.wav").string()) && !recorder.isRecording(),
				  "an unopenable path leaves the recorder stopped");
		}
	}

	// (o) The pacer: a hop's burst out one per frame, catching up after a stall
	{
		MouthPacer pacer;
		pacer.configure(6, 100.0);
		auto frameAt= [](int64_t index) {
			MouthChainFrame frame;
			frame.frameIndex= index;
			return frame;
		};
		MouthChainFrame out;
		check(!pacer.poll(1000.0, out), "nothing pending gives nothing");
		pacer.push(frameAt(0));
		pacer.push(frameAt(1));
		pacer.push(frameAt(2));
		check(pacer.poll(1000.0, out) && out.frameIndex == 0, "the first frame goes out at once");
		check(!pacer.poll(1050.0, out), "the second waits its interval");
		check(pacer.poll(1100.0, out) && out.frameIndex == 1 && pacer.poll(1200.0, out) && out.frameIndex == 2,
			  "then one per interval");
		check(!pacer.poll(1300.0, out) && pacer.getPendingCount() == 0, "and the burst is spent");

		// A late burst (a slow run) under a reader that kept polling: the
		// frames go out paced from its arrival, none dropped, the latency
		// grown by the delay
		check(!pacer.poll(1400.0, out) && !pacer.poll(1459.0, out), "polls while the run is late give nothing");
		pacer.push(frameAt(3));
		pacer.push(frameAt(4));
		pacer.push(frameAt(5));
		check(pacer.poll(1460.0, out) && out.frameIndex == 3 && !pacer.poll(1500.0, out), "a late burst starts paced from its arrival");
		check(pacer.poll(1559.0, out) && out.frameIndex == 4 && pacer.poll(1659.0, out) && out.frameIndex == 5 &&
				  pacer.getDroppedFrames() == 0,
			  "and drains at the interval with nothing dropped");

		// A reader that polls late finds several frames due and takes the
		// newest
		pacer.push(frameAt(6));
		pacer.push(frameAt(7));
		pacer.push(frameAt(8));
		check(pacer.poll(1760.0, out) && out.frameIndex == 6, "the next burst keeps the cadence");
		check(pacer.poll(2100.0, out) && out.frameIndex == 8 && pacer.getDroppedFrames() == 1,
			  "a late reader takes the newest due frame and the superseded one is dropped");

		// The cap drops the oldest of a backlog
		pacer.reset();
		for (int64_t index= 0; index < 8; ++index)
			pacer.push(frameAt(index));
		check(pacer.getPendingCount() == 6 && pacer.getDroppedFrames() == 2 && pacer.poll(0.0, out) && out.frameIndex == 2,
			  "a backlog past the cap drops its oldest");
	}

	// (p) The replay alignment: the sidecar's start on the live hop grid
	{
		const MouthWindowConfig window;
		MouthReplayAlignment alignment= mouthReplayAlignment(0, window);
		check(alignment.padSamples == 0 && alignment.frameOffset == 0, "a sidecar from the stream's start needs no pad");
		alignment= mouthReplayAlignment(1600, window);
		check(alignment.padSamples == 0 && alignment.frameOffset == 3, "a start on a hop boundary is three frames per hop in");
		alignment= mouthReplayAlignment(256, window);
		check(alignment.padSamples == 256 && alignment.frameOffset == 0, "a start inside the first hop pads to the boundary");
		alignment= mouthReplayAlignment(1600 * 77 + 768, window);
		check(alignment.padSamples == 768 && alignment.frameOffset == 231, "a start inside a later hop pads and offsets");
	}

	// (q) The shared chain: live and replay run the same code, and a stretch
	// of the stream replays to the live frames once the window has filled
	{
		{
			// Five seconds of a buzz with a slow amplitude envelope, so the
			// gate and the visemes change along it
			const int streamSeconds= 5;
			std::vector<float> stream(streamSeconds * AUDIO_SAMPLE_RATE);
			for (size_t index= 0; index < stream.size(); ++index)
			{
				const float t= (float)index / AUDIO_SAMPLE_RATE;
				const float envelope= 0.3f + 0.25f * std::sin(2.f * glm::pi<float>() * 0.7f * t);
				float buzz= 0.f;
				for (int harmonic= 1; harmonic <= 6; ++harmonic)
					buzz+= std::sin(2.f * glm::pi<float>() * 140.f * (float)harmonic * t) / (float)harmonic;
				stream[index]= envelope * 0.25f * buzz;
			}

			MfccBackend liveBackend;
			MfccBackend replayBackend;
			liveBackend.create("");
			replayBackend.create("");
			check(liveBackend.isReady() && replayBackend.isReady(), "two volume-only backends are ready");

			MouthChainConfig chainConfig;
			chainConfig.gateDbfs= -40.f;
			MouthReplayOutput live;
			replayMouthChain(stream, 0, chainConfig, liveBackend, live);
			check(live.frames.size() == streamSeconds * MOUTH_FRAME_RATE - (size_t)chainConfig.window.lookaheadFrames &&
					  live.runs.size() == streamSeconds * 10,
				  "the chain emits a frame per 33 ms less the lookahead, one run per hop");
			bool bIndexed= true;
			for (size_t index= 0; index < live.frames.size(); ++index)
				bIndexed&= live.frames[index].frameIndex == (int64_t)index;
			check(bIndexed, "frames carry their stream indices in order");

			// A backend or profile change mid-stream sets the gain and the
			// gate on the chain and nothing else, so the indices keep
			// counting from the device open, which is what ties a recorded
			// frame to its sidecar
			{
				MouthChain chain;
				chain.configure(chainConfig);
				std::vector<MouthChainFrame> frames;
				std::vector<MouthChainRun> runs;
				chain.push(stream.data(), 2 * AUDIO_SAMPLE_RATE, &liveBackend, frames, runs);
				const int64_t lastBefore= frames.back().frameIndex;
				MfccBackend swapped;
				swapped.create("");
				chain.setGainDb(chainConfig.gainDb);
				chain.setGateDbfs(chainConfig.gateDbfs);
				chain.push(stream.data() + 2 * AUDIO_SAMPLE_RATE, AUDIO_SAMPLE_RATE, &swapped, frames, runs);
				check(chain.getPushedSamples() == 3 * AUDIO_SAMPLE_RATE && frames.back().frameIndex == lastBefore + MOUTH_FRAME_RATE,
					  "a backend swap through the setters keeps the stream count and the frame indices");
				chain.configure(chainConfig);
				check(chain.getPushedSamples() == 0, "configure restarts the stream");
			}

			// A sidecar that began inside the stream, replayed onto the grid
			const int64_t startSample= 1600 * 10 + 768;
			const std::vector<float> sidecar(stream.begin() + startSample, stream.end());
			const MouthReplayAlignment alignment= mouthReplayAlignment(startSample, chainConfig.window);
			MouthReplayOutput replayed;
			replayMouthChain(sidecar, alignment.padSamples, chainConfig, replayBackend, replayed);
			check(replayed.frames.size() + (size_t)alignment.frameOffset == live.frames.size(),
				  "the replayed frames count up to the live frames");

			// Past the warm-up both see identical windows, the replay's
			// zero padding long out of them
			const int64_t warmup= 2 * MOUTH_FRAME_RATE;
			float maxDiff= 0.f;
			int compared= 0;
			for (size_t index= (size_t)warmup; index < replayed.frames.size(); ++index)
			{
				const MouthFrame& a= replayed.frames[index].frame;
				const MouthFrame& b= live.frames[index + (size_t)alignment.frameOffset].frame;
				for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
					maxDiff= std::max(maxDiff, std::fabs(a.weights[viseme] - b.weights[viseme]));
				++compared;
			}
			MIKAN_LOG_INFO("test-audio") << "aligned replay: " << compared << " frames compared, max diff " << maxDiff;
			check(compared > 0 && maxDiff < 1e-3f, "an aligned replay reproduces the live frames past the warm-up");

			// The same stretch as a recording with its sidecar, left in the
			// temp folder for --replay-audio to run on
			{
				const std::filesystem::path tempDir= std::filesystem::temp_directory_path() / "mikan_audio_test";
				const std::string recordingPath= (tempDir / "replay.jsonl").string();
				WavWriter writer;
				std::string error;
				check(writer.open(TrackingRecording::makeAudioSidecarPath(recordingPath), AUDIO_SAMPLE_RATE, 1, error) &&
						  writer.write(sidecar.data(), sidecar.size()) && writer.close(),
					  "the test sidecar is written");

				AppConfig config;
				config.cameras.push_back(CameraProfile());
				config.audio.enabled= true;
				config.audio.gateDbfs= chainConfig.gateDbfs;
				RecordingHeader header;
				header.formatVersion= TrackingRecording::k_formatVersion;
				header.appConfigJsonText= config.toJsonString();
				header.cameraCount= 1;
				header.audio.present= true;
				header.audio.fileName= "replay.wav";
				header.audio.sampleRate= AUDIO_SAMPLE_RATE;
				header.audio.startSampleIndex= startSample;
				TrackingRecorder recorder;
				check(recorder.start(recordingPath, TrackingRecording::headerToJson(header)), "the test recording starts");
				// Two iterations per live frame, the way a 60 Hz loop sees a
				// 30 fps mouth
				int64_t seq= 0;
				for (const MouthChainFrame& frame : live.frames)
				{
					for (int repeat= 0; repeat < 2; ++repeat)
					{
						RecordedFrame recorded;
						recorded.seq= seq++;
						recorded.nowTimestampMs= 1000.0 + (double)frame.frameIndex * 1000.0 / MOUTH_FRAME_RATE + repeat * 16.0;
						recorded.mouth.present= true;
						recorded.mouth.audioFrameIndex= frame.frameIndex;
						recorded.mouth.frame= frame.frame;
						recorder.enqueueFrame(std::move(recorded));
					}
				}
				recorder.stop();
				MIKAN_LOG_INFO("test-audio") << "wrote " << recordingPath << " and its sidecar for --replay-audio";
			}
		}

		// The gate and a backend that runs while gated: it sees every hop
		// of a silence the gate holds closed, with only the hop's frames as
		// new, and the chain still emits the neutral mouth. A backend that
		// does not run while gated is not called at all.
		{
			const std::vector<float> silence(AUDIO_SAMPLE_RATE, 0.f);
			std::vector<float> loud(AUDIO_SAMPLE_RATE);
			for (size_t index= 0; index < loud.size(); ++index)
				loud[index]= 0.3f * std::sin(2.f * glm::pi<float>() * 200.f * (float)index / AUDIO_SAMPLE_RATE);
			const MouthChainConfig chainConfig;

			for (const bool bRunsWhileGated : {true, false})
			{
				const std::string label= bRunsWhileGated ? "a backend that runs while gated" : "a backend that does not";

				CountingBackend counting;
				counting.bRunsWhileGated= bRunsWhileGated;
				MouthChain chain;
				chain.configure(chainConfig);
				std::vector<MouthChainFrame> frames;
				std::vector<MouthChainRun> runs;
				chain.push(silence.data(), silence.size(), &counting, frames, runs);

				bool bClosed= runs.size() == 10;
				for (const MouthChainRun& run : runs)
					bClosed&= !run.bGateOpen && run.bInferred == bRunsWhileGated;
				bool bNeutral= frames.size() == 30;
				for (const MouthChainFrame& frame : frames)
				{
					for (float weight : frame.frame.weights)
						bNeutral&= weight == 0.f;
				}
				if (bRunsWhileGated)
				{
					bool bHops= counting.inferCalls == 10 && counting.newFrameCounts.size() == 10;
					for (int newFrameCount : counting.newFrameCounts)
						bHops&= newFrameCount == 3;
					check(bClosed && bHops, (label + " is called once per gated hop with its three new frames").c_str());
				}
				else
				{
					check(bClosed && counting.inferCalls == 0, (label + " is not called while the gate is closed").c_str());
				}
				check(bNeutral, (label + " leaves the gated mouth exactly neutral").c_str());

				chain.configure(chainConfig);
				frames.clear();
				runs.clear();
				chain.push(loud.data(), loud.size(), &counting, frames, runs);
				bool bOpen= !frames.empty();
				for (const MouthChainFrame& frame : frames)
					bOpen&= frame.frame.weights[(int)eViseme::aa] > 0.f;
				check(bOpen, (label + " opens aa through an open gate").c_str());
			}
		}
	}

	// (r) The calibration frames of a clip and the gate a quiet room suggests
	{
		std::vector<float> tone(16000);
		for (size_t index= 0; index < tone.size(); ++index)
		{
			const float t= (float)index / AUDIO_SAMPLE_RATE;
			tone[index]= 0.3f * (std::sin(2.f * glm::pi<float>() * 150.f * t) + 0.5f * std::sin(2.f * glm::pi<float>() * 450.f * t) +
								 0.25f * std::sin(2.f * glm::pi<float>() * 900.f * t));
		}
		MfccExtractor extractor;
		std::vector<MfccVector> frames;
		calibrationFrames(tone.data(), tone.size(), -40.f, extractor, frames);
		check(frames.size() == 29, "a second of a loud tone gives a frame per 30 fps frame once a block has arrived");
		calibrationFrames(tone.data(), tone.size(), -5.f, extractor, frames);
		check(frames.empty(), "a gate above the tone keeps no frame");
		std::vector<float> silence(16000, 0.f);
		calibrationFrames(silence.data(), silence.size(), -40.f, extractor, frames);
		check(frames.empty(), "digital silence gives no frames past a gate");
		calibrationFrames(silence.data(), silence.size(), -100.f, extractor, frames);
		check(frames.size() == 29, "and every frame with the gate off, the noise phoneme's capture");
		calibrationFrames(tone.data(), 300, -100.f, extractor, frames);
		check(frames.empty(), "less than a block gives no frames");

		// Uniform noise at -60 dBFS RMS: the suggestion is 6 dB over it
		std::vector<float> room(16000);
		uint32_t seed= 99;
		for (float& sample : room)
		{
			seed= seed * 1664525u + 1013904223u;
			sample= (seed / 4294967296.f - 0.5f) * 2.f * 0.001f * std::sqrt(3.f);
		}
		const float suggested= suggestGateDbfs(room.data(), room.size(), 0.f);
		check(std::fabs(suggested - (-54.f)) < 1.5f, "a -60 dBFS room suggests a gate near -54 dBFS");
		check(std::fabs(suggestGateDbfs(room.data(), room.size(), 10.f) - (suggested + 10.f)) < 1e-4f,
			  "the gain the stage applies moves the suggestion with it");
		check(suggestGateDbfs(silence.data(), silence.size(), 0.f) == -80.f, "digital silence suggests the gate's floor");
	}

#ifdef MIKANTRACK_OVRLIPSYNC
	// (s) The Oculus LipSync engine, where the DLL is beside the exe: the
	// remap, silence and a voiced buzz, only the new frames fed, the reset
	// reproducing a run, and the chain end to end
	{
		OvrLipSyncBackend ovr;
		if (!ovr.create())
		{
			MIKAN_LOG_INFO("test-audio") << "(s) Oculus LipSync skipped: " << ovr.getReason();
		}
		else
		{
			// The heaviest of the fifteen, silence included
			const auto heaviest= [](const MouthFrame& frame) {
				return (int)(std::max_element(frame.weights.begin(), frame.weights.end()) - frame.weights.begin());
			};
			// The chain's neutral mouth: every weight zero
			const auto isNeutral= [](const MouthFrame& frame) {
				return std::all_of(frame.weights.begin(), frame.weights.end(), [](float weight) { return weight == 0.f; });
			};

			check(ovr.isReady() && ovr.getInfo().rfind("Oculus LipSync 1.61", 0) == 0 && ovr.getReason().empty(),
				  "the Oculus engine is ready at 1.61");

			const std::array<int, VISEME_COUNT>& remap= OvrLipSyncBackend::getVisemeRemap();
			std::array<int, VISEME_COUNT> hits{};
			bool bInRange= true;
			for (int stageIndex : remap)
			{
				bInRange&= stageIndex >= 0 && stageIndex < VISEME_COUNT;
				if (stageIndex >= 0 && stageIndex < VISEME_COUNT)
					hits[stageIndex]++;
			}
			bool bPermutation= bInRange;
			for (int count : hits)
				bPermutation&= count == 1;
			check(bPermutation && remap[0] == VISEME_SIL && remap[10] == (int)eViseme::aa,
				  "the remap is a permutation with sil and aa in place");

			// A 120 Hz voiced buzz with its harmonics up to 2.4 kHz
			std::vector<float> buzz(AUDIO_SAMPLE_RATE);
			for (size_t index= 0; index < buzz.size(); ++index)
			{
				const float t= (float)index / AUDIO_SAMPLE_RATE;
				float sum= 0.f;
				for (int harmonic= 1; harmonic <= 20; ++harmonic)
					sum+= std::sin(2.f * glm::pi<float>() * 120.f * (float)harmonic * t) / (float)harmonic;
				buzz[index]= 0.1f * sum;
			}
			const std::vector<float> silence(AUDIO_SAMPLE_RATE, 0.f);

			std::vector<MouthFrame> frames;
			ovr.reset();
			check(ovr.infer(silence.data(), silence.size(), 30, frames) && frames.size() == 30 &&
					  heaviest(frames.back()) == VISEME_SIL,
				  "a second of digital silence ends on silence");

			ovr.reset();
			std::vector<MouthFrame> first;
			check(ovr.infer(buzz.data(), buzz.size(), 30, first) && first.size() == 30, "a second of buzz gives thirty frames");
			bool bWeightsInRange= first.size() == 30;
			bool bSumsToOne= first.size() == 30;
			bool bVoiced= false;
			for (size_t index= first.size() >= 10 ? first.size() - 10 : 0; index < first.size(); ++index)
			{
				float sum= 0.f;
				for (float weight : first[index].weights)
				{
					bWeightsInRange&= weight >= 0.f && weight <= 1.f;
					sum+= weight;
				}
				bSumsToOne&= std::fabs(sum - 1.f) < 0.05f;
				bVoiced|= heaviest(first[index]) != VISEME_SIL;
			}
			check(bWeightsInRange, "the buzz's weights are within 0..1");
			check(bSumsToOne, "the buzz's fifteen weights sum to one past the warm-up");
			check(bVoiced, "the buzz is voiced on the engine's mouth");
			const std::string firstLast= visemeName(heaviest(first.back()));
			MIKAN_LOG_INFO("test-audio") << "Oculus buzz: last frame heaviest " << firstLast << " at "
										 << first.back().weights[heaviest(first.back())] << ", inference "
										 << ovr.getLastInferMs() << " ms for one second";

			check(ovr.infer(buzz.data(), buzz.size(), 3, frames) && frames.size() == 30 &&
					  std::all_of(frames.begin(), frames.begin() + 27, isNeutral),
				  "only the new frames are fed, the earlier ones neutral");

			ovr.reset();
			std::vector<MouthFrame> second;
			ovr.infer(buzz.data(), buzz.size(), 30, second);
			float maxDiff= second.size() == first.size() ? 0.f : 1.f;
			if (second.size() == first.size())
			{
				for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
					maxDiff= std::max(maxDiff, std::fabs(second.back().weights[viseme] - first.back().weights[viseme]));
			}
			MIKAN_LOG_INFO("test-audio") << "Oculus reset: last frame max diff " << maxDiff;
			check(maxDiff < 1e-5f, "a reset engine reproduces the run");

			// The chain end to end: the replay resets the engine, the gate
			// opens on the buzz, and the emitted mouth is voiced
			std::vector<float> stream(2 * AUDIO_SAMPLE_RATE);
			for (size_t index= 0; index < stream.size(); ++index)
				stream[index]= buzz[index % buzz.size()];
			MouthChainConfig chainConfig;
			MouthReplayOutput replayed;
			replayMouthChain(stream, 0, chainConfig, ovr, replayed);
			bool bAllOpen= !replayed.runs.empty();
			for (const MouthChainRun& run : replayed.runs)
				bAllOpen&= run.bGateOpen && run.bInferred;
			int voicedFrames= 0;
			for (const MouthChainFrame& frame : replayed.frames)
			{
				if (heaviest(frame.frame) != VISEME_SIL)
					++voicedFrames;
			}
			MIKAN_LOG_INFO("test-audio") << "Oculus chain: " << replayed.frames.size() << " frames, " << voicedFrames
										 << " voiced";
			check(bAllOpen && replayed.frames.size() == 60 && voicedFrames > 0,
				  "the chain runs the engine with the gate open and emits a voiced mouth");
		}
	}
#endif

	MIKAN_LOG_INFO("test-audio") << (failures == 0 ? "ALL PASSED" : "FAILURES") << " (" << failures << " failed)";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-audio",
					"Audio stage: mouth column group, window policy, wav reader, filter, the MFCC chain against uLipSync, "
					"the profile, the viseme table, the backends and the model selector, the wav writer, the sidecar recorder, "
					"the pacer, the replay alignment, the chain, the calibration frames and the gate suggestion, the Oculus "
					"LipSync engine",
					eTestCategory::SelfTest, runAudioTest)
