#include "TestCommon.h"

#include <limits>

#include "AppConfig.h"
#include "AudioRecorder.h"
#include "AudioService.h"
#include "AudioTypes.h"
#include "MfccBackend.h"
#include "MfccFeatures.h"
#include "MouthChain.h"
#include "MouthFilter.h"
#include "MouthReplay.h"
#include "MouthWindow.h"
#include "VadGate.h"
#include "VisemeFit.h"
#include "VisemeModel.h"
#include "VisemeTable.h"
#include "VisemeTypes.h"
#include "WavFile.h"

// Tests for the audio stage's pieces: the mouth columns and the frame
// accounting, the rolling-window policy, the wav reader and resampler, the
// emission filter, the MFCC chain against HeadAudio's vectors, the viseme
// model and classifier, the viseme table, the backend, the level gate, the
// config, the wav writer and the sidecar recorder, the pacer, the chain and
// its replay, and the calibration fit.

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
		check(defaults.windowSamples == AUDIO_SAMPLE_RATE && defaults.hopSamples == AUDIO_SAMPLE_RATE / 10 &&
				  defaults.lookaheadFrames == 2,
			  "the defaults are a 1 s window, a 100 ms hop, and a two-frame lookahead");
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

	// (g) The MFCC chain against vectors HeadAudio's JavaScript computed on
	// the same signal: a 150 Hz buzz under LCG noise, pre-emphasised, one
	// second at 16 kHz, blocks of 512 at a 256 hop
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
		std::vector<float> emphasized(signal.size());
		MfccExtractor::preEmphasize(signal.data(), signal.size(), emphasized.data());
		check(emphasized[0] == signal[0] && std::fabs(emphasized[1] - (signal[1] - 0.97f * signal[0])) < 1e-7f,
			  "pre-emphasis subtracts 0.97 of the previous raw sample");

		struct Reference
		{
			int block;
			float logEnergy;
			std::array<float, MFCC_COEFF_COUNT> raw;
		};
		const Reference references[]= {
			{0, -0.176817f, {-17.49235f, -1.03007f, -3.42338f, -1.78725f, -2.15971f, -1.47795f, -5.02754f, -2.16766f, -4.60539f, -0.93857f, -1.55210f, -9.00206f}},
			{30, -0.217089f, {-17.30108f, -2.28640f, -4.39289f, -4.32410f, -4.27136f, 0.77890f, -3.36385f, -6.45078f, -5.17831f, -5.88783f, -3.70380f, -5.90096f}},
			{60, -0.147579f, {-17.16085f, -2.83122f, -5.83828f, -0.24228f, -2.24084f, -2.47702f, -4.45766f, -3.91879f, -5.09414f, -2.06604f, -0.97480f, -5.36106f}},
		};
		MfccExtractor extractor;
		for (const Reference& reference : references)
		{
			const size_t blockEnd= MFCC_BLOCK_SAMPLES + (size_t)reference.block * MFCC_HOP_SAMPLES;
			float logEnergy= 0.f;
			MfccVector raw;
			extractor.compute(emphasized.data() + blockEnd - MFCC_BLOCK_SAMPLES, logEnergy, raw);
			float worst= std::fabs(logEnergy - reference.logEnergy);
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
				worst= std::max(worst, std::fabs(raw[index] - reference.raw[index]));
			const std::string label= "block " + std::to_string(reference.block) +
									 " matches HeadAudio's log energy and coefficients (worst " + std::to_string(worst) + ")";
			check(worst < 2e-3f, label.c_str());
			MfccExtractor::compress(raw);
			bool bCompressed= true;
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
				bCompressed&= std::fabs(raw[index] - std::tanh(reference.raw[index])) < 2e-3f;
			check(bCompressed, "compression is tanh");
		}
		check(MfccBackend::hopsForSamples(511) == 0 && MfccBackend::hopsForSamples(512) == 1 &&
				  MfccBackend::hopsForSamples(16000) == 61,
			  "a block is 512 samples and a second of audio is 61 hops");
	}

	// (h) The model reader and the classifier on the shipped model
	{
		check(VisemeModel::k_recordBytes == 368, "a record is 368 bytes");
		if (std::filesystem::exists(MfccBackend::k_defaultModelPath))
		{
			VisemeModel model;
			std::string error;
			check(model.load(MfccBackend::k_defaultModelPath, error) && model.prototypes.size() == 39,
				  "the shipped model holds 39 prototypes");
			bool bFirst= !model.prototypes.empty() && model.prototypes.front().phoneme == "\xC3\xB0" &&
						 model.prototypes.front().viseme == (uint8_t)eViseme::TH &&
						 model.prototypes.front().group == 0;
			check(bFirst, "the first record is the eth phoneme on TH in group 0");
			bool bSilLast= !model.prototypes.empty() && model.prototypes.back().phoneme == "s1" &&
						   model.prototypes.back().viseme == VISEME_SIL;
			check(bSilLast, "the last record is the silence prototype");

			VisemeClassifier classifier;
			classifier.setModel(model);
			bool bMeansClassify= true;
			std::vector<float> distances;
			for (size_t index= 0; index < model.prototypes.size(); ++index)
			{
				const VisemePrototype& prototype= model.prototypes[index];
				classifier.reset();
				int predicted= VISEME_SIL;
				for (int repeat= 0; repeat < 10; ++repeat)
					predicted= classifier.predict(prototype.mu, &distances);
				bMeansClassify&= predicted == prototype.viseme && distances[index] < 1e-5f;
			}
			check(bMeansClassify, "every prototype's mean classifies as its viseme at zero distance");

			classifier.reset();
			check(classifier.predict(MfccVector{}) == VISEME_SIL, "the zero vector is silence");
		}
		else
		{
			MIKAN_LOG_WARNING("test-audio") << MfccBackend::k_defaultModelPath << " missing - SKIPPING the model checks";
		}

		VisemeModel missing;
		std::string error;
		check(!missing.load("resources/audio/does_not_exist.bin", error) && error.find("missing") != std::string::npos,
			  "a missing model file reports the reason");
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
			  "the viseme names follow HeadAudio's id order");

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
		config.audio.profilePath= "audio/profile.bin";
		AppConfig loaded;
		const bool bLoaded= loaded.loadFromJsonString(config.toJsonString());
		check(bLoaded && loaded.audio.enabled && loaded.audio.deviceName == "USB Microphone" &&
				  std::fabs(loaded.audio.gainDb - 6.f) < 1e-6f && std::fabs(loaded.audio.gateDbfs + 38.f) < 1e-6f &&
				  loaded.audio.profilePath == "audio/profile.bin",
			  "the audio section round-trips through the project JSON");
		AppConfig fresh;
		check(fresh.loadFromJsonString("{}") && !fresh.audio.enabled &&
				  fresh.audio.deviceName.empty() && std::fabs(fresh.audio.gateDbfs + 45.f) < 1e-6f,
			  "a project without an audio section gets the defaults");
	}

	// (l) The energy gate
	{
		VadGate gate;
		VadGateConfig gateConfig;
		gateConfig.activeHops= 2;
		gateConfig.inactiveHops= 3;
		gate.configure(gateConfig);
		check(!gate.isActive() && !gate.process(-3.f), "the gate starts closed and one loud hop does not open it");
		check(gate.process(-3.f), "the second loud hop opens it");
		check(gate.process(-4.5f) && gate.process(-6.f) && gate.process(-6.f),
			  "a hop between the thresholds and two quiet hops keep it open");
		check(!gate.process(-6.f), "the third quiet hop closes it");
		gate.reset();
		gate.process(-3.f);
		check(!gate.isActive(), "reset clears the count toward opening");
	}

	// (m) The mfcc backend on a window
	{
		if (std::filesystem::exists(MfccBackend::k_defaultModelPath))
		{
			MfccBackend backend;
			check(backend.create() && backend.isReady(), "the mfcc backend loads the shipped model");

			std::vector<float> tone(16000);
			for (size_t index= 0; index < tone.size(); ++index)
			{
				const float t= (float)index / AUDIO_SAMPLE_RATE;
				tone[index]= 0.3f * (std::sin(2.f * glm::pi<float>() * 150.f * t) +
									 0.5f * std::sin(2.f * glm::pi<float>() * 450.f * t) +
									 0.25f * std::sin(2.f * glm::pi<float>() * 900.f * t));
			}
			std::vector<MouthFrame> frames;
			check(backend.infer(tone.data(), tone.size(), frames) && frames.size() == 30 &&
					  backend.getLastHopVisemes().size() == 61,
				  "a second of audio gives thirty frames from 61 hops");
			check(backend.getLastHopVisemes().back() != VISEME_SIL, "a loud tone passes the gate");
			bool bInRange= true;
			for (const MouthFrame& frame : frames)
			{
				for (float weight : frame.weights)
					bInRange&= weight >= 0.f && weight <= 1.f;
			}
			check(bInRange, "every weight is within 0..1");
			MIKAN_LOG_INFO("test-audio") << "mfcc inference " << backend.getLastInferMs() << " ms for one second, last viseme "
										 << visemeName(backend.getLastHopVisemes().back());

			std::vector<float> silence(16000, 0.f);
			check(backend.infer(silence.data(), silence.size(), frames) && frames.size() == 30, "silence runs through the same shape");
			bool bNeutral= true;
			for (const MouthFrame& frame : frames)
			{
				for (float weight : frame.weights)
					bNeutral&= weight == 0.f;
			}
			check(bNeutral, "digital silence is the neutral mouth");
			check(!backend.infer(tone.data(), 511, frames) && frames.empty(), "less than a block is refused");
		}

		MfccBackend missing;
		check(!missing.create("resources/audio/does_not_exist.bin") && !missing.isReady() &&
				  missing.getUnavailableReason().find("missing") != std::string::npos,
			  "a missing model leaves the mfcc backend not ready with the reason");
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
		if (std::filesystem::exists(MfccBackend::k_defaultModelPath))
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
			check(liveBackend.create() && replayBackend.create(), "two backends load the shipped model");

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
				swapped.create();
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

			// Past the warm-up both see identical windows, and the mfcc vote
			// and crossfade have settled on the stationary signal
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
		else
		{
			MIKAN_LOG_WARNING("test-audio") << MfccBackend::k_defaultModelPath << " missing, chain checks skipped";
		}
	}

	// (r) The calibration fit: the profile writer round-trips, a fit on
	// synthetic vowel clusters classifies them back over the shipped model,
	// and the speech vectors and the gate suggestion read a clip
	{
		const std::filesystem::path tempDir= std::filesystem::temp_directory_path() / "mikan_audio_test";
		std::filesystem::create_directories(tempDir);

		// The writer against the reader, with a two-byte phoneme, a one-byte
		// one, and a three-code-point one cut to two
		{
			VisemeModel model;
			VisemePrototype open;
			open.phoneme= "\xC9\x91";
			open.group= VISEME_FIT_GROUP;
			open.viseme= (uint8_t)eViseme::aa;
			VisemePrototype mid;
			mid.phoneme= "aa";
			mid.viseme= (uint8_t)eViseme::E;
			VisemePrototype silence;
			silence.phoneme= "s1x";
			silence.viseme= VISEME_SIL;
			for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
			{
				open.mu[index]= 0.1f * index - 0.5f;
				mid.mu[index]= -0.07f * index;
			}
			for (int index= 0; index < VISEME_SIGMA_LOWER_COUNT; ++index)
			{
				open.sigmaInvLower[index]= 1.f + 0.01f * index;
				mid.sigmaInvLower[index]= -0.5f + 0.02f * index;
				silence.sigmaInvLower[index]= 1200.f;
			}
			model.prototypes= {open, mid, silence};

			const auto sameRecord= [](const VisemePrototype& a, const VisemePrototype& b) {
				return a.phoneme == b.phoneme && a.group == b.group && a.viseme == b.viseme && a.mu == b.mu &&
					   a.sigmaInvLower == b.sigmaInvLower;
			};
			uint8_t bytes[VisemeModel::k_recordBytes];
			VisemeModel::encodeRecord(open, bytes);
			check(sameRecord(VisemeModel::decodeRecord(bytes), open) && bytes[5] == VISEME_FIT_GROUP && bytes[7] == 0,
				  "a record encodes and decodes to itself with the group and viseme bytes in place");

			const std::filesystem::path profilePath= tempDir / "profile.bin";
			std::string error;
			VisemeModel loaded;
			check(model.save(profilePath, error) && std::filesystem::file_size(profilePath) == 3 * VisemeModel::k_recordBytes &&
					  loaded.load(profilePath, error) && loaded.prototypes.size() == 3,
				  "a three-prototype model saves to three records and loads back");
			check(loaded.prototypes.size() == 3 && sameRecord(loaded.prototypes[0], open) && sameRecord(loaded.prototypes[1], mid),
				  "the loaded prototypes match the saved ones float-exact, the two-byte phoneme included");
			check(loaded.prototypes.size() == 3 && loaded.prototypes[2].phoneme == "s1" &&
					  loaded.prototypes[2].viseme == VISEME_SIL,
				  "a phoneme past two code points is cut to two");
			check(!model.save(tempDir, error) && error.find("cannot write") != std::string::npos,
				  "a path that cannot be opened reports the reason");
		}

		// Five synthetic vowel clusters: the fit lands on their centers and
		// each cluster classifies back as its viseme over the shipped model
		{
			const int clusterCount= 5;
			const eViseme visemes[clusterCount]= {eViseme::aa, eViseme::I, eViseme::U, eViseme::E, eViseme::O};
			const char* const phonemes[clusterCount]= {"aa", "ih", "ou", "eh", "oh"};
			uint32_t seed= 777;
			const auto gaussian= [&seed]() {
				float sum= 0.f;
				for (int draw= 0; draw < 12; ++draw)
				{
					seed= seed * 1664525u + 1013904223u;
					sum+= seed / 4294967296.f;
				}
				return sum - 6.f;
			};
			MfccVector centers[clusterCount];
			std::vector<MfccVector> clusters[clusterCount];
			for (int cluster= 0; cluster < clusterCount; ++cluster)
			{
				for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
					centers[cluster][index]= 0.8f * std::sin(1.7f * cluster + 0.9f * index);
				for (int sample= 0; sample < 60; ++sample)
				{
					MfccVector vector;
					for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
						vector[index]= centers[cluster][index] + 0.25f * gaussian();
					clusters[cluster].push_back(vector);
				}
			}

			std::vector<VisemePrototype> fitted;
			bool bFit= true;
			bool bCentered= true;
			std::string error;
			for (int cluster= 0; cluster < clusterCount; ++cluster)
			{
				VisemePrototype prototype;
				bFit&= fitVisemePrototype(clusters[cluster], (int)visemes[cluster], phonemes[cluster], prototype, error);
				bFit&= prototype.viseme == (uint8_t)visemes[cluster] && prototype.group == VISEME_FIT_GROUP &&
					   prototype.phoneme == phonemes[cluster];
				for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
					bCentered&= std::fabs(prototype.mu[index] - centers[cluster][index]) < 0.12f;
				fitted.push_back(prototype);
			}
			check(bFit, "five clusters of sixty vectors fit as prototypes of their visemes");
			check(bCentered, "each fit lands within 0.12 of its cluster's center");
			// The floor keeps the prototype from being tighter than the floor
			// variance: a diagonal term is at most 1 / 0.25
			bool bFloored= true;
			for (const VisemePrototype& prototype : fitted)
			{
				int packed= 0;
				for (int row= 0; row < MFCC_COEFF_COUNT; ++row)
				{
					packed+= row;
					bFloored&= prototype.sigmaInvLower[packed] > 0.f && prototype.sigmaInvLower[packed] <= 1.f / VISEME_FIT_VARIANCE_FLOOR + 1e-3f;
					packed++;
				}
			}
			check(bFloored, "the inverse variances stay under the floor's inverse");

			VisemeModel base;
			size_t keptCount= 0;
			if (std::filesystem::exists(MfccBackend::k_defaultModelPath))
			{
				check(base.load(MfccBackend::k_defaultModelPath, error), "the shipped model loads as the overlay's base");
				for (const VisemePrototype& prototype : base.prototypes)
				{
					bool bVowel= false;
					for (eViseme viseme : visemes)
						bVowel|= prototype.viseme == (uint8_t)viseme;
					keptCount+= bVowel ? 0 : 1;
				}
				check(keptCount < base.prototypes.size() && keptCount >= 15, "the shipped model holds vowel prototypes to replace");
			}
			const VisemeModel overlay= overlayVisemePrototypes(base, fitted);
			check(overlay.prototypes.size() == keptCount + clusterCount, "the overlay keeps the consonants and the silence and adds the five fits");
			bool bAppended= true;
			for (int cluster= 0; cluster < clusterCount; ++cluster)
				bAppended&= overlay.prototypes[keptCount + cluster].viseme == (uint8_t)visemes[cluster];
			check(bAppended, "the fits sit at the overlay's end in prompt order");

			VisemeClassifier classifier;
			classifier.setModel(overlay);
			bool bClassifies= true;
			for (int cluster= 0; cluster < clusterCount; ++cluster)
			{
				int hits= 0;
				classifier.reset();
				int voted= VISEME_SIL;
				for (const MfccVector& vector : clusters[cluster])
				{
					float nearest= std::numeric_limits<float>::infinity();
					int nearestViseme= VISEME_SIL;
					for (const VisemePrototype& prototype : overlay.prototypes)
					{
						const float d= VisemeClassifier::distance(vector, prototype);
						if (d < nearest)
						{
							nearest= d;
							nearestViseme= prototype.viseme;
						}
					}
					hits+= nearestViseme == (int)visemes[cluster] ? 1 : 0;
					voted= classifier.predict(vector);
				}
				bClassifies&= hits >= 57 && voted == (int)visemes[cluster];
			}
			check(bClassifies, "each cluster classifies back as its viseme, by nearest prototype and by the vote");

			std::vector<MfccVector> few(clusters[0].begin(), clusters[0].begin() + VISEME_FIT_MIN_VECTORS - 1);
			VisemePrototype rejected;
			check(!fitVisemePrototype(few, (int)eViseme::aa, "aa", rejected, error) && error.find("too few") != std::string::npos,
				  "twelve vectors are too few to fit");
		}

		// The speech vectors of a clip and the gate a quiet room suggests
		{
			std::vector<float> tone(16000);
			for (size_t index= 0; index < tone.size(); ++index)
			{
				const float t= (float)index / AUDIO_SAMPLE_RATE;
				tone[index]= 0.3f * (std::sin(2.f * glm::pi<float>() * 150.f * t) +
									 0.5f * std::sin(2.f * glm::pi<float>() * 450.f * t) +
									 0.25f * std::sin(2.f * glm::pi<float>() * 900.f * t));
			}
			std::vector<MfccVector> vectors;
			extractSpeechVectors(tone.data(), tone.size(), VadGateConfig(), vectors);
			check(vectors.size() >= 50 && vectors.size() <= 61, "a second of a loud tone gives most of its 61 hops as vectors");
			bool bCompressed= true;
			for (const MfccVector& vector : vectors)
				for (float value : vector)
					bCompressed&= value >= -1.f && value <= 1.f;
			check(bCompressed, "the vectors are compressed into -1..1");
			std::vector<float> silence(16000, 0.f);
			extractSpeechVectors(silence.data(), silence.size(), VadGateConfig(), vectors);
			check(vectors.empty(), "digital silence gives no vectors");
			extractSpeechVectors(tone.data(), 300, VadGateConfig(), vectors);
			check(vectors.empty(), "less than a block gives no vectors");

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
	}

	MIKAN_LOG_INFO("test-audio") << (failures == 0 ? "ALL PASSED" : "FAILURES") << " (" << failures << " failed)";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-audio",
					"Audio stage: mouth column group, model column table, window policy, wav reader, filter, the two backends, "
					"the MFCC chain, the viseme model and table, the wav writer, the sidecar recorder, the pacer, the replay alignment, "
					"the calibration fit and the profile writer",
					eTestCategory::SelfTest, runAudioTest)
