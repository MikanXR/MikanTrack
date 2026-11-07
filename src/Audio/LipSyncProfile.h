#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "MfccFeatures.h"
#include "VisemeTypes.h"

// A calibration profile in uLipSync's form: per phoneme the MFCC frames the
// user held it over, whose mean is the template a block is scored against.
// The file is the JSON uLipSync's Export writes (and VTube Studio hands
// out), field for field, so a profile moves between the two: the analysis
// parameters, then mfccs[{name, mfccCalibrationDataList[{array}]}]. On load
// each phoneme keeps only its last mfccDataCount frames, as uLipSync does
// when it enables a profile, so the templates come out the same there and
// here.

// uLipSync's CompareMethod, serialized as its ordinal
enum class eCompareMethod : int
{
	L1Norm= 0,
	L2Norm= 1,
	CosineSimilarity= 2,
};

struct LipSyncPhoneme
{
	std::string name;
	std::vector<MfccVector> frames;
	// The mean of the frames, zero while there are none
	MfccVector mean{};
	// The viseme the name maps to, or -1 for a name the stage does not know
	int viseme= -1;
};

// What one block scored: uLipSync's normalized score per phoneme, the same
// folded onto the visemes (a phoneme the stage does not know counts as
// silence), and the viseme of the best phoneme
struct LipSyncScores
{
	std::vector<float> perPhoneme;
	VisemeWeights perViseme{};
	int mainViseme= VISEME_SIL;
};

class LipSyncProfile
{
public:
	static constexpr int k_defaultMfccDataCount= 16;

	// uLipSync's Profile fields, its defaults
	int mfccNum= MFCC_COEFF_COUNT;
	int mfccDataCount= k_defaultMfccDataCount;
	int melFilterBankChannels= LIPSYNC_DEFAULT_MEL_CHANNELS;
	int targetSampleRate= 16000;
	int sampleCount= LIPSYNC_BLOCK_SAMPLES;
	bool useStandardization= false;
	eCompareMethod compareMethod= eCompareMethod::L2Norm;
	std::string jsonPath;

	std::vector<LipSyncPhoneme> phonemes;
	// The pooled mean and population standard deviation of every kept frame
	// per coefficient; zero and one while standardization is off
	MfccVector means{};
	MfccVector stdDevs{};

	LipSyncProfile();

	// Reads a uLipSync profile JSON. False with the reason on a missing or
	// malformed file, or one whose coefficient count, block size, or sample
	// rate the stage cannot run.
	bool load(const std::filesystem::path& path, std::string& outError);
	// Writes the profile as uLipSync's Export does
	bool save(const std::filesystem::path& path, std::string& outError) const;

	// Appends a phoneme with its frames; finalize before scoring
	void addPhoneme(const std::string& name, const std::vector<MfccVector>& frames);
	// Trims each phoneme to its last mfccDataCount frames, takes the means,
	// and the standardization statistics
	void finalize();

	bool empty() const { return phonemes.empty(); }

	// Scores one block's coefficients against every template
	void score(const MfccVector& mfcc, LipSyncScores& outScores) const;

private:
	float phonemeScore(const LipSyncPhoneme& phoneme, const MfccVector& mfcc) const;
};

// The viseme a uLipSync phoneme name drives, case-insensitive: A/aa, I/ih,
// U/ou/oo, E/eh, O/oh, and the noise phoneme -/N/sil/silence/noise; -1 for
// any other name
int visemeForPhonemeName(const std::string& name);

// The frames a calibration prompt yields: one block per 30 fps frame over
// the samples, keeping those whose level reaches gateDbfs (every frame at
// -100 or under). What the wizard holds a phoneme over.
void calibrationFrames(const float* samples, size_t sampleCount, float gateDbfs, MfccExtractor& extractor,
					   std::vector<MfccVector>& outFrames);
