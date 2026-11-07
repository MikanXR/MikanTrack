#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "MfccFeatures.h"
#include "VisemeTypes.h"

// The packed lower triangle of a 12 x 12 inverse covariance
constexpr int VISEME_SIGMA_LOWER_COUNT= MFCC_COEFF_COUNT * (MFCC_COEFF_COUNT + 1) / 2;

// One Gaussian prototype of a viseme model: the mean of a phoneme's
// compressed MFCC vectors and the inverse of their covariance
struct VisemePrototype
{
	// The IPA phoneme the prototype was fit on, up to two code points
	std::string phoneme;
	// The group the prototype belongs to; a later model with the same group
	// replaces it, which is how a calibration overlays the shipped model
	uint8_t group= 0;
	uint8_t viseme= VISEME_SIL;
	MfccVector mu{};
	std::array<float, VISEME_SIGMA_LOWER_COUNT> sigmaInvLower{};
};

// A viseme model: HeadAudio's binary format, a flat run of fixed-size
// records (an 8-byte header holding the packed phoneme, the group, and the
// viseme id, then the mean and the packed inverse covariance as little-endian
// float32)
struct VisemeModel
{
	static constexpr size_t k_recordBytes= 8 + sizeof(float) * (MFCC_COEFF_COUNT + VISEME_SIGMA_LOWER_COUNT);

	std::vector<VisemePrototype> prototypes;

	// Reads every record of the file; false with the reason on a missing,
	// short, or empty file
	bool load(const std::filesystem::path& path, std::string& outError);
	// Writes every prototype as a record, creating the parent folder; false
	// with the reason when the file cannot be written
	bool save(const std::filesystem::path& path, std::string& outError) const;
	// Decodes one k_recordBytes record
	static VisemePrototype decodeRecord(const uint8_t* bytes);
	// Encodes one prototype into k_recordBytes at bytes. A phoneme past two
	// code points is cut to its first two.
	static void encodeRecord(const VisemePrototype& prototype, uint8_t* bytes);
};

// Classifies compressed MFCC vectors against a model's prototypes by squared
// Mahalanobis distance, with HeadAudio's vote: the nearest prototype's viseme
// goes into a ring of the last six and the most frequent one there wins,
// ties to the highest id. Sequential over a run, reset between runs.
class VisemeClassifier
{
public:
	// Silence prototypes have their distance divided by this, so a larger
	// value calls silence sooner
	float silSensitivity= 1.2f;

	void setModel(const VisemeModel& model);
	bool hasModel() const { return !m_prototypes.empty(); }

	// Restarts the vote from silence
	void reset();

	// The voted viseme id for one vector; outDistances, when given, gets the
	// distance to every prototype in model order
	int predict(const MfccVector& vector, std::vector<float>* outDistances= nullptr);

	// The squared Mahalanobis distance from a vector to a prototype
	static float distance(const MfccVector& vector, const VisemePrototype& prototype);

private:
	static constexpr int k_voteCount= 6;

	std::vector<VisemePrototype> m_prototypes;
	std::array<int, k_voteCount> m_votes{};
	int m_voteIndex= 0;
};
