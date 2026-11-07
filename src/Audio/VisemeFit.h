#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "MfccFeatures.h"
#include "VadGate.h"
#include "VisemeModel.h"

// The calibration fit behind the audio wizard: a held vowel's compressed
// MFCC vectors to one Gaussian prototype, and the shipped model with its
// vowel prototypes replaced by the fitted ones. A profile is that overlay
// written as a model file of its own.

// A fit needs at least one more vector than the coefficient count, the rank
// the covariance needs
constexpr int VISEME_FIT_MIN_VECTORS= MFCC_COEFF_COUNT + 1;

// Added to the diagonal of the sample covariance before it is inverted, so
// a vowel held too steadily does not get a prototype so tight that the same
// vowel in speech misses it. About a third of the shipped prototypes'
// typical per-coefficient variance.
constexpr float VISEME_FIT_VARIANCE_FLOOR= 0.25f;

// The group byte of a fitted prototype, which tells it from the shipped
// model's (group 0) in a dump
constexpr uint8_t VISEME_FIT_GROUP= 1;

// The compressed MFCC vectors of the hops of a clip the energy gate passes,
// through the mfcc backend's own chain (pre-emphasis, the extractor, the
// compression), so a fit reads the vectors the classifier will see. The gate
// restarts at the clip's start.
void extractSpeechVectors(const float* samples, size_t sampleCount, const VadGateConfig& gateConfig,
						  std::vector<MfccVector>& outVectors);

// Fits one prototype: the mean of the vectors and the inverse of their
// sample covariance with the variance floor on its diagonal. False with the
// reason under the minimum count or when the covariance does not invert.
bool fitVisemePrototype(const std::vector<MfccVector>& vectors, int viseme, const std::string& phoneme,
						VisemePrototype& outPrototype, std::string& outError);

// The base model with every prototype of a fitted prototype's viseme
// dropped and the fitted ones appended, in their order
VisemeModel overlayVisemePrototypes(const VisemeModel& base, const std::vector<VisemePrototype>& fitted);

// The gate threshold a stretch of room silence suggests: its loudest hop in
// dBFS, after the gain the stage applies, plus a 6 dB margin, clamped to the
// gate's range
float suggestGateDbfs(const float* samples, size_t sampleCount, float gainDb);
