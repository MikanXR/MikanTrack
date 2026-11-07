#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "AudioTypes.h"
#include "MfccFeatures.h"
#include "VadGate.h"
#include "VisemeModel.h"
#include "VisemeTypes.h"

// The mouth backend: HeadAudio's method, MFCC vectors per 256-sample hop
// classified against Gaussian prototypes into the 15 Oculus visemes, emitted
// as 30 fps frames of viseme weights. A run is pure over its window (the
// same samples give the same frames): the vote and the crossfade restart
// from silence at the window's start, so the first few hops of a window are
// warm-up and the emitted tail is not. Which frames are new is the window
// policy's business (MouthWindow), and smoothing happens after emission
// (MouthFilter), so the bench can compare chunked and whole-clip output.
//
// The shipped model's silence prototype is the zero vector with a tight
// covariance, which only digital silence reaches, so HeadAudio's energy gate
// runs in front of the classifier: a hop below it is silence and feeds no
// vote, which is what keeps a pause or a quiet room from reading as speech.
class MfccBackend
{
public:
	static constexpr const char* k_defaultModelPath= "resources/audio/model-en-mixed.bin";

	// The crossfade between visemes: a viseme's weight ramps up over this
	// long when it wins and down over the same when it loses, through
	// HeadAudio's sigmoid easing
	float rampMs= 100.f;

	// Loads the model, the shipped one or a calibration profile. A missing or
	// malformed file leaves the backend not ready with the reason set.
	bool create(const std::string& modelPath= k_defaultModelPath);

	bool isReady() const { return m_classifier.hasModel(); }
	// Why the backend is not ready, when it is not
	const std::string& getUnavailableReason() const { return m_unavailableReason; }
	// Runs over the samples and fills outFrames with mouthFramesForSamples
	// frames covering them, first to last. Returns false on a failed run
	// (outFrames is left empty).
	bool infer(const float* samples, size_t sampleCount, std::vector<MouthFrame>& outFrames);
	// Milliseconds the last infer took, for the panel's readout and the bench
	double getLastInferMs() const { return m_lastInferMs; }

	// The voted viseme of each hop of the last run, in hop order
	const std::vector<uint8_t>& getLastHopVisemes() const { return m_hopVisemes; }
	// The silence sensitivity of the classifier
	void setSilSensitivity(float value) { m_classifier.silSensitivity= value; }
	// The energy gate's thresholds
	void configureGate(const VadGateConfig& config) { m_gate.configure(config); }
	const VadGateConfig& getGateConfig() const { return m_gate.getConfig(); }

	// The number of hops a run over sampleCount samples classifies, one per
	// 256 samples once the first 512 are in; 0 below a block
	static int hopsForSamples(size_t sampleCount);

private:
	MfccExtractor m_extractor;
	VadGate m_gate;
	VisemeClassifier m_classifier;
	std::string m_unavailableReason= "not created";
	double m_lastInferMs= 0.0;

	std::vector<float> m_preEmphasized;
	std::vector<uint8_t> m_hopVisemes;
	std::vector<VisemeWeights> m_hopWeights;
};
