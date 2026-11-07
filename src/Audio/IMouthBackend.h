#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "AudioTypes.h"

// A backend turns a window of 16 kHz mono audio into 30 fps mouth frames.
// Backends are pure over the window: the same samples give the same frames,
// and the window policy (MouthWindow) decides which of them are new. Any
// temporal smoothing happens after emission (MouthFilter), so the bench can
// compare a backend's chunked and whole-clip output without it.
class IMouthBackend
{
public:
	virtual ~IMouthBackend()= default;

	// The config name of the backend ("wav2arkit", "mfcc")
	virtual const char* getName() const= 0;

	// False when the backend cannot run (its model file is missing, its
	// profile is unreadable); getUnavailableReason says why
	virtual bool isReady() const= 0;
	virtual const std::string& getUnavailableReason() const= 0;

	// Runs over the samples and fills outFrames with mouthFramesForSamples
	// frames covering them, first to last. Returns false on a failed run
	// (outFrames is left empty).
	virtual bool infer(const float* samples, size_t sampleCount, std::vector<MouthFrame>& outFrames)= 0;

	// Milliseconds the last infer took, for the panel's readout and the bench
	virtual double getLastInferMs() const= 0;
};
