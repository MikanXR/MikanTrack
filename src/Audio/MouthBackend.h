#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "AudioTypes.h"

// What the chain runs over each window: one implementation per mouth model.
// A run fills one MouthFrame per 30 fps frame of the window it is handed.
// The MFCC and loudness models are pure over the window (the same samples
// give the same frames); the Oculus engine carries state between runs, so
// it asks for the audio while the gate is closed too and scores only the
// frames the chain will emit.
class IMouthBackend
{
public:
	virtual ~IMouthBackend()= default;

	virtual bool isReady() const= 0;
	// One line for the panel on what runs: the profile state, the engine
	// version
	virtual std::string getInfo() const= 0;
	// Why the backend is not ready, or why it runs degraded; empty when fine
	virtual const std::string& getReason() const= 0;

	// Whether the backend must see the audio while the gate is closed (a
	// stateful engine). The chain still emits the neutral mouth for those
	// runs.
	virtual bool runsWhileGated() const { return false; }

	// The level that leaves the mouth closed and the level that opens it
	// fully, in dBFS, for the models that open the mouth by level
	virtual void setVolumeWindow(float minDbfs, float maxDbfs) {}

	// The engine's own viseme smoothing, OVR_SMOOTHING_MIN..MAX, for the
	// model that has one (the Oculus engine); the others ignore it
	virtual void setEngineSmoothing(int amount) {}

	// Drops whatever carries between runs; called where a stream starts
	virtual void reset() {}

	// Runs over the window and fills outFrames with mouthFramesForSamples
	// frames covering it, first to last. newFrameCount is how many tail
	// frames the chain emits from this run; a stateful backend scores only
	// those and leaves the earlier frames neutral. Returns false on a failed
	// run (outFrames is left empty).
	virtual bool infer(const float* samples, size_t sampleCount, int newFrameCount,
					   std::vector<MouthFrame>& outFrames)= 0;
	// Milliseconds the last infer took, for the panel's readout and the bench
	virtual double getLastInferMs() const= 0;
};

// The backend for a model. The MFCC model loads the profile at the path
// (volume-only for an empty or unloadable one). Returns null for a model this
// build does not carry, with the reason in outReason.
std::unique_ptr<IMouthBackend> createMouthBackend(eMouthModel model, const std::string& profilePath,
												  std::string& outReason);
