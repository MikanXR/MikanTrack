#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "IMouthBackend.h"
#include "OnnxSession.h"

// The wav2arkit_cpu model: raw 16 kHz mono float audio in, 52 ARKit weights
// at 30 fps out, a fused export of the wav2vec2-base-960h encoder and the LAM
// Audio2Expression decoder. The session runs on the CPU provider so the
// DirectML queue stays with the vision models, and the backend must be
// created, run, and destroyed on one thread (OnnxSession's rule).
class Wav2ArkitBackend : public IMouthBackend
{
public:
	static constexpr const char* k_defaultModelPath= "models/wav2arkit_cpu.onnx";

	// Loads the model. preferredEp is OnnxSession's provider string: "cpu"
	// (the default, which keeps the DirectML queue with the vision models)
	// or "directml". intraOpThreads caps a CPU session's compute threads so
	// the audio stage shares the machine with the vision thread; 0 leaves the
	// runtime's default. A missing or unloadable model leaves the backend not
	// ready with the reason set.
	bool create(const std::string& modelPath= k_defaultModelPath, const std::string& preferredEp= "cpu",
				int intraOpThreads= 4);

	const char* getName() const override { return "wav2arkit"; }
	bool isReady() const override { return m_session != nullptr && m_session->isValid(); }
	const std::string& getUnavailableReason() const override { return m_unavailableReason; }
	bool infer(const float* samples, size_t sampleCount, std::vector<MouthFrame>& outFrames) override;
	double getLastInferMs() const override { return m_lastInferMs; }

	// The execution provider the session landed on ("CPU" or "DirectML")
	const char* getActiveEp() const { return m_session != nullptr ? m_session->activeEp() : "none"; }

	// The ARKit index of one of the model's 52 output columns, which it lists
	// in alphabetical order rather than ARKit's; -1 out of range
	static int outputColumnArkitIndex(int outputColumn);

private:
	// The encoder's convolution stack needs at least this many samples
	static constexpr size_t k_minSamples= 400;

	std::unique_ptr<OnnxSession> m_session;
	std::string m_unavailableReason= "not created";
	double m_lastInferMs= 0.0;
	// Output column to mouth column, -1 for the columns the voice does not drive
	std::array<int, ARKIT_BLENDSHAPE_COUNT> m_outputToMouthColumn{};
};
