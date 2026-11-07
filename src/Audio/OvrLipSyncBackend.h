#pragma once

#ifdef MIKANTRACK_OVRLIPSYNC

#include <array>
#include <string>
#include <vector>

#include "MouthBackend.h"
#include "VisemeTypes.h"

// The Oculus LipSync engine (the Oculus Lipsync native SDK, Meta Platforms,
// under the Oculus SDK License; the SDK is installed by hand under deps/).
// One context per backend, fed each new 30 fps frame's samples at 16 kHz
// as one ProcessFrameEx call, its fifteen viseme scores remapped from the
// SDK's order into the stage's and emitted as they come: no volume window,
// no smoothing signal, the engine's own mouth. The engine is stateful, so
// it runs while the gate is closed too and only the frames the chain emits
// are fed, in stream order.
class OvrLipSyncBackend : public IMouthBackend
{
public:
	// The Oculus viseme order (sil, PP, FF, TH, DD, kk, CH, SS, nn, RR,
	// aa, E, ih, oh, ou) as stage viseme indices
	static const std::array<int, VISEME_COUNT>& getVisemeRemap();

	OvrLipSyncBackend();
	~OvrLipSyncBackend() override;
	OvrLipSyncBackend(const OvrLipSyncBackend&)= delete;
	OvrLipSyncBackend& operator=(const OvrLipSyncBackend&)= delete;

	// Loads the engine (the DLL beside the exe) and creates the context.
	// Returns whether the backend is ready; the reason is set otherwise.
	bool create();

	bool isReady() const override { return m_context != 0; }
	std::string getInfo() const override { return m_info; }
	const std::string& getReason() const override { return m_reason; }
	bool runsWhileGated() const override { return true; }
	void reset() override;
	bool infer(const float* samples, size_t sampleCount, int newFrameCount, std::vector<MouthFrame>& outFrames) override;
	double getLastInferMs() const override { return m_lastInferMs; }

private:
	void destroy();

	unsigned int m_context= 0;
	bool m_bEngineHeld= false;
	std::string m_info;
	std::string m_reason;
	double m_lastInferMs= 0.0;
	std::array<float, VISEME_COUNT> m_scores{};
};

#endif
