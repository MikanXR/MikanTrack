#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "AudioTypes.h"
#include "LipSyncProfile.h"
#include "MfccFeatures.h"
#include "MouthBackend.h"
#include "VisemeTypes.h"

// The MFCC mouth model: uLipSync's method (MIT, hecomi). Each 30 fps frame
// scores the 1024-sample block ending at the frame against the profile's
// phoneme templates, the normalized scores fold onto the vowels, and the
// block's level through the volume window opens the mouth: a vowel's weight
// is its share times the volume, silence is what the vowels leave. Without
// a profile the stage is volume-only, aa at the volume, so the jaw tracks
// speech before a calibration exists. A run is pure over its window (the
// same samples give the same frames) and nothing carries between frames;
// which frames are new is the window policy's business (MouthWindow), and
// smoothing happens after emission (MouthFilter).
class MfccBackend : public IMouthBackend
{
public:
	// What one frame of the last run scored, for the bench and the probe
	struct FrameScores
	{
		float levelDbfs= -100.f;
		// The level through the volume window, 0..1
		float volume= 0.f;
		LipSyncScores scores;
	};

	// Loads the profile at the path, or none for an empty path. The backend
	// is ready either way; a file that fails to load leaves it volume-only
	// with the reason set. Returns whether a profile loaded.
	bool create(const std::string& profilePath);

	bool isReady() const override { return m_bCreated; }
	// "uLipSync profile, N phonemes" or "no profile, volume only"
	std::string getInfo() const override;
	// Why no profile loaded, empty while one did or none was asked for
	const std::string& getReason() const override { return m_profileError; }
	bool hasProfile() const { return !m_profile.empty(); }
	const LipSyncProfile& getProfile() const { return m_profile; }

	// The level that leaves the mouth closed and the level that opens it
	// fully, in dBFS
	void setVolumeWindow(float minDbfs, float maxDbfs) override;

	// Every frame of the window is scored, whatever the chain emits
	bool infer(const float* samples, size_t sampleCount, int newFrameCount, std::vector<MouthFrame>& outFrames) override;
	double getLastInferMs() const override { return m_lastInferMs; }
	// The scores of each frame of the last run, in frame order
	const std::vector<FrameScores>& getLastFrameScores() const { return m_frameScores; }

private:
	MfccExtractor m_extractor;
	LipSyncProfile m_profile;
	bool m_bCreated= false;
	std::string m_profileError;
	float m_minDbfs= -45.f;
	float m_maxDbfs= -25.f;
	double m_lastInferMs= 0.0;

	std::array<float, LIPSYNC_BLOCK_SAMPLES> m_block{};
	std::vector<FrameScores> m_frameScores;
};
