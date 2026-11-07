#pragma once

#include <string>
#include <vector>

#include "MouthBackend.h"

// The simplest mouth: each 30 fps frame's level, read over the same
// 1024-sample block the MFCC model analyzes, through the volume window opens
// aa, and silence is the remainder. No calibration, no vowels.
class LoudnessBackend : public IMouthBackend
{
public:
	bool isReady() const override { return true; }
	std::string getInfo() const override { return "loudness"; }
	const std::string& getReason() const override { return m_reason; }
	void setVolumeWindow(float minDbfs, float maxDbfs) override;
	bool infer(const float* samples, size_t sampleCount, int newFrameCount, std::vector<MouthFrame>& outFrames) override;
	double getLastInferMs() const override { return m_lastInferMs; }

private:
	std::string m_reason;
	float m_minDbfs= -45.f;
	float m_maxDbfs= -25.f;
	double m_lastInferMs= 0.0;
};
