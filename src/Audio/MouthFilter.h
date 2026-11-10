#pragma once

#include <array>

#include "AudioTypes.h"
#include "OneEuroFilter.h"

struct MouthFilterConfig
{
	// The one-euro cutoffs per viseme. A minimum cutoff of 0 is no filter:
	// the frames pass through as emitted.
	float minCutoffHz= 3.f;
	float beta= 0.5f;
	float derivativeCutoffHz= 1.f;
};

// A one-euro filter per viseme over the emitted 30 fps frames: it smooths
// the frames across runs and eases the mouth shut when the gate closes
// instead of snapping it. The Microphone stage's Smoothing slider sets the
// minimum cutoff per model through its settling time.
class MouthFilter
{
public:
	void configure(const MouthFilterConfig& config);
	const MouthFilterConfig& getConfig() const { return m_config; }
	// Changes the minimum cutoff on the fly, keeping the filters' state
	void setMinCutoffHz(float minCutoffHz);
	void reset();

	// Filters one emitted frame; dtSeconds is the frame interval
	MouthFrame filter(const MouthFrame& frame, float dtSeconds= 1.f / MOUTH_FRAME_RATE);

private:
	MouthFilterConfig m_config;
	std::array<OneEuroFilter, VISEME_COUNT> m_visemeFilters;
};
