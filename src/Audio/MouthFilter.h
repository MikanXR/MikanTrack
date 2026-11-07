#pragma once

#include <array>

#include "AudioTypes.h"
#include "OneEuroFilter.h"

struct MouthFilterConfig
{
	// The one-euro cutoffs per viseme
	float minCutoffHz= 3.f;
	float beta= 0.5f;
	float derivativeCutoffHz= 1.f;
};

// A one-euro filter per viseme over the emitted 30 fps frames. The backend's
// crossfade already ramps between the hops' votes inside a run; this smooths
// the frames across runs and eases the mouth shut when the gate closes
// instead of snapping it.
class MouthFilter
{
public:
	void configure(const MouthFilterConfig& config);
	void reset();

	// Filters one emitted frame; dtSeconds is the frame interval
	MouthFrame filter(const MouthFrame& frame, float dtSeconds= 1.f / MOUTH_FRAME_RATE);

private:
	std::array<OneEuroFilter, VISEME_COUNT> m_visemeFilters;
};
