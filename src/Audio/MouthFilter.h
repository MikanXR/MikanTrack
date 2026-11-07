#pragma once

#include <array>

#include "AudioTypes.h"
#include "OneEuroFilter.h"

struct MouthFilterConfig
{
	// The one-euro cutoffs per column. The LAM post-processing that smooths
	// the decoder's frames lives in Python and is not in the exported graph,
	// so emitted frames are smoothed here instead.
	float minCutoffHz= 3.f;
	float beta= 0.5f;
	float derivativeCutoffHz= 1.f;
};

// A one-euro filter per mouth column over the emitted 30 fps frames
class MouthFilter
{
public:
	void configure(const MouthFilterConfig& config);
	void reset();

	// Filters one emitted frame; dtSeconds is the frame interval
	MouthFrame filter(const MouthFrame& frame, float dtSeconds= 1.f / MOUTH_FRAME_RATE);

private:
	std::array<OneEuroFilter, MOUTH_COLUMN_COUNT> m_columnFilters;
};
