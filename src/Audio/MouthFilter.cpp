#include "MouthFilter.h"

void MouthFilter::configure(const MouthFilterConfig& config)
{
	for (OneEuroFilter& visemeFilter : m_visemeFilters)
		visemeFilter.configure(config.minCutoffHz, config.beta, config.derivativeCutoffHz);
	reset();
}

void MouthFilter::reset()
{
	for (OneEuroFilter& visemeFilter : m_visemeFilters)
		visemeFilter.reset();
}

MouthFrame MouthFilter::filter(const MouthFrame& frame, float dtSeconds)
{
	MouthFrame filtered;
	for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		filtered.weights[viseme]= m_visemeFilters[viseme].filter(frame.weights[viseme], dtSeconds);
	return filtered;
}
