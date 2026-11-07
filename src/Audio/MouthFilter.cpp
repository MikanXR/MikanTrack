#include "MouthFilter.h"

void MouthFilter::configure(const MouthFilterConfig& config)
{
	m_config= config;
	for (OneEuroFilter& visemeFilter : m_visemeFilters)
		visemeFilter.configure(config.minCutoffHz, config.beta, config.derivativeCutoffHz);
	reset();
}

void MouthFilter::setMinCutoffHz(float minCutoffHz)
{
	m_config.minCutoffHz= minCutoffHz;
	for (OneEuroFilter& visemeFilter : m_visemeFilters)
		visemeFilter.configure(m_config.minCutoffHz, m_config.beta, m_config.derivativeCutoffHz);
}

void MouthFilter::reset()
{
	for (OneEuroFilter& visemeFilter : m_visemeFilters)
		visemeFilter.reset();
}

MouthFrame MouthFilter::filter(const MouthFrame& frame, float dtSeconds)
{
	// The filters track the frames even while bypassed, so a cutoff set
	// later eases from the held state instead of restarting
	MouthFrame filtered;
	for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		filtered.weights[viseme]= m_visemeFilters[viseme].filter(frame.weights[viseme], dtSeconds);
	return m_config.minCutoffHz <= 0.f ? frame : filtered;
}
