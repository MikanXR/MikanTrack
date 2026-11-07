#include "MouthFilter.h"

void MouthFilter::configure(const MouthFilterConfig& config)
{
	for (OneEuroFilter& columnFilter : m_columnFilters)
		columnFilter.configure(config.minCutoffHz, config.beta, config.derivativeCutoffHz);
	reset();
}

void MouthFilter::reset()
{
	for (OneEuroFilter& columnFilter : m_columnFilters)
		columnFilter.reset();
}

MouthFrame MouthFilter::filter(const MouthFrame& frame, float dtSeconds)
{
	MouthFrame filtered;
	for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
		filtered.weights[column]= m_columnFilters[column].filter(frame.weights[column], dtSeconds);
	return filtered;
}
