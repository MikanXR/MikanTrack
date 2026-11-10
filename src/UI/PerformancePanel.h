#pragma once

#include <cstdint>
#include <vector>

#include "IterationHistory.h"

class VisionThread;

// The Performance panel: the vision thread's last frame loop iteration by
// phase, its hitch count, and a chart of its recent iterations with the
// phases stacked under each total. Always the live vision thread, also while
// a replay is shown.
class PerformancePanel
{
public:
	// bReplayShown: a replay is the shown frame, which the panel says it is
	// not following
	void draw(bool* pOpen, const VisionThread& visionThread, bool bReplayShown);

private:
	// Appends the iterations the vision thread published since the last
	// fetch, keeping at most the ring's capacity
	void fetchHistory(const VisionThread& visionThread);
	void drawChart();

	// Oldest first, numbered without gaps up to m_newestSequence
	std::vector<IterationHistory::Entry> m_history;
	uint64_t m_newestSequence= 0;
	std::vector<IterationHistory::Entry> m_fetched;

	// The chart's series, rebuilt per draw: x is iterations ago, and each
	// phase has its lower and upper edge in the stack
	std::vector<float> m_xs;
	std::vector<std::vector<float>> m_stackEdges;
	std::vector<float> m_totals;
	std::vector<float> m_hitchXs;
	std::vector<float> m_hitchYs;
};
