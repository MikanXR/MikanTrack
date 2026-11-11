#pragma once

#include <cstdint>
#include <vector>

#include "IterationHistory.h"

struct RecordedTelemetry;

// One iteration of the frame loop on the Performance panel's chart: where it
// sits on the X axis (how far before the newest or the playhead), its time
// per phase in the vision thread's phase order, its total and whether it was
// a hitch
struct PerformanceSample
{
	float x= 0.f;
	float phaseMs[IterationHistory::k_phaseCount]= {};
	float totalMs= 0.f;
	bool bHitch= false;
};

// What the chart draws, oldest first, whichever source filled it
struct PerformanceSeries
{
	std::vector<PerformanceSample> samples;
	// The sample the readouts show: the newest live, the playhead's frame in
	// a recording; -1 when there is none (an empty history, or a playhead
	// frame that carries no telemetry)
	int markedIndex= -1;
};

// The live history, oldest first and numbered without gaps up to
// newestSequence: x is the iterations before the newest, which is marked
void buildLiveSeries(const std::vector<IterationHistory::Entry>& history, uint64_t newestSequence,
					 PerformanceSeries& outSeries);

// A take's frames, one entry each, null for a frame without telemetry: the
// window of windowLength frames ending at the playhead's frame, x the frames
// before the playhead, which is marked. A frame without telemetry has no
// sample. A recorded hitch is a total at or above the threshold, the rule
// the vision thread flags its iterations by.
void buildRecordedSeries(const std::vector<const RecordedTelemetry*>& frames, int playheadFrame, int windowLength,
						 double hitchThresholdMs, PerformanceSeries& outSeries);

// The hitches over a whole take, by the same rule
int countRecordedHitches(const std::vector<const RecordedTelemetry*>& frames, double hitchThresholdMs);
