#include "PerformanceSeries.h"

#include <algorithm>

#include "TrackingRecording.h"

static_assert(RecordedTelemetry::k_phaseCount == IterationHistory::k_phaseCount,
			  "A recorded frame and a history entry must hold the same phases");

void buildLiveSeries(const std::vector<IterationHistory::Entry>& history, uint64_t newestSequence,
					 PerformanceSeries& outSeries)
{
	outSeries.samples.clear();
	outSeries.samples.reserve(history.size());
	for (const IterationHistory::Entry& entry : history)
	{
		PerformanceSample sample;
		sample.x= (float)(newestSequence - entry.sequence);
		std::copy(entry.phaseMs, entry.phaseMs + IterationHistory::k_phaseCount, sample.phaseMs);
		sample.totalMs= entry.totalMs;
		sample.bHitch= entry.bHitch;
		outSeries.samples.push_back(sample);
	}
	outSeries.markedIndex= outSeries.samples.empty() ? -1 : (int)outSeries.samples.size() - 1;
}

void buildRecordedSeries(const std::vector<const RecordedTelemetry*>& frames, int playheadFrame, int windowLength,
						 double hitchThresholdMs, PerformanceSeries& outSeries)
{
	outSeries.samples.clear();
	outSeries.markedIndex= -1;
	if (frames.empty() || windowLength <= 0)
		return;
	const int playhead= std::clamp(playheadFrame, 0, (int)frames.size() - 1);
	const int firstFrame= std::max(0, playhead - windowLength + 1);
	for (int frameIndex= firstFrame; frameIndex <= playhead; ++frameIndex)
	{
		const RecordedTelemetry* telemetry= frames[frameIndex];
		if (telemetry == nullptr || !telemetry->present)
			continue;
		PerformanceSample sample;
		sample.x= (float)(playhead - frameIndex);
		std::copy(telemetry->phaseMs, telemetry->phaseMs + RecordedTelemetry::k_phaseCount, sample.phaseMs);
		sample.totalMs= telemetry->totalMs;
		sample.bHitch= (double)telemetry->totalMs >= hitchThresholdMs;
		if (frameIndex == playhead)
			outSeries.markedIndex= (int)outSeries.samples.size();
		outSeries.samples.push_back(sample);
	}
}

int countRecordedHitches(const std::vector<const RecordedTelemetry*>& frames, double hitchThresholdMs)
{
	int hitchCount= 0;
	for (const RecordedTelemetry* telemetry : frames)
	{
		if (telemetry != nullptr && telemetry->present && (double)telemetry->totalMs >= hitchThresholdMs)
			++hitchCount;
	}
	return hitchCount;
}
