#include "TestCommon.h"

#include "IterationHistory.h"
#include "PerformanceSeries.h"

// The frame loop history ring the Performance chart reads, without a vision
// thread: numbering from 1, a fetch returning only what came after the
// reader's newest, oldest first, with the values pushed; wraparound keeping
// the newest capacity entries; a reader that fell behind getting what is
// left and the count it lost; a reader ahead of the ring (one that started
// over) getting nothing; and clear numbering from 1 again. Then the chart's
// series from the live history and from a take's recorded frames: the
// window ending at the playhead at the start, the middle and the end, the
// hitch flags, and frames without telemetry.
static int runIterationHistoryTest(const TestArgs& args)
{
	const char* kTag= "test-iteration-history";
	int failures= 0;
	auto check= [&](bool bPass, const char* name) {
		if (bPass)
			MIKAN_LOG_INFO(kTag) << "PASS " << name;
		else
		{
			MIKAN_LOG_ERROR(kTag) << "FAIL " << name;
			failures++;
		}
	};

	// Every value of an entry derives from its push index, so a fetched entry
	// says which push it came from
	auto pushNumbered= [](IterationHistory& history, int pushIndex) {
		float phaseMs[IterationHistory::k_phaseCount];
		for (int phaseIndex= 0; phaseIndex < IterationHistory::k_phaseCount; ++phaseIndex)
			phaseMs[phaseIndex]= (float)(pushIndex * 10 + phaseIndex);
		history.push(phaseMs, (float)pushIndex * 100.f, pushIndex % 3 == 0);
	};
	auto entryMatches= [](const IterationHistory::Entry& entry, int pushIndex) {
		if (entry.sequence != (uint64_t)pushIndex || entry.totalMs != (float)pushIndex * 100.f ||
			entry.bHitch != (pushIndex % 3 == 0))
			return false;
		for (int phaseIndex= 0; phaseIndex < IterationHistory::k_phaseCount; ++phaseIndex)
		{
			if (entry.phaseMs[phaseIndex] != (float)(pushIndex * 10 + phaseIndex))
				return false;
		}
		return true;
	};
	// The entries are exactly the pushes first through last, in order
	auto entriesAre= [&](const std::vector<IterationHistory::Entry>& entries, int first, int last) {
		if ((int)entries.size() != last - first + 1)
			return false;
		for (int index= 0; index < (int)entries.size(); ++index)
		{
			if (!entryMatches(entries[index], first + index))
				return false;
		}
		return true;
	};

	{
		IterationHistory history(4);
		std::vector<IterationHistory::Entry> entries;
		const uint64_t lost= history.fetchNewerThan(0, entries);
		check(entries.empty() && lost == 0 && history.getNewestSequence() == 0 && history.getCount() == 0 &&
				  history.getOldestSequence() == 0,
			  "(a) an empty ring fetches nothing and numbers nothing");
	}

	{
		IterationHistory history(4);
		for (int pushIndex= 1; pushIndex <= 3; ++pushIndex)
			pushNumbered(history, pushIndex);
		std::vector<IterationHistory::Entry> entries;
		uint64_t lost= history.fetchNewerThan(0, entries);
		check(entriesAre(entries, 1, 3) && lost == 0 && history.getNewestSequence() == 3 && history.getCount() == 3,
			  "(b) entries are numbered from 1 and fetched oldest first with their values");

		entries.clear();
		lost= history.fetchNewerThan(1, entries);
		check(entriesAre(entries, 2, 3) && lost == 0, "(c) a fetch returns only the entries after the reader's newest");

		entries.clear();
		lost= history.fetchNewerThan(3, entries);
		check(entries.empty() && lost == 0, "(d) a reader holding the newest fetches nothing");

		pushNumbered(history, 4);
		entries.clear();
		lost= history.fetchNewerThan(3, entries);
		check(entriesAre(entries, 4, 4) && lost == 0, "(e) the next fetch returns only the entry pushed since");
	}

	{
		IterationHistory history(4);
		for (int pushIndex= 1; pushIndex <= 10; ++pushIndex)
			pushNumbered(history, pushIndex);
		check(history.getCount() == 4 && history.getOldestSequence() == 7 && history.getNewestSequence() == 10,
			  "(f) a wrapped ring holds the newest capacity entries");

		std::vector<IterationHistory::Entry> entries;
		uint64_t lost= history.fetchNewerThan(0, entries);
		check(entriesAre(entries, 7, 10) && lost == 6,
			  "(g) a reader that fell behind gets what is left, oldest first, and the count it lost");

		entries.clear();
		lost= history.fetchNewerThan(8, entries);
		check(entriesAre(entries, 9, 10) && lost == 0, "(h) a fetch inside the held range after wrapping loses nothing");

		entries.clear();
		lost= history.fetchNewerThan(6, entries);
		check(entriesAre(entries, 7, 10) && lost == 0, "(i) a reader exactly one before the oldest loses nothing");

		entries.clear();
		lost= history.fetchNewerThan(25, entries);
		check(entries.empty() && lost == 0, "(j) a reader ahead of the ring fetches nothing");

		// A fetch appends, so a reader's buffer keeps what it held
		entries.clear();
		pushNumbered(history, 11);
		history.fetchNewerThan(9, entries);
		history.fetchNewerThan(10, entries);
		check(entries.size() == 3 && entryMatches(entries[0], 10) && entryMatches(entries[1], 11) &&
				  entryMatches(entries[2], 11),
			  "(k) a fetch appends to the reader's entries");
	}

	{
		IterationHistory history(4);
		for (int pushIndex= 1; pushIndex <= 6; ++pushIndex)
			pushNumbered(history, pushIndex);
		history.clear();
		std::vector<IterationHistory::Entry> entries;
		history.fetchNewerThan(0, entries);
		const bool bEmpty= entries.empty() && history.getNewestSequence() == 0;
		pushNumbered(history, 1);
		history.fetchNewerThan(0, entries);
		check(bEmpty && entriesAre(entries, 1, 1), "(l) clear empties the ring and numbers from 1 again");
	}

	{
		IterationHistory history;
		check(history.getCapacity() == IterationHistory::k_defaultCapacity, "(m) the default capacity");
	}

	// The chart's series from the live history: x counts back from the
	// newest, which is marked
	{
		IterationHistory history(4);
		for (int pushIndex= 1; pushIndex <= 6; ++pushIndex)
			pushNumbered(history, pushIndex);
		std::vector<IterationHistory::Entry> entries;
		history.fetchNewerThan(0, entries);
		PerformanceSeries series;
		buildLiveSeries(entries, history.getNewestSequence(), series);
		bool bOk= series.samples.size() == 4 && series.markedIndex == 3;
		for (int index= 0; bOk && index < (int)series.samples.size(); ++index)
		{
			const PerformanceSample& sample= series.samples[index];
			const int pushIndex= 3 + index;
			bOk= sample.x == (float)(6 - pushIndex) && sample.totalMs == (float)pushIndex * 100.f &&
				sample.bHitch == (pushIndex % 3 == 0) && sample.phaseMs[2] == (float)(pushIndex * 10 + 2);
		}
		PerformanceSeries emptySeries;
		buildLiveSeries({}, 0, emptySeries);
		check(bOk && emptySeries.samples.empty() && emptySeries.markedIndex == -1,
			  "(n) the live series counts back from the newest and marks it");
	}

	// The chart's series from a take: 10 synthetic frames, totals of 10 ms
	// per frame index, every third frame over the hitch threshold, frames 4
	// and 7 without telemetry
	{
		const double hitchMs= 50.0;
		std::vector<RecordedTelemetry> telemetry(10);
		std::vector<const RecordedTelemetry*> frames;
		for (int frameIndex= 0; frameIndex < 10; ++frameIndex)
		{
			RecordedTelemetry& frame= telemetry[frameIndex];
			frame.present= frameIndex != 4 && frameIndex != 7;
			for (int phaseIndex= 0; phaseIndex < RecordedTelemetry::k_phaseCount; ++phaseIndex)
				frame.phaseMs[phaseIndex]= (float)(frameIndex * 10 + phaseIndex);
			frame.totalMs= frameIndex % 3 == 0 ? 60.f + (float)frameIndex : 10.f + (float)frameIndex;
			frames.push_back(frame.present ? &frame : nullptr);
		}
		// A frame whose block is there but marked absent counts as none
		frames[7]= &telemetry[7];

		// Each sample is the frame it says it is: x back from the playhead
		auto sampleIsFrame= [&](const PerformanceSample& sample, int playhead, int frameIndex) {
			return sample.x == (float)(playhead - frameIndex) && sample.totalMs == telemetry[frameIndex].totalMs &&
				sample.phaseMs[3] == (float)(frameIndex * 10 + 3) && sample.bHitch == (frameIndex % 3 == 0);
		};

		PerformanceSeries series;
		buildRecordedSeries(frames, 0, 600, hitchMs, series);
		check(series.samples.size() == 1 && series.markedIndex == 0 && sampleIsFrame(series.samples[0], 0, 0) &&
				  series.samples[0].bHitch,
			  "(o) a playhead at the start charts its own frame alone, marked, a hitch by its total");

		buildRecordedSeries(frames, 5, 4, hitchMs, series);
		check(series.samples.size() == 3 && series.markedIndex == 2 && sampleIsFrame(series.samples[0], 5, 2) &&
				  sampleIsFrame(series.samples[1], 5, 3) && sampleIsFrame(series.samples[2], 5, 5),
			  "(p) a playhead in the middle charts the window ending at it, skipping a frame without telemetry");

		buildRecordedSeries(frames, 9, 600, hitchMs, series);
		bool bAllFrames= series.samples.size() == 8 && series.markedIndex == 7;
		int hitchSamples= 0;
		const int expectedFrames[]= {0, 1, 2, 3, 5, 6, 8, 9};
		for (int index= 0; bAllFrames && index < 8; ++index)
		{
			bAllFrames= sampleIsFrame(series.samples[index], 9, expectedFrames[index]);
			hitchSamples+= series.samples[index].bHitch ? 1 : 0;
		}
		check(bAllFrames && hitchSamples == 4,
			  "(q) a playhead at the end charts every frame with telemetry, the hitches flagged");

		buildRecordedSeries(frames, 7, 600, hitchMs, series);
		check(series.samples.size() == 6 && series.markedIndex == -1 && sampleIsFrame(series.samples[5], 7, 6),
			  "(r) a playhead frame without telemetry is not marked, the frames before it still charted");

		buildRecordedSeries(frames, 99, 3, hitchMs, series);
		const bool bClamped= series.samples.size() == 2 && series.markedIndex == 1 &&
			sampleIsFrame(series.samples[0], 9, 8) && sampleIsFrame(series.samples[1], 9, 9);
		buildRecordedSeries({}, 0, 600, hitchMs, series);
		check(bClamped && series.samples.empty() && series.markedIndex == -1,
			  "(s) a playhead past the end is the last frame, and an empty take charts nothing");

		check(countRecordedHitches(frames, hitchMs) == 4, "(t) the take's hitches are counted over every frame");
	}

	MIKAN_LOG_INFO(kTag) << (failures == 0 ? "ALL PASSED" : "FAILED") << " (" << failures << " failures)";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-iteration-history",
					"Frame loop history ring and the Performance chart's live and recorded series",
					eTestCategory::SelfTest, runIterationHistoryTest)
