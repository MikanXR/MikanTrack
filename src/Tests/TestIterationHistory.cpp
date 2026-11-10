#include "TestCommon.h"

#include "IterationHistory.h"

// The frame loop history ring the Performance chart reads, without a vision
// thread: numbering from 1, a fetch returning only what came after the
// reader's newest, oldest first, with the values pushed; wraparound keeping
// the newest capacity entries; a reader that fell behind getting what is
// left and the count it lost; a reader ahead of the ring (one that started
// over) getting nothing; and clear numbering from 1 again.
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

	MIKAN_LOG_INFO(kTag) << (failures == 0 ? "ALL PASSED" : "FAILED") << " (" << failures << " failures)";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-iteration-history",
					"Frame loop history ring: numbering, fetch after the reader's newest, wraparound and loss",
					eTestCategory::SelfTest, runIterationHistoryTest)
