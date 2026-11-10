#pragma once

#include <cstdint>
#include <vector>

// A fixed-capacity ring of frame loop iterations: per entry the time per
// phase, the total and whether it was a hitch, numbered in the order they
// were pushed. A reader keeps the newest number it holds and fetches only
// what came after it, so nothing is copied twice. Storage is allocated once,
// at construction. Not thread-safe: the owner locks around it.
class IterationHistory
{
public:
	// One slot per vision phase, in the vision thread's phase order
	static constexpr int k_phaseCount= 8;
	static constexpr int k_defaultCapacity= 600;

	struct Entry
	{
		// 1 for the first entry pushed, one more for each after it
		uint64_t sequence= 0;
		float phaseMs[k_phaseCount]= {};
		float totalMs= 0.f;
		bool bHitch= false;
	};

	explicit IterationHistory(int capacity= k_defaultCapacity);

	// Appends one iteration, overwriting the oldest once the ring is full.
	// phaseMs holds k_phaseCount values.
	void push(const float* phaseMs, float totalMs, bool bHitch);

	// Appends to outEntries every held entry numbered after afterSequence,
	// oldest first. Returns how many entries numbered after afterSequence
	// were overwritten before this fetch, 0 when none were lost.
	uint64_t fetchNewerThan(uint64_t afterSequence, std::vector<Entry>& outEntries) const;

	// The newest entry's number, 0 before the first push
	uint64_t getNewestSequence() const { return m_nextSequence - 1; }
	// The oldest entry still held, 0 when the ring is empty
	uint64_t getOldestSequence() const;
	int getCapacity() const { return (int)m_entries.size(); }
	int getCount() const;

	// Empties the ring and numbers the next entry 1 again
	void clear();

private:
	std::vector<Entry> m_entries;
	uint64_t m_nextSequence= 1;
};
