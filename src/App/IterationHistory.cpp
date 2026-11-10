#include "IterationHistory.h"

#include <algorithm>

IterationHistory::IterationHistory(int capacity)
	: m_entries((size_t)std::max(capacity, 1))
{
}

void IterationHistory::push(const float* phaseMs, float totalMs, bool bHitch)
{
	// The entry numbered n lives in slot (n - 1) modulo the capacity
	const uint64_t sequence= m_nextSequence++;
	Entry& entry= m_entries[(size_t)((sequence - 1) % m_entries.size())];
	entry.sequence= sequence;
	std::copy(phaseMs, phaseMs + k_phaseCount, entry.phaseMs);
	entry.totalMs= totalMs;
	entry.bHitch= bHitch;
}

uint64_t IterationHistory::getOldestSequence() const
{
	const uint64_t newest= getNewestSequence();
	if (newest == 0)
		return 0;
	const uint64_t capacity= m_entries.size();
	return newest > capacity ? newest - capacity + 1 : 1;
}

int IterationHistory::getCount() const
{
	const uint64_t newest= getNewestSequence();
	return newest == 0 ? 0 : (int)(newest - getOldestSequence() + 1);
}

uint64_t IterationHistory::fetchNewerThan(uint64_t afterSequence, std::vector<Entry>& outEntries) const
{
	const uint64_t newest= getNewestSequence();
	if (newest == 0 || afterSequence >= newest)
		return 0;
	const uint64_t oldest= getOldestSequence();
	const uint64_t first= std::max(afterSequence + 1, oldest);
	for (uint64_t sequence= first; sequence <= newest; ++sequence)
		outEntries.push_back(m_entries[(size_t)((sequence - 1) % m_entries.size())]);
	return first - (afterSequence + 1);
}

void IterationHistory::clear()
{
	std::fill(m_entries.begin(), m_entries.end(), Entry());
	m_nextSequence= 1;
}
