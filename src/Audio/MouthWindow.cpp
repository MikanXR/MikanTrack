#include "MouthWindow.h"

#include <algorithm>

void MouthWindow::configure(const MouthWindowConfig& config)
{
	m_config= config;
	if (m_config.windowSamples < 1)
		m_config.windowSamples= 1;
	if (m_config.hopSamples < 1)
		m_config.hopSamples= 1;
	if (m_config.lookaheadFrames < 0)
		m_config.lookaheadFrames= 0;
	reset();
}

void MouthWindow::reset()
{
	m_buffer.clear();
	m_pendingSamples= 0;
	m_consumedSamples= 0;
	m_emittedFrames= 0;
}

void MouthWindow::push(const float* samples, size_t sampleCount)
{
	m_buffer.insert(m_buffer.end(), samples, samples + sampleCount);
	m_pendingSamples+= sampleCount;
}

int64_t MouthWindow::framesDueAt(int64_t consumedSamples)
{
	return consumedSamples * MOUTH_FRAME_RATE / AUDIO_SAMPLE_RATE;
}

int MouthWindow::beginRun(std::vector<float>& outWindow)
{
	const size_t hop= std::min((size_t)m_config.hopSamples, m_pendingSamples);
	m_pendingSamples-= hop;
	m_consumedSamples+= (int64_t)hop;

	// The buffer holds history up to the old boundary plus the pending
	// samples; the window ends at the new boundary, pending samples past it
	// are not in the window yet
	const size_t boundaryEnd= m_buffer.size() - m_pendingSamples;
	const size_t windowSamples= (size_t)m_config.windowSamples;
	const size_t available= std::min(windowSamples, boundaryEnd);
	const size_t padding= windowSamples - available;

	outWindow.resize(windowSamples);
	std::fill(outWindow.begin(), outWindow.begin() + padding, 0.f);
	std::copy(m_buffer.begin() + (boundaryEnd - available), m_buffer.begin() + boundaryEnd,
			  outWindow.begin() + padding);

	// Trim history older than the window
	if (boundaryEnd > windowSamples)
		m_buffer.erase(m_buffer.begin(), m_buffer.begin() + (boundaryEnd - windowSamples));

	// The frames up to the lookahead before the boundary are emitted; the
	// clamp keeps the first runs from emitting the zero padding before the
	// clip's own first frame
	const int64_t due= std::max<int64_t>(framesDueAt(m_consumedSamples) - m_config.lookaheadFrames, 0);
	const int emitCount= (int)(due - m_emittedFrames);
	m_emittedFrames= due;
	return emitCount;
}

void MouthWindow::takeTail(const std::vector<MouthFrame>& windowFrames, int emitCount, int lookaheadFrames,
						   std::vector<MouthFrame>& outFrames)
{
	const size_t heldBack= std::min((size_t)std::max(lookaheadFrames, 0), windowFrames.size());
	const size_t available= windowFrames.size() - heldBack;
	const size_t count= std::min((size_t)std::max(emitCount, 0), available);
	const auto end= windowFrames.end() - (std::ptrdiff_t)heldBack;
	outFrames.insert(outFrames.end(), end - (std::ptrdiff_t)count, end);
}
