#include "MouthChain.h"

#include <algorithm>
#include <cmath>

#include "MfccBackend.h"

float audioLevelDbfs(const float* samples, size_t sampleCount)
{
	if (sampleCount == 0)
		return -100.f;
	double sum= 0.0;
	for (size_t index= 0; index < sampleCount; ++index)
		sum+= (double)samples[index] * samples[index];
	const double rms= std::sqrt(sum / (double)sampleCount);
	return std::max(-100.f, 20.f * (float)std::log10(rms + 1e-10));
}

bool LevelGate::update(float levelDbfs)
{
	if (m_bOpen)
		m_bOpen= levelDbfs >= m_openDbfs - k_hysteresisDb;
	else
		m_bOpen= levelDbfs >= m_openDbfs;
	return m_bOpen;
}

void MouthChain::configure(const MouthChainConfig& config)
{
	m_config= config;
	m_window.configure(config.window);
	m_filter.configure(config.filter);
	setGainDb(config.gainDb);
	setGateDbfs(config.gateDbfs);
	reset();
}

void MouthChain::setGainDb(float gainDb)
{
	m_config.gainDb= gainDb;
	m_gainLinear= std::pow(10.f, gainDb / 20.f);
}

void MouthChain::setGateDbfs(float gateDbfs)
{
	m_config.gateDbfs= gateDbfs;
	m_gate.setThreshold(gateDbfs);
}

void MouthChain::reset()
{
	m_window.reset();
	m_filter.reset();
	m_gate.reset();
	m_pushedSamples= 0;
}

void MouthChain::push(const float* samples, size_t sampleCount, MfccBackend* backend,
					  std::vector<MouthChainFrame>& outFrames, std::vector<MouthChainRun>& outRuns)
{
	m_scaled.resize(sampleCount);
	for (size_t index= 0; index < sampleCount; ++index)
		m_scaled[index]= samples[index] * m_gainLinear;
	m_window.push(m_scaled.data(), sampleCount);
	m_pushedSamples+= (int64_t)sampleCount;

	while (m_window.isRunDue())
	{
		const int64_t firstFrameIndex= m_window.getEmittedFrames();
		const int emitCount= m_window.beginRun(m_windowSamples);

		// The hop just consumed is the window's tail
		const size_t hopSamples= std::min(m_windowSamples.size(), (size_t)m_config.window.hopSamples);
		MouthChainRun run;
		run.levelDbfs= audioLevelDbfs(m_windowSamples.data() + m_windowSamples.size() - hopSamples, hopSamples);
		run.bGateOpen= m_gate.update(run.levelDbfs);

		m_tailFrames.clear();
		if (run.bGateOpen && backend != nullptr && backend->isReady() &&
			backend->infer(m_windowSamples.data(), m_windowSamples.size(), m_windowFrames))
		{
			MouthWindow::takeTail(m_windowFrames, emitCount, m_config.window.lookaheadFrames, m_tailFrames);
			run.bInferred= true;
			run.inferMs= backend->getLastInferMs();
		}
		// Gated, or no backend: the neutral mouth, which the filter eases into
		while ((int)m_tailFrames.size() < emitCount)
			m_tailFrames.emplace_back();

		for (size_t index= 0; index < m_tailFrames.size(); ++index)
			outFrames.push_back({firstFrameIndex + (int64_t)index, m_filter.filter(m_tailFrames[index])});
		outRuns.push_back(run);
	}
}

// -- MouthPacer --------------------------------------------------------------

void MouthPacer::configure(size_t maxPending, double intervalMs)
{
	m_maxPending= std::max<size_t>(maxPending, 1);
	m_intervalMs= std::max(intervalMs, 1.0);
	reset();
}

void MouthPacer::reset()
{
	m_pending.clear();
	m_nextMs= 0.0;
	m_droppedFrames= 0;
}

void MouthPacer::push(const MouthChainFrame& frame)
{
	if (m_pending.size() >= m_maxPending)
	{
		m_pending.pop_front();
		++m_droppedFrames;
	}
	m_pending.push_back(frame);
}

bool MouthPacer::poll(double nowMs, MouthChainFrame& outFrame)
{
	if (m_pending.empty())
	{
		// Nothing to pace: the schedule tracks the clock so the next burst
		// does not read as overdue
		m_nextMs= std::max(m_nextMs, nowMs);
		return false;
	}
	if (nowMs < m_nextMs)
		return false;

	// Every frame whose time has passed is due; the newest of them goes out
	size_t due= 1 + (size_t)((nowMs - m_nextMs) / m_intervalMs);
	due= std::min(due, m_pending.size());
	for (size_t index= 1; index < due; ++index)
	{
		m_pending.pop_front();
		++m_droppedFrames;
	}
	outFrame= m_pending.front();
	m_pending.pop_front();
	m_nextMs+= (double)due * m_intervalMs;
	return true;
}
