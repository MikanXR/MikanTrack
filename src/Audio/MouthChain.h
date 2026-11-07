#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "AudioTypes.h"
#include "MouthFilter.h"
#include "MouthWindow.h"

class MfccBackend;

// The level of a span of samples in dBFS RMS, floored at -100
float audioLevelDbfs(const float* samples, size_t sampleCount);

// The gate a room's silence suggests: the loudest hop of the samples at
// the gain, plus a 6 dB margin, clamped to the gate slider's range
float suggestGateDbfs(const float* samples, size_t sampleCount, float gainDb);

// The noise gate with hysteresis: opens at the threshold, closes 6 dB under
// it, so a level hovering at the threshold does not flutter the mouth
class LevelGate
{
public:
	static constexpr float k_hysteresisDb= 6.f;

	void setThreshold(float openDbfs) { m_openDbfs= openDbfs; }
	void reset() { m_bOpen= false; }
	// Feeds one hop's level and returns whether the gate is open after it
	bool update(float levelDbfs);
	bool isOpen() const { return m_bOpen; }

private:
	float m_openDbfs= -45.f;
	bool m_bOpen= false;
};

struct MouthChainConfig
{
	MouthWindowConfig window;
	MouthFilterConfig filter;
	// Applied to the samples before everything else
	float gainDb= 0.f;
	// Hop level below which the mouth is neutral and no backend runs
	float gateDbfs= -45.f;
	// The backend's volume window sits on the gate: closed at the gate,
	// fully open this far above it
	static constexpr float k_volumeSpanDb= 20.f;
};

// An emitted frame with its index in the stream: frame i scores the audio
// around i/30 s after the stream began, so the index ties a frame to a
// recording of the stream
struct MouthChainFrame
{
	int64_t frameIndex= 0;
	MouthFrame frame;
};

// What one run of the chain saw
struct MouthChainRun
{
	float levelDbfs= -100.f;
	bool bGateOpen= false;
	bool bInferred= false;
	double inferMs= 0.0;
};

// The stage's chain from captured samples to emitted frames: the gain, the
// rolling window, the hop's level through the gate, the backend over the
// window while the gate is open (the neutral mouth otherwise), and the tail
// frames through the filter. The live service and the offline replay both run
// this one chain, so a recording replays through exactly what live ran.
class MouthChain
{
public:
	// Adopts the config and resets, so the stream restarts: for a new stream
	// only, since the frame indices count from the reset
	void configure(const MouthChainConfig& config);
	const MouthChainConfig& getConfig() const { return m_config; }
	// The gain and the gate take effect on the next hop without a reset
	void setGainDb(float gainDb);
	void setGateDbfs(float gateDbfs);
	// Drops the window, the filter state, the gate, and the sample count
	void reset();

	// Pushes captured samples and runs every run they make due. The emitted
	// frames are appended with their stream indices, one MouthChainRun per
	// run. A null or unready backend emits the neutral mouth.
	void push(const float* samples, size_t sampleCount, MfccBackend* backend, std::vector<MouthChainFrame>& outFrames,
			  std::vector<MouthChainRun>& outRuns);

	// Samples pushed since the reset, the stream position the next push
	// starts at
	int64_t getPushedSamples() const { return m_pushedSamples; }

private:
	MouthChainConfig m_config;
	float m_gainLinear= 1.f;
	MouthWindow m_window;
	MouthFilter m_filter;
	LevelGate m_gate;
	int64_t m_pushedSamples= 0;

	std::vector<float> m_scaled;
	std::vector<float> m_windowSamples;
	std::vector<MouthFrame> m_windowFrames;
	std::vector<MouthFrame> m_tailFrames;
};

// Paces a run's burst of frames (a hop's worth at once) out one per frame
// interval, so the mouth moves at 30 fps instead of stepping ten times a
// second. It is polled by the reader, not the audio thread, since a run
// holds that thread for longer than a frame: the burst lands whenever the run
// ends and the reader draws it down on the clock, a jitter buffer whose depth
// settles at the runs' jitter. A reader that polls late finds several frames
// due and takes the newest, the others superseded. A backlog past the cap
// drops its oldest frames.
class MouthPacer
{
public:
	static constexpr size_t k_defaultMaxPending= 6;

	void configure(size_t maxPending= k_defaultMaxPending, double intervalMs= 1000.0 / MOUTH_FRAME_RATE);
	void reset();

	void push(const MouthChainFrame& frame);
	// The newest frame due at nowMs, if any is; the frames it supersedes are
	// dropped
	bool poll(double nowMs, MouthChainFrame& outFrame);

	size_t getPendingCount() const { return m_pending.size(); }
	uint64_t getDroppedFrames() const { return m_droppedFrames; }

private:
	std::deque<MouthChainFrame> m_pending;
	size_t m_maxPending= k_defaultMaxPending;
	double m_intervalMs= 1000.0 / MOUTH_FRAME_RATE;
	double m_nextMs= 0.0;
	uint64_t m_droppedFrames= 0;
};
