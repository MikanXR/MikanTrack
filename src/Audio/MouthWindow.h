#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "AudioTypes.h"
#include "MfccFeatures.h"

struct MouthWindowConfig
{
	// How much new audio arrives between runs. 100 ms is three frames, so
	// a replay's frame grid lands exactly on live's.
	int hopSamples= AUDIO_SAMPLE_RATE / 10;
	// The span of audio a backend sees on each run, ending at the newest
	// hop boundary. Two hops: a whole number of frames, so the frame grid
	// the backend lays over the window is the stream's own, and the
	// analysis block the first emitted frame reaches back over is inside it.
	int windowSamples= 2 * (AUDIO_SAMPLE_RATE / 10);
	// Frames held back from the tail of each run, so an emitted frame has
	// that much audio after it for context. The backend scores a frame
	// from the audio up to its end alone, so none is held back; each frame
	// held would cost 33 ms of latency.
	int lookaheadFrames= 0;
};

// The rolling-window policy of the audio stage, pure accounting with no
// backend and no clock. Captured samples are pushed in; once a hop's worth has
// arrived a run is due, the window ending at that hop boundary is handed out
// (zero-padded at the front until enough audio has arrived), and the frames
// of the backend's output that are new since the last run are the tail ones.
// Frames are due at floor(30 * consumedSamples / 16000) less the lookahead,
// so a 1600-sample hop is always three new frames and an odd hop size still
// never skips or repeats a frame.
class MouthWindow
{
public:
	void configure(const MouthWindowConfig& config);
	const MouthWindowConfig& getConfig() const { return m_config; }

	// Drops every sample and the counters
	void reset();

	// Appends captured samples past the current hop boundary
	void push(const float* samples, size_t sampleCount);

	// True while at least one hop of samples sits past the boundary
	bool isRunDue() const { return m_pendingSamples >= (size_t)m_config.hopSamples; }

	// Advances the boundary by one hop, fills outWindow with the
	// windowSamples ending there, and returns how many frames of the
	// backend's output for that window are new (the ones ending lookahead
	// frames before its tail). Call only while isRunDue.
	int beginRun(std::vector<float>& outWindow);

	// Copies the emitCount of windowFrames that end lookaheadFrames before
	// the last onto the end of outFrames, or what the backend returned when
	// it returned fewer
	static void takeTail(const std::vector<MouthFrame>& windowFrames, int emitCount, int lookaheadFrames,
						 std::vector<MouthFrame>& outFrames);

	// Samples consumed up to the current boundary
	int64_t getConsumedSamples() const { return m_consumedSamples; }
	// Frames handed out across every run, which trails the frames due by the
	// lookahead
	int64_t getEmittedFrames() const { return m_emittedFrames; }
	// Samples waiting past the boundary
	size_t getPendingSamples() const { return m_pendingSamples; }

private:
	static int64_t framesDueAt(int64_t consumedSamples);

	MouthWindowConfig m_config;
	// The samples up to the boundary (at most windowSamples of them) followed
	// by the pending ones
	std::vector<float> m_buffer;
	size_t m_pendingSamples= 0;
	int64_t m_consumedSamples= 0;
	int64_t m_emittedFrames= 0;
};
