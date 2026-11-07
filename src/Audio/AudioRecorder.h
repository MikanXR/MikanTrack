#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "WavFile.h"

// Asynchronous wav writer for the audio sidecar of a tracking recording: the
// microphone as captured, 16 kHz mono 16-bit PCM, before the gain and the
// gate, so a replay can rerun the whole chain under the recorded settings or
// others. The audio thread hands over each captured block and the writer
// thread converts and writes it.
//
// A queue overflow neither aborts nor shifts the file: the dropped blocks are
// counted and written as silence in their place when the queue has room
// again, so a sample's index in the file stays its index in the stream, which
// is what ties the recorded mouth frames to the audio.
//
// THREAD AFFINITY: start/enqueue/stop are called under the audio service's
// lock; the getters are safe from any thread (atomics).
class AudioRecorder
{
public:
	~AudioRecorder() { stop(); }

	// Opens the file and spawns the writer thread. Returns false (and records
	// nothing) when the file cannot be opened.
	bool start(const std::string& filePath);

	// Queues captured samples, any count
	void enqueue(const float* samples, int sampleCount);

	// Flushes the queue, finishes the file, joins the writer thread. Safe to
	// call when not recording.
	void stop();

	bool isRecording() const { return m_bRecording; }
	// Samples accepted so far, the dropped ones counted as the silence that
	// stands in for them
	uint64_t getSamplesQueued() const { return m_samplesQueued; }
	uint64_t getSamplesWritten() const { return m_samplesWritten; }
	uint64_t getDroppedBlocks() const { return m_droppedBlocks; }
	const std::string& getFilePath() const { return m_filePath; }

private:
	struct QueuedBlock
	{
		static constexpr int k_capacity= 256;
		std::array<float, k_capacity> samples{};
		int count= 0;
		// Silence to write ahead of this block for the blocks dropped before
		// it was accepted
		int64_t silentSamplesBefore= 0;
	};

	void writerLoop();

	// About four seconds of blocks; the writer only falls behind on disk
	// stalls
	static constexpr size_t k_maxQueuedBlocks= 256;

	std::string m_filePath;
	WavWriter m_writer;

	std::thread m_writerThread;
	std::mutex m_queueMutex;
	std::condition_variable m_queueCondition;
	std::deque<QueuedBlock> m_queue;
	bool m_bWriterShouldExit= false;
	// Dropped samples not yet accounted for by an accepted block
	int64_t m_pendingSilentSamples= 0;

	std::atomic_bool m_bRecording{false};
	std::atomic<uint64_t> m_samplesQueued{0};
	std::atomic<uint64_t> m_samplesWritten{0};
	std::atomic<uint64_t> m_droppedBlocks{0};
};
