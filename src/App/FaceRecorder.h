#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "FaceSources.h"
#include "FaceTypes.h"

// One phone face sample as the vision thread saw it, the line format of the
// face sidecar: {"seq", "t", "tracked", "w"[52], "head"[3]}. t is
// steadyNowMs() at receipt, the recording's clock. head is present only when
// the phone sent one, and a sidecar written before the key carried none.
struct RecordedFaceSample
{
	uint64_t sequence= 0;
	double timestampMs= 0.0;
	bool bTracked= false;
	std::array<float, ARKIT_BLENDSHAPE_COUNT> blendshapes{};
	// The phone's head rotation as it sent it, Euler degrees in the phone's
	// own axis convention, unconverted
	bool bHasHead= false;
	glm::vec3 headEulerDegrees{0.f};
};

// Asynchronous writer for the face sidecar of a tracking recording,
// `<stem>.face.jsonl`: one line per phone sample, at the phone's own rate,
// beside the frame stream (whose records stay one per fusion iteration).
// The vision thread hands over each new sample and the writer thread formats
// and writes it. A queue overflow drops samples and counts them.
//
// THREAD AFFINITY: start/enqueue/stop on the vision thread; the getters are
// safe from any thread.
class FaceRecorder
{
public:
	~FaceRecorder() { stop(); }

	// Opens the file and spawns the writer thread. False (and records
	// nothing) when the file cannot be opened.
	bool start(const std::string& filePath);
	void enqueue(const RecordedFaceSample& sample);
	// Flushes the queue, closes the file, joins the writer thread. Deletes
	// the file when no sample was written, so a recording with no phone
	// leaves no sidecar. Safe to call when not recording.
	void stop();

	bool isRecording() const { return m_bRecording; }
	uint64_t getSamplesWritten() const { return m_samplesWritten; }
	uint64_t getDroppedSamples() const { return m_droppedSamples; }

private:
	void writerLoop();

	// About ten seconds of a 60 Hz phone; the writer only falls behind on
	// disk stalls
	static constexpr size_t k_maxQueuedSamples= 600;

	std::ofstream m_file;
	std::string m_filePath;
	std::thread m_writerThread;
	std::mutex m_queueMutex;
	std::condition_variable m_queueCondition;
	std::deque<RecordedFaceSample> m_queue;
	bool m_bWriterShouldExit= false;

	std::atomic_bool m_bRecording{false};
	std::atomic<uint64_t> m_samplesWritten{0};
	std::atomic<uint64_t> m_droppedSamples{0};
};

namespace FaceSidecar
{
std::string sampleToLine(const RecordedFaceSample& sample);
bool sampleFromLine(const std::string& line, RecordedFaceSample& outSample);
// Reads a whole sidecar, in file order. A torn final line ends the read
// (a crash-truncated recording); false only when the file cannot be opened.
bool load(const std::string& filePath, std::vector<RecordedFaceSample>& outSamples, std::string& outError);
// The newest sample at or before timestampMs, null when none is. samples
// must be in timestamp order, as the sidecar writes them.
const RecordedFaceSample* findAtOrBefore(const std::vector<RecordedFaceSample>& samples, double timestampMs);
// The phone source at timestampMs from the newest sample at or before it:
// present while that sample is younger than streamingWindowMs, tracked from
// the sample, blendshapes only when tracked. All false and zero when no
// sample qualifies.
void samplePhone(const std::vector<RecordedFaceSample>& samples, double timestampMs, double streamingWindowMs,
				 FaceSources::Phone& outPhone);
} // namespace FaceSidecar
