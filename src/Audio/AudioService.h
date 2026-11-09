#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "readerwriterqueue.h"

#include "AudioTypes.h"
#include "MouthChain.h"

class AudioRecorder;
class IMouthBackend;

struct AudioServiceConfig
{
	bool enabled= false;
	// The capture device by name; empty or unknown takes the system default
	std::string deviceName;
	// Applied to the samples before everything else
	float gainDb= 0.f;
	// Hop level below which the mouth is neutral and no backend runs
	float gateDbfs= -45.f;
	// The mouth model the backend runs
	eMouthModel model= eMouthModel::Mfcc;
	// The calibration profile the MFCC model loads; empty is volume-only
	std::string profilePath;
	// The emission filter's settling time for the model, ms, 0 for none
	float smoothingMs= MOUTH_SMOOTHING_DEFAULT_MS[(int)eMouthModel::Mfcc];
	// The Oculus engine's own smoothing, OVR_SMOOTHING_MIN..MAX
	int engineSmoothing= OVR_SMOOTHING_DEFAULT;
};

// Snapshot for the panel
struct AudioServiceStatus
{
	bool enabled= false;
	bool deviceOpen= false;
	// The device that opened, "default" for the system default
	std::string deviceName;
	int deviceSampleRate= 0;
	bool backendReady= false;
	// What the backend runs: the profile state, the engine version
	std::string backendInfo;
	// Why the backend is not ready or runs degraded, empty when fine
	std::string backendReason;
	// The newest hop's level after gain
	float levelDbfs= -100.f;
	bool gateOpen= false;
	// The backend's run time, a one-second EMA
	float inferMsEma= 0.f;
	// Capture blocks dropped because the audio thread fell behind
	uint64_t droppedBlocks= 0;
	// The audio sidecar of a tracking recording, while one is written
	bool recording= false;
	uint64_t recordedSamples= 0;
	uint64_t recordingDroppedBlocks= 0;
	MouthSample latest;
};

// Where a recording's audio sidecar began in the capture stream
struct AudioRecordingStart
{
	// The stream index of the sidecar's first sample, so a recorded frame's
	// audioFrameIndex finds its audio
	int64_t startSampleIndex= 0;
	// The stream the index counts in
	uint32_t captureEpoch= 0;
	// steadyNowMs() when the sidecar began
	double timestampMs= 0.0;
};

// The microphone mouth stage: SDL captures the chosen device as 16 kHz mono
// float, its callback hands fixed blocks to the audio thread over an SPSC
// queue, and the thread runs the chain (the gain, the rolling window, the
// gate, the backend, the filter) and pushes the emitted frames
// into the pacer. Readers draw the pacer down one frame per frame interval
// into the newest MouthSample, latest-wins behind a mutex: the vision thread
// reads it once per iteration beside the face sample. The backend is
// created, run, and destroyed on the audio thread alone, the one-thread rule
// of its session. While a tracking recording runs, the raw blocks also go to
// its wav sidecar.
class AudioService
{
public:
	AudioService();
	~AudioService();

	// Initializes SDL audio and spawns the thread. Opens no device: setConfig
	// does, once enabled.
	bool startup();
	void shutdown();

	// Thread-safe. The audio thread applies it: a device or enable change
	// reopens the device, a profile change recreates the backend, the gain
	// and the gate take effect on the next hop. A config set before
	// startup() is applied by it.
	void setConfig(const AudioServiceConfig& config);

	// Newest sample, after releasing the frames due now; false before the
	// first frame is emitted
	bool getLatestSample(MouthSample& outSample);

	AudioServiceStatus getStatus();

	// The capture devices SDL sees, by name, after startup
	std::vector<std::string> listCaptureDevices() const;

	// Thread-safe. Starts writing the captured stream to a wav sidecar and
	// reports where in the stream it begins. False, and nothing written,
	// while no device is open or the file cannot be opened. A device reopen
	// ends the sidecar, since the stream restarts.
	bool startRecording(const std::string& wavPath, AudioRecordingStart& outStart);
	void stopRecording();

	// Thread-safe. Collects the raw captured stream from the next block on,
	// up to maxSeconds, for the calibration wizard's prompts. False while no
	// device is open. A device reopen or close drops the collection.
	bool startCapture(float maxSeconds);
	// Ends the collection and hands over what it holds, empty when none ran
	void stopCapture(std::vector<float>& outSamples);
	bool isCapturing() const;

private:
	// A fixed block of captured samples, the unit on the capture queue
	struct AudioBlock
	{
		static constexpr int k_capacity= 256;
		std::array<float, k_capacity> samples{};
		int count= 0;
	};

	static void captureCallback(void* userdata, uint8_t* stream, int lengthBytes);
	void onCaptured(const float* samples, int sampleCount);

	void threadLoop();
	// Audio thread: picks up a config the UI set
	void applyPendingConfig();
	void openDevice();
	void closeDevice();
	void createBackend();
	void destroyBackend();
	// Audio thread: one captured block to the sidecar, then through the chain
	void processBlock(const AudioBlock& block);
	// Under the lock: the frames due at nowMs into the latest sample
	void releaseDueFramesLocked(double nowMs);
	// Under the lock: ends the sidecar
	void stopRecordingLocked(const char* why);

	mutable std::mutex m_mutex;
	AudioServiceConfig m_config;
	bool m_bConfigPending= false;
	bool m_bStarted= false;
	bool m_bAudioSubsystem= false;
	AudioServiceStatus m_status;
	// The stream position and epoch, under the lock so a sidecar start reads
	// the exact index of the next block
	int64_t m_capturedSamples= 0;
	uint32_t m_captureEpoch= 0;
	std::unique_ptr<AudioRecorder> m_recorder;
	// The wizard's collection of the raw stream, reserved at its start so the
	// audio thread only appends
	bool m_bCapturing= false;
	std::vector<float> m_capture;
	// Emitted frames waiting for their frame interval, drawn down by readers
	MouthPacer m_pacer;
	uint64_t m_sequence= 0;

	std::atomic_bool m_bRunning{false};
	std::thread m_thread;

	// Capture callback (producer) to the audio thread (consumer)
	moodycamel::BlockingReaderWriterQueue<AudioBlock> m_blocks;
	AudioBlock m_partialBlock;
	std::atomic<uint64_t> m_droppedBlocks{0};

	// Audio thread state
	AudioServiceConfig m_threadConfig;
	uint32_t m_device= 0;
	std::unique_ptr<IMouthBackend> m_backend;
	MouthChain m_chain;
	std::vector<MouthChainFrame> m_chainFrames;
	std::vector<MouthChainRun> m_chainRuns;
	float m_inferMsEma= 0.f;
	bool m_bInferEmaValid= false;
};
