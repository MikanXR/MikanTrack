#include "AudioService.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "SDL.h"

#include "AudioRecorder.h"
#include "Logger.h"
#include "MouthBackend.h"
#include "SteadyClock.h"
#include "ThreadUtils.h"

namespace
{
// Captured blocks the queue holds, about a second
constexpr size_t k_queueBlocks= 64;
constexpr float k_inferEmaAlpha= 0.1f;
} // namespace

AudioService::AudioService()
	: m_blocks(k_queueBlocks)
{
}

AudioService::~AudioService()
{
	shutdown();
}

bool AudioService::startup()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	if (m_bStarted)
		return true;

	if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0)
	{
		MIKAN_LOG_ERROR("AudioService::startup") << "SDL audio init failed: " << SDL_GetError();
		return false;
	}
	m_bAudioSubsystem= true;
	m_bStarted= true;
	m_bConfigPending= true;
	m_bRunning= true;
	m_thread= std::thread([this]() { threadLoop(); });
	return true;
}

void AudioService::shutdown()
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (!m_bStarted)
			return;
		m_bStarted= false;
	}
	m_bRunning= false;
	if (m_thread.joinable())
		m_thread.join();

	std::lock_guard<std::mutex> lock(m_mutex);
	stopRecordingLocked("the audio service shut down");
	if (m_bAudioSubsystem)
	{
		SDL_QuitSubSystem(SDL_INIT_AUDIO);
		m_bAudioSubsystem= false;
	}
	m_status= AudioServiceStatus();
}

void AudioService::setConfig(const AudioServiceConfig& config)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_config= config;
	m_bConfigPending= true;
}

bool AudioService::getLatestSample(MouthSample& outSample)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	releaseDueFramesLocked(steadyNowMs());
	outSample= m_status.latest;
	return m_status.latest.valid;
}

AudioServiceStatus AudioService::getStatus()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	releaseDueFramesLocked(steadyNowMs());
	AudioServiceStatus status= m_status;
	status.droppedBlocks= m_droppedBlocks.load();
	if (m_recorder != nullptr && m_recorder->isRecording())
	{
		status.recording= true;
		status.recordedSamples= m_recorder->getSamplesQueued();
		status.recordingDroppedBlocks= m_recorder->getDroppedBlocks();
	}
	return status;
}

std::vector<std::string> AudioService::listCaptureDevices() const
{
	std::vector<std::string> names;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (!m_bAudioSubsystem)
			return names;
	}
	const int count= SDL_GetNumAudioDevices(1);
	for (int index= 0; index < count; ++index)
	{
		const char* name= SDL_GetAudioDeviceName(index, 1);
		if (name != nullptr)
			names.emplace_back(name);
	}
	return names;
}

bool AudioService::startRecording(const std::string& wavPath, AudioRecordingStart& outStart)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	stopRecordingLocked("a new sidecar started");
	if (!m_status.deviceOpen)
	{
		MIKAN_MT_LOG_WARNING("AudioService::startRecording") << "no capture device open, no audio sidecar";
		return false;
	}
	if (m_recorder == nullptr)
		m_recorder= std::make_unique<AudioRecorder>();
	if (!m_recorder->start(wavPath))
		return false;

	// The next block the audio thread takes under this lock is the sidecar's
	// first, at this stream position
	outStart.startSampleIndex= m_capturedSamples;
	outStart.captureEpoch= m_captureEpoch;
	outStart.timestampMs= steadyNowMs();
	return true;
}

void AudioService::stopRecording()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	stopRecordingLocked(nullptr);
}

bool AudioService::startCapture(float maxSeconds)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	if (!m_status.deviceOpen)
		return false;
	m_capture.clear();
	m_capture.reserve((size_t)std::max(0.f, maxSeconds) * AUDIO_SAMPLE_RATE + AudioBlock::k_capacity);
	m_bCapturing= true;
	return true;
}

void AudioService::stopCapture(std::vector<float>& outSamples)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	outSamples.swap(m_capture);
	m_capture.clear();
	m_bCapturing= false;
}

bool AudioService::isCapturing() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_bCapturing;
}

void AudioService::stopRecordingLocked(const char* why)
{
	if (m_recorder == nullptr || !m_recorder->isRecording())
		return;
	if (why != nullptr)
		MIKAN_MT_LOG_INFO("AudioService::stopRecording") << "audio sidecar ended: " << why;
	m_recorder->stop();
}

// -- Capture callback --------------------------------------------------------

void AudioService::captureCallback(void* userdata, uint8_t* stream, int lengthBytes)
{
	AudioService* self= static_cast<AudioService*>(userdata);
	self->onCaptured(reinterpret_cast<const float*>(stream), lengthBytes / (int)sizeof(float));
}

void AudioService::onCaptured(const float* samples, int sampleCount)
{
	// SDL's buffer need not be a block multiple, so a partial block carries
	// across callbacks
	int offset= 0;
	while (offset < sampleCount)
	{
		const int take= std::min(AudioBlock::k_capacity - m_partialBlock.count, sampleCount - offset);
		std::copy(samples + offset, samples + offset + take, m_partialBlock.samples.data() + m_partialBlock.count);
		m_partialBlock.count+= take;
		offset+= take;
		if (m_partialBlock.count == AudioBlock::k_capacity)
		{
			if (!m_blocks.try_enqueue(m_partialBlock))
				m_droppedBlocks++;
			m_partialBlock.count= 0;
		}
	}
}

// -- Audio thread --------------------------------------------------------------

void AudioService::threadLoop()
{
	ThreadUtils::setCurrentThreadName("AudioService");

	AudioBlock block;
	while (m_bRunning)
	{
		applyPendingConfig();
		if (m_blocks.wait_dequeue_timed(block, std::chrono::milliseconds(5)))
			processBlock(block);
	}

	closeDevice();
	destroyBackend();
}

void AudioService::applyPendingConfig()
{
	AudioServiceConfig config;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (!m_bConfigPending)
			return;
		m_bConfigPending= false;
		config= m_config;
	}

	const bool bDeviceChanged= config.enabled != m_threadConfig.enabled || config.deviceName != m_threadConfig.deviceName;
	const bool bBackendChanged= config.enabled != m_threadConfig.enabled || config.model != m_threadConfig.model ||
								config.profilePath != m_threadConfig.profilePath;
	const bool bEngineSmoothingChanged= config.engineSmoothing != m_threadConfig.engineSmoothing;
	m_threadConfig= config;
	m_chain.setGainDb(config.gainDb);
	m_chain.setGateDbfs(config.gateDbfs);
	m_chain.setSmoothingMs(config.smoothingMs);

	if (bBackendChanged)
	{
		destroyBackend();
		if (config.enabled)
			createBackend();
	}
	else if (bEngineSmoothingChanged && m_backend != nullptr)
	{
		m_backend->setEngineSmoothing(config.engineSmoothing);
	}
	if (bDeviceChanged)
	{
		closeDevice();
		if (config.enabled)
			openDevice();
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	m_status.enabled= config.enabled;
	if (!config.enabled)
	{
		// A disabled stage publishes nothing: the vision thread reads an
		// invalid sample as no mouth
		m_status.latest= MouthSample();
		m_status.levelDbfs= -100.f;
		m_status.gateOpen= false;
	}
}

void AudioService::openDevice()
{
	SDL_AudioSpec want{};
	want.freq= AUDIO_SAMPLE_RATE;
	want.format= AUDIO_F32SYS;
	want.channels= 1;
	want.samples= AudioBlock::k_capacity;
	want.callback= &AudioService::captureCallback;
	want.userdata= this;

	// The configured name, when a device still carries it, else the default
	const char* deviceName= nullptr;
	std::string resolvedName= "default";
	if (!m_threadConfig.deviceName.empty())
	{
		const std::vector<std::string> names= listCaptureDevices();
		if (std::find(names.begin(), names.end(), m_threadConfig.deviceName) != names.end())
		{
			deviceName= m_threadConfig.deviceName.c_str();
			resolvedName= m_threadConfig.deviceName;
		}
		else
		{
			MIKAN_MT_LOG_WARNING("AudioService::openDevice")
				<< "capture device '" << m_threadConfig.deviceName << "' not found, using the default";
		}
	}

	// No allowed changes: SDL converts whatever the device gives to the
	// requested spec, so the window only ever sees 16 kHz mono float
	SDL_AudioSpec have{};
	m_partialBlock.count= 0;
	m_device= SDL_OpenAudioDevice(deviceName, 1, &want, &have, 0);
	if (m_device == 0)
	{
		MIKAN_MT_LOG_ERROR("AudioService::openDevice") << "cannot open capture device '" << resolvedName
													   << "': " << SDL_GetError();
		std::lock_guard<std::mutex> lock(m_mutex);
		m_status.deviceOpen= false;
		m_status.deviceName= resolvedName;
		m_status.deviceSampleRate= 0;
		return;
	}

	m_chain.reset();
	{
		// A new stream: the indices restart, and a sidecar of the old one ends
		std::lock_guard<std::mutex> lock(m_mutex);
		m_pacer.reset();
		stopRecordingLocked("the capture device reopened");
		m_bCapturing= false;
		m_capture.clear();
		m_capturedSamples= 0;
		++m_captureEpoch;
	}
	SDL_PauseAudioDevice(m_device, 0);
	MIKAN_MT_LOG_INFO("AudioService::openDevice") << "capturing '" << resolvedName << "' at " << have.freq << " Hz";

	std::lock_guard<std::mutex> lock(m_mutex);
	m_status.deviceOpen= true;
	m_status.deviceName= resolvedName;
	m_status.deviceSampleRate= have.freq;
}

void AudioService::closeDevice()
{
	if (m_device != 0)
	{
		SDL_CloseAudioDevice(m_device);
		m_device= 0;
	}
	// Drain what the callback queued before it stopped
	AudioBlock discarded;
	while (m_blocks.try_dequeue(discarded))
	{
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	m_pacer.reset();
	stopRecordingLocked("the capture device closed");
	m_bCapturing= false;
	m_capture.clear();
	m_status.deviceOpen= false;
	m_status.deviceSampleRate= 0;
}

void AudioService::createBackend()
{
	std::string reason;
	m_backend= createMouthBackend(m_threadConfig.model, m_threadConfig.profilePath, reason);
	// The stream did not restart with the backend: the window keeps its
	// samples and the frame indices keep counting, so a recording's mouth
	// frames stay tied to its sidecar across a model change. The backend
	// itself starts fresh.
	if (m_backend != nullptr)
	{
		m_backend->setEngineSmoothing(m_threadConfig.engineSmoothing);
		m_backend->reset();
	}
	m_chain.setGainDb(m_threadConfig.gainDb);
	m_chain.setGateDbfs(m_threadConfig.gateDbfs);
	m_chain.setSmoothingMs(m_threadConfig.smoothingMs);
	m_inferMsEma= 0.f;
	m_bInferEmaValid= false;

	std::lock_guard<std::mutex> lock(m_mutex);
	m_status.backendReady= m_backend != nullptr && m_backend->isReady();
	m_status.backendInfo= m_backend != nullptr ? m_backend->getInfo() : std::string();
	m_status.backendReason= m_backend != nullptr ? m_backend->getReason() : reason;
	m_status.inferMsEma= 0.f;
}

void AudioService::destroyBackend()
{
	m_backend= nullptr;
	std::lock_guard<std::mutex> lock(m_mutex);
	m_status.backendReady= false;
	m_status.backendInfo.clear();
	m_status.backendReason.clear();
}

void AudioService::processBlock(const AudioBlock& block)
{
	if (m_device == 0)
		return;

	{
		// The sidecar takes the raw block at its stream position
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_recorder != nullptr && m_recorder->isRecording())
			m_recorder->enqueue(block.samples.data(), block.count);
		if (m_bCapturing && m_capture.size() + (size_t)block.count <= m_capture.capacity())
			m_capture.insert(m_capture.end(), block.samples.data(), block.samples.data() + block.count);
		m_capturedSamples+= block.count;
	}

	m_chainFrames.clear();
	m_chainRuns.clear();
	m_chain.push(block.samples.data(), (size_t)block.count, m_backend.get(), m_chainFrames, m_chainRuns);
	if (m_chainRuns.empty())
		return;

	for (const MouthChainRun& run : m_chainRuns)
	{
		if (run.bInferred)
		{
			const float inferMs= (float)run.inferMs;
			m_inferMsEma= m_bInferEmaValid ? m_inferMsEma + k_inferEmaAlpha * (inferMs - m_inferMsEma) : inferMs;
			m_bInferEmaValid= true;
		}
	}
	const MouthChainRun& lastRun= m_chainRuns.back();
	std::lock_guard<std::mutex> lock(m_mutex);
	for (const MouthChainFrame& frame : m_chainFrames)
		m_pacer.push(frame);
	m_status.levelDbfs= lastRun.levelDbfs;
	m_status.gateOpen= lastRun.bGateOpen;
	m_status.inferMsEma= m_inferMsEma;
}

void AudioService::releaseDueFramesLocked(double nowMs)
{
	MouthChainFrame frame;
	if (!m_pacer.poll(nowMs, frame))
		return;
	m_status.latest.valid= true;
	m_status.latest.sequence= ++m_sequence;
	m_status.latest.timestampMs= nowMs;
	m_status.latest.audioFrameIndex= frame.frameIndex;
	m_status.latest.captureEpoch= m_captureEpoch;
	m_status.latest.frame= frame.frame;
}
