#include "AudioRecorder.h"

#include <algorithm>

#include "AudioTypes.h"
#include "Logger.h"

bool AudioRecorder::start(const std::string& filePath)
{
	stop();

	std::string error;
	if (!m_writer.open(filePath, AUDIO_SAMPLE_RATE, 1, error))
	{
		MIKAN_MT_LOG_ERROR("AudioRecorder") << "Failed to open " << filePath << ": " << error;
		return false;
	}

	m_filePath= filePath;
	m_samplesQueued= 0;
	m_samplesWritten= 0;
	m_droppedBlocks= 0;
	m_pendingSilentSamples= 0;
	m_bWriterShouldExit= false;
	m_bRecording= true;
	m_writerThread= std::thread([this]() { writerLoop(); });

	MIKAN_MT_LOG_INFO("AudioRecorder") << "Recording audio to " << filePath;
	return true;
}

void AudioRecorder::enqueue(const float* samples, int sampleCount)
{
	if (!m_bRecording)
		return;

	int offset= 0;
	while (offset < sampleCount)
	{
		QueuedBlock block;
		block.count= std::min(QueuedBlock::k_capacity, sampleCount - offset);
		std::copy(samples + offset, samples + offset + block.count, block.samples.data());
		offset+= block.count;

		bool bAccepted= false;
		{
			std::lock_guard<std::mutex> lock(m_queueMutex);
			if (m_queue.size() < k_maxQueuedBlocks)
			{
				block.silentSamplesBefore= m_pendingSilentSamples;
				m_pendingSilentSamples= 0;
				m_queue.push_back(block);
				bAccepted= true;
			}
		}
		if (bAccepted)
		{
			m_queueCondition.notify_one();
		}
		else
		{
			// Silence keeps the file's clock; the drop is counted
			m_pendingSilentSamples+= block.count;
			m_droppedBlocks++;
		}
		m_samplesQueued+= (uint64_t)block.count;
	}
}

void AudioRecorder::stop()
{
	if (!m_writerThread.joinable())
		return;

	m_bRecording= false;
	{
		std::lock_guard<std::mutex> lock(m_queueMutex);
		m_bWriterShouldExit= true;
	}
	m_queueCondition.notify_one();
	m_writerThread.join();

	// The writer has drained the queue; the drops since the last accepted
	// block still owe their silence
	if (m_pendingSilentSamples > 0)
	{
		m_writer.writeSilence((size_t)m_pendingSilentSamples);
		m_samplesWritten+= (uint64_t)m_pendingSilentSamples;
		m_pendingSilentSamples= 0;
	}
	m_writer.close();

	MIKAN_MT_LOG_INFO("AudioRecorder") << "Finalized " << m_filePath << ": " << m_samplesWritten << " samples ("
									   << (double)m_samplesWritten / AUDIO_SAMPLE_RATE << " s)"
									   << (m_droppedBlocks > 0 ? ", " + std::to_string(m_droppedBlocks.load()) + " blocks dropped"
															   : "");
}

void AudioRecorder::writerLoop()
{
	while (true)
	{
		QueuedBlock block;
		{
			std::unique_lock<std::mutex> lock(m_queueMutex);
			m_queueCondition.wait(lock, [this]() { return !m_queue.empty() || m_bWriterShouldExit; });
			if (m_queue.empty())
			{
				if (m_bWriterShouldExit)
					return;
				continue;
			}
			block= m_queue.front();
			m_queue.pop_front();
		}

		if (block.silentSamplesBefore > 0)
		{
			m_writer.writeSilence((size_t)block.silentSamplesBefore);
			m_samplesWritten+= (uint64_t)block.silentSamplesBefore;
		}
		m_writer.write(block.samples.data(), (size_t)block.count);
		m_samplesWritten+= (uint64_t)block.count;
	}
}
