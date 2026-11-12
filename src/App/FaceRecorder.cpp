#include "FaceRecorder.h"

#include <algorithm>
#include <filesystem>
#include <system_error>

#include "nlohmann/json.hpp"

#include "Logger.h"
#include "PathUtils.h"

using json= nlohmann::json;

bool FaceRecorder::start(const std::string& filePath)
{
	stop();
	m_file.open(PathUtils::utf8ToPath(filePath), std::ios::binary | std::ios::trunc);
	if (!m_file.is_open())
	{
		MIKAN_LOG_ERROR("FaceRecorder") << "Failed to open " << filePath << " for writing";
		return false;
	}
	m_filePath= filePath;
	m_samplesWritten= 0;
	m_droppedSamples= 0;
	m_bWriterShouldExit= false;
	m_queue.clear();
	m_bRecording= true;
	m_writerThread= std::thread(&FaceRecorder::writerLoop, this);
	return true;
}

void FaceRecorder::enqueue(const RecordedFaceSample& sample)
{
	if (!m_bRecording)
		return;
	{
		std::lock_guard<std::mutex> lock(m_queueMutex);
		if (m_queue.size() >= k_maxQueuedSamples)
		{
			++m_droppedSamples;
			return;
		}
		m_queue.push_back(sample);
	}
	m_queueCondition.notify_one();
}

void FaceRecorder::stop()
{
	if (!m_bRecording)
		return;
	{
		std::lock_guard<std::mutex> lock(m_queueMutex);
		m_bWriterShouldExit= true;
	}
	m_queueCondition.notify_one();
	if (m_writerThread.joinable())
		m_writerThread.join();
	m_file.close();
	m_bRecording= false;
	if (m_droppedSamples > 0)
		MIKAN_LOG_WARNING("FaceRecorder") << "Dropped " << m_droppedSamples << " face samples on queue overflow";
	// No phone sample during the recording: no sidecar, which replay reads as
	// no phone
	if (m_samplesWritten == 0)
	{
		std::error_code error;
		std::filesystem::remove(PathUtils::utf8ToPath(m_filePath), error);
		if (error)
			MIKAN_LOG_WARNING("FaceRecorder") << "Failed to remove empty " << m_filePath << ": " << error.message();
	}
}

void FaceRecorder::writerLoop()
{
	for (;;)
	{
		std::deque<RecordedFaceSample> batch;
		bool bExit= false;
		{
			std::unique_lock<std::mutex> lock(m_queueMutex);
			m_queueCondition.wait(lock, [this] { return m_bWriterShouldExit || !m_queue.empty(); });
			batch.swap(m_queue);
			bExit= m_bWriterShouldExit;
		}
		for (const RecordedFaceSample& sample : batch)
		{
			m_file << FaceSidecar::sampleToLine(sample) << '\n';
			++m_samplesWritten;
		}
		if (bExit)
			break;
	}
	m_file.flush();
}

namespace FaceSidecar
{
std::string sampleToLine(const RecordedFaceSample& sample)
{
	json weights= json::array();
	for (float weight : sample.blendshapes)
		weights.push_back(weight);
	json j= {{"seq", sample.sequence}, {"t", sample.timestampMs}, {"tracked", sample.bTracked}, {"w", weights}};
	if (sample.bHasHead)
	{
		j["head"]=
			json::array({sample.headEulerDegrees.x, sample.headEulerDegrees.y, sample.headEulerDegrees.z});
	}
	return j.dump();
}

bool sampleFromLine(const std::string& line, RecordedFaceSample& outSample)
{
	const json j= json::parse(line, nullptr, false);
	if (j.is_discarded() || !j.is_object())
		return false;
	const json& weights= j.value("w", json::array());
	if (!weights.is_array() || weights.size() != ARKIT_BLENDSHAPE_COUNT)
		return false;
	RecordedFaceSample sample;
	sample.sequence= j.value("seq", (uint64_t)0);
	sample.timestampMs= j.value("t", 0.0);
	sample.bTracked= j.value("tracked", false);
	for (size_t index= 0; index < weights.size(); ++index)
	{
		if (!weights[index].is_number())
			return false;
		sample.blendshapes[index]= weights[index].get<float>();
	}
	if (j.contains("head"))
	{
		const json& head= j["head"];
		if (!head.is_array() || head.size() != 3 || !head[0].is_number() || !head[1].is_number() ||
			!head[2].is_number())
			return false;
		sample.bHasHead= true;
		sample.headEulerDegrees= glm::vec3(head[0].get<float>(), head[1].get<float>(), head[2].get<float>());
	}
	outSample= sample;
	return true;
}

bool load(const std::string& filePath, std::vector<RecordedFaceSample>& outSamples, std::string& outError)
{
	outSamples.clear();
	std::ifstream file(PathUtils::utf8ToPath(filePath), std::ios::binary);
	if (!file.is_open())
	{
		outError= "Cannot open " + filePath;
		return false;
	}
	std::string line;
	while (std::getline(file, line))
	{
		if (!line.empty() && line.back() == '\r')
			line.pop_back();
		if (line.empty())
			continue;
		RecordedFaceSample sample;
		if (!sampleFromLine(line, sample))
		{
			MIKAN_LOG_WARNING("FaceSidecar") << "Unparseable line in " << filePath << " - treating as truncation";
			break;
		}
		outSamples.push_back(sample);
	}
	return true;
}

const RecordedFaceSample* findAtOrBefore(const std::vector<RecordedFaceSample>& samples, double timestampMs)
{
	const auto after= std::upper_bound(samples.begin(), samples.end(), timestampMs,
									   [](double time, const RecordedFaceSample& sample) { return time < sample.timestampMs; });
	if (after == samples.begin())
		return nullptr;
	return &*(after - 1);
}

void samplePhone(const std::vector<RecordedFaceSample>& samples, double timestampMs, double streamingWindowMs,
				 FaceSources::Phone& outPhone)
{
	outPhone= FaceSources::Phone();
	const RecordedFaceSample* sample= findAtOrBefore(samples, timestampMs);
	if (sample == nullptr || timestampMs - sample->timestampMs >= streamingWindowMs)
		return;
	outPhone.present= true;
	outPhone.tracked= sample->bTracked;
	if (outPhone.tracked)
		outPhone.blendshapes= sample->blendshapes;
}
} // namespace FaceSidecar
