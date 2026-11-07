#include "Wav2ArkitBackend.h"

#include <chrono>
#include <filesystem>

#include "Logger.h"

// The model's output order, from its config.json: the 52 ARKit names sorted
// alphabetically
static const char* const k_outputColumnNames[ARKIT_BLENDSHAPE_COUNT]= {
	"browDownLeft",     "browDownRight",     "browInnerUp",        "browOuterUpLeft",    "browOuterUpRight",
	"cheekPuff",        "cheekSquintLeft",   "cheekSquintRight",   "eyeBlinkLeft",       "eyeBlinkRight",
	"eyeLookDownLeft",  "eyeLookDownRight",  "eyeLookInLeft",      "eyeLookInRight",     "eyeLookOutLeft",
	"eyeLookOutRight",  "eyeLookUpLeft",     "eyeLookUpRight",     "eyeSquintLeft",      "eyeSquintRight",
	"eyeWideLeft",      "eyeWideRight",      "jawForward",         "jawLeft",            "jawOpen",
	"jawRight",         "mouthClose",        "mouthDimpleLeft",    "mouthDimpleRight",   "mouthFrownLeft",
	"mouthFrownRight",  "mouthFunnel",       "mouthLeft",          "mouthLowerDownLeft", "mouthLowerDownRight",
	"mouthPressLeft",   "mouthPressRight",   "mouthPucker",        "mouthRight",         "mouthRollLower",
	"mouthRollUpper",   "mouthShrugLower",   "mouthShrugUpper",    "mouthSmileLeft",     "mouthSmileRight",
	"mouthStretchLeft", "mouthStretchRight", "mouthUpperUpLeft",   "mouthUpperUpRight",  "noseSneerLeft",
	"noseSneerRight",   "tongueOut",
};

int Wav2ArkitBackend::outputColumnArkitIndex(int outputColumn)
{
	if (outputColumn < 0 || outputColumn >= ARKIT_BLENDSHAPE_COUNT)
		return -1;
	return arkitBlendshapeFromName(k_outputColumnNames[outputColumn]);
}

bool Wav2ArkitBackend::create(const std::string& modelPath, const std::string& preferredEp, int intraOpThreads)
{
	m_lastInferMs= 0.0;
	for (int column= 0; column < ARKIT_BLENDSHAPE_COUNT; ++column)
		m_outputToMouthColumn[column]= mouthColumnFromArkitIndex(outputColumnArkitIndex(column));

	if (!std::filesystem::exists(modelPath))
	{
		m_unavailableReason= "model file missing: " + modelPath;
		MIKAN_MT_LOG_WARNING("Wav2ArkitBackend::create") << m_unavailableReason;
		return false;
	}

	m_session= std::make_unique<OnnxSession>();
	if (!m_session->create(modelPath, preferredEp, intraOpThreads))
	{
		m_unavailableReason= "model failed to load: " + modelPath;
		m_session= nullptr;
		return false;
	}

	if (m_session->getInputCount() != 1 || m_session->getOutputCount() != 1 ||
		m_session->getOutputShape(0).size() != 3 || m_session->getOutputShape(0)[2] != ARKIT_BLENDSHAPE_COUNT)
	{
		m_unavailableReason= "model has an unexpected signature: " + modelPath;
		MIKAN_MT_LOG_ERROR("Wav2ArkitBackend::create") << m_unavailableReason;
		m_session= nullptr;
		return false;
	}

	m_unavailableReason.clear();
	return true;
}

bool Wav2ArkitBackend::infer(const float* samples, size_t sampleCount, std::vector<MouthFrame>& outFrames)
{
	outFrames.clear();
	if (!isReady() || sampleCount < k_minSamples)
		return false;

	const auto start= std::chrono::steady_clock::now();

	const std::array<int64_t, 2> inputShape= {1, (int64_t)sampleCount};
	Ort::Value inputTensor= Ort::Value::CreateTensor<float>(
		OnnxSession::getCpuMemoryInfo(), const_cast<float*>(samples), sampleCount, inputShape.data(),
		inputShape.size());

	std::vector<Ort::Value> outputs;
	try
	{
		outputs= m_session->run(&inputTensor, 1);
	}
	catch (const std::exception& e)
	{
		MIKAN_MT_LOG_ERROR("Wav2ArkitBackend::infer") << "run failed: " << e.what();
		return false;
	}

	const std::vector<int64_t> outputShape= outputs[0].GetTensorTypeAndShapeInfo().GetShape();
	if (outputShape.size() != 3 || outputShape[2] != ARKIT_BLENDSHAPE_COUNT)
		return false;

	const int64_t frameCount= outputShape[1];
	const float* values= outputs[0].GetTensorData<float>();
	outFrames.resize((size_t)frameCount);
	for (int64_t frameIndex= 0; frameIndex < frameCount; ++frameIndex)
	{
		const float* row= values + frameIndex * ARKIT_BLENDSHAPE_COUNT;
		MouthFrame& frame= outFrames[(size_t)frameIndex];
		for (int column= 0; column < ARKIT_BLENDSHAPE_COUNT; ++column)
		{
			const int mouthColumn= m_outputToMouthColumn[column];
			if (mouthColumn >= 0)
				frame.weights[mouthColumn]= std::min(std::max(row[column], 0.f), 1.f);
		}
	}

	m_lastInferMs= std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	return true;
}
