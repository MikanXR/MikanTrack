#include "OnnxSession.h"

#include <filesystem>
#include <sstream>

#include "dml_provider_factory.h"

#include <dxgi1_6.h>

#include "Logger.h"

#pragma comment(lib, "dxgi.lib")

// Name of the adapter DXGI ranks first for high performance, which is the
// adapter the DirectML provider's HighPerformance preference resolves to.
static std::string describeHighPerformanceAdapter()
{
	std::string name= "unknown";

	IDXGIFactory6* factory= nullptr;
	if (SUCCEEDED(CreateDXGIFactory1(IID_PPV_ARGS(&factory))))
	{
		IDXGIAdapter1* adapter= nullptr;
		if (SUCCEEDED(factory->EnumAdapterByGpuPreference(
				0, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&adapter))))
		{
			DXGI_ADAPTER_DESC1 desc= {};
			char utf8Name[256]= {};
			if (SUCCEEDED(adapter->GetDesc1(&desc)) &&
				WideCharToMultiByte(
					CP_UTF8, 0, desc.Description, -1, utf8Name, (int)sizeof(utf8Name), nullptr, nullptr) > 0)
			{
				name= utf8Name;
			}
			adapter->Release();
		}
		factory->Release();
	}

	return name;
}

// Adds the DirectML provider on the highest-performance GPU. The device-index
// form of this call takes adapter 0, which on a hybrid-GPU laptop is the
// integrated GPU.
static void appendDirectMlProvider(Ort::SessionOptions& sessionOptions)
{
	const OrtDmlApi* dmlApi= nullptr;
	Ort::ThrowOnError(Ort::GetApi().GetExecutionProviderApi(
		"DML", ORT_API_VERSION, reinterpret_cast<const void**>(&dmlApi)));

	OrtDmlDeviceOptions deviceOptions= {};
	deviceOptions.Preference= OrtDmlPerformancePreference::HighPerformance;
	deviceOptions.Filter= OrtDmlDeviceFilter::Gpu;
	Ort::ThrowOnError(dmlApi->SessionOptionsAppendExecutionProvider_DML2(sessionOptions, &deviceOptions));

	// Sessions are only created on the inference thread, so a plain flag is enough
	static bool s_bAdapterLogged= false;
	if (!s_bAdapterLogged)
	{
		s_bAdapterLogged= true;
		MIKAN_MT_LOG_INFO("OnnxSession") << "DirectML adapter: " << describeHighPerformanceAdapter();
	}
}

Ort::Env& OnnxSession::getSharedEnv()
{
	// One process-wide environment shared by every session.
	// Constructed on first use (on the inference thread) and intentionally
	// leaked at process exit (function-local static).
	static Ort::Env s_env(ORT_LOGGING_LEVEL_WARNING, "MikanTrack");
	return s_env;
}

const Ort::MemoryInfo& OnnxSession::getCpuMemoryInfo()
{
	static Ort::MemoryInfo s_memoryInfo= Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
	return s_memoryInfo;
}

bool OnnxSession::create(const std::string& modelPath, const std::string& preferredEp)
{
	const std::filesystem::path fsPath(modelPath);
	const std::wstring widePath= fsPath.wstring();

	m_session= nullptr;
	m_activeEp= "none";

	if (preferredEp == "directml")
	{
		try
		{
			Ort::SessionOptions sessionOptions;
			// DirectML requires memory pattern optimization off and
			// sequential execution
			sessionOptions.DisableMemPattern();
			sessionOptions.SetExecutionMode(ORT_SEQUENTIAL);
			sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
			appendDirectMlProvider(sessionOptions);

			m_session= std::make_unique<Ort::Session>(getSharedEnv(), widePath.c_str(), sessionOptions);
			m_activeEp= "DirectML";
		}
		catch (const std::exception& e)
		{
			MIKAN_MT_LOG_WARNING("OnnxSession::create")
				<< "DirectML session failed for " << modelPath
				<< " (" << e.what() << ") - falling back to CPU";
			m_session= nullptr;
		}
	}

	if (m_session == nullptr)
	{
		try
		{
			Ort::SessionOptions sessionOptions;
			sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

			m_session= std::make_unique<Ort::Session>(getSharedEnv(), widePath.c_str(), sessionOptions);
			m_activeEp= "CPU";
		}
		catch (const std::exception& e)
		{
			MIKAN_MT_LOG_ERROR("OnnxSession::create")
				<< "Failed to load model " << modelPath << " : " << e.what();
			m_session= nullptr;
			m_activeEp= "none";
			return false;
		}
	}

	try
	{
		cacheIoMetadata();
	}
	catch (const std::exception& e)
	{
		MIKAN_MT_LOG_ERROR("OnnxSession::create")
			<< "Failed to query I/O metadata for " << modelPath << " : " << e.what();
		m_session= nullptr;
		m_activeEp= "none";
		return false;
	}

	logModelInfo(modelPath);
	return true;
}

void OnnxSession::cacheIoMetadata()
{
	m_inputNames.clear();
	m_outputNames.clear();
	m_inputNamePtrs.clear();
	m_outputNamePtrs.clear();
	m_inputShapes.clear();
	m_outputShapes.clear();

	Ort::AllocatorWithDefaultOptions allocator;

	const size_t inputCount= m_session->GetInputCount();
	for (size_t i= 0; i < inputCount; ++i)
	{
		Ort::AllocatedStringPtr name= m_session->GetInputNameAllocated(i, allocator);
		m_inputNames.push_back(name.get());
		m_inputShapes.push_back(m_session->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
	}

	const size_t outputCount= m_session->GetOutputCount();
	for (size_t i= 0; i < outputCount; ++i)
	{
		Ort::AllocatedStringPtr name= m_session->GetOutputNameAllocated(i, allocator);
		m_outputNames.push_back(name.get());
		m_outputShapes.push_back(m_session->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
	}

	// Pointer arrays must be built after the string vectors stop reallocating
	for (const std::string& name : m_inputNames)
		m_inputNamePtrs.push_back(name.c_str());
	for (const std::string& name : m_outputNames)
		m_outputNamePtrs.push_back(name.c_str());
}

static std::string shapeToString(const std::vector<int64_t>& shape)
{
	std::ostringstream ss;
	ss << "[";
	for (size_t i= 0; i < shape.size(); ++i)
	{
		if (i > 0)
			ss << "x";
		ss << shape[i];
	}
	ss << "]";
	return ss.str();
}

void OnnxSession::logModelInfo(const std::string& modelPath) const
{
	std::ostringstream ss;
	ss << "Loaded " << modelPath << " (EP: " << m_activeEp << ")";
	for (size_t i= 0; i < m_inputNames.size(); ++i)
		ss << " | in " << m_inputNames[i] << " " << shapeToString(m_inputShapes[i]);
	for (size_t i= 0; i < m_outputNames.size(); ++i)
		ss << " | out " << m_outputNames[i] << " " << shapeToString(m_outputShapes[i]);

	MIKAN_MT_LOG_INFO("OnnxSession") << ss.str();
}

int OnnxSession::findOutputByLastDim(int64_t lastDim) const
{
	for (size_t i= 0; i < m_outputShapes.size(); ++i)
	{
		const std::vector<int64_t>& shape= m_outputShapes[i];
		if (!shape.empty() && shape.back() == lastDim)
			return (int)i;
	}
	return -1;
}

std::vector<Ort::Value> OnnxSession::run(const Ort::Value* inputs, size_t inputCount)
{
	return m_session->Run(
		Ort::RunOptions{nullptr},
		m_inputNamePtrs.data(), inputs, inputCount,
		m_outputNamePtrs.data(), m_outputNamePtrs.size());
}

std::vector<Ort::Value> OnnxSession::runOutputs(
	const Ort::Value* inputs, size_t inputCount,
	const int* outputIndices, size_t outputIndexCount)
{
	std::vector<const char*> requestedNames;
	requestedNames.reserve(outputIndexCount);
	for (size_t i= 0; i < outputIndexCount; ++i)
	{
		const int outputIndex= outputIndices[i];
		if (outputIndex < 0 || outputIndex >= (int)m_outputNamePtrs.size())
			return {}; // caller mapped an output that doesn't exist
		requestedNames.push_back(m_outputNamePtrs[outputIndex]);
	}

	return m_session->Run(
		Ort::RunOptions{nullptr},
		m_inputNamePtrs.data(), inputs, inputCount,
		requestedNames.data(), requestedNames.size());
}
