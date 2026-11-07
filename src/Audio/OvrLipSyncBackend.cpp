#ifdef MIKANTRACK_OVRLIPSYNC

#include "OvrLipSyncBackend.h"

#include <algorithm>
#include <chrono>
#include <mutex>

#include "OVRLipSync.h"

#include "Logger.h"

namespace
{
// ovrLipSync_Initialize and ovrLipSync_Shutdown are process-wide: the
// engine is held while any backend lives
std::mutex g_engineMutex;
int g_engineUsers= 0;
ovrLipSyncResult g_engineResult= ovrLipSyncSuccess;

bool acquireEngine(ovrLipSyncResult& outResult)
{
	std::lock_guard<std::mutex> lock(g_engineMutex);
	if (g_engineUsers == 0)
	{
		// The buffer size is a default for contexts created without one;
		// every call here passes its own sample count
		g_engineResult= ovrLipSync_Initialize(AUDIO_SAMPLE_RATE, AUDIO_SAMPLE_RATE / MOUTH_FRAME_RATE + 1);
	}
	outResult= g_engineResult;
	if (g_engineResult != ovrLipSyncSuccess)
		return false;
	++g_engineUsers;
	return true;
}

void releaseEngine()
{
	std::lock_guard<std::mutex> lock(g_engineMutex);
	if (--g_engineUsers == 0)
		ovrLipSync_Shutdown();
}

const char* resultName(ovrLipSyncResult result)
{
	switch (result)
	{
	case ovrLipSyncSuccess:
		return "success";
	case ovrLipSyncError_Unknown:
		return "unknown error";
	case ovrLipSyncError_CannotCreateContext:
		return "cannot create context";
	case ovrLipSyncError_InvalidParam:
		return "invalid parameter";
	case ovrLipSyncError_BadSampleRate:
		return "unsupported sample rate";
	case ovrLipSyncError_MissingDLL:
		return "OVRLipSync.dll is not beside MikanTrack.exe";
	case ovrLipSyncError_BadVersion:
		return "header and library versions differ";
	case ovrLipSyncError_UndefinedFunction:
		return "undefined function";
	}
	return "unknown result";
}

static_assert(ovrLipSyncViseme_Count == VISEME_COUNT, "the stage's viseme set is the Oculus set");

constexpr std::array<int, VISEME_COUNT> k_visemeRemap= {
	VISEME_SIL,		  // ovrLipSyncViseme_sil
	(int)eViseme::PP, // ovrLipSyncViseme_PP
	(int)eViseme::FF, // ovrLipSyncViseme_FF
	(int)eViseme::TH, // ovrLipSyncViseme_TH
	(int)eViseme::DD, // ovrLipSyncViseme_DD
	(int)eViseme::kk, // ovrLipSyncViseme_kk
	(int)eViseme::CH, // ovrLipSyncViseme_CH
	(int)eViseme::SS, // ovrLipSyncViseme_SS
	(int)eViseme::nn, // ovrLipSyncViseme_nn
	(int)eViseme::RR, // ovrLipSyncViseme_RR
	(int)eViseme::aa, // ovrLipSyncViseme_aa
	(int)eViseme::E,  // ovrLipSyncViseme_E
	(int)eViseme::I,  // ovrLipSyncViseme_ih
	(int)eViseme::O,  // ovrLipSyncViseme_oh
	(int)eViseme::U,  // ovrLipSyncViseme_ou
};
} // namespace

const std::array<int, VISEME_COUNT>& OvrLipSyncBackend::getVisemeRemap()
{
	return k_visemeRemap;
}

OvrLipSyncBackend::OvrLipSyncBackend()= default;

OvrLipSyncBackend::~OvrLipSyncBackend()
{
	destroy();
}

bool OvrLipSyncBackend::create()
{
	destroy();
	m_lastInferMs= 0.0;

	ovrLipSyncResult result;
	if (!acquireEngine(result))
	{
		m_reason= resultName(result);
		MIKAN_MT_LOG_WARNING("OvrLipSyncBackend::create") << "ovrLipSync_Initialize failed: " << m_reason;
		return false;
	}
	m_bEngineHeld= true;

	ovrLipSyncContext context= 0;
	result= ovrLipSync_CreateContextEx(&context, ovrLipSyncContextProvider_Enhanced, AUDIO_SAMPLE_RATE, false);
	if (result != ovrLipSyncSuccess)
	{
		m_reason= resultName(result);
		MIKAN_MT_LOG_WARNING("OvrLipSyncBackend::create") << "ovrLipSync_CreateContextEx failed: " << m_reason;
		destroy();
		return false;
	}
	m_context= context;

	// The header's version is the one the shim checked the library against
	m_info= "Oculus LipSync " + std::to_string(OVR_LIPSYNC_MAJOR_VERSION) + "." +
			std::to_string(OVR_LIPSYNC_MINOR_VERSION) + "." + std::to_string(OVR_LIPSYNC_PATCH_VERSION);
	m_reason.clear();
	return true;
}

void OvrLipSyncBackend::destroy()
{
	if (m_context != 0)
	{
		ovrLipSync_DestroyContext(m_context);
		m_context= 0;
	}
	if (m_bEngineHeld)
	{
		releaseEngine();
		m_bEngineHeld= false;
	}
	m_info.clear();
}

void OvrLipSyncBackend::reset()
{
	// ovrLipSync_ResetContext leaves state behind (two runs after it differ
	// by their history), so a fresh stream gets a fresh context
	if (m_context == 0)
		return;
	ovrLipSync_DestroyContext(m_context);
	m_context= 0;
	ovrLipSyncContext context= 0;
	const ovrLipSyncResult result=
		ovrLipSync_CreateContextEx(&context, ovrLipSyncContextProvider_Enhanced, AUDIO_SAMPLE_RATE, false);
	if (result != ovrLipSyncSuccess)
	{
		m_reason= resultName(result);
		MIKAN_MT_LOG_WARNING("OvrLipSyncBackend::reset") << "ovrLipSync_CreateContextEx failed: " << m_reason;
		return;
	}
	m_context= context;
}

bool OvrLipSyncBackend::infer(const float* samples, size_t sampleCount, int newFrameCount, std::vector<MouthFrame>& outFrames)
{
	outFrames.clear();
	if (m_context == 0 || sampleCount == 0)
		return false;

	const auto start= std::chrono::steady_clock::now();

	const int frameCount= mouthFramesForSamples(sampleCount);
	// Only the frames the chain emits are new audio to the engine: the
	// window's earlier frames were fed by the runs before and come back as
	// the neutral mouth (every weight zero, the chain's own neutral)
	outFrames.assign((size_t)frameCount, MouthFrame());
	const int firstFrame= std::max(0, frameCount - std::max(newFrameCount, 0));
	ovrLipSyncFrame result= {};
	result.visemes= m_scores.data();
	result.visemesLength= (unsigned int)m_scores.size();
	for (int frameIndex= firstFrame; frameIndex < frameCount; ++frameIndex)
	{
		const size_t begin= frameIndex > 0 ? std::min(mouthFrameEndSample(frameIndex - 1), sampleCount) : 0;
		const size_t end= std::min(mouthFrameEndSample(frameIndex), sampleCount);
		if (end <= begin)
			continue;
		const ovrLipSyncResult rc= ovrLipSync_ProcessFrameEx(m_context, samples + begin, (int)(end - begin),
															 ovrLipSyncAudioDataType_F32_Mono, &result);
		if (rc != ovrLipSyncSuccess)
		{
			m_reason= resultName(rc);
			MIKAN_MT_LOG_WARNING("OvrLipSyncBackend::infer") << "ovrLipSync_ProcessFrameEx failed: " << m_reason;
			outFrames.clear();
			return false;
		}
		VisemeWeights& weights= outFrames[(size_t)frameIndex].weights;
		for (size_t ovrIndex= 0; ovrIndex < k_visemeRemap.size(); ++ovrIndex)
			weights[(size_t)k_visemeRemap[ovrIndex]]= std::clamp(m_scores[ovrIndex], 0.f, 1.f);
	}

	m_lastInferMs= std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	return true;
}

#endif
