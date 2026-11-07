#include "MouthBackend.h"

#include "LoudnessBackend.h"
#include "MfccBackend.h"
#include "OvrLipSyncBackend.h"

std::unique_ptr<IMouthBackend> createMouthBackend(eMouthModel model, const std::string& profilePath,
												  std::string& outReason)
{
	outReason.clear();
	switch (model)
	{
	case eMouthModel::Loudness:
		return std::make_unique<LoudnessBackend>();
	case eMouthModel::Mfcc: {
		auto backend= std::make_unique<MfccBackend>();
		backend->create(profilePath);
		return backend;
	}
	case eMouthModel::OvrLipSync:
#ifdef MIKANTRACK_OVRLIPSYNC
	{
		auto backend= std::make_unique<OvrLipSyncBackend>();
		backend->create();
		return backend;
	}
#else
		outReason= "this build has no Oculus LipSync SDK";
		return nullptr;
#endif
	}
	outReason= "unknown mouth model";
	return nullptr;
}
