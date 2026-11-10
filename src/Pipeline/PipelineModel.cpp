#include "PipelineModel.h"

#include <algorithm>
#include <cstdlib>

namespace
{
using S= ePipelineStage;
using Inputs= PipelineStatusInputs;

struct StageInfo
{
	// The localization key suffix ("pipeline.stage<name>", "pipeline.desc<name>")
	const char* name;
	// The last token of the stage's text id
	const char* token;
	// The top-level stage it sits under; itself for a top-level one
	S parent;
};

// Indexed by ePipelineStage
const StageInfo k_stages[(int)S::Count]= {
	{"Capture", "capture", S::Capture},
	{"Camera", "cam", S::Capture},
	{"Video", "video", S::Capture},
	{"HandModels", "hands", S::Capture},
	{"BodyModel", "body", S::Capture},
	{"Lift", "lift", S::Capture},
	{"Fusion", "fusion", S::Fusion},
	{"Imu", "imu", S::Imu},
	{"BodySolver", "body", S::BodySolver},
	{"DirectSolver", "direct", S::DirectSolver},
	{"Filters", "filters", S::DirectSolver},
	{"Torso", "torso", S::DirectSolver},
	{"ArmChain", "arm", S::DirectSolver},
	{"Palm", "palm", S::DirectSolver},
	{"HoldRest", "hold", S::DirectSolver},
	{"SourcePose", "source", S::SourcePose},
	{"Retarget", "retarget", S::Retarget},
	{"Output", "output", S::Output},
	{"DeliveryRules", "rules", S::Output},
	{"VmcStream", "vmc", S::Output},
	{"Face", "face", S::Face},
	{"Phone", "phone", S::Face},
	{"Microphone", "mic", S::Face},
	{"Profile", "profile", S::Face},
};

bool isPerCamera(S stage)
{
	return stage == S::Camera || stage == S::Video || stage == S::HandModels || stage == S::BodyModel ||
		stage == S::Lift;
}

PipelineStageId makeId(S stage, int cameraIndex= -1)
{
	PipelineStageId id;
	id.stage= stage;
	id.cameraIndex= cameraIndex;
	return id;
}

StageText text(const char* key, double a0= 0.0, double a1= 0.0, double a2= 0.0)
{
	StageText out;
	out.key= std::string("pipeline.") + key;
	out.args[0]= a0;
	out.args[1]= a1;
	out.args[2]= a2;
	return out;
}

StageStatus status(eStageState state, const StageText& value, const StageText& reason= StageText())
{
	StageStatus out;
	out.state= state;
	out.value= value;
	out.reason= reason;
	return out;
}

StageStatus running(const StageText& value, const StageText& reason= StageText())
{
	return status(eStageState::Running, value, reason);
}
StageStatus degraded(const StageText& value, const StageText& reason)
{
	return status(eStageState::Degraded, value, reason);
}
StageStatus inactive(const StageText& value, const StageText& reason)
{
	return status(eStageState::Inactive, value, reason);
}
StageStatus notConfigured(const StageText& value, const StageText& reason)
{
	return status(eStageState::NotConfigured, value, reason);
}

// A replayed frame from a take that holds no stage state: a stage that is
// nothing but that state says so rather than reading as stopped
bool isUnrecorded(const Inputs& in)
{
	return in.bReplay && !in.bStageStateRecorded;
}

StageStatus notRecorded()
{
	return inactive(text("valNotRecorded"), text("whyNotRecorded"));
}

int clampedCameraCount(const Inputs& in)
{
	return std::clamp(in.cameraCount, 0, Inputs::k_maxCameras);
}

// The multi-camera pipeline with no camera placed in the world: fusion
// passes camera 0 through in camera space and nothing hand-side is streamed
bool isWithoutWorld(const Inputs& in)
{
	if (in.bDirect)
		return false;
	if (in.path == Inputs::ePath::Passthrough)
		return true;
	for (int cameraIndex= 0; cameraIndex < clampedCameraCount(in); ++cameraIndex)
	{
		if (in.cameras[cameraIndex].bExtrinsics)
			return false;
	}
	return true;
}

int countTrackedHands(const Inputs& in)
{
	return (in.hands[0].bTracked ? 1 : 0) + (in.hands[1].bTracked ? 1 : 0);
}

StageStatus computeCameraDevice(const Inputs::Camera& camera, int cameraIndex)
{
	if (!camera.bDeviceOpen)
		return notConfigured(text("valNoDevice"), text("whyNoDeviceFmt", cameraIndex + 1));
	if (!camera.bStreaming)
		return notConfigured(text("valStopped"), text("whyStreamStoppedFmt", cameraIndex + 1));
	return running(text("valFpsFmt", camera.captureFps));
}

StageStatus computeCapture(const Inputs& in)
{
	const int cameraCount= clampedCameraCount(in);
	if (cameraCount == 0)
		return notConfigured(text("valNoCameras"), text("whyNoCameras"));

	int streamingCount= 0;
	int firstStopped= -1;
	int firstWithoutIntrinsics= -1;
	float fps= 0.f;
	for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
	{
		const Inputs::Camera& camera= in.cameras[cameraIndex];
		if (camera.bDeviceOpen && camera.bStreaming)
		{
			++streamingCount;
			// The slowest camera that has delivered a frame
			if (camera.captureFps > 0.f)
				fps= fps > 0.f ? std::min(fps, camera.captureFps) : camera.captureFps;
		}
		else if (firstStopped < 0)
		{
			firstStopped= cameraIndex;
		}
		if (!camera.bIntrinsics && firstWithoutIntrinsics < 0)
			firstWithoutIntrinsics= cameraIndex;
	}
	if (firstStopped >= 0)
	{
		return notConfigured(text("valCamsStreamingFmt", streamingCount, cameraCount),
							 text("whyCameraNotStreamingFmt", firstStopped + 1));
	}
	if (firstWithoutIntrinsics >= 0)
	{
		return notConfigured(text("valCamsUncalibratedFmt", cameraCount),
							 text("whyCameraNoIntrinsicsFmt", firstWithoutIntrinsics + 1));
	}
	if (isWithoutWorld(in))
		return degraded(text("valCamsNoWorldFmt", cameraCount), text("whyNoExtrinsicsAll"));
	return running(text("valCamsFpsFmt", cameraCount, fps));
}

StageStatus computePerCamera(const Inputs& in, const PipelineStageId& id)
{
	if (id.cameraIndex < 0 || id.cameraIndex >= clampedCameraCount(in))
		return inactive(text("valNoCamera"), text("whyNoSuchCamera"));
	const Inputs::Camera& camera= in.cameras[id.cameraIndex];
	const StageStatus device= computeCameraDevice(camera, id.cameraIndex);

	switch (id.stage)
	{
	case S::Camera:
		if (device.state != eStageState::Running)
			return device;
		if (camera.bTrackingPaused)
			return inactive(text("valPaused"), text("whyTrackingPaused"));
		return device;
	case S::Video:
		if (device.state != eStageState::Running)
			return device;
		if (!camera.bIntrinsics)
			return degraded(text("valDistorted"), text("whyNoIntrinsicsVideo"));
		return device;
	case S::HandModels:
		if (device.state != eStageState::Running)
			return inactive(text("valNoFrames"), text("whyCameraNotRunning"));
		if (!camera.bHandPipelineReady)
			return notConfigured(text("valFailed"), text("whyHandPipelineFailed"));
		if (camera.bTrackingPaused)
			return inactive(text("valPaused"), text("whyTrackingPaused"));
		return running(text("valHandsFmt", camera.handsTracked));
	case S::BodyModel:
		if (!camera.bBodyEnabled)
			return inactive(text("valOff"), text("whyBodyDisabled"));
		if (device.state != eStageState::Running)
			return inactive(text("valNoFrames"), text("whyCameraNotRunning"));
		if (!camera.bBodyModelLoaded)
			return notConfigured(text("valNotLoaded"), text("whyBodyModelMissing"));
		if (camera.bTrackingPaused)
			return inactive(text("valPaused"), text("whyTrackingPaused"));
		if (!camera.bBodyValid)
			return running(text("valNoPerson"));
		return running(text("valScoreFmt", camera.bodyConfidence));
	case S::Lift:
		if (device.state != eStageState::Running)
			return inactive(text("valNoFrames"), text("whyCameraNotRunning"));
		if (!camera.bIntrinsics)
			return notConfigured(text("val2dOnly"), text("whyNoIntrinsicsLift"));
		if (in.bDirect)
			return running(text("valCameraSpace"), text("whyDirectNoWorld"));
		if (!camera.bExtrinsics)
			return notConfigured(text("valCameraSpace"), text("whyNoExtrinsicsLift"));
		return running(text("valWorld"));
	default:
		break;
	}
	return StageStatus();
}

StageStatus computeFusion(const Inputs& in)
{
	if (isWithoutWorld(in))
		return notConfigured(text("valPassthrough"), text("whyFusionPassthrough"));
	if (in.path == Inputs::ePath::Idle || in.path == Inputs::ePath::Unknown)
		return inactive(text("valIdle"), text("whyNoCameraFrames"));
	const int handCount= countTrackedHands(in);
	if (handCount == 0)
		return running(text("valNoHands"));
	float confidence= 1.f;
	for (const Inputs::Hand& hand : in.hands)
	{
		if (hand.bTracked)
			confidence= std::min(confidence, hand.confidence);
	}
	if (handCount == 1)
		return running(text("valOneHandConfFmt", confidence));
	return running(text("valConfFmt", confidence));
}

StageStatus computeImu(const Inputs& in)
{
	if (!in.bImuEnabled)
		return inactive(text("valOff"), text("whyImuDisabled"));
	if (isUnrecorded(in))
		return notRecorded();
	const int connected= (in.imu[0].bConnected ? 1 : 0) + (in.imu[1].bConnected ? 1 : 0);
	if (connected == 0)
		return notConfigured(text("valImuCountFmt", 0, 2), text("whyImuNotConnected"));
	if (connected == 1)
		return degraded(text("valImuCountFmt", 1, 2), text("whyImuOneMissing"));
	if (!in.imu[0].bCalibrated || !in.imu[1].bCalibrated)
		return notConfigured(text("valUncalibrated"), text("whyImuNotCalibrated"));
	if (isWithoutWorld(in))
		return degraded(text("valNoYawAnchor"), text("whyImuNoYawAnchor"));
	const int streaming= (in.imu[0].bStreaming ? 1 : 0) + (in.imu[1].bStreaming ? 1 : 0);
	if (streaming < 2)
		return degraded(text("valImuCountFmt", streaming, 2), text("whyImuSilent"));
	if (!in.imu[0].bOrientationValid || !in.imu[1].bOrientationValid)
		return degraded(text("valConverging"), text("whyImuConverging"));
	// A replay shows the wrists as they were recorded, not a filter re-run
	return running(text("valRateFmt", std::min(in.imu[0].sampleRateHz, in.imu[1].sampleRateHz)),
				   in.bReplay ? text("whyImuReplay") : StageText());
}

StageStatus computeBodySolver(const Inputs& in)
{
	int bodyCamera= -1;
	for (int cameraIndex= 0; cameraIndex < clampedCameraCount(in); ++cameraIndex)
	{
		if (in.cameras[cameraIndex].bBodyEnabled)
		{
			bodyCamera= cameraIndex;
			break;
		}
	}
	if (bodyCamera < 0)
		return inactive(text("valNoBodyCamera"), text("whyNoBodyCamera"));
	const Inputs::Camera& camera= in.cameras[bodyCamera];
	if (!camera.bBodyModelLoaded)
		return notConfigured(text("valNotLoaded"), text("whyBodyModelMissing"));
	if (isWithoutWorld(in))
		return inactive(text("valNoWrists"), text("whyBodyNeedsWorld"));
	if (!camera.bIntrinsics || !camera.bExtrinsics)
		return notConfigured(text("valUncalibrated"), text("whyBodyCameraUncalibratedFmt", bodyCamera + 1));
	const int shoulders= (in.hands[0].bShoulder ? 1 : 0) + (in.hands[1].bShoulder ? 1 : 0);
	if (!camera.bBodyValid && shoulders == 0 && !in.bHeadValid)
		return running(text("valNoPerson"));
	const bool bImuForearms= in.bImuEnabled && (in.imu[0].bOrientationValid || in.imu[1].bOrientationValid);
	return running(text("valShouldersFmt", shoulders), bImuForearms ? text("whyBodyImuForearms") : StageText());
}

int countDirect(const Inputs& in, bool Inputs::DirectArm::*field)
{
	return (in.directArms[0].*field ? 1 : 0) + (in.directArms[1].*field ? 1 : 0);
}

StageStatus computeDirect(const Inputs& in, S stage)
{
	if (!in.bDirectIntrinsics)
		return notConfigured(text("valNoIntrinsics"), text("whyDirectNoIntrinsics"));
	if (!in.bAvatarLoaded)
		return notConfigured(text("valNoAvatar"), text("whyDirectNoAvatar"));
	const StageText noNeutral= in.bDirectNeutral ? StageText() : text("whyDirectNoNeutral");
	const int resting= countDirect(in, &Inputs::DirectArm::bResting);
	const bool bEasing= (in.directArms[0].bResting && in.directArms[0].restBlend < 1.f) ||
		(in.directArms[1].bResting && in.directArms[1].restBlend < 1.f);

	switch (stage)
	{
	case S::DirectSolver:
		if (!in.bDirectNeutral)
			return degraded(text("valNoNeutral"), noNeutral);
		if (!in.bDirectTorso)
			return running(text("valNoPerson"));
		if (resting == 2)
			return running(text("valArmsResting"));
		return running(text("valDepthFmt", in.directShoulderDepthMeters));
	case S::Filters:
		return running(text("valHandsSeenFmt", countDirect(in, &Inputs::DirectArm::bHandSeen)));
	case S::Torso:
		if (!in.bDirectTorso)
			return running(text("valNoPerson"), noNeutral);
		return in.bDirectNeutral ? running(text("valLeanFmt", in.directLeanDegrees))
								 : degraded(text("valNoNeutral"), noNeutral);
	case S::ArmChain:
		return running(text("valChainsFmt", countDirect(in, &Inputs::DirectArm::bChain)));
	case S::Palm:
		return running(text("valPalmsFmt", countDirect(in, &Inputs::DirectArm::bHandMeasured)));
	case S::HoldRest:
		if (resting == 0)
			return running(text("valAllLive"));
		return running(text(bEasing ? "valEasingFmt" : "valRestingFmt", resting));
	default:
		break;
	}
	return StageStatus();
}

int countMeasuredArms(const Inputs& in)
{
	if (in.bDirect)
		return countDirect(in, &Inputs::DirectArm::bChain);
	int count= 0;
	for (const Inputs::Hand& hand : in.hands)
		count+= (hand.bTracked && hand.bWorldPose) ? 1 : 0;
	return count;
}

StageStatus computeSourcePose(const Inputs& in)
{
	if (isWithoutWorld(in))
		return degraded(text("valFaceOnly"), text("whySourceNoArms"));
	return running(text("valArmsFmt", countMeasuredArms(in)));
}

StageStatus computeRetarget(const Inputs& in)
{
	if (!in.bAvatarLoaded)
		return notConfigured(text("valNoAvatar"), text(in.bDirect ? "whyDirectNoAvatar" : "whyRetargetNoAvatar"));
	if (isWithoutWorld(in))
		return inactive(text("valNothingToPose"), text("whySourceNoArms"));
	return running(text("valArmsPlacedFmt", (in.bArmPlaced[0] ? 1 : 0) + (in.bArmPlaced[1] ? 1 : 0)));
}

StageStatus computeOutput(const Inputs& in, S stage)
{
	if (!in.bOscEnabled)
		return inactive(text("valOff"), text("whyOscDisabled"));
	if (stage == S::DeliveryRules)
		return running(text("valMinConfFmt", in.oscMinConfidence));
	if (isUnrecorded(in))
		return notRecorded();
	if (!in.bOscRunning)
		return notConfigured(text("valSocketClosed"), text("whyOscSocket"));
	if (isWithoutWorld(in))
		return degraded(text("valFaceOnly"), text("whyOutputNoWorld"));
	if (stage == S::VmcStream)
		return running(text("valPortFmt", in.oscPort));
	return running(text("valSendRateFmt", in.oscSendRateHz));
}

StageStatus computePhone(const Inputs& in)
{
	if (!in.bFaceEnabled)
		return inactive(text("valOff"), text("whyPhoneDisabled"));
	if (isUnrecorded(in))
		return notRecorded();
	if (!in.bFaceBound)
		return notConfigured(text("valPortBusy"), text("whyPhonePort"));
	if (!in.bFaceStreaming)
		return degraded(text("valWaiting"), text("whyPhoneWaiting"));
	if (!in.bFaceTracked)
		return running(text("valNoFace"));
	return running(text("valRateFmt", in.faceDatagramsPerSecond));
}

StageStatus computeMicrophone(const Inputs& in)
{
	if (!in.bAudioEnabled)
		return inactive(text("valOff"), text("whyMicDisabled"));
	if (isUnrecorded(in))
		return notRecorded();
	if (!in.bAudioDeviceOpen)
		return notConfigured(text("valNoDevice"), text("whyMicDevice"));
	if (!in.bAudioBackendReady)
		return notConfigured(text("valModelUnavailable"), text("whyMicBackend"));
	if (!in.bAudioGateOpen)
		return running(text("valGated"));
	return running(text("valLevelFmt", in.audioLevelDbfs));
}

StageStatus computeFace(const Inputs& in)
{
	if (!in.bFaceEnabled && !in.bAudioEnabled)
		return inactive(text("valOff"), text("whyFaceOff"));
	if (isUnrecorded(in))
		return notRecorded();
	const StageStatus phone= computePhone(in);
	const StageStatus microphone= computeMicrophone(in);
	// A source the project wants and cannot run is the face's problem too
	if (phone.state == eStageState::NotConfigured)
		return notConfigured(phone.value, phone.reason);
	if (microphone.state == eStageState::NotConfigured)
		return notConfigured(microphone.value, microphone.reason);
	const bool bPhoneLive= in.bFaceEnabled && in.bFaceStreaming;
	const bool bMicLive= in.bAudioEnabled && in.bAudioDeviceOpen && in.bAudioBackendReady;
	if (bPhoneLive && bMicLive)
		return running(text("valPhoneAndMic"));
	if (bPhoneLive)
		return running(text("valPhoneOnly"));
	if (bMicLive)
		return running(text("valMicOnly"));
	return degraded(text("valNoSource"), text("whyFaceNoSource"));
}

StageStatus computeProfile(const Inputs& in)
{
	if (in.profileProblemCount > 0)
		return degraded(text("valProblemsFmt", in.profileProblemCount), text("whyProfileProblems"));
	return running(text(in.bProfileAuto ? "valAutoTargetsFmt" : "valTargetsFmt", in.profileTargetCount));
}
} // namespace

namespace PipelineModel
{
std::vector<PipelineStageId> getTopRow(bool bDirect)
{
	if (bDirect)
	{
		return {makeId(S::Capture), makeId(S::DirectSolver), makeId(S::SourcePose), makeId(S::Retarget),
				makeId(S::Output)};
	}
	return {makeId(S::Capture),    makeId(S::Fusion),   makeId(S::Imu),   makeId(S::BodySolver),
			makeId(S::SourcePose), makeId(S::Retarget), makeId(S::Output)};
}

PipelineStageId getSideStage()
{
	return makeId(S::Face);
}

std::vector<std::vector<PipelineStageId>> getSubStageLanes(const PipelineStageId& stage, int cameraCount)
{
	std::vector<std::vector<PipelineStageId>> lanes;
	switch (getTopLevel(stage).stage)
	{
	case S::Capture:
		for (int cameraIndex= 0; cameraIndex < std::clamp(cameraCount, 0, Inputs::k_maxCameras); ++cameraIndex)
		{
			lanes.push_back({makeId(S::Camera, cameraIndex), makeId(S::Video, cameraIndex),
							 makeId(S::HandModels, cameraIndex), makeId(S::BodyModel, cameraIndex),
							 makeId(S::Lift, cameraIndex)});
		}
		break;
	case S::DirectSolver:
		lanes.push_back({makeId(S::Filters), makeId(S::Torso), makeId(S::ArmChain), makeId(S::Palm),
						 makeId(S::HoldRest)});
		break;
	case S::Face:
		lanes.push_back({makeId(S::Phone), makeId(S::Microphone), makeId(S::Profile)});
		break;
	case S::Output:
		lanes.push_back({makeId(S::DeliveryRules), makeId(S::VmcStream)});
		break;
	default:
		break;
	}
	return lanes;
}

PipelineStageId getTopLevel(const PipelineStageId& stage)
{
	if (!stage.isValid())
		return stage;
	return makeId(k_stages[(int)stage.stage].parent);
}

std::vector<PipelineStageId> getAllStages(bool bDirect, int cameraCount)
{
	std::vector<PipelineStageId> all;
	std::vector<PipelineStageId> topLevel= getTopRow(bDirect);
	topLevel.push_back(getSideStage());
	for (const PipelineStageId& top : topLevel)
	{
		all.push_back(top);
		for (const std::vector<PipelineStageId>& lane : getSubStageLanes(top, cameraCount))
			all.insert(all.end(), lane.begin(), lane.end());
	}
	return all;
}

bool isLaneFlow(ePipelineStage topLevel)
{
	// The face's sources are a set feeding the profile, not a chain
	return topLevel != S::Face;
}

const char* getNameKey(ePipelineStage stage)
{
	static std::string keys[(int)S::Count];
	std::string& key= keys[(int)stage];
	if (key.empty())
		key= std::string("pipeline.stage") + k_stages[(int)stage].name;
	return key.c_str();
}

const char* getDescriptionKey(ePipelineStage stage)
{
	static std::string keys[(int)S::Count];
	std::string& key= keys[(int)stage];
	if (key.empty())
		key= std::string("pipeline.desc") + k_stages[(int)stage].name;
	return key.c_str();
}

std::string toString(const PipelineStageId& stage)
{
	if (!stage.isValid())
		return std::string();
	const StageInfo& info= k_stages[(int)stage.stage];
	if (isPerCamera(stage.stage))
	{
		std::string out= std::string("capture.cam") + std::to_string(stage.cameraIndex);
		if (stage.stage != S::Camera)
			out+= std::string(".") + info.token;
		return out;
	}
	if (info.parent == stage.stage)
		return info.token;
	return std::string(k_stages[(int)info.parent].token) + "." + info.token;
}

bool fromString(const std::string& text, PipelineStageId& outStage)
{
	outStage= PipelineStageId();
	const std::string cameraPrefix= "capture.cam";
	if (text.compare(0, cameraPrefix.size(), cameraPrefix) == 0)
	{
		const size_t digitsBegin= cameraPrefix.size();
		size_t digitsEnd= digitsBegin;
		while (digitsEnd < text.size() && text[digitsEnd] >= '0' && text[digitsEnd] <= '9')
			++digitsEnd;
		if (digitsEnd == digitsBegin)
			return false;
		const int cameraIndex= std::atoi(text.substr(digitsBegin, digitsEnd - digitsBegin).c_str());
		if (digitsEnd == text.size())
		{
			outStage= makeId(S::Camera, cameraIndex);
			return true;
		}
		if (text[digitsEnd] != '.')
			return false;
		const std::string token= text.substr(digitsEnd + 1);
		for (const S stage : {S::Video, S::HandModels, S::BodyModel, S::Lift})
		{
			if (token == k_stages[(int)stage].token)
			{
				outStage= makeId(stage, cameraIndex);
				return true;
			}
		}
		return false;
	}
	for (int index= 0; index < (int)S::Count; ++index)
	{
		const S stage= (S)index;
		if (isPerCamera(stage))
			continue;
		if (toString(makeId(stage)) == text)
		{
			outStage= makeId(stage);
			return true;
		}
	}
	return false;
}

StageStatus computeStatus(const PipelineStatusInputs& inputs, const PipelineStageId& stage)
{
	switch (stage.stage)
	{
	case S::Capture:
		return computeCapture(inputs);
	case S::Camera:
	case S::Video:
	case S::HandModels:
	case S::BodyModel:
	case S::Lift:
		return computePerCamera(inputs, stage);
	case S::Fusion:
		return computeFusion(inputs);
	case S::Imu:
		return computeImu(inputs);
	case S::BodySolver:
		return computeBodySolver(inputs);
	case S::DirectSolver:
	case S::Filters:
	case S::Torso:
	case S::ArmChain:
	case S::Palm:
	case S::HoldRest:
		return computeDirect(inputs, stage.stage);
	case S::SourcePose:
		return computeSourcePose(inputs);
	case S::Retarget:
		return computeRetarget(inputs);
	case S::Output:
	case S::DeliveryRules:
	case S::VmcStream:
		return computeOutput(inputs, stage.stage);
	case S::Face:
		return computeFace(inputs);
	case S::Phone:
		return computePhone(inputs);
	case S::Microphone:
		return computeMicrophone(inputs);
	case S::Profile:
		return computeProfile(inputs);
	default:
		break;
	}
	return StageStatus();
}

const char* getStateKey(eStageState state)
{
	switch (state)
	{
	case eStageState::Running:
		return "pipeline.stateRunning";
	case eStageState::Degraded:
		return "pipeline.stateDegraded";
	case eStageState::Inactive:
		return "pipeline.stateInactive";
	case eStageState::NotConfigured:
		return "pipeline.stateNotConfigured";
	}
	return "pipeline.stateInactive";
}
} // namespace PipelineModel
