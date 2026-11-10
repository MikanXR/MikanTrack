#include "TrackingRecording.h"

#include <algorithm>
#include <cstring>

#include "TrackingJson.h"

using json= nlohmann::json;
using namespace TrackingJson;

namespace
{
constexpr uint64_t k_fnvBasis= 0xcbf29ce484222325ull;
constexpr uint64_t k_fnvPrime= 0x100000001b3ull;

void fnvByte(uint64_t& hash, uint8_t byte)
{
	hash^= byte;
	hash*= k_fnvPrime;
}

void fnvFloat(uint64_t& hash, float value)
{
	uint32_t bits= 0;
	std::memcpy(&bits, &value, sizeof(bits));
	fnvByte(hash, (uint8_t)(bits & 0xff));
	fnvByte(hash, (uint8_t)((bits >> 8) & 0xff));
	fnvByte(hash, (uint8_t)((bits >> 16) & 0xff));
	fnvByte(hash, (uint8_t)((bits >> 24) & 0xff));
}

std::string checksumToHex(uint64_t checksum)
{
	char buffer[17];
	snprintf(buffer, sizeof(buffer), "%016llx", (unsigned long long)checksum);
	return buffer;
}

uint64_t checksumFromHex(const std::string& hex)
{
	return strtoull(hex.c_str(), nullptr, 16);
}

json skeletonToJson(const HandSkeleton& skeleton)
{
	json baseInPalm= json::array();
	json phalanx= json::array();
	json neutralDir= json::array();
	for (int fingerIndex= 0; fingerIndex < FINGER_COUNT; ++fingerIndex)
	{
		baseInPalm.push_back(vec3ToJson(skeleton.baseInPalm[fingerIndex]));
		phalanx.push_back(json::array({skeleton.phalanxLengths[fingerIndex][0],
									   skeleton.phalanxLengths[fingerIndex][1],
									   skeleton.phalanxLengths[fingerIndex][2]}));
		neutralDir.push_back(vec3ToJson(skeleton.neutralDirInPalm[fingerIndex]));
	}
	return {{"baseInPalm", baseInPalm}, {"phalanx", phalanx}, {"neutralDir", neutralDir}};
}

void skeletonFromJson(const json& j, HandSkeleton& outSkeleton)
{
	outSkeleton= HandSkeleton();
	if (!j.is_object())
		return;
	const json& baseInPalm= j.value("baseInPalm", json::array());
	const json& phalanx= j.value("phalanx", json::array());
	const json& neutralDir= j.value("neutralDir", json::array());
	for (int fingerIndex= 0; fingerIndex < FINGER_COUNT; ++fingerIndex)
	{
		if (fingerIndex < (int)baseInPalm.size())
			outSkeleton.baseInPalm[fingerIndex]= vec3FromJson(baseInPalm[fingerIndex]);
		if (fingerIndex < (int)phalanx.size() && phalanx[fingerIndex].is_array() &&
			phalanx[fingerIndex].size() == 3)
		{
			for (int bone= 0; bone < 3; ++bone)
				outSkeleton.phalanxLengths[fingerIndex][bone]= floatFromJson(phalanx[fingerIndex][bone]);
		}
		if (fingerIndex < (int)neutralDir.size())
			outSkeleton.neutralDirInPalm[fingerIndex]= vec3FromJson(neutralDir[fingerIndex]);
	}
}

json handInputToJson(const RecordedHandInput& hand)
{
	if (!hand.tracked)
		return {{"tracked", false}};

	json out= {
		{"tracked", true},
		{"side", hand.side},
		{"slotId", hand.slotId},
		{"presence", hand.presence},
		{"handednessScore", hand.handednessScore},
		{"rightProb", hand.rightProb},
		{"imagePoints", landmarksToJson(hand.imagePoints)},
		{"modelPoints", landmarksToJson(hand.modelPoints)},
	};
	if (hand.imageQuality.valid)
		out["imageQuality"]= imageQualityToJson(hand.imageQuality);
	return out;
}

void handInputFromJson(const json& j, RecordedHandInput& outHand)
{
	outHand= RecordedHandInput();
	outHand.tracked= j.value("tracked", false);
	if (!outHand.tracked)
		return;
	outHand.side= j.value("side", 0);
	outHand.slotId= j.value("slotId", -1);
	outHand.presence= j.value("presence", 0.f);
	outHand.handednessScore= j.value("handednessScore", 0.f);
	outHand.rightProb= j.value("rightProb", 0.5f);
	landmarksFromJson(j.value("imagePoints", json::array()), outHand.imagePoints);
	landmarksFromJson(j.value("modelPoints", json::array()), outHand.modelPoints);
	if (j.contains("imageQuality"))
	{
		const json& quality= j["imageQuality"];
		outHand.imageQuality.valid= true;
		outHand.imageQuality.meanLuma= quality.value("meanLuma", 0.f);
		outHand.imageQuality.shadowClipRatio= quality.value("shadowClipRatio", 0.f);
		outHand.imageQuality.highlightClipRatio= quality.value("highlightClipRatio", 0.f);
		outHand.imageQuality.contrast= quality.value("contrast", 0.f);
		outHand.imageQuality.backgroundSeparation= quality.value("backgroundSeparation", 0.f);
		outHand.imageQuality.sharpness= quality.value("sharpness", 0.f);
		outHand.imageQuality.noise= quality.value("noise", 0.f);
	}
}

json cameraInputToJson(const RecordedCameraInput& camera)
{
	json out= {
		{"cam", camera.cameraIndex},
		{"tMs", camera.timestampMs},
		{"valid", camera.valid},
		{"refLenM", camera.refLengthMeters},
		{"frameIndex", camera.frameIndex},
		{"w", camera.frameWidth},
		{"h", camera.frameHeight},
		{"captureFps", camera.captureFps},
		{"inferenceMs", camera.inferenceMs},
		{"lumaInstability", camera.lumaInstability},
		{"lumaFlickerHz", camera.lumaFlickerHz},
		{"hands", json::array({handInputToJson(camera.hands[0]), handInputToJson(camera.hands[1])})},
	};
	if (camera.bHaveBodyPose)
		out["body"]= bodyPoseToJson(camera.body);
	if (camera.bHaveCaptureTimings)
		out["timings"]= captureTimingsToJson(camera.captureTimings);
	return out;
}

bool cameraInputFromJson(const json& j, RecordedCameraInput& outCamera)
{
	outCamera= RecordedCameraInput();
	outCamera.cameraIndex= j.value("cam", -1);
	if (outCamera.cameraIndex < 0)
		return false;
	outCamera.timestampMs= j.value("tMs", 0.0);
	outCamera.valid= j.value("valid", false);
	outCamera.refLengthMeters= j.value("refLenM", 0.f);
	outCamera.frameIndex= j.value("frameIndex", (int64_t)-1);
	outCamera.frameWidth= j.value("w", 0);
	outCamera.frameHeight= j.value("h", 0);
	outCamera.captureFps= j.value("captureFps", 0.f);
	outCamera.inferenceMs= j.value("inferenceMs", 0.f);
	outCamera.lumaInstability= j.value("lumaInstability", 0.f);
	outCamera.lumaFlickerHz= j.value("lumaFlickerHz", 0.f);
	const json& hands= j.value("hands", json::array());
	for (int sideIndex= 0; sideIndex < 2 && sideIndex < (int)hands.size(); ++sideIndex)
		handInputFromJson(hands[sideIndex], outCamera.hands[sideIndex]);
	// (Old recordings may carry a "depth" block from the removed RealSense
	// experiment; it is simply not read - those frames replay through PnP.)
	if (j.contains("body"))
	{
		outCamera.bHaveBodyPose= true;
		bodyPoseFromJson(j["body"], outCamera.body);
	}
	// A camera input without the block (every recording before it was
	// carried) has no capture timings
	if (j.contains("timings") && j["timings"].is_object())
	{
		outCamera.bHaveCaptureTimings= true;
		captureTimingsFromJson(j["timings"], outCamera.captureTimings);
	}
	return true;
}

// Type-tolerant reads for the display-only telemetry block: a missing key, a
// null (a non-finite float) or a value of another type reads as the default,
// and keys this build does not know are ignored
const json* findMember(const json& j, const char* key)
{
	const auto it= j.find(key);
	return it != j.end() ? &*it : nullptr;
}

float readFloat(const json& j, const char* key, float defaultValue)
{
	const json* value= findMember(j, key);
	return value != nullptr && value->is_number() ? value->get<float>() : defaultValue;
}

int readInt(const json& j, const char* key, int defaultValue)
{
	const json* value= findMember(j, key);
	return value != nullptr && value->is_number_integer() ? value->get<int>() : defaultValue;
}

bool readBool(const json& j, const char* key, bool defaultValue)
{
	const json* value= findMember(j, key);
	return value != nullptr && value->is_boolean() ? value->get<bool>() : defaultValue;
}

const json& readObject(const json& j, const char* key)
{
	static const json k_emptyObject= json::object();
	const json* value= findMember(j, key);
	return value != nullptr && value->is_object() ? *value : k_emptyObject;
}

const json& readArray(const json& j, const char* key)
{
	static const json k_emptyArray= json::array();
	const json* value= findMember(j, key);
	return value != nullptr && value->is_array() ? *value : k_emptyArray;
}

float numberOr(const json& value, float defaultValue)
{
	return value.is_number() ? value.get<float>() : defaultValue;
}

json telemetryToJson(const RecordedTelemetry& telemetry)
{
	const int cameraCount= std::clamp(telemetry.cameraCount, 0, RecordedTelemetry::k_maxCameras);

	json phaseMs= json::array();
	for (float phase : telemetry.phaseMs)
		phaseMs.push_back(phase);

	json confidence= json::array();
	json cameras= json::array();
	for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
	{
		confidence.push_back(json::array({telemetry.observationConfidence[cameraIndex][0],
										  telemetry.observationConfidence[cameraIndex][1]}));
		const RecordedTelemetry::Camera& camera= telemetry.cameras[cameraIndex];
		cameras.push_back({
			{"open", camera.deviceOpen},
			{"streaming", camera.streaming},
			{"fps", camera.deviceFrameRate},
			{"dropped", camera.droppedFrames},
			{"queued", camera.queuedFrames},
			{"copyMs", camera.callbackCopyMs},
			{"tracking", camera.trackingEnabled},
			{"handReady", camera.handPipelineReady},
			{"bodyModel", camera.bodyModelLoaded},
		});
	}

	json imu= json::array();
	for (const RecordedImuStatus& status : telemetry.imu)
	{
		imu.push_back({
			{"connected", status.deviceConnected},
			{"streaming", status.streaming},
			{"calibrated", status.calibrated},
			{"orientationValid", status.orientationValid},
			{"rateHz", status.sampleRateHz},
			{"battery", status.batteryLevel},
			{"yawDrift", status.yawDriftDegPerSecond},
			{"rollError", status.rollErrorDegrees},
		});
	}

	json out= {
		{"path", telemetry.path},
		{"phaseMs", phaseMs},
		{"totalMs", telemetry.totalMs},
		{"dominant", json::array({telemetry.dominantCamera[0], telemetry.dominantCamera[1]})},
		{"obsConf", confidence},
		{"autoScale", telemetry.autoHandScaleFactor},
		{"cameras", cameras},
		{"imuEnabled", telemetry.imuEnabled},
		{"imu", imu},
	};
	out["osc"]= {
		{"running", telemetry.oscRunning},
		{"rateHz", telemetry.oscSendRateHz},
		{"packets", telemetry.oscLastPacketCount},
		{"bytes", telemetry.oscLastFrameBytes},
	};
	out["face"]= {
		{"enabled", telemetry.face.enabled},
		{"bound", telemetry.face.bound},
		{"streaming", telemetry.face.streaming},
		{"tracked", telemetry.face.faceTracked},
		{"rateHz", telemetry.face.datagramsPerSecond},
	};
	out["audio"]= {
		{"enabled", telemetry.audio.enabled},
		{"deviceOpen", telemetry.audio.deviceOpen},
		{"backendReady", telemetry.audio.backendReady},
		{"gateOpen", telemetry.audio.gateOpen},
		{"levelDbfs", telemetry.audio.levelDbfs},
		{"inferMs", telemetry.audio.inferMsEma},
	};
	return out;
}

void telemetryFromJson(const json& j, RecordedTelemetry& outTelemetry)
{
	outTelemetry= RecordedTelemetry();
	if (!j.is_object())
		return;
	const RecordedTelemetry defaults;
	outTelemetry.present= true;
	outTelemetry.path= readInt(j, "path", defaults.path);
	const json& phaseMs= readArray(j, "phaseMs");
	for (int phase= 0; phase < RecordedTelemetry::k_phaseCount && phase < (int)phaseMs.size(); ++phase)
		outTelemetry.phaseMs[phase]= numberOr(phaseMs[phase], 0.f);
	outTelemetry.totalMs= readFloat(j, "totalMs", defaults.totalMs);

	const json& osc= readObject(j, "osc");
	outTelemetry.oscRunning= readBool(osc, "running", defaults.oscRunning);
	outTelemetry.oscSendRateHz= readFloat(osc, "rateHz", defaults.oscSendRateHz);
	outTelemetry.oscLastPacketCount= readInt(osc, "packets", defaults.oscLastPacketCount);
	outTelemetry.oscLastFrameBytes= readInt(osc, "bytes", defaults.oscLastFrameBytes);

	const json& dominant= readArray(j, "dominant");
	for (int sideIndex= 0; sideIndex < 2 && sideIndex < (int)dominant.size(); ++sideIndex)
	{
		if (dominant[sideIndex].is_number_integer())
			outTelemetry.dominantCamera[sideIndex]= dominant[sideIndex].get<int>();
	}
	const json& confidence= readArray(j, "obsConf");
	for (int cameraIndex= 0; cameraIndex < RecordedTelemetry::k_maxCameras && cameraIndex < (int)confidence.size();
		 ++cameraIndex)
	{
		const json& sides= confidence[cameraIndex];
		for (int sideIndex= 0; sides.is_array() && sideIndex < 2 && sideIndex < (int)sides.size(); ++sideIndex)
			outTelemetry.observationConfidence[cameraIndex][sideIndex]= numberOr(sides[sideIndex], -1.f);
	}
	outTelemetry.autoHandScaleFactor= readFloat(j, "autoScale", defaults.autoHandScaleFactor);

	const json& cameras= readArray(j, "cameras");
	outTelemetry.cameraCount= std::min((int)cameras.size(), RecordedTelemetry::k_maxCameras);
	for (int cameraIndex= 0; cameraIndex < outTelemetry.cameraCount; ++cameraIndex)
	{
		const json& camera= cameras[cameraIndex];
		if (!camera.is_object())
			continue;
		const RecordedTelemetry::Camera cameraDefaults;
		RecordedTelemetry::Camera& outCamera= outTelemetry.cameras[cameraIndex];
		outCamera.deviceOpen= readBool(camera, "open", cameraDefaults.deviceOpen);
		outCamera.streaming= readBool(camera, "streaming", cameraDefaults.streaming);
		outCamera.deviceFrameRate= readFloat(camera, "fps", cameraDefaults.deviceFrameRate);
		outCamera.droppedFrames= readInt(camera, "dropped", cameraDefaults.droppedFrames);
		outCamera.queuedFrames= readInt(camera, "queued", cameraDefaults.queuedFrames);
		outCamera.callbackCopyMs= readFloat(camera, "copyMs", cameraDefaults.callbackCopyMs);
		outCamera.trackingEnabled= readBool(camera, "tracking", cameraDefaults.trackingEnabled);
		outCamera.handPipelineReady= readBool(camera, "handReady", cameraDefaults.handPipelineReady);
		outCamera.bodyModelLoaded= readBool(camera, "bodyModel", cameraDefaults.bodyModelLoaded);
	}

	outTelemetry.imuEnabled= readBool(j, "imuEnabled", defaults.imuEnabled);
	const json& imu= readArray(j, "imu");
	for (int sideIndex= 0; sideIndex < 2 && sideIndex < (int)imu.size(); ++sideIndex)
	{
		const json& status= imu[sideIndex];
		if (!status.is_object())
			continue;
		const RecordedImuStatus statusDefaults;
		RecordedImuStatus& outStatus= outTelemetry.imu[sideIndex];
		outStatus.deviceConnected= readBool(status, "connected", statusDefaults.deviceConnected);
		outStatus.streaming= readBool(status, "streaming", statusDefaults.streaming);
		outStatus.calibrated= readBool(status, "calibrated", statusDefaults.calibrated);
		outStatus.orientationValid= readBool(status, "orientationValid", statusDefaults.orientationValid);
		outStatus.sampleRateHz= readFloat(status, "rateHz", statusDefaults.sampleRateHz);
		outStatus.batteryLevel= readFloat(status, "battery", statusDefaults.batteryLevel);
		outStatus.yawDriftDegPerSecond= readFloat(status, "yawDrift", statusDefaults.yawDriftDegPerSecond);
		outStatus.rollErrorDegrees= readFloat(status, "rollError", statusDefaults.rollErrorDegrees);
	}

	const json& face= readObject(j, "face");
	outTelemetry.face.enabled= readBool(face, "enabled", defaults.face.enabled);
	outTelemetry.face.bound= readBool(face, "bound", defaults.face.bound);
	outTelemetry.face.streaming= readBool(face, "streaming", defaults.face.streaming);
	outTelemetry.face.faceTracked= readBool(face, "tracked", defaults.face.faceTracked);
	outTelemetry.face.datagramsPerSecond= readFloat(face, "rateHz", defaults.face.datagramsPerSecond);

	const json& audio= readObject(j, "audio");
	outTelemetry.audio.enabled= readBool(audio, "enabled", defaults.audio.enabled);
	outTelemetry.audio.deviceOpen= readBool(audio, "deviceOpen", defaults.audio.deviceOpen);
	outTelemetry.audio.backendReady= readBool(audio, "backendReady", defaults.audio.backendReady);
	outTelemetry.audio.gateOpen= readBool(audio, "gateOpen", defaults.audio.gateOpen);
	outTelemetry.audio.levelDbfs= readFloat(audio, "levelDbfs", defaults.audio.levelDbfs);
	outTelemetry.audio.inferMsEma= readFloat(audio, "inferMs", defaults.audio.inferMsEma);
}

json poseOutputToJson(const RecordedPoseOutput& pose)
{
	if (!pose.tracked)
		return {{"tracked", false}};

	json out= {
		{"tracked", true},
		{"side", pose.side},
		{"presence", pose.presence},
		{"visibility", pose.visibility},
		{"confidence", pose.confidence},
		{"fkReprojectionPx", pose.fkReprojectionPx},
		{"stereoTriangulated", pose.stereoTriangulated},
		{"fingers", fingersToJson(pose.fingers)},
		{"skeleton", skeletonToJson(pose.skeleton)},
	};
	if (pose.hasCameraPose)
	{
		out["palmPosC"]= vec3ToJson(pose.palmPositionCamera);
		out["palmQuatC"]= quatToJson(pose.palmOrientationCamera);
	}
	if (pose.hasWorldPose)
	{
		out["palmPosW"]= vec3ToJson(pose.palmPositionWorld);
		out["palmQuatW"]= quatToJson(pose.palmOrientationWorld);
	}
	return out;
}

void poseOutputFromJson(const json& j, RecordedPoseOutput& outPose)
{
	outPose= RecordedPoseOutput();
	outPose.tracked= j.value("tracked", false);
	if (!outPose.tracked)
		return;
	outPose.side= j.value("side", 0);
	outPose.presence= j.value("presence", 0.f);
	outPose.visibility= j.value("visibility", 0.f);
	outPose.confidence= j.value("confidence", 0.f);
	outPose.fkReprojectionPx= j.value("fkReprojectionPx", 0.f);
	outPose.stereoTriangulated= j.value("stereoTriangulated", false);
	fingersFromJson(j.value("fingers", json::array()), outPose.fingers);
	skeletonFromJson(j.value("skeleton", json::object()), outPose.skeleton);
	if (j.contains("palmPosC"))
	{
		outPose.hasCameraPose= true;
		outPose.palmPositionCamera= vec3FromJson(j["palmPosC"]);
		outPose.palmOrientationCamera= quatFromJson(j.value("palmQuatC", json::array()));
	}
	if (j.contains("palmPosW"))
	{
		outPose.hasWorldPose= true;
		outPose.palmPositionWorld= vec3FromJson(j["palmPosW"]);
		outPose.palmOrientationWorld= quatFromJson(j.value("palmQuatW", json::array()));
	}
}
} // namespace

namespace TrackingRecording
{
uint64_t computeFusedChecksum(const TrackingFrameResult& preImuFused)
{
	uint64_t hash= k_fnvBasis;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= preImuFused.poses[sideIndex];
		fnvByte(hash, pose.tracked ? 1 : 0);
		if (!pose.tracked)
			continue;

		fnvByte(hash, pose.hasWorldPose ? 1 : 0);
		const glm::vec3& position= pose.hasWorldPose ? pose.palmPositionWorld : pose.palmPositionCamera;
		const glm::quat& orientation=
			pose.hasWorldPose ? pose.palmOrientationWorld : pose.palmOrientationCamera;
		fnvFloat(hash, position.x);
		fnvFloat(hash, position.y);
		fnvFloat(hash, position.z);
		fnvFloat(hash, orientation.x);
		fnvFloat(hash, orientation.y);
		fnvFloat(hash, orientation.z);
		fnvFloat(hash, orientation.w);
		for (int fingerIndex= 0; fingerIndex < FINGER_COUNT; ++fingerIndex)
		{
			const FingerAngles& angles= pose.fingers[fingerIndex];
			fnvFloat(hash, angles.lateral);
			fnvFloat(hash, angles.proximal);
			fnvFloat(hash, angles.intermediate);
			fnvFloat(hash, angles.distal);
		}
		fnvFloat(hash, pose.confidence);
	}
	return hash;
}

void snapshotFusedOutput(const TrackingFrameResult& fused,
						 std::array<RecordedPoseOutput, 2>& outPoses)
{
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= fused.poses[sideIndex];
		RecordedPoseOutput& out= outPoses[sideIndex];
		out= RecordedPoseOutput();
		out.tracked= pose.tracked;
		if (!pose.tracked)
			continue;
		out.side= (int)pose.side;
		out.presence= pose.presence;
		out.visibility= pose.visibility;
		out.confidence= pose.confidence;
		out.fkReprojectionPx= pose.fkReprojectionPx;
		out.stereoTriangulated= pose.stereoTriangulated;
		out.hasCameraPose= pose.hasCameraPose;
		out.palmPositionCamera= pose.palmPositionCamera;
		out.palmOrientationCamera= pose.palmOrientationCamera;
		out.hasWorldPose= pose.hasWorldPose;
		out.palmPositionWorld= pose.palmPositionWorld;
		out.palmOrientationWorld= pose.palmOrientationWorld;
		out.fingers= pose.fingers;
		out.skeleton= pose.skeleton;
	}
}

std::string makeFrameDirectoryPath(const std::string& recordingFilePath)
{
	const size_t extension= recordingFilePath.find_last_of('.');
	const std::string stem=
		extension == std::string::npos ? recordingFilePath : recordingFilePath.substr(0, extension);
	return stem + "_frames";
}

std::string makeAudioSidecarPath(const std::string& recordingFilePath)
{
	const size_t extension= recordingFilePath.find_last_of('.');
	const std::string stem=
		extension == std::string::npos ? recordingFilePath : recordingFilePath.substr(0, extension);
	return stem + ".wav";
}

std::string makeFaceSidecarPath(const std::string& recordingFilePath)
{
	const size_t extension= recordingFilePath.find_last_of('.');
	const std::string stem=
		extension == std::string::npos ? recordingFilePath : recordingFilePath.substr(0, extension);
	return stem + ".face.jsonl";
}

json headerToJson(const RecordingHeader& header)
{
	json appConfig= json::parse(header.appConfigJsonText, nullptr, false);
	if (appConfig.is_discarded())
		appConfig= json::object();
	json out= {
		{"type", "header"},
		{"formatVersion", header.formatVersion},
		{"appConfig", appConfig},
		{"cameraCount", header.cameraCount},
	};
	if (header.audio.present)
	{
		out["audio"]= {
			{"file", header.audio.fileName},
			{"sampleRate", header.audio.sampleRate},
			{"startSample", header.audio.startSampleIndex},
			{"startMs", header.audio.startTimestampMs},
		};
	}
	if (header.face.present)
		out["face"]= {{"file", header.face.fileName}, {"startMs", header.face.startTimestampMs}};
	return out;
}

bool headerFromJson(const json& j, RecordingHeader& outHeader)
{
	outHeader= RecordingHeader();
	if (j.value("type", "") != "header")
		return false;
	outHeader.formatVersion= j.value("formatVersion", 0);
	outHeader.cameraCount= j.value("cameraCount", 0);
	if (j.contains("appConfig"))
		outHeader.appConfigJsonText= j["appConfig"].dump();
	if (j.contains("audio") && j["audio"].is_object())
	{
		const json& audio= j["audio"];
		outHeader.audio.present= true;
		outHeader.audio.fileName= audio.value("file", "");
		outHeader.audio.sampleRate= audio.value("sampleRate", 0);
		outHeader.audio.startSampleIndex= audio.value("startSample", (int64_t)0);
		outHeader.audio.startTimestampMs= audio.value("startMs", 0.0);
	}
	// A header without the block (every recording before the face sidecar)
	// has no phone face
	if (j.contains("face") && j["face"].is_object())
	{
		const json& face= j["face"];
		outHeader.face.present= true;
		outHeader.face.fileName= face.value("file", "");
		outHeader.face.startTimestampMs= face.value("startMs", 0.0);
	}
	return outHeader.formatVersion == k_formatVersion && outHeader.cameraCount > 0;
}

json frameToJson(const RecordedFrame& frame)
{
	json fresh= json::array();
	for (const RecordedCameraInput& camera : frame.freshCameras)
		fresh.push_back(cameraInputToJson(camera));

	json imu= json::array();
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (frame.imu[sideIndex].hasForearmPose)
			imu.push_back({{"has", true},
						   {"quatW", quatToJson(frame.imu[sideIndex].forearmOrientationWorld)},
						   {"conf", frame.imu[sideIndex].forearmConfidence}});
		else
			imu.push_back({{"has", false}});
	}

	json out= {
		{"type", "frame"},
		{"seq", frame.seq},
		{"nowMs", frame.nowTimestampMs},
		{"fused", frame.bFused},
		{"fresh", fresh},
		{"out", json::array({poseOutputToJson(frame.outPoses[0]), poseOutputToJson(frame.outPoses[1])})},
		{"imu", imu},
		{"checksum", checksumToHex(frame.checksum)},
	};
	if (frame.mouth.present)
	{
		json weights= json::array();
		for (float weight : frame.mouth.frame.weights)
			weights.push_back(weight);
		out["mouth"]= {{"frameIndex", frame.mouth.audioFrameIndex}, {"w", weights}};
	}
	if (frame.telemetry.present)
		out["telemetry"]= telemetryToJson(frame.telemetry);
	return out;
}

bool frameFromJson(const json& j, RecordedFrame& outFrame)
{
	outFrame= RecordedFrame();
	if (j.value("type", "") != "frame")
		return false;
	outFrame.seq= j.value("seq", (int64_t)-1);
	outFrame.nowTimestampMs= j.value("nowMs", 0.0);
	outFrame.bFused= j.value("fused", false);
	for (const json& cameraJson : j.value("fresh", json::array()))
	{
		RecordedCameraInput camera;
		if (cameraInputFromJson(cameraJson, camera))
			outFrame.freshCameras.push_back(camera);
	}
	const json& out= j.value("out", json::array());
	for (int sideIndex= 0; sideIndex < 2 && sideIndex < (int)out.size(); ++sideIndex)
		poseOutputFromJson(out[sideIndex], outFrame.outPoses[sideIndex]);
	const json& imu= j.value("imu", json::array());
	for (int sideIndex= 0; sideIndex < 2 && sideIndex < (int)imu.size(); ++sideIndex)
	{
		outFrame.imu[sideIndex].hasForearmPose= imu[sideIndex].value("has", false);
		if (outFrame.imu[sideIndex].hasForearmPose)
		{
			outFrame.imu[sideIndex].forearmOrientationWorld=
				quatFromJson(imu[sideIndex].value("quatW", json::array()));
			outFrame.imu[sideIndex].forearmConfidence= imu[sideIndex].value("conf", 0.f);
		}
	}
	// A mouth block is the 15 viseme weights; a block of another width is
	// from before the frame carried visemes and is skipped, since its values
	// mean something else
	if (j.contains("mouth") && j["mouth"].is_object())
	{
		const json& mouth= j["mouth"];
		const json& weights= mouth.value("w", json::array());
		if (weights.is_array() && (int)weights.size() == VISEME_COUNT)
		{
			outFrame.mouth.present= true;
			outFrame.mouth.audioFrameIndex= mouth.value("frameIndex", (int64_t)-1);
			for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
				outFrame.mouth.frame.weights[viseme]= floatFromJson(weights[viseme]);
		}
	}
	// A frame without the block (every recording before it was carried) has
	// no stage state to show
	if (j.contains("telemetry") && j["telemetry"].is_object())
		telemetryFromJson(j["telemetry"], outFrame.telemetry);
	outFrame.checksum= checksumFromHex(j.value("checksum", "0"));
	return true;
}

json footerToJson(const RecordingFooter& footer)
{
	return {
		{"type", "end"},
		{"frameCount", footer.frameCount},
		{"aborted", footer.bAborted},
		{"reason", footer.reason},
	};
}

bool footerFromJson(const json& j, RecordingFooter& outFooter)
{
	outFooter= RecordingFooter();
	if (j.value("type", "") != "end")
		return false;
	outFooter.frameCount= j.value("frameCount", (int64_t)0);
	outFooter.bAborted= j.value("aborted", false);
	outFooter.reason= j.value("reason", "");
	return true;
}
} // namespace TrackingRecording
