#include "AvatarJson.h"

#include "AvatarRetarget.h"
#include "AvatarSkeleton.h"
#include "AvatarTypes.h"

using json= nlohmann::json;

namespace
{
json vec3ToJson(const glm::vec3& v)
{
	return json::array({v.x, v.y, v.z});
}

json quatToJson(const glm::quat& q)
{
	return json::array({q.x, q.y, q.z, q.w});
}

json fingersToJson(const std::array<FingerAngles, FINGER_COUNT>& fingers)
{
	json out= json::array();
	for (const FingerAngles& angles : fingers)
		out.push_back(json::array({angles.lateral, angles.proximal, angles.intermediate, angles.distal}));
	return out;
}

json chainToJson(const DirectRetarget::ArmChain& chain)
{
	json out= {{"valid", chain.valid}};
	if (chain.valid)
	{
		out["shoulder"]= vec3ToJson(chain.shoulder);
		out["elbow"]= vec3ToJson(chain.elbow);
		out["wrist"]= vec3ToJson(chain.wrist);
	}
	return out;
}

json armToJson(const DirectRetarget::SolveState::Arm& arm)
{
	return {
		{"chain", chainToJson(arm.chain)},
		{"handSeen", arm.bHandSeen},
		{"elbowSeen", arm.bElbowSeen},
		{"depthChosen", arm.bDepthChosen},
		{"depthLow", arm.depthLow},
		{"depthHigh", arm.depthHigh},
		{"modelDepthDelta", arm.modelDepthDelta},
		{"pnpDistance", arm.pnpDistance},
		{"liftDepth", arm.liftDepth},
		{"chosenDepth", arm.chosenDepth},
		{"chosenCost", arm.chosenCost},
		{"chosenElbowPx", arm.chosenElbowPx},
		{"filteredDepth", arm.filteredDepth},
		{"pnpScale", arm.pnpScale.scale},
		{"pnpScaleSamples", arm.pnpScale.sampleCount},
		{"handMeasured", arm.bHandMeasured},
		{"twist", arm.twist},
		{"wristRotation", quatToJson(arm.wristRotation)},
		{"filteredPalm", quatToJson(arm.filteredPalm)},
		{"palmMirrored", arm.bPalmMirrored},
		{"palmMirrorGain", arm.palmMirrorGain},
		{"measuredFingers", fingersToJson(arm.measuredFingers)},
		{"fingers", fingersToJson(arm.fingers)},
	};
}
} // namespace

namespace AvatarJson
{
json avatarPoseToJson(const AvatarSkeleton& skeleton, const AvatarPose& pose)
{
	json out= {
		{"valid", pose.valid},
		{"timestampMs", pose.timestampMs},
		{"rootPositionWorld", vec3ToJson(pose.rootPositionWorld)},
		{"rootRotationWorld", quatToJson(pose.rootRotationWorld)},
	};
	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
	computePosedBones(skeleton, pose, posed);
	json bones= json::object();
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		if (!pose.bones[index].present || !skeleton.getBone((eHumanoidBone)index).present)
			continue;
		bones[humanoidBoneName((eHumanoidBone)index)]= {
			{"deltaWorld", quatToJson(pose.bones[index].deltaWorld)},
			{"positionWorld", vec3ToJson(posed[index].positionWorld)},
			{"rotationWorld", quatToJson(posed[index].rotationWorld)},
		};
	}
	out["bones"]= bones;
	return out;
}

json directSolveStateToJson(const DirectRetarget::SolveState& state)
{
	json out= {
		{"camera",
		 {{"fx", state.camera.fx},
		  {"fy", state.camera.fy},
		  {"cx", state.camera.cx},
		  {"cy", state.camera.cy},
		  {"width", state.camera.width},
		  {"height", state.camera.height}}},
		{"tiltRadians", state.tiltRadians},
		{"torsoValid", state.torsoValid},
		{"spineDelta", quatToJson(state.spineDelta)},
		{"headMeasured", state.headMeasured},
		{"headDelta", quatToJson(state.headDelta)},
		{"handednessSwapped", state.bHandednessSwapped},
		{"oppositeWristRejections",
		 json::array({state.oppositeWristRejections[0], state.oppositeWristRejections[1]})},
		{"sideCorrections", json::array({state.sideCorrections[0], state.sideCorrections[1]})},
		{"arms", json::array({armToJson(state.arms[0]), armToJson(state.arms[1])})},
	};
	if (state.torsoValid)
	{
		out["shoulders"]= json::array({vec3ToJson(state.shoulder[0]), vec3ToJson(state.shoulder[1])});
		out["shoulderMidpoint"]= vec3ToJson(state.shoulderMidpoint);
		out["torsoFrame"]= quatToJson(state.torsoFrame);
	}
	if (state.headMeasured)
		out["headFrame"]= quatToJson(state.headFrame);
	return out;
}
} // namespace AvatarJson
