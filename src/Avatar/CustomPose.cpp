#include "CustomPose.h"

#include <cstdio>
#include <random>

#include "glm/gtc/quaternion.hpp"
#include "glm/trigonometric.hpp"
#include "nlohmann/json.hpp"

#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "DirectRetarget.h"

using json= nlohmann::json;

namespace
{
using B= eHumanoidBone;

const char* const k_sideKeys[2]= {"left", "right"};
const char* const k_fingerKeys[FINGER_COUNT]= {"thumb", "index", "middle", "ring", "little"};

json vec3ToJson(const glm::vec3& v)
{
	return json::array({v.x, v.y, v.z});
}

glm::vec3 vec3FromJson(const json& j, const glm::vec3& fallback)
{
	if (!j.is_array() || j.size() != 3 || !j[0].is_number() || !j[1].is_number() || !j[2].is_number())
		return fallback;
	return glm::vec3(j[0].get<float>(), j[1].get<float>(), j[2].get<float>());
}

bool isZero(const FingerAngles& angles)
{
	return angles.lateral == 0.f && angles.proximal == 0.f && angles.intermediate == 0.f && angles.distal == 0.f;
}

glm::quat quatFromDegrees(const glm::vec3& degrees)
{
	return glm::quat(glm::radians(degrees));
}

// Whichever of a side's two bones the side index names
B sideBone(int sideIndex, B left, B right)
{
	return sideIndex == 0 ? left : right;
}
} // namespace

const char* const k_demoCustomPoseId= "demo";

bool isBuiltInCustomPoseId(const std::string& id)
{
	return id == k_demoCustomPoseId;
}

std::string makeCustomPoseId()
{
	std::random_device device;
	std::mt19937_64 generator(((uint64_t)device() << 32) ^ device());
	std::uniform_int_distribution<uint32_t> byteDistribution(0, 255);
	unsigned char bytes[16];
	for (unsigned char& byte : bytes)
		byte= (unsigned char)byteDistribution(generator);
	// Version 4, variant 10
	bytes[6]= (unsigned char)((bytes[6] & 0x0F) | 0x40);
	bytes[8]= (unsigned char)((bytes[8] & 0x3F) | 0x80);

	char text[37];
	snprintf(text, sizeof(text), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", bytes[0],
			 bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9], bytes[10],
			 bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]);
	return text;
}

CustomPose makeDefaultCustomPose()
{
	// Each wrist ahead of its shoulder, a little inward and below it, the
	// fingers forward and the palm rolled a quarter turn off flat
	CustomPose pose;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const float sign= sideIndex == 0 ? 1.f : -1.f;
		CustomPose::Arm& arm= pose.arms[sideIndex];
		arm.wristOffset= glm::vec3(0.6f, -sign * 0.17f, -0.26f);
		arm.palmDegrees= glm::vec3(-sign * 90.f, 0.f, 0.f);
		arm.elbowOffset= glm::vec3(0.2f, sign * 0.1f, -0.4f);
	}
	return pose;
}

CustomPose makeDemoCustomPose()
{
	CustomPose pose= makeDefaultCustomPose();
	pose.id= k_demoCustomPoseId;
	pose.name= "Demo";
	pose.headDegrees= glm::vec3(0.f, 10.f, 25.f);

	// The left hand spread
	std::array<FingerAngles, FINGER_COUNT>& left= pose.fingers[0];
	left[(int)eFinger::Index].lateral= 0.25f;
	left[(int)eFinger::Pinky].lateral= -0.3f;
	left[(int)eFinger::Thumb].lateral= 0.6f;
	// The right hand curled
	std::array<FingerAngles, FINGER_COUNT>& right= pose.fingers[1];
	for (int finger= 1; finger < FINGER_COUNT; ++finger)
		right[finger]= {0.f, 1.3f, 1.2f, 0.7f};
	right[(int)eFinger::Thumb]= {0.3f, 0.6f, 0.8f, 0.4f};
	return pose;
}

CustomPose::Arm mirrorCustomPoseArm(const CustomPose::Arm& arm)
{
	CustomPose::Arm mirrored= arm;
	mirrored.wristOffset.y= -arm.wristOffset.y;
	mirrored.elbowOffset.y= -arm.elbowOffset.y;
	// A rotation reflected across the midplane turns the other way about the
	// two axes that lie in it. The other hand's palm frame is that
	// reflection, its +Y being whatever its +Z and +X make it.
	const glm::quat palm= quatFromDegrees(arm.palmDegrees);
	mirrored.palmDegrees= glm::degrees(glm::eulerAngles(glm::quat(palm.w, -palm.x, palm.y, -palm.z)));
	return mirrored;
}

std::array<FingerAngles, FINGER_COUNT> mirrorCustomPoseFingers(const std::array<FingerAngles, FINGER_COUNT>& fingers)
{
	// The splay is about the palm's +Z, whose side of the hand changes with
	// the hand, so the same spread has the other sign
	std::array<FingerAngles, FINGER_COUNT> mirrored= fingers;
	for (FingerAngles& angles : mirrored)
		angles.lateral= -angles.lateral;
	return mirrored;
}

std::string customPoseToJson(const CustomPose& pose)
{
	json j;
	j["version"]= CustomPose::k_version;
	j["id"]= pose.id;
	j["name"]= pose.name;
	j["headDegrees"]= vec3ToJson(pose.headDegrees);
	j["leanDegrees"]= vec3ToJson(pose.leanDegrees);

	json arms= json::object();
	json hands= json::object();
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const CustomPose::Arm& arm= pose.arms[sideIndex];
		json ja= {{"enabled", arm.bEnabled},
				  {"wristOffset", vec3ToJson(arm.wristOffset)},
				  {"palmDegrees", vec3ToJson(arm.palmDegrees)}};
		if (arm.bHasElbow)
			ja["elbowOffset"]= vec3ToJson(arm.elbowOffset);
		arms[k_sideKeys[sideIndex]]= ja;

		json fingers= json::object();
		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			const FingerAngles& angles= pose.fingers[sideIndex][finger];
			if (isZero(angles))
				continue;
			fingers[k_fingerKeys[finger]]= json::array({glm::degrees(angles.lateral), glm::degrees(angles.proximal),
														glm::degrees(angles.intermediate), glm::degrees(angles.distal)});
		}
		hands[k_sideKeys[sideIndex]]= {{"fingerDegrees", fingers}};
	}
	j["arms"]= arms;
	j["hands"]= hands;

	json face= json::object();
	for (const auto& [name, value] : pose.face)
	{
		if (value != 0.f)
			face[name]= value;
	}
	j["face"]= face;
	j["lostFaceSources"]= pose.lostFaceSources;
	return j.dump(2);
}

bool customPoseFromJson(const std::string& text, CustomPose& outPose, std::string& outError)
{
	CustomPose pose= makeDefaultCustomPose();
	try
	{
		const json j= json::parse(text);
		if (!j.is_object())
		{
			outError= "not a JSON object";
			return false;
		}
		pose.id= j.value("id", "");
		pose.name= j.value("name", "");
		if (pose.id.empty())
		{
			outError= "the pose has no id";
			return false;
		}
		pose.headDegrees= vec3FromJson(j.value("headDegrees", json()), pose.headDegrees);
		pose.leanDegrees= vec3FromJson(j.value("leanDegrees", json()), pose.leanDegrees);

		const json arms= j.value("arms", json::object());
		const json hands= j.value("hands", json::object());
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			CustomPose::Arm& arm= pose.arms[sideIndex];
			const json ja= arms.is_object() ? arms.value(k_sideKeys[sideIndex], json::object()) : json::object();
			if (ja.is_object())
			{
				arm.bEnabled= ja.value("enabled", arm.bEnabled);
				arm.wristOffset= vec3FromJson(ja.value("wristOffset", json()), arm.wristOffset);
				arm.palmDegrees= vec3FromJson(ja.value("palmDegrees", json()), arm.palmDegrees);
				arm.bHasElbow= ja.contains("elbowOffset");
				arm.elbowOffset= vec3FromJson(ja.value("elbowOffset", json()), arm.elbowOffset);
			}

			const json jh= hands.is_object() ? hands.value(k_sideKeys[sideIndex], json::object()) : json::object();
			const json fingers= jh.is_object() ? jh.value("fingerDegrees", json::object()) : json::object();
			for (int finger= 0; fingers.is_object() && finger < FINGER_COUNT; ++finger)
			{
				const json jf= fingers.value(k_fingerKeys[finger], json());
				if (!jf.is_array() || jf.size() != 4)
					continue;
				pose.fingers[sideIndex][finger]= {
					glm::radians(jf[0].get<float>()), glm::radians(jf[1].get<float>()), glm::radians(jf[2].get<float>()),
					glm::radians(jf[3].get<float>())};
			}
		}

		const json face= j.value("face", json::object());
		if (face.is_object())
		{
			for (auto it= face.begin(); it != face.end(); ++it)
			{
				if (it.value().is_number())
					pose.face[it.key()]= it.value().get<float>();
			}
		}

		const json lostFaceSources= j.value("lostFaceSources", json::array());
		if (lostFaceSources.is_array())
		{
			for (const json& source : lostFaceSources)
			{
				if (source.is_string())
					pose.lostFaceSources.insert(source.get<std::string>());
			}
		}
	}
	catch (const json::exception& exception)
	{
		outError= exception.what();
		return false;
	}
	outPose= pose;
	return true;
}

void fillCustomSource(const CustomPose& pose, const CustomPoseSolveInputs& inputs, const SourcePose::Clock& clock,
					  RetargetResult& outResult, std::array<glm::quat, 2>* outDirectPalms)
{
	outResult.source= SourcePose();
	outResult.hands= LossHands();
	outResult.bDirectState= false;
	if (inputs.skeleton == nullptr)
		return;
	const AvatarSkeleton& skeleton= *inputs.skeleton;
	SourcePose& source= outResult.source;
	source.pipeline= inputs.bDirect ? SourcePose::ePipeline::Direct : SourcePose::ePipeline::Position;
	source.clock= clock;
	source.upperArmLength= skeleton.getUpperArmLength(eHandSide::Left);
	source.forearmLength= skeleton.getForearmLength(eHandSide::Left);

	const SourcePose::Confidence sure{true, 1.f};
	const SourcePose::Confidence unsure{true, 0.f};
	const glm::quat head= quatFromDegrees(pose.headDegrees);

	if (inputs.bDirect)
	{
		// The solve's world is the avatar's with the root at the identity.
		// The lean turns the spine, and the shoulders, the offsets and the
		// palms ride it.
		const glm::quat spineDelta= quatFromDegrees(pose.leanDegrees);
		source.torso.state= SourcePose::eGroupState::Measured;
		source.torso.spineDelta= spineDelta;
		source.head.state= SourcePose::eGroupState::Measured;
		source.head.confidence= sure;
		source.head.orientation= head;

		AvatarPose leaned;
		leaned.valid= true;
		for (const B bone : {B::Spine, B::Chest, B::UpperChest, B::Neck})
		{
			if (!skeleton.getBone(bone).present)
				continue;
			leaned.bones[(int)bone].present= true;
			leaned.bones[(int)bone].deltaWorld= spineDelta;
		}
		std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
		computePosedBones(skeleton, leaned, posed);

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const CustomPose::Arm& arm= pose.arms[sideIndex];
			if (outDirectPalms != nullptr)
				(*outDirectPalms)[sideIndex]= glm::quat(1.f, 0.f, 0.f, 0.f);
			if (!arm.bEnabled)
				continue;
			const eHandSide side= (eHandSide)sideIndex;
			const float armLength= skeleton.getUpperArmLength(side) + skeleton.getForearmLength(side);
			const glm::vec3 shoulder= posed[(int)sideBone(sideIndex, B::LeftUpperArm, B::RightUpperArm)].positionWorld;
			const glm::vec3 wristTarget= shoulder + spineDelta * (arm.wristOffset * armLength);
			const glm::vec3 givenElbow= shoulder + spineDelta * (arm.elbowOffset * armLength);
			const glm::vec3 elbowHint= inputs.rig != nullptr
				? shoulder + spineDelta * inputs.rig->sides[sideIndex].elbowHintOffset
				: shoulder + spineDelta * glm::vec3(0.f, 0.f, -1.f);
			const glm::quat palm= spineDelta * quatFromDegrees(arm.palmDegrees);

			glm::vec3 elbow, wrist;
			const DirectRetarget::ArmPlacement placement= DirectRetarget::makePosedPlacement(
				skeleton, sideIndex, spineDelta, shoulder, wristTarget, arm.bHasElbow ? &givenElbow : nullptr, elbowHint,
				palm, pose.fingers[sideIndex], elbow, wrist);

			SourcePose::Arm& sourceArm= source.arms[sideIndex];
			sourceArm.state= SourcePose::eGroupState::Measured;
			sourceArm.bHasShoulder= true;
			sourceArm.shoulderPosition= shoulder;
			sourceArm.shoulderConfidence= sure;
			sourceArm.bHasElbow= true;
			sourceArm.elbowPosition= elbow;
			sourceArm.wristPosition= wrist;
			// An elbow the pose gives counts as measured, one the hint placed
			// does not, which is what the elbow weight reads
			sourceArm.forearmConfidence= arm.bHasElbow ? sure : unsure;
			sourceArm.upperDirection= placement.upperDirection;
			sourceArm.forearmDirection= placement.forearmDirection;
			sourceArm.twist= placement.twist;

			SourcePose::Hand& sourceHand= source.hands[sideIndex];
			sourceHand.state= SourcePose::eGroupState::Measured;
			sourceHand.confidence= sure;
			sourceHand.wristRotation= placement.wristRotation;
			sourceHand.fingerZero= SourcePose::eFingerZero::Metacarpal;
			sourceHand.fingers= placement.fingers;
			if (outDirectPalms != nullptr)
				(*outDirectPalms)[sideIndex]= palm;
		}
		return;
	}

	// The position pipeline measures in the world, where the fixed root
	// stands the avatar. With no shoulders in the source the solve keeps to
	// that root, and the avatar's own shoulders are where the offsets start.
	const glm::quat root= glm::angleAxis(glm::radians(inputs.rootYawDegrees), glm::vec3(0.f, 0.f, 1.f));
	source.head.state= SourcePose::eGroupState::Measured;
	source.head.confidence= sure;
	source.head.orientation= root * head;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const CustomPose::Arm& arm= pose.arms[sideIndex];
		if (!arm.bEnabled)
			continue;
		const eHandSide side= (eHandSide)sideIndex;
		const float armLength= skeleton.getUpperArmLength(side) + skeleton.getForearmLength(side);
		const glm::vec3 shoulder= inputs.rootPositionWorld +
			root * skeleton.getBone(sideBone(sideIndex, B::LeftUpperArm, B::RightUpperArm)).restPositionWorld;
		const glm::vec3 wrist= shoulder + root * (arm.wristOffset * armLength);
		const glm::quat palm= root * quatFromDegrees(arm.palmDegrees);

		SourcePose::Arm& sourceArm= source.arms[sideIndex];
		sourceArm.state= SourcePose::eGroupState::Measured;
		sourceArm.wristPosition= wrist;
		sourceArm.bHasElbow= arm.bHasElbow;
		sourceArm.elbowPosition= shoulder + root * (arm.elbowOffset * armLength);
		sourceArm.forearmConfidence= arm.bHasElbow ? sure : unsure;

		SourcePose::Hand& sourceHand= source.hands[sideIndex];
		sourceHand.state= SourcePose::eGroupState::Measured;
		sourceHand.confidence= sure;
		sourceHand.palmOrientation= palm;
		sourceHand.fingerZero= SourcePose::eFingerZero::CapturedRest;
		sourceHand.fingers= pose.fingers[sideIndex];

		// The hand as a tracked one, which the source pose layer draws
		const AvatarSkeleton::HandRest& handRest= skeleton.getHand(side);
		HandPose& handPose= outResult.hands.poses[sideIndex];
		handPose.tracked= true;
		handPose.side= side;
		handPose.presence= 1.f;
		handPose.confidence= 1.f;
		handPose.skeleton= handRest.skeleton;
		handPose.hasWorldPose= true;
		handPose.palmOrientationWorld= palm;
		handPose.palmPositionWorld=
			wrist + palm * glm::vec3(handRest.skeleton.baseInPalm[(int)eFinger::Middle].x, 0.f, 0.f);
		handPose.fingers= pose.fingers[sideIndex];
		outResult.hands.bCarried[sideIndex]= true;
	}
}
