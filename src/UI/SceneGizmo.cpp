#include "SceneGizmo.h"

#include <cmath>

#include "glm/geometric.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/trigonometric.hpp"

#include "AvatarSkeleton.h"
#include "CustomPose.h"

namespace
{
using B= eHumanoidBone;

const char* const k_valueIds[(int)eSceneGizmoValue::Count]= {
	"none", "elbowHint", "goalOffset", "restElbow", "restWrist", "trim", "poseWrist", "poseElbow", "posePalm", "poseHead",
};

bool isArmChain(eRetargetChain chain)
{
	return chain == eRetargetChain::LeftArm || chain == eRetargetChain::RightArm;
}

bool isHandChain(eRetargetChain chain)
{
	return chain == eRetargetChain::LeftHand || chain == eRetargetChain::RightHand;
}

// 0 for a left chain, 1 for a right one
int getSideIndex(eRetargetChain chain)
{
	return chain == eRetargetChain::RightArm || chain == eRetargetChain::RightHand ? 1 : 0;
}

// The bone whose trim a chain's Trim value is
bool getTrimBone(eRetargetChain chain, B& outBone)
{
	switch (chain)
	{
		case eRetargetChain::Head:
			outBone= B::Head;
			return true;
		case eRetargetChain::LeftArm:
			outBone= B::LeftLowerArm;
			return true;
		case eRetargetChain::RightArm:
			outBone= B::RightLowerArm;
			return true;
		case eRetargetChain::LeftHand:
			outBone= B::LeftHand;
			return true;
		case eRetargetChain::RightHand:
			outBone= B::RightHand;
			return true;
		default:
			return false;
	}
}

// A translated value's storage and the flag its edit sets; null for a target
// that is not one, or whose destination the edit lacks
glm::vec3* findOffset(const SceneGizmoTarget& target, SceneGizmoEdit& edit, bool*& outChanged)
{
	if (!isArmChain(target.chain))
		return nullptr;
	const int sideIndex= getSideIndex(target.chain);
	switch (target.value)
	{
		case eSceneGizmoValue::ElbowHint:
			outChanged= &edit.bRigChanged;
			return edit.rig != nullptr ? &edit.rig->sides[sideIndex].elbowHintOffset : nullptr;
		case eSceneGizmoValue::GoalOffset:
			outChanged= &edit.bChainsChanged;
			return edit.chains != nullptr ? &edit.chains->arms[sideIndex].goalOffset : nullptr;
		case eSceneGizmoValue::RestElbow:
			outChanged= &edit.bRigChanged;
			return edit.rig != nullptr ? &edit.rig->sides[sideIndex].restElbowOffset : nullptr;
		case eSceneGizmoValue::RestWrist:
			outChanged= &edit.bRigChanged;
			return edit.rig != nullptr ? &edit.rig->sides[sideIndex].restWristOffset : nullptr;
		case eSceneGizmoValue::PoseWrist:
			outChanged= &edit.bPoseChanged;
			return edit.pose != nullptr ? &edit.pose->arms[sideIndex].wristOffset : nullptr;
		case eSceneGizmoValue::PoseElbow:
			outChanged= &edit.bPoseChanged;
			return edit.pose != nullptr ? &edit.pose->arms[sideIndex].elbowOffset : nullptr;
		default:
			return nullptr;
	}
}

// A frame the custom pose stores as Euler degrees; null for a target that
// is not one, or a pose that is not there
glm::vec3* findPoseFrame(const SceneGizmoTarget& target, CustomPose* pose)
{
	if (pose == nullptr)
		return nullptr;
	if (target.value == eSceneGizmoValue::PoseHead && target.chain == eRetargetChain::Head)
		return &pose->headDegrees;
	if (target.value == eSceneGizmoValue::PosePalm && isArmChain(target.chain))
		return &pose->arms[getSideIndex(target.chain)].palmDegrees;
	return nullptr;
}

// Degrees into (-180, 180], the range the trim fields show
float wrapTrimDegrees(float degrees)
{
	const float wrapped= std::fmod(degrees + 180.f, 360.f);
	return (wrapped <= 0.f ? wrapped + 360.f : wrapped) - 180.f;
}
} // namespace

const char* getSceneGizmoValueId(eSceneGizmoValue value)
{
	return value < eSceneGizmoValue::Count ? k_valueIds[(int)value] : k_valueIds[0];
}

bool parseSceneGizmoValueId(const std::string& id, eSceneGizmoValue& outValue)
{
	for (int index= 1; index < (int)eSceneGizmoValue::Count; ++index)
	{
		if (id == k_valueIds[index])
		{
			outValue= (eSceneGizmoValue)index;
			return true;
		}
	}
	return false;
}

const char* getSceneGizmoValueLabelKey(const SceneGizmoTarget& target)
{
	switch (target.value)
	{
		case eSceneGizmoValue::ElbowHint:
			return "avatarPanel.elbowHintDrag";
		case eSceneGizmoValue::GoalOffset:
			return "chains.goalOffsetDrag";
		case eSceneGizmoValue::RestElbow:
			return "avatarPanel.restElbowDrag";
		case eSceneGizmoValue::RestWrist:
			return "avatarPanel.restWristDrag";
		case eSceneGizmoValue::Trim:
			if (target.chain == eRetargetChain::Head)
				return "chains.headTrim";
			return isHandChain(target.chain) ? "chains.handTrim" : "chains.rollTrim";
		case eSceneGizmoValue::PoseWrist:
			return "chains.poseWristDrag";
		case eSceneGizmoValue::PoseElbow:
			return "chains.poseElbowDrag";
		case eSceneGizmoValue::PosePalm:
			return "chains.posePalmDrag";
		case eSceneGizmoValue::PoseHead:
			return "chains.poseHeadDrag";
		default:
			return "";
	}
}

void buildSceneGizmoHandles(const SceneGizmoInputs& inputs, eRetargetChain chain,
							std::vector<SceneGizmoHandle>& outHandles)
{
	outHandles.clear();
	if (inputs.skeleton == nullptr || inputs.pose == nullptr || inputs.rig == nullptr || inputs.chains == nullptr)
		return;
	const AvatarSkeleton& skeleton= *inputs.skeleton;
	const AvatarRigSettings& rig= *inputs.rig;
	const glm::quat rootRotation= inputs.pose->rootRotationWorld;
	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
	computePosedBones(skeleton, *inputs.pose, posed);

	// A frame given in the rest skeleton's world, carried along by a bone as
	// the pose turned it
	auto carryFrame= [&](B bone, const glm::quat& restFrameWorld) {
		return posed[(int)bone].rotationWorld * glm::inverse(skeleton.getBone(bone).restRotationWorld) * restFrameWorld;
	};
	auto makeTrimQuat= [&](B bone) { return glm::quat(glm::radians(rig.rotationTrimDegrees[(int)bone])); };
	const bool bTracking= inputs.mode == eChainMode::Tracking;
	const bool bRest= inputs.mode == eChainMode::RestPose;
	if (!bTracking && !bRest)
	{
		// The custom pose's own values, where the pose put them
		const CustomPose* custom= inputs.customPose;
		if (custom == nullptr)
			return;
		if (chain == eRetargetChain::Head)
		{
			if (!skeleton.getBone(B::Head).present)
				return;
			SceneGizmoHandle handle;
			handle.target= {chain, eSceneGizmoValue::PoseHead};
			handle.kind= eSceneGizmoKind::Rotate;
			handle.trimRule= eSceneGizmoTrimRule::StoredFrame;
			handle.baseFrameWorld= rootRotation;
			handle.positionWorld= posed[(int)B::Head].positionWorld;
			handle.frameWorld= glm::normalize(rootRotation * glm::quat(glm::radians(custom->headDegrees)));
			outHandles.push_back(handle);
			return;
		}
		if (!isArmChain(chain))
			return;
		const int sideIndex= getSideIndex(chain);
		const eHandSide side= sideIndex == 0 ? eHandSide::Left : eHandSide::Right;
		const B upperArmBone= sideIndex == 0 ? B::LeftUpperArm : B::RightUpperArm;
		const B handBone= sideIndex == 0 ? B::LeftHand : B::RightHand;
		const CustomPose::Arm& arm= custom->arms[sideIndex];
		if (!arm.bEnabled || !skeleton.getBone(upperArmBone).present)
			return;
		// The frame the pose's offsets and palm are in: the rooted torso,
		// which the direct pipeline leans
		const glm::quat torsoFrame= glm::normalize(
			inputs.bDirect ? rootRotation * glm::quat(glm::radians(custom->leanDegrees)) : rootRotation);
		const float armLength= skeleton.getUpperArmLength(side) + skeleton.getForearmLength(side);
		const glm::vec3 shoulder= posed[(int)upperArmBone].positionWorld;
		auto addPoint= [&](eSceneGizmoValue value, const glm::vec3& offset, bool bLine) {
			SceneGizmoHandle handle;
			handle.target= {chain, value};
			handle.kind= eSceneGizmoKind::Translate;
			handle.anchorWorld= shoulder;
			handle.offsetScale= armLength;
			handle.positionWorld= shoulder + torsoFrame * (offset * armLength);
			handle.frameWorld= torsoFrame;
			handle.bDrawAnchorLine= bLine;
			outHandles.push_back(handle);
		};
		addPoint(eSceneGizmoValue::PoseWrist, arm.wristOffset, false);
		if (arm.bHasElbow)
			addPoint(eSceneGizmoValue::PoseElbow, arm.elbowOffset, true);
		if (skeleton.getBone(handBone).present)
		{
			SceneGizmoHandle handle;
			handle.target= {chain, eSceneGizmoValue::PosePalm};
			handle.kind= eSceneGizmoKind::Rotate;
			handle.trimRule= eSceneGizmoTrimRule::StoredFrame;
			handle.baseFrameWorld= torsoFrame;
			handle.frameWorld= glm::normalize(torsoFrame * glm::quat(glm::radians(arm.palmDegrees)));
			const float halfPalm= skeleton.getHand(side).skeleton.baseInPalm[(int)eFinger::Middle].x;
			handle.positionWorld= posed[(int)handBone].positionWorld + handle.frameWorld * glm::vec3(halfPalm, 0.f, 0.f);
			outHandles.push_back(handle);
		}
		return;
	}

	if (isArmChain(chain))
	{
		const int sideIndex= getSideIndex(chain);
		const B upperArmBone= sideIndex == 0 ? B::LeftUpperArm : B::RightUpperArm;
		const B lowerArmBone= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
		const B handBone= sideIndex == 0 ? B::LeftHand : B::RightHand;
		if (!skeleton.getBone(upperArmBone).present)
			return;
		const AvatarRigSettings::Side& side= rig.sides[sideIndex];
		const RetargetChains::Arm& arm= inputs.chains->arms[sideIndex];
		const glm::vec3 shoulder= posed[(int)upperArmBone].positionWorld;

		auto addOffset= [&](eSceneGizmoValue value, const glm::vec3& anchor, const glm::vec3& offset, bool bLine) {
			SceneGizmoHandle handle;
			handle.target= {chain, value};
			handle.kind= eSceneGizmoKind::Translate;
			handle.anchorWorld= anchor;
			handle.positionWorld= anchor + rootRotation * offset;
			handle.frameWorld= rootRotation;
			handle.bDrawAnchorLine= bLine;
			outHandles.push_back(handle);
		};

		if (bRest)
		{
			addOffset(eSceneGizmoValue::RestElbow, shoulder, side.restElbowOffset, true);
			addOffset(eSceneGizmoValue::RestWrist, shoulder, side.restWristOffset, true);
			return;
		}

		addOffset(eSceneGizmoValue::ElbowHint, shoulder, side.elbowHintOffset, true);
		if (arm.mapping == eArmMapping::BoneAnglesThenIk && inputs.goals != nullptr &&
			inputs.goals->bHasWristGoal[sideIndex])
		{
			// The goal as shown already carries the offset
			const glm::vec3 goal= inputs.goals->wristGoalWorld[sideIndex];
			addOffset(eSceneGizmoValue::GoalOffset, goal - rootRotation * arm.goalOffset, arm.goalOffset, false);
		}
		if (skeleton.getBone(lowerArmBone).present && skeleton.getBone(handBone).present)
		{
			const glm::vec3 elbow= posed[(int)lowerArmBone].positionWorld;
			const glm::vec3 wrist= posed[(int)handBone].positionWorld;
			const glm::vec3 delta= wrist - elbow;
			const float length= glm::length(delta);
			if (length > 1e-5f)
			{
				// X along the forearm, the other two axes riding the bone so the
				// ring is seen to turn with the roll
				const glm::vec3 axisX= delta / length;
				const glm::quat carried= carryFrame(lowerArmBone, glm::quat(1.f, 0.f, 0.f, 0.f));
				glm::vec3 reference= carried * glm::vec3(0.f, 0.f, 1.f);
				if (std::fabs(glm::dot(reference, axisX)) > 0.95f)
					reference= carried * glm::vec3(0.f, 1.f, 0.f);
				const glm::vec3 axisY= glm::normalize(glm::cross(reference, axisX));
				const glm::vec3 axisZ= glm::cross(axisX, axisY);
				SceneGizmoHandle handle;
				handle.target= {chain, eSceneGizmoValue::Trim};
				handle.kind= eSceneGizmoKind::RotateX;
				handle.trimRule= eSceneGizmoTrimRule::ForearmRoll;
				handle.positionWorld= elbow + delta * 0.5f;
				handle.frameWorld= glm::normalize(glm::quat_cast(glm::mat3(axisX, axisY, axisZ)));
				outHandles.push_back(handle);
			}
		}
		return;
	}

	// The head's and a hand's trims are how they follow tracking
	B trimBone;
	if (!bTracking || !getTrimBone(chain, trimBone) || !skeleton.getBone(trimBone).present)
		return;
	SceneGizmoHandle handle;
	handle.target= {chain, eSceneGizmoValue::Trim};
	handle.kind= eSceneGizmoKind::Rotate;
	handle.positionWorld= posed[(int)trimBone].positionWorld;
	if (chain == eRetargetChain::Head)
	{
		// The head's assumed rest frame is the rest skeleton's own axes
		handle.trimRule= eSceneGizmoTrimRule::BoneFollows;
		handle.frameWorld= carryFrame(B::Head, glm::quat(1.f, 0.f, 0.f, 0.f));
	}
	else
	{
		const int sideIndex= getSideIndex(chain);
		const AvatarSkeleton::HandRest& handRest= skeleton.getHand(sideIndex == 0 ? eHandSide::Left : eHandSide::Right);
		const glm::quat restPalm= glm::quat_cast(glm::mat3(handRest.palmFrameWorld));
		if (inputs.bDirect)
		{
			// The direct placement turns the wrist about the trimmed rest
			// palm's axes as the forearm carries them
			const B lowerArmBone= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
			handle.trimRule= eSceneGizmoTrimRule::AxesTurn;
			handle.frameWorld= carryFrame(lowerArmBone, restPalm) * makeTrimQuat(trimBone);
		}
		else
		{
			handle.trimRule= eSceneGizmoTrimRule::BoneFollows;
			handle.frameWorld= carryFrame(trimBone, restPalm);
		}
	}
	handle.frameWorld= glm::normalize(handle.frameWorld);
	outHandles.push_back(handle);
}

void applySceneGizmoEdit(const SceneGizmoHandle& handle, const glm::vec3& draggedPositionWorld,
						 const glm::quat& draggedFrameWorld, SceneGizmoEdit& ioEdit)
{
	if (handle.kind == eSceneGizmoKind::Translate)
	{
		bool* changed= nullptr;
		glm::vec3* offset= findOffset(handle.target, ioEdit, changed);
		if (offset == nullptr || handle.offsetScale <= 0.f)
			return;
		*offset= glm::inverse(handle.frameWorld) * (draggedPositionWorld - handle.anchorWorld) / handle.offsetScale;
		*changed= true;
		return;
	}

	if (handle.trimRule == eSceneGizmoTrimRule::StoredFrame)
	{
		glm::vec3* degrees= findPoseFrame(handle.target, ioEdit.pose);
		if (degrees == nullptr)
			return;
		*degrees= glm::degrees(glm::eulerAngles(glm::normalize(glm::inverse(handle.baseFrameWorld) * draggedFrameWorld)));
		ioEdit.bPoseChanged= true;
		return;
	}

	B trimBone;
	if (ioEdit.rig == nullptr || handle.target.value != eSceneGizmoValue::Trim ||
		!getTrimBone(handle.target.chain, trimBone))
		return;
	glm::vec3& trimDegrees= ioEdit.rig->rotationTrimDegrees[(int)trimBone];
	// The turn in the handle's own axes
	const glm::quat turn= glm::normalize(glm::inverse(handle.frameWorld) * draggedFrameWorld);
	if (handle.trimRule == eSceneGizmoTrimRule::ForearmRoll)
	{
		const float angle= 2.f * std::atan2(turn.x, turn.w);
		trimDegrees.x= wrapTrimDegrees(trimDegrees.x + glm::degrees(angle));
	}
	else
	{
		const glm::quat trim= glm::quat(glm::radians(trimDegrees));
		// The posed bone is the measurement turned back by the trim, so a bone
		// that is to follow the turn takes its inverse ahead of the trim. The
		// trimmed axes take the turn itself.
		const glm::quat turned= handle.trimRule == eSceneGizmoTrimRule::BoneFollows ? glm::inverse(turn) * trim
																					 : trim * turn;
		trimDegrees= glm::degrees(glm::eulerAngles(glm::normalize(turned)));
	}
	ioEdit.bRigChanged= true;
}

bool getSceneGizmoValue(const SceneGizmoTarget& target, const AvatarRigSettings& rig, const RetargetChains& chains,
						const CustomPose* pose, glm::vec3& outValue)
{
	if (target.value == eSceneGizmoValue::Trim)
	{
		B trimBone;
		if (!getTrimBone(target.chain, trimBone))
			return false;
		outValue= rig.rotationTrimDegrees[(int)trimBone];
		return true;
	}
	// Read through the edit's lookups, which write nothing
	SceneGizmoEdit read;
	read.rig= const_cast<AvatarRigSettings*>(&rig);
	read.chains= const_cast<RetargetChains*>(&chains);
	read.pose= const_cast<CustomPose*>(pose);
	if (const glm::vec3* degrees= findPoseFrame(target, read.pose))
	{
		outValue= *degrees;
		return true;
	}
	bool* changed= nullptr;
	const glm::vec3* offset= findOffset(target, read, changed);
	if (offset == nullptr)
		return false;
	outValue= *offset;
	return true;
}

bool setSceneGizmoValue(const SceneGizmoTarget& target, const glm::vec3& value, SceneGizmoEdit& ioEdit)
{
	if (target.value == eSceneGizmoValue::Trim)
	{
		B trimBone;
		if (ioEdit.rig == nullptr || !getTrimBone(target.chain, trimBone))
			return false;
		// A forearm's trim is its roll alone
		ioEdit.rig->rotationTrimDegrees[(int)trimBone]= isArmChain(target.chain) ? glm::vec3(value.x, 0.f, 0.f) : value;
		ioEdit.bRigChanged= true;
		return true;
	}
	if (glm::vec3* degrees= findPoseFrame(target, ioEdit.pose))
	{
		*degrees= value;
		ioEdit.bPoseChanged= true;
		return true;
	}
	bool* changed= nullptr;
	glm::vec3* offset= findOffset(target, ioEdit, changed);
	if (offset == nullptr)
		return false;
	*offset= value;
	*changed= true;
	return true;
}
