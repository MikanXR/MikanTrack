#pragma once

#include <array>
#include <map>
#include <string>

#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/vector_float3.hpp"

#include "RetargetStage.h" // RetargetResult, SourcePose
#include "TrackingTypes.h" // FingerAngles

class AvatarSkeleton;
struct AvatarRigSettings;

// A pose made by hand, stored as what tracking would have measured rather
// than as bones, so it runs through the retarget the way a tracked frame
// does and every chain setting shows on it. Nothing in it is an avatar's:
// the wrists are placed in arm lengths and every rotation is in the rooted
// torso frame (+X facing, +Y the avatar's left, +Z up), so one pose fits any
// avatar and either pipeline.
struct CustomPose
{
	static constexpr int k_version= 1;

	std::string id;
	std::string name;

	// The head's rotation from facing straight ahead, XYZ Euler degrees
	glm::vec3 headDegrees{0.f};
	// The torso's lean from upright, XYZ Euler degrees. Only the direct
	// pipeline places a spine, so a multi-camera project ignores it.
	glm::vec3 leanDegrees{0.f};

	struct Arm
	{
		// Off, the arm and its hand read as lost
		bool bEnabled= true;
		// The wrist from the shoulder joint, in arm lengths (the upper arm
		// plus the forearm), so 1 is a straight arm
		glm::vec3 wristOffset{0.6f, 0.f, -0.26f};
		// The palm frame (+X toward the fingers, +Z out of the palm), XYZ
		// Euler degrees
		glm::vec3 palmDegrees{0.f};
		// The elbow as a measured one, from the shoulder joint in arm
		// lengths. Without it the elbow is left to the rig's hint.
		bool bHasElbow= false;
		glm::vec3 elbowOffset{0.2f, 0.f, -0.4f};
	};
	std::array<Arm, 2> arms{}; // by eHandSide

	// Radians, against the avatar's rest hand
	std::array<std::array<FingerAngles, FINGER_COUNT>, 2> fingers{}; // by eHandSide

	// The face inputs a profile reads, by the name the expression language
	// gives them ("phone.jawOpen"). An input that is not listed is zero.
	std::map<std::string, float> face;
};

// The id of the built-in pose, which is never a file's
extern const char* const k_demoCustomPoseId;
bool isBuiltInCustomPoseId(const std::string& id);

// A fresh id for a stored pose
std::string makeCustomPoseId();

// The built-in pose: hands up in front of the chest, the right hand curled
// and the left spread, the head turned, no measured elbows, so the elbow
// hints show
CustomPose makeDemoCustomPose();
// What a new pose starts as: the built-in pose's arms with open hands and
// the head straight
CustomPose makeDefaultCustomPose();

// A pose mirrored across the body's midline, the left side's values on the
// right and the right's on the left
CustomPose::Arm mirrorCustomPoseArm(const CustomPose::Arm& arm);
std::array<FingerAngles, FINGER_COUNT> mirrorCustomPoseFingers(const std::array<FingerAngles, FINGER_COUNT>& fingers);

// The file format: the finger angles in degrees, a finger with all four
// angles at zero and an arm's elbow without one left out. Reading takes a
// file with any part missing and leaves that part at its default; false,
// with a message, for text that is not a pose.
std::string customPoseToJson(const CustomPose& pose);
bool customPoseFromJson(const std::string& text, CustomPose& outPose, std::string& outError);

// What the pose is placed on
struct CustomPoseSolveInputs
{
	const AvatarSkeleton* skeleton= nullptr;
	// The rig's elbow hints, where the direct chain's elbow goes when the
	// pose gives none
	const AvatarRigSettings* rig= nullptr;
	// The project's pipeline: the direct one, or the position one
	bool bDirect= false;
	// The position pipeline's fixed root, which places the pose in the world
	glm::vec3 rootPositionWorld{0.f};
	float rootYawDegrees= 0.f;
};

// The pose as the source the target solve takes, every group it sets
// measured and each arm that is off lost, in the pipeline's own form, with
// the clock given. The pose's lengths are the avatar's own, so a wrist is
// reached where it was put.
//
// Position: the wrist, the palm and the elbow in the world under the fixed
// root, no shoulders, so the fixed root places the avatar, and the hands the
// source pose layer draws. Direct: the chain's joints in the solve's world,
// which is the avatar's with the root at the identity, the elbow nearest the
// rig's hint when the pose gives none, the chain's directions and the palm
// as the forearm's twist and the wrist, and the lean as the spine's. outDirectPalms,
// when given, receives each direct palm frame in that world for the source
// pose layer.
//
// Fills outResult's source and hands and leaves its pose to the target solve.
// Nothing is filled without a skeleton.
void fillCustomSource(const CustomPose& pose, const CustomPoseSolveInputs& inputs, const SourcePose::Clock& clock,
					  RetargetResult& outResult, std::array<glm::quat, 2>* outDirectPalms= nullptr);
