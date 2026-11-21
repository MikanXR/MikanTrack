#include "TestCommon.h"

#include <filesystem>
#include <fstream>

#include "glm/gtc/constants.hpp"
#include "glm/gtc/quaternion.hpp"

#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "CustomPose.h"
#include "CustomPoseLibrary.h"
#include "RetargetStage.h"
#include "SyntheticAvatar.h"

using namespace SyntheticAvatar;

// The custom pose, no GL and no window: its file format, the library of
// poses, the built-in pose and its mirror, and the pose as a source in each
// pipeline's form, posed by the target solve.
static int runCustomPoseTest(const TestArgs& args)
{
	int result= 0;
	auto check= [&](bool bPass, const char* name) {
		if (bPass)
			MIKAN_LOG_INFO("test-custom-pose") << "PASS " << name;
		else
		{
			MIKAN_LOG_ERROR("test-custom-pose") << "FAIL " << name;
			result= 1;
		}
	};
	using B= eHumanoidBone;

	// (a) The file: every part round trips, a part that is missing keeps its
	// default, and text that is not a pose is refused
	{
		CustomPose pose= makeDemoCustomPose();
		pose.id= makeCustomPoseId();
		pose.name= "Wave";
		pose.leanDegrees= glm::vec3(4.f, -3.f, 2.f);
		pose.arms[0].bEnabled= false;
		pose.arms[1].bHasElbow= true;
		pose.arms[1].elbowOffset= glm::vec3(0.1f, -0.3f, -0.2f);
		pose.face["phone.jawOpen"]= 0.4f;
		pose.face["viseme.aa"]= 0.f;

		CustomPose read;
		std::string error;
		const bool bRead= customPoseFromJson(customPoseToJson(pose), read, error);
		float maxFingerMiss= 0.f;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				const FingerAngles& a= pose.fingers[sideIndex][finger];
				const FingerAngles& b= read.fingers[sideIndex][finger];
				maxFingerMiss= std::max({maxFingerMiss, std::fabs(a.lateral - b.lateral), std::fabs(a.proximal - b.proximal),
										 std::fabs(a.intermediate - b.intermediate), std::fabs(a.distal - b.distal)});
			}
		}
		const bool bSame= bRead && read.id == pose.id && read.name == "Wave" && read.headDegrees == pose.headDegrees &&
			read.leanDegrees == pose.leanDegrees && !read.arms[0].bEnabled && read.arms[1].bEnabled &&
			!read.arms[0].bHasElbow && read.arms[1].bHasElbow && read.arms[1].elbowOffset == pose.arms[1].elbowOffset &&
			read.arms[0].wristOffset == pose.arms[0].wristOffset && read.arms[1].palmDegrees == pose.arms[1].palmDegrees &&
			maxFingerMiss < 1e-5f;
		// A zero face value is not written
		const bool bFace= read.face.size() == 1 && read.face.count("phone.jawOpen") == 1 &&
			std::fabs(read.face["phone.jawOpen"] - 0.4f) < 1e-6f;
		check(bSame && bFace, "(a) a pose round trips through its file");

		CustomPose partial;
		const bool bPartial= customPoseFromJson("{\"id\": \"abc\", \"arms\": {\"left\": {\"enabled\": false}}}", partial, error);
		const CustomPose defaults= makeDefaultCustomPose();
		CustomPose refused;
		check(bPartial && !partial.arms[0].bEnabled && partial.arms[0].wristOffset == defaults.arms[0].wristOffset &&
				  partial.arms[1].palmDegrees == defaults.arms[1].palmDegrees &&
				  !customPoseFromJson("{ not json", refused, error) && !customPoseFromJson("[1, 2]", refused, error) &&
				  !customPoseFromJson("{\"name\": \"no id\"}", refused, error),
			  "(a2) a missing part keeps its default, and text that is not a pose is refused");
	}

	// (b) The library: one file per id, rename keeps the id, a duplicate gets
	// a fresh one, an unreadable file is skipped, and the built-in id never
	// names a file
	{
		const std::filesystem::path folder= std::filesystem::temp_directory_path() / "mikan-test-customposes";
		std::error_code error;
		std::filesystem::remove_all(folder, error);
		CustomPoseLibrary library(folder);
		library.refresh();
		const bool bEmpty= library.getPoses().empty();

		const std::string firstId= library.create("Zeta");
		CustomPose edited= *library.find(firstId);
		edited.headDegrees= glm::vec3(1.f, 2.f, 3.f);
		library.save(edited);
		const std::string copyId= library.duplicate(edited, "Alpha");
		library.rename(firstId, "Omega");
		{
			std::ofstream junk(folder / "broken.json");
			junk << "{ not json";
		}

		CustomPoseLibrary reread(folder);
		reread.refresh();
		const CustomPose* first= reread.find(firstId);
		const CustomPose* copy= reread.find(copyId);
		const bool bFiles= bEmpty && reread.getPoses().size() == 2 && first != nullptr && copy != nullptr &&
			std::filesystem::is_regular_file(folder / (firstId + ".json"));
		const bool bRenamed= first != nullptr && first->name == "Omega" && first->headDegrees == glm::vec3(1.f, 2.f, 3.f);
		const bool bCopied= copy != nullptr && copyId != firstId && copy->name == "Alpha" &&
			copy->headDegrees == glm::vec3(1.f, 2.f, 3.f) && reread.getPoses()[0].name == "Alpha";
		const bool bRemoved= reread.remove(copyId) && reread.find(copyId) == nullptr &&
			!std::filesystem::exists(folder / (copyId + ".json"));
		CustomPose hostile;
		hostile.id= "../escape";
		const bool bRefused= !reread.save(hostile) && !reread.save(makeDemoCustomPose()) &&
			!CustomPoseLibrary::isValidId(k_demoCustomPoseId) && CustomPoseLibrary::isValidId(firstId);
		check(bFiles && bRenamed && bCopied && bRemoved && bRefused,
			  "(b) the library is one <id>.json per pose, and neither a hostile id nor the built-in one names a file");
		std::filesystem::remove_all(folder, error);
	}

	// (c) The built-in pose is the mirror image of itself in its arms, and a
	// mirrored hand spreads the other way
	{
		const CustomPose demo= makeDemoCustomPose();
		const CustomPose::Arm mirrored= mirrorCustomPoseArm(demo.arms[0]);
		const glm::quat mirroredPalm= glm::quat(glm::radians(mirrored.palmDegrees));
		const glm::quat rightPalm= glm::quat(glm::radians(demo.arms[1].palmDegrees));
		const bool bArm= glm::length(mirrored.wristOffset - demo.arms[1].wristOffset) < 1e-6f &&
			std::fabs(glm::dot(mirroredPalm, rightPalm)) > 1.f - 1e-6f;
		const std::array<FingerAngles, FINGER_COUNT> spread= mirrorCustomPoseFingers(demo.fingers[0]);
		const bool bFingers= spread[(int)eFinger::Index].lateral == -demo.fingers[0][(int)eFinger::Index].lateral &&
			spread[(int)eFinger::Index].proximal == demo.fingers[0][(int)eFinger::Index].proximal;
		check(isBuiltInCustomPoseId(demo.id) && !isBuiltInCustomPoseId(makeCustomPoseId()) && bArm && bFingers &&
				  demo.fingers[1][(int)eFinger::Index].proximal > 1.f && demo.headDegrees.z == 25.f,
			  "(c) the built-in pose, and an arm and a hand mirrored across the midline");
	}

	const AvatarModel model= makeModel(eVrmVersion::Vrm1, false);
	const std::shared_ptr<const AvatarSkeleton> skeleton= std::make_shared<AvatarSkeleton>(model);
	const AvatarRigSettings rig;
	const float armLength= skeleton->getUpperArmLength(eHandSide::Left) + skeleton->getForearmLength(eHandSide::Left);
	CustomPose pose= makeDemoCustomPose();
	pose.arms[1].bEnabled= false;

	// (d) The position form: the wrist where the pose puts it off the avatar's
	// shoulder under the fixed root, the hand the source layer draws, an arm
	// that is off lost, and the solve reaching the wrist
	{
		CustomPoseSolveInputs inputs;
		inputs.skeleton= skeleton.get();
		inputs.rig= &rig;
		inputs.rootPositionWorld= glm::vec3(0.3f, -0.2f, 0.f);
		inputs.rootYawDegrees= 90.f;
		RetargetResult solved;
		fillCustomSource(pose, inputs, SourcePose::Clock(), solved);

		const glm::quat root= glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.f, 0.f, 1.f));
		const glm::vec3 shoulder= inputs.rootPositionWorld + root * skeleton->getBone(B::LeftUpperArm).restPositionWorld;
		const glm::vec3 wrist= shoulder + root * (pose.arms[0].wristOffset * armLength);
		const SourcePose& source= solved.source;
		const bool bSource= source.pipeline == SourcePose::ePipeline::Position &&
			source.arms[0].state == SourcePose::eGroupState::Measured && !source.arms[0].bHasShoulder &&
			!source.arms[0].bHasElbow && glm::length(source.arms[0].wristPosition - wrist) < 1e-5f &&
			source.hands[0].state == SourcePose::eGroupState::Measured &&
			source.head.state == SourcePose::eGroupState::Measured && source.torso.state == SourcePose::eGroupState::Lost;
		const bool bOff= source.arms[1].state == SourcePose::eGroupState::Lost &&
			source.hands[1].state == SourcePose::eGroupState::Lost && !solved.hands.bCarried[1];
		const bool bHand= solved.hands.bCarried[0] && solved.hands.poses[0].hasWorldPose &&
			glm::length(solved.hands.poses[0].getWristPositionWorld() - wrist) < 1e-5f;

		RetargetStageConfig config;
		config.skeleton= skeleton;
		config.position.followShoulders= false;
		config.position.fixedRootPositionWorld= inputs.rootPositionWorld;
		config.position.fixedRootYawDegrees= inputs.rootYawDegrees;
		RetargetStage stage;
		stage.setConfig(config);
		stage.solveTarget(solved.source, solved.pose);
		std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
		computePosedBones(*skeleton, solved.pose, posed);
		const float wristMiss= glm::length(posed[(int)B::LeftHand].positionWorld - wrist);
		check(bSource && bOff && bHand && solved.pose.valid && wristMiss < 2e-3f,
			  "(d) a position source puts the wrist where the pose asks under the fixed root, and the solve reaches it");
	}

	// (e) The direct form: the chain's joints at the avatar's bone lengths
	// with the wrist on its target, the lean as the spine's, an elbow the
	// pose gives taken, and the placement posing the hand's palm where the
	// pose turned it
	{
		pose.leanDegrees= glm::vec3(0.f, 8.f, 0.f);
		CustomPoseSolveInputs inputs;
		inputs.skeleton= skeleton.get();
		inputs.rig= &rig;
		inputs.bDirect= true;
		RetargetResult solved;
		std::array<glm::quat, 2> palms;
		fillCustomSource(pose, inputs, SourcePose::Clock(), solved, &palms);

		const SourcePose& source= solved.source;
		const SourcePose::Arm& arm= source.arms[0];
		const float upper= skeleton->getUpperArmLength(eHandSide::Left);
		const float fore= skeleton->getForearmLength(eHandSide::Left);
		const glm::quat lean= glm::quat(glm::radians(pose.leanDegrees));
		const glm::vec3 target= arm.shoulderPosition + lean * (pose.arms[0].wristOffset * armLength);
		const bool bChain= source.pipeline == SourcePose::ePipeline::Direct && arm.bHasShoulder && arm.bHasElbow &&
			std::fabs(glm::length(arm.elbowPosition - arm.shoulderPosition) - upper) < 1e-5f &&
			std::fabs(glm::length(arm.wristPosition - arm.elbowPosition) - fore) < 1e-5f &&
			glm::length(arm.wristPosition - target) < 1e-4f && arm.forearmConfidence.value == 0.f;
		const bool bTorso= source.torso.state == SourcePose::eGroupState::Measured &&
			std::fabs(glm::dot(source.torso.spineDelta, lean)) > 1.f - 1e-6f &&
			source.arms[1].state == SourcePose::eGroupState::Lost;

		RetargetStageConfig config;
		config.skeleton= skeleton;
		config.direct.wristLimitRadians= {glm::pi<float>(), glm::pi<float>()};
		RetargetStage stage;
		stage.setConfig(config);
		stage.solveTarget(solved.source, solved.pose);
		std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
		computePosedBones(*skeleton, solved.pose, posed);
		const float wristMiss= glm::length(posed[(int)B::LeftHand].positionWorld - arm.wristPosition);
		// The hand bone carries the avatar's rest palm frame to the posed one
		const glm::quat restPalm= glm::quat_cast(glm::mat3(skeleton->getHand(eHandSide::Left).palmFrameWorld));
		const glm::quat posedPalm= posed[(int)B::LeftHand].rotationWorld *
			glm::inverse(skeleton->getBone(B::LeftHand).restRotationWorld) * restPalm;
		const float palmMatch= std::fabs(glm::dot(glm::normalize(posedPalm), palms[0]));

		// A given elbow sets the chain's directions and reads as measured
		CustomPose elbowed= pose;
		elbowed.arms[0].bHasElbow= true;
		elbowed.arms[0].elbowOffset= glm::vec3(0.1f, 0.5f, -0.1f);
		RetargetResult withElbow;
		fillCustomSource(elbowed, inputs, SourcePose::Clock(), withElbow);
		const SourcePose::Arm& elbowArm= withElbow.source.arms[0];
		const glm::vec3 givenDirection= glm::normalize(lean * elbowed.arms[0].elbowOffset);
		const bool bElbow= glm::length(elbowArm.upperDirection - givenDirection) < 1e-5f &&
			elbowArm.forearmConfidence.value == 1.f;
		check(bChain && bTorso && solved.pose.valid && wristMiss < 2e-3f && palmMatch > 1.f - 1e-4f && bElbow,
			  "(e) a direct source is a chain at the avatar's lengths on its wrist target, and the placement poses the "
			  "palm the pose gives");
	}

	// (f) Nothing is filled without a skeleton
	{
		RetargetResult empty;
		fillCustomSource(pose, CustomPoseSolveInputs(), SourcePose::Clock(), empty);
		check(empty.source.pipeline == SourcePose::ePipeline::None, "(f) no skeleton fills no source");
	}

	MIKAN_LOG_INFO("test-custom-pose") << (result == 0 ? "ALL PASS" : "FAILURES");
	return result;
}

MIKAN_REGISTER_TEST("--test-custom-pose",
					"Custom pose file, library, built-in pose, and its source in the position and direct forms",
					eTestCategory::SelfTest, runCustomPoseTest);
