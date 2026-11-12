#include "TestCommon.h"

#include "glm/gtx/quaternion.hpp"

#include "AvatarRetarget.h"
#include "AvatarRetargetMath.h"
#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "DirectRetarget.h"
#include "DirectStageSolver.h"
#include "MathGLM.h"
#include "OneEuroFilter.h"
#include "SyntheticAvatar.h"
#include "SyntheticDirectUser.h"

// The direct retarget on the shared synthetic rig, seen by a synthetic
// pinhole pitched down at a synthetic user built from the avatar's own
// lengths: the neutral capture zeroes the torso, head and wrists, the arm
// directions come back through the tilt conversion, the model's depth and
// the elbow's reprojection pick the wrist's distance, the model's depth
// places an unseen elbow's wrist and the hand lift's depth is measured
// against the seen elbow's, a lost hand holds, a palm flip is held out,
// pixel jitter stays out of the depth, a gap rests, and two solvers agree
// frame for frame.

namespace
{
using B= eHumanoidBone;
using namespace SyntheticAvatar;
using namespace SyntheticDirectUser;

constexpr float kTiltDegrees= 10.f;
constexpr float kDirectionToleranceRadians= 3.f * glm::pi<float>() / 180.f;

float angleBetween(const glm::vec3& a, const glm::vec3& b)
{
	const float d= glm::dot(glm::normalize(a), glm::normalize(b));
	return acosf(std::clamp(d, -1.f, 1.f));
}

float rotationAngle(const glm::quat& q)
{
	const glm::quat unit= glm::normalize(q);
	return 2.f * acosf(std::min(std::fabs(unit.w), 1.f));
}

bool nearlySameRotation(const glm::quat& a, const glm::quat& b, float toleranceRadians)
{
	return rotationAngle(glm::inverse(a) * b) <= toleranceRadians;
}

bool nearlyEqual(float a, float b, float tolerance)
{
	return std::fabs(a - b) <= tolerance;
}

struct Rig
{
	AvatarModel model;
	std::unique_ptr<AvatarSkeleton> skeleton;
	DirectRetargetConfig config;
};

Rig makeRig(float tiltDegrees)
{
	Rig rig;
	rig.model= makeModel(eVrmVersion::Vrm1, false);
	rig.skeleton= std::make_unique<AvatarSkeleton>(rig.model);
	rig.config.cameraTiltRadians= glm::radians(tiltDegrees);
	// The user's shoulders and arms are the avatar's, so the head keeps its
	// size and the chain the avatar's lengths
	rig.config.userShoulderWidthMeters= kShoulderWidth;
	rig.config.userUpperArmLengthMeters= kUpperArmLength;
	rig.config.userForearmLengthMeters= kForearmLength;
	return rig;
}

// Runs the neutral frame a few times (the filters settle on repeats of one
// pose), captures it, and installs it in the config
void captureNeutral(Rig& rig, const Camera& camera, DirectStageSolver& solver, double& ioTimestampMs)
{
	const User user= makeNeutralUser();
	const TrackingFrameResult::HeadPose noPhone;
	AvatarPose pose;
	for (int repeat= 0; repeat < 3; ++repeat)
	{
		const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, ioTimestampMs, repeat);
		solver.solve(frame, camera.model, noPhone, ioTimestampMs, *rig.skeleton, rig.config, pose);
		ioTimestampMs+= 33.0;
	}
	solver.captureNeutral(rig.config.neutral);
}

// Posed world directions of the arm bones for one side
struct ArmDirections
{
	glm::vec3 upper;
	glm::vec3 forearm;
	glm::quat handInForearm; // the hand's rotation relative to the forearm's
};

ArmDirections posedArm(const AvatarSkeleton& skeleton, const AvatarPose& pose, int sideIndex)
{
	std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> posed;
	computePosedBones(skeleton, pose, posed);
	const B upper= sideIndex == 0 ? B::LeftUpperArm : B::RightUpperArm;
	const B lower= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
	const B hand= sideIndex == 0 ? B::LeftHand : B::RightHand;
	ArmDirections out;
	out.upper= posed[(int)lower].positionWorld - posed[(int)upper].positionWorld;
	out.forearm= posed[(int)hand].positionWorld - posed[(int)lower].positionWorld;
	out.handInForearm= glm::inverse(posed[(int)lower].rotationWorld) * posed[(int)hand].rotationWorld;
	return out;
}

glm::quat restHandInForearm(const AvatarSkeleton& skeleton, int sideIndex)
{
	const B lower= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
	const B hand= sideIndex == 0 ? B::LeftHand : B::RightHand;
	return glm::inverse(skeleton.getBone(lower).restRotationWorld) * skeleton.getBone(hand).restRotationWorld;
}

bool samePose(const AvatarPose& a, const AvatarPose& b)
{
	if (a.valid != b.valid)
		return false;
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		if (a.bones[index].present != b.bones[index].present)
			return false;
		if (std::memcmp(&a.bones[index].deltaWorld, &b.bones[index].deltaWorld, sizeof(glm::quat)) != 0)
			return false;
	}
	return true;
}

// Whether an arm's bones (upper arm, forearm, hand, index base) sit where a
// reference pose puts them
bool armAtPose(const AvatarPose& pose, const AvatarPose& reference, int sideIndex)
{
	const B bones[]= {sideIndex == 0 ? B::LeftUpperArm : B::RightUpperArm,
					  sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm, sideIndex == 0 ? B::LeftHand : B::RightHand,
					  sideIndex == 0 ? B::LeftIndexProximal : B::RightIndexProximal};
	for (const B bone : bones)
	{
		const AvatarPose::Bone& a= pose.bones[(int)bone];
		const AvatarPose::Bone& b= reference.bones[(int)bone];
		if (a.present != b.present || (a.present && !nearlySameRotation(a.deltaWorld, b.deltaWorld, 1e-3f)))
			return false;
	}
	return true;
}
} // namespace

static int runDirectRetargetTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO("test-direct-retarget") << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR("test-direct-retarget") << "FAIL " << name;
			failures++;
		}
	};
	const TrackingFrameResult::HeadPose noPhone;

	// (a) The conversion: a level camera sees the user's left on its right
	// and the user's up as image up; the tilt turns a point ahead of a
	// pitched camera into one below the horizon
	{
		const glm::quat level= DirectRetarget::worldFromCamera(0.f);
		check(glm::length(level * glm::vec3(1.f, 0.f, 0.f) - glm::vec3(0.f, 1.f, 0.f)) < 1e-5f &&
				  glm::length(level * glm::vec3(0.f, -1.f, 0.f) - glm::vec3(0.f, 0.f, 1.f)) < 1e-5f &&
				  glm::length(level * glm::vec3(0.f, 0.f, 1.f) - glm::vec3(-1.f, 0.f, 0.f)) < 1e-5f,
			  "(a) a level camera's right is world +Y, its up world +Z, its forward world -X");
		const glm::vec3 ahead= DirectRetarget::worldFromCamera(glm::radians(kTiltDegrees)) * glm::vec3(0.f, 0.f, 1.f);
		check(ahead.z < -0.1f && ahead.x < -0.9f, "(a) a camera pitched down looks down in the world");
	}

	// (b) Neutral: captured then solved, the torso, head and wrists sit at
	// identity, the arm directions come back through the tilt conversion,
	// and the chain keeps the avatar's lengths
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);
		check(rig.config.neutral.present, "(b) the neutral capture measured a torso");

		const User user= makeNeutralUser();
		AvatarPose pose;
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 10 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, pose);
			timestampMs+= 33.0;
		}
		check(pose.valid && pose.bones[(int)B::Hips].present && pose.bones[(int)B::Spine].present &&
				  rotationAngle(pose.bones[(int)B::Spine].deltaWorld) < 1e-3f &&
				  rotationAngle(pose.bones[(int)B::Neck].deltaWorld) < 1e-3f,
			  "(b) the neutral torso leaves the spine at identity");
		check(pose.bones[(int)B::Head].present && rotationAngle(pose.bones[(int)B::Head].deltaWorld) < 1e-3f,
			  "(b) the neutral head is identity");
		bool bArms= true;
		bool bWrists= true;
		bool bLengths= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const ArmDirections arm= posedArm(*rig.skeleton, pose, sideIndex);
			bArms&= angleBetween(arm.upper, user.elbow[sideIndex] - user.shoulder[sideIndex]) < kDirectionToleranceRadians;
			bArms&= angleBetween(arm.forearm, user.wrist[sideIndex] - user.elbow[sideIndex]) < kDirectionToleranceRadians;
			bWrists&= nearlySameRotation(arm.handInForearm, restHandInForearm(*rig.skeleton, sideIndex), 1e-3f);
			const DirectRetarget::ArmChain& chain= solver.getArmChain(sideIndex);
			bLengths&= chain.valid && nearlyEqual(glm::length(chain.elbow - chain.shoulder), kUpperArmLength, 1e-3f) &&
				nearlyEqual(glm::length(chain.wrist - chain.elbow), kForearmLength, 1e-3f);
			const glm::vec2 wristPx= project(camera, chain.wrist);
			bLengths&= glm::length(wristPx - project(camera, user.wrist[sideIndex])) < 1.f;
		}
		check(bArms, "(b) hanging upper arms and forward forearms come back within 3 degrees");
		check(bWrists, "(b) a straight wrist is the avatar's rest wrist");
		check(bLengths, "(b) the chain keeps the avatar's lengths with the wrist on its pixel");
		// The finger bones are placed, at identity for zero angles
		check(pose.bones[(int)B::LeftIndexProximal].present && pose.bones[(int)B::RightLittleDistal].present,
			  "(b) the finger bones are placed");

		// A hand unseen at the capture has no reference to miss: seen straight
		// afterwards it poses the rest wrist like the other
		{
			User unseen= makeNeutralUser();
			unseen.bHandTracked[0]= false;
			DirectStageSolver later;
			Rig laterRig= makeRig(kTiltDegrees);
			for (int repeat= 0; repeat < 3; ++repeat)
			{
				const TrackingFrameResult frame= makeFrame(camera, unseen, *laterRig.skeleton, timestampMs, 20 + repeat);
				later.solve(frame, camera.model, noPhone, timestampMs, *laterRig.skeleton, laterRig.config, pose);
				timestampMs+= 33.0;
			}
			later.captureNeutral(laterRig.config.neutral);
			for (int repeat= 0; repeat < 3; ++repeat)
			{
				const TrackingFrameResult frame= makeFrame(camera, user, *laterRig.skeleton, timestampMs, 30 + repeat);
				later.solve(frame, camera.model, noPhone, timestampMs, *laterRig.skeleton, laterRig.config, pose);
				timestampMs+= 33.0;
			}
			bool bRestWrists= laterRig.config.neutral.present;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const ArmDirections arm= posedArm(*laterRig.skeleton, pose, sideIndex);
				bRestWrists&= nearlySameRotation(arm.handInForearm, restHandInForearm(*laterRig.skeleton, sideIndex), 1e-3f);
			}
			check(bRestWrists, "(b) a hand unseen at the capture poses the rest wrist once seen straight");
		}
	}

	// (c) The depth sign: a forearm pointing away from the camera takes the
	// far crossing, toward it the near one, on the same wrist pixel geometry
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		User user= makeNeutralUser();
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			// Elbow out to the side so the backward forearm clears the torso
			user.elbow[sideIndex]= user.shoulder[sideIndex] +
				glm::normalize(glm::vec3(0.f, sideIndex == 0 ? 0.6f : -0.6f, -0.8f)) * kUpperArmLength;
			user.wrist[sideIndex]= user.elbow[sideIndex] + glm::vec3(-kForearmLength, 0.f, 0.f);
			user.palm[sideIndex]= makePalm(glm::vec3(-1.f, 0.f, 0.f), glm::vec3(0.f, 0.f, -1.f));
		}
		AvatarPose pose;
		solver.reset();
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 20 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, pose);
			timestampMs+= 33.0;
		}
		bool bAway= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const ArmDirections arm= posedArm(*rig.skeleton, pose, sideIndex);
			bAway&= angleBetween(arm.forearm, glm::vec3(-1.f, 0.f, 0.f)) < kDirectionToleranceRadians;
			bAway&= angleBetween(arm.upper, user.elbow[sideIndex] - user.shoulder[sideIndex]) < kDirectionToleranceRadians;
		}
		check(bAway, "(c) a forearm pointing away from the camera takes the far crossing");

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			user.wrist[sideIndex]= user.elbow[sideIndex] + glm::vec3(kForearmLength, 0.f, 0.f);
			user.palm[sideIndex]= makePalm(glm::vec3(1.f, 0.f, 0.f), glm::vec3(0.f, 0.f, -1.f));
		}
		solver.reset();
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 30 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, pose);
			timestampMs+= 33.0;
		}
		bool bToward= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			bToward&= angleBetween(posedArm(*rig.skeleton, pose, sideIndex).forearm, glm::vec3(1.f, 0.f, 0.f)) <
				kDirectionToleranceRadians;
		check(bToward, "(c) a forearm pointing at the camera takes the near crossing");
	}

	// (d) An unseen elbow: the wrist stays on its ray within reach, both
	// bones keep their lengths, and the model's depth places the wrist
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		User user= makeNeutralUser();
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			user.bElbowSeen[sideIndex]= false;
			user.elbow[sideIndex]= user.shoulder[sideIndex] +
				glm::normalize(glm::vec3(0.6f, sideIndex == 0 ? 0.2f : -0.2f, 0.8f)) * kUpperArmLength;
			user.wrist[sideIndex]= user.elbow[sideIndex] + glm::vec3(kForearmLength, 0.f, 0.f);
		}
		AvatarPose pose;
		solver.reset();
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 40 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, pose);
			timestampMs+= 33.0;
		}
		bool bFallback= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::ArmChain& chain= solver.getArmChain(sideIndex);
			bFallback&= chain.valid;
			bFallback&= nearlyEqual(glm::length(chain.elbow - chain.shoulder), kUpperArmLength, 1e-3f);
			bFallback&= nearlyEqual(glm::length(chain.wrist - chain.elbow), kForearmLength, 1e-3f);
			bFallback&= glm::length(project(camera, chain.wrist) - project(camera, user.wrist[sideIndex])) < 1.f;
			bFallback&= glm::length(chain.wrist - chain.shoulder) <= kUpperArmLength + kForearmLength + 1e-4f;
		}
		check(bFallback, "(d) an unseen elbow keeps the wrist on its ray within reach with both lengths");
		bool bDepth= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const float truth= glm::length(user.wrist[sideIndex]);
			bDepth&= nearlyEqual(glm::length(solver.getArmChain(sideIndex).wrist), truth, 0.05f * truth);
		}
		check(bDepth, "(d) the model's depth places an unseen elbow's wrist within 5 percent");
	}

	// (e) An open hand poses the avatar's rest fingers with no zero captured
	// from the user, curled fingers bend them, and a lost hand: the body
	// model's wrist keeps the arm placed and the fingers hold their last
	// angles
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		User user= makeNeutralUser();
		AvatarPose open;
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 40 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, open);
			timestampMs+= 33.0;
		}
		bool bRestFingers= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const B hand= sideIndex == 0 ? B::LeftHand : B::RightHand;
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				for (int phalanx= 0; phalanx < 3; ++phalanx)
				{
					const AvatarPose::Bone& bone=
						open.bones[(int)firstHumanoidFingerBone(sideIndex) + finger * 3 + phalanx];
					if (!bone.present)
						continue;
					bRestFingers&= nearlySameRotation(bone.deltaWorld, open.bones[(int)hand].deltaWorld, 1e-3f);
				}
			}
		}
		check(bRestFingers, "(e) an open hand poses the avatar's rest fingers");

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
				user.fingers[sideIndex][finger].proximal= 1.f;
		AvatarPose curled;
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 50 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, curled);
			timestampMs+= 33.0;
		}
		check(rotationAngle(curled.bones[(int)B::LeftIndexProximal].deltaWorld) > 0.5f,
			  "(e) curled fingers bend the finger bones");

		user.bHandTracked[0]= false;
		user.bHandTracked[1]= false;
		// The arm moves while the hand is lost, and stays long enough for the
		// position filters to settle on the new wrist
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.wrist[sideIndex]= user.elbow[sideIndex] + glm::normalize(glm::vec3(0.8f, 0.f, 0.6f)) * kForearmLength;
		AvatarPose lost;
		for (int repeat= 0; repeat < 40; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 60 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, lost);
			timestampMs+= 33.0;
		}
		bool bPlaced= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::ArmChain& chain= solver.getArmChain(sideIndex);
			bPlaced&= chain.valid && glm::length(project(camera, chain.wrist) - project(camera, user.wrist[sideIndex])) < 1.f;
		}
		check(bPlaced, "(e) the body model's wrist keeps the arm placed through a hand dropout");
		bool bHeld= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			// The finger bend relative to the hand is what holds; the hand
			// itself followed the arm
			const B hand= sideIndex == 0 ? B::LeftHand : B::RightHand;
			const B index= sideIndex == 0 ? B::LeftIndexProximal : B::RightIndexProximal;
			const glm::quat before= glm::inverse(curled.bones[(int)hand].deltaWorld) * curled.bones[(int)index].deltaWorld;
			const glm::quat after= glm::inverse(lost.bones[(int)hand].deltaWorld) * lost.bones[(int)index].deltaWorld;
			bHeld&= nearlySameRotation(before, after, 1e-3f);
		}
		check(bHeld, "(e) the fingers hold their last angles while the hand is lost");
	}

	// (f) A short dropout holds, a gap past the hold rests, and tracking
	// returns without a jump from the gap
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		User user= makeNeutralUser();
		AvatarPose tracked;
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 70 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, tracked);
			timestampMs+= 33.0;
		}

		TrackingFrameResult empty;
		empty.timestampMs= timestampMs;
		AvatarPose held;
		solver.solve(empty, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, held);
		check(samePose(tracked, held), "(f) a frame with nothing seen holds the last pose");

		timestampMs+= 700.0;
		empty.timestampMs= timestampMs;
		AvatarPose rest;
		solver.solve(empty, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, rest);
		AvatarPose restReference;
		DirectRetarget::makeRestPose(*rig.skeleton, rig.config, restReference);
		bool bRest= rest.valid && armAtPose(rest, restReference, 0) && armAtPose(rest, restReference, 1);
		int presentCount= 0;
		for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
			presentCount+= rest.bones[index].present ? 1 : 0;
		check(bRest && presentCount > 10, "(f) past the hold the arms sit at the rig's rest pose, every bone placed");

		timestampMs+= 33.0;
		AvatarPose back;
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 80 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, back);
			timestampMs+= 33.0;
		}
		bool bBack= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const ArmDirections arm= posedArm(*rig.skeleton, back, sideIndex);
			bBack&= angleBetween(arm.forearm, user.wrist[sideIndex] - user.elbow[sideIndex]) < kDirectionToleranceRadians;
		}
		check(bBack, "(f) tracking returns after the gap");
	}

	// (g) Two solvers over the same frames produce identical poses
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solverA;
		DirectStageSolver solverB;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solverA, timestampMs);
		solverA.reset();

		bool bSame= true;
		User user= makeNeutralUser();
		for (int step= 0; step < 12; ++step)
		{
			const float phase= (float)step * 0.2f;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				user.wrist[sideIndex]= user.elbow[sideIndex] +
					glm::normalize(glm::vec3(1.f, 0.3f * sinf(phase), 0.4f * cosf(phase))) * kForearmLength;
				user.fingers[sideIndex][1].proximal= 0.5f + 0.4f * sinf(phase);
			}
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 90 + step);
			AvatarPose poseA;
			AvatarPose poseB;
			solverA.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, poseA);
			solverB.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, poseB);
			bSame&= samePose(poseA, poseB);
			timestampMs+= 33.0;
		}
		check(bSame, "(g) two solvers agree bit for bit over a moving sequence");
	}

	// (h) The phone head wins over the camera head while it streams
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		TrackingFrameResult::HeadPose phone;
		phone.valid= true;
		phone.orientationWorld= glm::angleAxis(glm::radians(30.f), glm::vec3(0.f, 0.f, 1.f));
		AvatarPose pose;
		const TrackingFrameResult frame= makeFrame(camera, makeNeutralUser(), *rig.skeleton, timestampMs, 100);
		solver.solve(frame, camera.model, phone, timestampMs, *rig.skeleton, rig.config, pose);
		check(nearlySameRotation(pose.bones[(int)B::Head].deltaWorld, phone.orientationWorld, 1e-4f),
			  "(h) the phone head drives the head bone while it streams");
	}

	// (j) A palm lifted on the wrong depth guess (mirrored along its view
	// ray) solves the same wrist and twist as the true palm: the forearm
	// picks the mirror
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		// The wrist bent 30 degrees up and the palm rolled 20 degrees
		User user= makeNeutralUser();
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const glm::quat bend= glm::angleAxis(glm::radians(-30.f), glm::vec3(0.f, 1.f, 0.f)) *
				glm::angleAxis(glm::radians(20.f), glm::vec3(1.f, 0.f, 0.f));
			user.palm[sideIndex]= makePalm(bend * glm::vec3(1.f, 0.f, 0.f), bend * glm::vec3(0.f, 0.f, -1.f));
		}
		AvatarPose truePose;
		solver.reset();
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 110 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, truePose);
			timestampMs+= 33.0;
		}
		User mirrored= user;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			mirrored.palm[sideIndex]= mirrorPalmAlongRay(user.palm[sideIndex], user.wrist[sideIndex]);
		AvatarPose mirroredPose;
		solver.reset();
		for (int repeat= 0; repeat < 3; ++repeat)
		{
			const TrackingFrameResult frame= makeFrame(camera, mirrored, *rig.skeleton, timestampMs, 120 + repeat);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, mirroredPose);
			timestampMs+= 33.0;
		}
		bool bBent= true;
		bool bSame= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const ArmDirections trueArm= posedArm(*rig.skeleton, truePose, sideIndex);
			const ArmDirections mirroredArm= posedArm(*rig.skeleton, mirroredPose, sideIndex);
			bBent&= !nearlySameRotation(trueArm.handInForearm, restHandInForearm(*rig.skeleton, sideIndex),
										glm::radians(10.f));
			bSame&= nearlySameRotation(trueArm.handInForearm, mirroredArm.handInForearm, 1e-3f);
			const B lower= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
			bSame&= nearlySameRotation(truePose.bones[(int)lower].deltaWorld, mirroredPose.bones[(int)lower].deltaWorld,
									   1e-3f);
		}
		check(bBent, "(j) a bent wrist leaves the rest wrist");
		check(bSame, "(j) a palm mirrored along its view ray solves the same wrist and forearm roll");
	}

	// (j2) The mirror needs evidence. With the fingers upright the finger
	// axis barely leans along the view ray, so which of the palm and its
	// mirror continues the forearm better is noise: the lifted palm stands,
	// and a hand turned through edge-on with that lean changing sign turns
	// smoothly. The mirror is taken above the enter bar and kept down to the
	// leave bar.
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);
		int frameIndex= 600;
		auto solveFrame= [&](const User& frameUser) {
			AvatarPose pose;
			const TrackingFrameResult frame= makeFrame(camera, frameUser, *rig.skeleton, timestampMs, frameIndex++);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, pose);
			timestampMs+= 33.0;
		};

		// The hands held up: the upper arms forward and a little down, the
		// forearms straight up, the fingers continuing them
		User raised= makeNeutralUser();
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			raised.elbow[sideIndex]=
				raised.shoulder[sideIndex] + kUpperArmLength * glm::normalize(glm::vec3(0.95f, 0.f, -0.3f));
			raised.wrist[sideIndex]= raised.elbow[sideIndex] + glm::vec3(0.f, 0.f, kForearmLength);
		}
		// The palm turned by an angle about the fingers from facing the
		// camera, the finger axis leaning along the view ray by lean
		auto setRaisedPalms= [&](float turnRadians, float lean) {
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const glm::vec3 ray= glm::normalize(raised.wrist[sideIndex]);
				const glm::vec3 up(0.f, 0.f, 1.f);
				const glm::vec3 facing= glm::normalize(-ray - up * glm::dot(-ray, up));
				const glm::vec3 normal= glm::angleAxis(turnRadians, up) * facing;
				raised.palm[sideIndex]= makePalm(up + lean * ray, normal);
			}
		};
		solver.reset();
		setRaisedPalms(0.f, 0.05f);
		for (int repeat= 0; repeat < 5; ++repeat)
			solveFrame(raised);
		bool bRead= true;
		bool bNeverMirrored= true;
		float largestStep= 0.f;
		float lastTwist[2]= {solver.getSolveState().arms[0].twist, solver.getSolveState().arms[1].twist};
		for (int step= 1; step <= 60; ++step)
		{
			// The lean's sign holds for three frames, long enough to pass
			// the palm's one-frame hold
			setRaisedPalms(glm::radians(3.f * (float)step), (step / 3) % 2 == 0 ? 0.05f : -0.05f);
			solveFrame(raised);
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const DirectRetarget::SolveState::Arm& arm= solver.getSolveState().arms[sideIndex];
				bRead&= arm.bHandSeen && arm.bHandMeasured;
				bNeverMirrored&= !arm.bPalmMirrored;
				largestStep= std::max(largestStep, fabsf(AvatarRetargetMath::wrapAngle(arm.twist - lastTwist[sideIndex])));
				lastTwist[sideIndex]= arm.twist;
			}
		}
		check(bRead, "(j2) the raised hands are read on every frame of the turn");
		check(bNeverMirrored, "(j2) an upright hand's lean along the view ray never takes the mirror");
		check(largestStep < glm::radians(30.f), "(j2) a hand turned through edge-on turns the forearm smoothly");

		// The bars, on the desk pose, where the forearm points at the camera
		// and the gain is the lean times about 1.5: a lifted palm whose
		// finger axis leans along the view ray by the given amount
		User desk= makeNeutralUser();
		auto setDeskPalms= [&](float lean) {
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const glm::vec3 ray= glm::normalize(desk.wrist[sideIndex]);
				const glm::vec3 forward(1.f, 0.f, 0.f);
				const glm::vec3 across= glm::normalize(forward - ray * glm::dot(forward, ray));
				desk.palm[sideIndex]= makePalm(lean * ray + sqrtf(1.f - lean * lean) * across, glm::vec3(0.f, 0.f, -1.f));
			}
		};
		auto bothMirrored= [&](bool bExpected) {
			return solver.getSolveState().arms[0].bPalmMirrored == bExpected &&
				solver.getSolveState().arms[1].bPalmMirrored == bExpected;
		};
		auto gainBetween= [&](float low, float high) {
			bool bInside= true;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const float gain= solver.getSolveState().arms[sideIndex].palmMirrorGain;
				bInside&= gain > low && gain < high;
			}
			return bInside;
		};
		solver.reset();
		setDeskPalms(0.267f);
		for (int repeat= 0; repeat < 5; ++repeat)
			solveFrame(desk);
		check(gainBetween(0.3f, 0.5f) && bothMirrored(false),
			  "(j2) a gain between the bars, from the lifted palm, stays the lifted palm");
		setDeskPalms(0.5f);
		solveFrame(desk);
		check(gainBetween(0.5f, 2.f) && bothMirrored(true), "(j2) a gain above the enter bar takes the mirror");
		setDeskPalms(0.267f);
		solveFrame(desk);
		check(gainBetween(0.3f, 0.5f) && bothMirrored(true),
			  "(j2) a gain between the bars, from the mirror, stays the mirror");
		setDeskPalms(0.03f);
		solveFrame(desk);
		check(gainBetween(-0.3f, 0.3f) && bothMirrored(false),
			  "(j2) a gain under the leave bar returns to the lifted palm");
	}

	// (k) A hand slot the body model disagrees with is not the hand: beside
	// an unseen body wrist the arm rests once the hold passes, beside a seen
	// one the body wrist places the arm, and the slot's palm and fingers
	// stay out until it is back where the body says the wrist is
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		// The slot on something in the scene well away from the wrists, which
		// themselves sit below the frame
		User user= makeNeutralUser();
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			user.bWristSeen[sideIndex]= false;
			user.handPixelOffset[sideIndex]= glm::vec2(250.f, -200.f);
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
				user.fingers[sideIndex][finger].proximal= 1.f;
		}
		auto run= [&](User& frameUser, int firstFrame, int frameCount, AvatarPose& outPose) {
			for (int repeat= 0; repeat < frameCount; ++repeat)
			{
				const TrackingFrameResult frame=
					makeFrame(camera, frameUser, *rig.skeleton, timestampMs, firstFrame + repeat);
				solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, outPose);
				timestampMs+= 33.0;
			}
		};
		// The finger bend relative to the hand, wherever the arm put it
		auto indexBend= [](const AvatarPose& pose, int sideIndex) {
			const B hand= sideIndex == 0 ? B::LeftHand : B::RightHand;
			const B index= sideIndex == 0 ? B::LeftIndexProximal : B::RightIndexProximal;
			return rotationAngle(glm::inverse(pose.bones[(int)hand].deltaWorld) * pose.bones[(int)index].deltaWorld);
		};

		AvatarPose phantom;
		run(user, 50, 40, phantom);
		AvatarPose restReference;
		DirectRetarget::makeRestPose(*rig.skeleton, rig.config, restReference);
		bool bRested= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			bRested&= !solver.getArmChain(sideIndex).valid && solver.getSolveState().arms[sideIndex].bResting;
			bRested&= armAtPose(phantom, restReference, sideIndex);
		}
		check(bRested, "(k) a hand slot beside an unseen body wrist rests the arm and its fingers");

		// The body sees the wrists again, the slot still elsewhere
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= true;
		AvatarPose bodyPlaced;
		run(user, 100, 40, bodyPlaced);
		bool bPlaced= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::ArmChain& chain= solver.getArmChain(sideIndex);
			bPlaced&= chain.valid && glm::length(project(camera, chain.wrist) - project(camera, user.wrist[sideIndex])) < 1.f;
			bPlaced&= indexBend(bodyPlaced, sideIndex) < 1e-3f;
		}
		check(bPlaced, "(k) the body wrist places the arm past a disagreeing hand slot, its fingers at rest");

		// The slot back on the wrist: the hand is seen and its fingers bend
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.handPixelOffset[sideIndex]= glm::vec2(0.f);
		AvatarPose seen;
		run(user, 150, 3, seen);
		check(indexBend(seen, 0) > 0.5f && indexBend(seen, 1) > 0.5f,
			  "(k) a hand slot on the body wrist is the hand");
	}

	// (k2) Hand slots carrying the wrong side sit on the other arm's body
	// wrist. A second of that with no slot on its own wrist is reported as
	// swapped labels, one agreeing frame ends it, and wrists held together,
	// where a slot is beside both, are not a swap
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		User user= makeNeutralUser();
		auto putSlotsOnOppositeWrists= [&](User& frameUser) {
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
				frameUser.handPixelOffset[sideIndex]=
					project(camera, frameUser.wrist[1 - sideIndex]) - project(camera, frameUser.wrist[sideIndex]);
		};
		int frameIndex= 400;
		auto run= [&](User& frameUser, int frameCount) {
			AvatarPose pose;
			for (int repeat= 0; repeat < frameCount; ++repeat)
			{
				const TrackingFrameResult frame= makeFrame(camera, frameUser, *rig.skeleton, timestampMs, frameIndex++);
				solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, pose);
				timestampMs+= 33.0;
			}
		};
		auto swapped= [&]() { return solver.getSolveState().bHandednessSwapped; };

		putSlotsOnOppositeWrists(user);
		run(user, 20);
		check(!swapped(), "(k2) slots on the opposite wrists for under a second are not reported");
		run(user, 15);
		bool bBodyPlaced= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::ArmChain& chain= solver.getArmChain(sideIndex);
			bBodyPlaced&= chain.valid && !solver.getSolveState().arms[sideIndex].bHandSeen;
			bBodyPlaced&= glm::length(project(camera, chain.wrist) - project(camera, user.wrist[sideIndex])) < 1.f;
		}
		check(swapped(), "(k2) slots on the opposite wrists for over a second are reported as swapped");
		check(bBodyPlaced, "(k2) swapped slots are not the hands, and the body wrists place the arms");

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.handPixelOffset[sideIndex]= glm::vec2(0.f);
		run(user, 1);
		check(!swapped(), "(k2) one frame with a slot on its own wrist ends the report");

		// One hand in view, on the other arm's wrist
		putSlotsOnOppositeWrists(user);
		user.bHandTracked[1]= false;
		run(user, 35);
		check(swapped(), "(k2) a single slot on the opposite wrist is reported as swapped");
		user.bHandTracked[1]= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.handPixelOffset[sideIndex]= glm::vec2(0.f);
		run(user, 1);

		// The wrists drawn together in front of the chest: each slot is
		// beside both body wrists, its own among them
		const glm::vec3 wristMidpoint= 0.5f * (user.wrist[0] + user.wrist[1]);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.wrist[sideIndex]= wristMidpoint + 0.15f * (user.wrist[sideIndex] - wristMidpoint);
		putSlotsOnOppositeWrists(user);
		run(user, 40);
		check(!swapped(), "(k2) wrists held together are not a swap");
	}

	// (p) The wrist's entry gate and the earned hold: a body wrist whose
	// visibility flickers across the bar every other frame never poses the
	// arm, one steady above it gets in with its hand on the third frame, a
	// short sighting leaves an arm that rests within a few frames of the
	// wrist going, and a long one leaves an arm held for the full hold
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		User user= makeNeutralUser();
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
				user.fingers[sideIndex][finger].proximal= 1.f;
		auto runWith= [&](int firstFrame, int frameCount, AvatarPose& outPose, auto&& visibilityOf) {
			for (int repeat= 0; repeat < frameCount; ++repeat)
			{
				const float visibility= visibilityOf(repeat);
				user.wristVisibility[0]= user.wristVisibility[1]= visibility;
				const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, firstFrame + repeat);
				solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, outPose);
				timestampMs+= 33.0;
			}
		};
		auto armsRest= [&](const AvatarPose&) {
			bool bRest= true;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
				bRest&= !solver.getArmChain(sideIndex).valid && solver.getSolveState().arms[sideIndex].bResting;
			return bRest;
		};
		auto handsSeen= [&]() {
			const DirectRetarget::SolveState& state= solver.getSolveState();
			return state.arms[0].bHandSeen && state.arms[1].bHandSeen;
		};

		AvatarPose pose;
		// The neutral capture ran at full visibility: the gates are in, so a
		// stretch of unseen wrists past the hold empties them first
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= false;
		runWith(300, 20, pose, [](int) { return 1.f; });
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= true;
		check(armsRest(pose), "(p) wrists unseen past the hold rest the arms");

		runWith(320, 20, pose, [](int repeat) { return repeat % 2 == 0 ? 0.7f : 0.3f; });
		check(armsRest(pose), "(p) a wrist visibility flickering across the bar every other frame never poses the arm");

		runWith(340, 2, pose, [](int) { return 0.6f; });
		const bool bNotYet= armsRest(pose) && !handsSeen();
		runWith(342, 1, pose, [](int) { return 0.6f; });
		const bool bIn= !armsRest(pose) && handsSeen();
		check(bNotYet && bIn, "(p) a wrist steady above the bar gets in, with its hand, on the third frame");

		// Placed for two frames past its entry (about 70 ms), then gone: the
		// arm rests within a few frames, not the half second of the full hold
		runWith(343, 2, pose, [](int) { return 0.6f; });
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= false;
		runWith(345, 4, pose, [](int) { return 1.f; });
		check(armsRest(pose), "(p) a short sighting's arm rests within a few frames of the wrist going");

		// Placed for 40 frames, then gone for 10 (330 ms): still held
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= true;
		runWith(349, 40, pose, [](int) { return 0.6f; });
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= false;
		runWith(389, 10, pose, [](int) { return 1.f; });
		check(!armsRest(pose), "(p) a long sighting's arm is held through a gap within the full hold");

		// The wrists back for one frame while the arm is held: the hand is
		// seen on that frame, the entry gate being for an arm from rest
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= true;
		runWith(399, 1, pose, [](int) { return 0.6f; });
		check(!armsRest(pose) && handsSeen(), "(p) a held arm takes its wrist and hand back the frame they return");
	}

	// (q) The ease into rest: an arm placed well away from the rest pose,
	// its wrist then unseen past the hold, turns toward the rest pose frame
	// by frame and reaches it at the blend time, with the palm facing the
	// rig's rest normal and the fingers at the rest curl; a wrist returning
	// mid-ease places the solved chain the frame the entry gate admits it; a
	// blend of zero rests the arm on the first frame past the hold
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		// The rest forearm level, so the rest palm normal (down) needs no
		// projection clear of it
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			rig.config.sides[sideIndex].restWristOffset.z= rig.config.sides[sideIndex].restElbowOffset.z;
			rig.config.sides[sideIndex].restFingerCurlRadians= glm::radians(20.f);
		}
		AvatarPose restReference;
		DirectRetarget::makeRestPose(*rig.skeleton, rig.config, restReference);

		// Hands raised in front of the shoulders, palms to the camera
		auto makeRaisedUser= [&]() {
			User user= makeNeutralUser();
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				user.elbow[sideIndex]= user.shoulder[sideIndex] + glm::vec3(0.7071f, 0.f, 0.7071f) * kUpperArmLength;
				user.wrist[sideIndex]= user.elbow[sideIndex] + glm::vec3(0.f, 0.f, kForearmLength);
				user.palm[sideIndex]= makePalm(glm::vec3(0.f, 0.f, 1.f), glm::vec3(1.f, 0.f, 0.f));
			}
			return user;
		};
		auto upperAngleBetween= [](const AvatarPose& a, const AvatarPose& b, int sideIndex) {
			const B upper= sideIndex == 0 ? B::LeftUpperArm : B::RightUpperArm;
			return rotationAngle(glm::inverse(a.bones[(int)upper].deltaWorld) * b.bones[(int)upper].deltaWorld);
		};
		auto resting= [](const DirectStageSolver& solver, int sideIndex) {
			return solver.getSolveState().arms[sideIndex].bResting;
		};

		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);
		User user= makeRaisedUser();
		auto step= [&](DirectStageSolver& stepSolver, int frameIndex, AvatarPose& outPose) {
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, frameIndex);
			stepSolver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, outPose);
			timestampMs+= 33.0;
		};
		AvatarPose placed;
		for (int repeat= 0; repeat < 40; ++repeat)
			step(solver, 400 + repeat, placed);
		const bool bAway= upperAngleBetween(placed, restReference, 0) > glm::radians(30.f) &&
			upperAngleBetween(placed, restReference, 1) > glm::radians(30.f);

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= false;
		AvatarPose pose;
		int frameIndex= 440;
		while (!resting(solver, 0) && frameIndex < 500)
			step(solver, frameIndex++, pose);
		const int easeStartFrame= frameIndex;
		bool bMonotonic= resting(solver, 0) && resting(solver, 1);
		float fromPlaced[2]= {upperAngleBetween(placed, pose, 0), upperAngleBetween(placed, pose, 1)};
		float toRest[2]= {upperAngleBetween(pose, restReference, 0), upperAngleBetween(pose, restReference, 1)};
		bool bAtRest= false;
		for (int repeat= 0; repeat < 12 && !bAtRest; ++repeat)
		{
			step(solver, frameIndex++, pose);
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const float placedAngle= upperAngleBetween(placed, pose, sideIndex);
				const float restAngle= upperAngleBetween(pose, restReference, sideIndex);
				bMonotonic&= placedAngle >= fromPlaced[sideIndex] - 1e-4f && restAngle <= toRest[sideIndex] + 1e-4f;
				fromPlaced[sideIndex]= placedAngle;
				toRest[sideIndex]= restAngle;
			}
			bAtRest= armAtPose(pose, restReference, 0) && armAtPose(pose, restReference, 1);
		}
		const int easeFrames= frameIndex - easeStartFrame;
		check(bAway && bMonotonic, "(q) past the hold the arm turns toward the rest pose frame by frame");
		check(bAtRest && easeFrames >= 8 && easeFrames <= 10,
			  "(q) the arm reaches the rest pose at the blend time");

		bool bPalmDown= true;
		bool bCurled= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const B hand= sideIndex == 0 ? B::LeftHand : B::RightHand;
			const B index= sideIndex == 0 ? B::LeftIndexProximal : B::RightIndexProximal;
			const glm::vec3 palmNormal= pose.bones[(int)hand].deltaWorld *
				glm::vec3(rig.skeleton->getHand((eHandSide)sideIndex).palmFrameWorld[2]);
			bPalmDown&= angleBetween(palmNormal, rig.config.sides[sideIndex].restPalmNormal) < glm::radians(3.f);
			const float bend=
				rotationAngle(glm::inverse(pose.bones[(int)hand].deltaWorld) * pose.bones[(int)index].deltaWorld);
			bCurled&= std::fabs(bend - rig.config.sides[sideIndex].restFingerCurlRadians) < glm::radians(1.f);
		}
		check(bPalmDown, "(q) at rest the palm faces the rig's rest normal");
		check(bCurled, "(q) at rest the index base bends by the rest curl");

		// Back up, then unseen into the ease, then seen again mid-ease
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= true;
		for (int repeat= 0; repeat < 40; ++repeat)
			step(solver, frameIndex++, pose);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= false;
		while (!resting(solver, 0) && frameIndex < 700)
			step(solver, frameIndex++, pose);
		for (int repeat= 0; repeat < 3; ++repeat)
			step(solver, frameIndex++, pose);
		const bool bMidEase= resting(solver, 0) && solver.getSolveState().arms[0].restBlend < 1.f;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= true;
		// The arm is from rest, so the wrist passes the entry gate first
		for (int repeat= 0; repeat < 3; ++repeat)
			step(solver, frameIndex++, pose);
		bool bReturned= bMidEase;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::ArmChain& chain= solver.getArmChain(sideIndex);
			bReturned&= chain.valid && !resting(solver, sideIndex) &&
				glm::length(project(camera, chain.wrist) - project(camera, user.wrist[sideIndex])) < 1.f;
		}
		check(bReturned, "(q) a wrist returning mid-ease places the solved chain the frame the gate admits it");

		// No blend: the first frame past the hold is the rest pose
		rig.config.restBlendMs= 0.f;
		DirectStageSolver snapSolver;
		timestampMs= 0.0;
		captureNeutral(rig, camera, snapSolver, timestampMs);
		for (int repeat= 0; repeat < 40; ++repeat)
			step(snapSolver, 800 + repeat, pose);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			user.bWristSeen[sideIndex]= false;
		frameIndex= 840;
		while (!resting(snapSolver, 0) && frameIndex < 900)
			step(snapSolver, frameIndex++, pose);
		check(resting(snapSolver, 0) && armAtPose(pose, restReference, 0) && armAtPose(pose, restReference, 1),
			  "(q) with no blend the first frame past the hold is the rest pose");
	}

	// (l) The scorer: with the model's depth withheld, a seen elbow picks
	// the wrist distance whose elbow reprojects; the hand lift's depth scale
	// is measured against those frames, free of the lift's own pull; with
	// the elbow unseen the lift's distance places the wrist; and with no
	// depth cue at all the wrist keeps its distance
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		// The elbow forward and out, the forearm swung out and up: a bend
		// that no straight arm along a line of sight can mimic. The hands
		// are not tracked, so only the elbow's reprojection decides.
		User user= makeNeutralUser();
		user.bModelDepth= false;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const float sy= sideIndex == 0 ? 1.f : -1.f;
			user.bHandTracked[sideIndex]= false;
			user.elbow[sideIndex]= user.shoulder[sideIndex] + glm::normalize(glm::vec3(0.4f, sy * 0.6f, -0.7f)) * kUpperArmLength;
			user.wrist[sideIndex]= user.elbow[sideIndex] + glm::normalize(glm::vec3(0.5f, sy * 0.7f, 0.5f)) * kForearmLength;
		}
		auto run= [&](int firstFrame, int frameCount, AvatarPose& outPose) {
			for (int repeat= 0; repeat < frameCount; ++repeat)
			{
				const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, firstFrame + repeat);
				solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, outPose);
				timestampMs+= 33.0;
			}
		};
		auto depthWithin= [&](float fraction) {
			bool bWithin= true;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const float truth= glm::length(user.wrist[sideIndex]);
				bWithin&= nearlyEqual(glm::length(solver.getArmChain(sideIndex).wrist), truth, fraction * truth);
			}
			return bWithin;
		};

		AvatarPose pose;
		solver.reset();
		run(200, 3, pose);
		bool bElbowPicks= depthWithin(0.05f);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			bElbowPicks&= angleBetween(posedArm(*rig.skeleton, pose, sideIndex).forearm,
									   user.wrist[sideIndex] - user.elbow[sideIndex]) < kDirectionToleranceRadians;
		check(bElbowPicks, "(l) a seen elbow picks the wrist distance whose elbow reprojects, without the model's depth");

		// The hands tracked with a lift that reads 1.3 times too far, for 40
		// frames of seen elbow past the hand gate's entry
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			user.bHandTracked[sideIndex]= true;
			user.pnpDepthScale[sideIndex]= 1.3f;
		}
		run(210, 40 + 3, pose);
		bool bScale= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::PnpScale scale= solver.getPnpScale(sideIndex);
			bScale&= scale.sampleCount >= 40 && nearlyEqual(scale.scale, 1.f / 1.3f, 0.05f / 1.3f);
		}
		check(bScale, "(l) the lift's depth scale converges on the measured ratio over seen-elbow frames");

		// The elbow drops out of view and the wrist moves, the lift reading
		// its true distance: the wrist lands on the new ray at the lift's
		// distance, a different one from the distance it held
		const float heldDepth= glm::length(user.wrist[0]);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			user.bElbowSeen[sideIndex]= false;
			user.pnpDepthScale[sideIndex]= 1.f;
			user.wrist[sideIndex]= user.elbow[sideIndex] + glm::normalize(glm::vec3(0.6f, 0.f, -0.8f)) * kForearmLength;
		}
		run(260, 80, pose);
		bool bLift= std::fabs(glm::length(user.wrist[0]) - heldDepth) > 0.05f * heldDepth;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::ArmChain& chain= solver.getArmChain(sideIndex);
			const float truth= glm::length(user.wrist[sideIndex]);
			bLift&= chain.valid && nearlyEqual(glm::length(chain.wrist), truth, 0.05f * truth);
			bLift&= glm::length(project(camera, chain.wrist) - project(camera, user.wrist[sideIndex])) < 1.f;
		}
		check(bLift, "(l) with the elbow unseen the lift's distance places the wrist on its new ray");

		// The hands drop out too and the wrist moves again with nothing to
		// say how far: it keeps its distance along the new ray
		const float liftDepth= glm::length(user.wrist[0]);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			user.bHandTracked[sideIndex]= false;
			user.wrist[sideIndex]= user.elbow[sideIndex] + glm::normalize(glm::vec3(0.3f, 0.f, -0.95f)) * kForearmLength;
		}
		run(340, 40, pose);
		bool bHeld= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::ArmChain& chain= solver.getArmChain(sideIndex);
			bHeld&= chain.valid && nearlyEqual(glm::length(chain.wrist), liftDepth, 0.05f * liftDepth);
			bHeld&= glm::length(project(camera, chain.wrist) - project(camera, user.wrist[sideIndex])) < 1.f;
		}
		check(bHeld, "(l) with the elbow unseen and no depth cue the wrist keeps its distance on its new ray");
	}

	// (m) The palm hold: a one-frame palm roll in a steady sequence leaves
	// the forearm where it was, the same roll held for three frames is
	// adopted by the third
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		const User steady= makeNeutralUser();
		User rolled= steady;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const glm::quat roll= glm::angleAxis(glm::radians(90.f), glm::vec3(1.f, 0.f, 0.f));
			rolled.palm[sideIndex]= makePalm(glm::vec3(1.f, 0.f, 0.f), roll * glm::vec3(0.f, 0.f, -1.f));
		}
		auto run= [&](const User& user, int firstFrame, int frameCount, AvatarPose& outPose) {
			for (int repeat= 0; repeat < frameCount; ++repeat)
			{
				const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, firstFrame + repeat);
				solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, outPose);
				timestampMs+= 33.0;
			}
		};
		auto forearmsWithin= [&](const AvatarPose& a, const AvatarPose& b, float toleranceRadians) {
			bool bWithin= true;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const B lower= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
				bWithin&= nearlySameRotation(a.bones[(int)lower].deltaWorld, b.bones[(int)lower].deltaWorld, toleranceRadians);
			}
			return bWithin;
		};

		AvatarPose steadyPose;
		run(steady, 300, 10, steadyPose);
		AvatarPose glitchPose;
		run(rolled, 310, 1, glitchPose);
		AvatarPose afterPose;
		run(steady, 311, 1, afterPose);
		check(forearmsWithin(steadyPose, glitchPose, glm::radians(1.f)) && forearmsWithin(steadyPose, afterPose, glm::radians(1.f)),
			  "(m) a one-frame palm roll is held out of the forearm");

		AvatarPose adoptedPose;
		run(rolled, 320, 3, adoptedPose);
		check(!forearmsWithin(steadyPose, adoptedPose, glm::radians(45.f)),
			  "(m) a palm roll that persists three frames is adopted");
	}

	// (n) The filter split: a wrist pixel jittered by 2 px per frame at a
	// fixed depth leaves the chain's depth within 1 percent of steady
	{
		const Camera camera= makeCamera(kTiltDegrees);
		Rig rig= makeRig(kTiltDegrees);
		DirectStageSolver solver;
		double timestampMs= 0.0;
		captureNeutral(rig, camera, solver, timestampMs);

		User user= makeNeutralUser();
		AvatarPose pose;
		float minDepth[2]= {1e9f, 1e9f};
		float maxDepth[2]= {0.f, 0.f};
		for (int step= 0; step < 60; ++step)
		{
			const float jitter= (step % 2) == 0 ? 2.f : -2.f;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
				user.handPixelOffset[sideIndex]= glm::vec2(0.f, jitter);
			const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 400 + step);
			solver.solve(frame, camera.model, noPhone, timestampMs, *rig.skeleton, rig.config, pose);
			timestampMs+= 33.0;
			if (step < 30)
				continue;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const float depth= glm::length(solver.getArmChain(sideIndex).wrist);
				minDepth[sideIndex]= std::min(minDepth[sideIndex], depth);
				maxDepth[sideIndex]= std::max(maxDepth[sideIndex], depth);
			}
		}
		bool bSteady= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const float truth= glm::length(user.wrist[sideIndex]);
			bSteady&= (maxDepth[sideIndex] - minDepth[sideIndex]) < 0.01f * truth;
			bSteady&= nearlyEqual(0.5f * (maxDepth[sideIndex] + minDepth[sideIndex]), truth, 0.02f * truth);
		}
		check(bSteady, "(n) a wrist pixel jittered 2 px per frame leaves the chain's depth within 1 percent");
	}

	// (r) The rig's joint limits: one input with the upper body shifted to
	// the user's left (a lean), the phone head turned 50 degrees and both
	// wrists bent 30 degrees, solved under different limits. The default
	// limits leave it as no limit does, bit for bit. A lower limit clamps
	// its joint onto the same axis at the limit, the other joints as at the
	// default. A limit of zero locks the joint at its rest.
	{
		const Camera camera= makeCamera(kTiltDegrees);
		User user= makeNeutralUser();
		const glm::vec3 shift(0.f, 0.12f, 0.f);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			user.shoulder[sideIndex]+= shift;
			user.elbow[sideIndex]+= shift;
			user.wrist[sideIndex]+= shift;
			user.ear[sideIndex]+= shift;
			const glm::quat bend= glm::angleAxis(glm::radians(-30.f), glm::vec3(0.f, 1.f, 0.f));
			user.palm[sideIndex]= makePalm(bend * glm::vec3(1.f, 0.f, 0.f), bend * glm::vec3(0.f, 0.f, -1.f));
		}
		user.nose+= shift;
		TrackingFrameResult::HeadPose phone;
		phone.valid= true;
		phone.orientationWorld= glm::angleAxis(glm::radians(50.f), glm::vec3(0.f, 0.f, 1.f));

		struct Limited
		{
			AvatarPose pose;
			DirectRetarget::SolveState state;
		};
		auto solveUnder= [&](float headDegrees, float spineDegrees, float wristDegrees) {
			Rig rig= makeRig(kTiltDegrees);
			rig.config.headLimitRadians= glm::radians(headDegrees);
			rig.config.spineLimitRadians= glm::radians(spineDegrees);
			rig.config.wristLimitRadians= {glm::radians(wristDegrees), glm::radians(wristDegrees)};
			DirectStageSolver solver;
			double timestampMs= 0.0;
			captureNeutral(rig, camera, solver, timestampMs);
			Limited out;
			for (int repeat= 0; repeat < 10; ++repeat)
			{
				const TrackingFrameResult frame= makeFrame(camera, user, *rig.skeleton, timestampMs, 200 + repeat);
				solver.solve(frame, camera.model, phone, timestampMs, *rig.skeleton, rig.config, out.pose);
				timestampMs+= 33.0;
			}
			out.state= solver.getSolveState();
			return out;
		};
		// The default rig's limits, read from the rig settings in degrees
		const AvatarRigSettings defaultRig;
		const float headDefault= defaultRig.headLimitDegrees;
		const float spineDefault= defaultRig.spineLimitDegrees;
		const float wristDefault= defaultRig.sides[0].wristLimitDegrees;
		const Limited atDefault= solveUnder(headDefault, spineDefault, wristDefault);
		const Limited unlimited= solveUnder(180.f, 180.f, 180.f);
		check(samePose(atDefault.pose, unlimited.pose),
			  "(r) inside the default limits the pose is the one with no limit, bit for bit");

		// The relative rotation the wrist took: the hand in the forearm
		// against the avatar's rest hand in the forearm
		const Rig shapeRig= makeRig(kTiltDegrees);
		auto wristTaken= [&](const AvatarPose& pose, int sideIndex) {
			return glm::inverse(restHandInForearm(*shapeRig.skeleton, sideIndex)) *
				posedArm(*shapeRig.skeleton, pose, sideIndex).handInForearm;
		};
		// A rotation clamped to a limit: the same axis at the limit's angle
		auto clampedTo= [&](const glm::quat& rotation, float limitDegrees) {
			return glm::slerp(glm::quat(1.f, 0.f, 0.f, 0.f), rotation, glm::radians(limitDegrees) / rotationAngle(rotation));
		};
		const float tolerance= 1e-3f;

		// The head: the phone's 50 degrees inside the default, clamped to 20
		const glm::quat headFree= atDefault.pose.bones[(int)B::Head].deltaWorld;
		const Limited headLimited= solveUnder(20.f, spineDefault, wristDefault);
		const glm::quat headClamped= headLimited.pose.bones[(int)B::Head].deltaWorld;
		AvatarPose headRestored= headLimited.pose;
		headRestored.bones[(int)B::Head]= atDefault.pose.bones[(int)B::Head];
		check(nearlySameRotation(headFree, phone.orientationWorld, tolerance) &&
				  nearlyEqual(rotationAngle(headClamped), glm::radians(20.f), tolerance) &&
				  nearlySameRotation(headClamped, clampedTo(headFree, 20.f), tolerance) &&
				  samePose(headRestored, atDefault.pose),
			  "(r) a head limit below the measured head clamps the head to it, the rest of the pose unchanged");

		// The spine: the lean, measured between the two limits, clamped to 6
		const glm::quat spineFree= atDefault.pose.bones[(int)B::Spine].deltaWorld;
		const float leanRadians= rotationAngle(spineFree);
		const Limited spineLimited= solveUnder(headDefault, 6.f, wristDefault);
		const glm::quat spineClamped= spineLimited.pose.bones[(int)B::Spine].deltaWorld;
		check(leanRadians > glm::radians(10.f) && leanRadians < glm::radians(spineDefault - 10.f) &&
				  nearlyEqual(rotationAngle(spineClamped), glm::radians(6.f), tolerance) &&
				  nearlySameRotation(spineClamped, clampedTo(spineFree, 6.f), tolerance) &&
				  nearlySameRotation(spineLimited.pose.bones[(int)B::Neck].deltaWorld, spineClamped, tolerance) &&
				  nearlySameRotation(spineLimited.pose.bones[(int)B::Head].deltaWorld, headFree, tolerance),
			  "(r) a spine limit below the measured lean clamps the spine and the bones it carries to it");

		// The wrists: the measured bend inside the default, clamped to 15
		const Limited wristLimited= solveUnder(headDefault, spineDefault, 15.f);
		bool bWristFree= true;
		bool bWristClamped= true;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const float measured= rotationAngle(atDefault.state.arms[sideIndex].wristRotation);
			const glm::quat free= wristTaken(atDefault.pose, sideIndex);
			const glm::quat clamped= wristTaken(wristLimited.pose, sideIndex);
			bWristFree&= measured > glm::radians(20.f) && measured < glm::radians(wristDefault) &&
				nearlyEqual(rotationAngle(free), measured, tolerance);
			bWristClamped&= nearlyEqual(rotationAngle(clamped), glm::radians(15.f), tolerance) &&
				nearlySameRotation(clamped, clampedTo(free, 15.f), tolerance);
			const B lower= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
			bWristClamped&= nearlySameRotation(wristLimited.pose.bones[(int)lower].deltaWorld,
											   atDefault.pose.bones[(int)lower].deltaWorld, tolerance);
		}
		check(bWristFree, "(r) the measured wrist bend inside the default limit poses as measured");
		check(bWristClamped, "(r) a wrist limit below the measured bend clamps the wrist to it, the forearm unchanged");

		// Zero: every joint locked at its rest
		const Limited locked= solveUnder(0.f, 0.f, 0.f);
		bool bLocked= locked.pose.bones[(int)B::Head].present && locked.pose.bones[(int)B::Spine].present &&
			rotationAngle(locked.pose.bones[(int)B::Head].deltaWorld) < tolerance &&
			rotationAngle(locked.pose.bones[(int)B::Spine].deltaWorld) < tolerance &&
			rotationAngle(locked.pose.bones[(int)B::Neck].deltaWorld) < tolerance;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			bLocked&= rotationAngle(wristTaken(locked.pose, sideIndex)) < tolerance;
		check(bLocked, "(r) a limit of zero locks the head, the spine and the wrists at their rest");
	}

	// (o) The vec2 One Euro filter: initialized on its first sample, then
	// each component the scalar filter's
	{
		OneEuroFilterVec2 filter;
		filter.configure(1.f, 0.f, 1.f);
		OneEuroFilter scalar;
		scalar.configure(1.f, 0.f, 1.f);
		const glm::vec2 first= filter.filter(glm::vec2(1.f, 2.f), 0.f);
		scalar.filter(2.f, 0.f);
		const glm::vec2 second= filter.filter(glm::vec2(3.f, 4.f), 0.033f);
		const float scalarSecond= scalar.filter(4.f, 0.033f);
		check(glm::length(first - glm::vec2(1.f, 2.f)) < 1e-6f && second.x > 1.f && second.x < 3.f &&
				  nearlyEqual(second.y, scalarSecond, 1e-6f),
			  "(o) the vec2 One Euro filter runs the scalar filter per component");
	}

	// (i) The config section round-trips through the project JSON
	{
		AppConfig config;
		config.direct.enabled= true;
		config.direct.cameraIndex= 2;
		config.direct.cameraTiltDegrees= 12.5f;
		config.direct.neutralPresent= true;
		config.direct.neutralTorso= glm::angleAxis(0.3f, glm::vec3(0.f, 0.f, 1.f));
		config.direct.neutralShoulderMidpoint= glm::vec3(-0.8f, 0.01f, -0.2f);
		config.direct.neutralHead= glm::angleAxis(-0.2f, glm::vec3(0.f, 1.f, 0.f));
		const std::string text= config.toJsonString();
		AppConfig loaded;
		const bool bLoaded= loaded.loadFromJsonString(text);
		const DirectConfig& direct= loaded.direct;
		check(bLoaded && direct.enabled && direct.cameraIndex == 2 && nearlyEqual(direct.cameraTiltDegrees, 12.5f, 1e-5f) &&
				  direct.neutralPresent && nearlySameRotation(direct.neutralTorso, config.direct.neutralTorso, 1e-5f) &&
				  glm::length(direct.neutralShoulderMidpoint - config.direct.neutralShoulderMidpoint) < 1e-5f &&
				  nearlySameRotation(direct.neutralHead, config.direct.neutralHead, 1e-5f),
			  "(i) the direct section round-trips through the project JSON");
		AppConfig fresh;
		check(nearlyEqual(fresh.direct.cameraTiltDegrees, 10.f, 1e-5f) &&
				  fresh.loadFromJsonString("{}") && nearlyEqual(fresh.direct.cameraTiltDegrees, 10.f, 1e-5f),
			  "(i) a fresh config tilts the camera 10 degrees down, with or without a direct section");

		AvatarRigSettings rig;
		rig.sides[1].curlGain= 1.5f;
		const DirectRetargetConfig solverConfig= makeDirectRetargetConfig(loaded, rig);
		check(nearlyEqual(solverConfig.cameraTiltRadians, glm::radians(12.5f), 1e-5f) && solverConfig.neutral.present &&
				  nearlyEqual(solverConfig.sides[1].curlGain, 1.5f, 1e-5f) &&
				  nearlyEqual(solverConfig.userShoulderWidthMeters, loaded.body.shoulderWidthMeters, 1e-5f) &&
				  nearlyEqual(solverConfig.userUpperArmLengthMeters,
							  loaded.body.shoulderWidthMeters * loaded.body.upperArmPerShoulderWidth, 1e-5f) &&
				  nearlyEqual(solverConfig.userForearmLengthMeters, loaded.body.forearmLengthMeters, 1e-5f),
			  "(i) makeDirectRetargetConfig carries the section, the body lengths and the rig onto the solver");
	}

	MIKAN_LOG_INFO("test-direct-retarget") << (failures == 0 ? "ALL PASS" : "FAILURES") << " (" << failures << ")";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-direct-retarget",
					"Direct retarget: camera world, neutral capture, the wrist-depth scorer, the lift's scale, the palm hold, hold and the ease into rest, determinism",
					eTestCategory::SelfTest, runDirectRetargetTest);
