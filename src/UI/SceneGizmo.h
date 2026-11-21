#pragma once

#include <array>
#include <string>
#include <vector>

#include "glm/ext/quaternion_float.hpp"
#include "glm/ext/vector_float3.hpp"

#include "AvatarPanel.h" // eRetargetChain, eSceneGizmoValue, SceneGizmoTarget
#include "AvatarRetarget.h"
#include "AvatarRig.h"
#include "RetargetChains.h"

class AvatarSkeleton;
struct CustomPose;

// The scene's transform gizmo, apart from its drawing: which of a retarget
// chain's settings it edits, where each one's handle sits on the posed
// avatar, and how a dragged handle becomes the stored value again. No ImGui
// and no GL, so the self tests run it.

// The value's name in automation: elbowHint, goalOffset, restElbow,
// restWrist, trim, poseWrist, poseElbow, posePalm, poseHead
const char* getSceneGizmoValueId(eSceneGizmoValue value);
bool parseSceneGizmoValueId(const std::string& id, eSceneGizmoValue& outValue);
// The localization key of the value's row in the chain's settings, which
// names its handle in the scene
const char* getSceneGizmoValueLabelKey(const SceneGizmoTarget& target);

// What a handle's gizmo does
enum class eSceneGizmoKind : int
{
	// Moves a point along the rooted torso frame's axes
	Translate,
	// Turns a frame about its three axes
	Rotate,
	// Turns a frame about its X axis alone
	RotateX,
};

// How a turned trim handle becomes the trim
enum class eSceneGizmoTrimRule : int
{
	// The handle is the posed bone's frame, which the solve turns against
	// the trim, so the trim takes the turn's inverse and the bone follows
	// the ring: the head, and a hand in the position pipeline
	BoneFollows,
	// The handle is the trimmed frame itself, the axes the direct pipeline
	// applies the measured wrist rotation about, so the trim takes the turn
	AxesTurn,
	// The handle's X axis runs along the forearm, and the trim's x takes the
	// angle turned about it
	ForearmRoll,
	// Not a trim: the handle is a frame the custom pose stores as Euler
	// degrees in baseFrameWorld, and the value takes the turned frame
	StoredFrame,
};

// One setting's handle on the posed avatar, world space
struct SceneGizmoHandle
{
	SceneGizmoTarget target;
	eSceneGizmoKind kind= eSceneGizmoKind::Translate;
	eSceneGizmoTrimRule trimRule= eSceneGizmoTrimRule::BoneFollows;
	// Where the handle is and the frame its axes run along
	glm::vec3 positionWorld{0.f};
	glm::quat frameWorld{1.f, 0.f, 0.f, 0.f};
	// A translated value is the handle's offset from this point in the
	// handle's frame, in units of offsetScale meters (1 for a value in
	// meters, the arm's length for one in arm lengths), and bDrawAnchorLine
	// joins the two in the scene
	glm::vec3 anchorWorld{0.f};
	float offsetScale= 1.f;
	bool bDrawAnchorLine= false;
	// The frame a StoredFrame value is stored in
	glm::quat baseFrameWorld{1.f, 0.f, 0.f, 0.f};
};

// What the handles are built from: the loaded avatar as the shown frame
// posed it, and the settings the frame was posed with
struct SceneGizmoInputs
{
	const AvatarSkeleton* skeleton= nullptr;
	const AvatarPose* pose= nullptr;
	const AvatarRigSettings* rig= nullptr;
	const RetargetChains* chains= nullptr;
	// The shown frame's wrist goals; null, or an arm without one, has no
	// goal offset handle
	const AvatarRetargetGoals* goals= nullptr;
	bool bDirect= false;
	// The mode whose settings get handles
	eChainMode mode= eChainMode::Tracking;
	// The custom pose the frame was posed from, for the Custom pose mode's
	// handles; null, as for the built-in pose, has none
	const CustomPose* customPose= nullptr;
};

// What an edit writes to, and which of them it changed. A destination that is
// null takes no edit.
struct SceneGizmoEdit
{
	AvatarRigSettings* rig= nullptr;
	RetargetChains* chains= nullptr;
	CustomPose* pose= nullptr;
	bool bRigChanged= false;
	bool bChainsChanged= false;
	bool bPoseChanged= false;
};

// The handles of the chain's settings the mode shows, in the order of
// eSceneGizmoValue. Tracking: an arm's elbow hint, its goal offset while the
// arm maps toward a goal and the frame has one, and the trims. Rest pose: an
// arm's rest elbow and rest wrist. Custom pose: an arm's wrist, its elbow
// when the pose gives one and its palm, at the palm's center so it stands
// clear of the wrist's, while the arm is on, and the head.
void buildSceneGizmoHandles(const SceneGizmoInputs& inputs, eRetargetChain chain,
							std::vector<SceneGizmoHandle>& outHandles);

// Takes a handle moved to draggedPositionWorld, or turned to
// draggedFrameWorld, back into the setting it stands for, and sets the flag
// of what it changed
void applySceneGizmoEdit(const SceneGizmoHandle& handle, const glm::vec3& draggedPositionWorld,
						 const glm::quat& draggedFrameWorld, SceneGizmoEdit& ioEdit);

// The stored value a target names: an offset in its own units (meters, or
// arm lengths for the pose's), or a rotation's Euler degrees. False for a
// target the chain does not have, or one whose destination is null.
bool getSceneGizmoValue(const SceneGizmoTarget& target, const AvatarRigSettings& rig, const RetargetChains& chains,
						const CustomPose* pose, glm::vec3& outValue);
bool setSceneGizmoValue(const SceneGizmoTarget& target, const glm::vec3& value, SceneGizmoEdit& ioEdit);
