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

// The scene's transform gizmo, apart from its drawing: which of a retarget
// chain's settings it edits, where each one's handle sits on the posed
// avatar, and how a dragged handle becomes the stored value again. No ImGui
// and no GL, so the self tests run it.

// The value's name in automation: elbowHint, goalOffset, restElbow,
// restWrist, trim
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
	// A translated value is the handle's offset from this point in the rooted
	// torso frame, and bDrawAnchorLine joins the two in the scene
	glm::vec3 anchorWorld{0.f};
	bool bDrawAnchorLine= false;
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
};

// The handles of the chain's settings the mode shows, in the order of
// eSceneGizmoValue. Tracking: an arm's elbow hint, its goal offset while the
// arm maps toward a goal and the frame has one, and the trims. Rest pose: an
// arm's rest elbow and rest wrist. Custom pose: none.
void buildSceneGizmoHandles(const SceneGizmoInputs& inputs, eRetargetChain chain,
							std::vector<SceneGizmoHandle>& outHandles);

// Takes a handle moved to draggedPositionWorld, or turned to
// draggedFrameWorld, back into the setting it stands for. rootRotationWorld
// is the pose's the handle was built from. Sets the flag of what it changed.
void applySceneGizmoEdit(const SceneGizmoHandle& handle, const glm::vec3& draggedPositionWorld,
						 const glm::quat& draggedFrameWorld, const glm::quat& rootRotationWorld, AvatarRigSettings& ioRig,
						 RetargetChains& ioChains, bool& outRigChanged, bool& outChainsChanged);

// The stored value a target names: meters in the rooted torso frame, or a
// trim's Euler degrees. False for a target the chain does not have.
bool getSceneGizmoValue(const SceneGizmoTarget& target, const AvatarRigSettings& rig, const RetargetChains& chains,
						glm::vec3& outValue);
bool setSceneGizmoValue(const SceneGizmoTarget& target, const glm::vec3& value, AvatarRigSettings& ioRig,
						RetargetChains& ioChains, bool& outRigChanged, bool& outChainsChanged);
