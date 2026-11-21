#pragma once

#include <array>
#include <string>

#include "AvatarMapFigure.h"
#include "ExprParameterField.h" // ExprParameterLiveValue
#include "RetargetStage.h"      // RetargetResult

class App;
class AppConfig;
struct RetargetChains;

// The chains of the avatar the Retarget stage's settings show one at a time,
// in the order of their row
enum class eRetargetChain
{
	Torso,
	Head,
	LeftArm,
	RightArm,
	LeftHand,
	RightHand,
	Count,
};

// A chain's setting the scene's gizmo edits. An arm has the first four and
// its forearm's roll trim, the head and a hand their frame trim, the torso
// none.
enum class eSceneGizmoValue : int
{
	None,
	// AvatarRigSettings::Side::elbowHintOffset
	ElbowHint,
	// RetargetChains::Arm::goalOffset
	GoalOffset,
	// AvatarRigSettings::Side::restElbowOffset and restWristOffset
	RestElbow,
	RestWrist,
	// AvatarRigSettings::rotationTrimDegrees of the chain's bone
	Trim,
	Count,
};

// One setting of one chain
struct SceneGizmoTarget
{
	eRetargetChain chain= eRetargetChain::Count;
	eSceneGizmoValue value= eSceneGizmoValue::None;

	bool isValid() const { return chain != eRetargetChain::Count && value != eSceneGizmoValue::None; }
	bool operator==(const SceneGizmoTarget& other) const { return chain == other.chain && value == other.value; }
	bool operator!=(const SceneGizmoTarget& other) const { return !(*this == other); }
};

// The Retarget stage's sections, which the Inspector hosts: which VRM is
// loaded and what the loader read from it, its placement, the humanoid map
// with its per-bone overrides (Mapping), a preview pose to tune against, and
// the chains, one at a time: each chain's rig corrections and the project's
// chain settings. The face profiles are the Profile stage's (see
// AvatarFaceTab). Owns the preview pose, the setting the scene's gizmo
// edits, the mapping view's selection and the chain selection.
// Loading goes through App (the model is main-thread, GL-adjacent state);
// placement edits go to the persisted AvatarConfig, rig edits to
// App::setAvatarRig and chain edits to App::setRetargetChains.
class AvatarPanel
{
public:
	AvatarPanel(App* app, AppConfig* config);

	// Per-frame state, run every frame whether or not a section is drawn:
	// the preview pose falls back to Live and the chain selection to the
	// left arm while no model is loaded
	void update();

	// What the load controls' buttons and the File menu's items run: the
	// open-file dialog and a load, and an unload, each persisting the
	// project's model path
	void loadAvatarFromDialog();
	void unloadAvatar();

	// The sections, each drawable in a host window, and twice in one frame.
	// All but the load controls and the chains need a loaded model;
	// drawModelSummary and drawMapping draw nothing without one.
	void drawLoadControls();
	void drawModelSummary();
	// The scene's avatar toggle, which both pipelines read
	void drawShowInScene();
	// Follow shoulders and the fixed root, the position retarget's alone
	void drawPlacement();
	// The view buttons, the figure, the reset, the filter and the bone table
	void drawMapping();
	void drawPreviewPose();
	// The chain row and the selected chain's settings. Without a model the
	// row holds the two arms and an arm shows its loss controls only, the
	// rules the stream applies to the measured-length skeleton.
	void drawChains();

	// The chain the settings show. A setter for whatever else picks a chain
	// (the row, automation); without a model only an arm can be selected.
	eRetargetChain getSelectedChain() const { return m_selectedChain; }
	// False, and no change, for a chain the loaded model cannot show
	bool setSelectedChain(eRetargetChain chain);
	// The chain's id, the key of its block in the project's chain settings
	// ("torso", "head", "leftArm", "rightArm", "leftHand", "rightHand")
	static const char* getChainId(eRetargetChain chain);
	static bool parseChainId(const std::string& id, eRetargetChain& outChain);

	// What the 3D scene's avatar is posed from. Rest and Demo are canned
	// frames for tuning the rig without cameras; the VMC output always
	// follows live tracking.
	enum class ePreviewPose
	{
		Live,
		Rest,
		Demo,
	};
	ePreviewPose getPreviewPose() const { return m_previewPose; }
	// The setting the scene's gizmo is attached to, one of the selected
	// chain's or none: a row's toggle and a click on a handle in the scene
	// both set it, and selecting another chain clears it
	const SceneGizmoTarget& getGizmoTarget() const { return m_gizmoTarget; }
	void setGizmoTarget(const SceneGizmoTarget& target);

	// The result the scene shows, or null when nothing was solved: what each
	// arm's parameters evaluated to, which the parameter fields draw, and the
	// position solve's wrist goals
	void setShownResult(const RetargetResult* result);
	// A parameter's live value as the field draws it
	static ExprParameterLiveValue toLiveValue(const RetargetParameterReadout& readout);
	const std::array<RetargetArmReadout, 2>& getShownArmParameters() const { return m_shownArmParameters; }
	const AvatarRetargetGoals& getShownGoals() const { return m_shownGoals; }

private:
	void drawBoneRow(eHumanoidBone bone, const std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT>& states);

	// A model and its skeleton are loaded
	bool hasAvatar() const;
	// The direct pipeline poses the avatar: a direct project with a model.
	// Without a model the position pipeline's stream runs in either shape.
	bool isDirectPipeline() const;
	void drawChainRow(bool bAvatar);
	void drawTorsoChain();
	void drawHeadChain();
	void drawArmChain(int sideIndex, bool bAvatar);
	// The arm's loss controls, edited in chains: the hold as the pipeline
	// performs it, then what follows it; true when anything changed
	bool drawArmLoss(int sideIndex, bool bDirect, bool bAvatar, RetargetChains& chains);
	void drawHandChain(int sideIndex);
	// The toggle ahead of a row the gizmo can edit, on the row's line
	void drawGizmoToggle(eSceneGizmoValue value);

	App* m_app;
	AppConfig* m_config;

	AvatarMapFigure m_figure;
	eHumanoidBone m_selectedBone= eHumanoidBone::Count;
	bool m_bScrollToSelected= false;
	char m_nodeFilter[64]= {};

	ePreviewPose m_previewPose= ePreviewPose::Live;
	SceneGizmoTarget m_gizmoTarget;
	eRetargetChain m_selectedChain= eRetargetChain::LeftArm;
	std::array<RetargetArmReadout, 2> m_shownArmParameters{};
	AvatarRetargetGoals m_shownGoals;
};
