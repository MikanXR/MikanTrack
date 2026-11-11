#pragma once

#include <array>
#include <string>

#include "AvatarMapFigure.h"
#include "ExprParameterField.h" // ExprParameterLiveValue
#include "RetargetStage.h"      // RetargetElbowWeight

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

// The Retarget stage's sections, which the Inspector hosts: which VRM is
// loaded and what the loader read from it, its placement, the humanoid map
// with its per-bone overrides (Mapping), a preview pose to tune against, and
// the chains, one at a time: each chain's rig corrections and the project's
// chain settings. The face profiles are the Profile stage's (see
// AvatarFaceTab). Owns the preview pose, the elbow gizmo toggles, the
// mapping view's selection and the chain selection.
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
	// Whether an arm's elbow hint is shown as a draggable gizmo in the scene
	bool getShowElbowGizmo(int sideIndex) const { return m_bShowElbowGizmo[sideIndex]; }

	// What each arm's elbow weight evaluated to on the frame the scene shows,
	// or null when nothing was solved; the elbow weight fields draw it
	void setShownElbowWeights(const std::array<RetargetElbowWeight, 2>* weights);
	// The setting's live value as the field draws it
	static ExprParameterLiveValue toLiveValue(const RetargetElbowWeight& weight);
	const std::array<RetargetElbowWeight, 2>& getShownElbowWeights() const { return m_shownElbowWeights; }

private:
	void drawBoneRow(eHumanoidBone bone, const std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT>& states);
	std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT> computeDotStates() const;

	// A model and its skeleton are loaded
	bool hasAvatar() const;
	// The direct pipeline poses the avatar: a direct project with a model.
	// Without a model the position pipeline's stream runs in either shape.
	bool isDirectPipeline() const;
	void drawChainRow(bool bAvatar);
	void drawTorsoChain();
	void drawHeadChain();
	void drawArmChain(int sideIndex, bool bAvatar);
	// The arm's loss controls the pipeline performs, edited in chains;
	// true when anything changed
	bool drawArmLoss(int sideIndex, bool bDirect, RetargetChains& chains);
	void drawHandChain(int sideIndex);

	App* m_app;
	AppConfig* m_config;

	AvatarMapFigure m_figure;
	eHumanoidBone m_selectedBone= eHumanoidBone::Count;
	bool m_bScrollToSelected= false;
	char m_nodeFilter[64]= {};

	ePreviewPose m_previewPose= ePreviewPose::Live;
	bool m_bShowElbowGizmo[2]= {false, false};
	eRetargetChain m_selectedChain= eRetargetChain::LeftArm;
	std::array<RetargetElbowWeight, 2> m_shownElbowWeights{};
};
