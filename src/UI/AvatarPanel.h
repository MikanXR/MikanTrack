#pragma once

#include <array>
#include <string>

#include "AvatarMapFigure.h"

class App;
class AppConfig;

// The Retarget stage's sections, which the Inspector hosts: which VRM is
// loaded and what the loader read from it, its placement, the humanoid map
// with its per-bone overrides and rotation trims (Mapping), the elbow hints
// and hand tweaks plus a preview pose to tune them against (Retarget), and
// the rest blend. The face profiles are the Profile stage's (see
// AvatarFaceTab). Owns the preview pose, the elbow gizmo toggles and the
// mapping view's selection.
// Loading goes through App (the model is main-thread, GL-adjacent state);
// placement edits go to the persisted AvatarConfig and rig edits to
// App::setAvatarRig.
class AvatarPanel
{
public:
	AvatarPanel(App* app, AppConfig* config);

	// Per-frame state, run every frame whether or not a section is drawn:
	// the preview pose falls back to Live while no model is loaded
	void update();

	// What the load controls' buttons and the File menu's items run: the
	// open-file dialog and a load, and an unload, each persisting the
	// project's model path
	void loadAvatarFromDialog();
	void unloadAvatar();

	// The sections, each drawable in a host window. All but the load
	// controls need a loaded model; drawModelSummary and drawMappingTab draw
	// nothing without one.
	void drawLoadControls();
	void drawModelSummary();
	void drawPlacement();
	void drawMappingTab();
	// The preview pose and each arm's and hand's controls
	void drawRetargetTab();
	// How long a lost arm takes to ease into its rest pose
	void drawRestBlendControl();
	void drawArmControls(int sideIndex);
	void drawHandControls(int sideIndex);

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

private:
	void drawBoneRow(eHumanoidBone bone, const std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT>& states);
	std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT> computeDotStates() const;

	App* m_app;
	AppConfig* m_config;

	AvatarMapFigure m_figure;
	eHumanoidBone m_selectedBone= eHumanoidBone::Count;
	bool m_bScrollToSelected= false;
	char m_nodeFilter[64]= {};

	ePreviewPose m_previewPose= ePreviewPose::Live;
	bool m_bShowElbowGizmo[2]= {false, false};
};
