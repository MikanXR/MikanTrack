#pragma once

#include <array>
#include <string>

#include "AvatarMapFigure.h"

class App;
class AppConfig;

// The avatar panel: which VRM is loaded and what the loader read from it
// (Model), the humanoid map with its per-bone overrides and rotation trims
// (Mapping), and the elbow hints and hand tweaks plus a preview pose to tune
// them against (Retarget). The face profiles are the Profile stage's (see
// AvatarFaceTab).
// Loading goes through App (the model is main-thread, GL-adjacent state);
// placement edits go to the persisted AvatarConfig and rig edits to
// App::setAvatarRig.
class AvatarPanel
{
public:
	AvatarPanel(App* app, AppConfig* config);

	void draw(bool* pOpen);
	// The panel's contents without its window, for a host window
	void drawBody();
	// Per-frame state, run every frame whether or not the panel is drawn:
	// the preview pose falls back to Live while no model is loaded
	void update();

	// The panel's sections, each drawable in a host window. The tabs and the
	// model summary need a loaded model; drawModelSummary and drawMappingTab
	// draw nothing without one.
	void drawLoadControls();
	void drawModelTab();
	void drawModelSummary();
	void drawPlacement();
	void drawMappingTab();
	void drawRetargetTab();
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
