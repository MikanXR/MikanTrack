#include "AvatarPanel.h"

#include <algorithm>
#include <cctype>
#include <optional>

#include "imgui.h"
#include "misc/cpp/imgui_stdlib.h"

#include "LocText.h"

#include "App.h"
#include "AppConfig.h"
#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "BodyPoseSolver.h"
#include "CustomPoseLibrary.h"
#include "FaceExprInputs.h"
#include "FaceProfileBinding.h"
#include "HandPoseModel.h"
#include "PathUtils.h"
#include "RetargetChains.h"
#include "RetargetStage.h"
#include "SourcePoseInputs.h"
#include "VrmLoader.h"

#include "tinyfiledialogs.h"

namespace
{
using B= eHumanoidBone;

const ImVec4 k_warningColor(1.f, 0.8f, 0.4f, 1.f);

// The chain row's labels and ids, by eRetargetChain. The ids are the keys of
// the chain blocks in the project's chain settings.
const char* const k_chainLabelKeys[(int)eRetargetChain::Count]= {
	"sourceStage.torso",
	"directStage.head",
	"avatarPanel.leftArmHeader",
	"avatarPanel.rightArmHeader",
	"avatarPanel.leftHandHeader",
	"avatarPanel.rightHandHeader",
};
const char* const k_modeIds[(int)eChainMode::Count]= {"tracking", "rest", "custom"};
const char* const k_modeLabelKeys[(int)eChainMode::Count]= {"chains.modeTracking", "chains.modeRest",
															  "chains.modeCustom"};
const char* const k_modeTooltipKeys[(int)eChainMode::Count]= {"chains.modeTrackingTooltip", "chains.modeRestTooltip",
																"chains.modeCustomTooltip"};
const char* const k_chainIds[(int)eRetargetChain::Count]= {
	"torso", "head", "leftArm", "rightArm", "leftHand", "rightHand",
};

bool isArmChain(eRetargetChain chain)
{
	return chain == eRetargetChain::LeftArm || chain == eRetargetChain::RightArm;
}

// A point or direction in the rooted torso frame (+X facing, +Y the avatar's
// left, +Z up) reflected onto the other side
glm::vec3 mirrorAcrossSagittal(const glm::vec3& v)
{
	return glm::vec3(v.x, -v.y, v.z);
}

void drawNote(const char* key)
{
	ImGui::PushTextWrapPos(0.f);
	ImGui::TextDisabled("%s", locText(key));
	ImGui::PopTextWrapPos();
}

void drawSubHeading(const char* key)
{
	ImGui::Spacing();
	ImGui::TextDisabled("%s", locText(key));
}

// A rotation trim from the rig, three axes on a frame-bearing bone (the head,
// a hand) or the roll in x on a forearm; true when it changed
bool drawTrimControl(AvatarRigSettings& rig, eHumanoidBone bone, const char* labelKey, bool bRollOnly)
{
	glm::vec3& trim= rig.rotationTrimDegrees[(int)bone];
	const bool bChanged= bRollOnly ? ImGui::DragFloat(locLabel(labelKey), &trim.x, 0.25f, -180.f, 180.f, "%.1f deg")
								   : ImGui::DragFloat3(locLabel(labelKey), &trim.x, 0.25f, -180.f, 180.f, "%.1f");
	ImGui::SetItemTooltip("%s", locText(bRollOnly ? "avatarPanel.rollTrimTooltip" : "avatarPanel.frameTrimTooltip"));
	return bChanged;
}

// A joint limit from the rig, degrees: zero locks the joint at its rest and
// 180 never clamps, since a rotation's short arc is at most a half turn. A
// typed value clamps too, since a negative limit would turn the joint the
// other way.
bool drawLimitControl(float& degrees, const char* labelKey, const char* tooltipKey)
{
	const bool bChanged=
		ImGui::SliderFloat(locLabel(labelKey), &degrees, 0.f, 180.f, "%.0f deg", ImGuiSliderFlags_AlwaysClamp);
	ImGui::SetItemTooltip("%s", locText(tooltipKey));
	return bChanged;
}

// What follows a chain's hold, as stored: freeze, the ease to the rest pose,
// or release; true when anything changed
bool drawLossAfter(RetargetChains::Loss& loss)
{
	struct Choice
	{
		eLossAfter after;
		const char* labelKey;
		const char* tooltipKey;
	};
	static const Choice k_choices[]= {
		{eLossAfter::Freeze, "chains.afterFreezeRadio", "chains.afterFreezeTooltip"},
		{eLossAfter::EaseToRest, "chains.afterEaseRadio", "chains.afterEaseTooltip"},
		{eLossAfter::Release, "chains.afterReleaseRadio", "chains.afterReleaseTooltip"},
	};
	bool bChanged= false;
	ImGui::TextUnformatted(locText("chains.thenLabel"));
	for (const Choice& choice : k_choices)
	{
		if (ImGui::RadioButton(locLabel(choice.labelKey), loss.after == choice.after) && loss.after != choice.after)
		{
			loss.after= choice.after;
			bChanged= true;
		}
		ImGui::SetItemTooltip("%s", locText(choice.tooltipKey));
	}
	return bChanged;
}

// How long a chain takes to ease into its rest pose, and for an arm back out
// of it; true when either changed. A chain that does not ease to rest says
// so, since the times then do nothing.
bool drawRestBlend(RetargetChains::Loss& loss, bool bArm)
{
	if (loss.after != eLossAfter::EaseToRest)
		drawNote("chains.restNotUsedNote");
	bool bChanged=
		ImGui::SliderFloat(locLabel("avatarPanel.restBlendSlider"), &loss.restBlendMs, 0.f, 1000.f, "%.0f ms");
	ImGui::SetItemTooltip("%s", locText("avatarPanel.restBlendTooltip"));
	if (bArm)
	{
		bChanged|= ImGui::SliderFloat(locLabel("avatarPanel.riseBlendSlider"), &loss.riseBlendMs, 0.f, 1000.f, "%.0f ms");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.riseBlendTooltip"));
	}
	return bChanged;
}

bool containsCaseInsensitive(const std::string& text, const char* filter)
{
	if (filter[0] == '\0')
		return true;
	const std::string needle(filter);
	auto it= std::search(text.begin(), text.end(), needle.begin(), needle.end(), [](char a, char b) {
		return std::tolower((unsigned char)a) == std::tolower((unsigned char)b);
	});
	return it != text.end();
}

const char* nodeName(const AvatarModel& model, int node)
{
	return node >= 0 && node < (int)model.nodes.size() ? model.nodes[node].name.c_str() : "";
}
} // namespace

AvatarPanel::AvatarPanel(App* app, AppConfig* config)
	: m_app(app)
	, m_config(config)
{
}

void AvatarPanel::update()
{
	if (m_app->getAvatarModel() == nullptr)
	{
		m_chainMode= eChainMode::Tracking;
		if (!isArmChain(m_selectedChain))
			m_selectedChain= eRetargetChain::LeftArm;
		m_gizmoTarget= SceneGizmoTarget();
	}
}

bool AvatarPanel::setSelectedChain(eRetargetChain chain)
{
	if (chain == eRetargetChain::Count || (!hasAvatar() && !isArmChain(chain)))
		return false;
	if (chain != m_selectedChain)
		m_gizmoTarget= SceneGizmoTarget();
	m_selectedChain= chain;
	m_bPoseFaceSelected= false;
	return true;
}

bool AvatarPanel::setChainMode(eChainMode mode)
{
	if (mode == eChainMode::Count || (mode != eChainMode::Tracking && !hasAvatar()))
		return false;
	// A setting's handle belongs to the mode that shows the setting
	if (mode != m_chainMode)
		m_gizmoTarget= SceneGizmoTarget();
	m_chainMode= mode;
	return true;
}

const char* AvatarPanel::getChainModeId(eChainMode mode)
{
	return mode < eChainMode::Count ? k_modeIds[(int)mode] : k_modeIds[0];
}

bool AvatarPanel::parseChainModeId(const std::string& id, eChainMode& outMode)
{
	for (int index= 0; index < (int)eChainMode::Count; ++index)
	{
		if (id == k_modeIds[index])
		{
			outMode= (eChainMode)index;
			return true;
		}
	}
	return false;
}

void AvatarPanel::setGizmoTarget(const SceneGizmoTarget& target)
{
	m_gizmoTarget= target.isValid() && target.chain == m_selectedChain ? target : SceneGizmoTarget();
}

void AvatarPanel::drawGizmoToggle(eSceneGizmoValue value)
{
	const SceneGizmoTarget target{m_selectedChain, value};
	const bool bAttached= m_gizmoTarget == target;
	ImGui::PushID((int)value);
	if (ImGui::RadioButton("##gizmo", bAttached))
		m_gizmoTarget= bAttached ? SceneGizmoTarget() : target;
	ImGui::PopID();
	ImGui::SetItemTooltip("%s", locText("avatarPanel.gizmoToggleTooltip"));
	// The row keeps its right edge
	ImGui::SameLine();
	ImGui::SetNextItemWidth(ImGui::CalcItemWidth() - ImGui::GetFrameHeight() - ImGui::GetStyle().ItemSpacing.x);
}

void AvatarPanel::setShownResult(const RetargetResult* result)
{
	m_shownArmParameters= result != nullptr ? result->armParameters : std::array<RetargetArmReadout, 2>{};
	m_shownGoals= result != nullptr ? result->goals : AvatarRetargetGoals();
}

ExprParameterLiveValue AvatarPanel::toLiveValue(const RetargetParameterReadout& readout)
{
	ExprParameterLiveValue live;
	live.bValid= readout.bEvaluated;
	live.value= readout.value;
	live.rawValue= readout.rawValue;
	live.bFromExpression= readout.bExpression && readout.bCompiled;
	return live;
}

const char* AvatarPanel::getChainId(eRetargetChain chain)
{
	return chain < eRetargetChain::Count ? k_chainIds[(int)chain] : "";
}

bool AvatarPanel::parseChainId(const std::string& id, eRetargetChain& outChain)
{
	for (int index= 0; index < (int)eRetargetChain::Count; ++index)
	{
		if (id == k_chainIds[index])
		{
			outChain= (eRetargetChain)index;
			return true;
		}
	}
	return false;
}

bool AvatarPanel::hasAvatar() const
{
	return m_app->getAvatarModel() != nullptr && m_app->getAvatarSkeleton() != nullptr;
}

bool AvatarPanel::isDirectPipeline() const
{
	return m_config->direct.enabled && hasAvatar();
}

void AvatarPanel::loadAvatarFromDialog()
{
	AvatarConfig& avatar= m_config->avatar;

	// Seed the dialog at the shipped samples the first time, at the current
	// model afterwards
	std::string defaultPath= (PathUtils::getModulePath() / "models" / "avatars" / "").string();
	if (!avatar.modelPath.empty())
		defaultPath= m_app->resolveAvatarPath(avatar.modelPath).string();

	const char* filterPatterns[]= {"*.vrm"};
	const char* selectedPath= tinyfd_openFileDialog(
		locText("avatarPanel.browseDialogTitle"), defaultPath.c_str(), 1, filterPatterns,
		locText("avatarPanel.browseDialogFilterDescription"), 0);
	if (selectedPath != nullptr)
	{
		const std::filesystem::path path= PathUtils::utf8ToPath(selectedPath);
		if (m_app->loadAvatar(path))
		{
			avatar.modelPath= PathUtils::pathToUtf8(path);
			m_config->markDirty();
			m_selectedBone= HUMANOID_BONE_NONE;
		}
	}
}

void AvatarPanel::unloadAvatar()
{
	m_app->clearAvatar();
	m_config->avatar.modelPath.clear();
	m_config->markDirty();
	m_selectedBone= HUMANOID_BONE_NONE;
}

void AvatarPanel::drawLoadControls()
{
	const AvatarConfig& avatar= m_config->avatar;

	if (ImGui::Button(locLabel("avatarPanel.browseButton")))
		loadAvatarFromDialog();
	ImGui::SameLine();
	if (ImGui::Button(locLabel("avatarPanel.unloadButton")))
		unloadAvatar();

	if (avatar.modelPath.empty())
	{
		ImGui::TextDisabled("%s", locText("avatarPanel.noModelText"));
	}
	else
	{
		const std::filesystem::path shown= PathUtils::utf8ToPath(avatar.modelPath);
		ImGui::TextWrapped("%s", PathUtils::createTrimmedPathString(shown, 60).c_str());
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", avatar.modelPath.c_str());
	}

	const std::string& error= m_app->getAvatarLoadError();
	if (!error.empty())
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), "%s", error.c_str());
}

void AvatarPanel::drawModelSummary()
{
	if (m_app->getAvatarModel() == nullptr || m_app->getAvatarSkeleton() == nullptr)
		return;
	const AvatarModel& model= *m_app->getAvatarModel();
	const AvatarSkeleton& skeleton= *m_app->getAvatarSkeleton();

	ImGui::Text("%s", locFormat("avatarPanel.modelNameFmt", model.meta.name.c_str(),
								VrmLoader::versionName(model.version))
						  .c_str());
	if (!model.meta.author.empty())
		ImGui::Text("%s", locFormat("avatarPanel.authorFmt", model.meta.author.c_str()).c_str());
	if (!model.meta.license.empty())
		ImGui::TextWrapped("%s", locFormat("avatarPanel.licenseFmt", model.meta.license.c_str()).c_str());
	ImGui::Text("%s", locFormat("avatarPanel.contentsFmt", (int)model.nodes.size(), (int)model.skins.size(),
								(int)model.triangleCount(), (int)model.images.size())
						  .c_str());

	int presentCount= 0;
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
		presentCount+= model.humanoidNodes[index] >= 0 ? 1 : 0;
	ImGui::Text("%s", locFormat("avatarPanel.bonesPresentFmt", presentCount, HUMANOID_BONE_COUNT).c_str());

	// The avatar's arm against the user's measured one: the ratio the hand
	// retarget will scale reach by
	const BodyDimensions user= makeBodyDimensions(*m_config);
	ImGui::Text("%s", locFormat("avatarPanel.armLengthsFmt",
								skeleton.getUpperArmLength(eHandSide::Left) * 100.f,
								skeleton.getForearmLength(eHandSide::Left) * 100.f,
								user.upperArmLengthMeters * 100.f, user.forearmLengthMeters * 100.f)
						  .c_str());
	ImGui::Text("%s", locFormat("avatarPanel.shoulderWidthFmt", skeleton.getShoulderWidth() * 100.f,
								user.shoulderWidthMeters * 100.f)
						  .c_str());
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (!skeleton.getHand((eHandSide)sideIndex).hasAnyFinger())
		{
			ImGui::TextColored(k_warningColor, "%s",
							   locText(sideIndex == 0 ? "avatarPanel.leftHandIncompleteText"
													  : "avatarPanel.rightHandIncompleteText"));
		}
	}
	for (const std::string& warning : m_app->getAvatarRigWarnings())
		ImGui::TextWrapped("%s", warning.c_str());

	if (!model.sourcePath.empty())
	{
		const std::filesystem::path rigPath= getAvatarRigPath(PathUtils::utf8ToPath(model.sourcePath));
		ImGui::TextDisabled("%s", locFormat("avatarPanel.rigFileFmt",
											PathUtils::pathToUtf8(rigPath.filename()).c_str())
									  .c_str());
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", PathUtils::pathToUtf8(rigPath).c_str());
	}
}

void AvatarPanel::drawShowInScene()
{
	if (ImGui::Checkbox(locLabel("avatarPanel.showInSceneCheckbox"), &m_config->avatar.showInScene))
		m_config->markDirty();
}

void AvatarPanel::drawPlacement()
{
	AvatarConfig& avatar= m_config->avatar;
	bool bChanged= false;

	bChanged|= ImGui::Checkbox(locLabel("avatarPanel.followShouldersCheckbox"), &avatar.followShoulders);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", locText("avatarPanel.followShouldersTooltip"));

	ImGui::TextDisabled("%s", locText("avatarPanel.fixedRootHeader"));
	bChanged|= ImGui::DragFloat3(locLabel("avatarPanel.rootPositionDrag"), &avatar.rootPositionWorld.x, 0.005f,
								 -5.f, 5.f, "%.3f m");
	bChanged|= ImGui::DragFloat(locLabel("avatarPanel.rootYawDrag"), &avatar.rootYawDegrees, 0.5f, -180.f, 180.f,
								"%.1f deg");
	if (ImGui::Button(locLabel("avatarPanel.resetRootButton")))
	{
		avatar.rootPositionWorld= glm::vec3(0.f);
		avatar.rootYawDegrees= 0.f;
		bChanged= true;
	}

	if (bChanged)
		m_config->markDirty();
}

void AvatarPanel::drawMapping()
{
	if (m_app->getAvatarModel() == nullptr || m_app->getAvatarSkeleton() == nullptr)
		return;
	const std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT> states=
		AvatarMapFigure::computeDotStates(*m_app->getAvatarModel(), *m_app->getAvatarSkeleton());

	ImGui::TextWrapped("%s", locText("avatarPanel.mappingHelpText"));
	if (m_figure.draw(states, m_selectedBone, m_app->getAvatarModel().get()))
		m_bScrollToSelected= true;

	if (ImGui::Button(locLabel("avatarPanel.resetAllMappingButton")))
	{
		AvatarRigSettings rig= m_app->getAvatarRig();
		rig.boneNodeOverrides.fill(std::nullopt);
		m_app->setAvatarRig(rig);
	}
	ImGui::SameLine();
	ImGui::SetNextItemWidth(-FLT_MIN);
	ImGui::InputTextWithHint("##nodeFilter", locText("avatarPanel.nodeFilterHint"), m_nodeFilter,
							 sizeof(m_nodeFilter));

	const ImGuiTableFlags flags=
		ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
	if (!ImGui::BeginTable("avatarBoneTable", 2, flags))
		return;
	ImGui::TableSetupColumn(locText("avatarPanel.boneColumn"), ImGuiTableColumnFlags_WidthStretch, 1.f);
	ImGui::TableSetupColumn(locText("avatarPanel.nodeColumn"), ImGuiTableColumnFlags_WidthStretch, 1.4f);
	ImGui::TableHeadersRow();
	for (eHumanoidBone bone : AvatarMapFigure::getSectionBones(m_figure.getSection()))
		drawBoneRow(bone, states);
	ImGui::EndTable();
	m_bScrollToSelected= false;
}

void AvatarPanel::drawBoneRow(eHumanoidBone bone, const std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT>& states)
{
	const AvatarModel& model= *m_app->getAvatarModel();
	const int index= (int)bone;
	const std::optional<std::string>& nodeOverride= m_app->getAvatarRig().boneNodeOverrides[index];

	ImGui::PushID(index);
	ImGui::TableNextRow();

	// Bone: selectable, in the dot's warning color when it carries one
	ImGui::TableSetColumnIndex(0);
	const bool bSelected= m_selectedBone == bone;
	// A dot's click brings its row into view from below, no further than
	// that, so the figure above stays in view as far as the window allows
	if (bSelected && m_bScrollToSelected && !ImGui::IsRectVisible(ImVec2(1.f, ImGui::GetFrameHeight())))
		ImGui::SetScrollHereY(1.f);
	const eAvatarMapDotState state= states[index];
	if (state == eAvatarMapDotState::Warning || state == eAvatarMapDotState::RequiredUnmapped)
		ImGui::PushStyleColor(ImGuiCol_Text, state == eAvatarMapDotState::Warning ? k_warningColor
																				  : ImVec4(1.f, 0.4f, 0.4f, 1.f));
	else
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_Text));
	if (ImGui::Selectable(humanoidBoneName(bone), bSelected))
		m_selectedBone= bone;
	ImGui::PopStyleColor();

	// Node: a picker over the model's node names, plus unmapped, plus a
	// reset to the file's own mapping when overridden
	ImGui::TableSetColumnIndex(1);
	const int node= model.humanoidNodes[index];
	const std::string preview= node >= 0 ? nodeName(model, node) : locText("avatarPanel.nodeUnmappedItem");
	const float resetWidth= nodeOverride.has_value() ? ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.x : 0.f;
	ImGui::SetNextItemWidth(-FLT_MIN - resetWidth);
	if (ImGui::BeginCombo("##node", preview.c_str(), ImGuiComboFlags_HeightLarge))
	{
		std::optional<std::string> chosen;
		bool bChose= false;
		const bool bRequired= isHumanoidBoneRequired(bone);
		ImGui::BeginDisabled(bRequired);
		if (ImGui::Selectable(locText("avatarPanel.nodeUnmappedItem"), node < 0))
		{
			chosen= std::string();
			bChose= true;
		}
		ImGui::EndDisabled();
		if (bRequired && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", locText("avatarPanel.requiredUnmapTooltip"));
		for (int candidate= 0; candidate < (int)model.nodes.size(); ++candidate)
		{
			const std::string& name= model.nodes[candidate].name;
			if (!containsCaseInsensitive(name, m_nodeFilter))
				continue;
			ImGui::PushID(candidate);
			if (ImGui::Selectable(name.c_str(), candidate == node))
			{
				chosen= name;
				bChose= true;
			}
			if (candidate == node)
				ImGui::SetItemDefaultFocus();
			ImGui::PopID();
		}
		ImGui::EndCombo();

		if (bChose)
		{
			// Choosing what the file already says clears the override
			AvatarRigSettings rig= m_app->getAvatarRig();
			const int fileNode= model.fileHumanoidNodes[index];
			const bool bSameAsFile= chosen->empty() ? fileNode < 0 : (fileNode >= 0 && *chosen == nodeName(model, fileNode));
			rig.boneNodeOverrides[index]= bSameAsFile ? std::nullopt : chosen;
			m_app->setAvatarRig(rig);
			m_selectedBone= bone;
		}
	}
	if (nodeOverride.has_value())
	{
		if (ImGui::IsItemHovered())
		{
			const int fileNode= model.fileHumanoidNodes[index];
			ImGui::SetTooltip("%s", locFormat("avatarPanel.overriddenTooltipFmt",
											  fileNode >= 0 ? nodeName(model, fileNode)
															: locText("avatarPanel.nodeUnmappedItem"))
										.c_str());
		}
		ImGui::SameLine();
		if (ImGui::Button("X##reset", ImVec2(ImGui::GetFrameHeight(), 0.f)))
		{
			AvatarRigSettings rig= m_app->getAvatarRig();
			rig.boneNodeOverrides[index]= std::nullopt;
			m_app->setAvatarRig(rig);
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", locText("avatarPanel.resetBoneTooltip"));
	}

	ImGui::PopID();
}

void AvatarPanel::drawModeRow()
{
	// One button per mode, the current one drawn pressed, as the chain row
	// below draws its chains
	const ImGuiStyle& style= ImGui::GetStyle();
	for (int index= 0; index < (int)eChainMode::Count; ++index)
	{
		if (index > 0)
			ImGui::SameLine();
		const bool bSelected= index == (int)m_chainMode;
		if (bSelected)
			ImGui::PushStyleColor(ImGuiCol_Button, style.Colors[ImGuiCol_ButtonActive]);
		if (ImGui::Button(locLabel(k_modeLabelKeys[index])))
			setChainMode((eChainMode)index);
		if (bSelected)
			ImGui::PopStyleColor();
		ImGui::SetItemTooltip("%s", locText(k_modeTooltipKeys[index]));
	}
}

void AvatarPanel::drawChains()
{
	const bool bAvatar= hasAvatar();
	// The inputs above can unload the model this frame, ahead of update
	const eRetargetChain chain= bAvatar || isArmChain(m_selectedChain) ? m_selectedChain : eRetargetChain::LeftArm;
	const eChainMode mode= bAvatar ? m_chainMode : eChainMode::Tracking;

	if (bAvatar)
	{
		ImGui::SeparatorText(locText("chains.modeHeading"));
		drawModeRow();
	}
	if (mode == eChainMode::CustomPose)
	{
		ImGui::SeparatorText(locText("chains.poseHeading"));
		ImGui::PushID("pose");
		drawPoseBar();
		ImGui::PopID();
	}
	// The pose has a face as well as chains
	const bool bFace= mode == eChainMode::CustomPose && m_bPoseFaceSelected;
	ImGui::SeparatorText(locText("chains.chainsHeading"));
	drawChainRow(bAvatar, mode == eChainMode::CustomPose);
	ImGui::SeparatorText(locText(bFace ? "chains.poseFaceButton" : k_chainLabelKeys[(int)chain]));

	ImGui::PushID("chain");
	if (mode == eChainMode::RestPose)
	{
		drawRestRows(chain);
	}
	else if (bFace)
	{
		drawPoseFaceRows();
	}
	else if (mode == eChainMode::CustomPose)
	{
		drawPoseRows(chain);
	}
	else
	{
		switch (chain)
		{
		case eRetargetChain::Torso:
			drawTorsoChain();
			break;
		case eRetargetChain::Head:
			drawHeadChain();
			break;
		case eRetargetChain::LeftArm:
		case eRetargetChain::RightArm:
			drawArmChain(chain == eRetargetChain::LeftArm ? 0 : 1, bAvatar);
			break;
		case eRetargetChain::LeftHand:
		case eRetargetChain::RightHand:
			drawHandChain(chain == eRetargetChain::LeftHand ? 0 : 1);
			break;
		default:
			break;
		}
	}
	ImGui::PopID();
}

void AvatarPanel::drawPoseBar()
{
	CustomPoseLibrary* library= m_app->getCustomPoseLibrary();
	// A copy: every action below can replace the app's
	const CustomPose active= m_app->getActiveCustomPose();
	const bool bBuiltIn= isBuiltInCustomPoseId(active.id);

	const std::string activeLabel= bBuiltIn ? std::string(locText("chains.poseDemoItem")) : active.name;
	if (ImGui::BeginCombo(locLabel("chains.poseCombo"), activeLabel.c_str()))
	{
		if (ImGui::Selectable(locText("chains.poseDemoItem"), bBuiltIn))
			m_app->setActiveCustomPose(std::string());
		for (const CustomPose& pose : library->getPoses())
		{
			ImGui::PushID(pose.id.c_str());
			if (ImGui::Selectable(pose.name.c_str(), pose.id == active.id))
				m_app->setActiveCustomPose(pose.id);
			ImGui::PopID();
		}
		ImGui::EndCombo();
	}

	if (ImGui::Button(locLabel("faceTab.newButton")))
		m_app->createCustomPose(locText("chains.newPoseName"));
	ImGui::SameLine();
	if (ImGui::Button(locLabel(bBuiltIn ? "faceTab.duplicateAutoButton" : "faceTab.duplicateButton")))
	{
		const std::string sourceName= bBuiltIn ? std::string(locText("chains.poseDemoName")) : active.name;
		m_app->duplicateCustomPose(active, locFormat("faceTab.copyNameFmt", sourceName.c_str()));
	}
	ImGui::BeginDisabled(bBuiltIn);
	ImGui::SameLine();
	if (ImGui::Button(locLabel("faceTab.deleteButton")))
	{
		m_pendingDeletePoseId= active.id;
		ImGui::OpenPopup(locWindowTitle("windows.modalDeleteCustomPose"));
	}
	ImGui::EndDisabled();

	// Rename in place: the name inside the file changes, the id stays
	if (!bBuiltIn)
	{
		if (m_poseNameEditId != active.id)
		{
			m_poseNameEditId= active.id;
			m_poseNameEdit= active.name;
		}
		ImGui::InputText(locLabel("faceTab.nameInput"), &m_poseNameEdit);
		if (ImGui::IsItemDeactivatedAfterEdit() && !m_poseNameEdit.empty() && m_poseNameEdit != active.name)
			m_app->renameCustomPose(active.id, m_poseNameEdit);
	}
	else
	{
		m_poseNameEditId.clear();
		drawNote("chains.poseReadOnlyNote");
	}

	if (ImGui::BeginPopupModal(locWindowTitle("windows.modalDeleteCustomPose"), nullptr,
							   ImGuiWindowFlags_AlwaysAutoResize))
	{
		const CustomPose* pending= library->find(m_pendingDeletePoseId);
		const std::string pendingName= pending != nullptr ? pending->name : m_pendingDeletePoseId;
		ImGui::TextWrapped("%s", locFormat("chains.deletePosePromptFmt", pendingName.c_str()).c_str());
		if (ImGui::Button(locLabel("chains.deletePoseConfirmButton")))
		{
			m_app->deleteCustomPose(m_pendingDeletePoseId);
			m_pendingDeletePoseId.clear();
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button(locLabel("common.cancel")))
		{
			m_pendingDeletePoseId.clear();
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

void AvatarPanel::drawPoseFaceRows()
{
	CustomPose pose= m_app->getActiveCustomPose();
	const bool bBuiltIn= isBuiltInCustomPoseId(pose.id);
	bool bChanged= false;

	// The inputs the bound profile's outputs read, themselves or through
	// their variables, in the input table's order
	std::vector<int> slots;
	const std::shared_ptr<const FaceProfileBinding>& binding= m_app->getFaceBinding();
	if (binding != nullptr && binding->getProgram() != nullptr)
	{
		for (const FaceExprProgram::Expression& output : binding->getProgram()->outputs)
		{
			for (const int slot : output.inputs)
			{
				if (!FaceExprInputs::isFlagSlot(slot))
					slots.push_back(slot);
			}
		}
		std::sort(slots.begin(), slots.end());
		slots.erase(std::unique(slots.begin(), slots.end()), slots.end());
	}

	drawNote(slots.empty() ? "chains.poseFaceNoInputsNote" : "chains.poseFaceNote");
	ImGui::BeginDisabled(bBuiltIn);
	for (const int slot : slots)
	{
		const std::string name= FaceExprInputs::name(slot);
		const auto found= pose.face.find(name);
		float value= found != pose.face.end() ? found->second : 0.f;
		if (ImGui::SliderFloat(name.c_str(), &value, 0.f, 1.f, "%.2f"))
		{
			pose.face[name]= value;
			bChanged= true;
		}
	}
	if (!slots.empty())
	{
		ImGui::Spacing();
		if (ImGui::Button(locLabel("chains.poseResetFaceButton")))
		{
			pose.face.clear();
			bChanged= true;
		}
	}
	ImGui::EndDisabled();

	if (bChanged && !bBuiltIn)
		m_app->editActiveCustomPose(pose);
}

void AvatarPanel::drawPoseRows(eRetargetChain chain)
{
	CustomPose pose= m_app->getActiveCustomPose();
	const bool bBuiltIn= isBuiltInCustomPoseId(pose.id);
	bool bChanged= false;

	ImGui::BeginDisabled(bBuiltIn);
	switch (chain)
	{
	case eRetargetChain::Torso:
		// Only the direct placement places a spine
		if (!isDirectPipeline())
			drawNote("chains.poseLeanPositionNote");
		bChanged|= ImGui::DragFloat3(locLabel("chains.poseLeanDrag"), &pose.leanDegrees.x, 0.25f, -90.f, 90.f, "%.1f");
		ImGui::SetItemTooltip("%s", locText("chains.poseLeanTooltip"));
		break;
	case eRetargetChain::Head:
		drawGizmoToggle(eSceneGizmoValue::PoseHead);
		bChanged|= ImGui::DragFloat3(locLabel("chains.poseHeadDrag"), &pose.headDegrees.x, 0.25f, -180.f, 180.f, "%.1f");
		ImGui::SetItemTooltip("%s", locText("chains.poseHeadTooltip"));
		break;
	case eRetargetChain::LeftArm:
	case eRetargetChain::RightArm:
	{
		const int sideIndex= chain == eRetargetChain::LeftArm ? 0 : 1;
		CustomPose::Arm& arm= pose.arms[sideIndex];
		bChanged|= ImGui::Checkbox(locLabel("chains.poseArmEnabledCheckbox"), &arm.bEnabled);
		ImGui::SetItemTooltip("%s", locText("chains.poseArmEnabledTooltip"));
		ImGui::BeginDisabled(!arm.bEnabled);
		drawGizmoToggle(eSceneGizmoValue::PoseWrist);
		bChanged|= ImGui::DragFloat3(locLabel("chains.poseWristDrag"), &arm.wristOffset.x, 0.005f, -1.2f, 1.2f, "%.3f");
		ImGui::SetItemTooltip("%s", locText("chains.poseWristTooltip"));
		drawGizmoToggle(eSceneGizmoValue::PosePalm);
		bChanged|= ImGui::DragFloat3(locLabel("chains.posePalmDrag"), &arm.palmDegrees.x, 0.5f, -180.f, 180.f, "%.1f");
		ImGui::SetItemTooltip("%s", locText("chains.posePalmTooltip"));
		bChanged|= ImGui::Checkbox(locLabel("chains.poseElbowCheckbox"), &arm.bHasElbow);
		ImGui::SetItemTooltip("%s", locText("chains.poseElbowTooltip"));
		if (arm.bHasElbow)
		{
			drawGizmoToggle(eSceneGizmoValue::PoseElbow);
			bChanged|=
				ImGui::DragFloat3(locLabel("chains.poseElbowDrag"), &arm.elbowOffset.x, 0.005f, -1.2f, 1.2f, "%.3f");
			ImGui::SetItemTooltip("%s", locText("chains.poseWristTooltip"));
		}
		ImGui::EndDisabled();

		ImGui::Spacing();
		if (ImGui::Button(locLabel("chains.poseResetArmButton")))
		{
			arm= makeDefaultCustomPose().arms[sideIndex];
			bChanged= true;
		}
		ImGui::SameLine();
		if (ImGui::Button(locLabel("chains.copyToOtherArmButton")))
		{
			pose.arms[1 - sideIndex]= mirrorCustomPoseArm(arm);
			bChanged= true;
		}
		ImGui::SetItemTooltip("%s", locText("chains.poseMirrorArmTooltip"));
		break;
	}
	case eRetargetChain::LeftHand:
	case eRetargetChain::RightHand:
	{
		const int sideIndex= chain == eRetargetChain::LeftHand ? 0 : 1;
		std::array<FingerAngles, FINGER_COUNT>& fingers= pose.fingers[sideIndex];
		if (!pose.arms[sideIndex].bEnabled)
			drawNote("chains.poseHandArmOffNote");

		// One row a finger, its four angles in degrees
		static const char* const k_fingerKeys[FINGER_COUNT]= {
			"avatarPanel.fingerThumbCheckbox", "avatarPanel.fingerIndexCheckbox", "avatarPanel.fingerMiddleCheckbox",
			"avatarPanel.fingerRingCheckbox", "avatarPanel.fingerLittleCheckbox",
		};
		static const char* const k_columnKeys[5]= {"chains.poseFingerColumn", "chains.poseSplayColumn",
													"chains.poseBaseColumn", "chains.poseMiddleColumn",
													"chains.poseTipColumn"};
		if (ImGui::BeginTable("##poseFingers", 5, ImGuiTableFlags_SizingStretchSame))
		{
			for (const char* columnKey : k_columnKeys)
				ImGui::TableSetupColumn(locText(columnKey));
			ImGui::TableHeadersRow();
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				FingerAngles& angles= fingers[finger];
				float* const values[4]= {&angles.lateral, &angles.proximal, &angles.intermediate, &angles.distal};
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::AlignTextToFramePadding();
				ImGui::TextUnformatted(locText(k_fingerKeys[finger]));
				ImGui::PushID(finger);
				for (int column= 0; column < 4; ++column)
				{
					ImGui::TableNextColumn();
					ImGui::PushID(column);
					ImGui::SetNextItemWidth(-FLT_MIN);
					float degrees= glm::degrees(*values[column]);
					// The splay runs either way, a bend from a little back
					// to folded
					const float low= column == 0 ? -60.f : -30.f;
					const float high= column == 0 ? 60.f : 120.f;
					if (ImGui::DragFloat("##angle", &degrees, 0.5f, low, high, "%.0f"))
					{
						*values[column]= glm::radians(degrees);
						bChanged= true;
					}
					ImGui::PopID();
				}
				ImGui::PopID();
			}
			ImGui::EndTable();
		}

		if (ImGui::Button(locLabel("chains.poseOpenHandButton")))
		{
			fingers= {};
			bChanged= true;
		}
		ImGui::SameLine();
		if (ImGui::Button(locLabel("chains.poseFistButton")))
		{
			for (int finger= 1; finger < FINGER_COUNT; ++finger)
				fingers[finger]= {0.f, 1.3f, 1.2f, 0.7f};
			fingers[(int)eFinger::Thumb]= {0.3f, 0.6f, 0.8f, 0.4f};
			bChanged= true;
		}
		ImGui::SameLine();
		if (ImGui::Button(locLabel("chains.poseCopyHandButton")))
		{
			pose.fingers[1 - sideIndex]= mirrorCustomPoseFingers(fingers);
			bChanged= true;
		}
		break;
	}
	default:
		break;
	}
	ImGui::EndDisabled();

	if (bChanged && !bBuiltIn)
		m_app->editActiveCustomPose(pose);
}

void AvatarPanel::drawRestRows(eRetargetChain chain)
{
	RetargetChains chains= m_app->getRetargetChains();
	AvatarRigSettings rig= m_app->getAvatarRig();
	bool bRigChanged= false;
	bool bChainChanged= false;

	switch (chain)
	{
	case eRetargetChain::Torso:
		// A multi-camera project measures no torso, so it has none to rest
		if (!isDirectPipeline())
		{
			drawNote("chains.torsoPositionNote");
			break;
		}
		drawNote("chains.restIdentityNote");
		bChainChanged|= drawRestBlend(chains.torso.loss, false);
		break;
	case eRetargetChain::Head:
		drawNote("chains.restIdentityNote");
		bChainChanged|= drawRestBlend(chains.head.loss, false);
		break;
	case eRetargetChain::LeftArm:
	case eRetargetChain::RightArm:
	{
		const int sideIndex= chain == eRetargetChain::LeftArm ? 0 : 1;
		AvatarRigSettings::Side& side= rig.sides[sideIndex];
		RetargetChains::Arm& arm= chains.arms[sideIndex];
		drawGizmoToggle(eSceneGizmoValue::RestElbow);
		bRigChanged|= ImGui::DragFloat3(locLabel("avatarPanel.restElbowDrag"), &side.restElbowOffset.x, 0.005f, -1.f,
										1.f, "%.3f m");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restElbowTooltip"));
		drawGizmoToggle(eSceneGizmoValue::RestWrist);
		bRigChanged|= ImGui::DragFloat3(locLabel("avatarPanel.restWristDrag"), &side.restWristOffset.x, 0.005f, -1.f,
										1.f, "%.3f m");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restWristTooltip"));
		bRigChanged|= ImGui::DragFloat3(locLabel("avatarPanel.restPalmDrag"), &side.restPalmNormal.x, 0.01f, -1.f, 1.f,
										"%.2f");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restPalmTooltip"));
		bChainChanged|= drawRestBlend(arm.loss, true);

		ImGui::Spacing();
		if (ImGui::Button(locLabel("avatarPanel.resetRestButton")))
		{
			const AvatarRigSettings::Side defaults;
			side.restElbowOffset= defaults.restElbowOffset;
			side.restWristOffset= defaults.restWristOffset;
			side.restPalmNormal= defaults.restPalmNormal;
			bRigChanged= true;
			const RetargetChains::Arm armDefaults= RetargetChains::makeDefaultArm(sideIndex);
			arm.loss.restBlendMs= armDefaults.loss.restBlendMs;
			arm.loss.riseBlendMs= armDefaults.loss.riseBlendMs;
			bChainChanged= true;
		}
		ImGui::SetItemTooltip("%s", locText("avatarPanel.resetRestTooltip"));
		break;
	}
	case eRetargetChain::LeftHand:
	case eRetargetChain::RightHand:
	{
		AvatarRigSettings::Side& side= rig.sides[chain == eRetargetChain::LeftHand ? 0 : 1];
		drawNote("chains.handLossNote");
		bRigChanged|= ImGui::SliderFloat(locLabel("avatarPanel.restCurlSlider"), &side.restFingerCurlDegrees, 0.f, 90.f,
										 "%.0f deg");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restCurlTooltip"));
		break;
	}
	default:
		break;
	}

	if (bRigChanged)
		m_app->setAvatarRig(rig);
	if (bChainChanged)
		m_app->setRetargetChains(chains);
}

void AvatarPanel::drawChainRow(bool bAvatar, bool bFaceEntry)
{
	// One button per chain, the selected one drawn pressed, wrapping when
	// the next would not fit
	const ImGuiStyle& style= ImGui::GetStyle();
	const float rightEdge= ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
	bool bFirst= true;
	for (int index= 0; index < (int)eRetargetChain::Count; ++index)
	{
		const eRetargetChain chain= (eRetargetChain)index;
		if (!bAvatar && !isArmChain(chain))
			continue;
		const char* label= locLabel(k_chainLabelKeys[index]);
		const float width= ImGui::CalcTextSize(label, nullptr, true).x + style.FramePadding.x * 2.f;
		if (!bFirst && ImGui::GetItemRectMax().x + style.ItemSpacing.x + width <= rightEdge)
			ImGui::SameLine();
		bFirst= false;

		const bool bSelected= chain == m_selectedChain && !(bFaceEntry && m_bPoseFaceSelected);
		if (bSelected)
			ImGui::PushStyleColor(ImGuiCol_Button, style.Colors[ImGuiCol_ButtonActive]);
		if (ImGui::Button(label))
			setSelectedChain(chain);
		if (bSelected)
			ImGui::PopStyleColor();
	}
	if (bFaceEntry)
	{
		const char* label= locLabel("chains.poseFaceButton");
		const float width= ImGui::CalcTextSize(label, nullptr, true).x + style.FramePadding.x * 2.f;
		if (ImGui::GetItemRectMax().x + style.ItemSpacing.x + width <= rightEdge)
			ImGui::SameLine();
		const bool bSelected= m_bPoseFaceSelected;
		if (bSelected)
			ImGui::PushStyleColor(ImGuiCol_Button, style.Colors[ImGuiCol_ButtonActive]);
		if (ImGui::Button(label))
		{
			// A chain's handle does not belong to the face
			m_bPoseFaceSelected= true;
			m_gizmoTarget= SceneGizmoTarget();
		}
		if (bSelected)
			ImGui::PopStyleColor();
	}
}

void AvatarPanel::drawTorsoChain()
{
	if (!isDirectPipeline())
	{
		drawNote("chains.torsoPositionNote");
		return;
	}
	drawNote("chains.torsoDirectNote");
	AvatarRigSettings rig= m_app->getAvatarRig();
	if (drawLimitControl(rig.spineLimitDegrees, "chains.spineLimit", "chains.spineLimitTooltip"))
		m_app->setAvatarRig(rig);
	// The solver's hold, then what follows it
	drawSubHeading("chains.lossHeading");
	drawNote("chains.torsoDirectLossNote");
	RetargetChains chains= m_app->getRetargetChains();
	if (drawLossAfter(chains.torso.loss))
		m_app->setRetargetChains(chains);
}

void AvatarPanel::drawHeadChain()
{
	const bool bDirect= isDirectPipeline();
	AvatarRigSettings rig= m_app->getAvatarRig();
	drawGizmoToggle(eSceneGizmoValue::Trim);
	bool bChanged= drawTrimControl(rig, B::Head, "chains.headTrim", false);
	// Only the direct placement limits the head
	if (bDirect)
		bChanged|= drawLimitControl(rig.headLimitDegrees, "chains.headLimit", "chains.headLimitTooltip");
	if (bChanged)
		m_app->setAvatarRig(rig);

	// The direct solver's own hold, or a timed one, then what follows it
	drawSubHeading("chains.lossHeading");
	RetargetChains chains= m_app->getRetargetChains();
	RetargetChains::Loss& loss= chains.head.loss;
	bool bChainChanged= false;
	if (bDirect)
	{
		drawNote("chains.headDirectLossNote");
	}
	else
	{
		if (ImGui::SliderFloat(locLabel("oscPanel.dropoutHold"), &loss.holdMs, 0.f, 1000.f, "%.0f ms"))
		{
			// A length the head holds for, which a block stored as holding
			// nothing would not take
			loss.hold= eLossHold::Timed;
			bChainChanged= true;
		}
		ImGui::SetItemTooltip("%s", locText("chains.headHoldTooltip"));
	}
	bChainChanged|= drawLossAfter(loss);
	if (bChainChanged)
		m_app->setRetargetChains(chains);
}

bool AvatarPanel::drawArmLoss(int sideIndex, bool bDirect, bool bAvatar, RetargetChains& chains)
{
	RetargetChains::Loss& loss= chains.arms[sideIndex].loss;
	bool bChanged= false;
	if (bDirect)
	{
		drawNote("chains.directArmLossNote");
	}
	else
	{
		bChanged|= ImGui::SliderFloat(locLabel("oscPanel.dropoutHold"), &loss.holdMs, 0.f, 1000.f, "%.0f ms");
		ImGui::SetItemTooltip("%s", locText("oscPanel.dropoutHoldTooltip"));
	}
	bChanged|= drawLossAfter(loss);
	// The measured-length stream has no rest pose
	if (!bAvatar && loss.after == eLossAfter::EaseToRest)
		drawNote("chains.easeWithoutAvatarNote");
	return bChanged;
}

void AvatarPanel::drawArmChain(int sideIndex, bool bAvatar)
{
	const bool bDirect= isDirectPipeline();
	RetargetChains chains= m_app->getRetargetChains();
	bool bChainChanged= false;

	// Without a model only the stream's loss rules apply
	if (!bAvatar)
	{
		drawSubHeading("chains.lossHeading");
		if (drawArmLoss(sideIndex, false, false, chains))
			m_app->setRetargetChains(chains);
		return;
	}

	AvatarRigSettings rig= m_app->getAvatarRig();
	AvatarRigSettings::Side& side= rig.sides[sideIndex];
	bool bRigChanged= false;
	const eHumanoidBone lowerArmBone= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;

	// The mapping as stored, which both pipelines run: bone angles alone, or
	// then the pull toward the goal by the IK weight, with the goal's reach
	// scale and offset
	RetargetChains::Arm& arm= chains.arms[sideIndex];
	struct MappingChoice
	{
		eArmMapping mapping;
		const char* labelKey;
		const char* tooltipKey;
	};
	static const MappingChoice k_mappingChoices[]= {
		{eArmMapping::BoneAngles, "chains.mappingBoneAngles", "chains.mappingBoneAnglesTooltip"},
		{eArmMapping::BoneAnglesThenIk, "chains.mappingBoneAnglesThenIk", "chains.mappingBoneAnglesThenIkTooltip"},
	};
	ImGui::TextUnformatted(locText("chains.mappingLabel"));
	for (const MappingChoice& choice : k_mappingChoices)
	{
		if (ImGui::RadioButton(locLabel(choice.labelKey), arm.mapping == choice.mapping) && arm.mapping != choice.mapping)
		{
			arm.mapping= choice.mapping;
			bChainChanged= true;
		}
		ImGui::SetItemTooltip("%s", locText(choice.tooltipKey));
	}
	if (arm.mapping == eArmMapping::BoneAngles)
	{
		drawNote("chains.boneAnglesNote");
	}
	else
	{
		bChainChanged|= drawExprParameterField(sideIndex == 0 ? "leftIkWeight" : "rightIkWeight",
											   locText("chains.ikWeightLabel"), arm.ikWeight, 0.f, 1.f, "%.2f",
											   locText("chains.ikWeightTooltip"), SourcePoseInputs::table(),
											   toLiveValue(m_shownArmParameters[sideIndex].ikWeight));
		// The source's reach is in meters on a multi-camera project and in
		// the avatar's units with the user's proportions on a direct one; the
		// scale has an effect on both, since the source's arm is not the
		// avatar's
		bChainChanged|= drawExprParameterField(
			sideIndex == 0 ? "leftReachScale" : "rightReachScale", locText("chains.reachScaleField"), arm.reachScale, 0.f,
			1.f, "%.2f", locText(bDirect ? "chains.reachScaleDirectTooltip" : "chains.reachScaleTooltip"),
			SourcePoseInputs::table(), toLiveValue(m_shownArmParameters[sideIndex].reachScale));
		drawGizmoToggle(eSceneGizmoValue::GoalOffset);
		bChainChanged|= ImGui::DragFloat3(locLabel("chains.goalOffsetDrag"), &arm.goalOffset.x, 0.005f, -1.f, 1.f,
										  "%.3f m");
		ImGui::SetItemTooltip("%s", locText("chains.goalOffsetTooltip"));
	}

	drawSubHeading("chains.elbowHeading");
	drawGizmoToggle(eSceneGizmoValue::ElbowHint);
	bRigChanged|= ImGui::DragFloat3(locLabel("avatarPanel.elbowHintDrag"), &side.elbowHintOffset.x, 0.005f, -1.f, 1.f,
									"%.3f m");
	ImGui::SetItemTooltip("%s", locText("avatarPanel.elbowHintTooltip"));
	// The elbow weight is the arm chain's, the project's rather than the
	// rig's
	bChainChanged|= drawExprParameterField(sideIndex == 0 ? "leftElbowWeight" : "rightElbowWeight",
										   locText("avatarPanel.elbowWeightField"), arm.elbowWeight, 0.f, 1.f, "%.2f",
										   locText("avatarPanel.elbowWeightTooltip"), SourcePoseInputs::table(),
										   toLiveValue(m_shownArmParameters[sideIndex].elbowWeight));

	drawSubHeading("chains.forearmHeading");
	drawGizmoToggle(eSceneGizmoValue::Trim);
	bRigChanged|= drawTrimControl(rig, lowerArmBone, "chains.rollTrim", true);

	drawSubHeading("chains.lossHeading");
	bChainChanged|= drawArmLoss(sideIndex, bDirect, true, chains);

	ImGui::Spacing();
	if (ImGui::Button(locLabel("avatarPanel.resetArmButton")))
	{
		const AvatarRigSettings::Side defaults;
		side.elbowHintOffset= defaults.elbowHintOffset;
		bRigChanged= true;
		// The chain's solve settings, the mapping and the IK weight the
		// project's shape defaults to; its loss stays
		const RetargetChains::Arm armDefaults= RetargetChains::makeDefaultArm(sideIndex);
		arm.elbowWeight= armDefaults.elbowWeight;
		arm.reachScale= armDefaults.reachScale;
		arm.goalOffset= armDefaults.goalOffset;
		const RetargetChains::Arm shapeDefaults= makeDefaultRetargetChains(m_config->direct.enabled).arms[sideIndex];
		arm.mapping= shapeDefaults.mapping;
		arm.ikWeight= shapeDefaults.ikWeight;
		bChainChanged= true;
	}
	ImGui::SetItemTooltip("%s", locText("chains.resetArmTooltip"));
	ImGui::SameLine();
	if (ImGui::Button(locLabel("chains.copyToOtherArmButton")))
	{
		// The other arm becomes this one's mirror image: the points and
		// directions in the torso frame reflected across the body's midplane,
		// the forearm roll reversed (it turns about the forearm, which points
		// the other way), the elbow weight's source names spelled for the
		// other side, every other value as it is
		const int otherIndex= 1 - sideIndex;
		AvatarRigSettings::Side& other= rig.sides[otherIndex];
		other.elbowHintOffset= mirrorAcrossSagittal(side.elbowHintOffset);
		other.restElbowOffset= mirrorAcrossSagittal(side.restElbowOffset);
		other.restWristOffset= mirrorAcrossSagittal(side.restWristOffset);
		other.restPalmNormal= mirrorAcrossSagittal(side.restPalmNormal);
		other.restFingerCurlDegrees= side.restFingerCurlDegrees;
		const eHumanoidBone otherLowerArmBone= otherIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
		rig.rotationTrimDegrees[(int)otherLowerArmBone].x= -rig.rotationTrimDegrees[(int)lowerArmBone].x;
		bRigChanged= true;

		chains.arms[otherIndex]= mirrorArm(chains.arms[sideIndex]);
		bChainChanged= true;
	}
	ImGui::SetItemTooltip("%s", locText("chains.mirrorToOtherArmTooltip"));

	if (bRigChanged)
		m_app->setAvatarRig(rig);
	if (bChainChanged)
		m_app->setRetargetChains(chains);
}

void AvatarPanel::drawHandChain(int sideIndex)
{
	AvatarRigSettings rig= m_app->getAvatarRig();
	AvatarRigSettings::Side& side= rig.sides[sideIndex];
	bool bChanged= false;

	drawGizmoToggle(eSceneGizmoValue::Trim);
	bChanged|= drawTrimControl(rig, sideIndex == 0 ? B::LeftHand : B::RightHand, "chains.handTrim", false);
	// Only the direct placement limits the wrist
	if (isDirectPipeline())
		bChanged|= drawLimitControl(side.wristLimitDegrees, "chains.wristLimit", "chains.wristLimitTooltip");

	if (ImGui::Checkbox(locLabel("avatarPanel.thumbPronationAutoCheckbox"), &side.bThumbPronationAuto))
	{
		// Leaving auto starts from what auto was doing
		if (!side.bThumbPronationAuto)
		{
			const AvatarSkeleton& skeleton= *m_app->getAvatarSkeleton();
			side.thumbPronationDegrees=
				glm::degrees(HandPoseModel::getThumbPronationRad(skeleton.getHand((eHandSide)sideIndex).skeleton));
		}
		bChanged= true;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", locText("avatarPanel.thumbPronationTooltip"));
	if (!side.bThumbPronationAuto)
	{
		bChanged|= ImGui::DragFloat(locLabel("avatarPanel.thumbPronationDrag"), &side.thumbPronationDegrees, 0.5f,
									-180.f, 180.f, "%.1f deg");
	}
	bChanged|= ImGui::SliderFloat(locLabel("avatarPanel.curlGainSlider"), &side.curlGain, 0.f, 2.f, "%.2f");
	bChanged|= ImGui::SliderFloat(locLabel("avatarPanel.splayGainSlider"), &side.splayGain, 0.f, 2.f, "%.2f");

	ImGui::TextUnformatted(locText("avatarPanel.fingersDrivenLabel"));
	const char* k_fingerKeys[FINGER_COUNT]= {
		"avatarPanel.fingerThumbCheckbox", "avatarPanel.fingerIndexCheckbox", "avatarPanel.fingerMiddleCheckbox",
		"avatarPanel.fingerRingCheckbox", "avatarPanel.fingerLittleCheckbox",
	};
	for (int finger= 0; finger < FINGER_COUNT; ++finger)
	{
		if (finger > 0)
			ImGui::SameLine();
		bool bEnabled= side.fingerEnabled[finger];
		if (ImGui::Checkbox(locLabel(k_fingerKeys[finger]), &bEnabled))
		{
			side.fingerEnabled[finger]= bEnabled;
			bChanged= true;
		}
	}

	drawNote("chains.handLossNote");

	if (ImGui::Button(locLabel("avatarPanel.resetHandButton")))
	{
		const AvatarRigSettings::Side defaults;
		side.bThumbPronationAuto= defaults.bThumbPronationAuto;
		side.thumbPronationDegrees= defaults.thumbPronationDegrees;
		side.curlGain= defaults.curlGain;
		side.splayGain= defaults.splayGain;
		side.fingerEnabled= defaults.fingerEnabled;
		side.wristLimitDegrees= defaults.wristLimitDegrees;
		bChanged= true;
	}

	if (bChanged)
		m_app->setAvatarRig(rig);
}
