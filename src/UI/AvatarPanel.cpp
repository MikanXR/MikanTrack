#include "AvatarPanel.h"

#include <algorithm>
#include <cctype>
#include <optional>

#include "imgui.h"

#include "LocText.h"

#include "App.h"
#include "AppConfig.h"
#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "BodyPoseSolver.h"
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

eHumanoidBone fingerBone(int sideIndex, int finger, int phalanx)
{
	return (eHumanoidBone)((int)firstHumanoidFingerBone(sideIndex) + finger * 3 + phalanx);
}

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
		m_previewPose= ePreviewPose::Live;
		if (!isArmChain(m_selectedChain))
			m_selectedChain= eRetargetChain::LeftArm;
	}
}

bool AvatarPanel::setSelectedChain(eRetargetChain chain)
{
	if (chain == eRetargetChain::Count || (!hasAvatar() && !isArmChain(chain)))
		return false;
	m_selectedChain= chain;
	return true;
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

void AvatarPanel::drawPlacement()
{
	AvatarConfig& avatar= m_config->avatar;
	bool bChanged= false;

	bChanged|= ImGui::Checkbox(locLabel("avatarPanel.showInSceneCheckbox"), &avatar.showInScene);
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

std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT> AvatarPanel::computeDotStates() const
{
	const AvatarModel& model= *m_app->getAvatarModel();
	const AvatarSkeleton& skeleton= *m_app->getAvatarSkeleton();

	std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT> states;
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		if (model.humanoidNodes[index] >= 0)
			states[index]= eAvatarMapDotState::Mapped;
		else if (isHumanoidBoneRequired((eHumanoidBone)index))
			states[index]= eAvatarMapDotState::RequiredUnmapped;
		else
			states[index]= eAvatarMapDotState::OptionalUnmapped;
	}

	// One node on two bones
	for (int index= 0; index < HUMANOID_BONE_COUNT; ++index)
	{
		for (int other= index + 1; other < HUMANOID_BONE_COUNT; ++other)
		{
			if (model.humanoidNodes[index] >= 0 && model.humanoidNodes[index] == model.humanoidNodes[other])
			{
				states[index]= eAvatarMapDotState::Warning;
				states[other]= eAvatarMapDotState::Warning;
			}
		}
	}

	// The index sitting on the far side of the middle finger from the thumb
	// (the skeleton's swapped-finger warning) tags both fingers
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const AvatarSkeleton::HandRest& hand= skeleton.getHand((eHandSide)sideIndex);
		if (!hand.hasFinger((int)eFinger::Thumb) || !hand.hasFinger((int)eFinger::Index))
			continue;
		if (hand.skeleton.baseInPalm[(int)eFinger::Thumb].y * hand.skeleton.baseInPalm[(int)eFinger::Index].y >= 0.f)
			continue;
		for (int finger : {(int)eFinger::Index, (int)eFinger::Middle})
		{
			for (int phalanx= 0; phalanx < 3; ++phalanx)
			{
				const int bone= (int)fingerBone(sideIndex, finger, phalanx);
				if (states[bone] == eAvatarMapDotState::Mapped)
					states[bone]= eAvatarMapDotState::Warning;
			}
		}
	}
	return states;
}

void AvatarPanel::drawMapping()
{
	if (m_app->getAvatarModel() == nullptr || m_app->getAvatarSkeleton() == nullptr)
		return;
	const std::array<eAvatarMapDotState, HUMANOID_BONE_COUNT> states= computeDotStates();

	ImGui::TextWrapped("%s", locText("avatarPanel.mappingHelpText"));
	if (m_figure.draw(states, m_selectedBone))
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
	if (bSelected && m_bScrollToSelected)
		ImGui::SetScrollHereY(0.5f);
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

void AvatarPanel::drawPreviewPose()
{
	ImGui::TextUnformatted(locText("avatarPanel.previewPoseLabel"));
	ImGui::SameLine();
	int preview= (int)m_previewPose;
	ImGui::RadioButton(locLabel("avatarPanel.previewLiveRadio"), &preview, (int)ePreviewPose::Live);
	ImGui::SameLine();
	ImGui::RadioButton(locLabel("avatarPanel.previewRestRadio"), &preview, (int)ePreviewPose::Rest);
	ImGui::SameLine();
	ImGui::RadioButton(locLabel("avatarPanel.previewDemoRadio"), &preview, (int)ePreviewPose::Demo);
	m_previewPose= (ePreviewPose)preview;
	ImGui::SameLine();
	ImGui::TextDisabled("(?)");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", locText("avatarPanel.previewPoseTooltip"));
}

void AvatarPanel::drawChains()
{
	const bool bAvatar= hasAvatar();
	// The inputs above can unload the model this frame, ahead of update
	const eRetargetChain chain= bAvatar || isArmChain(m_selectedChain) ? m_selectedChain : eRetargetChain::LeftArm;

	ImGui::SeparatorText(locText("chains.chainsHeading"));
	drawChainRow(bAvatar);
	ImGui::SeparatorText(locText(k_chainLabelKeys[(int)chain]));

	ImGui::PushID("chain");
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
	ImGui::PopID();
}

void AvatarPanel::drawChainRow(bool bAvatar)
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

		const bool bSelected= chain == m_selectedChain;
		if (bSelected)
			ImGui::PushStyleColor(ImGuiCol_Button, style.Colors[ImGuiCol_ButtonActive]);
		if (ImGui::Button(label))
			setSelectedChain(chain);
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
	drawSubHeading("chains.lossHeading");
	drawNote("chains.torsoDirectLossNote");
}

void AvatarPanel::drawHeadChain()
{
	const bool bDirect= isDirectPipeline();
	AvatarRigSettings rig= m_app->getAvatarRig();
	bool bChanged= drawTrimControl(rig, B::Head, "chains.headTrim", false);
	// Only the direct placement limits the head
	if (bDirect)
		bChanged|= drawLimitControl(rig.headLimitDegrees, "chains.headLimit", "chains.headLimitTooltip");
	if (bChanged)
		m_app->setAvatarRig(rig);

	drawSubHeading("chains.lossHeading");
	drawNote(bDirect ? "chains.headDirectLossNote" : "chains.headPositionLossNote");
}

bool AvatarPanel::drawArmLoss(int sideIndex, bool bDirect, RetargetChains& chains)
{
	RetargetChains::Loss& loss= chains.arms[sideIndex].loss;
	bool bChanged= false;
	if (bDirect)
	{
		drawNote("chains.directArmLossNote");
		bChanged|= ImGui::SliderFloat(locLabel("avatarPanel.restBlendSlider"), &loss.restBlendMs, 0.f, 1000.f, "%.0f ms");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restBlendTooltip"));
		return bChanged;
	}

	bChanged|= ImGui::SliderFloat(locLabel("oscPanel.dropoutHold"), &loss.holdMs, 0.f, 1000.f, "%.0f ms");
	ImGui::SetItemTooltip("%s", locText("oscPanel.dropoutHoldTooltip"));

	// What follows the hold as the position pipeline performs it, freeze or
	// release
	const bool bFreeze=
		makeChainLossPolicy(chains, SourcePose::ePipeline::Position, 0.f).arms[sideIndex].after == eLossAfter::Freeze;
	int after= bFreeze ? 0 : 1;
	ImGui::TextUnformatted(locText("chains.thenLabel"));
	ImGui::SameLine();
	ImGui::RadioButton(locLabel("chains.afterFreezeRadio"), &after, 0);
	ImGui::SetItemTooltip("%s", locText("oscPanel.freezeOnLossTooltip"));
	ImGui::SameLine();
	ImGui::RadioButton(locLabel("chains.afterReleaseRadio"), &after, 1);
	ImGui::SetItemTooltip("%s", locText("oscPanel.freezeOnLossTooltip"));
	if ((after == 0) != bFreeze)
	{
		loss.after= after == 0 ? eLossAfter::Freeze : eLossAfter::Release;
		bChanged= true;
	}
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
		if (drawArmLoss(sideIndex, false, chains))
			m_app->setRetargetChains(chains);
		return;
	}

	AvatarRigSettings rig= m_app->getAvatarRig();
	AvatarRigSettings::Side& side= rig.sides[sideIndex];
	bool bRigChanged= false;
	const eHumanoidBone lowerArmBone= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;

	// What the pipeline runs, whatever the chain stores
	const SourcePose::ePipeline pipeline= bDirect ? SourcePose::ePipeline::Direct : SourcePose::ePipeline::Position;
	ImGui::LabelText(locLabel("chains.mappingLabel"), "%s",
					 locText(bDirect ? "chains.mappingBoneAngles" : "chains.mappingBoneAnglesThenIk"));
	ImGui::LabelText(locLabel("chains.ikWeightLabel"), "%.2f", RetargetStage::getArmIkWeight(pipeline));
	drawNote("chains.fixedByPipelineNote");
	// The goal the IK pulls the wrist toward; the direct pipeline places the
	// arm by bone angles alone and has none
	RetargetChains::Arm& arm= chains.arms[sideIndex];
	if (!bDirect)
	{
		bChainChanged|= drawExprParameterField(sideIndex == 0 ? "leftReachScale" : "rightReachScale",
											   locText("chains.reachScaleField"), arm.reachScale, 0.f, 1.f, "%.2f",
											   locText("chains.reachScaleTooltip"), SourcePoseInputs::table(),
											   toLiveValue(m_shownArmParameters[sideIndex].reachScale));
		bChainChanged|= ImGui::DragFloat3(locLabel("chains.goalOffsetDrag"), &arm.goalOffset.x, 0.005f, -1.f, 1.f,
										  "%.3f m");
		ImGui::SetItemTooltip("%s", locText("chains.goalOffsetTooltip"));
	}

	drawSubHeading("chains.elbowHeading");
	bRigChanged|= ImGui::DragFloat3(locLabel("avatarPanel.elbowHintDrag"), &side.elbowHintOffset.x, 0.005f, -1.f, 1.f,
									"%.3f m");
	ImGui::SetItemTooltip("%s", locText("avatarPanel.elbowHintTooltip"));
	// The elbow weight is the arm chain's, the project's rather than the
	// rig's
	bChainChanged|= drawExprParameterField(sideIndex == 0 ? "leftElbowWeight" : "rightElbowWeight",
										   locText("avatarPanel.elbowWeightField"), arm.elbowWeight, 0.f, 1.f, "%.2f",
										   locText("avatarPanel.elbowWeightTooltip"), SourcePoseInputs::table(),
										   toLiveValue(m_shownArmParameters[sideIndex].elbowWeight));
	ImGui::Checkbox(locLabel("avatarPanel.showGizmoCheckbox"), &m_bShowElbowGizmo[sideIndex]);

	drawSubHeading("chains.forearmHeading");
	bRigChanged|= drawTrimControl(rig, lowerArmBone, "chains.rollTrim", true);

	drawSubHeading("chains.lossHeading");
	bChainChanged|= drawArmLoss(sideIndex, bDirect, chains);

	// The rest pose is where an arm that eases to rest goes
	if (makeChainLossPolicy(chains, pipeline, 0.f).arms[sideIndex].after == eLossAfter::EaseToRest &&
		ImGui::TreeNode(locLabel("avatarPanel.restPoseLabel")))
	{
		bRigChanged|= ImGui::DragFloat3(locLabel("avatarPanel.restElbowDrag"), &side.restElbowOffset.x, 0.005f, -1.f,
										1.f, "%.3f m");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restElbowTooltip"));
		bRigChanged|= ImGui::DragFloat3(locLabel("avatarPanel.restWristDrag"), &side.restWristOffset.x, 0.005f, -1.f,
										1.f, "%.3f m");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restWristTooltip"));
		bRigChanged|= ImGui::DragFloat3(locLabel("avatarPanel.restPalmDrag"), &side.restPalmNormal.x, 0.01f, -1.f, 1.f,
										"%.2f");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restPalmTooltip"));
		bRigChanged|= ImGui::SliderFloat(locLabel("avatarPanel.restCurlSlider"), &side.restFingerCurlDegrees, 0.f,
										 90.f, "%.0f deg");
		ImGui::SetItemTooltip("%s", locText("avatarPanel.restCurlTooltip"));
		ImGui::TreePop();
	}

	ImGui::Spacing();
	if (ImGui::Button(locLabel("avatarPanel.resetArmButton")))
	{
		const AvatarRigSettings::Side defaults;
		side.elbowHintOffset= defaults.elbowHintOffset;
		side.restElbowOffset= defaults.restElbowOffset;
		side.restWristOffset= defaults.restWristOffset;
		side.restPalmNormal= defaults.restPalmNormal;
		side.restFingerCurlDegrees= defaults.restFingerCurlDegrees;
		bRigChanged= true;
		// The chain's solve settings; its mapping, IK weight and loss stay
		const RetargetChains::Arm armDefaults= RetargetChains::makeDefaultArm(sideIndex);
		arm.elbowWeight= armDefaults.elbowWeight;
		arm.reachScale= armDefaults.reachScale;
		arm.goalOffset= armDefaults.goalOffset;
		bChainChanged= true;
	}
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
