#include "AvatarMouthReadout.h"

#include "imgui.h"

#include "AudioTypes.h"
#include "FaceExprInputs.h"
#include "FaceProfileBinding.h"
#include "LocText.h"
#include "WeightBars.h"

namespace
{
bool isMouthInput(int slot)
{
	if (slot >= FaceExprInputs::k_visemeBase && slot < FaceExprInputs::k_visemePresent)
		return true;
	return slot >= FaceExprInputs::k_phoneBlendshapeBase && slot < FaceExprInputs::k_phonePresent &&
		   isMouthArkitIndex(slot - FaceExprInputs::k_phoneBlendshapeBase);
}
} // namespace

void AvatarMouthReadout::draw(const std::shared_ptr<const AvatarModel>& model,
							  const std::shared_ptr<const FaceProfileBinding>& binding, const FaceSources& face)
{
	if (!ImGui::CollapsingHeader(locLabel("oscPanel.mouthSection")))
		return;

	if (model == nullptr || binding == nullptr || binding->getProgram() == nullptr)
	{
		ImGui::TextDisabled("%s", locText("oscPanel.mouthNone"));
		m_binding= nullptr;
		return;
	}
	if (binding != m_binding)
	{
		m_binding= binding;
		m_mouthTargets.clear();
		m_bReadsPhone= false;
		m_bReadsVisemes= false;
		const std::vector<FaceProfileBinding::Target>& targets= binding->getTargets();
		const std::vector<FaceExprProgram::Expression>& outputs= binding->getProgram()->outputs;
		for (size_t targetIndex= 0; targetIndex < targets.size(); ++targetIndex)
		{
			bool bMouth= false;
			for (int outputIndex : targets[targetIndex].outputs)
			{
				for (int slot : outputs[(size_t)outputIndex].inputs)
					bMouth|= isMouthInput(slot);
			}
			if (!bMouth)
				continue;
			m_mouthTargets.push_back((int)targetIndex);
			// Every slot the shown targets read, mouth or not, names a source
			for (int outputIndex : targets[targetIndex].outputs)
			{
				for (int slot : outputs[(size_t)outputIndex].inputs)
				{
					if (slot < FaceExprInputs::k_visemeBase)
						m_bReadsPhone= true;
					else
						m_bReadsVisemes= true;
				}
			}
		}
	}
	if (m_mouthTargets.empty())
	{
		ImGui::TextDisabled("%s", locText("oscPanel.mouthNoMouth"));
		return;
	}

	// The sources the shown targets read that are live this frame
	const bool bPhone= m_bReadsPhone && face.phone.tracked;
	const bool bVisemes= m_bReadsVisemes && face.viseme.present;
	const char* sourceKey= bPhone && bVisemes ? "oscPanel.mouthFromBoth"
						   : bPhone           ? "oscPanel.mouthFromArkit"
						   : bVisemes         ? "oscPanel.mouthFromVisemes"
											  : "oscPanel.mouthFromNone";
	ImGui::TextDisabled(locText("oscPanel.mouthHeaderFmt"), (int)m_mouthTargets.size(), locText(sourceKey));

	// The rest face unless the frame's values came from this binding
	const bool bValues= face.isPresent() && face.outputs.generation == binding->getGeneration();
	m_values.resize(m_mouthTargets.size());
	for (size_t index= 0; index < m_mouthTargets.size(); ++index)
	{
		const size_t targetIndex= (size_t)m_mouthTargets[index];
		m_values[index]= bValues && targetIndex < face.outputs.targetValues.size() ? face.outputs.targetValues[targetIndex]
																				   : 0.f;
	}
	struct Context
	{
		const FaceProfileBinding* binding;
		const std::vector<int>* targets;
	};
	const Context context{binding.get(), &m_mouthTargets};
	drawWeightTable(
		"avatarMouth", (int)m_mouthTargets.size(),
		[](int index, const void* raw) -> const char* {
			const Context* c= (const Context*)raw;
			return c->binding->getTargets()[(size_t)(*c->targets)[(size_t)index]].name.c_str();
		},
		m_values.data(), &context);
}
