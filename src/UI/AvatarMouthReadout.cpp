#include "AvatarMouthReadout.h"

#include "imgui.h"

#include "AudioTypes.h"
#include "AvatarFaceMap.h"
#include "AvatarTypes.h"
#include "LocText.h"
#include "WeightBars.h"

void AvatarMouthReadout::draw(const std::shared_ptr<const AvatarModel>& model, const TrackingFrameResult::FacePose& face)
{
	if (!ImGui::CollapsingHeader(locLabel("oscPanel.mouthSection")))
		return;

	if (model == nullptr)
	{
		ImGui::TextDisabled("%s", locText("oscPanel.mouthNone"));
		m_faceMapModel= nullptr;
		m_faceMap= nullptr;
		return;
	}
	if (model != m_faceMapModel)
	{
		m_faceMapModel= model;
		m_faceMap= AvatarFaceMap::build(*model);
		m_mouthOutputs.clear();
		const std::vector<AvatarFaceMap::Output>& outputs= m_faceMap->getOutputs();
		for (size_t outputIndex= 0; outputIndex < outputs.size(); ++outputIndex)
		{
			bool bMouth= outputs[outputIndex].viseme >= 0;
			for (const AvatarFaceMap::Term& outputTerm : outputs[outputIndex].terms)
				bMouth|= isMouthArkitIndex(outputTerm.arkitIndex);
			if (bMouth)
				m_mouthOutputs.push_back((int)outputIndex);
		}
	}
	if (m_mouthOutputs.empty())
	{
		ImGui::TextDisabled("%s", locText("oscPanel.mouthNoMouth"));
		return;
	}

	ImGui::TextDisabled(locText("oscPanel.mouthHeaderFmt"), (int)m_mouthOutputs.size(),
						locText(face.visemesPresent ? "oscPanel.mouthFromVisemes" : "oscPanel.mouthFromArkit"));
	std::array<float, ARKIT_BLENDSHAPE_COUNT> blendshapes{};
	if (face.present)
		blendshapes= face.blendshapes;
	m_faceMap->evaluate(blendshapes, face.present && face.visemesPresent ? &face.visemes : nullptr, m_outputValues);

	m_values.resize(m_mouthOutputs.size());
	for (size_t index= 0; index < m_mouthOutputs.size(); ++index)
		m_values[index]= m_outputValues[(size_t)m_mouthOutputs[index]];
	struct Context
	{
		const AvatarFaceMap* faceMap;
		const std::vector<int>* outputs;
	};
	const Context context{m_faceMap.get(), &m_mouthOutputs};
	drawWeightTable(
		"avatarMouth", (int)m_mouthOutputs.size(),
		[](int index, const void* raw) -> const char* {
			const Context* c= (const Context*)raw;
			return c->faceMap->getOutputs()[(size_t)(*c->outputs)[(size_t)index]].name.c_str();
		},
		m_values.data(), &context);
}
