#include "AudioPanel.h"

#include <algorithm>
#include <array>
#include <cstdio>

#include "imgui.h"

#include "App.h"
#include "AppConfig.h"
#include "AudioService.h"
#include "AudioTypes.h"
#include "AvatarFaceMap.h"
#include "AvatarTypes.h"
#include "FaceService.h"
#include "LocText.h"
#include "SteadyClock.h"
#include "TrackingTypes.h"

namespace
{
const char* const k_backendNames[]= {"wav2arkit", "mfcc"};
const char* const k_backendLabelKeys[]= {"audioPanel.backendWav2arkit", "audioPanel.backendMfcc"};
constexpr float k_meterFloorDbfs= -60.f;
} // namespace

AudioPanel::AudioPanel(App* app, AppConfig* config)
	: m_app(app)
	, m_config(config)
{
}

void AudioPanel::refreshDeviceList()
{
	m_deviceNames.clear();
	if (AudioService* audioService= m_app->getAudioService())
		m_deviceNames= audioService->listCaptureDevices();
	m_bDeviceListFetched= true;
}

AudioPanel::DrawResult AudioPanel::draw(bool* pOpen, bool bWizardActive, const TrackingFrameResult& shownFused,
										bool bReplayView)
{
	DrawResult result;
	if (!ImGui::Begin(locWindowTitle("windows.audio"), pOpen))
	{
		ImGui::End();
		return result;
	}

	if (!m_bDeviceListFetched)
		refreshDeviceList();

	drawControls(bWizardActive, result);
	ImGui::Separator();
	drawStatus();
	ImGui::Separator();
	drawWeights(shownFused, bReplayView);
	ImGui::Separator();
	drawAvatarExpressions(shownFused);

	ImGui::End();
	return result;
}

namespace
{
// A table of names and bars, two per row
void drawWeightTable(const char* id, int count, const char* (*name)(int, const void*), const float* values,
					 const void* context)
{
	if (!ImGui::BeginTable(id, 4, ImGuiTableFlags_SizingStretchProp))
		return;
	ImGui::TableSetupColumn("nameA", ImGuiTableColumnFlags_WidthFixed, 150.f);
	ImGui::TableSetupColumn("barA", ImGuiTableColumnFlags_WidthStretch);
	ImGui::TableSetupColumn("nameB", ImGuiTableColumnFlags_WidthFixed, 150.f);
	ImGui::TableSetupColumn("barB", ImGuiTableColumnFlags_WidthStretch);
	const int rows= (count + 1) / 2;
	char overlay[16];
	for (int row= 0; row < rows; ++row)
	{
		ImGui::TableNextRow();
		for (int half= 0; half < 2; ++half)
		{
			const int index= row + half * rows;
			ImGui::TableSetColumnIndex(half * 2);
			if (index >= count)
				continue;
			ImGui::TextDisabled("%s", name(index, context));
			ImGui::TableSetColumnIndex(half * 2 + 1);
			const float weight= std::clamp(values[index], 0.f, 1.f);
			snprintf(overlay, sizeof(overlay), "%.2f", weight);
			ImGui::ProgressBar(weight, ImVec2(-FLT_MIN, 0.f), overlay);
		}
	}
	ImGui::EndTable();
}
} // namespace

void AudioPanel::drawAvatarExpressions(const TrackingFrameResult& shownFused)
{
	const std::shared_ptr<const AvatarModel> model= m_app->getAvatarModel();
	if (model == nullptr)
	{
		ImGui::TextDisabled("%s", locText("audioPanel.avatarNone"));
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
			bool bMouth= false;
			for (const std::vector<AvatarFaceMap::Term>& group : outputs[outputIndex].groups)
				for (const AvatarFaceMap::Term& outputTerm : group)
					bMouth|= mouthColumnFromArkitIndex(outputTerm.arkitIndex) >= 0;
			if (bMouth)
				m_mouthOutputs.push_back((int)outputIndex);
		}
	}
	if (m_mouthOutputs.empty())
	{
		ImGui::TextDisabled("%s", locText("audioPanel.avatarNoMouth"));
		return;
	}

	ImGui::TextDisabled(locText("audioPanel.avatarHeaderFmt"), (int)m_mouthOutputs.size());
	std::array<float, ARKIT_BLENDSHAPE_COUNT> face{};
	if (shownFused.face.present)
		face= shownFused.face.blendshapes;
	m_faceMap->evaluate(face, m_outputValues);

	std::vector<float> values(m_mouthOutputs.size());
	for (size_t index= 0; index < m_mouthOutputs.size(); ++index)
		values[index]= m_outputValues[(size_t)m_mouthOutputs[index]];
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
		values.data(), &context);
}

void AudioPanel::drawWeights(const TrackingFrameResult& shownFused, bool bReplayView)
{
	if (bReplayView)
		ImGui::TextDisabled("%s", locText("audioPanel.weightsReplay"));
	else if (!shownFused.face.present)
		ImGui::TextDisabled("%s", locText("audioPanel.weightsNone"));
	else
		ImGui::TextDisabled("%s", locText("audioPanel.weightsHeader"));

	const MouthFrame mouth= shownFused.face.present ? mouthFrameFromFace(shownFused.face.blendshapes) : MouthFrame();
	drawWeightTable(
		"mouthWeights", MOUTH_COLUMN_COUNT, [](int index, const void*) { return mouthColumnName(index); },
		mouth.weights.data(), nullptr);
}

void AudioPanel::drawControls(bool bWizardActive, DrawResult& outResult)
{
	AudioConfig& audio= m_config->audio;
	bool bChanged= false;

	bChanged|= ImGui::Checkbox(locLabel("audioPanel.enableCheckbox"), &audio.enabled);
	ImGui::SetItemTooltip("%s", locText("audioPanel.enableTooltip"));

	ImGui::BeginDisabled(!audio.enabled);

	int backendIndex= audio.backend == k_backendNames[1] ? 1 : 0;
	if (ImGui::BeginCombo(locLabel("audioPanel.backendCombo"), locText(k_backendLabelKeys[backendIndex])))
	{
		for (int index= 0; index < 2; ++index)
		{
			if (ImGui::Selectable(locLabel(k_backendLabelKeys[index]), index == backendIndex) && index != backendIndex)
			{
				audio.backend= k_backendNames[index];
				bChanged= true;
			}
		}
		ImGui::EndCombo();
	}
	ImGui::SetItemTooltip("%s", locText("audioPanel.backendTooltip"));

	const char* currentDevice= audio.deviceName.empty() ? locText("audioPanel.deviceDefault") : audio.deviceName.c_str();
	if (ImGui::BeginCombo(locLabel("audioPanel.deviceCombo"), currentDevice))
	{
		if (ImGui::Selectable(locLabel("audioPanel.deviceDefault"), audio.deviceName.empty()) && !audio.deviceName.empty())
		{
			audio.deviceName.clear();
			bChanged= true;
		}
		for (const std::string& name : m_deviceNames)
		{
			if (ImGui::Selectable(name.c_str(), name == audio.deviceName) && name != audio.deviceName)
			{
				audio.deviceName= name;
				bChanged= true;
			}
		}
		ImGui::EndCombo();
	}
	ImGui::SameLine();
	if (ImGui::Button(locLabel("audioPanel.refreshButton")))
		refreshDeviceList();

	// The meter reads the newest hop's level against the gate
	float levelDbfs= -100.f;
	bool bGateOpen= false;
	bool bDeviceOpen= false;
	if (AudioService* audioService= m_app->getAudioService())
	{
		const AudioServiceStatus status= audioService->getStatus();
		levelDbfs= status.levelDbfs;
		bGateOpen= status.gateOpen;
		bDeviceOpen= status.deviceOpen;
	}
	const float fraction= std::clamp((levelDbfs - k_meterFloorDbfs) / -k_meterFloorDbfs, 0.f, 1.f);
	const std::string levelText= locFormat("audioPanel.levelFmt", levelDbfs);
	ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
						  bGateOpen ? ImVec4(0.4f, 0.8f, 0.4f, 1.f) : ImVec4(0.5f, 0.5f, 0.5f, 1.f));
	ImGui::ProgressBar(fraction, ImVec2(-FLT_MIN, 0.f), levelText.c_str());
	ImGui::PopStyleColor();

	bChanged|= ImGui::SliderFloat(locLabel("audioPanel.gateSlider"), &audio.gateDbfs, -80.f, 0.f, "%.0f dBFS");
	ImGui::SetItemTooltip("%s", locText("audioPanel.gateTooltip"));
	bChanged|= ImGui::SliderFloat(locLabel("audioPanel.gainSlider"), &audio.gainDb, -20.f, 20.f, "%.0f dB");
	ImGui::SetItemTooltip("%s", locText("audioPanel.gainTooltip"));

	if (audio.backend == k_backendNames[1])
	{
		if (audio.profilePath.empty())
			ImGui::TextDisabled("%s", locText("audioPanel.profileShipped"));
		else
			ImGui::TextDisabled(locText("audioPanel.profileFmt"), audio.profilePath.c_str());

		// The wizard needs an open device, which the service only has while
		// the stage is enabled
		ImGui::BeginDisabled(bWizardActive || !bDeviceOpen);
		if (ImGui::Button(locLabel("audioPanel.calibrateButton")))
			outResult.bLaunchAudioWizard= true;
		ImGui::EndDisabled();
		ImGui::SetItemTooltip("%s", locText("audioPanel.calibrateTooltip"));
		if (!audio.profilePath.empty())
		{
			ImGui::SameLine();
			if (ImGui::Button(locLabel("audioPanel.useShippedButton")))
			{
				audio.profilePath.clear();
				bChanged= true;
			}
		}
	}

	ImGui::EndDisabled();

	if (bChanged)
	{
		m_config->markDirty();
		m_app->applyAudioConfig();
	}
}

void AudioPanel::drawStatus()
{
	AudioService* audioService= m_app->getAudioService();
	if (audioService == nullptr || !m_config->audio.enabled)
	{
		ImGui::TextDisabled("%s", locText("audioPanel.statusSourceOff"));
		return;
	}
	const AudioServiceStatus status= audioService->getStatus();

	if (status.deviceOpen)
		ImGui::TextDisabled(locText("audioPanel.statusDeviceOpenFmt"), status.deviceName.c_str(), status.deviceSampleRate);
	else
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), "%s", locText("audioPanel.statusDeviceClosed"));

	if (status.backendReady)
		ImGui::TextDisabled(locText("audioPanel.statusBackendReadyFmt"), status.backendName.c_str(), status.inferMsEma);
	else if (!status.backendName.empty())
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), locText("audioPanel.statusBackendNotReadyFmt"),
						   status.backendName.c_str(), status.backendReason.c_str());

	// Who owns the mouth: the phone while it streams a tracked face, else
	// the microphone when its gate is open
	const FaceService* faceService= m_app->getFaceService();
	FaceSample faceSample;
	const bool bPhone= faceService != nullptr && m_config->face.enabled && faceService->isStreaming(steadyNowMs()) &&
					   faceService->getLatestSample(faceSample) && faceSample.faceTracked;
	const char* sourceKey= "audioPanel.statusSourceOff";
	if (bPhone)
		sourceKey= "audioPanel.statusSourcePhone";
	else if (status.deviceOpen && status.backendReady)
		sourceKey= status.gateOpen ? "audioPanel.statusSourceAudio" : "audioPanel.statusSourceGated";
	ImGui::Text("%s", locText(sourceKey));

	if (status.droppedBlocks > 0)
		ImGui::TextDisabled(locText("audioPanel.droppedFmt"), (unsigned long long)status.droppedBlocks);
}
