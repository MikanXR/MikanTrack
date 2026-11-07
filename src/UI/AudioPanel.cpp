#include "AudioPanel.h"

#include <cstdio>

#include "imgui.h"

#include "App.h"
#include "AppConfig.h"
#include "AudioService.h"
#include "AudioTypes.h"
#include "FaceService.h"
#include "LocText.h"
#include "SteadyClock.h"
#include "TrackingTypes.h"
#include "WeightBars.h"

#include "tinyfiledialogs.h"

namespace
{
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

	ImGui::End();
	return result;
}

void AudioPanel::drawWeights(const TrackingFrameResult& shownFused, bool bReplayView)
{
	const bool bVisemes= shownFused.face.present && shownFused.face.visemesPresent;
	if (bReplayView)
		ImGui::TextDisabled("%s", locText("audioPanel.weightsReplay"));
	else if (!bVisemes)
		ImGui::TextDisabled("%s", locText("audioPanel.weightsNone"));
	else
		ImGui::TextDisabled("%s", locText("audioPanel.weightsHeader"));

	const VisemeWeights weights= bVisemes ? shownFused.face.visemes : VisemeWeights{};
	drawWeightTable(
		"visemeWeights", VISEME_COUNT, [](int index, const void*) { return visemeName(index); }, weights.data(), nullptr);
}

void AudioPanel::drawControls(bool bWizardActive, DrawResult& outResult)
{
	AudioConfig& audio= m_config->audio;
	bool bChanged= false;

	bChanged|= ImGui::Checkbox(locLabel("audioPanel.enableCheckbox"), &audio.enabled);
	ImGui::SetItemTooltip("%s", locText("audioPanel.enableTooltip"));

	ImGui::BeginDisabled(!audio.enabled);

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

	if (audio.profilePath.empty())
		ImGui::TextDisabled("%s", locText("audioPanel.profileNone"));
	else
		ImGui::TextDisabled(locText("audioPanel.profileFmt"), audio.profilePath.c_str());

	// The wizard needs an open device, which the service only has while
	// the stage is enabled
	ImGui::BeginDisabled(bWizardActive || !bDeviceOpen);
	if (ImGui::Button(locLabel("audioPanel.calibrateButton")))
		outResult.bLaunchAudioWizard= true;
	ImGui::EndDisabled();
	ImGui::SetItemTooltip("%s", locText("audioPanel.calibrateTooltip"));
	ImGui::SameLine();
	if (ImGui::Button(locLabel("audioPanel.importButton")))
	{
		// Any uLipSync profile, the one VTube Studio exports included
		const char* filters[]= {"*.json"};
		const char* selectedPath= tinyfd_openFileDialog(locText("audioPanel.importButton"), "", 1, filters,
														 "uLipSync profile (*.json)", 0);
		if (selectedPath != nullptr && selectedPath[0] != '\0')
		{
			audio.profilePath= selectedPath;
			bChanged= true;
		}
	}
	ImGui::SetItemTooltip("%s", locText("audioPanel.importTooltip"));
	if (!audio.profilePath.empty())
	{
		ImGui::SameLine();
		if (ImGui::Button(locLabel("audioPanel.clearProfileButton")))
		{
			audio.profilePath.clear();
			bChanged= true;
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

	if (status.backendReady && status.profileLoaded)
		ImGui::TextDisabled(locText("audioPanel.statusBackendReadyFmt"), status.inferMsEma);
	else if (status.backendReady && status.backendReason.empty())
		ImGui::TextDisabled("%s", locText("audioPanel.statusBackendVolumeOnly"));
	else if (!status.backendReason.empty())
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), locText("audioPanel.statusBackendNotReadyFmt"),
						   status.backendReason.c_str());

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
