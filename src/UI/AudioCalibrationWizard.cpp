#include "AudioCalibrationWizard.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>

#include "imgui.h"

#include "LocText.h"

#include "App.h"
#include "AppConfig.h"
#include "AudioService.h"
#include "AudioTypes.h"
#include "MouthChain.h"
#include "PathUtils.h"

namespace
{
// A vowel prompt: its label, the uLipSync phoneme its frames are recorded
// under, and the viseme that phoneme drives
struct VowelPrompt
{
	const char* labelKey;
	const char* phoneme;
	eViseme viseme;
};

const VowelPrompt k_vowels[AudioCalibrationWizard::k_vowelCount]= {
	{"audioWizard.promptAa", "A", eViseme::aa}, {"audioWizard.promptI", "I", eViseme::I},
	{"audioWizard.promptU", "U", eViseme::U},   {"audioWizard.promptE", "E", eViseme::E},
	{"audioWizard.promptO", "O", eViseme::O},
};
// uLipSync's noise phoneme, what the silence prompt teaches
const char* const k_noisePhoneme= "-";

// A frame count as seconds of voice
float framesToSeconds(size_t frameCount)
{
	return (float)frameCount / MOUTH_FRAME_RATE;
}

const char* promptLabel(int promptIndex)
{
	return promptIndex < AudioCalibrationWizard::k_vowelCount ? locText(k_vowels[promptIndex].labelKey)
															   : locText("audioWizard.promptSilence");
}
} // namespace

AudioCalibrationWizard::AudioCalibrationWizard(App* app, AppConfig* config)
	: m_app(app)
	, m_config(config)
{
}

std::string AudioCalibrationWizard::makeProfilePath(const AppConfig& config)
{
	return PathUtils::pathToUtf8(config.getProjectDirectory() / "audio" / "profile.json");
}

void AudioCalibrationWizard::enter()
{
	m_bActive= true;
	m_bWantsClose= false;
	m_result= eWizardResult::None;
	m_state= eState::Intro;
	m_phase= ePhase::Ready;
	m_promptIndex= 0;
	m_countdown.cancel();
	for (int index= 0; index < k_vowelCount; ++index)
		m_hitRate[index]= 0.f;
	for (std::vector<MfccVector>& frames : m_promptFrames)
		frames.clear();
	for (PromptResult& result : m_results)
		result= PromptResult();
	m_profile= LipSyncProfile();
	m_suggestedGateDbfs= m_config->audio.gateDbfs;
	m_profileError.clear();
}

void AudioCalibrationWizard::exit()
{
	// Leaving mid-prompt must not leave the service collecting
	if (AudioService* audioService= m_app->getAudioService())
	{
		std::vector<float> discarded;
		if (audioService->isCapturing())
			audioService->stopCapture(discarded);
	}
	m_bActive= false;
}

void AudioCalibrationWizard::cancel()
{
	m_result= eWizardResult::Cancelled;
	m_bWantsClose= true;
}

void AudioCalibrationWizard::beginRecording()
{
	AudioService* audioService= m_app->getAudioService();
	if (audioService == nullptr || !audioService->startCapture(k_recordSeconds + 1.f))
	{
		m_results[m_promptIndex].errorKey= "audioWizard.errorNoCapture";
		m_phase= ePhase::Ready;
		return;
	}
	m_countdown.start(k_recordSeconds);
	m_phase= ePhase::Recording;
}

void AudioCalibrationWizard::finishRecording()
{
	std::vector<float> samples;
	if (AudioService* audioService= m_app->getAudioService())
		audioService->stopCapture(samples);
	m_phase= ePhase::Ready;

	PromptResult& result= m_results[m_promptIndex];
	result= PromptResult();
	if (samples.empty())
	{
		result.errorKey= "audioWizard.errorNoCapture";
		return;
	}

	std::vector<MfccVector>& frames= m_promptFrames[m_promptIndex];
	if (m_promptIndex < k_vowelCount)
	{
		// The frames the live gate would let through, so the template is
		// the held vowel and not the breaths around it
		calibrationFrames(samples.data(), samples.size(), m_config->audio.gateDbfs, m_extractor, frames);
		result.voiceSeconds= framesToSeconds(frames.size());
		if ((int)frames.size() < k_minFrames)
		{
			result.errorKey= "audioWizard.errorTooQuietFmt";
			return;
		}
	}
	else
	{
		// Every frame of the room: the noise phoneme, and the gate
		calibrationFrames(samples.data(), samples.size(), -100.f, m_extractor, frames);
		m_suggestedGateDbfs= suggestGateDbfs(samples.data(), samples.size(), m_config->audio.gainDb);
		result.gateDbfs= m_suggestedGateDbfs;
	}
	result.bCaptured= true;

	if (++m_promptIndex < k_promptCount)
		return;
	m_promptIndex= k_promptCount - 1;
	buildProfile();
	m_state= eState::Review;
}

void AudioCalibrationWizard::buildProfile()
{
	// uLipSync's defaults, and every frame kept: the data count is what a
	// uLipSync import would trim each phoneme to
	m_profile= LipSyncProfile();
	size_t largest= 1;
	for (const std::vector<MfccVector>& frames : m_promptFrames)
		largest= std::max(largest, frames.size());
	m_profile.mfccDataCount= (int)largest;
	for (int index= 0; index < k_vowelCount; ++index)
		m_profile.addPhoneme(k_vowels[index].phoneme, m_promptFrames[index]);
	m_profile.addPhoneme(k_noisePhoneme, m_promptFrames[k_vowelCount]);
	m_profile.finalize();

	// Each vowel's own frames against the profile: the share whose best
	// phoneme is the vowel itself, which drops where two vowels overlap
	LipSyncScores scores;
	for (int index= 0; index < k_vowelCount; ++index)
	{
		const std::vector<MfccVector>& frames= m_promptFrames[index];
		int hits= 0;
		for (const MfccVector& frame : frames)
		{
			m_profile.score(frame, scores);
			hits+= scores.mainViseme == (int)k_vowels[index].viseme ? 1 : 0;
		}
		m_hitRate[index]= frames.empty() ? 0.f : (float)hits / (float)frames.size();
	}
}

bool AudioCalibrationWizard::writeProfile(std::string& outError)
{
	const std::string profilePath= makeProfilePath(*m_config);
	if (!m_profile.save(PathUtils::utf8ToPath(profilePath), outError))
		return false;

	// A fresh calibration is meant to drive the mouth
	m_config->audio.model= eMouthModel::Mfcc;
	m_config->audio.profilePath= profilePath;
	m_config->audio.gateDbfs= m_suggestedGateDbfs;
	m_config->markDirty();
	m_app->applyAudioConfig();
	return true;
}

bool AudioCalibrationWizard::update(float deltaSeconds)
{
	if (!m_bActive)
		return false;

	ImGui::SetNextWindowSize(ImVec2(460, 0), ImGuiCond_Appearing);
	if (!ImGui::Begin(locWindowTitle("windows.audioWizard"), nullptr, ImGuiWindowFlags_NoCollapse))
	{
		ImGui::End();
		return true;
	}

	switch (m_state)
	{
		case eState::Intro:
			drawIntro();
			break;
		case eState::Prompts:
			drawPrompts(deltaSeconds);
			break;
		case eState::Review:
			drawReview();
			break;
	}

	ImGui::End();
	return !m_bWantsClose;
}

void AudioCalibrationWizard::drawIntro()
{
	ImGui::TextWrapped("%s", locText("audioWizard.intro"));
	ImGui::Spacing();
	ImGui::TextWrapped("%s", locText("audioWizard.introSetup"));
	ImGui::Spacing();

	bool bDeviceOpen= false;
	if (AudioService* audioService= m_app->getAudioService())
	{
		const AudioServiceStatus status= audioService->getStatus();
		bDeviceOpen= status.deviceOpen;
		if (bDeviceOpen)
			ImGui::TextDisabled(locText("audioWizard.deviceOpenFmt"), status.deviceName.c_str());
	}
	if (!bDeviceOpen)
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), "%s", locText("audioWizard.deviceNotOpen"));
	ImGui::Spacing();

	ImGui::BeginDisabled(!bDeviceOpen);
	if (ImGui::Button(locLabel("common.start"), ImVec2(180, 0)))
	{
		m_state= eState::Prompts;
		m_phase= ePhase::Ready;
		m_promptIndex= 0;
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button(locLabel("common.cancel"), ImVec2(120, 0)))
		cancel();
}

void AudioCalibrationWizard::drawPrompts(float deltaSeconds)
{
	ImGui::TextWrapped("%s", locText("audioWizard.promptsIntro"));
	ImGui::Spacing();

	for (int index= 0; index < k_promptCount; ++index)
	{
		const PromptResult& result= m_results[index];
		const bool bCurrent= index == m_promptIndex;
		const char* marker= result.bCaptured ? "[ok]" : bCurrent ? "[>>]" : "[..]";
		ImVec4 color= bCurrent ? ImVec4(1.f, 1.f, 1.f, 1.f) : ImVec4(0.6f, 0.6f, 0.6f, 1.f);
		if (result.bCaptured)
			color= ImVec4(0.4f, 1.f, 0.4f, 1.f);
		std::string status= locText("audioWizard.pendingStatus");
		if (result.bCaptured)
		{
			status= index < k_vowelCount ? locFormat("audioWizard.capturedHopsFmt", result.voiceSeconds)
										 : locFormat("audioWizard.capturedSilenceFmt", result.gateDbfs);
		}
		ImGui::TextColored(color, "%s %s: %s", marker, promptLabel(index), status.c_str());
	}

	ImGui::Spacing();
	const std::string header= locFormat("audioWizard.promptFmt", m_promptIndex + 1, k_promptCount);
	ImGui::SeparatorText(header.c_str());
	if (m_promptIndex < k_vowelCount)
		ImGui::TextWrapped(locText("audioWizard.holdFmt"), promptLabel(m_promptIndex));
	else
		ImGui::TextWrapped("%s", locText("audioWizard.holdSilence"));
	ImGui::Spacing();

	switch (m_phase)
	{
		case ePhase::Ready:
		{
			const PromptResult& result= m_results[m_promptIndex];
			if (!result.errorKey.empty())
			{
				const std::string error= result.errorKey == "audioWizard.errorTooQuietFmt"
											 ? locFormat(result.errorKey.c_str(), result.voiceSeconds)
											 : std::string(locText(result.errorKey.c_str()));
				ImGui::TextColored(ImVec4(1.f, 0.85f, 0.3f, 1.f), "%s", error.c_str());
				ImGui::Spacing();
			}
			if (ImGui::Button(locLabel("audioWizard.recordButton"), ImVec2(180, 0)))
			{
				m_countdown.start(k_countdownSeconds);
				m_phase= ePhase::Countdown;
			}
			break;
		}

		case ePhase::Countdown:
		{
			const std::string overlay=
				locFormat("audioWizard.countdownFmt", (int)std::ceil(m_countdown.getRemainingSeconds()));
			ImGui::ProgressBar(m_countdown.getProgress(), ImVec2(180, 0), overlay.c_str());
			if (m_countdown.update(deltaSeconds))
				beginRecording();
			break;
		}

		case ePhase::Recording:
		{
			ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.9f, 0.3f, 0.3f, 1.f));
			ImGui::ProgressBar(m_countdown.getProgress(), ImVec2(180, 0), locText("audioWizard.recordingOverlay"));
			ImGui::PopStyleColor();
			if (AudioService* audioService= m_app->getAudioService())
			{
				const AudioServiceStatus status= audioService->getStatus();
				ImGui::SameLine();
				ImGui::TextDisabled(locText("audioWizard.levelFmt"), status.levelDbfs);
			}
			if (m_countdown.update(deltaSeconds))
				finishRecording();
			break;
		}
	}

	ImGui::SameLine();
	if (ImGui::Button(locLabel("common.cancel"), ImVec2(120, 0)))
		cancel();
}

void AudioCalibrationWizard::drawReview()
{
	const bool bProfileValid= !m_profile.empty();
	if (bProfileValid)
	{
		ImGui::TextWrapped("%s", locText("audioWizard.reviewIntro"));
		ImGui::Spacing();
		for (int index= 0; index < k_vowelCount; ++index)
		{
			const ImVec4 color= m_hitRate[index] >= 0.8f ? ImVec4(0.4f, 1.f, 0.4f, 1.f) : ImVec4(1.f, 0.85f, 0.3f, 1.f);
			ImGui::TextColored(color, locText("audioWizard.reviewRowFmt"), promptLabel(index), m_hitRate[index] * 100.f,
							   m_results[index].voiceSeconds);
		}
		ImGui::Spacing();
		ImGui::TextDisabled(locText("audioWizard.reviewGateFmt"), m_suggestedGateDbfs);
		ImGui::TextDisabled(locText("audioWizard.reviewProfileFmt"), makeProfilePath(*m_config).c_str());
	}
	if (!m_profileError.empty())
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), locText("audioWizard.writeFailedFmt"), m_profileError.c_str());
	ImGui::Spacing();

	ImGui::BeginDisabled(!bProfileValid);
	if (ImGui::Button(locLabel("common.finish"), ImVec2(180, 0)))
	{
		if (writeProfile(m_profileError))
		{
			m_result= eWizardResult::Completed;
			m_bWantsClose= true;
		}
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button(locLabel("audioWizard.redoButton"), ImVec2(120, 0)))
	{
		for (PromptResult& result : m_results)
			result= PromptResult();
		m_profileError.clear();
		m_promptIndex= 0;
		m_phase= ePhase::Ready;
		m_state= eState::Prompts;
	}
	ImGui::SameLine();
	if (ImGui::Button(locLabel("common.cancel"), ImVec2(120, 0)))
		cancel();
}
