#pragma once

#include <string>
#include <vector>

#include "Countdown.h"
#include "LipSyncProfile.h"
#include "MfccFeatures.h"
#include "WizardResult.h"

class App;
class AppConfig;

// The microphone calibration wizard: six prompts of three seconds each
// after a countdown, the five vowels held and then the room's silence. Each
// prompt's capture becomes the MFCC frames of one phoneme of a uLipSync
// profile (A, I, U, E, O, and the noise phoneme from the silence), the
// silence also suggests the gate, and the review scores each vowel's own
// frames against the new profile before it is written beside the project
// and pointed at by AudioConfig::profilePath.
class AudioCalibrationWizard
{
public:
	enum class eState
	{
		Intro,
		Prompts,
		Review,
	};

	// Where the current prompt is
	enum class ePhase
	{
		Ready,
		Countdown,
		Recording,
	};

	static constexpr int k_vowelCount= 5;
	// The vowels, then the silence
	static constexpr int k_promptCount= k_vowelCount + 1;
	static constexpr float k_countdownSeconds= 3.f;
	static constexpr float k_recordSeconds= 3.f;
	// The frames a vowel prompt must yield past the gate, half a second
	static constexpr int k_minFrames= 15;

	AudioCalibrationWizard(App* app, AppConfig* config);

	void enter();
	void exit();
	bool isActive() const { return m_bActive; }
	eWizardResult getResult() const { return m_result; }

	// Returns false once the wizard wants to close
	bool update(float deltaSeconds);

	// The profile a project's calibration is written to
	static std::string makeProfilePath(const AppConfig& config);

private:
	struct PromptResult
	{
		bool bCaptured= false;
		// The localization key of the failure, empty while none
		std::string errorKey;
		// A vowel's frames that passed the gate, as seconds of voice
		float voiceSeconds= 0.f;
		// The silence prompt's suggested gate
		float gateDbfs= 0.f;
	};

	void beginRecording();
	void finishRecording();
	// Builds the profile from the prompts' frames and scores each vowel's
	// own frames back against it
	void buildProfile();
	bool writeProfile(std::string& outError);

	void drawIntro();
	void drawPrompts(float deltaSeconds);
	void drawReview();
	void cancel();

	App* m_app;
	AppConfig* m_config;

	eState m_state= eState::Intro;
	ePhase m_phase= ePhase::Ready;
	int m_promptIndex= 0;
	bool m_bActive= false;
	bool m_bWantsClose= false;
	eWizardResult m_result= eWizardResult::None;

	Countdown m_countdown;
	MfccExtractor m_extractor;
	// The vowels' frames, then the silence prompt's
	std::vector<MfccVector> m_promptFrames[k_promptCount];
	PromptResult m_results[k_promptCount];

	// The review
	LipSyncProfile m_profile;
	float m_hitRate[k_vowelCount]= {};
	float m_suggestedGateDbfs= -45.f;
	std::string m_profileError;
};
