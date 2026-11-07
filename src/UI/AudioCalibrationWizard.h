#pragma once

#include <string>
#include <vector>

#include "Countdown.h"
#include "MfccFeatures.h"
#include "VisemeModel.h"
#include "WizardResult.h"

class App;
class AppConfig;

// The microphone calibration wizard: six prompts of three seconds each
// after a countdown, the five vowels of the mfcc backend's vowel visemes
// held and then the room's silence. The vowel captures are fit as Gaussian
// prototypes over the shipped model (the consonants and the silence keep
// theirs), the silence suggests the gate, and the review classifies each
// vowel's own hops against the new profile before it is written beside the
// project and pointed at by AudioConfig::profilePath.
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
		// A vowel's hops that passed the gate, as seconds of voice
		float voiceSeconds= 0.f;
		// The silence prompt's suggested gate
		float gateDbfs= 0.f;
	};

	void beginRecording();
	void finishRecording();
	// Fits the vowels over the shipped model and measures them back; false
	// with the reason when the shipped model cannot be read
	bool buildProfile(std::string& outError);
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
	std::vector<MfccVector> m_vowelVectors[k_vowelCount];
	PromptResult m_results[k_promptCount];

	// The review
	VisemeModel m_profile;
	float m_hitRate[k_vowelCount]= {};
	float m_suggestedGateDbfs= -45.f;
	std::string m_profileError;
};
