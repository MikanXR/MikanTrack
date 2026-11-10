#pragma once

#include <cstdint>
#include <string>
#include <vector>

class App;
class AppConfig;
struct TrackingFrameResult;

// What the microphone's mouth stage reported for the shown frame. A recording
// carries the switch, the device being open, the backend being ready, the
// gate and the level; the rest is live only and unset in a replay.
struct MicrophoneReadout
{
	bool bAvailable= false; // false: a replayed frame from a take that recorded no stage state
	bool bReplay= false;
	bool bEnabled= false;
	bool bDeviceOpen= false;
	bool bBackendReady= false;
	bool bGateOpen= false;
	float levelDbfs= -100.f;
	// The phone streams a tracked face, so it owns the mouth under the Auto
	// face profile
	bool bPhoneOwnsMouth= false;
	// Live only
	std::string deviceName;
	int deviceSampleRate= 0;
	std::string backendInfo;
	std::string backendReason;
	float inferMsEma= 0.f;
	uint64_t droppedBlocks= 0;
};

// The microphone mouth stage's sections, which the Inspector hosts: the
// device and the profile with its Calibrate button (inputs), the enable, gate,
// gain, model and smoothing (settings), and the level meter, a status block
// (the device, the model, the run time, which source owns the mouth) and the
// viseme weights the stage is putting out (diagnostics). Edits go to the
// persisted AudioConfig and through App::applyAudioConfig to the service.
// Owns the capture device list.
class AudioPanel
{
public:
	// Launch requests raised by the draws, for MainWindow to act on
	struct DrawResult
	{
		bool bLaunchAudioWizard= false;
	};

	AudioPanel(App* app, AppConfig* config);

	// The inputs: the device combo with its refresh, which fetches the
	// capture device list the first time it draws, and for the MFCC model
	// the profile with Calibrate, Import and Clear. bWizardActive disables
	// Calibrate while another wizard runs; a Calibrate press sets
	// outResult.bLaunchAudioWizard.
	void drawDeviceControls();
	void drawProfileControls(bool bWizardActive, DrawResult& outResult);
	// The settings: the enable, the gate and gain, the model and its
	// smoothing
	void drawSettingsControls();
	// The diagnostics. The meter and the status read the shown frame's
	// readout. For a take without one the meter says not recorded and the
	// status draws nothing.
	void drawLevelMeter(const MicrophoneReadout& readout);
	void drawStatus(const MicrophoneReadout& readout);
	// The shown frame's viseme weights as bars, so what the microphone is
	// putting out can be read live and during playback
	void drawWeights(const TrackingFrameResult& shownFused, bool bReplayView);

private:
	void refreshDeviceList();
	void commitAudioConfigChange();

	App* m_app;
	AppConfig* m_config;

	std::vector<std::string> m_deviceNames;
	bool m_bDeviceListFetched= false;
};
