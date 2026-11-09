#pragma once

#include <string>
#include <vector>

class App;
class AppConfig;
struct TrackingFrameResult;

// The Audio panel: the microphone mouth stage's enable, device, level meter
// with the gate and gain, the profile with its Calibrate button, a status
// block (the device, the model, the run time, which source owns the mouth),
// and the viseme weights the stage is putting out. Edits go to the
// persisted AudioConfig and through App::applyAudioConfig to the service.
class AudioPanel
{
public:
	// Launch requests raised by the draw, for MainWindow to act on
	struct DrawResult
	{
		bool bLaunchAudioWizard= false;
	};

	AudioPanel(App* app, AppConfig* config);

	// bWizardActive disables the Calibrate button while another wizard runs.
	// shownFused is the result the scene is showing (the live output, or the
	// timeline's display frame while its replay view is active), whose
	// visemes the weights block reads.
	DrawResult draw(bool* pOpen, bool bWizardActive, const TrackingFrameResult& shownFused, bool bReplayView);

private:
	void refreshDeviceList();
	void drawControls(bool bWizardActive, DrawResult& outResult);
	void drawStatus();
	// The shown frame's viseme weights as bars, so what the microphone is
	// putting out can be read live and during playback
	void drawWeights(const TrackingFrameResult& shownFused, bool bReplayView);

	App* m_app;
	AppConfig* m_config;

	std::vector<std::string> m_deviceNames;
	bool m_bDeviceListFetched= false;
};
