#pragma once

#include <memory>
#include <string>
#include <vector>

class App;
class AppConfig;
class AvatarFaceMap;
struct AvatarModel;
struct TrackingFrameResult;

// The Audio panel: the microphone mouth stage's enable, backend, device,
// level meter with the gate and gain, the mfcc profile with its Calibrate
// button, and a status block (the device, the backend, the run time, which
// source owns the mouth). Edits go to the persisted AudioConfig and through
// App::applyAudioConfig to the service.
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
	// timeline's display frame while its replay view is active), whose face
	// the weights block reads.
	DrawResult draw(bool* pOpen, bool bWizardActive, const TrackingFrameResult& shownFused, bool bReplayView);

private:
	void refreshDeviceList();
	void drawControls(bool bWizardActive, DrawResult& outResult);
	void drawStatus();
	// The mouth columns of the shown face as bars, so the weights reaching
	// the avatar can be read live and during playback, then the avatar's
	// mouth expressions as the face map derives them from that face
	void drawWeights(const TrackingFrameResult& shownFused, bool bReplayView);
	void drawAvatarExpressions(const TrackingFrameResult& shownFused);

	App* m_app;
	AppConfig* m_config;

	std::vector<std::string> m_deviceNames;
	bool m_bDeviceListFetched= false;

	// The loaded avatar's face map, rebuilt when the avatar changes
	std::shared_ptr<const AvatarModel> m_faceMapModel;
	std::shared_ptr<const AvatarFaceMap> m_faceMap;
	// The outputs a mouth column feeds, in face map order
	std::vector<int> m_mouthOutputs;
	std::vector<float> m_outputValues;
};
