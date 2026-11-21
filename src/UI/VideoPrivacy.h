#pragma once

#include <memory>
#include <string>
#include <vector>

#include "opencv2/core/mat.hpp"

#include "StreamingSoftware.h"

class GlTexture;
struct VisionPreviewFrame;
enum class eWizardKind;

// Which cameras' video may be shown. A camera's video is hidden unless the
// user turned it on, or a calibration that needs the picture holds a grant
// for it. The user's choice is never stored: it clears with the project,
// with a change of the camera count, and when the camera's slot opens
// another device, so it never carries to a camera the user did not look at.
class VideoPrivacyState
{
public:
	// The cameras and the device each one has open, empty for none. A changed
	// count clears every choice, and a changed name that camera's.
	void setCameras(const std::vector<std::string>& deviceNames);
	// The calibration grants on this frame: one camera (-1 for none), or all
	void setGrants(int grantedCamera, bool bAllGranted);
	void setUserShown(int cameraIndex, bool bShown);
	// Clears every choice and grant
	void reset();

	size_t getCameraCount() const { return m_cameras.size(); }
	bool getUserShown(int cameraIndex) const;
	bool isGranted(int cameraIndex) const;
	bool isVisible(int cameraIndex) const { return getUserShown(cameraIndex) || isGranted(cameraIndex); }
	bool isAnyVisible() const;

private:
	struct Camera
	{
		std::string deviceName;
		bool bUserShown= false;
	};
	std::vector<Camera> m_cameras;
	int m_grantedCamera= -1;
	bool m_bAllGranted= false;
};

// A frame cut down to blockColumns blocks across and drawn back up with hard
// block edges, at the widest whole multiple of blockColumns that fits
// outputWidth. Empty for an empty frame.
cv::Mat makePixelatedPreview(const cv::Mat& bgr, int blockColumns, int outputWidth);

// The camera video rule's state and its prompts: the confirmation a user's
// request to show a camera goes through, with its hold-to-look pixelated
// preview, and the warning in front of a calibration that shows video. Both
// name a streaming program when one is running.
class VideoPrivacy
{
public:
	VideoPrivacy();
	~VideoPrivacy();

	VideoPrivacyState& getState() { return m_state; }
	const VideoPrivacyState& getState() const { return m_state; }

	// Opens the confirmation for a camera. Nothing shows until it is accepted.
	void requestShow(int cameraIndex);
	// Opens the warning for a calibration wizard that shows video
	void requestWizardLaunch(eWizardKind kind, int cameraIndex);
	// Closes whichever prompt is open, accepting nothing
	void dismiss();
	// Clears the choices and closes the prompts, for a closed project
	void reset();

	enum class ePrompt
	{
		None,
		ShowVideo,
		WizardLaunch,
	};
	ePrompt getPrompt() const { return m_prompt; }
	int getPromptCamera() const { return m_promptCamera; }

	// Draws the open prompt. True on the frame a wizard launch was accepted,
	// with the wizard and its camera.
	bool drawPrompts(const std::vector<VisionPreviewFrame>& previews, eWizardKind& outKind, int& outCameraIndex);

	// The running streaming program's name, empty for none. Polls the
	// process list, so it is asked only while there is a warning to show.
	const std::string& getStreamingSoftwareName() { return m_streamingWatch.poll(); }

	// The line a video warning adds while a streaming program runs. Draws
	// nothing for an empty name.
	static void drawStreamingWarning(const std::string& streamingSoftwareName);

private:
	void drawShowVideoPrompt(const std::vector<VisionPreviewFrame>& previews);
	// True on the frame the launch was accepted
	bool drawWizardLaunchPrompt();
	// Closes the open prompt from inside its popup
	void closePrompt();

	VideoPrivacyState m_state;
	StreamingSoftwareWatch m_streamingWatch;

	ePrompt m_prompt= ePrompt::None;
	int m_promptCamera= -1;
	eWizardKind m_promptWizard;
	// A dismissal asked from outside the prompt's own buttons
	bool m_bDismissRequested= false;
	// The pixelated preview, which exists only while its button is held
	std::unique_ptr<GlTexture> m_pixelatedTexture;
};
