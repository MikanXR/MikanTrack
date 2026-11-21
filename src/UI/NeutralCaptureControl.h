#pragma once

class AppConfig;
class VisionThread;
struct FrontCameraConfig;

// The front-camera retarget's neutral capture as the Front-camera solver stage and the
// guided setup share it: a settle countdown, the capture request to the vision
// thread at zero, then the landed capture written into the project as the
// front-camera config's neutral references. The status is drawn here too, since
// both callers show the same; the countdown and the buttons stay with them.
class NeutralCaptureControl
{
public:
	static constexpr float k_countdownSeconds= 3.f;

	// Arms the countdown; update() runs it down
	void start() { m_countdownSeconds= k_countdownSeconds; }
	// Cancels a running countdown. A request already sent still lands, and
	// is dropped when the front camera has changed since.
	void reset() { m_countdownSeconds= 0.f; }

	// Counts down with the frame's dt, sends the request at zero, then polls
	// for the landed capture and writes it into the config. True on the frame
	// a capture was applied, so the caller marks the config dirty and
	// refreshes the vision thread.
	bool update(float dtSeconds, VisionThread* visionThread, AppConfig* config);

	bool isCountingDown() const { return m_countdownSeconds > 0.f; }
	float getRemainingSeconds() const { return m_countdownSeconds; }

	// No neutral, a neutral loaded with the project, or one captured this
	// session
	void drawStatus(const FrontCameraConfig& frontCamera) const;

private:
	float m_countdownSeconds= 0.f;
	bool m_bCapturedThisSession= false;
};
