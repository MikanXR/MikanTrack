#pragma once

class AppConfig;
class VisionThread;

// The direct retarget's neutral capture as the Tracking panel and the guided
// setup share it: a settle countdown, the capture request to the vision
// thread at zero, then the landed capture written into the project: the
// neutral references and the camera tilt (when the hips let it be measured)
// into the direct config, and the hand angles the direct camera saw at the
// same instant as the finger rest zero. Drawing stays with the caller.
class NeutralCaptureControl
{
public:
	static constexpr float k_countdownSeconds= 3.f;

	// Arms the countdown; update() runs it down
	void start() { m_countdownSeconds= k_countdownSeconds; }
	// Cancels a running countdown (a request already sent still lands)
	void reset() { m_countdownSeconds= 0.f; }

	// Counts down with the frame's dt, sends the request at zero, then polls
	// for the landed capture and writes it into the config. True on the frame
	// a capture was applied, so the caller marks the config dirty and
	// refreshes the vision thread.
	bool update(float dtSeconds, VisionThread* visionThread, AppConfig* config);

	bool isCountingDown() const { return m_countdownSeconds > 0.f; }
	float getRemainingSeconds() const { return m_countdownSeconds; }
	// Whether the last applied capture measured the camera tilt (else the
	// configured tilt was kept)
	bool wasTiltMeasured() const { return m_bTiltMeasured; }

private:
	float m_countdownSeconds= 0.f;
	bool m_bTiltMeasured= false;
};
