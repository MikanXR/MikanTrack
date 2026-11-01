#pragma once

class AppConfig;
class VisionThread;
struct DirectConfig;
struct TrackingFrameResult;

// The direct retarget's neutral capture as the Tracking panel and the guided
// setup share it: a settle countdown, the capture request to the vision
// thread at zero, then the landed capture written into the project: the
// neutral references into the direct config, and the hand angles the direct
// camera saw at the same instant as the finger rest zero of the hands it
// saw. The per-hand lines and the status are drawn here too, since both
// callers show the same; the countdown and the buttons stay with them.
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

	// Which hands the camera sees now, so the user knows whose finger zero
	// a capture will take. latest: the window's copy of the thread's output.
	static void drawHandLines(const TrackingFrameResult& latest);
	// No neutral, a neutral loaded with the project, or the one captured
	// this session with the hands it zeroed
	void drawStatus(const DirectConfig& direct) const;

private:
	float m_countdownSeconds= 0.f;
	bool m_bCapturedThisSession= false;
	bool m_bHandZeroed[2]= {false, false};
};
