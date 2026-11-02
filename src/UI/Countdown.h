#pragma once

// A settle countdown for the captures that need the user in position
// first: armed for a number of seconds, run down with the frame's dt, and
// reporting the one frame it reaches zero
class Countdown
{
public:
	void start(float seconds)
	{
		m_durationSeconds= seconds;
		m_remainingSeconds= seconds;
	}
	void cancel() { m_remainingSeconds= 0.f; }

	// True on the frame the countdown ends
	bool update(float dtSeconds)
	{
		if (m_remainingSeconds <= 0.f)
			return false;
		m_remainingSeconds-= dtSeconds;
		if (m_remainingSeconds > 0.f)
			return false;
		m_remainingSeconds= 0.f;
		return true;
	}

	bool isRunning() const { return m_remainingSeconds > 0.f; }
	float getRemainingSeconds() const { return m_remainingSeconds; }
	// The fraction elapsed, for a progress bar
	float getProgress() const
	{
		return m_durationSeconds > 0.f ? 1.f - m_remainingSeconds / m_durationSeconds : 1.f;
	}

private:
	float m_durationSeconds= 0.f;
	float m_remainingSeconds= 0.f;
};
