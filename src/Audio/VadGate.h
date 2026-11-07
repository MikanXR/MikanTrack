#pragma once

// HeadAudio's energy gate, the voice activity detection in front of the
// mfcc classifier: a hop is active once its level has been above the active
// threshold for the active count of hops, and inactive again once it has
// been below the inactive threshold for the inactive count. Levels are log10
// energies (a tenth of the dB value), the MFCC chain's log energy.
struct VadGateConfig
{
	float activeDb= -40.f;
	float inactiveDb= -50.f;
	// Hops above or below the threshold before the gate switches
	int activeHops= 1;
	int inactiveHops= 1;
};

class VadGate
{
public:
	void configure(const VadGateConfig& config);
	const VadGateConfig& getConfig() const { return m_config; }

	// Back to inactive with the counters cleared
	void reset();

	// Feeds one hop's log energy and returns whether the gate is open after it
	bool process(float logEnergy);
	bool isActive() const { return m_bActive; }

private:
	VadGateConfig m_config;
	bool m_bActive= false;
	int m_preActive= 0;
	int m_preInactive= 0;
};
