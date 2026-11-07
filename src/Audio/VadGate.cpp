#include "VadGate.h"

#include <algorithm>

void VadGate::configure(const VadGateConfig& config)
{
	m_config= config;
	m_config.activeHops= std::max(1, config.activeHops);
	m_config.inactiveHops= std::max(1, config.inactiveHops);
	reset();
}

void VadGate::reset()
{
	m_bActive= false;
	m_preActive= 0;
	m_preInactive= 0;
}

bool VadGate::process(float logEnergy)
{
	if (m_bActive)
	{
		if (logEnergy < m_config.inactiveDb / 10.f)
		{
			if (++m_preInactive >= m_config.inactiveHops)
			{
				m_bActive= false;
				m_preInactive= 0;
				m_preActive= 0;
			}
		}
		else if (m_preInactive > 0)
		{
			--m_preInactive;
		}
	}
	else
	{
		if (logEnergy > m_config.activeDb / 10.f)
		{
			if (++m_preActive >= m_config.activeHops)
			{
				m_bActive= true;
				m_preActive= 0;
				m_preInactive= 0;
			}
		}
		else if (m_preActive > 0)
		{
			--m_preActive;
		}
	}
	return m_bActive;
}
