#include "NeutralCaptureControl.h"

#include "imgui.h"

#include "AppConfig.h"
#include "LocText.h"
#include "VisionThread.h"

bool NeutralCaptureControl::update(float dtSeconds, VisionThread* visionThread, AppConfig* config)
{
	if (m_countdownSeconds > 0.f)
	{
		m_countdownSeconds-= dtSeconds;
		if (m_countdownSeconds <= 0.f)
		{
			m_countdownSeconds= 0.f;
			visionThread->requestNeutralCapture();
		}
	}

	VisionThread::NeutralCapture capture;
	if (!visionThread->fetchNeutralCapture(capture))
		return false;

	DirectConfig& direct= config->direct;
	direct.neutralPresent= capture.neutral.present;
	direct.neutralTorso= capture.neutral.torso;
	direct.neutralShoulderMidpoint= capture.neutral.shoulderMidpoint;
	direct.neutralHead= capture.neutral.head;
	m_bCapturedThisSession= true;
	return true;
}

void NeutralCaptureControl::drawStatus(const DirectConfig& direct) const
{
	if (!direct.neutralPresent)
		ImGui::TextDisabled("%s", locText("trackingPanel.directStatusNone"));
	else if (m_bCapturedThisSession)
		ImGui::TextDisabled("%s", locText("trackingPanel.directStatusCapturedNow"));
	else
		ImGui::TextDisabled("%s", locText("trackingPanel.directStatusCaptured"));
}
