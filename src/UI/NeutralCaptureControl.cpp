#include "NeutralCaptureControl.h"

#include "imgui.h"

#include "AppConfig.h"
#include "LocText.h"
#include "TrackingTypes.h"
#include "VisionThread.h"

static const ImVec4 k_colorInView(0.4f, 1.f, 0.5f, 1.f);

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

	// The finger zero, a side at a time: a hand out of view keeps its old zero
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		m_bHandZeroed[sideIndex]= capture.rest.bCaptured[sideIndex];
		if (!capture.rest.bCaptured[sideIndex])
			continue;
		config->fusedRestAngles.angles[sideIndex]= capture.rest.angles[sideIndex];
		config->fusedRestAngles.present[sideIndex]= true;
	}
	m_bCapturedThisSession= true;
	return true;
}

void NeutralCaptureControl::drawHandLines(const TrackingFrameResult& latest)
{
	static const char* const k_inViewKeys[2]= {"trackingPanel.directLeftHandInView",
											   "trackingPanel.directRightHandInView"};
	static const char* const k_outKeys[2]= {"trackingPanel.directLeftHandOut", "trackingPanel.directRightHandOut"};
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (latest.poses[sideIndex].tracked)
			ImGui::TextColored(k_colorInView, "%s", locText(k_inViewKeys[sideIndex]));
		else
			ImGui::TextDisabled("%s", locText(k_outKeys[sideIndex]));
	}
}

void NeutralCaptureControl::drawStatus(const DirectConfig& direct) const
{
	if (!direct.neutralPresent)
	{
		ImGui::TextDisabled("%s", locText("trackingPanel.directStatusNone"));
		return;
	}
	if (!m_bCapturedThisSession)
	{
		ImGui::TextDisabled("%s", locText("trackingPanel.directStatusCaptured"));
		return;
	}
	const char* zeroedKey= "trackingPanel.directZeroedNone";
	if (m_bHandZeroed[0] && m_bHandZeroed[1])
		zeroedKey= "trackingPanel.directZeroedBoth";
	else if (m_bHandZeroed[0])
		zeroedKey= "trackingPanel.directZeroedLeft";
	else if (m_bHandZeroed[1])
		zeroedKey= "trackingPanel.directZeroedRight";
	ImGui::TextDisabled(locText("trackingPanel.directStatusCapturedFmt"), locText(zeroedKey));
}
