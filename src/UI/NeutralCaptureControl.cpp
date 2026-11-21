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
			visionThread->requestNeutralCapture(config->frontCamera.cameraIndex);
		}
	}

	VisionThread::NeutralCapture capture;
	if (!visionThread->fetchNeutralCapture(capture))
		return false;
	// A capture requested through a camera that has since lost the front-camera
	// role is not this camera's neutral
	if (capture.cameraIndex != config->frontCamera.cameraIndex)
		return false;

	FrontCameraConfig& frontCamera= config->frontCamera;
	frontCamera.neutralPresent= capture.neutral.present;
	frontCamera.neutralTorso= capture.neutral.torso;
	frontCamera.neutralShoulderMidpoint= capture.neutral.shoulderMidpoint;
	frontCamera.neutralHead= capture.neutral.head;
	frontCamera.neutralFacePresent= capture.neutral.facePresent;
	frontCamera.neutralFace= capture.neutral.face;
	m_bCapturedThisSession= true;
	return true;
}

void NeutralCaptureControl::drawStatus(const FrontCameraConfig& frontCamera) const
{
	if (!frontCamera.neutralPresent)
		ImGui::TextDisabled("%s", locText("trackingPanel.frontCameraStatusNone"));
	else if (m_bCapturedThisSession)
		ImGui::TextDisabled("%s", locText("trackingPanel.frontCameraStatusCapturedNow"));
	else
		ImGui::TextDisabled("%s", locText("trackingPanel.frontCameraStatusCaptured"));
}
