#include "NeutralCaptureControl.h"

#include "glm/trigonometric.hpp"

#include "AppConfig.h"
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
	direct.neutralPresent= capture.direct.neutral.present;
	direct.neutralTorso= capture.direct.neutral.torso;
	direct.neutralShoulderMidpoint= capture.direct.neutral.shoulderMidpoint;
	direct.neutralHead= capture.direct.neutral.head;
	direct.neutralWrist= capture.direct.neutral.wrist;
	direct.neutralForearmTwistRadians= capture.direct.neutral.forearmTwist;
	if (capture.direct.bTiltMeasured)
		direct.cameraTiltDegrees= glm::degrees(capture.direct.cameraTiltRadians);
	m_bTiltMeasured= capture.direct.bTiltMeasured;

	// The finger zero, a side at a time: a hand out of frame keeps its old zero
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (!capture.rest.bCaptured[sideIndex])
			continue;
		config->fusedRestAngles.angles[sideIndex]= capture.rest.angles[sideIndex];
		config->fusedRestAngles.present[sideIndex]= true;
	}
	return true;
}
