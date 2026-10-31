#include "HandCalibrationWizard.h"

#include "imgui.h"

#include "LocText.h"

#include "AppConfig.h"

namespace
{
constexpr float k_countdownSeconds= 3.f;
constexpr float k_boneSampleSeconds= 10.f;
// Per-bone spread above this suggests the capture window was unlucky (bones
// aligned with view rays) and a retry is worth it
constexpr float k_boneSpreadWarnMm= 4.f;

const ImVec4 k_colorGood(0.4f, 1.f, 0.4f, 1.f);
const ImVec4 k_colorWait(1.f, 0.85f, 0.3f, 1.f);
const ImVec4 k_colorBad(1.f, 0.4f, 0.4f, 1.f);

void drawCenteredCountdown(float remainingSeconds, float totalSeconds)
{
	char countdownText[16];
	snprintf(countdownText, sizeof(countdownText), "%d", (int)ceilf(remainingSeconds));
	ImGui::SetWindowFontScale(3.f);
	const float textWidth= ImGui::CalcTextSize(countdownText).x;
	ImGui::SetCursorPosX((ImGui::GetWindowWidth() - textWidth) * 0.5f);
	ImGui::TextColored(k_colorWait, "%s", countdownText);
	ImGui::SetWindowFontScale(1.f);
	ImGui::ProgressBar(1.f - remainingSeconds / totalSeconds, ImVec2(-1, 4), "");
}

// Both sides tracked right now at the quality the stage needs: stereo with
// two or more cameras, any world pose with one
void trackingStatus(const TrackingFrameResult& fused, bool bStereoRequired, bool outTracked[2])
{
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= fused.poses[sideIndex];
		outTracked[sideIndex]= pose.tracked && pose.hasWorldPose &&
							   (!bStereoRequired || pose.stereoTriangulated);
	}
}

void drawTrackingStatusLine(const TrackingFrameResult& fused, bool bStereoRequired)
{
	bool bTracked[2];
	trackingStatus(fused, bStereoRequired, bTracked);
	ImGui::Text("%s", locText(bStereoRequired ? "handWizard.stereoTrackingLeftLabel"
											  : "handWizard.trackingLeftLabel"));
	ImGui::SameLine();
	ImGui::TextColored(bTracked[0] ? k_colorGood : k_colorBad, "%s",
					   bTracked[0] ? locText("common.yes") : locText("common.no"));
	ImGui::SameLine();
	ImGui::Text("%s", locText(bStereoRequired ? "handWizard.stereoTrackingRightLabel"
											  : "handWizard.trackingRightLabel"));
	ImGui::SameLine();
	ImGui::TextColored(bTracked[1] ? k_colorGood : k_colorBad, "%s",
					   bTracked[1] ? locText("common.yes") : locText("common.no"));
}

const char* sideName(int sideIndex)
{
	return sideIndex == 0 ? locText("handWizard.leftLabel") : locText("handWizard.rightLabel");
}
} // namespace

HandCalibrationWizard::HandCalibrationWizard(AppConfig* config, VisionThread* visionThread)
	: m_config(config)
	, m_visionThread(visionThread)
{
}

void HandCalibrationWizard::enter()
{
	m_bActive= true;
	m_bWantsClose= false;
	m_result= eWizardResult::None;
	m_state= eState::BonesIntro;
	m_countdown= 0.f;
	m_boneCapture= VisionThread::BoneCalibrationCapture();
	m_restCapture= VisionThread::RestPoseCapture();

	// One camera has no second view to triangulate against, so the desk
	// plane measures the bones instead
	m_bDeskMode= m_config->cameraCount() == 1;
	m_deskCalibrator.reset();
	m_deskStatus= DeskHandCalibrator::SampleStatus();
	m_deskResult= DeskHandCalibrator::Result();
	m_deskSamplingRemaining= 0.f;
}

void HandCalibrationWizard::exit()
{
	m_visionThread->cancelBoneCalibration();
	m_previews= nullptr;
	m_bActive= false;
}

bool HandCalibrationWizard::update(float deltaSeconds, const std::vector<VisionPreviewFrame>& previews,
								   const TrackingFrameResult& fusedResult)
{
	if (!m_bActive)
		return false;

	// The previews are only read within this call; the pointer saves
	// threading them through every draw function
	m_previews= &previews;

	ImGui::SetNextWindowSize(ImVec2(560, 0), ImGuiCond_Appearing);
	if (ImGui::Begin(locWindowTitle("windows.handWizard"), nullptr, ImGuiWindowFlags_NoCollapse))
	{
		switch (m_state)
		{
		case eState::BonesIntro: drawBonesIntro(fusedResult); break;
		case eState::BonesCountdown: drawBonesCountdown(deltaSeconds); break;
		case eState::BonesSampling: drawBonesSampling(deltaSeconds); break;
		case eState::BonesReview: drawBonesReview(); break;
		case eState::RestIntro: drawRestIntro(fusedResult); break;
		case eState::RestCountdown: drawRestCountdown(deltaSeconds); break;
		case eState::RestResult: drawRestResult(); break;
		}

		ImGui::Separator();
		if (ImGui::Button(locLabel("handWizard.cancelCalibrationButton")))
		{
			m_result= eWizardResult::Cancelled;
			m_bWantsClose= true;
		}
	}
	ImGui::End();

	m_previews= nullptr;
	return !m_bWantsClose;
}

bool HandCalibrationWizard::makeDeskCameraFrame(CameraFrameResult& outCamera) const
{
	constexpr int cameraIndex= 0;
	if (m_previews == nullptr || cameraIndex >= (int)m_previews->size() || !(*m_previews)[cameraIndex].valid)
		return false;

	const CameraProfile& profile= m_config->camera(cameraIndex);
	if (!profile.intrinsics.present || !profile.extrinsics.present)
		return false;

	outCamera= CameraFrameResult();
	outCamera.cameraIndex= cameraIndex;
	outCamera.valid= true;
	outCamera.hasExtrinsics= true;
	outCamera.markerFromCamera= profile.extrinsics.markerFromCamera;
	outCamera.hasIntrinsics= true;

	const MikanMatrix3d& cameraMatrix= profile.intrinsics.intrinsics.undistorted_camera_matrix;
	outCamera.fx= (float)cameraMatrix.x0;
	outCamera.fy= (float)cameraMatrix.y1;
	outCamera.cx= (float)cameraMatrix.z0;
	outCamera.cy= (float)cameraMatrix.z1;
	outCamera.result= (*m_previews)[cameraIndex].result;
	return true;
}

void HandCalibrationWizard::drawDeskStatusLines() const
{
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const bool bTracked= m_deskStatus.bTracked[sideIndex];
		const bool bFlat= m_deskStatus.bFlat[sideIndex];
		const char* state= !bTracked ? locText("handWizard.deskNotSeen")
					: bFlat ? locText("handWizard.deskFlat") : locText("handWizard.deskNotFlat");
		ImGui::TextColored(bFlat ? k_colorGood : bTracked ? k_colorWait : k_colorBad,
						   locText("handWizard.deskSideStatusFmt"), sideName(sideIndex), state,
						   m_deskStatus.tiltDegrees[sideIndex], m_deskStatus.curlDegrees[sideIndex]);
	}
}

void HandCalibrationWizard::drawBonesIntro(const TrackingFrameResult& fusedResult)
{
	if (m_bDeskMode)
	{
		ImGui::TextWrapped(locText("handWizard.deskIntroFmt"), (int)k_boneSampleSeconds);
		ImGui::Spacing();
		CameraFrameResult camera;
		if (makeDeskCameraFrame(camera))
			DeskHandCalibrator::assess(camera, m_deskStatus);
		else
			m_deskStatus= DeskHandCalibrator::SampleStatus();
		drawDeskStatusLines();
	}
	else
	{
		ImGui::TextWrapped(locText("handWizard.bonesIntroFmt"), (int)k_boneSampleSeconds);
		ImGui::Spacing();
		drawTrackingStatusLine(fusedResult, true);
	}
	ImGui::Spacing();

	if (ImGui::Button(locLabel("handWizard.startBoneMeasurementButton"), ImVec2(-1, 0)))
	{
		m_countdown= k_countdownSeconds;
		m_state= eState::BonesCountdown;
	}
}

void HandCalibrationWizard::drawBonesCountdown(float deltaSeconds)
{
	m_countdown-= deltaSeconds;
	if (m_countdown <= 0.f)
	{
		if (m_bDeskMode)
		{
			m_deskCalibrator.reset();
			m_deskSamplingRemaining= k_boneSampleSeconds;
		}
		else
		{
			m_visionThread->requestBoneCalibration(k_boneSampleSeconds);
		}
		m_state= eState::BonesSampling;
		return;
	}
	ImGui::TextWrapped("%s", locText(m_bDeskMode ? "handWizard.deskCountdownHint"
												 : "handWizard.bonesCountdownHint"));
	drawCenteredCountdown(m_countdown, k_countdownSeconds);
}

void HandCalibrationWizard::drawBonesSampling(float deltaSeconds)
{
	if (m_bDeskMode)
	{
		// Sampled here, from the camera's previews: the desk plane needs
		// only the 2D landmarks and the camera's calibration, both of which
		// the main thread already has
		CameraFrameResult camera;
		if (makeDeskCameraFrame(camera))
			m_deskCalibrator.addSample(camera, m_deskStatus);
		m_deskSamplingRemaining-= deltaSeconds;

		ImGui::TextColored(k_colorWait, "%s", locText("handWizard.deskSamplingHint"));
		drawDeskStatusLines();
		ImGui::Text(locText("handWizard.samplesFmt"), m_deskCalibrator.getSampleCount(eHandSide::Left),
					m_deskCalibrator.getSampleCount(eHandSide::Right), HandBoneCalibrator::k_minSamples);
		ImGui::ProgressBar(1.f - m_deskSamplingRemaining / k_boneSampleSeconds, ImVec2(-1, 4), "");

		if (m_deskSamplingRemaining <= 0.f)
		{
			m_deskCalibrator.finish(m_config->camera(0).intrinsics.intrinsics, m_deskResult);
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				m_boneCapture.bCaptured[sideIndex]= m_deskResult.bCaptured[sideIndex];
				m_boneCapture.skeleton[sideIndex]= m_deskResult.skeleton[sideIndex];
				m_boneCapture.quality[sideIndex]= m_deskResult.quality[sideIndex];
			}
			m_state= eState::BonesReview;
			return;
		}

		if (ImGui::Button(locLabel("handWizard.cancelSamplingButton"), ImVec2(-1, 0)))
		{
			m_deskCalibrator.reset();
			m_state= eState::BonesIntro;
		}
		return;
	}

	bool bSamplingActive= false;
	int liveSamples[2]= {0, 0};
	m_visionThread->getBoneCalibrationProgress(bSamplingActive, liveSamples[0], liveSamples[1]);

	ImGui::TextColored(k_colorWait, "%s", locText("handWizard.samplingHint"));
	ImGui::Text(locText("handWizard.samplesFmt"), liveSamples[0], liveSamples[1],
				HandBoneCalibrator::k_minSamples);

	if (m_visionThread->fetchBoneCalibration(m_boneCapture))
	{
		m_state= eState::BonesReview;
		return;
	}

	if (ImGui::Button(locLabel("handWizard.cancelSamplingButton"), ImVec2(-1, 0)))
	{
		m_visionThread->cancelBoneCalibration();
		m_state= eState::BonesIntro;
	}
}

void HandCalibrationWizard::drawBonesReview()
{
	// Review before anything is written: this replaces the geometry every
	// client rebuilds the hand from, so it gets looked at first
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (!m_boneCapture.bCaptured[sideIndex])
		{
			ImGui::TextColored(k_colorBad, locText("handWizard.boneSampleShortfallFmt"),
							   sideName(sideIndex), m_boneCapture.quality[sideIndex].sampleCount,
							   HandBoneCalibrator::k_minSamples);
			continue;
		}

		const HandSkeleton& skeleton= m_boneCapture.skeleton[sideIndex];
		const float referenceBone= skeleton.baseInPalm[(int)eFinger::Middle].x * 2.f;
		const float spreadMm= m_boneCapture.quality[sideIndex].worstPhalanxSpread * 1000.f;
		const ImVec4 spreadColor= spreadMm > k_boneSpreadWarnMm ? k_colorWait : k_colorGood;
		ImGui::TextColored(spreadColor, locText("handWizard.boneMeasurementSummaryFmt"),
						   sideName(sideIndex), referenceBone * 1000.f,
						   m_boneCapture.quality[sideIndex].sampleCount, spreadMm);

		// Proximal phalanxes only: they are the longest bones and where the
		// model is most wrong, so they are what a glance should check
		ImGui::Text("   proximal  index %.1f  middle %.1f  ring %.1f  pinky %.1f mm",
					skeleton.phalanxLengths[(int)eFinger::Index][0] * 1000.f,
					skeleton.phalanxLengths[(int)eFinger::Middle][0] * 1000.f,
					skeleton.phalanxLengths[(int)eFinger::Ring][0] * 1000.f,
					skeleton.phalanxLengths[(int)eFinger::Pinky][0] * 1000.f);

		// The desk frames also measured how deep this camera's lift reads
		// the hand; the scale saves beside the bones
		if (m_bDeskMode)
		{
			if (m_deskResult.bLiftScaleMeasured[sideIndex])
			{
				ImGui::Text(locText("handWizard.deskLiftScaleFmt"), sideName(sideIndex),
							m_deskResult.liftDepthScale[sideIndex], m_deskResult.liftDepthScaleSpread[sideIndex],
							m_deskResult.liftSampleCount[sideIndex]);
			}
			else
			{
				ImGui::TextColored(k_colorWait, locText("handWizard.deskLiftScaleMissingFmt"),
								   sideName(sideIndex), m_deskResult.liftSampleCount[sideIndex]);
			}
		}
	}

	const bool bAnyCaptured= m_boneCapture.bCaptured[0] || m_boneCapture.bCaptured[1];
	ImGui::Spacing();
	ImGui::BeginDisabled(!bAnyCaptured);
	if (ImGui::Button(locLabel("handWizard.acceptAndContinueButton"), ImVec2(-1, 0)))
	{
		saveBones();
		m_state= eState::RestIntro;
	}
	ImGui::EndDisabled();
	if (ImGui::Button(locLabel("handWizard.retryMeasurementButton"), ImVec2(-1, 0)))
		m_state= eState::BonesIntro;
}

void HandCalibrationWizard::drawRestIntro(const TrackingFrameResult& fusedResult)
{
	ImGui::TextWrapped("%s", locText(m_bDeskMode ? "handWizard.deskRestIntro" : "handWizard.restIntro"));
	ImGui::Spacing();
	drawTrackingStatusLine(fusedResult, !m_bDeskMode);
	ImGui::Spacing();

	if (ImGui::Button(locLabel("handWizard.captureRestPoseButton"), ImVec2(-1, 0)))
	{
		m_countdown= k_countdownSeconds;
		m_state= eState::RestCountdown;
	}
}

void HandCalibrationWizard::drawRestCountdown(float deltaSeconds)
{
	m_countdown-= deltaSeconds;
	if (m_countdown <= 0.f)
	{
		m_visionThread->requestRestPoseCapture();
		m_state= eState::RestResult;
		return;
	}
	ImGui::TextWrapped("%s", locText(m_bDeskMode ? "handWizard.deskRestCountdownHint"
												 : "handWizard.restCountdownHint"));
	drawCenteredCountdown(m_countdown, k_countdownSeconds);
}

void HandCalibrationWizard::drawRestResult()
{
	// The capture is serviced by the vision thread within a fuse; poll for it
	VisionThread::RestPoseCapture capture;
	if (m_visionThread->fetchRestPoseCapture(capture))
	{
		m_restCapture= capture;
		saveRest();
	}

	const bool bLeft= m_restCapture.bCaptured[0];
	const bool bRight= m_restCapture.bCaptured[1];
	if (bLeft && bRight)
		ImGui::TextColored(k_colorGood, "%s", locText("handWizard.bothHandsCaptured"));
	else if (bLeft || bRight)
		ImGui::TextColored(k_colorWait, "%s",
						   bLeft ? locText("handWizard.leftOnlyCaptured") : locText("handWizard.rightOnlyCaptured"));
	else
		ImGui::TextColored(k_colorBad, "%s", locText("handWizard.nothingCaptured"));

	ImGui::Spacing();
	if (ImGui::Button(locLabel("handWizard.retryRestPoseButton"), ImVec2(-1, 0)))
		m_state= eState::RestIntro;
	ImGui::BeginDisabled(!bLeft && !bRight);
	if (ImGui::Button(locLabel("common.done"), ImVec2(-1, 0)))
	{
		m_result= eWizardResult::Completed;
		m_bWantsClose= true;
	}
	ImGui::EndDisabled();
}

void HandCalibrationWizard::saveBones()
{
	double referenceSum= 0.0;
	int referenceCount= 0;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (!m_boneCapture.bCaptured[sideIndex])
			continue;
		m_config->handSkeleton.skeleton[sideIndex]= m_boneCapture.skeleton[sideIndex];
		m_config->handSkeleton.present[sideIndex]= true;
		referenceSum+= m_boneCapture.skeleton[sideIndex].baseInPalm[(int)eFinger::Middle].x * 2.0;
		++referenceCount;

		if (m_bDeskMode && m_deskResult.bLiftScaleMeasured[sideIndex])
			m_config->camera(0).liftDepthScale[sideIndex]= m_deskResult.liftDepthScale[sideIndex];
	}

	// One scale story: the reference bone the rest of the app reads now
	// comes from the measurement, not the EMA
	if (referenceCount > 0)
	{
		m_config->handScale.refLengthMeters= referenceSum / (double)referenceCount;
		m_config->handScale.present= true;
	}

	m_config->save();
	m_visionThread->requestConfigRefresh();
}

void HandCalibrationWizard::saveRest()
{
	bool bAnySide= false;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (!m_restCapture.bCaptured[sideIndex])
			continue;
		m_config->fusedRestAngles.angles[sideIndex]= m_restCapture.angles[sideIndex];
		m_config->fusedRestAngles.present[sideIndex]= true;
		bAnySide= true;
	}
	if (!bAnySide)
		return;

	m_config->save();
	m_visionThread->requestConfigRefresh();
}
