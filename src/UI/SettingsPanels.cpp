#include "SettingsPanels.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>

#include "imgui.h"
#include "implot.h"

#include "glm/gtc/quaternion.hpp"
#include "glm/trigonometric.hpp"

#include "App.h"
#include "AppConfig.h"
#include "BodyPoseTracker.h" // eBodyPoseBackend
#include "FaceService.h"
#include "HandFusion.h"
#include "HandPoseModel.h"
#include "HandRoiQuality.h"
#include "LocalizationManager.h"
#include "Logger.h"
#include "LocText.h"
#include "RetargetStage.h"
#include "Scene3dPanel.h"
#include "VideoCaptureSystem.h"
#include "VideoPreviewPanel.h"
#include "VisionThread.h"
#include "WebcamBlink.h"
#include "WebcamFace.h"

// Hold-still jitter test: countdown mirrors the calibration captures (the mouse
// hand needs to get back into position), then the sampling window
static constexpr float k_holdStillCountdownSeconds= 3.f;
static constexpr float k_holdStillSampleSeconds= 3.f;
// A side needs at least this many fused samples for its deviation to mean
// anything (one second at 30fps)
static constexpr int k_holdStillMinSamples= 30;

static void commitConfigChange(AppConfig* config, VisionThread* visionThread)
{
	config->markDirty();
	visionThread->requestConfigRefresh();
}

void SettingsPanels::commitTrackingConfigChange(const TrackingSectionContext& context)
{
	commitConfigChange(context.config, context.visionThread);
}

void SettingsPanels::commitOscConfigChange(const OscSectionContext& context)
{
	commitConfigChange(context.config, context.visionThread);
}

// The cameras the image-quality and capture-timing readouts have slots for
static int getReadoutCameraCount(const AppConfig* config)
{
	return std::min((int)config->cameraCount(), (int)TrackingPanelState::kQualityMaxCameras);
}

// -- Image quality readout ---------------------------------------------------

// Band classification: good inside [goodLo, goodHi], warn inside
// [warnLo, warnHi] (which contains the good range), bad outside both
struct MetricBand
{
	float goodLo, goodHi;
	float warnLo, warnHi;
};

static int classifyBand(float value, const MetricBand& band)
{
	if (value >= band.goodLo && value <= band.goodHi)
		return 0;
	if (value >= band.warnLo && value <= band.warnHi)
		return 1;
	return 2;
}

static ImVec4 bandColor(int bandClass)
{
	switch (bandClass)
	{
	case 0: return ImVec4(0.4f, 1.f, 0.5f, 1.f);
	case 1: return ImVec4(1.f, 0.85f, 0.3f, 1.f);
	default: return ImVec4(1.f, 0.4f, 0.4f, 1.f);
	}
}

// How far outside the good range a value sits (0 inside), used to pick the
// worst hand when both classify the same
static float bandSeverity(float value, const MetricBand& band)
{
	return std::max(0.f, std::max(band.goodLo - value, value - band.goodHi));
}

struct QualityRowDesc
{
	const char* labelKey;
	const char* format; // printf format for the value in display units
	float displayScale; // raw metric -> display units (100 for ratio -> %)
	MetricBand band;    // in display units
	float (*extract)(const HandImageQuality&);
	const char* tooltipKey;
};

// Bands live at the analyzer's fixed ~160px working scale, tuned against real
// captures: a well-lit low-jitter camera reads noise ~2.5 and sharpness in the
// low thousands, so sharpness only means something when it drops (blur), and
// high values are just texture/scale
static const QualityRowDesc k_qualityRows[]= {
	{"trackingPanel.luminanceLabel", "%.0f", 1.f, {90.f, 180.f, 60.f, 220.f},
	 [](const HandImageQuality& q) { return q.meanLuma; },
	 "trackingPanel.luminanceTooltip"},
	{"trackingPanel.highlightClipLabel", "%.1f%%", 100.f, {0.f, 1.f, 0.f, 5.f},
	 [](const HandImageQuality& q) { return q.highlightClipRatio; },
	 "trackingPanel.highlightClipTooltip"},
	{"trackingPanel.shadowClipLabel", "%.1f%%", 100.f, {0.f, 5.f, 0.f, 15.f},
	 [](const HandImageQuality& q) { return q.shadowClipRatio; },
	 "trackingPanel.shadowClipTooltip"},
	{"trackingPanel.contrastLabel", "%.0f", 1.f, {30.f, 1e9f, 15.f, 1e9f},
	 [](const HandImageQuality& q) { return q.contrast; },
	 "trackingPanel.contrastTooltip"},
	{"trackingPanel.separationLabel", "%.0f", 1.f, {20.f, 1e9f, 8.f, 1e9f},
	 [](const HandImageQuality& q) { return q.backgroundSeparation; },
	 "trackingPanel.separationTooltip"},
	{"trackingPanel.sharpnessLabel", "%.0f", 1.f, {1000.f, 1e9f, 300.f, 1e9f},
	 [](const HandImageQuality& q) { return q.sharpness; },
	 "trackingPanel.sharpnessTooltip"},
	{"trackingPanel.noiseLabel", "%.1f", 1.f, {0.f, 4.f, 0.f, 7.f},
	 [](const HandImageQuality& q) { return q.noise; },
	 "trackingPanel.noiseTooltip"},
};
// The flicker row is appended after these (per camera, not per hand)
static constexpr int k_flickerRowIndex= (int)IM_ARRAYSIZE(k_qualityRows);

// The band and display format of a row, the flicker row included
static void getQualityRowBand(int rowIndex, MetricBand& outBand, const char*& outFormat)
{
	if (rowIndex == k_flickerRowIndex)
	{
		outBand= {0.f, 2.f, 0.f, 5.f};
		outFormat= "%.1f%%";
	}
	else
	{
		outBand= k_qualityRows[rowIndex].band;
		outFormat= k_qualityRows[rowIndex].format;
	}
}

// ~1s EMA so the readout is legible at camera rate. A cell with no value this
// frame is left invalid, which the section draws as a dash.
static void updateImageQualityEma(const TrackingSectionContext& context, float deltaSeconds)
{
	TrackingPanelState& panelState= *context.state;
	const std::vector<VisionPreviewFrame>& latestPreviews= *context.latestPreviews;
	const int cameraCount= getReadoutCameraCount(context.config);
	const float emaAlpha= std::min(1.f, deltaSeconds);

	for (int rowIndex= 0; rowIndex <= k_flickerRowIndex; ++rowIndex)
	{
		const bool bFlickerRow= rowIndex == k_flickerRowIndex;
		MetricBand band;
		const char* format;
		getQualityRowBand(rowIndex, band, format);

		for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
		{
			const bool bHavePreview= cameraIndex < (int)latestPreviews.size() &&
									 latestPreviews[cameraIndex].valid;
			float& ema= panelState.qualityEma[cameraIndex][rowIndex];
			bool& bEmaValid= panelState.bQualityEmaValid[cameraIndex][rowIndex];

			float rawValue= 0.f;
			bool bHaveValue= false;
			if (bFlickerRow)
			{
				if (bHavePreview)
				{
					rawValue= latestPreviews[cameraIndex].result.lumaInstability * 100.f;
					bHaveValue= true;
				}
			}
			else
			{
				const QualityRowDesc& row= k_qualityRows[rowIndex];
				if (bHavePreview)
				{
					// Worst tracked hand: higher band class first, then the
					// distance outside the good range
					int worstClass= -1;
					float worstSeverity= 0.f;
					for (const TrackedHand& hand : latestPreviews[cameraIndex].result.hands)
					{
						if (!hand.tracked || !hand.imageQuality.valid)
							continue;
						const float value= row.extract(hand.imageQuality) * row.displayScale;
						const int bandClass= classifyBand(value, band);
						const float severity= bandSeverity(value, band);
						if (bandClass > worstClass ||
							(bandClass == worstClass && severity > worstSeverity))
						{
							worstClass= bandClass;
							worstSeverity= severity;
							rawValue= value;
							bHaveValue= true;
						}
					}
				}
			}

			if (!bHaveValue)
			{
				bEmaValid= false;
				continue;
			}

			ema= bEmaValid ? ema + (rawValue - ema) * emaAlpha : rawValue;
			bEmaValid= true;
		}
	}
}

void SettingsPanels::drawImageQualitySection(const TrackingSectionContext& context)
{
	ImGui::SeparatorText(locText("trackingPanel.imageQualitySection"));

	const TrackingPanelState& panelState= *context.state;
	const std::vector<VisionPreviewFrame>& latestPreviews= *context.latestPreviews;
	const int cameraCount= getReadoutCameraCount(context.config);
	if (cameraCount <= 0)
		return;

	ImGui::TextDisabled("%s", locText("trackingPanel.imageQualityHeader"));
	ImGui::SetItemTooltip("%s", locText("trackingPanel.imageQualityTooltip"));

	if (!ImGui::BeginTable("imageQuality", 1 + cameraCount,
						   ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
		return;

	ImGui::TableSetupColumn(locText("trackingPanel.metricColumn"));
	for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
	{
		const std::string header= locFormat("trackingPanel.cameraFmt", cameraIndex + 1);
		ImGui::TableSetupColumn(header.c_str());
	}
	ImGui::TableHeadersRow();

	for (int rowIndex= 0; rowIndex <= k_flickerRowIndex; ++rowIndex)
	{
		const bool bFlickerRow= rowIndex == k_flickerRowIndex;
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		ImGui::Text("%s", bFlickerRow ? locText("trackingPanel.flickerLabel") : locText(k_qualityRows[rowIndex].labelKey));
		ImGui::SetItemTooltip("%s", bFlickerRow
			? locText("trackingPanel.flickerTooltip")
			: locText(k_qualityRows[rowIndex].tooltipKey));

		MetricBand band;
		const char* format;
		getQualityRowBand(rowIndex, band, format);

		for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
		{
			ImGui::TableNextColumn();

			if (!panelState.bQualityEmaValid[cameraIndex][rowIndex])
			{
				ImGui::TextDisabled("-");
				continue;
			}
			const float ema= panelState.qualityEma[cameraIndex][rowIndex];

			char text[48];
			snprintf(text, sizeof(text), format, ema);
			if (bFlickerRow)
			{
				const bool bHavePreview= cameraIndex < (int)latestPreviews.size() &&
										 latestPreviews[cameraIndex].valid;
				const float flickerHz= bHavePreview ? latestPreviews[cameraIndex].result.lumaFlickerHz : 0.f;
				if (flickerHz > 0.f)
				{
					const size_t len= strlen(text);
					snprintf(text + len, sizeof(text) - len, " @%.1fHz", flickerHz);
				}
			}
			ImGui::TextColored(bandColor(classifyBand(ema, band)), "%s", text);
		}
	}
	ImGui::EndTable();
}

// -- Capture timing readout --------------------------------------------------

// One row of the capture-timing table: a per-step value out of the newest
// per-camera result, or (extract null) a capture-system counter
struct TimingRowDesc
{
	const char* labelKey;
	const char* tooltipKey;
	const char* format;
	float (*extract)(const TrackingFrameResult&);
};

static const TimingRowDesc k_timingRows[]= {
	{"trackingPanel.timingQueueAgeLabel", "trackingPanel.timingQueueAgeTooltip", "%.1f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.queueAgeMs; }},
	{"trackingPanel.timingConvertLabel", "trackingPanel.timingConvertTooltip", "%.2f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.convertMs; }},
	{"trackingPanel.timingUndistortLabel", "trackingPanel.timingUndistortTooltip", "%.2f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.undistortMs; }},
	{"trackingPanel.timingFlickerLabel", "trackingPanel.timingFlickerTooltip", "%.2f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.flickerMs; }},
	{"trackingPanel.timingHandInferenceLabel", "trackingPanel.timingHandInferenceTooltip", "%.1f ms",
	 [](const TrackingFrameResult& r) { return r.inferenceMs; }},
	{"trackingPanel.timingBodyPoseLabel", "trackingPanel.timingBodyPoseTooltip", "%.1f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.bodyPoseMs; }},
	{"trackingPanel.timingRoiQualityLabel", "trackingPanel.timingRoiQualityTooltip", "%.2f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.roiQualityMs; }},
	{"trackingPanel.timingLiftLabel", "trackingPanel.timingLiftTooltip", "%.2f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.liftMs; }},
	{"trackingPanel.timingPublishLabel", "trackingPanel.timingPublishTooltip", "%.2f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.publishMs; }},
	{"trackingPanel.timingTotalLabel", "trackingPanel.timingTotalTooltip", "%.1f ms",
	 [](const TrackingFrameResult& r) { return r.captureTimings.totalMs; }},
	{"trackingPanel.timingCallbackCopyLabel", "trackingPanel.timingCallbackCopyTooltip", "%.2f ms", nullptr},
	{"trackingPanel.timingQueuedFramesLabel", "trackingPanel.timingQueuedFramesTooltip", "%.0f", nullptr},
	{"trackingPanel.timingDroppedFramesLabel", "trackingPanel.timingDroppedFramesTooltip", "%.0f", nullptr},
};
static_assert(IM_ARRAYSIZE(k_timingRows) == TrackingPanelState::kTimingRowCount,
			  "timing EMA slots out of step with the rows");
// The capture-system rows, after the per-frame ones
static constexpr int k_timingCallbackCopyRow= 10;
static constexpr int k_timingQueuedFramesRow= 11;

// ~1s EMA, as for the image-quality readout: the per-frame values are twitchy
// at camera rate. The counters are kept as they are, and a cell with no value
// this frame is left invalid.
static void updateCaptureTimingEma(const TrackingSectionContext& context, float deltaSeconds)
{
	TrackingPanelState& panelState= *context.state;
	const std::vector<VisionPreviewFrame>& latestPreviews= *context.latestPreviews;
	VideoCaptureSystem* videoCapture= context.videoCapture;
	const int cameraCount= getReadoutCameraCount(context.config);
	const float emaAlpha= std::min(1.f, deltaSeconds);

	for (int rowIndex= 0; rowIndex < (int)IM_ARRAYSIZE(k_timingRows); ++rowIndex)
	{
		const TimingRowDesc& row= k_timingRows[rowIndex];
		for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
		{
			float rawValue= 0.f;
			bool bHaveValue= false;
			bool bSmoothed= true;
			if (row.extract != nullptr)
			{
				if (cameraIndex < (int)latestPreviews.size() && latestPreviews[cameraIndex].valid)
				{
					rawValue= row.extract(latestPreviews[cameraIndex].result);
					bHaveValue= true;
				}
			}
			else if (rowIndex == k_timingCallbackCopyRow)
			{
				rawValue= videoCapture->getCallbackCopyMs(cameraIndex);
				bHaveValue= rawValue > 0.f;
			}
			else
			{
				// Counters are shown as they are: a queue depth is already an
				// instantaneous fact and a drop count only ever grows
				rawValue= rowIndex == k_timingQueuedFramesRow
					? (float)videoCapture->getQueuedFrameCount(cameraIndex)
					: (float)videoCapture->getDroppedFrameCount(cameraIndex);
				bHaveValue= true;
				bSmoothed= false;
			}

			float& ema= panelState.timingEma[cameraIndex][rowIndex];
			bool& bEmaValid= panelState.bTimingEmaValid[cameraIndex][rowIndex];
			if (!bHaveValue)
			{
				bEmaValid= false;
				continue;
			}
			if (bSmoothed)
				ema= bEmaValid ? ema + (rawValue - ema) * emaAlpha : rawValue;
			else
				ema= rawValue;
			bEmaValid= true;
		}
	}
}

void SettingsPanels::drawCaptureTimingSection(const TrackingSectionContext& context)
{
	const TrackingPanelState& panelState= *context.state;
	const int cameraCount= getReadoutCameraCount(context.config);
	if (cameraCount <= 0)
		return;

	ImGui::SeparatorText(locText("trackingPanel.captureTimingSection"));
	ImGui::SetItemTooltip("%s", locText("trackingPanel.captureTimingTooltip"));

	if (!ImGui::BeginTable("captureTiming", 1 + cameraCount,
						   ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
		return;

	ImGui::TableSetupColumn(locText("trackingPanel.stepColumn"));
	for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
	{
		const std::string header= locFormat("trackingPanel.cameraFmt", cameraIndex + 1);
		ImGui::TableSetupColumn(header.c_str());
	}
	ImGui::TableHeadersRow();

	for (int rowIndex= 0; rowIndex < (int)IM_ARRAYSIZE(k_timingRows); ++rowIndex)
	{
		const TimingRowDesc& row= k_timingRows[rowIndex];
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		ImGui::Text("%s", locText(row.labelKey));
		ImGui::SetItemTooltip("%s", locText(row.tooltipKey));

		for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
		{
			ImGui::TableNextColumn();
			if (!panelState.bTimingEmaValid[cameraIndex][rowIndex])
			{
				ImGui::TextDisabled("-");
				continue;
			}
			ImGui::Text(row.format, panelState.timingEma[cameraIndex][rowIndex]);
		}
	}
	ImGui::EndTable();
}

// -- Hold-still jitter test --------------------------------------------------

// THE repeatable A/B number for camera-settings and lighting changes. Rebuilds
// the 21 world-space joints from each fused pose via forward kinematics
// (exactly what a client rebuilds) and reports the mean per-landmark deviation
// over a still window - with the hand genuinely still, deviation IS tracking
// noise. Runs the countdown and the sampling window the section starts.
static void updateHoldStill(const TrackingSectionContext& context, float deltaSeconds)
{
	TrackingPanelState& panelState= *context.state;
	const TrackingFrameResult& fusedResult= *context.fusedResult;

	if (panelState.holdStillCountdown > 0.f)
	{
		panelState.holdStillCountdown-= deltaSeconds;
		if (panelState.holdStillCountdown <= 0.f)
		{
			panelState.holdStillCountdown= 0.f;
			panelState.holdStillSecondsLeft= k_holdStillSampleSeconds;
			panelState.holdStillLastTimestampMs= 0.0;
			panelState.holdStillAccum[0]= TrackingPanelState::HoldStillAccum();
			panelState.holdStillAccum[1]= TrackingPanelState::HoldStillAccum();
			// A direct project has no world frame, so its window measures the
			// camera-space lift. The figure is a sum of per-axis variances,
			// which a rigid change of frame does not alter.
			panelState.bHoldStillCameraFrame= isDirectProject(*context.config);
		}
	}
	else if (panelState.holdStillSecondsLeft > 0.f)
	{
		// Accumulate each NEW fused result (they arrive slower than UI
		// frames; re-adding a held result would fake stability)
		if (fusedResult.timestampMs > 0.0 &&
			fusedResult.timestampMs != panelState.holdStillLastTimestampMs)
		{
			panelState.holdStillLastTimestampMs= fusedResult.timestampMs;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const HandPose& pose= fusedResult.poses[sideIndex];
				// One frame per window: a pose without it adds nothing
				const bool bCameraFrame= panelState.bHoldStillCameraFrame;
				if (!pose.tracked || !(bCameraFrame ? pose.hasCameraPose : pose.hasWorldPose))
					continue;
				const glm::vec3& palmPosition= bCameraFrame ? pose.palmPositionCamera : pose.palmPositionWorld;
				const glm::quat& palmOrientation=
					bCameraFrame ? pose.palmOrientationCamera : pose.palmOrientationWorld;

				// A pose repeated under a new timestamp is not a new sample: a
				// live pose never repeats exactly
				TrackingPanelState::HoldStillAccum& accum= panelState.holdStillAccum[sideIndex];
				if (accum.bHasLast && palmPosition == accum.lastPalmPosition &&
					palmOrientation == accum.lastPalmOrientation)
					continue;
				accum.bHasLast= true;
				accum.lastPalmPosition= palmPosition;
				accum.lastPalmOrientation= palmOrientation;

				glm::mat4 palmTransform= glm::mat4_cast(palmOrientation);
				palmTransform[3]= glm::vec4(palmPosition, 1.f);
				std::array<std::array<glm::vec3, 4>, FINGER_COUNT> joints;
				HandPoseModel::buildFingerJoints(palmTransform, pose.skeleton, pose.fingers, joints);

				int pointIndex= 0;
				auto addPoint= [&accum, &pointIndex](const glm::vec3& point) {
					const glm::dvec3 p(point);
					accum.sum[pointIndex]+= p;
					accum.sumSq[pointIndex]+= p * p;
					++pointIndex;
				};
				addPoint(palmPosition);
				for (int fingerIndex= 0; fingerIndex < FINGER_COUNT; ++fingerIndex)
					for (int jointIndex= 0; jointIndex < 4; ++jointIndex)
						addPoint(joints[fingerIndex][jointIndex]);
				++accum.sampleCount;
			}
		}

		panelState.holdStillSecondsLeft-= deltaSeconds;
		if (panelState.holdStillSecondsLeft <= 0.f)
		{
			panelState.holdStillSecondsLeft= 0.f;
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				const TrackingPanelState::HoldStillAccum& accum= panelState.holdStillAccum[sideIndex];
				panelState.holdStillSampleCount[sideIndex]= accum.sampleCount;
				if (accum.sampleCount < k_holdStillMinSamples)
				{
					panelState.holdStillDeviationMm[sideIndex]= -1.f;
					continue;
				}

				double deviationSum= 0.0;
				for (int pointIndex= 0; pointIndex < HAND_LANDMARK_COUNT; ++pointIndex)
				{
					const glm::dvec3 mean= accum.sum[pointIndex] / (double)accum.sampleCount;
					const glm::dvec3 variance=
						accum.sumSq[pointIndex] / (double)accum.sampleCount - mean * mean;
					deviationSum+= std::sqrt(std::max(0.0, variance.x + variance.y + variance.z));
				}
				panelState.holdStillDeviationMm[sideIndex]=
					(float)(deviationSum / HAND_LANDMARK_COUNT * 1000.0);
			}
			panelState.bHoldStillHasResult= true;
			MIKAN_LOG_INFO("HoldStill") << "Jitter test ("
										<< (panelState.bHoldStillCameraFrame ? "camera space" : "world space")
										<< "): left " << panelState.holdStillDeviationMm[0] << " mm over "
										<< panelState.holdStillSampleCount[0] << " samples, right "
										<< panelState.holdStillDeviationMm[1] << " mm over "
										<< panelState.holdStillSampleCount[1] << " samples (-1 mm: under "
										<< k_holdStillMinSamples << " samples)";
		}
	}
}

void SettingsPanels::drawHoldStillSection(const TrackingSectionContext& context)
{
	TrackingPanelState& panelState= *context.state;

	ImGui::SeparatorText(locText("trackingPanel.holdStillSection"));

	if (panelState.holdStillCountdown > 0.f)
	{
		ImGui::TextColored(ImVec4(1.f, 0.85f, 0.3f, 1.f), locText("trackingPanel.holdStillCountdownFmt"),
						   (int)ceilf(panelState.holdStillCountdown));
		ImGui::ProgressBar(1.f - panelState.holdStillCountdown / k_holdStillCountdownSeconds,
						   ImVec2(-1, 4), "");
	}
	else if (panelState.holdStillSecondsLeft > 0.f)
	{
		ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.f, 1.f), "%s", locText("trackingPanel.holdStillSampling"));
		ImGui::ProgressBar(1.f - panelState.holdStillSecondsLeft / k_holdStillSampleSeconds,
						   ImVec2(-1, 4), "");
	}
	else if (ImGui::Button(locLabel("trackingPanel.holdStillJitterTest")))
	{
		panelState.holdStillCountdown= k_holdStillCountdownSeconds;
	}
	if (panelState.holdStillCountdown <= 0.f && panelState.holdStillSecondsLeft <= 0.f)
	{
		ImGui::SetItemTooltip("%s", locText("trackingPanel.holdStillJitterTestTooltip"));
	}

	if (panelState.bHoldStillHasResult)
	{
		ImGui::Text("%s", locText("trackingPanel.jitterLabel"));
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			ImGui::SameLine();
			const float deviationMm= panelState.holdStillDeviationMm[sideIndex];
			const char* sideLabel=
				sideIndex == 0 ? locText("trackingPanel.leftAbbr") : locText("trackingPanel.rightAbbr");
			const int sampleCount= panelState.holdStillSampleCount[sideIndex];
			if (deviationMm < 0.f)
			{
				ImGui::TextDisabled(locText("trackingPanel.jitterTooFewSamplesFmt"), sideLabel, sampleCount);
				continue;
			}
			const ImVec4 color= deviationMm < 2.f
				? ImVec4(0.4f, 1.f, 0.5f, 1.f)
				: (deviationMm < 5.f ? ImVec4(1.f, 0.85f, 0.3f, 1.f) : ImVec4(1.f, 0.4f, 0.4f, 1.f));
			ImGui::TextColored(color, locText("trackingPanel.jitterResultFmt"), sideLabel, deviationMm, sampleCount);
		}
	}
}

// -- Tracking panel ----------------------------------------------------------

// The seconds of shown frames the Blink readout plots
static constexpr double k_blinkHistoryWindowMs= 6000.0;
static constexpr size_t k_blinkHistoryMaxSamples= 720;
// A step forward this long is a seek, which starts the history over as a
// step backwards does
static constexpr double k_blinkHistorySeekMs= 1000.0;

using BlinkHistory= std::deque<TrackingPanelState::BlinkSample>;

static void updateBlinkHistory(const TrackingSectionContext& context)
{
	BlinkHistory& history= context.state->blinkHistory;
	const double timestampMs= context.face->blinkTimestampMs;
	if (!history.empty())
	{
		const double stepMs= timestampMs - history.back().timestampMs;
		if (stepMs == 0.0)
			return;
		if (stepMs < 0.0 || stepMs > k_blinkHistorySeekMs)
			history.clear();
	}
	history.push_back({timestampMs, context.face->blink});
	while (history.front().timestampMs < timestampMs - k_blinkHistoryWindowMs ||
		   history.size() > k_blinkHistoryMaxSamples)
		history.pop_front();
}

void SettingsPanels::updateTrackingState(const TrackingSectionContext& context, float deltaSeconds)
{
	AppConfig* config= context.config;
	VisionThread* visionThread= context.visionThread;
	bool bChanged= false;

	updateImageQualityEma(context, deltaSeconds);

	// The phone head's straight ahead, captured by the vision thread at the
	// head alignment's request
	glm::quat capturedAnchor;
	if (visionThread->fetchFaceAnchorCapture(capturedAnchor))
	{
		config->face.headAnchor= capturedAnchor;
		config->face.headAnchorPresent= true;
		bChanged= true;
	}

	// A countdown left running while the mode is switched off would fire a
	// capture nobody is waiting for
	NeutralCaptureControl& neutral= context.state->directNeutral;
	if (!config->direct.enabled)
		neutral.reset();
	bChanged|= neutral.update(deltaSeconds, visionThread, config);

	updateCaptureTimingEma(context, deltaSeconds);
	updateHoldStill(context, deltaSeconds);
	updateBlinkHistory(context);

	if (bChanged)
		commitTrackingConfigChange(context);
}

void SettingsPanels::drawFlipHandednessSection(const TrackingSectionContext& context)
{
	TrackingConfig& tracking= context.config->tracking;
	if (ImGui::Checkbox(locLabel("trackingPanel.flipHandedness"), &tracking.flipHandedness))
		commitTrackingConfigChange(context);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.flipHandednessTooltip"));
}

void SettingsPanels::drawFusionSection(const TrackingSectionContext& context)
{
	const FusionReadout& fusion= *context.fusion;
	if (fusion.cameraCount <= 1)
		return;

	ImGui::SeparatorText(locText("trackingPanel.fusionSection"));

	if (!fusion.bAvailable)
	{
		ImGui::TextDisabled("%s", locText("stageInspector.notRecorded"));
		return;
	}

	// Which camera won each hand in the shown frame's fusion
	const int leftCam= fusion.dominantCamera[(int)eHandSide::Left];
	const int rightCam= fusion.dominantCamera[(int)eHandSide::Right];
	ImGui::Text(locText("trackingPanel.dominantCameraFmt"),
				leftCam >= 0 ? std::to_string(leftCam + 1).c_str() : "-",
				rightCam >= 0 ? std::to_string(rightCam + 1).c_str() : "-");

	// Per-camera confidence: the number the thresholds act on
	if (ImGui::BeginTable("confidence", 3, ImGuiTableFlags_SizingStretchProp))
	{
		ImGui::TableSetupColumn("");
		ImGui::TableSetupColumn(locText("trackingPanel.left"));
		ImGui::TableSetupColumn(locText("trackingPanel.right"));
		ImGui::TableHeadersRow();
		for (int cameraIndex= 0; cameraIndex < fusion.cameraCount; ++cameraIndex)
		{
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::Text(locText("trackingPanel.cameraFmt"), cameraIndex + 1);
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				ImGui::TableNextColumn();
				const float confidence= cameraIndex < FusionReadout::k_maxCameras
					? fusion.observationConfidence[cameraIndex][sideIndex]
					: -1.f;
				if (confidence < 0.f)
					ImGui::TextDisabled("-");
				else
					ImGui::Text("%.2f", confidence);
			}
		}
		ImGui::EndTable();
	}
}

void SettingsPanels::drawHandScaleSection(const TrackingSectionContext& context)
{
	const AppConfig* config= context.config;
	const FusionReadout& fusion= *context.fusion;
	if (fusion.cameraCount <= 1)
		return;

	ImGui::SeparatorText(locText("trackingPanel.handScaleSection"));
	if (!fusion.bAvailable)
	{
		ImGui::TextDisabled("%s", locText("stageInspector.notRecorded"));
		return;
	}
	// Measured live and applied live - there is nothing to press. The
	// wrist->knuckle bone is re-measured from stereo/depth every session,
	// so persisting it would only save the few seconds the EMA takes to
	// converge, at the cost of a button and a stale value to explain.
	const float scaleFactor= fusion.autoHandScaleFactor;
	const double autoScaleMeters= config->handScale.refLengthMeters * (double)scaleFactor;
	ImGui::Text(locText("trackingPanel.measuredScaleFmt"), autoScaleMeters * 100.0, scaleFactor);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.measuredScaleTooltip"));
}

void SettingsPanels::drawFusedPalmReadout(const TrackingSectionContext& context)
{
	const TrackingFrameResult* fused= context.fusion->fused;
	if (fused == nullptr)
		return;

	ImGui::SeparatorText(locText("trackingPanel.fusedHandsSection"));

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= fused->poses[sideIndex];
		ImGui::Text("%s", locText(sideIndex == (int)eHandSide::Left ? "oscPanel.leftHand" : "oscPanel.rightHand"));
		ImGui::SameLine();
		if (!pose.tracked)
		{
			ImGui::TextDisabled("%s", locText("oscPanel.notTracked"));
			continue;
		}
		const bool bWorld= pose.hasWorldPose;
		const glm::vec3& palmPos= bWorld ? pose.palmPositionWorld : pose.palmPositionCamera;
		ImGui::Text(locText("oscPanel.palmFmt"), palmPos.x, palmPos.y, palmPos.z,
					bWorld ? "" : locText("oscPanel.cameraSpaceSuffix"));
	}
}

void SettingsPanels::drawFusionDiagnosticsSection(const TrackingSectionContext& context)
{
	const FusionReadout& fusion= *context.fusion;

	ImGui::SeparatorText(locText("trackingPanel.fusionDiagnosticsSection"));

	if (fusion.diagnostics == nullptr)
	{
		ImGui::TextDisabled("%s", locText(fusion.bReplay ? "stageInspector.notRecorded"
														 : "trackingPanel.fusionDiagnosticsNone"));
		return;
	}

	// A cluster with no side was dropped or refused and reached no output
	const std::vector<FusionDiagnostics::Cluster>& clusters= fusion.diagnostics->clusters;
	const bool bAnyAssigned= std::any_of(clusters.begin(), clusters.end(),
										 [](const FusionDiagnostics::Cluster& cluster) { return cluster.assignedSide >= 0; });
	if (!bAnyAssigned)
	{
		ImGui::TextDisabled("%s", locText("trackingPanel.fusionDiagnosticsNone"));
		return;
	}

	if (!ImGui::BeginTable("fusionDetail", 6, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
		return;

	ImGui::TableSetupColumn(locText("trackingPanel.fusionHandColumn"));
	ImGui::TableSetupColumn(locText("trackingPanel.fusionCamerasColumn"));
	ImGui::TableSetupColumn(locText("trackingPanel.fusionTriangulatedColumn"));
	ImGui::TableSetupColumn(locText("trackingPanel.fusionResidualColumn"));
	ImGui::TableSetupColumn(locText("trackingPanel.fusionParallaxColumn"));
	ImGui::TableSetupColumn(locText("trackingPanel.fusionEstimatorColumn"));
	ImGui::TableHeadersRow();

	for (const FusionDiagnostics::Cluster& cluster : clusters)
	{
		if (cluster.assignedSide < 0)
			continue;

		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		ImGui::Text("%s", locText(cluster.assignedSide == (int)eHandSide::Left ? "trackingPanel.left"
																			   : "trackingPanel.right"));

		ImGui::TableNextColumn();
		ImGui::Text("%d", (int)cluster.observations.size());

		ImGui::TableNextColumn();
		if (cluster.triVetoed)
			ImGui::TextColored(ImVec4(1.f, 0.85f, 0.3f, 1.f), "%s", locText("trackingPanel.fusionVetoed"));
		else
			ImGui::Text("%s", locText(cluster.triangulated ? "common.yes" : "common.no"));

		// A vetoed pairing was triangulated and measured, and its residual is
		// why it was vetoed
		const bool bTriMeasured= cluster.triangulated || cluster.triVetoed;
		ImGui::TableNextColumn();
		if (bTriMeasured)
			ImGui::Text("%.1f px", cluster.triResidualRmsPx);
		else
			ImGui::TextDisabled("-");

		ImGui::TableNextColumn();
		if (bTriMeasured)
			ImGui::Text("%.1f deg", cluster.triParallaxDeg);
		else
			ImGui::TextDisabled("-");

		ImGui::TableNextColumn();
		if (!cluster.estimatorUsed)
		{
			ImGui::TextDisabled("-");
		}
		else
		{
			ImGui::Text(locText("trackingPanel.fusionEstimatorFmt"), cluster.estimatorCameraCount,
						cluster.estimatorResidualAfterPx);
			if (cluster.estimatorReseeded)
			{
				ImGui::SameLine();
				ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.f, 1.f), "%s", locText("trackingPanel.fusionEstimatorReseeded"));
			}
			if (cluster.estimatorHeldBadFit)
			{
				ImGui::SameLine();
				ImGui::TextColored(ImVec4(1.f, 0.85f, 0.3f, 1.f), "%s", locText("trackingPanel.fusionEstimatorHeld"));
			}
		}
	}
	ImGui::EndTable();
}

void SettingsPanels::drawWristImuEnableControl(const TrackingSectionContext& context)
{
	const bool bChanged= ImGui::Checkbox(locLabel("trackingPanel.enableWristImu"), &context.config->imu.enabled);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.enableWristImuTooltip"));

	if (bChanged)
		commitTrackingConfigChange(context);
}

void SettingsPanels::drawWristImuArmControls(const TrackingSectionContext& context)
{
	AppConfig* config= context.config;

	ImGui::BeginDisabled(!config->imu.enabled);

	if (ImGui::Checkbox(locLabel("trackingPanel.swapWrists"), &config->imu.swapSides))
		commitTrackingConfigChange(context);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.swapWristsTooltip"));

	// No "vision yaw anchor" slider: the value is settled, and it is still
	// in the config file for anyone who needs to retune it
	drawForearmLengthControl(context, "trackingPanel.forearmLengthTooltip");

	ImGui::EndDisabled();
}

void SettingsPanels::drawForearmLengthControl(const TrackingSectionContext& context, const char* tooltipKey)
{
	AppConfig* config= context.config;
	if (ImGui::SliderFloat(locLabel("trackingPanel.forearmLength"), &config->body.forearmLengthMeters, 0.10f, 0.40f,
						   "%.2f m"))
		commitTrackingConfigChange(context);
	ImGui::SetItemTooltip("%s", locText(tooltipKey));
}

void SettingsPanels::drawWristImuDeviceTable(const TrackingSectionContext& context)
{
	const ImuReadout& imu= *context.imu;
	if (!imu.bAvailable)
	{
		ImGui::TextDisabled("%s", locText("stageInspector.notRecorded"));
		return;
	}

	ImGui::BeginDisabled(!context.config->imu.enabled);

	// No scan button: the service rescans on its own while a wrist has no
	// controller, so pairing one in Windows Bluetooth settings is enough
	if (ImGui::BeginTable("imu", 5, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn(locText("trackingPanel.wristTableWrist"));
		ImGui::TableSetupColumn(locText("trackingPanel.wristTableDevice"));
		ImGui::TableSetupColumn(locText("trackingPanel.wristTableRate"));
		ImGui::TableSetupColumn(locText("trackingPanel.wristTableYawDrift"));
		ImGui::TableSetupColumn(locText("trackingPanel.wristTableRollError"));
		ImGui::TableHeadersRow();

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const ImuSideStatus& status= imu.sides[sideIndex];
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::Text("%s", sideIndex == 0 ? locText("trackingPanel.left") : locText("trackingPanel.right"));

			// A recording carries no device name, only that one was connected
			const char* deviceName= imu.bReplay ? locText("trackingPanel.deviceRecorded") : status.deviceName.c_str();
			ImGui::TableNextColumn();
			if (!status.deviceConnected)
				ImGui::TextDisabled("%s", locText("trackingPanel.deviceNone"));
			else if (!status.calibrated)
				ImGui::TextColored(ImVec4(1.f, 0.7f, 0.3f, 1.f), locText("trackingPanel.deviceUncalibratedFmt"),
								   deviceName);
			else if (status.orientationValid)
				ImGui::TextColored(ImVec4(0.4f, 1.f, 0.5f, 1.f), "%s", deviceName);
			else
				ImGui::TextDisabled(locText("trackingPanel.deviceConvergingFmt"), deviceName);

			ImGui::TableNextColumn();
			if (status.streaming)
			{
				ImGui::Text("%.0f Hz  %d%%", status.sampleRateHz, (int)(status.batteryLevel * 100.f));
			}
			else if (status.deviceConnected && status.millisecondsSinceLastSample > 0.0)
			{
				// Silent but open: asleep or the link dropped. The service
				// reopens it automatically; this just makes it visible.
				ImGui::TextColored(ImVec4(1.f, 0.7f, 0.3f, 1.f), locText("trackingPanel.deviceSilentFmt"),
								   status.millisecondsSinceLastSample / 1000.0);
			}
			else
			{
				ImGui::TextDisabled("-");
			}

			ImGui::TableNextColumn();
			// The yaw-axis gyro bias is the number that matters: it is the
			// drift vision has to keep correcting
			if (status.streaming)
				ImGui::Text("%.2f deg/s", status.gyroBiasDegreesPerSecond.z);
			else
				ImGui::TextDisabled("-");

			// Mounting error, measured against anatomy. Preferred over the
			// twist-axis score because it is SIGNED and means something
			// during any motion, not only a deliberate twist.
			ImGui::TableNextColumn();
			if (status.wristAxialTwistDegrees < -900.f)
			{
				ImGui::TextDisabled("%s", locText("trackingPanel.measuring"));
			}
			else
			{
				const float residual= fabsf(status.wristAxialTwistDegrees);
				const ImVec4 color= residual < 5.f
					? ImVec4(0.4f, 1.f, 0.5f, 1.f)
					: (residual < 15.f ? ImVec4(1.f, 0.85f, 0.3f, 1.f) : ImVec4(1.f, 0.4f, 0.4f, 1.f));
				ImGui::TextColored(color, "%+.0f deg", status.wristAxialTwistDegrees);
			}
			ImGui::SetItemTooltip("%s", locText("trackingPanel.wristRollErrorTooltip"));
		}
		ImGui::EndTable();
	}

	ImGui::EndDisabled();
}

void SettingsPanels::drawMountingCalibrationControl(const TrackingSectionContext& context)
{
	ImGui::BeginDisabled(!context.config->imu.enabled);

	if (ImGui::Button(locLabel("trackingPanel.calibrateMounting"), ImVec2(-1, 0)))
		context.state->bLaunchMountingWizard= true;
	ImGui::SetItemTooltip("%s", locText("trackingPanel.calibrateMountingTooltip"));

	ImGui::EndDisabled();
}

void SettingsPanels::drawWristReadoutSection(const TrackingSectionContext& context)
{
	const TrackingFrameResult* fused= context.fusion->fused;
	if (fused == nullptr)
		return;

	ImGui::SeparatorText(locText("trackingPanel.wristReadoutSection"));

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= fused->poses[sideIndex];
		ImGui::Text("%s", locText(sideIndex == (int)eHandSide::Left ? "oscPanel.leftHand" : "oscPanel.rightHand"));
		if (!pose.tracked)
		{
			ImGui::TextDisabled("%s", locText("oscPanel.notTracked"));
			continue;
		}

		// Wrist bend, the rotation between the measured forearm and palm. Shown
		// in DEGREES because it is the number that tells you whether the
		// mounting calibration is good: the wrist rotation is measured relative
		// to the pose you captured, so a straight wrist should read near zero.
		// A large angle with your wrist actually straight means recalibrate (or
		// yaw has drifted).
		if (pose.hasForearmPose && pose.hasWorldPose)
		{
			const glm::quat wristRotation= pose.getWristRotation();
			const float wristDegrees= glm::degrees(2.f * asinf(std::min(
				glm::length(glm::vec3(wristRotation.x, wristRotation.y, wristRotation.z)), 1.f)));

			const ImVec4 wristColor= wristDegrees < 25.f  ? ImVec4(0.4f, 1.f, 0.5f, 1.f)
									 : wristDegrees < 60.f ? ImVec4(1.f, 0.85f, 0.3f, 1.f)
														   : ImVec4(1.f, 0.5f, 0.4f, 1.f);
			ImGui::TextColored(wristColor, locText("oscPanel.wristBendFmt"), wristDegrees);
			ImGui::SetItemTooltip("%s", locText("oscPanel.wristBendTooltip"));

			const glm::quat& forearm= pose.forearmOrientationWorld;
			ImGui::Text(locText("oscPanel.forearmQuatFmt"), forearm.x, forearm.y, forearm.z,
						forearm.w);
			ImGui::Text(locText("oscPanel.wristQuatFmt"), wristRotation.x, wristRotation.y,
						wristRotation.z, wristRotation.w);
		}
		else
		{
			ImGui::TextDisabled("%s", locText("oscPanel.wristNoImu"));
		}
	}
}

void SettingsPanels::drawFaceStreamControls(const TrackingSectionContext& context)
{
	FaceConfig& face= context.config->face;
	bool bFaceChanged= false;
	bFaceChanged|= ImGui::Checkbox(locLabel("trackingPanel.enableFace"), &face.enabled);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.enableFaceTooltip"));

	ImGui::BeginDisabled(!face.enabled);
	bFaceChanged|= ImGui::InputInt(locLabel("trackingPanel.facePort"), &face.port, 0, 0);
	face.port= std::clamp(face.port, 1, 65535);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.facePortTooltip"));

	char addressBuffer[64]= {};
	strncpy(addressBuffer, face.phoneAddress.c_str(), sizeof(addressBuffer) - 1);
	if (ImGui::InputText(locLabel("trackingPanel.facePhoneAddress"), addressBuffer, sizeof(addressBuffer)))
	{
		face.phoneAddress= addressBuffer;
		bFaceChanged= true;
	}
	ImGui::SetItemTooltip("%s", locText("trackingPanel.facePhoneAddressTooltip"));
	ImGui::EndDisabled();

	if (bFaceChanged)
	{
		App::getInstance()->applyFaceConfig();
		commitTrackingConfigChange(context);
	}
}

void SettingsPanels::drawFaceHeadAlignControls(const TrackingSectionContext& context)
{
	FaceConfig& face= context.config->face;
	if (!face.enabled)
		return;

	// The phone's head turns about the world axes once its straight ahead is
	// known; the camera head supplies it (the landed capture is written into
	// the config by updateTrackingState). Capturing needs the live stream.
	const FaceService* faceService= App::getInstance()->getFaceService();
	const bool bStreaming= faceService->getStatus().streaming;
	bool bChanged= false;
	ImGui::BeginDisabled(!bStreaming);
	if (ImGui::Button(locLabel("trackingPanel.faceAlignHead")))
		context.visionThread->requestFaceAnchorCapture();
	ImGui::SetItemTooltip("%s", locText("trackingPanel.faceAlignHeadTooltip"));
	ImGui::EndDisabled();
	if (face.headAnchorPresent)
	{
		ImGui::SameLine();
		if (ImGui::Button(locLabel("trackingPanel.faceResetHeadAlign")))
		{
			face.headAnchorPresent= false;
			face.headAnchor= glm::quat(1.f, 0.f, 0.f, 0.f);
			bChanged= true;
		}
	}
	ImGui::TextDisabled("%s", locText(face.headAnchorPresent ? "trackingPanel.faceHeadAligned"
															 : "trackingPanel.faceHeadNotAligned"));

	if (bChanged)
		commitTrackingConfigChange(context);
}

void SettingsPanels::drawFaceStreamStatus(const TrackingSectionContext& context)
{
	const FaceReadout& face= *context.face;
	if (!face.bAvailable)
	{
		ImGui::TextDisabled("%s", locText("stageInspector.notRecorded"));
		return;
	}
	if (!face.bEnabled)
		return;

	if (!face.bBound)
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), "%s", locText("trackingPanel.faceNotBound"));
	else if (face.bStreaming)
		ImGui::TextColored(ImVec4(0.4f, 1.f, 0.5f, 1.f), locText("trackingPanel.faceStreamingFmt"),
						   face.datagramsPerSecond);
	else if (!face.bReplay)
	{
		// A recording does not say whether a sample had arrived, so a replay
		// tells neither quiet nor waiting
		if (face.bSampleValid)
			ImGui::TextColored(ImVec4(1.f, 0.7f, 0.3f, 1.f), "%s", locText("trackingPanel.faceQuiet"));
		else
			ImGui::TextDisabled("%s", locText("trackingPanel.faceWaiting"));
	}

	// The parser's counts and the head rotation are live only
	if (face.bReplay)
		return;
	ImGui::TextDisabled(locText("trackingPanel.faceCountsFmt"), (unsigned long long)face.unknownNameCount,
						(unsigned long long)face.parseFailures);
	if (!face.lastUnknownName.empty())
		ImGui::TextDisabled(locText("trackingPanel.faceLastUnknownFmt"), face.lastUnknownName.c_str());
	if (face.bHasHead)
		ImGui::TextDisabled(locText("trackingPanel.faceHeadFmt"), face.headEulerDegrees.x, face.headEulerDegrees.y,
							face.headEulerDegrees.z);
}

void SettingsPanels::drawWebcamFaceReadout(const TrackingSectionContext& context)
{
	const FaceReadout& face= *context.face;
	const FaceSources::Webcam& webcam= face.webcam;
	if (!webcam.present)
	{
		ImGui::TextDisabled("%s", locText("trackingPanel.webcamFaceNone"));
		return;
	}
	if (webcam.tracked)
		ImGui::TextColored(ImVec4(0.4f, 1.f, 0.5f, 1.f), "%s", locText("trackingPanel.webcamFaceTracked"));
	else
		ImGui::TextColored(ImVec4(1.f, 0.7f, 0.3f, 1.f), "%s", locText("trackingPanel.webcamFaceLost"));
	if (!face.bWebcamNeutral)
		ImGui::TextWrapped("%s", locText("trackingPanel.webcamFaceNeutralMissing"));

	// The columns the source measures, raw
	ImGui::SeparatorText(locText("trackingPanel.webcamFaceValuesHeader"));
	for (int index= 0; index < ARKIT_BLENDSHAPE_COUNT; ++index)
	{
		if (((webcam.measuredMask >> index) & 1ull) == 0)
			continue;
		char overlay[16];
		snprintf(overlay, sizeof(overlay), "%.2f", webcam.blendshapes[(size_t)index]);
		ImGui::ProgressBar(webcam.blendshapes[(size_t)index], ImVec2(ImGui::GetFontSize() * 8.f, 0.f), overlay);
		ImGui::SameLine();
		ImGui::TextUnformatted(arkitBlendshapeName(index));
	}
}

void SettingsPanels::drawBlinkControls(const TrackingSectionContext& context)
{
	BlinkConfig& blink= context.config->blink;
	bool bChanged= false;
	bChanged|= ImGui::Checkbox(locLabel("trackingPanel.blinkSyncEyes"), &blink.syncEyes);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.blinkSyncEyesTooltip"));
	ImGui::BeginDisabled(blink.syncEyes);
	bChanged|= ImGui::SliderFloat(locLabel("trackingPanel.blinkWinkThreshold"), &blink.winkThreshold,
								  BlinkConfig::k_minWinkThreshold, BlinkConfig::k_maxWinkThreshold, "%.2f");
	ImGui::SetItemTooltip("%s", locText("trackingPanel.blinkWinkThresholdTooltip"));
	ImGui::EndDisabled();
	bChanged|= ImGui::SliderInt(locLabel("trackingPanel.blinkClarity"), &blink.clarity, BlinkConfig::k_minClarity,
								BlinkConfig::k_maxClarity);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.blinkClarityTooltip"));
	bChanged|= ImGui::SliderFloat(locLabel("trackingPanel.blinkSensitivity"), &blink.sensitivity,
								  BlinkConfig::k_minSensitivity, BlinkConfig::k_maxSensitivity, "%.2f");
	ImGui::SetItemTooltip("%s", locText("trackingPanel.blinkSensitivityTooltip"));
	if (bChanged)
	{
		blink.winkThreshold= std::clamp(blink.winkThreshold, BlinkConfig::k_minWinkThreshold, BlinkConfig::k_maxWinkThreshold);
		blink.clarity= std::clamp(blink.clarity, BlinkConfig::k_minClarity, BlinkConfig::k_maxClarity);
		blink.sensitivity= std::clamp(blink.sensitivity, BlinkConfig::k_minSensitivity, BlinkConfig::k_maxSensitivity);
		// The stage reads the settings each frame, so an edit needs only a
		// save. A refresh would end a recording in progress.
		context.config->markDirty();
	}
}

static const char* const k_blinkStateKeys[3]= {"trackingPanel.blinkStateOpen", "trackingPanel.blinkStateClosing",
												"trackingPanel.blinkStateClosed"};

static const ImVec4 k_blinkOpennessColor(0.92f, 0.92f, 0.92f, 1.f);
static const ImVec4 k_blinkBaselineColor(0.45f, 0.82f, 0.45f, 1.f);
static const ImVec4 k_blinkFloorColor(0.42f, 0.62f, 1.f, 1.f);
static const ImVec4 k_blinkStartColor(1.f, 0.75f, 0.3f, 1.f);
static const ImVec4 k_blinkReleaseColor(0.45f, 0.85f, 0.85f, 1.f);
static const ImVec4 k_blinkClosedBandColor(0.42f, 0.62f, 1.f, 0.22f);
static const ImVec4 k_blinkClosingColor(1.f, 0.75f, 0.3f, 0.9f);
static const ImVec4 k_blinkClosedColor(1.f, 0.4f, 0.4f, 0.9f);
static const ImVec4 k_blinkHeldColor(0.78f, 0.5f, 1.f, 0.9f);
static const ImVec4 k_blinkHoldOnColor(0.78f, 0.5f, 1.f, 1.f);
static const ImVec4 k_blinkHoldOffColor(0.6f, 0.6f, 0.6f, 1.f);
// The two output columns. An eye's closure draws in the color of the column
// that reads it.
static const ImVec4 k_blinkOutputColors[2]= {ImVec4(0.95f, 0.6f, 0.3f, 1.f), ImVec4(0.42f, 0.7f, 1.f, 1.f)};
static constexpr float k_blinkClosureAlpha= 0.55f;
static constexpr float k_blinkMainLineWeight= 2.f;
// The strip along a plot's top edge that marks the frames of a closure
static constexpr float k_blinkStateStripPx= 5.f;

enum class eBlinkSeries
{
	Openness,
	Baseline,
	Floor,
	StartLevel,
	ReleaseLevel,
	Closure,
	Difference,
	// index 0 is eyeBlinkLeft and 1 eyeBlinkRight, not a person's side
	Output,
};

// The state a blink readout names: a held wink over the eye's state
static const char* getBlinkStateKey(const FaceSources::Blink& blink, int side)
{
	return blink.bWinkHeld[side] ? "trackingPanel.blinkStateHeld" : k_blinkStateKeys[std::min<int>(blink.state[side], 2)];
}

static float getBlinkSeriesValue(const FaceSources::Blink& blink, eBlinkSeries series, int index)
{
	switch (series)
	{
	case eBlinkSeries::Openness:
		return blink.openness[index];
	case eBlinkSeries::Baseline:
		return blink.baseline[index];
	case eBlinkSeries::Floor:
		return blink.floor[index];
	case eBlinkSeries::StartLevel:
		return blink.startLevel[index];
	case eBlinkSeries::ReleaseLevel:
		return blink.releaseLevel[index];
	case eBlinkSeries::Closure:
		return blink.closure[index];
	case eBlinkSeries::Difference:
		return blink.difference[index];
	case eBlinkSeries::Output:
		return index == 0 ? blink.eyeBlinkLeft : blink.eyeBlinkRight;
	}
	return 0.f;
}

// A sample's place on a blink plot's time axis: seconds before the newest
static double getBlinkPlotSeconds(const BlinkHistory& history, const TrackingPanelState::BlinkSample& sample)
{
	return (sample.timestampMs - history.back().timestampMs) / 1000.0;
}

static void plotBlinkSeries(const char* label, const BlinkHistory& history, eBlinkSeries series, int index,
							const ImVec4& color, float weight)
{
	struct PointSource
	{
		const BlinkHistory* history;
		eBlinkSeries series;
		int index;
	};
	const ImPlotGetter getPoint= [](int sampleIndex, void* data) -> ImPlotPoint {
		const PointSource& source= *static_cast<const PointSource*>(data);
		const TrackingPanelState::BlinkSample& sample= (*source.history)[(size_t)sampleIndex];
		// An untracked frame is a gap in the line
		const double value= sample.blink.tracked
								? (double)getBlinkSeriesValue(sample.blink, source.series, source.index)
								: std::numeric_limits<double>::quiet_NaN();
		return ImPlotPoint(getBlinkPlotSeconds(*source.history, sample), value);
	};
	PointSource source{&history, series, index};
	ImPlotSpec spec;
	spec.LineColor= color;
	spec.LineWeight= weight;
	ImPlot::PlotLineG(label, getPoint, &source, (int)history.size(), spec);
}

struct BlinkLegendEntry
{
	const char* label;
	ImVec4 color;
};

// A plot's legend under it: a swatch and a name per entry, wrapping at the
// available width, which ImPlot's own horizontal legend does not
static void drawBlinkLegend(const BlinkLegendEntry* entries, int entryCount)
{
	constexpr float k_swatchInset= 2.f;
	const ImGuiStyle& style= ImGui::GetStyle();
	const float lineHeight= ImGui::GetTextLineHeight();
	for (int index= 0; index < entryCount; ++index)
	{
		const BlinkLegendEntry& entry= entries[index];
		const float width= lineHeight + style.ItemInnerSpacing.x + ImGui::CalcTextSize(entry.label).x;
		if (index > 0)
		{
			ImGui::SameLine(0.f, style.ItemSpacing.x * 2.f);
			if (ImGui::GetContentRegionAvail().x < width)
				ImGui::NewLine();
		}
		const ImVec2 origin= ImGui::GetCursorScreenPos();
		ImGui::GetWindowDrawList()->AddRectFilled(
			ImVec2(origin.x + k_swatchInset, origin.y + k_swatchInset),
			ImVec2(origin.x + lineHeight - k_swatchInset, origin.y + lineHeight - k_swatchInset),
			ImGui::ColorConvertFloat4ToU32(entry.color));
		ImGui::Dummy(ImVec2(lineHeight, lineHeight));
		ImGui::SameLine(0.f, style.ItemInnerSpacing.x);
		ImGui::TextUnformatted(entry.label);
	}
}

// The axes are locked: time is seconds before the shown frame, and the value
// runs from zero to yMax
static bool beginBlinkPlot(const char* id, float heightInFontSizes, double yMax)
{
	constexpr ImPlotFlags k_plotFlags= ImPlotFlags_NoTitle | ImPlotFlags_NoLegend | ImPlotFlags_NoMenus |
									   ImPlotFlags_NoBoxSelect | ImPlotFlags_NoMouseText;
	constexpr ImPlotAxisFlags k_axisFlags=
		ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoHighlight | ImPlotAxisFlags_NoSideSwitch;
	if (!ImPlot::BeginPlot(id, ImVec2(-1.f, ImGui::GetFontSize() * heightInFontSizes), k_plotFlags))
		return false;
	ImPlot::SetupAxes(nullptr, nullptr, k_axisFlags, k_axisFlags);
	ImPlot::SetupAxisLimits(ImAxis_X1, -k_blinkHistoryWindowMs / 1000.0, 0.0, ImPlotCond_Always);
	ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, yMax, ImPlotCond_Always);
	ImPlot::SetupAxisFormat(ImAxis_X1, "%g s");
	return true;
}

// The tracked sample nearest the mouse over the current plot, with a cursor
// drawn at it. nullptr while the plot is not hovered or the sample is a gap.
static const TrackingPanelState::BlinkSample* drawBlinkPlotCursor(const BlinkHistory& history, double yMax)
{
	if (!ImPlot::IsPlotHovered() || history.empty())
		return nullptr;
	const double hoverMs= history.back().timestampMs + ImPlot::GetPlotMousePos().x * 1000.0;
	auto nearest= std::lower_bound(
		history.begin(), history.end(), hoverMs,
		[](const TrackingPanelState::BlinkSample& sample, double timestampMs) { return sample.timestampMs < timestampMs; });
	if (nearest == history.end())
		--nearest;
	else if (nearest != history.begin() && hoverMs - (nearest - 1)->timestampMs < nearest->timestampMs - hoverMs)
		--nearest;
	const double seconds= getBlinkPlotSeconds(history, *nearest);

	ImPlot::PushPlotClipRect();
	ImPlot::GetPlotDrawList()->AddLine(ImPlot::PlotToPixels(ImPlotPoint(seconds, yMax)),
									   ImPlot::PlotToPixels(ImPlotPoint(seconds, 0.0)), IM_COL32(255, 255, 255, 110),
									   1.f);
	ImPlot::PopPlotClipRect();
	return nearest->blink.tracked ? &*nearest : nullptr;
}

static void drawBlinkTooltipRow(const char* label, const ImVec4& color, const char* format, float value)
{
	ImGui::TextColored(color, "%s", label);
	ImGui::SameLine();
	ImGui::Text(format, value);
}

// One eye, by the person's own side: the openness over the levels it is read
// against, the closed band shaded, and a strip along the top over the frames
// of a closure
static void drawBlinkEyePlot(const BlinkHistory& history, int side)
{
	// The range never shrinks below a resting eye's, so a quiet history does
	// not zoom into noise
	float top= 0.4f;
	for (const TrackingPanelState::BlinkSample& sample : history)
	{
		if (sample.blink.tracked)
			top= std::max(top, std::max(sample.blink.openness[side], sample.blink.baseline[side]));
	}
	const double yMax= (double)top * 1.1;
	if (!beginBlinkPlot(side == 0 ? "##blinkEyeLeft" : "##blinkEyeRight", 11.f, yMax))
		return;

	// Under the lines. The fills butt against each other, so they draw
	// without the antialiased fringe that would show as seams.
	ImDrawList* drawList= ImPlot::GetPlotDrawList();
	const ImDrawListFlags savedFlags= drawList->Flags;
	drawList->Flags&= ~ImDrawListFlags_AntiAliasedFill;
	ImPlot::PushPlotClipRect();
	const float stripTop= ImPlot::GetPlotPos().y;
	const ImU32 bandColor= ImGui::ColorConvertFloat4ToU32(k_blinkClosedBandColor);
	for (size_t index= 1; index < history.size(); ++index)
	{
		const FaceSources::Blink& from= history[index - 1].blink;
		const FaceSources::Blink& to= history[index].blink;
		if (!from.tracked || !to.tracked)
			continue;
		const double fromSeconds= getBlinkPlotSeconds(history, history[index - 1]);
		const double toSeconds= getBlinkPlotSeconds(history, history[index]);
		drawList->AddQuadFilled(ImPlot::PlotToPixels(ImPlotPoint(fromSeconds, (double)from.floor[side])),
								ImPlot::PlotToPixels(ImPlotPoint(toSeconds, (double)to.floor[side])),
								ImPlot::PlotToPixels(ImPlotPoint(toSeconds, (double)to.closedLevel[side])),
								ImPlot::PlotToPixels(ImPlotPoint(fromSeconds, (double)from.closedLevel[side])),
								bandColor);
		if (to.state[side] != (uint8_t)WebcamBlinkFilter::eEyeState::Open)
		{
			const bool bClosed= to.state[side] == (uint8_t)WebcamBlinkFilter::eEyeState::Closed;
			const ImVec4& stripColor= to.bWinkHeld[side] ? k_blinkHeldColor
									  : bClosed			 ? k_blinkClosedColor
														 : k_blinkClosingColor;
			const float fromX= ImPlot::PlotToPixels(ImPlotPoint(fromSeconds, 0.0)).x;
			const float toX= ImPlot::PlotToPixels(ImPlotPoint(toSeconds, 0.0)).x;
			drawList->AddRectFilled(ImVec2(fromX, stripTop), ImVec2(toX, stripTop + k_blinkStateStripPx),
									ImGui::ColorConvertFloat4ToU32(stripColor));
		}
	}
	ImPlot::PopPlotClipRect();
	drawList->Flags= savedFlags;

	plotBlinkSeries(locText("trackingPanel.blinkSeriesBaseline"), history, eBlinkSeries::Baseline, side,
					k_blinkBaselineColor, 1.f);
	plotBlinkSeries(locText("trackingPanel.blinkSeriesStart"), history, eBlinkSeries::StartLevel, side,
					k_blinkStartColor, 1.f);
	plotBlinkSeries(locText("trackingPanel.blinkSeriesRelease"), history, eBlinkSeries::ReleaseLevel, side,
					k_blinkReleaseColor, 1.f);
	plotBlinkSeries(locText("trackingPanel.blinkSeriesFloor"), history, eBlinkSeries::Floor, side, k_blinkFloorColor,
					1.f);
	plotBlinkSeries(locText("trackingPanel.blinkSeriesOpenness"), history, eBlinkSeries::Openness, side,
					k_blinkOpennessColor, k_blinkMainLineWeight);

	if (const TrackingPanelState::BlinkSample* hovered= drawBlinkPlotCursor(history, yMax))
	{
		const FaceSources::Blink& blink= hovered->blink;
		ImGui::BeginTooltip();
		ImGui::TextDisabled("%.2f s", getBlinkPlotSeconds(history, *hovered));
		ImGui::SameLine();
		ImGui::TextUnformatted(locText(getBlinkStateKey(blink, side)));
		drawBlinkTooltipRow(locText("trackingPanel.blinkSeriesOpenness"), k_blinkOpennessColor, "%.3f",
							blink.openness[side]);
		drawBlinkTooltipRow(locText("trackingPanel.blinkSeriesBaseline"), k_blinkBaselineColor, "%.3f",
							blink.baseline[side]);
		drawBlinkTooltipRow(locText("trackingPanel.blinkSeriesStart"), k_blinkStartColor, "%.3f",
							blink.startLevel[side]);
		drawBlinkTooltipRow(locText("trackingPanel.blinkSeriesRelease"), k_blinkReleaseColor, "%.3f",
							blink.releaseLevel[side]);
		drawBlinkTooltipRow(locText("trackingPanel.blinkSeriesClosedBand"), k_blinkFloorColor, "%.3f",
							blink.closedLevel[side]);
		drawBlinkTooltipRow(locText("trackingPanel.blinkSeriesFloor"), k_blinkFloorColor, "%.3f", blink.floor[side]);
		ImGui::EndTooltip();
	}
	ImPlot::EndPlot();

	const BlinkLegendEntry legend[]= {
		{locText("trackingPanel.blinkSeriesOpenness"), k_blinkOpennessColor},
		{locText("trackingPanel.blinkSeriesBaseline"), k_blinkBaselineColor},
		{locText("trackingPanel.blinkSeriesStart"), k_blinkStartColor},
		{locText("trackingPanel.blinkSeriesRelease"), k_blinkReleaseColor},
		{locText("trackingPanel.blinkSeriesFloor"), k_blinkFloorColor},
		{locText("trackingPanel.blinkSeriesClosedBand"), k_blinkClosedBandColor},
		{locText("trackingPanel.blinkStateClosing"), k_blinkClosingColor},
		{locText("trackingPanel.blinkStateClosed"), k_blinkClosedColor},
		{locText("trackingPanel.blinkStateHeld"), k_blinkHeldColor},
	};
	drawBlinkLegend(legend, IM_ARRAYSIZE(legend));
}

// The two output columns over each eye's closure before the eyes are linked
static void drawBlinkOutputPlot(const BlinkHistory& history)
{
	constexpr double k_yMax= 1.05;
	if (!beginBlinkPlot("##blinkOutput", 9.f, k_yMax))
		return;

	static const char* const s_outputNames[2]= {"eyeBlinkLeft", "eyeBlinkRight"};
	static const char* const s_closureKeys[2]= {"trackingPanel.blinkSeriesClosureLeft",
												"trackingPanel.blinkSeriesClosureRight"};
	// The person's own side each output column reads
	static constexpr int s_sideOfOutput[2]= {k_ratioSideOfArkitLeft, k_ratioSideOfArkitRight};
	for (int output= 0; output < 2; ++output)
	{
		ImVec4 closureColor= k_blinkOutputColors[output];
		closureColor.w= k_blinkClosureAlpha;
		plotBlinkSeries(locText(s_closureKeys[s_sideOfOutput[output]]), history, eBlinkSeries::Closure,
						s_sideOfOutput[output], closureColor, 1.f);
	}
	for (int output= 0; output < 2; ++output)
	{
		plotBlinkSeries(s_outputNames[output], history, eBlinkSeries::Output, output, k_blinkOutputColors[output],
						k_blinkMainLineWeight);
	}

	if (const TrackingPanelState::BlinkSample* hovered= drawBlinkPlotCursor(history, k_yMax))
	{
		const FaceSources::Blink& blink= hovered->blink;
		ImGui::BeginTooltip();
		ImGui::TextDisabled("%.2f s", getBlinkPlotSeconds(history, *hovered));
		for (int output= 0; output < 2; ++output)
		{
			ImVec4 closureColor= k_blinkOutputColors[output];
			closureColor.w= k_blinkClosureAlpha;
			drawBlinkTooltipRow(s_outputNames[output], k_blinkOutputColors[output], "%.2f",
								getBlinkSeriesValue(blink, eBlinkSeries::Output, output));
			drawBlinkTooltipRow(locText(s_closureKeys[s_sideOfOutput[output]]), closureColor, "%.2f",
								blink.closure[s_sideOfOutput[output]]);
		}
		ImGui::EndTooltip();
	}
	ImPlot::EndPlot();

	BlinkLegendEntry legend[4];
	for (int output= 0; output < 2; ++output)
	{
		legend[output]= {s_outputNames[output], k_blinkOutputColors[output]};
		legend[2 + output]= {locText(s_closureKeys[s_sideOfOutput[output]]), k_blinkOutputColors[output]};
		legend[2 + output].color.w= k_blinkClosureAlpha;
	}
	drawBlinkLegend(legend, IM_ARRAYSIZE(legend));
}

// How much more closed than the other eye each eye reads, over the levels a
// wink is held and released at. An eye's line is in view only while it is the
// more closed one.
static void drawBlinkDifferencePlot(const BlinkHistory& history)
{
	constexpr double k_yMax= 0.4;
	if (!beginBlinkPlot("##blinkDifference", 7.f, k_yMax))
		return;

	static const char* const s_differenceKeys[2]= {"trackingPanel.blinkSeriesDifferenceLeft",
												   "trackingPanel.blinkSeriesDifferenceRight"};
	// An eye draws in the color of the output column that reads it
	const ImVec4 sideColors[2]= {k_blinkOutputColors[k_ratioSideOfArkitLeft == 0 ? 0 : 1],
								 k_blinkOutputColors[k_ratioSideOfArkitLeft == 1 ? 0 : 1]};
	const double holdOn= (double)WebcamBlinkFilter::k_winkHoldOnDifference;
	const double holdOff= (double)WebcamBlinkFilter::k_winkHoldOffDifference;
	ImPlotSpec levelSpec;
	levelSpec.Flags= ImPlotInfLinesFlags_Horizontal;
	levelSpec.LineColor= k_blinkHoldOnColor;
	ImPlot::PlotInfLines(locText("trackingPanel.blinkSeriesHoldOn"), &holdOn, 1, levelSpec);
	levelSpec.LineColor= k_blinkHoldOffColor;
	ImPlot::PlotInfLines(locText("trackingPanel.blinkSeriesHoldOff"), &holdOff, 1, levelSpec);
	for (int side= 0; side < 2; ++side)
	{
		plotBlinkSeries(locText(s_differenceKeys[side]), history, eBlinkSeries::Difference, side, sideColors[side],
						k_blinkMainLineWeight);
	}

	if (const TrackingPanelState::BlinkSample* hovered= drawBlinkPlotCursor(history, k_yMax))
	{
		ImGui::BeginTooltip();
		ImGui::TextDisabled("%.2f s", getBlinkPlotSeconds(history, *hovered));
		for (int side= 0; side < 2; ++side)
		{
			drawBlinkTooltipRow(locText(s_differenceKeys[side]), sideColors[side], "%.3f",
								hovered->blink.difference[side]);
		}
		ImGui::EndTooltip();
	}
	ImPlot::EndPlot();

	const BlinkLegendEntry legend[]= {
		{locText(s_differenceKeys[0]), sideColors[0]},
		{locText(s_differenceKeys[1]), sideColors[1]},
		{locText("trackingPanel.blinkSeriesHoldOn"), k_blinkHoldOnColor},
		{locText("trackingPanel.blinkSeriesHoldOff"), k_blinkHoldOffColor},
	};
	drawBlinkLegend(legend, IM_ARRAYSIZE(legend));
}

void SettingsPanels::drawBlinkReadout(const TrackingSectionContext& context)
{
	const FaceSources::Blink& blink= context.face->blink;
	if (!blink.present)
	{
		ImGui::TextDisabled("%s", locText("trackingPanel.webcamFaceNone"));
		return;
	}
	// A lost face keeps the plots, with a gap where it was lost
	if (!blink.tracked)
		ImGui::TextColored(ImVec4(1.f, 0.7f, 0.3f, 1.f), "%s", locText("trackingPanel.webcamFaceLost"));
	else if (!blink.bBaselineSettled)
		ImGui::TextWrapped("%s", locText("trackingPanel.blinkSettling"));

	const BlinkHistory& history= context.state->blinkHistory;
	static const char* const s_sideKeys[2]= {"trackingPanel.blinkEyeLeft", "trackingPanel.blinkEyeRight"};
	for (int side= 0; side < 2; ++side)
	{
		ImGui::SeparatorText(locText(s_sideKeys[side]));
		if (blink.tracked)
		{
			ImGui::Text(locText("trackingPanel.blinkLevelsFmt"), blink.openness[side], blink.baseline[side],
						blink.floor[side]);
			ImGui::SameLine();
			ImGui::TextUnformatted(locText(getBlinkStateKey(blink, side)));
		}
		drawBlinkEyePlot(history, side);
	}

	ImGui::SeparatorText(locText("trackingPanel.blinkDifferenceHeader"));
	drawBlinkDifferencePlot(history);

	// The output, in ARKit columns, over each eye's closure before the link
	ImGui::SeparatorText(locText("trackingPanel.blinkOutputHeader"));
	drawBlinkOutputPlot(history);
}

void SettingsPanels::drawBodyPoseIntro(const TrackingSectionContext& context)
{
	const AppConfig* shownConfig= context.shownConfig != nullptr ? context.shownConfig : context.config;
	const bool bDirect= shownConfig != nullptr && isDirectProject(*shownConfig);
	ImGui::TextWrapped("%s", locText(bDirect ? "trackingPanel.bodyPoseDirectIntro" : "trackingPanel.bodyPoseIntro"));
}

// The body model the shown frame's camera runs, none where it runs none or
// the shown config does not have the camera
static std::optional<eBodyPoseBackend> getShownBodyPoseBackend(const TrackingSectionContext& context,
															   int cameraIndex)
{
	const std::vector<std::optional<eBodyPoseBackend>>* backends= context.shownBodyBackends;
	if (backends == nullptr || cameraIndex < 0 || cameraIndex >= (int)backends->size())
		return std::nullopt;
	return (*backends)[cameraIndex];
}

void SettingsPanels::drawBodyPoseCameraControls(const TrackingSectionContext& context, int cameraIndex)
{
	AppConfig* config= context.config;
	const AppConfig* shownConfig= context.shownConfig != nullptr ? context.shownConfig : config;
	ImGui::PushID(cameraIndex);

	// The switch is a multi-camera project's choice, and edits the live
	// config: a direct project decides body pose from its mode
	bool bChanged= false;
	if (!isDirectProject(*config) && cameraIndex >= 0 && cameraIndex < (int)config->cameraCount())
	{
		const std::string label= locFormat("trackingPanel.cameraFmt", cameraIndex + 1);
		bChanged|= ImGui::Checkbox(label.c_str(), &config->camera(cameraIndex).bodyPose.enabled);
	}

	// What the shown project runs on this camera, and its model
	if (isDirectProject(*shownConfig))
	{
		const bool bDirectCamera= cameraIndex == shownConfig->direct.cameraIndex;
		ImGui::TextWrapped("%s", locText(bDirectCamera ? "trackingPanel.bodyPoseDirectCamera"
													   : "trackingPanel.bodyPoseDirectOtherCamera"));
	}
	const std::optional<eBodyPoseBackend> backend= getShownBodyPoseBackend(context, cameraIndex);
	if (backend.has_value())
	{
		ImGui::TextDisabled("%s", locText(*backend == eBodyPoseBackend::Rtmw3dWholeBody
											  ? "trackingPanel.bodyBackendRtmw3d"
											  : "trackingPanel.bodyBackendRtmPose"));
		ImGui::SetItemTooltip("%s", locText("trackingPanel.bodyBackendTooltip"));
	}

	ImGui::PopID();

	if (bChanged)
		commitTrackingConfigChange(context);
}

void SettingsPanels::drawBodyCalibrationControl(const TrackingSectionContext& context)
{
	ImGui::TextWrapped("%s", locText("trackingPanel.bodyPoseProportionsIntro"));

	if (ImGui::Button(locLabel("trackingPanel.measureMyBody"), ImVec2(-1, 0)))
		context.state->bLaunchBodyCalibrationWizard= true;
	ImGui::SetItemTooltip("%s", locText("trackingPanel.measureMyBodyTooltip"));
}

void SettingsPanels::drawBodyDimensionControls(const TrackingSectionContext& context,
												const BodyDimensionTooltips& tooltips)
{
	AppConfig* config= context.config;

	bool bChanged= false;
	bChanged|=
		ImGui::Checkbox(locLabel("trackingPanel.deriveUpperArmFromShoulderWidth"), &config->body.bDeriveUpperArmFromShoulderWidth);
	// The tooltip is its own format string (the multi-camera one carries a
	// literal percent)
	ImGui::SetItemTooltip(locText(tooltips.deriveUpperArm));
	if (config->body.bDeriveUpperArmFromShoulderWidth)
	{
		bChanged|= ImGui::SliderFloat(locLabel("trackingPanel.upperArmRatio"), &config->body.upperArmPerShoulderWidth, 0.80f,
									  1.40f, locText("trackingPanel.upperArmRatioFmt"));
		ImGui::SetItemTooltip("%s", locText(tooltips.upperArmRatio));
		ImGui::TextDisabled(locText("trackingPanel.upperArmDerivedFmt"), config->body.shoulderWidthMeters *
														 config->body.upperArmPerShoulderWidth * 100.f);
	}
	else
	{
		bChanged|= ImGui::SliderFloat(locLabel("trackingPanel.upperArm"), &config->body.upperArmLengthMeters, 0.20f, 0.45f,
									  "%.2f m");
		ImGui::SetItemTooltip("%s", locText(tooltips.upperArm));
	}
	bChanged|= ImGui::SliderFloat(locLabel("trackingPanel.shoulderWidth"), &config->body.shoulderWidthMeters, 0.25f, 0.60f, "%.2f m");
	ImGui::SetItemTooltip("%s", locText(tooltips.shoulderWidth));
	bChanged|= ImGui::SliderFloat(locLabel("trackingPanel.headWidth"), &config->body.headWidthMeters, 0.10f, 0.22f, "%.2f m");
	ImGui::SetItemTooltip("%s", locText(tooltips.headWidth));
	bChanged|= ImGui::SliderFloat(locLabel("trackingPanel.noseForward"), &config->body.noseForwardMeters, 0.05f, 0.18f, "%.2f m");
	ImGui::SetItemTooltip("%s", locText(tooltips.noseForward));

	if (bChanged)
		commitTrackingConfigChange(context);
}

void SettingsPanels::drawBodySolveReadout(const TrackingSectionContext& context)
{
	const TrackingFrameResult* fused= context.fusion->fused;
	if (fused == nullptr)
		return;

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= fused->poses[sideIndex];
		ImGui::Text("%s", locText(sideIndex == (int)eHandSide::Left ? "oscPanel.leftHand" : "oscPanel.rightHand"));

		// The shoulder is torso-fixed, so the solver places it with or without
		// a tracked hand
		if (pose.hasShoulder)
		{
			const glm::vec3& shoulder= pose.shoulderPositionWorld;
			ImGui::Text(locText("trackingPanel.bodyShoulderFmt"), shoulder.x, shoulder.y, shoulder.z,
						pose.shoulderConfidence);
		}
		else
		{
			ImGui::TextDisabled("%s", locText("trackingPanel.bodyNoShoulder"));
		}

		if (!pose.tracked)
		{
			ImGui::TextDisabled("%s", locText("oscPanel.notTracked"));
			continue;
		}

		if (pose.hasForearmPose && pose.hasWorldPose)
		{
			const glm::vec3 elbow= pose.getElbowPositionWorld(context.config->body.forearmLengthMeters);
			ImGui::Text(locText("oscPanel.elbowFmt"), elbow.x, elbow.y, elbow.z);
			ImGui::Text(locText("trackingPanel.bodyForearmConfidenceFmt"), pose.forearmConfidence);
		}
		else
		{
			ImGui::TextDisabled("%s", locText("trackingPanel.bodyNoForearm"));
		}
	}

	ImGui::Text("%s", locText("avatarPanel.sectionHead"));
	if (fused->head.valid)
	{
		const glm::vec3& head= fused->head.positionWorld;
		ImGui::Text(locText("trackingPanel.bodyHeadFmt"), head.x, head.y, head.z, fused->head.confidence);
	}
	else
	{
		ImGui::TextDisabled("%s", locText("oscPanel.notTracked"));
	}
}

// -- Direct solver -------------------------------------------------------------

void SettingsPanels::drawDirectCameraControl(const TrackingSectionContext& context)
{
	AppConfig* config= context.config;
	DirectConfig& direct= config->direct;
	bool bChanged= false;

	// The config keeps the index in its camera list. A project that is not
	// direct (one replaying a direct take) has no role to move.
	if (config->cameraCount() > 1)
	{
		ImGui::BeginDisabled(!isDirectProject(*config));
		const std::string selectedLabel= locFormat("trackingPanel.cameraFmt", direct.cameraIndex + 1);
		if (ImGui::BeginCombo(locLabel("trackingPanel.directCamera"), selectedLabel.c_str()))
		{
			for (int cameraIndex= 0; cameraIndex < (int)config->cameraCount(); ++cameraIndex)
			{
				ImGui::PushID(cameraIndex);
				const std::string optionLabel= locFormat("trackingPanel.cameraFmt", cameraIndex + 1);
				if (ImGui::Selectable(optionLabel.c_str(), direct.cameraIndex == cameraIndex) &&
					direct.cameraIndex != cameraIndex)
				{
					direct.cameraIndex= cameraIndex;
					bChanged= true;
				}
				ImGui::PopID();
			}
			ImGui::EndCombo();
		}
		ImGui::EndDisabled();
	}

	if (bChanged)
	{
		onDirectCameraMoved(context);
		commitTrackingConfigChange(context);
	}
}

void SettingsPanels::onDirectCameraMoved(const TrackingSectionContext& context)
{
	// The neutral belongs to the camera it was captured through, and a
	// countdown started for the old camera would capture through the new
	// one. A request already sent lands tagged with the old index and is
	// dropped.
	clearDirectNeutral(context.config->direct);
	context.state->directNeutral.reset();
}

void SettingsPanels::drawDirectTiltControl(const TrackingSectionContext& context)
{
	DirectConfig& direct= context.config->direct;
	if (ImGui::SliderFloat(locLabel("trackingPanel.directTilt"), &direct.cameraTiltDegrees, -30.f, 60.f,
						   locText("trackingPanel.directTiltFmt")))
		commitTrackingConfigChange(context);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.directTiltTooltip"));
}

void SettingsPanels::drawDirectWristDepthControls(const TrackingSectionContext& context)
{
	DirectConfig& direct= context.config->direct;
	bool bChanged= ImGui::SliderFloat(locLabel("trackingPanel.directFarPlane"), &direct.farPlaneReachFraction, 0.1f, 1.f,
									  "%.2f");
	ImGui::SetItemTooltip("%s", locText("trackingPanel.directFarPlaneTooltip"));
	bChanged|=
		ImGui::SliderFloat(locLabel("trackingPanel.directNearPlane"), &direct.nearPlaneReachFraction, 0.f, 1.f, "%.2f");
	ImGui::SetItemTooltip("%s", locText("trackingPanel.directNearPlaneTooltip"));
	if (bChanged)
		commitTrackingConfigChange(context);
}

void SettingsPanels::drawDirectNeutralControl(const TrackingSectionContext& context)
{
	DirectConfig& direct= context.config->direct;
	bool bChanged= false;

	// The countdown and the landed capture run in updateTrackingState
	NeutralCaptureControl& neutral= context.state->directNeutral;
	ImGui::BeginDisabled(neutral.isCountingDown() || !direct.enabled);
	if (ImGui::Button(locLabel("trackingPanel.directCapture")))
		neutral.start();
	ImGui::SetItemTooltip("%s", locText("trackingPanel.directCaptureTooltip"));
	ImGui::EndDisabled();

	if (direct.neutralPresent)
	{
		ImGui::SameLine();
		if (ImGui::Button(locLabel("trackingPanel.directClear")))
		{
			clearDirectNeutral(direct);
			bChanged= true;
		}
	}

	if (neutral.isCountingDown())
	{
		ImGui::TextColored(ImVec4(1.f, 0.85f, 0.3f, 1.f), locText("trackingPanel.directCountdownFmt"),
						   (int)ceilf(neutral.getRemainingSeconds()));
		ImGui::ProgressBar(1.f - neutral.getRemainingSeconds() / NeutralCaptureControl::k_countdownSeconds,
						   ImVec2(-1, 4), "");
	}

	neutral.drawStatus(direct);

	if (bChanged)
		commitTrackingConfigChange(context);
}

// The shown frame's direct solve, or null after one dim line saying it has
// none
static const DirectRetarget::SolveState* getShownDirectSolve(const TrackingSectionContext& context)
{
	const DirectRetarget::SolveState* state= context.direct != nullptr ? context.direct->state : nullptr;
	if (state == nullptr)
		ImGui::TextDisabled("%s", locText("directStage.noSolve"));
	return state;
}

// A readout table: a label column, then one value column or one per side
// under the side headers
static bool beginDirectTable(const char* id, bool bPerSide)
{
	if (!ImGui::BeginTable(id, bPerSide ? 3 : 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
		return false;
	ImGui::TableSetupColumn("");
	if (bPerSide)
	{
		ImGui::TableSetupColumn(locText("trackingPanel.left"));
		ImGui::TableSetupColumn(locText("trackingPanel.right"));
		ImGui::TableHeadersRow();
	}
	else
	{
		ImGui::TableSetupColumn("");
	}
	return true;
}

// A new row with its label, the label carrying the values' unit
static void drawDirectRowLabel(const char* labelKey)
{
	ImGui::TableNextRow();
	ImGui::TableNextColumn();
	ImGui::TextUnformatted(locText(labelKey));
}

static void drawDirectTextCell(const char* text)
{
	ImGui::TableNextColumn();
	ImGui::TextUnformatted(text);
}

static void drawDirectYesNoCell(bool bYes)
{
	drawDirectTextCell(locText(bYes ? "common.yes" : "common.no"));
}

// The value, or a dim dash where the solve did not measure it
static void drawDirectValueCell(bool bMeasured, const char* format, float value)
{
	ImGui::TableNextColumn();
	if (bMeasured)
		ImGui::Text(format, value);
	else
		ImGui::TextDisabled("-");
}

void SettingsPanels::drawDirectSolveSummary(const TrackingSectionContext& context)
{
	ImGui::SeparatorText(locText("directStage.solveSection"));
	const DirectRetarget::SolveState* state= getShownDirectSolve(context);
	if (state == nullptr || !beginDirectTable("directSummary", false))
		return;

	drawDirectRowLabel("directStage.person");
	drawDirectTextCell(locText(state->torsoValid ? "directStage.torsoSeen" : "directStage.noPerson"));
	// The undistorted pinhole the landmarks were read in
	drawDirectRowLabel("directStage.cameraModel");
	ImGui::TableNextColumn();
	ImGui::Text(locText("directStage.cameraModelFmt"), state->camera.width, state->camera.height, state->camera.fx,
				state->camera.fy);
	drawDirectRowLabel("directStage.tilt");
	drawDirectValueCell(true, "%.1f", glm::degrees(state->tiltRadians));
	ImGui::EndTable();
}

void SettingsPanels::drawDirectFiltersReadout(const TrackingSectionContext& context)
{
	const DirectRetarget::SolveState* state= getShownDirectSolve(context);
	if (state == nullptr || !beginDirectTable("directFilters", true))
		return;

	drawDirectRowLabel("directStage.handAgrees");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
		drawDirectYesNoCell(arm.bHandSeen);
	drawDirectRowLabel("directStage.elbowGate");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
		drawDirectYesNoCell(arm.bElbowSeen);
	ImGui::EndTable();

	// How often a label has cost a hand since the solver started
	if (state->oppositeWristRejections[0] > 0 || state->oppositeWristRejections[1] > 0)
	{
		ImGui::TextDisabled(locText("directStage.oppositeWristRejectionsFmt"), state->oppositeWristRejections[0],
							state->oppositeWristRejections[1]);
	}

	// And how often the camera's correction kept one
	if (state->sideCorrections[0] > 0 || state->sideCorrections[1] > 0)
	{
		ImGui::TextDisabled(locText("directStage.sideCorrectionsFmt"), state->sideCorrections[0],
							state->sideCorrections[1]);
	}

	// Labels the wrong way round as a standing condition: the correction
	// carries the hands while the body model sees them, and the fix is this
	// one setting
	if (state->bHandednessSwapped)
	{
		ImGui::PushTextWrapPos(0.f);
		ImGui::TextColored(ImVec4(1.f, 0.85f, 0.3f, 1.f), "%s", locText("directStage.handednessSwapped"));
		ImGui::PopTextWrapPos();
		drawFlipHandednessSection(context);
	}
}

void SettingsPanels::drawDirectTorsoReadout(const TrackingSectionContext& context)
{
	const DirectRetarget::SolveState* state= getShownDirectSolve(context);
	if (state == nullptr || !beginDirectTable("directTorso", false))
		return;

	const DirectReadout& readout= *context.direct;
	drawDirectRowLabel("directStage.person");
	drawDirectTextCell(locText(state->torsoValid ? "directStage.torsoSeen" : "directStage.noPerson"));
	drawDirectRowLabel("directStage.shoulderDepth");
	drawDirectValueCell(state->torsoValid, "%.2f", readout.shoulderDepth);
	// The lean is measured against the neutral's torso, so it needs one
	drawDirectRowLabel("directStage.lean");
	drawDirectValueCell(state->torsoValid && readout.bNeutralPresent, "%.0f", readout.leanDegrees);
	drawDirectRowLabel("directStage.head");
	drawDirectTextCell(locText(state->headMeasured ? "directStage.headMeasured" : "directStage.headNotMeasured"));
	drawDirectRowLabel("directStage.neutral");
	drawDirectTextCell(locText(readout.bNeutralPresent ? "directStage.neutralCaptured" : "directStage.neutralNone"));
	ImGui::EndTable();
}

void SettingsPanels::drawDirectArmChainReadout(const TrackingSectionContext& context)
{
	const DirectRetarget::SolveState* state= getShownDirectSolve(context);
	if (state == nullptr || !beginDirectTable("directArmChain", true))
		return;

	// Every row is measured only where the wrist's ray crosses the arm's
	// reach, and the interval is never empty there
	drawDirectRowLabel("directStage.plane");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
	{
		if (arm.depthHigh > 0.f)
			drawDirectTextCell(locText(arm.bNearPlane ? "directStage.planeNear" : "directStage.planeFar"));
		else
			drawDirectValueCell(false, "", 0.f);
	}
	drawDirectRowLabel("directStage.planeDistance");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
		drawDirectValueCell(arm.depthHigh > 0.f, "%.2f", arm.planeDistance);
	drawDirectRowLabel("directStage.wristDistance");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
		drawDirectValueCell(arm.depthHigh > 0.f, "%.2f", arm.wristDepth);
	drawDirectRowLabel("directStage.reachInterval");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
	{
		ImGui::TableNextColumn();
		if (arm.depthHigh > 0.f)
			ImGui::Text(locText("directStage.intervalFmt"), arm.depthLow, arm.depthHigh);
		else
			ImGui::TextDisabled("-");
	}
	// The elbow sits by the rig's hint at weight 0 and along the forearm the
	// camera sees at 1, easing between
	drawDirectRowLabel("directStage.elbowSource");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
	{
		if (arm.depthHigh <= 0.f)
			drawDirectValueCell(false, "", 0.f);
		else if (arm.elbowDirectionWeight <= 0.f)
			drawDirectTextCell(locText("directStage.elbowSourceHint"));
		else if (arm.elbowDirectionWeight >= 1.f)
			drawDirectTextCell(locText("directStage.elbowSourceLandmark"));
		else
			drawDirectValueCell(true, locText("directStage.elbowSourceEasingFmt"), 100.f * arm.elbowDirectionWeight);
	}
	ImGui::EndTable();
}

void SettingsPanels::drawDirectPalmReadout(const TrackingSectionContext& context)
{
	const DirectRetarget::SolveState* state= getShownDirectSolve(context);
	if (state == nullptr || !beginDirectTable("directPalm", true))
		return;

	// A resting arm reports no hand
	drawDirectRowLabel("directStage.handMeasured");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
	{
		if (arm.bResting)
		{
			ImGui::TableNextColumn();
			ImGui::TextDisabled("-");
		}
		else
		{
			drawDirectYesNoCell(arm.bHandMeasured);
		}
	}
	drawDirectRowLabel("directStage.forearmTwist");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
		drawDirectValueCell(!arm.bResting && arm.bHandMeasured, "%.0f", glm::degrees(arm.twist));
	ImGui::EndTable();
}

void SettingsPanels::drawDirectHoldRestReadout(const TrackingSectionContext& context)
{
	const DirectRetarget::SolveState* state= getShownDirectSolve(context);
	if (state == nullptr || !beginDirectTable("directHoldRest", true))
		return;

	drawDirectRowLabel("directStage.armState");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
		drawDirectTextCell(locText(arm.bResting ? "directStage.resting" : "directStage.live"));
	drawDirectRowLabel("directStage.intoRest");
	for (const DirectRetarget::SolveState::Arm& arm : state->arms)
	{
		ImGui::TableNextColumn();
		if (!arm.bResting)
		{
			ImGui::TextDisabled("-");
			continue;
		}
		char overlay[16];
		snprintf(overlay, sizeof(overlay), "%.0f%%", arm.restBlend * 100.f);
		ImGui::ProgressBar(arm.restBlend, ImVec2(-1.f, 0.f), overlay);
	}
	ImGui::EndTable();
}

// One wizard, in the only order that works
void SettingsPanels::drawHandCalibrationSection(const TrackingSectionContext& context)
{
	AppConfig* config= context.config;

	ImGui::SeparatorText(locText("trackingPanel.handCalibrationSection"));

	bool bChanged= false;
	ImGui::Text(locText("trackingPanel.handCalibrationStatusFmt"),
				config->handSkeleton.present[0] ? locText("common.yes") : locText("common.no"),
				config->handSkeleton.present[1] ? locText("common.yes") : locText("common.no"),
				config->fusedRestAngles.present[0] ? locText("common.yes") : locText("common.no"),
				config->fusedRestAngles.present[1] ? locText("common.yes") : locText("common.no"));

	if (ImGui::Button(locLabel("trackingPanel.calibrateHands"), ImVec2(-1, 0)))
		context.state->bLaunchHandCalibrationWizard= true;
	ImGui::SetItemTooltip("%s", locText("trackingPanel.calibrateHandsTooltip"));

	const bool bAnyHandCalibration= config->handSkeleton.present[0] ||
		config->handSkeleton.present[1] || config->fusedRestAngles.present[0] ||
		config->fusedRestAngles.present[1];
	if (bAnyHandCalibration)
	{
		if (ImGui::Button(locLabel("trackingPanel.clearHandCalibration")))
		{
			// One Clear for both: they are a matched set (the rest pose is
			// measured against the skeleton's thumb zero)
			config->handSkeleton.present[0]= false;
			config->handSkeleton.present[1]= false;
			config->fusedRestAngles.present[0]= false;
			config->fusedRestAngles.present[1]= false;
			bChanged= true;
		}
		ImGui::SetItemTooltip("%s", locText("trackingPanel.clearHandCalibrationTooltip"));
	}

	if (bChanged)
		commitTrackingConfigChange(context);
}

void SettingsPanels::drawHandOverlayControls(const TrackingSectionContext& context)
{
	VideoPreviewPanel* previewPanel= context.previewPanel;

	bool bShowOverlay= previewPanel->getShowOverlay();
	if (ImGui::Checkbox(locLabel("trackingPanel.showSkeletonOverlay"), &bShowOverlay))
		previewPanel->setShowOverlay(bShowOverlay);

	bool bShowBoxes= previewPanel->getShowDetectionBoxes();
	if (ImGui::Checkbox(locLabel("trackingPanel.showDetectionBoxes"), &bShowBoxes))
		previewPanel->setShowDetectionBoxes(bShowBoxes);

	if (context.config->direct.enabled)
	{
		struct SlotDrawModeOption
		{
			eHandSlotDrawMode mode;
			const char* key;
		};
		static const SlotDrawModeOption k_slotDrawModes[]= {
			{eHandSlotDrawMode::OnlyAccepted, "trackingPanel.handSlotDrawModeOnlyAccepted"},
			{eHandSlotDrawMode::DimRejected, "trackingPanel.handSlotDrawModeDimRejected"},
			{eHandSlotDrawMode::AllDetections, "trackingPanel.handSlotDrawModeAllDetections"},
		};
		const eHandSlotDrawMode currentMode= previewPanel->getHandSlotDrawMode();
		const char* currentKey= k_slotDrawModes[0].key;
		for (const SlotDrawModeOption& option : k_slotDrawModes)
		{
			if (option.mode == currentMode)
				currentKey= option.key;
		}
		ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.f);
		if (ImGui::BeginCombo(locLabel("trackingPanel.handSlotDrawMode"), locText(currentKey)))
		{
			for (const SlotDrawModeOption& option : k_slotDrawModes)
			{
				if (ImGui::Selectable(locText(option.key), option.mode == currentMode))
					previewPanel->setHandSlotDrawMode(option.mode);
			}
			ImGui::EndCombo();
		}
		ImGui::SetItemTooltip("%s", locText("trackingPanel.handSlotDrawModeTooltip"));
	}
}

void SettingsPanels::drawBodyOverlayControls(const TrackingSectionContext& context, int cameraIndex)
{
	VideoPreviewPanel* previewPanel= context.previewPanel;

	bool bShowBodyPose= previewPanel->getShowBodyPose();
	if (ImGui::Checkbox(locLabel("trackingPanel.showBodyLandmarks"), &bShowBodyPose))
		previewPanel->setShowBodyPose(bShowBodyPose);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.showBodyLandmarksTooltip"));

	// Only the whole-body model has face and hand blocks to draw
	if (getShownBodyPoseBackend(context, cameraIndex) == eBodyPoseBackend::Rtmw3dWholeBody)
	{
		bool bShowFace= previewPanel->getShowWholeBodyFace();
		if (ImGui::Checkbox(locLabel("trackingPanel.showWholeBodyFace"), &bShowFace))
			previewPanel->setShowWholeBodyFace(bShowFace);
		ImGui::SetItemTooltip("%s", locText("trackingPanel.showWholeBodyFaceTooltip"));

		bool bShowHands= previewPanel->getShowWholeBodyHands();
		if (ImGui::Checkbox(locLabel("trackingPanel.showWholeBodyHands"), &bShowHands))
			previewPanel->setShowWholeBodyHands(bShowHands);
		ImGui::SetItemTooltip("%s", locText("trackingPanel.showWholeBodyHandsTooltip"));
	}

	bool bShowLowVisibility= previewPanel->getShowLowVisibilityLandmarks();
	if (ImGui::Checkbox(locLabel("trackingPanel.showLowVisibilityLandmarks"), &bShowLowVisibility))
		previewPanel->setShowLowVisibilityLandmarks(bShowLowVisibility);
	ImGui::SetItemTooltip("%s", locText("trackingPanel.showLowVisibilityLandmarksTooltip"));
}

void SettingsPanels::drawPerCameraSkeletonsControl(const TrackingSectionContext& context)
{
	Scene3dPanel* scene3dPanel= context.scene3dPanel;

	if (context.config->cameraCount() > 1)
	{
		bool bShowPerCamera= scene3dPanel->getShowPerCameraSkeletons();
		if (ImGui::Checkbox(locLabel("trackingPanel.showPerCameraSkeletons"), &bShowPerCamera))
			scene3dPanel->setShowPerCameraSkeletons(bShowPerCamera);
		ImGui::SetItemTooltip("%s", locText("trackingPanel.showPerCameraSkeletonsTooltip"));
	}
}

void SettingsPanels::drawInferenceSection(const TrackingSectionContext& context)
{
	VisionThread* visionThread= context.visionThread;

	ImGui::SeparatorText(locText("trackingPanel.inferenceSection"));

	for (int cameraIndex= 0; cameraIndex < (int)context.config->cameraCount(); ++cameraIndex)
		ImGui::Text(locText("trackingPanel.cameraEpFmt"), cameraIndex + 1, visionThread->getActiveExecutionProvider(cameraIndex));
	ImGui::Text(locText("trackingPanel.inferenceMsFmt"), visionThread->getLastInferenceMs());

	// Frame-loop hitches: one thread serves every camera, so a long phase
	// starves all of them at once and reads downstream as a camera fault
	const int hitchCount= visionThread->getHitchCount();
	if (hitchCount == 0)
	{
		ImGui::TextColored(ImVec4(0.4f, 1.f, 0.5f, 1.f), "%s", locText("trackingPanel.frameLoopNoHitches"));
	}
	else
	{
		ImGui::TextColored(ImVec4(1.f, 0.85f, 0.3f, 1.f), locText("trackingPanel.frameLoopHitchesFmt"),
						   hitchCount,
						   VisionThread::getPhaseName(visionThread->getLastHitchPhase()),
						   visionThread->getLastHitchMs());
	}
	ImGui::SetItemTooltip("%s", locText("trackingPanel.frameLoopHitchesTooltip"));
}

void SettingsPanels::drawDiagnosticsSection(const TrackingSectionContext& context)
{
	TrackingPanelState& panelState= *context.state;

	ImGui::SeparatorText(locText("trackingPanel.diagnosticsSection"));

	if (ImGui::Button(locLabel("trackingPanel.dumpTrackingState")))
		panelState.bDumpRequested= true;
	ImGui::SetItemTooltip("%s", locText("trackingPanel.dumpTrackingStateTooltip"));
	ImGui::SameLine();
	ImGui::BeginDisabled(panelState.dumpCountdown.isRunning());
	if (ImGui::Button(locLabel("trackingPanel.dumpTrackingStateDelayed")))
		panelState.bDelayedDumpRequested= true;
	ImGui::EndDisabled();
	ImGui::SetItemTooltip("%s", locText("trackingPanel.dumpTrackingStateDelayedTooltip"));
	if (panelState.dumpCountdown.isRunning())
	{
		ImGui::TextColored(ImVec4(1.f, 0.85f, 0.3f, 1.f), locText("trackingPanel.dumpCountdownFmt"),
						   (int)ceilf(panelState.dumpCountdown.getRemainingSeconds()));
		ImGui::ProgressBar(panelState.dumpCountdown.getProgress(), ImVec2(-1, 4), "");
	}
	{
		const std::string lastDump= context.visionThread->getLastDumpPath();
		if (!lastDump.empty())
			ImGui::TextWrapped(locText("trackingPanel.lastDumpFmt"), lastDump.c_str());
	}
}

// -- OSC output --------------------------------------------------------------

void SettingsPanels::drawOscEnableControl(const OscSectionContext& context)
{
	OscConfig& osc= context.config->osc;
	if (ImGui::Checkbox(locLabel("oscPanel.enabled"), &osc.enabled))
		commitOscConfigChange(context);
}

void SettingsPanels::drawOscTargetControls(const OscSectionContext& context)
{
	bool bChanged= false;
	OscConfig& osc= context.config->osc;

	char ipBuffer[64];
	snprintf(ipBuffer, sizeof(ipBuffer), "%s", osc.targetIp.c_str());
	if (ImGui::InputText(locLabel("oscPanel.targetIp"), ipBuffer, sizeof(ipBuffer), ImGuiInputTextFlags_EnterReturnsTrue))
	{
		osc.targetIp= ipBuffer;
		bChanged= true;
	}

	int port= osc.port;
	if (ImGui::InputInt(locLabel("oscPanel.port"), &port, 0) && port > 0 && port <= 65535)
	{
		osc.port= port;
		bChanged= true;
	}
	ImGui::SetItemTooltip("%s", locText("oscPanel.portTooltip"));

	if (bChanged)
		commitOscConfigChange(context);
}

void SettingsPanels::drawOscDeliveryControls(const OscSectionContext& context)
{
	bool bChanged= false;
	OscConfig& osc= context.config->osc;

	bChanged|= ImGui::SliderInt(locLabel("oscPanel.maxRate"), &osc.maxRateHz, 10, 120, "%d Hz");

	bChanged|= ImGui::SliderFloat(locLabel("oscPanel.minConfidence"), &osc.minConfidence, 0.f, 1.f, "%.2f");
	ImGui::SetItemTooltip("%s", locText("oscPanel.minConfidenceTooltip"));

	if (bChanged)
		commitOscConfigChange(context);
}

void SettingsPanels::drawOscVmcControls(const OscSectionContext& context)
{
	bool bChanged= false;
	OscConfig& osc= context.config->osc;

	bChanged|= ImGui::SliderFloat(locLabel("oscPanel.headOffset"), &osc.vmcHeadOffsetMeters, 0.f, 0.25f, "%.3f m");
	ImGui::SetItemTooltip("%s", locText("oscPanel.headOffsetTooltip"));

	ImGui::TextDisabled("%s", locText("oscPanel.vmcBonesInfo"));
	ImGui::TextDisabled("%s", locText("oscPanel.vmcAvatarBoneLengths"));

	if (bChanged)
		commitOscConfigChange(context);
}

void SettingsPanels::drawOscStreamReadout(const OscSectionContext& context)
{
	const OscReadout& readout= *context.readout;
	if (!readout.bAvailable)
	{
		ImGui::TextDisabled("%s", locText("stageInspector.notRecorded"));
		return;
	}

	if (!readout.bRunning)
	{
		ImGui::TextDisabled("%s", locText("oscPanel.streamStopped"));
		return;
	}
	ImGui::TextColored(ImVec4(0.4f, 1.f, 0.5f, 1.f), locText("oscPanel.streamRunningFmt"), readout.framesPerSecond);
	ImGui::Text(locText("oscPanel.lastFrameFmt"), readout.lastPacketCount, readout.lastFrameBytes);
}

static const ImVec4 k_sourceHeldColor(1.f, 0.85f, 0.3f, 1.f);
static const ImVec4 k_sourceLostColor(1.f, 0.4f, 0.4f, 1.f);

// A group state as a word: measured plain, held and frozen amber, resting dim,
// lost red
static void drawGroupStateText(SourcePose::eGroupState state)
{
	using eGroupState= SourcePose::eGroupState;
	switch (state)
	{
	case eGroupState::Measured:
		ImGui::TextUnformatted(locText("sourceStage.stateMeasured"));
		break;
	case eGroupState::Held:
		ImGui::TextColored(k_sourceHeldColor, "%s", locText("sourceStage.stateHeld"));
		break;
	case eGroupState::Frozen:
		ImGui::TextColored(k_sourceHeldColor, "%s", locText("sourceStage.stateFrozen"));
		break;
	case eGroupState::Resting:
		ImGui::TextDisabled("%s", locText("sourceStage.stateResting"));
		break;
	case eGroupState::Lost:
		ImGui::TextColored(k_sourceLostColor, "%s", locText("sourceStage.stateLost"));
		break;
	}
}

void SettingsPanels::drawOscDeliveryReadout(const OscSectionContext& context, const TrackingFrameResult& fused,
											const SourcePose* source)
{
	const OscConfig& osc= context.config->osc;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= fused.poses[sideIndex];
		ImGui::Text("%s", locText(sideIndex == (int)eHandSide::Left ? "oscPanel.leftHand" : "oscPanel.rightHand"));

		if (source != nullptr)
		{
			// The confidence the loss stage handed on, beside the arm's state:
			// a hold decays it, and a hold or a freeze can carry one under the
			// gate
			const SourcePose::Confidence& confidence= source->hands[sideIndex].confidence;
			if (!confidence.bPresent)
				ImGui::TextDisabled("%s", locText("sourceStage.confidenceNone"));
			else if (confidence.value < osc.minConfidence)
				ImGui::TextColored(k_sourceLostColor, locText("sourceStage.confidenceBelowFmt"), confidence.value,
								   osc.minConfidence);
			else
				ImGui::Text(locText("oscPanel.confidenceFmt"), confidence.value);
			ImGui::SameLine();
			drawGroupStateText(source->arms[sideIndex].state);
			continue;
		}

		// Confidence vs the gate: red while the pose is being withheld
		const bool bGated= pose.tracked && pose.confidence < osc.minConfidence;
		if (bGated)
			ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), locText("oscPanel.confidenceWithheldFmt"),
							   pose.confidence, osc.minConfidence);
		else
			ImGui::Text(locText("oscPanel.confidenceFmt"), pose.confidence);

		if (!pose.tracked)
			ImGui::TextDisabled("%s", locText("oscPanel.notTracked"));
	}
}

// What a finger angle of zero means on a source: a carried hand's own
// convention, else the one its pipeline fills
static SourcePose::eFingerZero getSourceFingerZero(const SourcePose& source)
{
	for (const SourcePose::Hand& hand : source.hands)
	{
		if (hand.state != SourcePose::eGroupState::Lost)
			return hand.fingerZero;
	}
	return source.pipeline == SourcePose::ePipeline::Direct ? SourcePose::eFingerZero::Metacarpal
															: SourcePose::eFingerZero::CapturedRest;
}

// Appends one carried part to a list
static void appendCarriedPart(std::string& ioText, const char* partKey)
{
	if (!ioText.empty())
		ioText+= locText("sourceStage.partSeparator");
	ioText+= locText(partKey);
}

// What an arm carries, from its flags: the parts the position pipeline
// measured, the direct chain with the twist once its hand was measured, or
// how far a resting arm has eased into the rest pose. Empty while lost.
static std::string describeArmCarries(const SourcePose& source, int sideIndex)
{
	using eGroupState= SourcePose::eGroupState;
	const SourcePose::Arm& arm= source.arms[sideIndex];
	std::string text;
	if (arm.state == eGroupState::Lost)
		return text;
	if (arm.state == eGroupState::Resting)
	{
		char buffer[64];
		snprintf(buffer, sizeof(buffer), locText("sourceStage.restBlendFmt"), arm.restBlend * 100.f);
		text= buffer;
		return text;
	}
	if (source.pipeline == SourcePose::ePipeline::Direct)
	{
		appendCarriedPart(text, "sourceStage.partChain");
		// The twist is zero while the hand was never measured, which leaves
		// the hand lost
		if (source.hands[sideIndex].state != eGroupState::Lost)
			appendCarriedPart(text, "sourceStage.partTwist");
		return text;
	}
	if (arm.bHasShoulder)
		appendCarriedPart(text, "sourceStage.partShoulder");
	if (arm.bHasElbow)
		appendCarriedPart(text, "sourceStage.partElbow");
	appendCarriedPart(text, "sourceStage.partWrist");
	if (arm.bHasForearmOrientation)
		appendCarriedPart(text, "sourceStage.partForearm");
	return text;
}

// One group's row: its state, its confidence (a dim dash where the pipeline
// has none) and what it carries (a dim dash for nothing)
static void drawSourceGroupRow(const char* labelKey, SourcePose::eGroupState state,
							   const SourcePose::Confidence* confidence, const std::string& carries)
{
	ImGui::TableNextRow();
	ImGui::TableNextColumn();
	ImGui::TextUnformatted(locText(labelKey));
	ImGui::TableNextColumn();
	drawGroupStateText(state);
	ImGui::TableNextColumn();
	if (confidence != nullptr && confidence->bPresent)
		ImGui::Text("%.2f", confidence->value);
	else
		ImGui::TextDisabled("-");
	ImGui::TableNextColumn();
	if (carries.empty())
		ImGui::TextDisabled("-");
	else
		ImGui::TextUnformatted(carries.c_str());
}

static void drawSourceSummary(const SourcePose& source)
{
	if (!beginDirectTable("sourceSummary", false))
		return;

	drawDirectRowLabel("stageInspector.summaryPipeline");
	drawDirectTextCell(locText(source.pipeline == SourcePose::ePipeline::Direct ? "sourceStage.pipelineDirect"
																				: "sourceStage.pipelinePosition"));
	drawDirectRowLabel("sourceStage.fingerZero");
	drawDirectTextCell(locText(getSourceFingerZero(source) == SourcePose::eFingerZero::Metacarpal
								   ? "sourceStage.fingerZeroMetacarpal"
								   : "sourceStage.fingerZeroCapturedRest"));
	drawDirectRowLabel("sourceStage.frameInterval");
	drawDirectValueCell(true, "%.1f", (float)source.clock.frameIntervalMs);
	drawDirectRowLabel("sourceStage.resetEpoch");
	ImGui::TableNextColumn();
	ImGui::Text("%u", source.clock.resetEpoch);
	ImGui::EndTable();
}

static void drawSourceGroupsTable(const SourcePose& source)
{
	using eGroupState= SourcePose::eGroupState;
	if (!ImGui::BeginTable("sourceGroups", 4, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
		return;
	ImGui::TableSetupColumn("");
	ImGui::TableSetupColumn(locText("stageInspector.stateLabel"));
	ImGui::TableSetupColumn(locText("sourceStage.confidenceColumn"));
	ImGui::TableSetupColumn(locText("sourceStage.carriesColumn"));
	ImGui::TableHeadersRow();

	// A resting torso or head carries how far it has eased to the identity
	auto describeRest= [](float restBlend) {
		char buffer[64];
		snprintf(buffer, sizeof(buffer), locText("sourceStage.restBlendFmt"), restBlend * 100.f);
		return std::string(buffer);
	};

	std::string carries;
	// The torso is the spine's rotation on the direct pipeline and the line
	// between the shoulders on the position one, with no confidence of its own
	if (source.torso.state == eGroupState::Resting)
		carries= describeRest(source.torso.restBlend);
	else if (source.torso.state != eGroupState::Lost)
		carries= locText(source.pipeline == SourcePose::ePipeline::Direct ? "sourceStage.partSpine"
																		  : "sourceStage.partShoulderLine");
	drawSourceGroupRow("sourceStage.torso", source.torso.state, nullptr, carries);

	carries.clear();
	if (source.head.state == eGroupState::Resting)
		carries= describeRest(source.head.restBlend);
	else if (source.head.state != eGroupState::Lost)
		carries= locText("sourceStage.partOrientation");
	drawSourceGroupRow("directStage.head", source.head.state, &source.head.confidence, carries);

	// An arm's confidence is its forearm's
	static const char* s_armKeys[2]= {"avatarPanel.leftArmHeader", "avatarPanel.rightArmHeader"};
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SourcePose::Arm& arm= source.arms[sideIndex];
		drawSourceGroupRow(s_armKeys[sideIndex], arm.state, &arm.forearmConfidence,
						   describeArmCarries(source, sideIndex));
	}

	static const char* s_handKeys[2]= {"avatarPanel.leftHandHeader", "avatarPanel.rightHandHeader"};
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SourcePose::Hand& hand= source.hands[sideIndex];
		carries.clear();
		if (hand.state != eGroupState::Lost)
		{
			appendCarriedPart(carries, "sourceStage.partPalm");
			appendCarriedPart(carries, "sourceStage.partFingers");
		}
		drawSourceGroupRow(s_handKeys[sideIndex], hand.state, &hand.confidence, carries);
	}
	ImGui::EndTable();
}

// Each hand's finger angles as the target solve takes them, in degrees, so a
// held or frozen hand shows the angles it carries
static void drawSourceFingerAngles(const SourcePose& source)
{
	ImGui::TextDisabled("%s", locText("oscPanel.readoutInfo"));

	static const char* s_fingerNames[FINGER_COUNT]= {
		"oscPanel.fingerThumb", "oscPanel.fingerIndex", "oscPanel.fingerMiddle",
		"oscPanel.fingerRing", "oscPanel.fingerPinky"};
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SourcePose::Hand& hand= source.hands[sideIndex];
		ImGui::Text("%s", locText(sideIndex == (int)eHandSide::Left ? "oscPanel.leftHand" : "oscPanel.rightHand"));
		if (hand.state == SourcePose::eGroupState::Lost)
		{
			ImGui::TextDisabled("%s", locText("oscPanel.notTracked"));
			continue;
		}

		ImGui::PushID(sideIndex);
		if (ImGui::BeginTable("angles", 5,
							  ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
		{
			ImGui::TableSetupColumn(locText("oscPanel.fingerColumn"));
			ImGui::TableSetupColumn(locText("oscPanel.latColumn"));
			ImGui::TableSetupColumn(locText("oscPanel.proxColumn"));
			ImGui::TableSetupColumn(locText("oscPanel.interColumn"));
			ImGui::TableSetupColumn(locText("oscPanel.distColumn"));
			ImGui::TableHeadersRow();

			constexpr float kRadToDeg= 57.29578f;
			for (int finger= 0; finger < FINGER_COUNT; ++finger)
			{
				const FingerAngles& angles= hand.fingers[finger];
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::TextUnformatted(locText(s_fingerNames[finger]));
				ImGui::TableNextColumn();
				ImGui::Text("%+.0f", angles.lateral * kRadToDeg);
				ImGui::TableNextColumn();
				ImGui::Text("%+.0f", angles.proximal * kRadToDeg);
				ImGui::TableNextColumn();
				ImGui::Text("%+.0f", angles.intermediate * kRadToDeg);
				ImGui::TableNextColumn();
				ImGui::Text("%+.0f", angles.distal * kRadToDeg);
			}
			ImGui::EndTable();
		}
		ImGui::PopID();
	}
}

void SettingsPanels::drawSourcePoseReadout(const TrackingSectionContext& context)
{
	const SourcePose* source= context.sourcePose != nullptr ? context.sourcePose->source : nullptr;
	if (source == nullptr)
	{
		ImGui::TextDisabled("%s", locText("sourceStage.noSource"));
		return;
	}
	drawSourceSummary(*source);
	ImGui::SeparatorText(locText("sourceStage.groupsSection"));
	drawSourceGroupsTable(*source);
	ImGui::SeparatorText(locText("sourceStage.fingersSection"));
	drawSourceFingerAngles(*source);
}

void SettingsPanels::drawRetargetChainReadout(const TrackingSectionContext& context, bool bAvatar)
{
	const SourcePose* source= context.sourcePose != nullptr ? context.sourcePose->source : nullptr;
	if (source == nullptr)
	{
		ImGui::TextDisabled("%s", locText("sourceStage.noSource"));
		return;
	}
	if (!ImGui::BeginTable("retargetChains", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
		return;
	ImGui::TableSetupColumn(locText("chains.chainColumn"));
	ImGui::TableSetupColumn(locText("stageInspector.stateLabel"));
	ImGui::TableSetupColumn(locText("chains.ikWeightLabel"));
	ImGui::TableHeadersRow();

	// A chain's row: its state, and for an arm the IK weight the solve onto
	// the avatar ran it at on the shown frame
	const std::array<RetargetArmReadout, 2>* arms= context.sourcePose->armParameters;
	auto drawRow= [&](const char* labelKey, SourcePose::eGroupState state, int armIndex) {
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		ImGui::TextUnformatted(locText(labelKey));
		ImGui::TableNextColumn();
		drawGroupStateText(state);
		ImGui::TableNextColumn();
		if (armIndex >= 0 && bAvatar && arms != nullptr && (*arms)[armIndex].bPosed)
			ImGui::Text("%.2f", (*arms)[armIndex].effectiveIkWeight);
		else
			ImGui::TextDisabled("-");
		if (armIndex >= 0)
			ImGui::SetItemTooltip("%s", locText("chains.effectiveIkWeightTooltip"));
	};
	drawRow("sourceStage.torso", source->torso.state, -1);
	drawRow("directStage.head", source->head.state, -1);
	drawRow("avatarPanel.leftArmHeader", source->arms[0].state, 0);
	drawRow("avatarPanel.rightArmHeader", source->arms[1].state, 1);
	drawRow("avatarPanel.leftHandHeader", source->hands[0].state, -1);
	drawRow("avatarPanel.rightHandHeader", source->hands[1].state, -1);
	ImGui::EndTable();
}

void SettingsPanels::drawLanguageCombo()
{
	LocalizationManager* localization= LocalizationManager::getInstance();
	if (localization == nullptr)
		return;

	const std::vector<LocalizationManager::LanguageInfo> languages=
		localization->getSupportedLanguages();

	const char* currentNativeName= localization->getLanguage().c_str();
	for (const LocalizationManager::LanguageInfo& info : languages)
	{
		if (info.code == localization->getLanguage())
			currentNativeName= info.nativeName.c_str();
	}

	if (ImGui::BeginCombo(locLabel("settingsPanel.language"), currentNativeName))
	{
		for (const LocalizationManager::LanguageInfo& info : languages)
		{
			ImGui::PushID(info.code.c_str());
			if (ImGui::Selectable(info.nativeName.c_str(), info.code == localization->getLanguage()))
				localization->setLanguage(info.code);
			ImGui::PopID();
		}
		ImGui::EndCombo();
	}
}

void SettingsPanels::drawAppSettingsPanel(bool* pOpen)
{
	if (!ImGui::Begin(locWindowTitle("windows.settings"), pOpen))
	{
		ImGui::End();
		return;
	}

	drawLanguageCombo();

	ImGui::End();
}
