#include "WebcamBlink.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "glm/ext/vector_float2.hpp"
#include "glm/geometric.hpp"

#include "WebcamFace.h"

namespace
{
// The open frames the baseline is the trimmed mean of, and how many it needs
// before it stands on them rather than on its seed
constexpr double k_baselineWindowMs= 2500.0;
constexpr int k_baselineMinSamples= 8;
// The share of the window's samples dropped from each end before the mean
constexpr float k_baselineTrim= 0.3f;

// The closed level as a fraction of the open level, before any closure has
// measured it. A shut eye's openness is about two thirds of the open eye's
// on the whole-body points.
constexpr float k_seedFloorFraction= 0.65f;
constexpr float k_minFloorFraction= 0.2f;
constexpr float k_maxFloorFraction= 0.85f;
// How far each closure that reaches the floor band moves the floor toward
// its own lowest level
constexpr float k_floorAdaptRate= 0.3f;

// A closure starts on a drop of this fraction of the baseline within the
// drop window, landing at least as far under the baseline. A slow sag never
// drops that far within the window, so it moves the baseline instead.
constexpr double k_dropWindowMs= 200.0;
constexpr float k_closeDropFraction= 0.17f;
// The closed band: within this share of the travel over the floor
constexpr float k_closedBandFraction= 0.3f;
// A closure ends when the eye returns within this share of the travel under
// the baseline
constexpr float k_releaseFraction= 0.3f;
// A closure that sits between the closed band and the baseline this long is
// lowered lids, not a blink: it ends and the baseline restarts at the level
constexpr double k_longClosureMs= 600.0;

// The nose tip's offset from the eyes' midpoint along the eye line, over the
// face width, past which the head is turned too far for the baseline to
// learn from the frame
constexpr float k_maxYawOffset= 0.15f;
constexpr int k_noseTip= 30;
constexpr int k_rightEyeOuter= 36;
constexpr int k_leftEyeOuter= 45;

// A face narrower than the full width, in px of outer eye corner span, lets
// winks through less, and one at the none width links the eyes fully: an eye
// that small is a few px tall, and one eye's drop there is more often noise
constexpr float k_winkFaceFullPx= 90.f;
constexpr float k_winkFaceNonePx= 50.f;

float yawOffset(const WholeBodyExtras& extras, float faceWidthPx)
{
	const glm::vec2 right(extras.face[k_rightEyeOuter]);
	const glm::vec2 left(extras.face[k_leftEyeOuter]);
	const glm::vec2 nose(extras.face[k_noseTip]);
	const glm::vec2 leftAxis= (left - right) / faceWidthPx;
	return glm::dot(nose - (left + right) * 0.5f, leftAxis) / faceWidthPx;
}
} // namespace

float shapeBlinkClosure(float closure, int clarity)
{
	const float x= std::clamp(closure, 0.f, 1.f);
	const float exponent=
		1.5f * (float)std::clamp(clarity, BlinkConfig::k_minClarity, BlinkConfig::k_maxClarity);
	if (x < 0.5f)
		return 0.5f * std::pow(2.f * x, exponent);
	return 0.5f * std::pow(2.f * (x - 0.5f), 1.f / exponent) + 0.5f;
}

void linkBlinkEyes(float closures[2], float winkWeight, const BlinkConfig& config)
{
	if (config.syncEyes)
	{
		const float mean= 0.5f * (closures[0] + closures[1]);
		closures[0]= mean;
		closures[1]= mean;
		return;
	}
	const float threshold= std::max(config.winkThreshold, BlinkConfig::k_minWinkThreshold);
	const float own=
		std::min(std::fabs(closures[0] - closures[1]) / threshold, 1.f) * std::clamp(winkWeight, 0.f, 1.f);
	const float lesser= std::min(closures[0], closures[1]);
	for (int side= 0; side < 2; ++side)
		closures[side]= closures[side] * own + lesser * (1.f - own);
}

void WebcamBlinkFilter::reset(const WholeBodyFaceRatios* neutral)
{
	const WholeBodyFaceRatios seed= neutral != nullptr && neutral->valid ? *neutral : makeDefaultWebcamFaceNeutral();
	for (int side= 0; side < 2; ++side)
	{
		m_seedOpenness[side]= seed.eyeOpenness[side];
		m_eyes[side]= Eye();
		m_eyes[side].baseline= seed.eyeOpenness[side];
		m_eyes[side].floorFraction= k_seedFloorFraction;
	}
	m_lastModelFrameIndex= -1;
	m_bHasLast= false;
	m_last= FaceSources::Blink();
	m_bSeeded= true;
}

bool WebcamBlinkFilter::isSeededFrom(const WholeBodyFaceRatios* neutral) const
{
	const WholeBodyFaceRatios seed= neutral != nullptr && neutral->valid ? *neutral : makeDefaultWebcamFaceNeutral();
	return m_bSeeded && m_seedOpenness[0] == seed.eyeOpenness[0] && m_seedOpenness[1] == seed.eyeOpenness[1];
}

void WebcamBlinkFilter::update(const BodyPoseObservation& body, bool bFresh, double timestampMs,
							   const BlinkConfig& config, FaceSources::Blink& outBlink)
{
	if (!m_bSeeded)
		reset(nullptr);

	outBlink= FaceSources::Blink();
	if (!bFresh || !body.valid || !body.wholeBody.present)
		return;
	if (m_bHasLast && body.modelFrameIndex == m_lastModelFrameIndex)
	{
		outBlink= m_last;
		return;
	}
	m_lastModelFrameIndex= body.modelFrameIndex;
	m_bHasLast= true;
	outBlink.present= true;

	const WholeBodyFaceRatios ratios= computeWholeBodyFaceRatios(body.wholeBody);
	if (!ratios.valid || ratios.minScore < k_webcamFaceScoreGate)
	{
		// An unseen eye ends any closure. The baseline keeps what it learned.
		for (Eye& eye : m_eyes)
		{
			eye.state= eEyeState::Open;
			eye.recentSamples.clear();
			eye.closure= 0.f;
		}
		m_last= outBlink;
		return;
	}
	outBlink.tracked= true;

	const bool bOnAngle= std::fabs(yawOffset(body.wholeBody, ratios.faceWidthPx)) <= k_maxYawOffset;
	float closures[2];
	bool bSettled= true;
	for (int side= 0; side < 2; ++side)
	{
		Eye& eye= m_eyes[side];
		stepEye(eye, ratios.eyeOpenness[side], timestampMs, bOnAngle, config);
		closures[side]= eye.state == eEyeState::Open ? 0.f : shapeBlinkClosure(eye.closure, config.clarity);
		bSettled= bSettled && (int)eye.baselineSamples.size() >= k_baselineMinSamples;

		outBlink.openness[side]= eye.openness;
		outBlink.baseline[side]= eye.baseline;
		outBlink.floor[side]= eye.baseline * eye.floorFraction;
		outBlink.state[side]= (uint8_t)eye.state;
		outBlink.closure[side]= closures[side];
	}
	outBlink.bBaselineSettled= bSettled;

	const float winkWeight= (ratios.faceWidthPx - k_winkFaceNonePx) / (k_winkFaceFullPx - k_winkFaceNonePx);
	linkBlinkEyes(closures, winkWeight, config);
	outBlink.eyeBlinkLeft= closures[k_ratioSideOfArkitLeft];
	outBlink.eyeBlinkRight= closures[k_ratioSideOfArkitRight];
	m_last= outBlink;
}

void WebcamBlinkFilter::stepEye(Eye& eye, float openness, double timestampMs, bool bOnAngle,
								const BlinkConfig& config)
{
	eye.openness= openness;
	eye.recentSamples.push_back({timestampMs, openness});
	while (timestampMs - eye.recentSamples.front().timestampMs > k_dropWindowMs)
		eye.recentSamples.pop_front();

	const float sensitivity= std::clamp(config.sensitivity, BlinkConfig::k_minSensitivity, BlinkConfig::k_maxSensitivity);
	const float dropFraction= k_closeDropFraction / sensitivity;
	const float floorLevel= eye.baseline * eye.floorFraction;
	const float travel= std::max(eye.baseline - floorLevel, 1e-4f);

	if (eye.state == eEyeState::Open)
	{
		float recentMax= openness;
		for (const Sample& sample : eye.recentSamples)
			recentMax= std::max(recentMax, sample.openness);
		const float threshold= dropFraction * eye.baseline;
		if (eye.baseline > 1e-4f && recentMax - openness >= threshold && openness <= eye.baseline - threshold)
		{
			eye.state= eEyeState::Closing;
			eye.closureStartMs= timestampMs;
			eye.closureMinimum= openness;
		}
		else
		{
			if (bOnAngle)
				eye.baselineSamples.push_back({timestampMs, openness});
			updateBaseline(eye, timestampMs);
			eye.closure= 0.f;
			return;
		}
	}

	eye.closureMinimum= std::min(eye.closureMinimum, openness);
	if (openness >= eye.baseline - k_releaseFraction * travel)
	{
		endClosure(eye);
		return;
	}
	const bool bInClosedBand= openness <= floorLevel + k_closedBandFraction * travel;
	if (!bInClosedBand && timestampMs - eye.closureStartMs > k_longClosureMs)
	{
		// Lowered lids: the level is the new open level
		eye.state= eEyeState::Open;
		eye.closure= 0.f;
		eye.baselineSamples.clear();
		eye.baselineSamples.push_back({timestampMs, openness});
		eye.baseline= openness;
		return;
	}
	eye.state= bInClosedBand ? eEyeState::Closed : eEyeState::Closing;
	eye.closure= std::clamp((eye.baseline - openness) / travel, 0.f, 1.f);
}

void WebcamBlinkFilter::updateBaseline(Eye& eye, double timestampMs)
{
	while (!eye.baselineSamples.empty() && timestampMs - eye.baselineSamples.front().timestampMs > k_baselineWindowMs)
		eye.baselineSamples.pop_front();
	const int count= (int)eye.baselineSamples.size();
	if (count < k_baselineMinSamples)
		return;

	std::vector<float> levels;
	levels.reserve(count);
	for (const Sample& sample : eye.baselineSamples)
		levels.push_back(sample.openness);
	std::sort(levels.begin(), levels.end());
	const int trim= (int)((float)count * k_baselineTrim);
	float sum= 0.f;
	for (int index= trim; index < count - trim; ++index)
		sum+= levels[index];
	eye.baseline= sum / (float)(count - 2 * trim);
}

void WebcamBlinkFilter::endClosure(Eye& eye)
{
	// A closure that reached the closed band measured the floor
	const float floorLevel= eye.baseline * eye.floorFraction;
	const float travel= eye.baseline - floorLevel;
	if (eye.baseline > 1e-4f && eye.closureMinimum <= floorLevel + k_closedBandFraction * travel)
	{
		const float measured= eye.closureMinimum / eye.baseline;
		eye.floorFraction= std::clamp(eye.floorFraction + k_floorAdaptRate * (measured - eye.floorFraction),
									  k_minFloorFraction, k_maxFloorFraction);
	}
	eye.state= eEyeState::Open;
	eye.closure= 0.f;
}
