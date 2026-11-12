#include "WebcamFace.h"

#include <algorithm>

namespace
{
// The ratio's travel from the neutral value to a full shape. The eye's is a
// fraction of its own neutral openness, the rest are face widths.
constexpr float k_blinkFullFraction= 0.4f;
constexpr float k_jawOpenFull= 0.2f;
constexpr float k_smileFull= 0.18f;
constexpr float k_puckerFull= 0.14f;
// An opening jaw draws the mouth corners in: the width it costs, per face
// width of lip gap, which is given back before the width is read
constexpr float k_jawWidthLoss= 0.35f;
constexpr float k_browInnerFull= 0.1f;
constexpr float k_browOuterFull= 0.12f;

// The ratios' side that feeds each ARKit side. The ratios' sides are the
// person's own (0 left, 1 right). The phone's Left columns followed the
// person's right eye on the one take that carries a wink, so the columns
// cross the ratios' sides to read as the phone's do.
constexpr int k_ratioSideOfArkitLeft= 1;
constexpr int k_ratioSideOfArkitRight= 0;

float unit(float value)
{
	return std::clamp(value, 0.f, 1.f);
}

int column(const char* name)
{
	return arkitBlendshapeFromName(name);
}
} // namespace

WholeBodyFaceRatios makeDefaultWebcamFaceNeutral()
{
	WholeBodyFaceRatios neutral;
	neutral.valid= true;
	neutral.eyeOpenness[0]= 0.3f;
	neutral.eyeOpenness[1]= 0.3f;
	neutral.mouthOpen= 0.f;
	neutral.mouthWidth= 0.56f;
	neutral.mouthCornerLift[0]= 0.04f;
	neutral.mouthCornerLift[1]= 0.04f;
	neutral.browHeight[0]= 0.215f;
	neutral.browHeight[1]= 0.215f;
	neutral.browInnerHeight= 0.195f;
	return neutral;
}

uint64_t webcamFaceMeasuredMask()
{
	static const uint64_t s_mask= [] {
		uint64_t mask= 0;
		for (const char* name : {"eyeBlinkLeft", "eyeBlinkRight", "jawOpen", "mouthSmileLeft", "mouthSmileRight",
								 "mouthPucker", "browInnerUp", "browOuterUpLeft", "browOuterUpRight"})
			mask|= 1ull << column(name);
		return mask;
	}();
	return s_mask;
}

void fillWebcamFaceSource(const BodyPoseObservation& body, bool bFresh, const WholeBodyFaceRatios* neutral,
						  FaceSources::Webcam& outWebcam)
{
	outWebcam= FaceSources::Webcam();
	outWebcam.measuredMask= webcamFaceMeasuredMask();
	if (!bFresh || !body.valid || !body.wholeBody.present)
		return;
	outWebcam.present= true;

	const WholeBodyFaceRatios ratios= computeWholeBodyFaceRatios(body.wholeBody);
	if (!ratios.valid || ratios.minScore < k_webcamFaceScoreGate)
		return;
	outWebcam.tracked= true;

	const WholeBodyFaceRatios defaults= makeDefaultWebcamFaceNeutral();
	const WholeBodyFaceRatios& rest= neutral != nullptr && neutral->valid ? *neutral : defaults;
	auto set= [&](const char* name, float value) { outWebcam.blendshapes[(size_t)column(name)]= unit(value); };
	auto blink= [&](int side) {
		const float travel= k_blinkFullFraction * rest.eyeOpenness[side];
		return travel > 1e-4f ? (rest.eyeOpenness[side] - ratios.eyeOpenness[side]) / travel : 0.f;
	};

	const int left= k_ratioSideOfArkitLeft;
	const int right= k_ratioSideOfArkitRight;
	set("eyeBlinkLeft", blink(left));
	set("eyeBlinkRight", blink(right));
	set("jawOpen", (ratios.mouthOpen - rest.mouthOpen) / k_jawOpenFull);
	// The smile and the pucker are the two directions of the mouth's width
	// about its neutral. The width follows a smile closely where the corner
	// lift does not, and it is one number, so the two smile columns read
	// alike.
	const float jawGap= std::max(ratios.mouthOpen - rest.mouthOpen, 0.f);
	const float width= ratios.mouthWidth + k_jawWidthLoss * jawGap;
	set("mouthSmileLeft", (width - rest.mouthWidth) / k_smileFull);
	set("mouthSmileRight", (width - rest.mouthWidth) / k_smileFull);
	set("mouthPucker", (rest.mouthWidth - width) / k_puckerFull);
	set("browInnerUp", (ratios.browInnerHeight - rest.browInnerHeight) / k_browInnerFull);
	set("browOuterUpLeft", (ratios.browHeight[left] - rest.browHeight[left]) / k_browOuterFull);
	set("browOuterUpRight", (ratios.browHeight[right] - rest.browHeight[right]) / k_browOuterFull);
}

void WebcamFaceNeutralAverager::add(const BodyPoseObservation& body, double timestampMs)
{
	if (!body.valid || body.modelFrameIndex == m_lastModelFrameIndex)
		return;
	m_lastModelFrameIndex= body.modelFrameIndex;
	const WholeBodyFaceRatios ratios= computeWholeBodyFaceRatios(body.wholeBody);
	if (!ratios.valid || ratios.minScore < k_webcamFaceScoreGate)
		return;
	m_entries.push_back({timestampMs, ratios});
	while ((int)m_entries.size() > k_maxFrames)
		m_entries.pop_front();
}

bool WebcamFaceNeutralAverager::average(double nowMs, WholeBodyFaceRatios& outNeutral) const
{
	WholeBodyFaceRatios sum;
	int count= 0;
	for (const Entry& entry : m_entries)
	{
		if (nowMs - entry.timestampMs > k_windowMs)
			continue;
		count++;
		sum.mouthOpen+= entry.ratios.mouthOpen;
		sum.mouthWidth+= entry.ratios.mouthWidth;
		sum.browInnerHeight+= entry.ratios.browInnerHeight;
		for (int side= 0; side < 2; ++side)
		{
			sum.eyeOpenness[side]+= entry.ratios.eyeOpenness[side];
			sum.mouthCornerLift[side]+= entry.ratios.mouthCornerLift[side];
			sum.browHeight[side]+= entry.ratios.browHeight[side];
		}
	}
	if (count < k_minFrames)
		return false;

	const float scale= 1.f / (float)count;
	outNeutral= WholeBodyFaceRatios();
	outNeutral.valid= true;
	outNeutral.mouthOpen= sum.mouthOpen * scale;
	outNeutral.mouthWidth= sum.mouthWidth * scale;
	outNeutral.browInnerHeight= sum.browInnerHeight * scale;
	for (int side= 0; side < 2; ++side)
	{
		outNeutral.eyeOpenness[side]= sum.eyeOpenness[side] * scale;
		outNeutral.mouthCornerLift[side]= sum.mouthCornerLift[side] * scale;
		outNeutral.browHeight[side]= sum.browHeight[side] * scale;
	}
	return true;
}

void WebcamFaceNeutralAverager::clear()
{
	m_entries.clear();
	m_lastModelFrameIndex= -1;
}
