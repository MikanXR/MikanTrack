#pragma once

#include <cstdint>
#include <deque>

#include "BodyPoseTypes.h"
#include "FaceSources.h"
#include "WholeBodyFaceRatios.h"

// The Blink stage's user settings, saved in the project's blink section
struct BlinkConfig
{
	// Both eyes read their mean, so no wink passes
	bool syncEyes= false;
	// The difference between the eyes at which each eye reads on its own.
	// Under it both eyes are pulled toward the less closed one.
	float winkThreshold= 0.4f;
	// The S-curve's steepness, 1 to 3: a higher clarity pushes a partial
	// closure toward open or shut
	int clarity= 1;
	// Scales the drop a closure must make to start, 0.5 to 2: a higher
	// sensitivity starts a closure on a smaller, slower drop
	float sensitivity= 1.f;

	static constexpr float k_minWinkThreshold= 0.1f;
	static constexpr float k_maxWinkThreshold= 1.f;
	static constexpr int k_minClarity= 1;
	static constexpr int k_maxClarity= 3;
	static constexpr float k_minSensitivity= 0.5f;
	static constexpr float k_maxSensitivity= 2.f;

	bool operator==(const BlinkConfig& other) const
	{
		return syncEyes == other.syncEyes && winkThreshold == other.winkThreshold && clarity == other.clarity &&
			sensitivity == other.sensitivity;
	}
	bool operator!=(const BlinkConfig& other) const { return !(*this == other); }
};

// The Blink stage: the webcam's eye openness turned into blinks over time.
// The openness reads as low with the lids lowered or the gaze dropped as with
// the eyes shut, so one frame cannot tell them apart. Their timing can: a
// blink is a fast drop and a return, a lowered gaze a slow sag. Each eye keeps
// an open baseline that follows slow changes, and its output rises only once
// a fast drop under that baseline has started a closure. A shallow drop must
// hold for a second model frame before it reads, since point jitter makes
// drops as fast as a blink's and seldom two in a row.
//
// An eye held shut does not stay at the floor: its lid points drift back up
// within a few frames. So a wink is held by the difference between the eyes
// instead. Once one eye's closure has started with the other eye open, it
// reads fully closed for as long as that eye reads more closed than the
// other, with the head's turn taken out of the comparison.
//
// Stateful, and deterministic in the observations' own timestamps, so a
// replay that feeds the same frames in order reads the same values. It steps
// once per model frame. An output frame that repeats a model frame's
// observation re-emits that frame's values.
class WebcamBlinkFilter
{
public:
	enum class eEyeState : uint8_t
	{
		Open,
		// A closure has started and the eye is not yet near its closed floor
		Closing,
		// Near the closed floor, or a wink held by the eyes' difference
		Closed,
	};

	// How much more closed than the other eye an eye must read for its
	// closure to be held as a wink, and the difference under which a held
	// wink releases. The log of the eyes' openness ratio, against its own
	// slow reference.
	static constexpr float k_winkHoldOnDifference= 0.12f;
	static constexpr float k_winkHoldOffDifference= 0.06f;

	// Forgets every closure and reseeds the baseline from the neutral, or
	// the defaults when neutral is nullptr or not valid
	void reset(const WholeBodyFaceRatios* neutral);
	// True once reset with this neutral's eye openness (nullptr for the
	// defaults), so a caller can reseed when the neutral changes
	bool isSeededFrom(const WholeBodyFaceRatios* neutral) const;

	// Steps on a new model frame of the front camera's observation and
	// fills the source. bFresh and the gates are the webcam source's.
	// timestampMs is the observation's own time.
	void update(const BodyPoseObservation& body, bool bFresh, double timestampMs, const BlinkConfig& config,
				FaceSources::Blink& outBlink);

private:
	// A timed value: an eye's openness, or the eyes' ratio
	struct Sample
	{
		double timestampMs= 0.0;
		float value= 0.f;
	};
	struct Eye
	{
		eEyeState state= eEyeState::Open;
		// The open level and the closed level as a fraction of it
		float baseline= 0.f;
		float floorFraction= 0.f;
		// The open frames the baseline is the trimmed mean of
		std::deque<Sample> baselineSamples;
		// The last frames, for the drop a closure starts on
		std::deque<Sample> recentSamples;
		double closureStartMs= 0.0;
		// When the last closure that ran its course ended
		double closureEndMs= -1e18;
		float closureMinimum= 0.f;
		float openness= 0.f;
		float closure= 0.f;
		// The closure waits for a second frame under the start level, and
		// reads zero until it has one
		bool bPending= false;
		// The closure is a wink held by the eyes' difference
		bool bWinkHeld= false;
		// How much more closed than the other eye this one reads
		float difference= 0.f;
	};

	// Latches and releases each eye's held wink from the eyes' difference on
	// this frame and the states the last frame left
	void updateWinkHold(const float openness[2], float yawOffset, double timestampMs, bool bOnAngle);
	void stepEye(Eye& eye, float openness, double timestampMs, bool bOnAngle, const BlinkConfig& config);
	// Feeds the baseline an open frame
	static void feedBaseline(Eye& eye, float openness, double timestampMs, bool bOnAngle);
	static void updateBaseline(Eye& eye, double timestampMs);
	void endClosure(Eye& eye, double timestampMs);

	Eye m_eyes[2];
	// The eyes' log openness ratio, left over right with the head's turn
	// taken out: the quiet frames its reference is the median of, the
	// reference, and the last frames' ratio against it
	std::deque<Sample> m_ratioReferenceSamples;
	float m_ratioReference= 0.f;
	std::deque<Sample> m_ratioSamples;
	float m_seedOpenness[2]= {0.f, 0.f};
	int64_t m_lastModelFrameIndex= -1;
	bool m_bHasLast= false;
	FaceSources::Blink m_last;
	bool m_bSeeded= false;
};

// The S-curve a closure passes through, symmetric about one half, with the
// exponent 1.5 times the clarity
float shapeBlinkClosure(float closure, int clarity);

// Pulls two closures toward the less closed one unless they differ by the
// wink threshold, scaled down by winkWeight (0 links fully). With syncEyes
// both read their mean.
void linkBlinkEyes(float closures[2], float winkWeight, const BlinkConfig& config);
