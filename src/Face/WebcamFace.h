#pragma once

#include <cstdint>
#include <deque>

#include "BodyPoseTypes.h"
#include "FaceSources.h"
#include "WholeBodyFaceRatios.h"

// The webcam face source: the whole-body model's face block on the direct
// camera, as ARKit columns beside the phone's. Each column is one face shape
// ratio mapped from the user's neutral value to a full value, raw: no dead
// zone and no smoothing, as the phone's columns have none. A pure function of
// one observation and the neutral, so a replay evaluates it from the record.

// The face block's lowest score among the points the ratios read, at or over
// which the source is tracked
constexpr float k_webcamFaceScoreGate= 0.5f;
// The direct camera's last observation is fresh while it is no older
constexpr double k_webcamFaceFreshMs= 500.0;

// The neutral ratios of a face that was never captured
WholeBodyFaceRatios makeDefaultWebcamFaceNeutral();

// A bit per ARKit column the ratios measure. A column outside it reads zero.
uint64_t webcamFaceMeasuredMask();
inline bool isWebcamFaceColumnMeasured(int arkitIndex)
{
	return (webcamFaceMeasuredMask() >> arkitIndex) & 1ull;
}

// Fills the source from the direct camera's observation. bFresh is whether
// that observation is recent enough to stand for the face now. neutral is the
// captured neutral, or nullptr for the defaults.
void fillWebcamFaceSource(const BodyPoseObservation& body, bool bFresh, const WholeBodyFaceRatios* neutral,
						  FaceSources::Webcam& outWebcam);

// The neutral capture's face ratios: the mean over the last model frames
// whose face cleared the gate, since one frame's eye openness carries a
// quarter of a blink's range as noise
class WebcamFaceNeutralAverager
{
public:
	static constexpr int k_maxFrames= 15;
	static constexpr int k_minFrames= 5;
	static constexpr double k_windowMs= 1500.0;

	// Takes the observation once per model frame
	void add(const BodyPoseObservation& body, double timestampMs);
	// False with fewer than k_minFrames inside the window ending at nowMs
	bool average(double nowMs, WholeBodyFaceRatios& outNeutral) const;
	void clear();

private:
	struct Entry
	{
		double timestampMs= 0.0;
		WholeBodyFaceRatios ratios;
	};
	std::deque<Entry> m_entries;
	int64_t m_lastModelFrameIndex= -1;
};
