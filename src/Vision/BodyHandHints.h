#pragma once

#include <vector>

#include "BodyPoseTypes.h"
#include "HandTrackingPipeline.h"

// A whole-body hand seeds a hint only when its wrist and its index, middle
// and pinky knuckles all score at least this
constexpr float k_bodyHandHintScoreGate= 0.3f;
// and its 21 points score at least this on average
constexpr float k_bodyHandHintMeanScoreGate= 0.5f;
// and the body's own wrist landmark of that arm is at least this visible
constexpr float k_bodyHandHintWristVisibility= 0.5f;

// The whole-body model's two hands as search hints for the hand pipeline of
// the same camera. outHints gets one hint per hand that clears the three
// gates above and whose palm center lies inside the frame: the palm
// center, the wrist to middle knuckle direction and length, and the hand's
// own palm keypoints, so the seeded crop is the one a tracked slot would
// build from those landmarks. outHandBoxes gets both hands' boxes whatever
// they score, which is what the pipeline protects a slot by when a hint finds
// no free one: an unseen hand still sits where the model put its arm's end,
// and protecting it errs toward leaving a slot alone. Both stay empty for an
// observation without the whole-body blocks.
void makeBodyHandHints(const BodyPoseObservation& body, int frameWidth, int frameHeight,
					   std::vector<HandSearchHint>& outHints, std::vector<HandBox>& outHandBoxes);
