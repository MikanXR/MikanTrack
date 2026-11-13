#pragma once

#include <array>

#include "BodyPoseTypes.h"
#include "TrackingTypes.h"

// Which of the whole-body model's two hands a hand slot is, by shape: the
// mean distance over the 21 image points to each whole-body hand, in the
// slot's palm lengths. The body model labels a hand by the arm it ends, so
// the answer is the slot's true side whatever label the hand model gave it,
// and it still tells the hands apart where their wrists are together.

// A whole-body hand counts when its wrist and its index, middle and pinky
// knuckles all score at least this
constexpr float k_handShapeScoreGate= 0.3f;
// The nearer hand is the slot's when its distance is at most this share of
// the other's
constexpr float k_handShapeMarginRatio= 0.8f;

struct HandShapeMatch
{
	// 0 left, 1 right, as the whole-body block's sides are. -1 when the block
	// is absent, either whole-body hand scores under the gate, the slot has
	// no size, or neither hand is nearer by the margin.
	int side= -1;
	// The mean point distance to each whole-body hand, in the slot's palm
	// lengths. Meaningful only when both hands scored.
	float distance[2]= {0.f, 0.f};
};

HandShapeMatch matchHandSideByShape(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& slotImagePoints,
									const WholeBodyExtras& extras);
