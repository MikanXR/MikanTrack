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
// And at most this, in the slot's palm lengths. The margin alone names a
// side for a thing on neither hand, one distance of several palm lengths
// still being a fraction of the other.
constexpr float k_handShapeMaxDistance= 0.5f;

struct HandShapeMatch
{
	// 0 left, 1 right, as the whole-body block's sides are. -1 when the block
	// is absent, either whole-body hand scores under the gate, the slot has
	// no size, neither hand is nearer by the margin, or the nearer one is
	// past the distance bar.
	int side= -1;
	// Per whole-body hand: whether it scored, and the mean point distance to
	// it in the slot's palm lengths. A distance is meaningful only where its
	// hand scored.
	bool scored[2]= {false, false};
	float distance[2]= {0.f, 0.f};
};

HandShapeMatch matchHandSideByShape(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& slotImagePoints,
									const WholeBodyExtras& extras);

// Whether a frame's two published hands change places, so that each sits
// under the side the whole-body hands name for it. slotImagePoints[side] is
// the hand published under that side, null where none is. Per hand:
//   - one the shape match decides goes to its matched side
//   - one whose own label's whole-body hand does not score, and which is
//     within the distance bar of the other whole-body hand, goes to that
//     hand's side (one hand in view, displaced to the wrong label)
//   - any other keeps its label
// With two hands, one that is decided takes its side and the other takes the
// side left over. Two that want the same side are two slots on one hand, or
// one on it and one on nothing: the one under that side keeps it while it is
// within the bar of that side's whole-body hand, and gives it to a decided
// other when it is not. A pure function of the frame, and of its own outcome:
// applied to the hands it arranged, it moves nothing.
//
// outSideDecided, when given, is filled per side as the hands end up: whether
// the hand under that side is on that side's whole-body hand, within the bar,
// with the match or the one-hand rule deciding it or the other whole-body
// hand unscored. A hand the match leaves undecided between two scoring hands
// is not decided, and neither is the loser of a side two hands wanted.
bool correctHandSides(const std::array<glm::vec3, HAND_LANDMARK_COUNT>* const slotImagePoints[2],
					  const WholeBodyExtras& extras, bool* outSideDecided= nullptr);

// correctHandSides on a camera's result, ahead of the lift: where it says so
// the two hands change places, each takes its entry's side, and each is
// flagged as moved. A hand whose side the whole-body hands decided is flagged
// as that, moved or not. Reads the result's own body observation. Returns
// whether they changed places.
bool applyHandSideCorrection(TrackingFrameResult& ioResult);
