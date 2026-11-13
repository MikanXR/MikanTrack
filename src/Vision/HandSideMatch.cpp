#include "HandSideMatch.h"

#include <utility>

#include "glm/geometric.hpp"

HandShapeMatch matchHandSideByShape(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& slotImagePoints,
									const WholeBodyExtras& extras)
{
	static_assert(HAND_LANDMARK_COUNT == WHOLEBODY_HAND_LANDMARK_COUNT, "the slot and the block share a layout");

	HandShapeMatch match;
	if (!extras.present)
		return match;

	const float slotPalmPx= glm::length(glm::vec2(slotImagePoints[(int)eHandLandmark::MIDDLE_MCP]) -
										glm::vec2(slotImagePoints[(int)eHandLandmark::WRIST]));
	if (slotPalmPx < 1.f)
		return match;

	static const int kGatedLandmarks[]= {(int)eHandLandmark::WRIST, (int)eHandLandmark::INDEX_MCP,
										 (int)eHandLandmark::MIDDLE_MCP, (int)eHandLandmark::PINKY_MCP};
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		match.scored[sideIndex]= true;
		for (int landmark : kGatedLandmarks)
		{
			if (extras.handScores[sideIndex][landmark] < k_handShapeScoreGate)
				match.scored[sideIndex]= false;
		}
		if (!match.scored[sideIndex])
			continue;
		float sum= 0.f;
		for (int landmark= 0; landmark < HAND_LANDMARK_COUNT; ++landmark)
		{
			sum+= glm::length(glm::vec2(slotImagePoints[landmark]) - glm::vec2(extras.hands[sideIndex][landmark]));
		}
		match.distance[sideIndex]= sum / (float)HAND_LANDMARK_COUNT / slotPalmPx;
	}
	if (!match.scored[0] || !match.scored[1])
		return match;

	const int nearer= match.distance[0] <= match.distance[1] ? 0 : 1;
	if (match.distance[nearer] <= k_handShapeMarginRatio * match.distance[1 - nearer] &&
		match.distance[nearer] <= k_handShapeMaxDistance)
		match.side= nearer;
	return match;
}

bool correctHandSides(const std::array<glm::vec3, HAND_LANDMARK_COUNT>* const slotImagePoints[2],
					  const WholeBodyExtras& extras)
{
	// Per published hand: the side it should sit under, whether the
	// whole-body hands decided that, and whether it is on that side's hand
	int wantedSide[2]= {0, 1};
	bool bDecided[2]= {false, false};
	bool bOnClaimedHand[2]= {false, false};
	for (int labelSide= 0; labelSide < 2; ++labelSide)
	{
		if (slotImagePoints[labelSide] == nullptr)
			continue;
		const HandShapeMatch match= matchHandSideByShape(*slotImagePoints[labelSide], extras);
		const int otherSide= 1 - labelSide;
		if (match.side >= 0)
		{
			wantedSide[labelSide]= match.side;
			bDecided[labelSide]= true;
		}
		else if (!match.scored[labelSide] && match.scored[otherSide] &&
				 match.distance[otherSide] <= k_handShapeMaxDistance)
		{
			wantedSide[labelSide]= otherSide;
			bDecided[labelSide]= true;
		}
		const int claimedSide= wantedSide[labelSide];
		bOnClaimedHand[labelSide]= match.scored[claimedSide] && match.distance[claimedSide] <= k_handShapeMaxDistance;
	}

	const bool bTracked[2]= {slotImagePoints[0] != nullptr, slotImagePoints[1] != nullptr};
	if (!bTracked[0] && !bTracked[1])
		return false;
	if (bTracked[0] != bTracked[1])
	{
		// One hand: it moves to the empty entry when that is its side
		const int labelSide= bTracked[0] ? 0 : 1;
		return wantedSide[labelSide] != labelSide;
	}
	if (wantedSide[0] != wantedSide[1])
		return wantedSide[0] == 1;

	// Both want one side. An undecided hand wants its own label, so the one
	// labelled the other side is the challenger and only a decided one moves.
	// A holder on the contested whole-body hand keeps the entry, decided or
	// not: these are two slots on one hand, their distances to it differ by
	// jitter, and the nearer taking the entry trades it back and forth. With
	// one whole-body hand unscored the match decides nothing for a slot
	// already under its own side, so the holder's test is the bar. A holder
	// that is not on the hand gives way.
	const int contestedSide= wantedSide[0];
	const int holder= contestedSide;
	const int challenger= 1 - contestedSide;
	return bDecided[challenger] && !bOnClaimedHand[holder];
}

bool applyHandSideCorrection(TrackingFrameResult& ioResult)
{
	if (!ioResult.body.valid)
		return false;
	const std::array<glm::vec3, HAND_LANDMARK_COUNT>* const slotImagePoints[2]= {
		ioResult.hands[0].tracked ? &ioResult.hands[0].imagePoints : nullptr,
		ioResult.hands[1].tracked ? &ioResult.hands[1].imagePoints : nullptr};
	if (!correctHandSides(slotImagePoints, ioResult.body.wholeBody))
		return false;

	std::swap(ioResult.hands[0], ioResult.hands[1]);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		TrackedHand& hand= ioResult.hands[sideIndex];
		if (!hand.tracked)
			continue;
		hand.side= (eHandSide)sideIndex;
		hand.bSideCorrected= true;
	}
	return true;
}
