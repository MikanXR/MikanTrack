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

namespace
{
// Per published hand: the side it should sit under, whether the whole-body
// hands decided that, whether it is on that side's hand, and whether that
// reading can be trusted as the hand's side
struct SideClaims
{
	int wantedSide[2]= {0, 1};
	bool bDecided[2]= {false, false};
	bool bOnClaimedHand[2]= {false, false};
	bool bTrusted[2]= {false, false};
};

// Whether the two published hands change places
bool resolveSideClaims(const bool bTracked[2], const SideClaims& claims)
{
	if (!bTracked[0] && !bTracked[1])
		return false;
	if (bTracked[0] != bTracked[1])
	{
		// One hand: it moves to the empty entry when that is its side
		const int labelSide= bTracked[0] ? 0 : 1;
		return claims.wantedSide[labelSide] != labelSide;
	}
	if (claims.wantedSide[0] != claims.wantedSide[1])
		return claims.wantedSide[0] == 1;

	// Both want one side. An undecided hand wants its own label, so the one
	// labelled the other side is the challenger and only a decided one moves.
	// A holder on the contested whole-body hand keeps the entry, decided or
	// not: these are two slots on one hand, their distances to it differ by
	// jitter, and the nearer taking the entry trades it back and forth. With
	// one whole-body hand unscored the match decides nothing for a slot
	// already under its own side, so the holder's test is the bar. A holder
	// that is not on the hand gives way.
	const int contestedSide= claims.wantedSide[0];
	const int holder= contestedSide;
	const int challenger= 1 - contestedSide;
	return claims.bDecided[challenger] && !claims.bOnClaimedHand[holder];
}
} // namespace

bool correctHandSides(const std::array<glm::vec3, HAND_LANDMARK_COUNT>* const slotImagePoints[2],
					  const WholeBodyExtras& extras, bool* outSideDecided)
{
	SideClaims claims;
	for (int labelSide= 0; labelSide < 2; ++labelSide)
	{
		if (slotImagePoints[labelSide] == nullptr)
			continue;
		const HandShapeMatch match= matchHandSideByShape(*slotImagePoints[labelSide], extras);
		const int otherSide= 1 - labelSide;
		if (match.side >= 0)
		{
			claims.wantedSide[labelSide]= match.side;
			claims.bDecided[labelSide]= true;
		}
		else if (!match.scored[labelSide] && match.scored[otherSide] &&
				 match.distance[otherSide] <= k_handShapeMaxDistance)
		{
			claims.wantedSide[labelSide]= otherSide;
			claims.bDecided[labelSide]= true;
		}
		const int claimedSide= claims.wantedSide[labelSide];
		claims.bOnClaimedHand[labelSide]=
			match.scored[claimedSide] && match.distance[claimedSide] <= k_handShapeMaxDistance;
		claims.bTrusted[labelSide]=
			claims.bOnClaimedHand[labelSide] && (claims.bDecided[labelSide] || !match.scored[1 - claimedSide]);
	}

	const bool bTracked[2]= {slotImagePoints[0] != nullptr, slotImagePoints[1] != nullptr};
	const bool bSwap= resolveSideClaims(bTracked, claims);
	if (outSideDecided != nullptr)
	{
		outSideDecided[0]= false;
		outSideDecided[1]= false;
		for (int labelSide= 0; labelSide < 2; ++labelSide)
		{
			const int finalSide= bSwap ? 1 - labelSide : labelSide;
			if (bTracked[labelSide] && claims.bTrusted[labelSide] && claims.wantedSide[labelSide] == finalSide)
				outSideDecided[finalSide]= true;
		}
	}
	return bSwap;
}

bool applyHandSideCorrection(TrackingFrameResult& ioResult)
{
	ioResult.hands[0].bSideDecided= false;
	ioResult.hands[1].bSideDecided= false;
	if (!ioResult.body.valid)
		return false;
	const std::array<glm::vec3, HAND_LANDMARK_COUNT>* const slotImagePoints[2]= {
		ioResult.hands[0].tracked ? &ioResult.hands[0].imagePoints : nullptr,
		ioResult.hands[1].tracked ? &ioResult.hands[1].imagePoints : nullptr};
	bool bSideDecided[2]= {false, false};
	const bool bSwap= correctHandSides(slotImagePoints, ioResult.body.wholeBody, bSideDecided);
	if (bSwap)
	{
		std::swap(ioResult.hands[0], ioResult.hands[1]);
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			TrackedHand& hand= ioResult.hands[sideIndex];
			if (!hand.tracked)
				continue;
			hand.side= (eHandSide)sideIndex;
			hand.bSideCorrected= true;
		}
	}
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		ioResult.hands[sideIndex].bSideDecided= ioResult.hands[sideIndex].tracked && bSideDecided[sideIndex];
	return bSwap;
}
