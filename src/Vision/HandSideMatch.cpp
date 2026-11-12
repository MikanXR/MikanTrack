#include "HandSideMatch.h"

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
		for (int landmark : kGatedLandmarks)
		{
			if (extras.handScores[sideIndex][landmark] < k_handShapeScoreGate)
				return match;
		}
		float sum= 0.f;
		for (int landmark= 0; landmark < HAND_LANDMARK_COUNT; ++landmark)
		{
			sum+= glm::length(glm::vec2(slotImagePoints[landmark]) - glm::vec2(extras.hands[sideIndex][landmark]));
		}
		match.distance[sideIndex]= sum / (float)HAND_LANDMARK_COUNT / slotPalmPx;
	}

	const int nearer= match.distance[0] <= match.distance[1] ? 0 : 1;
	if (match.distance[nearer] <= k_handShapeMarginRatio * match.distance[1 - nearer])
		match.side= nearer;
	return match;
}
