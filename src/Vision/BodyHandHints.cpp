#include "BodyHandHints.h"

#include "glm/geometric.hpp"

#include "HandLandmarkModel.h"

void makeBodyHandHints(const BodyPoseObservation& body, int frameWidth, int frameHeight,
					   std::vector<HandSearchHint>& outHints, std::vector<HandBox>& outHandBoxes)
{
	outHints.clear();
	outHandBoxes.clear();
	if (!body.valid || !body.wholeBody.present)
		return;

	const int wristIndex= (int)eHandLandmark::WRIST;
	const int middleMcpIndex= (int)eHandLandmark::MIDDLE_MCP;
	const int gatedLandmarks[]= {wristIndex, (int)eHandLandmark::INDEX_MCP, middleMcpIndex,
								 (int)eHandLandmark::PINKY_MCP};

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const auto& points= body.wholeBody.hands[sideIndex];
		const auto& scores= body.wholeBody.handScores[sideIndex];

		HandBox box;
		box.min= glm::vec2(points[0]);
		box.max= glm::vec2(points[0]);
		for (const glm::vec3& point : points)
		{
			box.min= glm::min(box.min, glm::vec2(point));
			box.max= glm::max(box.max, glm::vec2(point));
		}
		outHandBoxes.push_back(box);

		bool bScored= true;
		for (int landmark : gatedLandmarks)
			bScored= bScored && scores[landmark] >= k_bodyHandHintScoreGate;
		if (!bScored)
			continue;

		const glm::vec2 wrist(points[wristIndex]);
		const glm::vec2 middleMcp(points[middleMcpIndex]);
		const float palmSize= glm::length(middleMcp - wrist);
		const glm::vec2 center= (wrist + middleMcp) * 0.5f;
		if (palmSize < 1e-3f || center.x < 0.f || center.y < 0.f || center.x >= (float)frameWidth ||
			center.y >= (float)frameHeight)
			continue;

		HandSearchHint hint;
		hint.centerPx= center;
		hint.dirPx= (middleMcp - wrist) / palmSize;
		hint.palmSizePx= palmSize;
		hint.bFromBody= true;
		hint.bHasPalmKeypoints= true;
		hint.palmKeypoints= HandRoi::fromLandmarks(points).palmKeypoints;
		outHints.push_back(hint);
	}
}
