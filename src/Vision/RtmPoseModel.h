#pragma once

#include <array>
#include <string>
#include <vector>

#include "glm/ext/vector_float2.hpp"
#include "glm/ext/vector_float3.hpp"
#include "opencv2/core/mat.hpp"

#include "OnnxSession.h"

// COCO 17-keypoint order, which both RTMPose body models emit first
enum class eCocoKeypoint : int
{
	NOSE= 0,
	LEFT_EYE= 1,
	RIGHT_EYE= 2,
	LEFT_EAR= 3,
	RIGHT_EAR= 4,
	LEFT_SHOULDER= 5,
	RIGHT_SHOULDER= 6,
	LEFT_ELBOW= 7,
	RIGHT_ELBOW= 8,
	LEFT_WRIST= 9,
	RIGHT_WRIST= 10,
	LEFT_HIP= 11,
	RIGHT_HIP= 12,
	LEFT_KNEE= 13,
	RIGHT_KNEE= 14,
	LEFT_ANKLE= 15,
	RIGHT_ANKLE= 16,
};
constexpr int COCO_KEYPOINT_COUNT= 17;

// COCO-WholeBody 133-keypoint order (the RTMW family): the 17 COCO body
// keypoints, then 6 foot points, the 68 iBUG face landmarks, and 21 landmarks
// per hand in the MediaPipe hand order (wrist, then thumb to pinky, base to
// tip). Only the starts of the blocks are named; the face and hand indices
// below are the ones the tracker reads.
constexpr int WHOLEBODY_KEYPOINT_COUNT= 133;
constexpr int WHOLEBODY_FOOT_START= 17;
constexpr int WHOLEBODY_FACE_START= 23;
constexpr int WHOLEBODY_LEFT_HAND_START= 91;
constexpr int WHOLEBODY_RIGHT_HAND_START= 112;
constexpr int WHOLEBODY_HAND_KEYPOINT_COUNT= 21;
// iBUG face points, offsets from WHOLEBODY_FACE_START. Sides are the
// subject's own.
constexpr int WHOLEBODY_FACE_RIGHT_EYE_OUTER= 36;
constexpr int WHOLEBODY_FACE_RIGHT_EYE_INNER= 39;
constexpr int WHOLEBODY_FACE_LEFT_EYE_INNER= 42;
constexpr int WHOLEBODY_FACE_LEFT_EYE_OUTER= 45;
constexpr int WHOLEBODY_FACE_MOUTH_RIGHT= 48;
constexpr int WHOLEBODY_FACE_MOUTH_LEFT= 54;
// Hand points, offsets from either hand's start
constexpr int WHOLEBODY_HAND_WRIST= 0;
constexpr int WHOLEBODY_HAND_THUMB_MCP= 2;
constexpr int WHOLEBODY_HAND_INDEX_MCP= 5;
constexpr int WHOLEBODY_HAND_PINKY_MCP= 17;

constexpr int RTM_MAX_KEYPOINT_COUNT= WHOLEBODY_KEYPOINT_COUNT;

struct RtmPoseResult
{
	bool valid= false;
	// 17 (body) or 133 (whole body), from the loaded model
	int keypointCount= 0;
	// True when the model carries a depth axis: points[].z is then the
	// keypoint's depth relative to the hip midpoint, meters, positive away
	// from the camera. Zero otherwise.
	bool hasDepth= false;
	// x, y full-frame px
	std::array<glm::vec3, RTM_MAX_KEYPOINT_COUNT> points{};
	// SimCC peak, ~[0,1]
	std::array<float, RTM_MAX_KEYPOINT_COUNT> scores{};
	// Peak of the depth axis, ~[0,1], where hasDepth. Lower than the planar
	// peaks by nature (the depth distribution is broad), so it is reported
	// beside the score rather than folded into it.
	std::array<float, RTM_MAX_KEYPOINT_COUNT> depthScores{};
};

// RTMPose-family SimCC models via ONNX Runtime: RTMPose body (17 keypoints,
// two planar axes) and RTMW3D whole body (133 keypoints, a third depth
// axis). One class because the two share everything but their sizes: the
// input size, the keypoint count and the presence of the depth axis are
// read from the model's own tensor shapes at load.
//
// TOP-DOWN, which is the reason this family was chosen: the caller supplies
// the person box and the model predicts each keypoint independently inside
// it. The holistic model tried before it owned its own region of interest,
// derived from a hip center and a "full body" point, and always emitted one
// coherent full-body skeleton - so a person truncated at a desk had no
// correct crop and got a fabricated lower body that dragged the arms with
// it.
//
// Model contract (rtmpose_body.onnx, MMDeploy end2end export):
//   input  input    [1,3,256,192] NCHW RGB, (x - mean) / std with the
//                   ImageNet statistics below, from a 1.25-padded
//                   aspect-corrected affine crop of the person box
//   output simcc_x  [1,17,384]  per-keypoint x class scores (192 * 2)
//   output simcc_y  [1,17,512]  per-keypoint y class scores (256 * 2)
// Model contract (rtmw3d_wholebody.onnx, RTMW3D-X torch export):
//   input  input    [1,3,384,288], the same crop and normalization
//   output simcc_x  [1,133,576] (288 * 2)
//   output simcc_y  [1,133,768] (384 * 2)
//   output simcc_z  [1,133,576] depth class scores over 288 bins * 2
// Decode is argmax over each axis divided by the 2.0 split ratio; x and y
// map back through the inverse affine, and the depth bin maps to meters as
// (bin / (depthBins / 2) - 1) * z_range with the codec's z_range of
// 2.1744869 m, relative to the hip midpoint the model was trained around.
// The score is min(peakX, peakY).
class RtmPoseModel
{
public:
	bool load(const std::string& modelPath, const std::string& preferredEp);
	bool isLoaded() const { return m_session.isValid(); }
	const char* activeEp() const { return m_session.activeEp(); }
	// 0 until the model states it: an export with a dynamic keypoint
	// dimension reports it after the first estimate
	int getKeypointCount() const { return m_keypointCount; }
	bool hasDepth() const { return m_depthBins > 0; }

	// boxMin/boxMax: the person box in full-frame px
	void estimate(const cv::Mat& bgrFrame, const glm::vec2& boxMin, const glm::vec2& boxMax,
				  RtmPoseResult& outResult);

private:
	OnnxSession m_session;
	int m_inputWidth= 0;
	int m_inputHeight= 0;
	int m_keypointCount= 0;
	// Depth axis bins after the split ratio (0 for a planar model)
	int m_depthBins= 0;
	int m_simccXOutputIndex= -1;
	int m_simccYOutputIndex= -1;
	int m_simccZOutputIndex= -1;

	// Preallocated scratch (steady-state: no per-frame heap churn)
	std::vector<float> m_inputBuffer;
	cv::Mat m_cropMat;
	cv::Mat m_rgbMat;
	std::array<cv::Mat, 3> m_channelPlanes;
};
