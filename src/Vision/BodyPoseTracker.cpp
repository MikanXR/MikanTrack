#include "BodyPoseTracker.h"

#include <algorithm>

#include "glm/geometric.hpp"

#include "Logger.h"
#include "Profiler.h"

// Grow the box around the tracked keypoints. The model applies its own 1.25
// padding on top; this only has to cover motion between model frames.
static constexpr float kTrackedBoxExpansion= 1.15f;
// A tracked box smaller than this (px) is treated as collapsed
static constexpr float kMinBoxSize= 24.f;

// COCO 17 -> the 33-slot BlazePose layout, which stays the canonical index
// space for observations so every downstream consumer (solver, overlay,
// recordings, dumps) is backend-independent - and so recordings made before
// the backend swap still load. Slots this model does not fill are reported
// through BodyPoseObservation::providedMask rather than being invented.
static constexpr int k_cocoToPoseLandmark[COCO_KEYPOINT_COUNT]= {
	0,  // nose
	2,  // left eye
	5,  // right eye
	7,  // left ear
	8,  // right ear
	11, // left shoulder
	12, // right shoulder
	13, // left elbow
	14, // right elbow
	15, // left wrist
	16, // right wrist
	23, // left hip
	24, // right hip
	25, // left knee
	26, // right knee
	27, // left ankle
	28, // right ankle
};

// The whole-body model's extra keypoints that have a BlazePose slot: the eye
// corners and mouth corners from the face block, and per hand the pinky and
// index knuckles and the thumb's second knuckle, which is what BlazePose
// means by its hand slots. The face and hand blocks ride whole beside the
// slots, in the observation's whole-body extras.
struct WholeBodySlot
{
	int keypoint;
	int landmark;
};
static constexpr WholeBodySlot k_wholeBodyExtraSlots[]= {
	{WHOLEBODY_FACE_START + WHOLEBODY_FACE_LEFT_EYE_INNER, 1},
	{WHOLEBODY_FACE_START + WHOLEBODY_FACE_LEFT_EYE_OUTER, 3},
	{WHOLEBODY_FACE_START + WHOLEBODY_FACE_RIGHT_EYE_INNER, 4},
	{WHOLEBODY_FACE_START + WHOLEBODY_FACE_RIGHT_EYE_OUTER, 6},
	{WHOLEBODY_FACE_START + WHOLEBODY_FACE_MOUTH_LEFT, 9},
	{WHOLEBODY_FACE_START + WHOLEBODY_FACE_MOUTH_RIGHT, 10},
	{WHOLEBODY_LEFT_HAND_START + WHOLEBODY_HAND_PINKY_MCP, 17},
	{WHOLEBODY_RIGHT_HAND_START + WHOLEBODY_HAND_PINKY_MCP, 18},
	{WHOLEBODY_LEFT_HAND_START + WHOLEBODY_HAND_INDEX_MCP, 19},
	{WHOLEBODY_RIGHT_HAND_START + WHOLEBODY_HAND_INDEX_MCP, 20},
	{WHOLEBODY_LEFT_HAND_START + WHOLEBODY_HAND_THUMB_MCP, 21},
	{WHOLEBODY_RIGHT_HAND_START + WHOLEBODY_HAND_THUMB_MCP, 22},
};

const char* bodyPoseBackendToString(eBodyPoseBackend backend)
{
	switch (backend)
	{
	case eBodyPoseBackend::Rtmw3dWholeBody:
		return "rtmw3d";
	case eBodyPoseBackend::RtmPoseBody:
	default:
		return "rtmpose";
	}
}

eBodyPoseBackend bodyPoseBackendFromString(const std::string& name)
{
	return name == "rtmw3d" ? eBodyPoseBackend::Rtmw3dWholeBody : eBodyPoseBackend::RtmPoseBody;
}

bool BodyPoseTracker::load(const std::string& modelDir, const std::string& preferredEp,
						   const BodyPoseTrackerConfig& config)
{
	setConfig(config);

	const char* landmarkModelFile=
		config.backend == eBodyPoseBackend::Rtmw3dWholeBody ? "/rtmw3d_wholebody.onnx" : "/rtmpose_body.onnx";
	m_bLoaded= m_detector.load(modelDir + "/person_detection.onnx", preferredEp) &&
		m_landmarkModel.load(modelDir + landmarkModelFile, preferredEp);

	if (!m_bLoaded)
	{
		MIKAN_MT_LOG_WARNING("BodyPoseTracker::load")
			<< "Body pose models missing or unloadable in " << modelDir
			<< " - body pose stage disabled (re-run InitialSetup_x64.bat)";
	}
	reset();
	return m_bLoaded;
}

void BodyPoseTracker::setConfig(const BodyPoseTrackerConfig& config)
{
	const eBodyPoseBackend loadedBackend= m_config.backend;
	m_config= config;
	if (m_bLoaded)
		m_config.backend= loadedBackend;
	m_config.frameDivider= std::max(config.frameDivider, 1);
	m_config.detectorIntervalFrames= std::max(config.detectorIntervalFrames, 1);
}

void BodyPoseTracker::reset()
{
	m_bBoxTracked= false;
	m_lastObservation= BodyPoseObservation();
	m_frameIndex= 0;
	m_modelFrameIndex= 0;
	m_lastDetectModelFrame= -1000;
	m_bChallengePending= false;
}

bool BodyPoseTracker::detectPersonBox(const cv::Mat& bgrFrame, glm::vec2& outBoxMin, glm::vec2& outBoxMax)
{
	m_detector.detect(bgrFrame, m_detections);

	for (const PersonDetection& detection : m_detections)
	{
		// The detection's own box is a FACE box, not a person box - feeding
		// it to a top-down model cropped the arms away (measured: landmark
		// scores collapsed from ~0.7 to ~0.3 on exactly the frames the
		// detector supplied the box). The person extent lives in the
		// keypoints: a square about the hip center reaching the full-body
		// point, which is the crop convention this detector was built for.
		const glm::vec2& hipCenter= detection.keypoints[0];
		const glm::vec2& fullBodyPoint= detection.keypoints[1];
		const float radius= glm::length(fullBodyPoint - hipCenter);
		if (radius < kMinBoxSize)
			continue;

		// Clipped to the image: for a person truncated at a desk that square
		// runs off the bottom, and a crop mostly outside the frame is mostly
		// padding
		const glm::vec2 imageMax((float)bgrFrame.cols, (float)bgrFrame.rows);
		const glm::vec2 clippedMin= glm::max(hipCenter - glm::vec2(radius), glm::vec2(0.f));
		const glm::vec2 clippedMax= glm::min(hipCenter + glm::vec2(radius), imageMax);
		if ((clippedMax.x - clippedMin.x) < kMinBoxSize || (clippedMax.y - clippedMin.y) < kMinBoxSize)
			continue;

		outBoxMin= clippedMin;
		outBoxMax= clippedMax;
		return true;
	}
	return false;
}

eBodyBoxSource BodyPoseTracker::acquireBox(const cv::Mat& bgrFrame, glm::vec2& outBoxMin, glm::vec2& outBoxMax)
{
	if (m_bBoxTracked)
	{
		outBoxMin= m_trackedBoxMin;
		outBoxMax= m_trackedBoxMax;
		return eBodyBoxSource::Tracked;
	}

	// Counted whether or not it fires. The person detector is trained on
	// whole people, so on a subject truncated at a desk it misses often, and
	// the periodic detect is not due again the frame after a miss.
	m_lastDetectModelFrame= m_modelFrameIndex;
	if (detectPersonBox(bgrFrame, outBoxMin, outBoxMax))
		return eBodyBoxSource::Detector;

	// Nothing to go on. A top-down model still reads a centered subject from
	// the whole frame, which beats reporting nothing until the detector fires.
	outBoxMin= glm::vec2(0.f, 0.f);
	outBoxMax= glm::vec2((float)bgrFrame.cols, (float)bgrFrame.rows);
	return eBodyBoxSource::FullFrame;
}

bool BodyPoseTracker::makeTrackedBox(const RtmPoseResult& result, glm::vec2& outBoxMin, glm::vec2& outBoxMax) const
{
	// The confident BODY keypoints' extent. No full-body assumption is
	// involved, so a truncated subject simply produces a smaller box instead
	// of a fabricated one. The whole body model's face and hand points stay
	// out of it: they sit inside the body's extent when present and would
	// only add their own misses.
	glm::vec2 trackedMin(FLT_MAX);
	glm::vec2 trackedMax(-FLT_MAX);
	int confidentCount= 0;
	for (int keypoint= 0; keypoint < COCO_KEYPOINT_COUNT; ++keypoint)
	{
		if (result.scores[keypoint] < m_config.keypointScoreThreshold)
			continue;
		const glm::vec2 point(result.points[keypoint]);
		trackedMin= glm::min(trackedMin, point);
		trackedMax= glm::max(trackedMax, point);
		confidentCount++;
	}
	if (confidentCount < 3 || (trackedMax.x - trackedMin.x) < kMinBoxSize || (trackedMax.y - trackedMin.y) < kMinBoxSize)
		return false;

	const glm::vec2 boxCenter= (trackedMin + trackedMax) * 0.5f;
	const glm::vec2 halfExtent= (trackedMax - trackedMin) * 0.5f * kTrackedBoxExpansion;
	outBoxMin= boxCenter - halfExtent;
	outBoxMax= boxCenter + halfExtent;
	return true;
}

void BodyPoseTracker::challengeTrackedBox(const cv::Mat& bgrFrame)
{
	m_bChallengePending= false;
	if (!m_bBoxTracked || !m_lastObservation.valid)
		return;

	glm::vec2 boxMin(0.f);
	glm::vec2 boxMax(0.f);
	if (!detectPersonBox(bgrFrame, boxMin, boxMax))
		return;
	m_landmarkModel.estimate(bgrFrame, boxMin, boxMax, m_challengeResult);
	if (!m_challengeResult.valid)
		return;

	// The observation's confidence, on the challenger
	float scoreSum= 0.f;
	for (int keypoint= 0; keypoint < COCO_KEYPOINT_COUNT; ++keypoint)
		scoreSum+= std::clamp(m_challengeResult.scores[keypoint], 0.f, 1.f);
	if (scoreSum / (float)COCO_KEYPOINT_COUNT <= m_lastObservation.confidence)
		return;

	glm::vec2 challengerMin(0.f);
	glm::vec2 challengerMax(0.f);
	if (makeTrackedBox(m_challengeResult, challengerMin, challengerMax))
	{
		m_trackedBoxMin= challengerMin;
		m_trackedBoxMax= challengerMax;
	}
}

// The observation's blocks are sized by the model's layout
static_assert(WHOLEBODY_LEFT_HAND_START - WHOLEBODY_FACE_START == WHOLEBODY_FACE_LANDMARK_COUNT);
static_assert(WHOLEBODY_HAND_KEYPOINT_COUNT == WHOLEBODY_HAND_LANDMARK_COUNT);

void BodyPoseTracker::fillObservation(const RtmPoseResult& result, BodyPoseObservation& outObservation)
{
	outObservation.hasDepth= result.hasDepth;

	// SimCC peaks are not probabilities; they are reported as-is and clamped
	// so downstream visibility gates keep their [0,1] meaning
	auto fill= [&](int keypoint, int landmark) {
		const float score= std::clamp(result.scores[keypoint], 0.f, 1.f);
		outObservation.imagePoints[landmark]= result.points[keypoint];
		outObservation.visibility[landmark]= score;
		outObservation.providedMask|= 1ull << landmark;
		return score;
	};

	// The observation's confidence is the body's: the mean over the 17 COCO
	// keypoints, whichever backend ran, so the two backends read alike
	float scoreSum= 0.f;
	for (int keypoint= 0; keypoint < COCO_KEYPOINT_COUNT; ++keypoint)
		scoreSum+= fill(keypoint, k_cocoToPoseLandmark[keypoint]);
	outObservation.confidence= scoreSum / (float)COCO_KEYPOINT_COUNT;

	if (result.keypointCount >= WHOLEBODY_KEYPOINT_COUNT)
	{
		for (const WholeBodySlot& slot : k_wholeBodyExtraSlots)
			fill(slot.keypoint, slot.landmark);

		// The face and hand blocks whole, in the model's own orders
		WholeBodyExtras& extras= outObservation.wholeBody;
		extras.present= true;
		for (int index= 0; index < WHOLEBODY_FACE_LANDMARK_COUNT; ++index)
		{
			const int keypoint= WHOLEBODY_FACE_START + index;
			extras.face[index]= result.points[keypoint];
			extras.faceScores[index]= std::clamp(result.scores[keypoint], 0.f, 1.f);
		}
		const int handStarts[2]= {WHOLEBODY_LEFT_HAND_START, WHOLEBODY_RIGHT_HAND_START};
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			for (int index= 0; index < WHOLEBODY_HAND_LANDMARK_COUNT; ++index)
			{
				const int keypoint= handStarts[sideIndex] + index;
				extras.hands[sideIndex][index]= result.points[keypoint];
				extras.handScores[sideIndex][index]= std::clamp(result.scores[keypoint], 0.f, 1.f);
			}
		}
	}
}

void BodyPoseTracker::process(const cv::Mat& bgrFrame, BodyPoseObservation& outObservation)
{
	MIKAN_PROFILE_FUNCTION();
	const int64_t frameIndex= m_frameIndex++;
	if ((frameIndex % (int64_t)m_config.frameDivider) != 0)
	{
		// Divided-out frame: hold the last model result. Recording taps sit
		// downstream, so this cadence is baked into recorded observations and
		// replay never needs to know the divider. The frame is free of the
		// landmark model, so a due detect scores its box here.
		if (m_bChallengePending && m_bLoaded && !bgrFrame.empty())
			challengeTrackedBox(bgrFrame);
		outObservation= m_lastObservation;
		return;
	}

	outObservation= BodyPoseObservation();
	if (m_bLoaded && !bgrFrame.empty())
	{
		// Drift guard: periodically score a person box built from the image
		// against the one built from the previous keypoints, whatever the
		// model claims about its own confidence
		if (m_bBoxTracked && (m_modelFrameIndex - m_lastDetectModelFrame) >= (int64_t)m_config.detectorIntervalFrames)
		{
			m_lastDetectModelFrame= m_modelFrameIndex;
			m_bChallengePending= true;
		}

		glm::vec2 boxMin(0.f);
		glm::vec2 boxMax(0.f);
		const eBodyBoxSource boxSource= acquireBox(bgrFrame, boxMin, boxMax);

		m_landmarkModel.estimate(bgrFrame, boxMin, boxMax, m_landmarkResult);
		if (m_landmarkResult.valid)
		{
			m_bBoxTracked= makeTrackedBox(m_landmarkResult, m_trackedBoxMin, m_trackedBoxMax);

			outObservation.valid= true;
			outObservation.boxSource= boxSource;
			outObservation.modelFrameIndex= m_modelFrameIndex;
			fillObservation(m_landmarkResult, outObservation);
		}
		else
		{
			m_bBoxTracked= false;
		}

		m_modelFrameIndex++;
	}

	m_lastObservation= outObservation;
	// Every frame is a model frame, so the challenge shares this one
	if (m_bChallengePending && m_config.frameDivider == 1 && m_bLoaded && !bgrFrame.empty())
		challengeTrackedBox(bgrFrame);
}
