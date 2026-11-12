#include "HandLandmarkModel.h"

#include <algorithm>
#include <cmath>

#include "glm/common.hpp"
#include "glm/ext/vector_int2.hpp"
#include "opencv2/imgproc.hpp"

#include "Logger.h"

static constexpr int kHandInputSize= 224;

// The crop is sampled at up to this many times the model resolution and box
// filtered down, so a large hand is averaged rather than point sampled
static constexpr int kMaxSupersample= 2;

// mp_handpose.py crop constants
static constexpr float kPalmBoxShiftY= -0.4f;
static constexpr float kPalmBoxEnlargeFactor= 3.f;
static constexpr float kHandBoxShiftY= -0.1f;
static constexpr float kHandBoxEnlargeFactor= 1.65f;

// The 7 palm keypoints expressed as hand landmark ids (PALM_LANDMARK_IDS)
static constexpr int kPalmLandmarkIds[7]= {0, 5, 9, 13, 17, 1, 2};

HandRoi HandRoi::fromPalmDetection(const PalmDetection& detection)
{
	HandRoi roi;
	roi.palmBoxMin= detection.boxMin;
	roi.palmBoxMax= detection.boxMax;
	roi.palmKeypoints= detection.keypoints;
	return roi;
}

HandRoi HandRoi::fromLandmarks(const std::array<glm::vec3, HAND_LANDMARK_COUNT>& imagePoints)
{
	HandRoi roi;
	for (int i= 0; i < 7; ++i)
		roi.palmKeypoints[i]= glm::vec2(imagePoints[kPalmLandmarkIds[i]]);

	roi.palmBoxMin= roi.palmKeypoints[0];
	roi.palmBoxMax= roi.palmKeypoints[0];
	for (int i= 1; i < 7; ++i)
	{
		roi.palmBoxMin= glm::min(roi.palmBoxMin, roi.palmKeypoints[i]);
		roi.palmBoxMax= glm::max(roi.palmBoxMax, roi.palmKeypoints[i]);
	}
	return roi;
}

bool HandLandmarkModel::load(const std::string& modelPath, const std::string& preferredEp)
{
	if (!m_session.create(modelPath, preferredEp))
		return false;

	// Map outputs by name (Identity/Identity_1/Identity_2/Identity_3, matching
	// the demo's tuple order: landmarks, conf, handedness, world landmarks);
	// fall back to positional order if the names differ.
	auto findByName= [this](const char* name, int fallback) -> int {
		for (size_t i= 0; i < m_session.getOutputCount(); ++i)
		{
			if (m_session.getOutputName(i) == name)
				return (int)i;
		}
		MIKAN_MT_LOG_WARNING("HandLandmarkModel::load")
			<< "Output '" << name << "' not found, assuming index " << fallback;
		return fallback;
	};
	m_landmarkOutputIndex= findByName("Identity", 0);
	m_confidenceOutputIndex= findByName("Identity_1", 1);
	m_handednessOutputIndex= findByName("Identity_2", 2);
	m_worldLandmarkOutputIndex= findByName("Identity_3", 3);

	m_inputBuffer.resize((size_t)kHandInputSize * kHandInputSize * 3);
	return true;
}

void HandLandmarkModel::estimate(
	const cv::Mat& bgrFrame,
	const HandRoi& roi,
	HandLandmarkResult& outResult)
{
	outResult= HandLandmarkResult();
	if (!m_session.isValid() || bgrFrame.empty())
		return;

	// -- Preprocess (mp_handpose.py::_preprocess geometry, one resampling) --
	// The crop is a square in the frame rotated so the wrist -> middle MCP
	// direction points up. The rotated frame is only ever a coordinate
	// system: no rotated image is built, and the square is sampled straight
	// out of the source frame, so the cost is fixed by the model resolution
	// rather than by how large the hand is.

	// 1) rotation from wrist (kp0) -> middle MCP (kp2), about the palm box center
	const float radians= PalmDetector::computeRotation(roi.palmKeypoints[0], roi.palmKeypoints[2]);
	const double angleDeg= (double)radians * 180.0 / 3.14159265358979;
	const glm::vec2 rotationCenter= (roi.palmBoxMin + roi.palmBoxMax) * 0.5f;
	const cv::Mat rotatedFromImage=
		cv::getRotationMatrix2D(cv::Point2f(rotationCenter.x, rotationCenter.y), angleDeg, 1.0);
	cv::Mat imageFromRotated;
	cv::invertAffineTransform(rotatedFromImage, imageFromRotated);

	auto applyAffine= [](const cv::Mat& m, const glm::vec2& p) -> glm::vec2 {
		const double* m0= m.ptr<double>(0);
		const double* m1= m.ptr<double>(1);
		return glm::vec2(
			(float)(m0[0] * p.x + m0[1] * p.y + m0[2]),
			(float)(m1[0] * p.x + m1[1] * p.y + m1[2]));
	};

	// 2) rotated palm keypoint bbox, shifted [0,-0.4], enlarged 3x, padded to
	// a square about its center (int truncation matches numpy astype(np.int32))
	glm::vec2 rotatedKpMin= applyAffine(rotatedFromImage, roi.palmKeypoints[0]);
	glm::vec2 rotatedKpMax= rotatedKpMin;
	for (int i= 1; i < 7; ++i)
	{
		const glm::vec2 p= applyAffine(rotatedFromImage, roi.palmKeypoints[i]);
		rotatedKpMin= glm::min(rotatedKpMin, p);
		rotatedKpMax= glm::max(rotatedKpMax, p);
	}
	const glm::vec2 kpExtent= rotatedKpMax - rotatedKpMin;
	const glm::vec2 boxCenter=
		(rotatedKpMin + rotatedKpMax) * 0.5f + glm::vec2(0.f, kPalmBoxShiftY) * kpExtent;
	const glm::vec2 boxHalfSize= kpExtent * kPalmBoxEnlargeFactor * 0.5f;
	const glm::ivec2 boxMin((int)(boxCenter.x - boxHalfSize.x), (int)(boxCenter.y - boxHalfSize.y));
	const glm::ivec2 boxMax((int)(boxCenter.x + boxHalfSize.x), (int)(boxCenter.y + boxHalfSize.y));
	const int side= std::max(boxMax.x - boxMin.x, boxMax.y - boxMin.y);
	if (side <= 0)
		return;
	const glm::ivec2 padBefore((side - (boxMax.x - boxMin.x)) / 2, (side - (boxMax.y - boxMin.y)) / 2);
	const glm::vec2 squareMin= glm::vec2(boxMin - padBefore);
	const glm::vec2 squareCenter= squareMin + glm::vec2(0.5f * (float)side);

	// A square wholly off the frame would feed the model a black image
	{
		glm::vec2 imageMin= applyAffine(imageFromRotated, squareMin);
		glm::vec2 imageMax= imageMin;
		const glm::vec2 corners[3]= {
			squareMin + glm::vec2((float)side, 0.f),
			squareMin + glm::vec2(0.f, (float)side),
			squareMin + glm::vec2((float)side)};
		for (const glm::vec2& corner : corners)
		{
			const glm::vec2 p= applyAffine(imageFromRotated, corner);
			imageMin= glm::min(imageMin, p);
			imageMax= glm::max(imageMax, p);
		}
		if (imageMax.x <= 0.f || imageMax.y <= 0.f || imageMin.x >= (float)bgrFrame.cols ||
			imageMin.y >= (float)bgrFrame.rows)
			return;
	}

	// 3) one warp from the frame into the square at the sampling resolution,
	// black outside the frame. Output pixel u samples the rotated frame at
	// squareMin + (u + 0.5) * step - 0.5, the pixel-center mapping a resize
	// uses, composed with the inverse rotation.
	const int supersample= std::clamp((side + kHandInputSize - 1) / kHandInputSize, 1, kMaxSupersample);
	const int sampleSize= kHandInputSize * supersample;
	const double step= (double)side / (double)sampleSize;
	const double* inv0= imageFromRotated.ptr<double>(0);
	const double* inv1= imageFromRotated.ptr<double>(1);
	const double originX= (double)squareMin.x + 0.5 * step - 0.5;
	const double originY= (double)squareMin.y + 0.5 * step - 0.5;
	const cv::Matx23d imageFromSample(
		inv0[0] * step, inv0[1] * step, inv0[0] * originX + inv0[1] * originY + inv0[2],
		inv1[0] * step, inv1[1] * step, inv1[0] * originX + inv1[1] * originY + inv1[2]);
	cv::warpAffine(
		bgrFrame, m_sampledMat, imageFromSample, cv::Size(sampleSize, sampleSize),
		cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

	const cv::Mat* modelSizedMat= &m_sampledMat;
	if (supersample > 1)
	{
		cv::resize(m_sampledMat, m_resizedMat, cv::Size(kHandInputSize, kHandInputSize), 0.0, 0.0, cv::INTER_AREA);
		modelSizedMat= &m_resizedMat;
	}
	cv::cvtColor(*modelSizedMat, m_rgbMat, cv::COLOR_BGR2RGB);

	cv::Mat inputMat(kHandInputSize, kHandInputSize, CV_32FC3, m_inputBuffer.data());
	m_rgbMat.convertTo(inputMat, CV_32FC3, 1.0 / 255.0);

	const std::array<int64_t, 4> inputShape= {1, kHandInputSize, kHandInputSize, 3};
	Ort::Value inputTensor= Ort::Value::CreateTensor<float>(
		OnnxSession::getCpuMemoryInfo(),
		m_inputBuffer.data(), m_inputBuffer.size(),
		inputShape.data(), inputShape.size());

	std::vector<Ort::Value> outputs= m_session.run(&inputTensor, 1);

	// -- Postprocess (mp_handpose.py::_postprocess) --
	const float* landmarkData= outputs[m_landmarkOutputIndex].GetTensorData<float>();
	const float* worldData= outputs[m_worldLandmarkOutputIndex].GetTensorData<float>();
	outResult.confidence= outputs[m_confidenceOutputIndex].GetTensorData<float>()[0];
	outResult.handedness= outputs[m_handednessOutputIndex].GetTensorData<float>()[0];

	// scale from 224-crop space back to rotated-frame pixels
	const float scaleFactor= (float)side / (float)kHandInputSize;

	// rotation-only part of the inverse rotation, for directions
	auto deRotate= [inv0, inv1](const glm::vec2& p) -> glm::vec2 {
		return glm::vec2(
			(float)(inv0[0] * p.x + inv0[1] * p.y),
			(float)(inv1[0] * p.x + inv1[1] * p.y));
	};

	glm::vec2 lmMin(0.f), lmMax(0.f);
	for (int i= 0; i < HAND_LANDMARK_COUNT; ++i)
	{
		// crop space -> rotated frame about the square center -> frame
		const glm::vec2 rotatedPos=
			squareCenter +
			glm::vec2(
				landmarkData[i * 3 + 0] - (float)kHandInputSize * 0.5f,
				landmarkData[i * 3 + 1] - (float)kHandInputSize * 0.5f) *
				scaleFactor;
		const float z= landmarkData[i * 3 + 2] * scaleFactor;

		const glm::vec2 imagePos= applyAffine(imageFromRotated, rotatedPos);
		outResult.imagePoints[i]= glm::vec3(imagePos, z);

		// world landmarks: de-rotate x/y, keep z (meters, hand-centered)
		const glm::vec2 worldXy= deRotate(glm::vec2(worldData[i * 3 + 0], worldData[i * 3 + 1]));
		outResult.modelPoints[i]= glm::vec3(worldXy, worldData[i * 3 + 2]);

		if (i == 0)
		{
			lmMin= imagePos;
			lmMax= imagePos;
		}
		else
		{
			lmMin= glm::min(lmMin, imagePos);
			lmMax= glm::max(lmMax, imagePos);
		}
	}

	// hand bounding box: landmark bbox shifted [0,-0.1], enlarged 1.65
	{
		const glm::vec2 wh= lmMax - lmMin;
		const glm::vec2 shift= glm::vec2(0.f, kHandBoxShiftY) * wh;
		const glm::vec2 boxMinShifted= lmMin + shift;
		const glm::vec2 boxMaxShifted= lmMax + shift;
		const glm::vec2 center= (boxMinShifted + boxMaxShifted) * 0.5f;
		const glm::vec2 halfSize= (boxMaxShifted - boxMinShifted) * kHandBoxEnlargeFactor * 0.5f;
		outResult.handBoxMin= center - halfSize;
		outResult.handBoxMax= center + halfSize;
	}

	// debug box: the sampled square mapped back to full-frame pixels
	{
		const float s= (float)side;
		const glm::vec2 corners[4]= {
			squareMin,
			squareMin + glm::vec2(0.f, s),
			squareMin + glm::vec2(s),
			squareMin + glm::vec2(s, 0.f)};
		for (int i= 0; i < 4; ++i)
			outResult.usedRoi.corners[i]= applyAffine(imageFromRotated, corners[i]);
		outResult.usedRoi.rotationRadians= radians;
		outResult.usedRoi.score= outResult.confidence;
	}

	outResult.valid= true;
}
