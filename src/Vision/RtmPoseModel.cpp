#include "RtmPoseModel.h"

#include <algorithm>
#include <cmath>

#include "opencv2/imgproc.hpp"

#include "Logger.h"

static constexpr float kSimccSplitRatio= 2.f;
static constexpr float kBoxPadding= 1.25f;
// SimCC3DLabel's z_range: the depth axis spans +/- this many meters about
// the hip midpoint
static constexpr float kDepthRangeMeters= 2.1744869f;

// ImageNet statistics, per the models' own deploy pipelines
static const cv::Scalar k_mean(123.675, 116.28, 103.53);
static const cv::Scalar k_std(58.395, 57.12, 57.375);

bool RtmPoseModel::load(const std::string& modelPath, const std::string& preferredEp)
{
	if (!m_session.create(modelPath, preferredEp))
		return false;

	// Sizes come from the model, not from a variant table: input [1,3,H,W],
	// and per axis an output [1,K,bins] whose bin count is the axis's
	// extent times the split ratio. The planar axes are identified by their
	// bin counts and the depth axis is whichever output remains (its bin
	// count is free to equal the width's, as RTMW3D's does).
	const std::vector<int64_t>& inputShape= m_session.getInputShape(0);
	if (inputShape.size() != 4 || inputShape[1] != 3 || inputShape[2] <= 0 || inputShape[3] <= 0)
	{
		MIKAN_MT_LOG_ERROR("RtmPoseModel::load") << "Unexpected input shape in " << modelPath;
		return false;
	}
	m_inputHeight= (int)inputShape[2];
	m_inputWidth= (int)inputShape[3];

	const int64_t simccX= (int64_t)(m_inputWidth * kSimccSplitRatio);
	const int64_t simccY= (int64_t)(m_inputHeight * kSimccSplitRatio);
	m_simccXOutputIndex= m_session.findOutputByLastDim(simccX);
	m_simccYOutputIndex= m_session.findOutputByLastDim(simccY);
	// A square input gives both planar axes one bin count, and the export
	// orders them x then y
	if (simccX == simccY && m_simccXOutputIndex >= 0)
	{
		m_simccYOutputIndex= -1;
		for (size_t i= (size_t)m_simccXOutputIndex + 1; i < m_session.getOutputCount(); ++i)
		{
			const std::vector<int64_t>& shape= m_session.getOutputShape(i);
			if (!shape.empty() && shape.back() == simccY)
			{
				m_simccYOutputIndex= (int)i;
				break;
			}
		}
	}
	m_simccZOutputIndex= -1;
	m_depthBins= 0;
	for (size_t i= 0; i < m_session.getOutputCount(); ++i)
	{
		if ((int)i == m_simccXOutputIndex || (int)i == m_simccYOutputIndex)
			continue;
		const std::vector<int64_t>& shape= m_session.getOutputShape(i);
		if (shape.size() == 3 && shape.back() > 0)
		{
			m_simccZOutputIndex= (int)i;
			m_depthBins= (int)((float)shape.back() / kSimccSplitRatio);
			break;
		}
	}

	if (m_simccXOutputIndex < 0 || m_simccYOutputIndex < 0 || m_simccXOutputIndex == m_simccYOutputIndex)
	{
		MIKAN_MT_LOG_ERROR("RtmPoseModel::load") << "Unexpected output shapes in " << modelPath;
		return false;
	}

	// The MMDeploy export leaves the keypoint dimension dynamic (-1); the
	// torch export states it. A dynamic count is read off the first run's
	// output tensor instead.
	const std::vector<int64_t>& xShape= m_session.getOutputShape((size_t)m_simccXOutputIndex);
	m_keypointCount= xShape.size() == 3 && xShape[1] > 0 ? (int)xShape[1] : 0;
	if (m_keypointCount > RTM_MAX_KEYPOINT_COUNT)
	{
		MIKAN_MT_LOG_ERROR("RtmPoseModel::load")
			<< "Unsupported keypoint count " << m_keypointCount << " in " << modelPath;
		return false;
	}

	m_inputBuffer.resize((size_t)m_inputWidth * m_inputHeight * 3);
	const size_t planeSize= (size_t)m_inputWidth * m_inputHeight;
	for (int c= 0; c < 3; ++c)
	{
		m_channelPlanes[c]=
			cv::Mat(m_inputHeight, m_inputWidth, CV_32FC1, m_inputBuffer.data() + planeSize * c);
	}

	return true;
}

static int argmax(const float* row, int count, float& outPeak)
{
	int best= 0;
	outPeak= row[0];
	for (int i= 1; i < count; ++i)
	{
		if (row[i] > outPeak)
		{
			outPeak= row[i];
			best= i;
		}
	}
	return best;
}

void RtmPoseModel::estimate(const cv::Mat& bgrFrame, const glm::vec2& boxMin, const glm::vec2& boxMax,
							RtmPoseResult& outResult)
{
	outResult= RtmPoseResult();
	if (!m_session.isValid() || bgrFrame.empty())
		return;

	// -- Box -> center/scale, aspect-corrected then padded (the model's
	// TopDownGetBboxCenterScale) --
	const glm::vec2 center= (boxMin + boxMax) * 0.5f;
	float boxWidth= boxMax.x - boxMin.x;
	float boxHeight= boxMax.y - boxMin.y;
	if (boxWidth < 2.f || boxHeight < 2.f)
		return;

	const float aspect= (float)m_inputWidth / (float)m_inputHeight;
	if (boxWidth > boxHeight * aspect)
		boxHeight= boxWidth / aspect;
	else
		boxWidth= boxHeight * aspect;
	boxWidth*= kBoxPadding;
	boxHeight*= kBoxPadding;

	// -- Affine to the input canvas (MMPose get_warp_matrix): the source
	// triangle is the box's center plus its "up" edge, so the crop keeps the
	// box upright and centered --
	const cv::Point2f srcDir(0.f, boxWidth * -0.5f);
	const cv::Point2f dstDir(0.f, (float)m_inputWidth * -0.5f);
	cv::Point2f src[3];
	cv::Point2f dst[3];
	src[0]= cv::Point2f(center.x, center.y);
	src[1]= src[0] + srcDir;
	src[2]= src[1] + cv::Point2f(-srcDir.y, srcDir.x);
	dst[0]= cv::Point2f(m_inputWidth * 0.5f, m_inputHeight * 0.5f);
	dst[1]= dst[0] + dstDir;
	dst[2]= dst[1] + cv::Point2f(-dstDir.y, dstDir.x);

	const cv::Mat warpMatrix= cv::getAffineTransform(src, dst);
	cv::warpAffine(bgrFrame, m_cropMat, warpMatrix, cv::Size(m_inputWidth, m_inputHeight), cv::INTER_LINEAR);

	// -- Normalize into the preallocated NCHW buffer --
	cv::cvtColor(m_cropMat, m_rgbMat, cv::COLOR_BGR2RGB);
	m_rgbMat.convertTo(m_rgbMat, CV_32FC3);
	cv::subtract(m_rgbMat, k_mean, m_rgbMat);
	cv::divide(m_rgbMat, k_std, m_rgbMat);
	cv::split(m_rgbMat, m_channelPlanes.data());

	const std::array<int64_t, 4> inputShape= {1, 3, m_inputHeight, m_inputWidth};
	Ort::Value inputTensor= Ort::Value::CreateTensor<float>(
		OnnxSession::getCpuMemoryInfo(),
		m_inputBuffer.data(), m_inputBuffer.size(),
		inputShape.data(), inputShape.size());

	std::vector<Ort::Value> outputs= m_session.run(&inputTensor, 1);
	if ((int)outputs.size() <= std::max(m_simccXOutputIndex, std::max(m_simccYOutputIndex, m_simccZOutputIndex)))
		return;

	if (m_keypointCount == 0)
	{
		const std::vector<int64_t> shape= outputs[m_simccXOutputIndex].GetTensorTypeAndShapeInfo().GetShape();
		const int count= shape.size() == 3 ? (int)shape[1] : 0;
		if (count <= 0 || count > RTM_MAX_KEYPOINT_COUNT)
		{
			MIKAN_MT_LOG_ERROR("RtmPoseModel::estimate") << "Unsupported keypoint count " << count;
			return;
		}
		m_keypointCount= count;
	}

	const int simccX= (int)(m_inputWidth * kSimccSplitRatio);
	const int simccY= (int)(m_inputHeight * kSimccSplitRatio);
	const int simccZ= (int)(m_depthBins * kSimccSplitRatio);
	const float* rowsX= outputs[m_simccXOutputIndex].GetTensorData<float>();
	const float* rowsY= outputs[m_simccYOutputIndex].GetTensorData<float>();
	const float* rowsZ= m_simccZOutputIndex >= 0 ? outputs[m_simccZOutputIndex].GetTensorData<float>() : nullptr;

	// -- Decode: per-axis argmax / split ratio, then back through the crop --
	cv::Mat inverseWarp;
	cv::invertAffineTransform(warpMatrix, inverseWarp);
	const double i00= inverseWarp.at<double>(0, 0), i01= inverseWarp.at<double>(0, 1),
				 i02= inverseWarp.at<double>(0, 2);
	const double i10= inverseWarp.at<double>(1, 0), i11= inverseWarp.at<double>(1, 1),
				 i12= inverseWarp.at<double>(1, 2);

	for (int keypoint= 0; keypoint < m_keypointCount; ++keypoint)
	{
		float peakX= 0.f;
		float peakY= 0.f;
		const int bestX= argmax(rowsX + (size_t)keypoint * simccX, simccX, peakX);
		const int bestY= argmax(rowsY + (size_t)keypoint * simccY, simccY, peakY);

		const double cropX= bestX / kSimccSplitRatio;
		const double cropY= bestY / kSimccSplitRatio;
		glm::vec3& point= outResult.points[keypoint];
		point.x= (float)(cropX * i00 + cropY * i01 + i02);
		point.y= (float)(cropX * i10 + cropY * i11 + i12);
		point.z= 0.f;
		// The weaker planar axis governs: a keypoint is only as located as
		// its worst coordinate
		outResult.scores[keypoint]= std::min(peakX, peakY);

		if (rowsZ != nullptr)
		{
			float peakZ= 0.f;
			const int bestZ= argmax(rowsZ + (size_t)keypoint * simccZ, simccZ, peakZ);
			const float depthBin= bestZ / kSimccSplitRatio;
			point.z= (depthBin / ((float)m_depthBins * 0.5f) - 1.f) * kDepthRangeMeters;
			outResult.depthScores[keypoint]= peakZ;
		}
	}

	outResult.keypointCount= m_keypointCount;
	outResult.hasDepth= rowsZ != nullptr;
	outResult.valid= true;
}
