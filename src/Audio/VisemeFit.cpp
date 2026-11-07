#include "VisemeFit.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "MouthChain.h"
#include "MouthWindow.h"

namespace
{
constexpr int N= MFCC_COEFF_COUNT;
using Matrix= std::array<std::array<double, N>, N>;

// Inverts a symmetric positive definite matrix through its Cholesky factor;
// false when the matrix is not positive definite
bool invertSpd(const Matrix& a, Matrix& outInverse)
{
	// a = L L^T, L lower triangular
	Matrix l{};
	for (int row= 0; row < N; ++row)
	{
		for (int column= 0; column <= row; ++column)
		{
			double sum= a[row][column];
			for (int k= 0; k < column; ++k)
				sum-= l[row][k] * l[column][k];
			if (row == column)
			{
				if (sum <= 0.0)
					return false;
				l[row][row]= std::sqrt(sum);
			}
			else
			{
				l[row][column]= sum / l[column][column];
			}
		}
	}

	// Each column of the inverse solves L L^T x = e_column
	for (int column= 0; column < N; ++column)
	{
		std::array<double, N> y{};
		for (int row= 0; row < N; ++row)
		{
			double sum= row == column ? 1.0 : 0.0;
			for (int k= 0; k < row; ++k)
				sum-= l[row][k] * y[k];
			y[row]= sum / l[row][row];
		}
		for (int row= N - 1; row >= 0; --row)
		{
			double sum= y[row];
			for (int k= row + 1; k < N; ++k)
				sum-= l[k][row] * outInverse[k][column];
			outInverse[row][column]= sum / l[row][row];
		}
	}
	return true;
}
} // namespace

void extractSpeechVectors(const float* samples, size_t sampleCount, const VadGateConfig& gateConfig,
						  std::vector<MfccVector>& outVectors)
{
	outVectors.clear();
	if (sampleCount < (size_t)MFCC_BLOCK_SAMPLES)
		return;

	std::vector<float> preEmphasized(sampleCount);
	MfccExtractor::preEmphasize(samples, sampleCount, preEmphasized.data());

	MfccExtractor extractor;
	VadGate gate;
	gate.configure(gateConfig);
	const size_t hopCount= 1 + (sampleCount - MFCC_BLOCK_SAMPLES) / MFCC_HOP_SAMPLES;
	MfccVector vector;
	float logEnergy= 0.f;
	for (size_t hop= 0; hop < hopCount; ++hop)
	{
		extractor.compute(preEmphasized.data() + hop * MFCC_HOP_SAMPLES, logEnergy, vector);
		if (!gate.process(logEnergy))
			continue;
		MfccExtractor::compress(vector);
		outVectors.push_back(vector);
	}
}

bool fitVisemePrototype(const std::vector<MfccVector>& vectors, int viseme, const std::string& phoneme,
						VisemePrototype& outPrototype, std::string& outError)
{
	if ((int)vectors.size() < VISEME_FIT_MIN_VECTORS)
	{
		outError= "too few vectors to fit: " + std::to_string(vectors.size()) + " of " +
				  std::to_string(VISEME_FIT_MIN_VECTORS) + " needed";
		return false;
	}

	std::array<double, N> mean{};
	for (const MfccVector& vector : vectors)
		for (int index= 0; index < N; ++index)
			mean[index]+= vector[index];
	for (double& value : mean)
		value/= (double)vectors.size();

	Matrix covariance{};
	for (const MfccVector& vector : vectors)
	{
		std::array<double, N> diff;
		for (int index= 0; index < N; ++index)
			diff[index]= vector[index] - mean[index];
		for (int row= 0; row < N; ++row)
			for (int column= 0; column <= row; ++column)
				covariance[row][column]+= diff[row] * diff[column];
	}
	for (int row= 0; row < N; ++row)
	{
		for (int column= 0; column <= row; ++column)
		{
			covariance[row][column]/= (double)(vectors.size() - 1);
			covariance[column][row]= covariance[row][column];
		}
		covariance[row][row]+= VISEME_FIT_VARIANCE_FLOOR;
	}

	Matrix inverse{};
	if (!invertSpd(covariance, inverse))
	{
		outError= "the covariance does not invert";
		return false;
	}

	outPrototype= VisemePrototype();
	outPrototype.phoneme= phoneme;
	outPrototype.group= VISEME_FIT_GROUP;
	outPrototype.viseme= (uint8_t)viseme;
	for (int index= 0; index < N; ++index)
		outPrototype.mu[index]= (float)mean[index];
	int packed= 0;
	for (int row= 0; row < N; ++row)
		for (int column= 0; column <= row; ++column)
			outPrototype.sigmaInvLower[packed++]= (float)inverse[row][column];
	outError.clear();
	return true;
}

VisemeModel overlayVisemePrototypes(const VisemeModel& base, const std::vector<VisemePrototype>& fitted)
{
	VisemeModel overlay;
	for (const VisemePrototype& prototype : base.prototypes)
	{
		const bool bReplaced= std::any_of(fitted.begin(), fitted.end(), [&](const VisemePrototype& candidate) {
			return candidate.viseme == prototype.viseme;
		});
		if (!bReplaced)
			overlay.prototypes.push_back(prototype);
	}
	overlay.prototypes.insert(overlay.prototypes.end(), fitted.begin(), fitted.end());
	return overlay;
}

float suggestGateDbfs(const float* samples, size_t sampleCount, float gainDb)
{
	constexpr float k_marginDb= 6.f;
	const size_t hopSamples= (size_t)MouthWindowConfig().hopSamples;
	float loudestDbfs= -100.f;
	for (size_t start= 0; start + hopSamples <= sampleCount; start+= hopSamples)
		loudestDbfs= std::max(loudestDbfs, audioLevelDbfs(samples + start, hopSamples));
	if (loudestDbfs <= -100.f && sampleCount > 0)
		loudestDbfs= audioLevelDbfs(samples, sampleCount);
	return std::clamp(loudestDbfs + gainDb + k_marginDb, -80.f, 0.f);
}
