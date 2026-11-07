#pragma once

#include <array>

// The 15 Oculus visemes in the stage's id order (which is not the Oculus
// SDK's), the output shape of the mfcc backend
constexpr int VISEME_COUNT= 15;

enum class eViseme : int
{
	aa= 0,
	E= 1,
	I= 2,
	O= 3,
	U= 4,
	PP= 5,
	SS= 6,
	TH= 7,
	DD= 8,
	FF= 9,
	kk= 10,
	nn= 11,
	RR= 12,
	CH= 13,
	sil= 14,
};

constexpr int VISEME_SIL= (int)eViseme::sil;

// The Oculus name of a viseme id ("aa", "PP", "sil"); "" out of range
const char* visemeName(int viseme);

// One weight per viseme, 0..1
using VisemeWeights= std::array<float, VISEME_COUNT>;
