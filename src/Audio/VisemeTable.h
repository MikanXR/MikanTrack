#pragma once

#include <vector>

#include "AudioTypes.h"
#include "VisemeTypes.h"

// One term of a viseme's row: a mouth column and the weight the viseme
// drives it to at full strength
struct VisemeTerm
{
	int mouthColumn= -1;
	float weight= 0.f;
};

// The row of one viseme, the terms it drives
const std::vector<VisemeTerm>& visemeTableRow(int viseme);

// Writes a set of viseme weights onto a mouth frame: each column is the sum
// of its terms' weights times their visemes' weights, clamped to one. The
// frame's other columns are left as they were.
void applyVisemeWeights(const VisemeWeights& weights, MouthFrame& outFrame);
