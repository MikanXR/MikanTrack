#pragma once

#include <array>
#include <vector>

#include "FaceTypes.h"
#include "VisemeTypes.h"

// One term of a viseme's row: an ARKit mouth column and the weight the
// viseme drives it to at full strength
struct VisemeTerm
{
	int arkitIndex= -1;
	float weight= 0.f;
};

// The row of one viseme, the terms it drives
const std::vector<VisemeTerm>& visemeTableRow(int viseme);

// Writes a set of viseme weights onto a 52-wide ARKit face: each column the
// table drives is the sum of its terms' weights times their visemes'
// weights, clamped to one. The columns the table does not drive are left as
// they were.
void applyVisemeWeights(const VisemeWeights& weights, std::array<float, ARKIT_BLENDSHAPE_COUNT>& outFace);
