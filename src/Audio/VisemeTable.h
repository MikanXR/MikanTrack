#pragma once

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

// The row of one viseme, the terms it drives. The table is input data for
// the built-in Auto face profile, which spreads the visemes over the ARKit
// mouth columns as ordinary expressions.
const std::vector<VisemeTerm>& visemeTableRow(int viseme);
