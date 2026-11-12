#pragma once

#include "BodyPoseTypes.h"

// The face shapes read off the whole-body model's 68 face landmarks, as
// distances over the face's own size so they hold across the user's distance
// from the camera. Raw measurements: no neutral, no mapping to 0..1, no
// smoothing. A pure function of one observation's face block.
//
// The face's size is the span of the outer eye corners. Its axes are the eye
// line (toward the person's left) and the perpendicular pointing from the
// mouth to the eyes, so a rolled head measures as an upright one. Sides are
// the person's own, 0 left and 1 right, as the block's are.
struct WholeBodyFaceRatios
{
	// False when the block is absent or the eye corners coincide
	bool valid= false;
	// The outer eye corner span, px
	float faceWidthPx= 0.f;
	// The lowest score among the points read, the gate a consumer sets
	float minScore= 0.f;

	// The eye aspect ratio: the lid gap over the eye's width. Falls as the
	// eye closes.
	float eyeOpenness[2]= {0.f, 0.f};
	// The inner lip gap over the face width
	float mouthOpen= 0.f;
	// The mouth corner span over the face width
	float mouthWidth= 0.f;
	// Each mouth corner above the lips' midpoint along the face's up axis,
	// over the face width. Rises with a smile.
	float mouthCornerLift[2]= {0.f, 0.f};
	// Each brow's midpoint above its eye's center, over the face width
	float browHeight[2]= {0.f, 0.f};
	// The inner brow ends above the inner eye corners, over the face width
	float browInnerHeight= 0.f;
};

WholeBodyFaceRatios computeWholeBodyFaceRatios(const WholeBodyExtras& extras);
