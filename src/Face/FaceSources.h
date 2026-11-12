#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "FaceTypes.h"
#include "VisemeTypes.h"

// The raw face sources of one output frame, side by side, as the face
// expressions read them. A source's values are zero unless it is tracked, so
// an expression without a guard relaxes instead of freezing on the last raw
// values.
struct FaceSources
{
	// The phone face stream (iFacialMocap)
	struct Phone
	{
		// The stream is live
		bool present= false;
		// present, and the phone sees a face
		bool tracked= false;
		// Raw, in ARKit order; all zero unless tracked
		std::array<float, ARKIT_BLENDSHAPE_COUNT> blendshapes{};
	};
	// The microphone's mouth stage
	struct Viseme
	{
		// The sample is fresh
		bool present= false;
		// Post-filter; all zero unless present
		VisemeWeights weights{};
	};
	// The direct camera's whole-body face block as ARKit columns
	struct Webcam
	{
		// The direct camera's last whole-body observation is fresh
		bool present= false;
		// present, and the face block's scores clear the gate
		bool tracked= false;
		// Raw, in ARKit order; all zero unless tracked
		std::array<float, ARKIT_BLENDSHAPE_COUNT> blendshapes{};
		// A bit per column the source measures. A column outside it reads zero.
		uint64_t measuredMask= 0;
	};
	// The face profile evaluated over the sources for this frame. Empty until
	// a profile has run on the frame.
	struct Outputs
	{
		// The FaceProfileBinding generation that produced the values; 0 = none
		uint64_t generation= 0;
		// One per profile output, clamped to 0..1
		std::vector<float> outputValues;
		// One per binding target: the max of the outputs sharing it
		std::vector<float> targetValues;
		// One per profile let, raw, for the inspector
		std::vector<float> letValues;
	};

	Phone phone;
	Viseme viseme;
	Webcam webcam;
	Outputs outputs;

	// The face is present while any source is
	bool isPresent() const { return phone.present || viseme.present || webcam.present; }
};
