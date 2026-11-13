#pragma once

#include <string>

#include "ExprInputTable.h"
#include "FaceSources.h"

// The raw input namespaces a face expression reads, as one flat slot table.
// This table is the single place a new source (a webcam face, a scripted
// input) is added: a name range here, its fill in fillFaceExprInputs.
//   phone.<arkitName>  0..51, ARKit order
//   phone.present      52
//   phone.tracked      53
//   viseme.<name>      54..68, the stage's viseme order
//   viseme.present     69
//   webcam.<arkitName> 70..121, ARKit order
//   webcam.present     122
//   webcam.tracked     123
//   blink.eyeBlinkLeft 124
//   blink.eyeBlinkRight 125
//   blink.present      126
//   blink.tracked      127
namespace FaceExprInputs
{
constexpr int k_phoneBlendshapeBase= 0;
constexpr int k_phonePresent= k_phoneBlendshapeBase + ARKIT_BLENDSHAPE_COUNT;
constexpr int k_phoneTracked= k_phonePresent + 1;
constexpr int k_visemeBase= k_phoneTracked + 1;
constexpr int k_visemePresent= k_visemeBase + VISEME_COUNT;
constexpr int k_webcamBlendshapeBase= k_visemePresent + 1;
constexpr int k_webcamPresent= k_webcamBlendshapeBase + ARKIT_BLENDSHAPE_COUNT;
constexpr int k_webcamTracked= k_webcamPresent + 1;
constexpr int k_blinkEyeBlinkLeft= k_webcamTracked + 1;
constexpr int k_blinkEyeBlinkRight= k_blinkEyeBlinkLeft + 1;
constexpr int k_blinkPresent= k_blinkEyeBlinkRight + 1;
constexpr int k_blinkTracked= k_blinkPresent + 1;
constexpr int k_count= k_blinkTracked + 1;

// The slot of a dotted name, compared without case ("phone.JawOpen",
// "phone.eyeBlink_L", "viseme.AA"); -1 for a name the table lacks
int fromName(const std::string& name);

// The canonical spelling of a slot ("phone.jawOpen"); "" out of range
std::string name(int slot);

// Writes every slot from the sources; outInputs holds k_count floats.
// Booleans are 0 and 1.
void fill(const FaceSources& sources, float* outInputs);

// This table as an ExprInputTable (fromName, name and k_count), the one a
// face profile compiles against
const ExprInputTable& table();
} // namespace FaceExprInputs
