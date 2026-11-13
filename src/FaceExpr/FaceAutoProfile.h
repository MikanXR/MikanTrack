#pragma once

#include <memory>
#include <string>

#include "FaceProfile.h"

struct AvatarModel;
class FaceProfileBinding;

// The id of the built-in Auto profile. It is generated per avatar, never
// stored in the library, and read-only: duplicating it copies it into the
// library under a fresh id.
constexpr const char* k_autoFaceProfileId= "auto";

bool isAutoFaceProfileId(const std::string& id);

// The Auto profile for an avatar, or for no avatar (model null):
// - Expressions named after ARKit blendshapes (perfect sync): those
//   expressions alone, one column each, in the avatar's own spelling.
// - Otherwise the VRM presets the avatar carries, each from the ARKit columns
//   that shape it (blinks from the blinks, the vowels from the jaw and mouth,
//   the gaze directions from the eye looks). The emotion presets have no
//   ARKit counterpart and get no output. Plus the ARKit names the avatar
//   carries as morph targets, as morph outputs.
// - No avatar: the 52 ARKit names as morph outputs.
// An ARKit column reads the phone while it sees a face and otherwise the
// visemes the viseme table spreads over it: select(phone.tracked,
// phone.<name>, <viseme terms>). A vowel preset reads its viseme directly.
// A column in webcamMask falls back to the webcam while the phone sees no
// face, inside a dead zone and a smoothing: select(phone.tracked,
// phone.<name>, <webcam term>). On a column with viseme terms the two combine
// with max, so either source alone opens the mouth. A column outside the mask
// keeps the rule above.
FaceProfile makeAutoFaceProfile(const AvatarModel* model);
FaceProfile makeAutoFaceProfile(const AvatarModel* model, uint64_t webcamMask);

// The webcam columns Auto reads: the columns the source measures less the
// blinks. The eye openness sits as low with the lids lowered as with them
// shut, so a blink read off it holds the avatar's eyes closed.
uint64_t autoFaceWebcamMask();

// The expression an ARKit column gets in the Auto profile
std::string makeAutoArkitExpr(int arkitIndex);
std::string makeAutoArkitExpr(int arkitIndex, uint64_t webcamMask);

// The Auto profile compiled and bound to model (null: no avatar)
std::shared_ptr<const FaceProfileBinding> makeAutoFaceBinding(const AvatarModel* model);
