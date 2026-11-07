#pragma once

#include <memory>
#include <vector>

#include "TrackingTypes.h"

class AvatarFaceMap;
struct AvatarModel;

// The loaded avatar's mouth expressions as its face map derives them from
// the output face: the vowel presets of a preset avatar, read from the
// frame's viseme weights while the microphone owns the mouth, or the
// mouth-named expressions of a perfect-sync avatar. The OSC Output panel's
// collapsible section, where a shape that looks wrong on the avatar is told
// from a weight that never arrived.
class AvatarMouthReadout
{
public:
	void draw(const std::shared_ptr<const AvatarModel>& model, const TrackingFrameResult::FacePose& face);

private:
	// The avatar's face map, rebuilt when the avatar changes
	std::shared_ptr<const AvatarModel> m_faceMapModel;
	std::shared_ptr<const AvatarFaceMap> m_faceMap;
	// The outputs the mouth drives, in face map order
	std::vector<int> m_mouthOutputs;
	std::vector<float> m_outputValues;
	std::vector<float> m_values;
};
