#pragma once

#include <memory>
#include <vector>

#include "TrackingTypes.h"

class FaceProfileBinding;
struct AvatarModel;

// The loaded avatar's mouth targets as the active face profile drives them:
// the targets with an output reading a mouth input (the phone's mouth
// columns or a viseme). The OSC Output panel's collapsible section, where a
// shape that looks wrong on the avatar is told from a weight that never
// arrived. A filtered view of the Avatar panel's Face tab.
class AvatarMouthReadout
{
public:
	void draw(const std::shared_ptr<const AvatarModel>& model, const std::shared_ptr<const FaceProfileBinding>& binding,
			  const FaceSources& face);

private:
	// The binding the target list was built from
	std::shared_ptr<const FaceProfileBinding> m_binding;
	// The targets the mouth drives, in binding order
	std::vector<int> m_mouthTargets;
	// Whether those targets read a phone slot, a viseme slot
	bool m_bReadsPhone= false;
	bool m_bReadsVisemes= false;
	std::vector<float> m_values;
};
