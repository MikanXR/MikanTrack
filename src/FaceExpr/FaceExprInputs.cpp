#include "FaceExprInputs.h"

#include <cctype>

namespace
{
std::string toLower(const std::string& text)
{
	std::string lower= text;
	for (char& character : lower)
		character= (char)std::tolower((unsigned char)character);
	return lower;
}

// The phone spells a side as a _L/_R suffix where ARKit spells it out
std::string expandSideSuffix(const std::string& lowerName)
{
	const size_t length= lowerName.size();
	if (length > 2 && lowerName[length - 2] == '_' && (lowerName[length - 1] == 'l' || lowerName[length - 1] == 'r'))
		return lowerName.substr(0, length - 2) + (lowerName[length - 1] == 'l' ? "left" : "right");
	return lowerName;
}
} // namespace

namespace FaceExprInputs
{
int fromName(const std::string& name)
{
	const std::string lower= toLower(name);
	const size_t dot= lower.find('.');
	if (dot == std::string::npos)
		return -1;
	const std::string space= lower.substr(0, dot);
	const std::string member= lower.substr(dot + 1);

	if (space == "phone")
	{
		if (member == "present")
			return k_phonePresent;
		if (member == "tracked")
			return k_phoneTracked;
		const std::string expanded= expandSideSuffix(member);
		for (int index= 0; index < ARKIT_BLENDSHAPE_COUNT; ++index)
		{
			if (expanded == toLower(arkitBlendshapeName(index)))
				return k_phoneBlendshapeBase + index;
		}
		return -1;
	}
	if (space == "viseme")
	{
		if (member == "present")
			return k_visemePresent;
		for (int index= 0; index < VISEME_COUNT; ++index)
		{
			if (member == toLower(visemeName(index)))
				return k_visemeBase + index;
		}
		return -1;
	}
	return -1;
}

std::string name(int slot)
{
	if (slot >= k_phoneBlendshapeBase && slot < k_phonePresent)
		return std::string("phone.") + arkitBlendshapeName(slot - k_phoneBlendshapeBase);
	if (slot == k_phonePresent)
		return "phone.present";
	if (slot == k_phoneTracked)
		return "phone.tracked";
	if (slot >= k_visemeBase && slot < k_visemePresent)
		return std::string("viseme.") + visemeName(slot - k_visemeBase);
	if (slot == k_visemePresent)
		return "viseme.present";
	return "";
}

void fill(const FaceSources& sources, float* outInputs)
{
	for (int index= 0; index < ARKIT_BLENDSHAPE_COUNT; ++index)
		outInputs[k_phoneBlendshapeBase + index]= sources.phone.tracked ? sources.phone.blendshapes[index] : 0.f;
	outInputs[k_phonePresent]= sources.phone.present ? 1.f : 0.f;
	outInputs[k_phoneTracked]= sources.phone.tracked ? 1.f : 0.f;
	for (int index= 0; index < VISEME_COUNT; ++index)
		outInputs[k_visemeBase + index]= sources.viseme.present ? sources.viseme.weights[index] : 0.f;
	outInputs[k_visemePresent]= sources.viseme.present ? 1.f : 0.f;
}
} // namespace FaceExprInputs
