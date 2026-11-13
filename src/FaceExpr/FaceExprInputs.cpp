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

class FaceInputTable final : public ExprInputTable
{
public:
	int getCount() const override { return FaceExprInputs::k_count; }
	int findSlot(const std::string& name) const override { return FaceExprInputs::fromName(name); }
	std::string getName(int slot) const override { return FaceExprInputs::name(slot); }
};
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

	// The phone and the webcam carry the same ARKit columns and flags
	if (space == "phone" || space == "webcam")
	{
		const bool bPhone= space == "phone";
		if (member == "present")
			return bPhone ? k_phonePresent : k_webcamPresent;
		if (member == "tracked")
			return bPhone ? k_phoneTracked : k_webcamTracked;
		const std::string expanded= expandSideSuffix(member);
		for (int index= 0; index < ARKIT_BLENDSHAPE_COUNT; ++index)
		{
			if (expanded == toLower(arkitBlendshapeName(index)))
				return (bPhone ? k_phoneBlendshapeBase : k_webcamBlendshapeBase) + index;
		}
		return -1;
	}
	if (space == "blink")
	{
		if (member == "present")
			return k_blinkPresent;
		if (member == "tracked")
			return k_blinkTracked;
		const std::string expanded= expandSideSuffix(member);
		if (expanded == "eyeblinkleft")
			return k_blinkEyeBlinkLeft;
		if (expanded == "eyeblinkright")
			return k_blinkEyeBlinkRight;
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
	if (slot >= k_webcamBlendshapeBase && slot < k_webcamPresent)
		return std::string("webcam.") + arkitBlendshapeName(slot - k_webcamBlendshapeBase);
	if (slot == k_webcamPresent)
		return "webcam.present";
	if (slot == k_webcamTracked)
		return "webcam.tracked";
	if (slot == k_blinkEyeBlinkLeft)
		return "blink.eyeBlinkLeft";
	if (slot == k_blinkEyeBlinkRight)
		return "blink.eyeBlinkRight";
	if (slot == k_blinkPresent)
		return "blink.present";
	if (slot == k_blinkTracked)
		return "blink.tracked";
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
	for (int index= 0; index < ARKIT_BLENDSHAPE_COUNT; ++index)
	{
		const bool bMeasured= (sources.webcam.measuredMask >> index) & 1ull;
		outInputs[k_webcamBlendshapeBase + index]=
			sources.webcam.tracked && bMeasured ? sources.webcam.blendshapes[index] : 0.f;
	}
	outInputs[k_webcamPresent]= sources.webcam.present ? 1.f : 0.f;
	outInputs[k_webcamTracked]= sources.webcam.tracked ? 1.f : 0.f;
	outInputs[k_blinkEyeBlinkLeft]= sources.blink.tracked ? sources.blink.eyeBlinkLeft : 0.f;
	outInputs[k_blinkEyeBlinkRight]= sources.blink.tracked ? sources.blink.eyeBlinkRight : 0.f;
	outInputs[k_blinkPresent]= sources.blink.present ? 1.f : 0.f;
	outInputs[k_blinkTracked]= sources.blink.tracked ? 1.f : 0.f;
}

const ExprInputTable& table()
{
	static const FaceInputTable s_table;
	return s_table;
}
} // namespace FaceExprInputs
