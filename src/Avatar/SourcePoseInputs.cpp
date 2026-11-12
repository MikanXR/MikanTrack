#include "SourcePoseInputs.h"

#include <cctype>
#include <vector>

namespace
{
using eGroupState= SourcePose::eGroupState;

// The group names in slot order
const char* const k_groupNames[SourcePoseInputs::k_groupCount]= {"torso",   "head",     "leftArm",
																 "rightArm", "leftHand", "rightHand"};

std::vector<std::string> makeNames()
{
	std::vector<std::string> names(SourcePoseInputs::k_count);
	names[SourcePoseInputs::k_headConfidence]= "source.head.confidence";
	names[SourcePoseInputs::k_upperArmConfidence + 0]= "source.leftUpperArm.confidence";
	names[SourcePoseInputs::k_upperArmConfidence + 1]= "source.rightUpperArm.confidence";
	names[SourcePoseInputs::k_lowerArmConfidence + 0]= "source.leftLowerArm.confidence";
	names[SourcePoseInputs::k_lowerArmConfidence + 1]= "source.rightLowerArm.confidence";
	names[SourcePoseInputs::k_handConfidence + 0]= "source.leftHand.confidence";
	names[SourcePoseInputs::k_handConfidence + 1]= "source.rightHand.confidence";
	for (int group= 0; group < SourcePoseInputs::k_groupCount; ++group)
	{
		const std::string prefix= std::string("source.") + k_groupNames[group];
		names[SourcePoseInputs::k_groupStateBase + 2 * group]= prefix + ".measured";
		names[SourcePoseInputs::k_groupStateBase + 2 * group + 1]= prefix + ".carried";
	}
	return names;
}

float confidenceValue(const SourcePose::Confidence& confidence)
{
	return confidence.bPresent ? confidence.value : 0.f;
}

void writeGroupState(eGroupState state, int group, float* outInputs)
{
	outInputs[SourcePoseInputs::k_groupStateBase + 2 * group]= state == eGroupState::Measured ? 1.f : 0.f;
	outInputs[SourcePoseInputs::k_groupStateBase + 2 * group + 1]= state != eGroupState::Lost ? 1.f : 0.f;
}

bool isNameStart(char character)
{
	return std::isalpha((unsigned char)character) || character == '_';
}

bool isNameChar(char character)
{
	return std::isalnum((unsigned char)character) || character == '_';
}

// Whether text at position starts with word, compared without case
bool startsWithIgnoringCase(const std::string& text, size_t position, const char* word)
{
	for (size_t index= 0; word[index] != '\0'; ++index)
	{
		if (position + index >= text.size() ||
			std::tolower((unsigned char)text[position + index]) != std::tolower((unsigned char)word[index]))
		{
			return false;
		}
	}
	return true;
}

// side spelled with the first letter's case of the word it replaces
std::string spellLike(const char* side, char firstLetter)
{
	std::string spelled= side;
	spelled[0]= std::isupper((unsigned char)firstLetter) ? (char)std::toupper((unsigned char)spelled[0]) : spelled[0];
	return spelled;
}
} // namespace

namespace SourcePoseInputs
{
const ExprInputTable& table()
{
	static const ExprNameTable s_table(makeNames());
	return s_table;
}

void fill(const SourcePose& source, float* outInputs)
{
	outInputs[k_headConfidence]= confidenceValue(source.head.confidence);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		outInputs[k_upperArmConfidence + sideIndex]= confidenceValue(source.arms[sideIndex].shoulderConfidence);
		outInputs[k_lowerArmConfidence + sideIndex]= confidenceValue(source.arms[sideIndex].forearmConfidence);
		outInputs[k_handConfidence + sideIndex]= confidenceValue(source.hands[sideIndex].confidence);
	}
	writeGroupState(source.torso.state, 0, outInputs);
	writeGroupState(source.head.state, 1, outInputs);
	writeGroupState(source.arms[0].state, 2, outInputs);
	writeGroupState(source.arms[1].state, 3, outInputs);
	writeGroupState(source.hands[0].state, 4, outInputs);
	writeGroupState(source.hands[1].state, 5, outInputs);
}

std::string swapSides(const std::string& expression)
{
	std::string out;
	out.reserve(expression.size() + 8);
	size_t position= 0;
	while (position < expression.size())
	{
		const char character= expression[position];
		if (std::isdigit((unsigned char)character) ||
			(character == '.' && position + 1 < expression.size() && std::isdigit((unsigned char)expression[position + 1])))
		{
			// A number, which may hold a dot and an exponent's letter
			size_t end= position;
			while (end < expression.size() && (isNameChar(expression[end]) || expression[end] == '.'))
				++end;
			out.append(expression, position, end - position);
			position= end;
			continue;
		}
		if (!isNameStart(character))
		{
			out.push_back(character);
			++position;
			continue;
		}

		// A dotted name: segments joined by a dot before a name start
		size_t end= position;
		for (;;)
		{
			while (end < expression.size() && isNameChar(expression[end]))
				++end;
			if (end + 1 < expression.size() && expression[end] == '.' && isNameStart(expression[end + 1]))
				++end;
			else
				break;
		}
		const std::string name= expression.substr(position, end - position);
		const size_t sideAt= 7; // after "source."
		if (startsWithIgnoringCase(name, 0, "source.") && startsWithIgnoringCase(name, sideAt, "left"))
			out+= name.substr(0, sideAt) + spellLike("right", name[sideAt]) + name.substr(sideAt + 4);
		else if (startsWithIgnoringCase(name, 0, "source.") && startsWithIgnoringCase(name, sideAt, "right"))
			out+= name.substr(0, sideAt) + spellLike("left", name[sideAt]) + name.substr(sideAt + 5);
		else
			out+= name;
		position= end;
	}
	return out;
}
} // namespace SourcePoseInputs
