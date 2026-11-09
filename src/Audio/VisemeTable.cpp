#include "VisemeTable.h"

#include <algorithm>
#include <array>
#include <cassert>

#include "AudioTypes.h"
#include "FaceTypes.h"

namespace
{
struct NamedTerm
{
	const char* arkitName;
	float weight;
};

// The fixed map from the 15 Oculus visemes to the ARKit mouth columns: the
// jaw and lip shape of each sound, which the Auto face profile gives an
// ARKit-named expression or morph target while the phone does not see a face.
// (The Auto profile's vowel presets read their visemes directly.) The
// vowels open the jaw and shape the lips, the consonants are the lip
// contact (PP), the lip under the teeth (FF), the tongue between them (TH),
// and the small openings of the rest. Silence drives nothing. Each vowel
// leads with its defining column (the jaw for aa, the smile for I, the
// stretch for E, the funnel for O, the pucker for U) at 0.8 to 0.85, so a
// full viseme is a full shape.
const std::vector<NamedTerm> k_rows[VISEME_COUNT]= {
	// aa
	{{"jawOpen", 0.85f}, {"mouthLowerDownLeft", 0.25f}, {"mouthLowerDownRight", 0.25f}},
	// E
	{{"jawOpen", 0.35f}, {"mouthStretchLeft", 0.8f}, {"mouthStretchRight", 0.8f}},
	// I
	{{"jawOpen", 0.15f}, {"mouthSmileLeft", 0.8f}, {"mouthSmileRight", 0.8f}, {"mouthStretchLeft", 0.3f},
	 {"mouthStretchRight", 0.3f}},
	// O
	{{"jawOpen", 0.5f}, {"mouthFunnel", 0.8f}, {"mouthPucker", 0.2f}},
	// U
	{{"jawOpen", 0.1f}, {"mouthPucker", 0.85f}, {"mouthFunnel", 0.3f}},
	// PP
	{{"mouthClose", 0.6f}, {"mouthPressLeft", 0.5f}, {"mouthPressRight", 0.5f}},
	// SS
	{{"jawOpen", 0.08f}, {"mouthStretchLeft", 0.25f}, {"mouthStretchRight", 0.25f}, {"mouthLowerDownLeft", 0.15f},
	 {"mouthLowerDownRight", 0.15f}},
	// TH
	{{"jawOpen", 0.15f}, {"tongueOut", 0.4f}, {"mouthLowerDownLeft", 0.1f}, {"mouthLowerDownRight", 0.1f}},
	// DD
	{{"jawOpen", 0.15f}, {"mouthLowerDownLeft", 0.15f}, {"mouthLowerDownRight", 0.15f}, {"mouthUpperUpLeft", 0.1f},
	 {"mouthUpperUpRight", 0.1f}},
	// FF
	{{"jawOpen", 0.05f}, {"mouthRollLower", 0.5f}, {"mouthPressLeft", 0.2f}, {"mouthPressRight", 0.2f},
	 {"mouthUpperUpLeft", 0.15f}, {"mouthUpperUpRight", 0.15f}},
	// kk
	{{"jawOpen", 0.2f}, {"mouthLowerDownLeft", 0.1f}, {"mouthLowerDownRight", 0.1f}},
	// nn
	{{"jawOpen", 0.12f}, {"mouthUpperUpLeft", 0.1f}, {"mouthUpperUpRight", 0.1f}},
	// RR
	{{"jawOpen", 0.15f}, {"mouthPucker", 0.35f}, {"mouthFunnel", 0.2f}},
	// CH
	{{"jawOpen", 0.15f}, {"mouthFunnel", 0.35f}, {"mouthPucker", 0.25f}, {"mouthUpperUpLeft", 0.1f},
	 {"mouthUpperUpRight", 0.1f}},
	// sil
	{},
};

// The rows with their names resolved to ARKit columns, built on first use
const std::array<std::vector<VisemeTerm>, VISEME_COUNT>& resolvedRows()
{
	static const std::array<std::vector<VisemeTerm>, VISEME_COUNT> rows= [] {
		std::array<std::vector<VisemeTerm>, VISEME_COUNT> result;
		for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		{
			for (const NamedTerm& named : k_rows[viseme])
			{
				VisemeTerm term;
				term.arkitIndex= arkitBlendshapeFromName(named.arkitName);
				term.weight= named.weight;
				assert(isMouthArkitIndex(term.arkitIndex));
				result[viseme].push_back(term);
			}
		}
		return result;
	}();
	return rows;
}
} // namespace

const std::vector<VisemeTerm>& visemeTableRow(int viseme)
{
	static const std::vector<VisemeTerm> empty;
	return viseme >= 0 && viseme < VISEME_COUNT ? resolvedRows()[viseme] : empty;
}
