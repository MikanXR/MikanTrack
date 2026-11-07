#include "VisemeTable.h"

#include <algorithm>
#include <array>
#include <cassert>

#include "FaceTypes.h"

namespace
{
struct NamedTerm
{
	const char* arkitName;
	float weight;
};

// The fixed map from the 15 Oculus visemes to the ARKit mouth columns: the
// jaw and lip shape of each sound. The vowels open the jaw and shape the
// lips, the consonants are the lip contact (PP), the lip under the teeth
// (FF), the tongue between them (TH), and the small openings of the rest.
// Silence drives nothing. Each vowel's lead column (the jaw for aa, the
// smile for I, the stretch for E, the funnel for O, the pucker for U) is the
// one the VRM preset map derives that vowel from, and it reaches 0.8 to
// 0.85 at a full viseme so a preset avatar gets a full shape; the secondary
// columns are the detail a perfect-sync avatar reads.
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

// The rows with their names resolved to mouth columns, built on first use
const std::array<std::vector<VisemeTerm>, VISEME_COUNT>& resolvedRows()
{
	static const std::array<std::vector<VisemeTerm>, VISEME_COUNT> rows= [] {
		std::array<std::vector<VisemeTerm>, VISEME_COUNT> result;
		for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
		{
			for (const NamedTerm& named : k_rows[viseme])
			{
				VisemeTerm term;
				term.mouthColumn= mouthColumnFromArkitIndex(arkitBlendshapeFromName(named.arkitName));
				term.weight= named.weight;
				assert(term.mouthColumn >= 0);
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

void applyVisemeWeights(const VisemeWeights& weights, MouthFrame& outFrame)
{
	std::array<float, MOUTH_COLUMN_COUNT> sums{};
	std::array<bool, MOUTH_COLUMN_COUNT> touched{};
	for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
	{
		for (const VisemeTerm& term : resolvedRows()[viseme])
		{
			if (term.mouthColumn < 0)
				continue;
			sums[term.mouthColumn]+= term.weight * weights[viseme];
			touched[term.mouthColumn]= true;
		}
	}
	for (int column= 0; column < MOUTH_COLUMN_COUNT; ++column)
	{
		if (touched[column])
			outFrame.weights[column]= std::min(std::max(sums[column], 0.f), 1.f);
	}
}
