#include "VisemeTypes.h"

namespace
{
const char* const k_visemeNames[VISEME_COUNT]= {"aa", "E",  "I",  "O",  "U",  "PP", "SS", "TH",
												"DD", "FF", "kk", "nn", "RR", "CH", "sil"};
} // namespace

const char* visemeName(int viseme)
{
	return viseme >= 0 && viseme < VISEME_COUNT ? k_visemeNames[viseme] : "";
}
