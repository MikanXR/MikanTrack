#include "VisemeModel.h"

#include <cstring>
#include <fstream>
#include <limits>

#include "PathUtils.h"

namespace
{
const char* const k_visemeNames[VISEME_COUNT]= {"aa", "E",  "I",  "O",  "U",  "PP", "SS", "TH",
												"DD", "FF", "kk", "nn", "RR", "CH", "sil"};

// A code point as UTF-8
void appendCodePoint(std::string& out, uint32_t codePoint)
{
	if (codePoint < 0x80)
	{
		out.push_back((char)codePoint);
	}
	else if (codePoint < 0x800)
	{
		out.push_back((char)(0xC0 | (codePoint >> 6)));
		out.push_back((char)(0x80 | (codePoint & 0x3F)));
	}
	else
	{
		out.push_back((char)(0xE0 | (codePoint >> 12)));
		out.push_back((char)(0x80 | ((codePoint >> 6) & 0x3F)));
		out.push_back((char)(0x80 | (codePoint & 0x3F)));
	}
}

// The first two code points of a UTF-8 string, zero where there are fewer;
// a code point past 16 bits does not fit the header and reads as zero
void decodeCodePoints(const std::string& utf8, uint32_t outCodePoints[2])
{
	outCodePoints[0]= 0;
	outCodePoints[1]= 0;
	int found= 0;
	size_t index= 0;
	while (index < utf8.size() && found < 2)
	{
		const uint8_t lead= (uint8_t)utf8[index];
		int length= 1;
		uint32_t codePoint= lead;
		if (lead >= 0xF0)
		{
			length= 4;
			codePoint= lead & 0x07;
		}
		else if (lead >= 0xE0)
		{
			length= 3;
			codePoint= lead & 0x0F;
		}
		else if (lead >= 0xC0)
		{
			length= 2;
			codePoint= lead & 0x1F;
		}
		for (int offset= 1; offset < length && index + offset < utf8.size(); ++offset)
			codePoint= (codePoint << 6) | ((uint8_t)utf8[index + offset] & 0x3F);
		outCodePoints[found++]= codePoint <= 0xFFFF ? codePoint : 0;
		index+= length;
	}
}

float readFloatLe(const uint8_t* bytes)
{
	uint32_t bits= (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) |
				   ((uint32_t)bytes[3] << 24);
	float value;
	std::memcpy(&value, &bits, sizeof(value));
	return value;
}

void writeFloatLe(uint8_t* bytes, float value)
{
	uint32_t bits;
	std::memcpy(&bits, &value, sizeof(bits));
	bytes[0]= (uint8_t)bits;
	bytes[1]= (uint8_t)(bits >> 8);
	bytes[2]= (uint8_t)(bits >> 16);
	bytes[3]= (uint8_t)(bits >> 24);
}
} // namespace

const char* visemeName(int viseme)
{
	return viseme >= 0 && viseme < VISEME_COUNT ? k_visemeNames[viseme] : "";
}

VisemePrototype VisemeModel::decodeRecord(const uint8_t* bytes)
{
	VisemePrototype prototype;

	// The header: a big-endian uint32 packing two 16-bit code points of the
	// phoneme, the group at byte 5, the viseme at byte 7
	const uint32_t packed= ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) | ((uint32_t)bytes[2] << 8) |
						   (uint32_t)bytes[3];
	const uint32_t first= packed >> 16;
	const uint32_t second= packed & 0xFFFF;
	if (first != 0)
		appendCodePoint(prototype.phoneme, first);
	if (second != 0)
		appendCodePoint(prototype.phoneme, second);
	prototype.group= bytes[5];
	prototype.viseme= bytes[7];

	const uint8_t* cursor= bytes + 8;
	for (float& value : prototype.mu)
	{
		value= readFloatLe(cursor);
		cursor+= sizeof(float);
	}
	for (float& value : prototype.sigmaInvLower)
	{
		value= readFloatLe(cursor);
		cursor+= sizeof(float);
	}
	return prototype;
}

void VisemeModel::encodeRecord(const VisemePrototype& prototype, uint8_t* bytes)
{
	uint32_t codePoints[2];
	decodeCodePoints(prototype.phoneme, codePoints);
	const uint32_t packed= (codePoints[0] << 16) | codePoints[1];
	bytes[0]= (uint8_t)(packed >> 24);
	bytes[1]= (uint8_t)(packed >> 16);
	bytes[2]= (uint8_t)(packed >> 8);
	bytes[3]= (uint8_t)packed;
	bytes[4]= 0;
	bytes[5]= prototype.group;
	bytes[6]= 0;
	bytes[7]= prototype.viseme;

	uint8_t* cursor= bytes + 8;
	for (float value : prototype.mu)
	{
		writeFloatLe(cursor, value);
		cursor+= sizeof(float);
	}
	for (float value : prototype.sigmaInvLower)
	{
		writeFloatLe(cursor, value);
		cursor+= sizeof(float);
	}
}

bool VisemeModel::save(const std::filesystem::path& path, std::string& outError) const
{
	std::error_code ec;
	if (path.has_parent_path())
		std::filesystem::create_directories(path.parent_path(), ec);

	std::ofstream file(path, std::ios::binary | std::ios::trunc);
	if (!file)
	{
		outError= "cannot write model file: " + PathUtils::pathToUtf8(path);
		return false;
	}
	std::vector<uint8_t> record(k_recordBytes);
	for (const VisemePrototype& prototype : prototypes)
	{
		encodeRecord(prototype, record.data());
		file.write((const char*)record.data(), (std::streamsize)record.size());
	}
	file.close();
	if (!file)
	{
		outError= "cannot write model file: " + PathUtils::pathToUtf8(path);
		return false;
	}
	outError.clear();
	return true;
}

bool VisemeModel::load(const std::filesystem::path& path, std::string& outError)
{
	prototypes.clear();

	std::ifstream file(path, std::ios::binary);
	if (!file)
	{
		outError= "model file missing: " + PathUtils::pathToUtf8(path);
		return false;
	}
	std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	if (bytes.size() < k_recordBytes)
	{
		outError= "model file is empty: " + PathUtils::pathToUtf8(path);
		return false;
	}

	for (size_t offset= 0; offset + k_recordBytes <= bytes.size(); offset+= k_recordBytes)
	{
		VisemePrototype prototype= decodeRecord(bytes.data() + offset);
		if (prototype.viseme >= VISEME_COUNT)
		{
			outError= "model record " + std::to_string(prototypes.size()) + " names viseme " +
					  std::to_string(prototype.viseme) + ": " + PathUtils::pathToUtf8(path);
			prototypes.clear();
			return false;
		}
		prototypes.push_back(std::move(prototype));
	}
	outError.clear();
	return true;
}

void VisemeClassifier::setModel(const VisemeModel& model)
{
	m_prototypes= model.prototypes;
	reset();
}

void VisemeClassifier::reset()
{
	m_votes.fill(VISEME_SIL);
	m_voteIndex= 0;
}

float VisemeClassifier::distance(const MfccVector& vector, const VisemePrototype& prototype)
{
	MfccVector diff;
	for (int index= 0; index < MFCC_COEFF_COUNT; ++index)
		diff[index]= vector[index] - prototype.mu[index];

	// Walk the packed lower triangle row by row: the diagonal once, the
	// off-diagonal terms twice for the symmetric half
	float total= 0.f;
	int packed= 0;
	for (int row= 0; row < MFCC_COEFF_COUNT; ++row)
	{
		float sum= 0.f;
		for (int column= 0; column < row; ++column)
			sum+= 2.f * prototype.sigmaInvLower[packed++] * diff[row] * diff[column];
		sum+= prototype.sigmaInvLower[packed++] * diff[row] * diff[row];
		total+= sum;
	}
	return total;
}

int VisemeClassifier::predict(const MfccVector& vector, std::vector<float>* outDistances)
{
	if (m_prototypes.empty())
		return VISEME_SIL;

	if (outDistances != nullptr)
		outDistances->resize(m_prototypes.size());

	float nearest= std::numeric_limits<float>::infinity();
	int nearestViseme= VISEME_SIL;
	for (size_t index= 0; index < m_prototypes.size(); ++index)
	{
		const VisemePrototype& prototype= m_prototypes[index];
		float d= distance(vector, prototype);
		if (prototype.viseme == VISEME_SIL)
			d/= silSensitivity;
		if (outDistances != nullptr)
			(*outDistances)[index]= d;
		if (d <= nearest)
		{
			nearest= d;
			nearestViseme= prototype.viseme;
		}
	}

	m_votes[m_voteIndex]= nearestViseme;
	m_voteIndex= (m_voteIndex + 1) % k_voteCount;

	std::array<int, VISEME_COUNT> counts{};
	for (int vote : m_votes)
		counts[vote]++;
	int winner= 0;
	int winnerCount= 0;
	for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
	{
		if (counts[viseme] >= winnerCount)
		{
			winner= viseme;
			winnerCount= counts[viseme];
		}
	}
	return winner;
}
