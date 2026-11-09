#include "FaceProfileLibrary.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include "Logger.h"
#include "PathUtils.h"

namespace
{
const char* k_logLabel= "FaceProfileLibrary";
} // namespace

FaceProfileLibrary::FaceProfileLibrary(std::filesystem::path directory)
	: m_directory(std::move(directory))
{}

std::filesystem::path FaceProfileLibrary::getDefaultDirectory()
{
	return PathUtils::getProjectsRootDirectory() / "FaceProfiles";
}

bool FaceProfileLibrary::isValidId(const std::string& id)
{
	if (id.empty() || id.size() > 64)
		return false;
	return std::all_of(id.begin(), id.end(), [](char character) {
		return (character >= '0' && character <= '9') || (character >= 'a' && character <= 'f') ||
			   (character >= 'A' && character <= 'F') || character == '-';
	});
}

std::filesystem::path FaceProfileLibrary::getProfilePath(const std::string& id) const
{
	return m_directory / (id + ".json");
}

void FaceProfileLibrary::refresh()
{
	m_profiles.clear();
	std::error_code error;
	if (!std::filesystem::is_directory(m_directory, error))
		return;

	for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(m_directory, error))
	{
		if (!entry.is_regular_file() || entry.path().extension() != ".json")
			continue;
		std::ifstream file(entry.path(), std::ios::binary);
		std::stringstream buffer;
		buffer << file.rdbuf();

		FaceProfile profile;
		std::string parseError;
		if (!faceProfileFromJson(buffer.str(), profile, parseError) || !isValidId(profile.id))
		{
			MIKAN_LOG_WARNING(k_logLabel) << "Skipping unreadable face profile " << PathUtils::pathToUtf8(entry.path())
										  << (parseError.empty() ? "" : ": ") << parseError;
			continue;
		}
		if (find(profile.id) != nullptr)
		{
			MIKAN_LOG_WARNING(k_logLabel) << "Skipping " << PathUtils::pathToUtf8(entry.path()) << ": id " << profile.id
										  << " is already in the library";
			continue;
		}
		m_profiles.push_back(profile);
	}
	sortProfiles();
}

const FaceProfile* FaceProfileLibrary::find(const std::string& id) const
{
	for (const FaceProfile& profile : m_profiles)
	{
		if (profile.id == id)
			return &profile;
	}
	return nullptr;
}

bool FaceProfileLibrary::save(const FaceProfile& profile)
{
	if (!isValidId(profile.id))
	{
		MIKAN_LOG_ERROR(k_logLabel) << "Refusing to save a face profile with id '" << profile.id << "'";
		return false;
	}
	std::error_code error;
	std::filesystem::create_directories(m_directory, error);
	const std::filesystem::path path= getProfilePath(profile.id);
	std::ofstream file(path, std::ios::binary);
	if (!file.is_open())
	{
		MIKAN_LOG_ERROR(k_logLabel) << "Failed to open " << PathUtils::pathToUtf8(path) << " for writing";
		return false;
	}
	file << faceProfileToJson(profile);
	file.close();

	auto existing= std::find_if(m_profiles.begin(), m_profiles.end(),
								[&profile](const FaceProfile& candidate) { return candidate.id == profile.id; });
	if (existing != m_profiles.end())
		*existing= profile;
	else
		m_profiles.push_back(profile);
	sortProfiles();
	return true;
}

bool FaceProfileLibrary::rename(const std::string& id, const std::string& name)
{
	const FaceProfile* existing= find(id);
	if (existing == nullptr)
		return false;
	FaceProfile renamed= *existing;
	renamed.name= name;
	return save(renamed);
}

std::string FaceProfileLibrary::duplicate(const FaceProfile& source, const std::string& name)
{
	FaceProfile copy= source;
	copy.id= makeFaceProfileId();
	copy.name= name;
	return save(copy) ? copy.id : std::string();
}

std::string FaceProfileLibrary::create(const std::string& name)
{
	FaceProfile profile;
	profile.id= makeFaceProfileId();
	profile.name= name;
	return save(profile) ? profile.id : std::string();
}

bool FaceProfileLibrary::remove(const std::string& id)
{
	if (!isValidId(id))
		return false;
	std::error_code error;
	const bool bRemoved= std::filesystem::remove(getProfilePath(id), error);
	m_profiles.erase(std::remove_if(m_profiles.begin(), m_profiles.end(),
									[&id](const FaceProfile& profile) { return profile.id == id; }),
					 m_profiles.end());
	return bRemoved;
}

void FaceProfileLibrary::sortProfiles()
{
	std::sort(m_profiles.begin(), m_profiles.end(), [](const FaceProfile& a, const FaceProfile& b) {
		return a.name != b.name ? a.name < b.name : a.id < b.id;
	});
}
