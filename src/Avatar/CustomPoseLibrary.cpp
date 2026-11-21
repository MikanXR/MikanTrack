#include "CustomPoseLibrary.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include "Logger.h"
#include "PathUtils.h"

namespace
{
const char* k_logLabel= "CustomPoseLibrary";
} // namespace

CustomPoseLibrary::CustomPoseLibrary(std::filesystem::path directory)
	: m_directory(std::move(directory))
{}

std::filesystem::path CustomPoseLibrary::getDefaultDirectory()
{
	return PathUtils::getProjectsRootDirectory() / "Poses";
}

bool CustomPoseLibrary::isValidId(const std::string& id)
{
	if (id.empty() || id.size() > 64)
		return false;
	return std::all_of(id.begin(), id.end(), [](char character) {
		return (character >= '0' && character <= '9') || (character >= 'a' && character <= 'f') ||
			   (character >= 'A' && character <= 'F') || character == '-';
	});
}

std::filesystem::path CustomPoseLibrary::getPosePath(const std::string& id) const
{
	return m_directory / (id + ".json");
}

void CustomPoseLibrary::refresh()
{
	m_poses.clear();
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

		CustomPose pose;
		std::string parseError;
		if (!customPoseFromJson(buffer.str(), pose, parseError) || !isValidId(pose.id))
		{
			MIKAN_LOG_WARNING(k_logLabel) << "Skipping unreadable pose " << PathUtils::pathToUtf8(entry.path())
										  << (parseError.empty() ? "" : ": ") << parseError;
			continue;
		}
		if (find(pose.id) != nullptr)
		{
			MIKAN_LOG_WARNING(k_logLabel) << "Skipping " << PathUtils::pathToUtf8(entry.path()) << ": id " << pose.id
										  << " is already in the library";
			continue;
		}
		m_poses.push_back(pose);
	}
	sortPoses();
}

const CustomPose* CustomPoseLibrary::find(const std::string& id) const
{
	for (const CustomPose& pose : m_poses)
	{
		if (pose.id == id)
			return &pose;
	}
	return nullptr;
}

bool CustomPoseLibrary::save(const CustomPose& pose)
{
	if (!isValidId(pose.id))
	{
		MIKAN_LOG_ERROR(k_logLabel) << "Refusing to save a pose with id '" << pose.id << "'";
		return false;
	}
	std::error_code error;
	std::filesystem::create_directories(m_directory, error);
	const std::filesystem::path path= getPosePath(pose.id);
	std::ofstream file(path, std::ios::binary);
	if (!file.is_open())
	{
		MIKAN_LOG_ERROR(k_logLabel) << "Failed to open " << PathUtils::pathToUtf8(path) << " for writing";
		return false;
	}
	file << customPoseToJson(pose);
	file.close();

	auto existing= std::find_if(m_poses.begin(), m_poses.end(),
								[&pose](const CustomPose& candidate) { return candidate.id == pose.id; });
	if (existing != m_poses.end())
		*existing= pose;
	else
		m_poses.push_back(pose);
	sortPoses();
	return true;
}

bool CustomPoseLibrary::rename(const std::string& id, const std::string& name)
{
	const CustomPose* existing= find(id);
	if (existing == nullptr)
		return false;
	CustomPose renamed= *existing;
	renamed.name= name;
	return save(renamed);
}

std::string CustomPoseLibrary::duplicate(const CustomPose& source, const std::string& name)
{
	CustomPose copy= source;
	copy.id= makeCustomPoseId();
	copy.name= name;
	return save(copy) ? copy.id : std::string();
}

std::string CustomPoseLibrary::create(const std::string& name)
{
	CustomPose pose= makeDefaultCustomPose();
	pose.id= makeCustomPoseId();
	pose.name= name;
	return save(pose) ? pose.id : std::string();
}

bool CustomPoseLibrary::remove(const std::string& id)
{
	if (!isValidId(id))
		return false;
	std::error_code error;
	const bool bRemoved= std::filesystem::remove(getPosePath(id), error);
	m_poses.erase(
		std::remove_if(m_poses.begin(), m_poses.end(), [&id](const CustomPose& pose) { return pose.id == id; }),
		m_poses.end());
	return bRemoved;
}

void CustomPoseLibrary::sortPoses()
{
	std::sort(m_poses.begin(), m_poses.end(), [](const CustomPose& a, const CustomPose& b) {
		return a.name != b.name ? a.name < b.name : a.id < b.id;
	});
}
