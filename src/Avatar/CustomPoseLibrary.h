#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "CustomPose.h"

// The global library of custom poses: one JSON file per pose, `<id>.json`,
// in `Documents/MikanTrack/Poses/` beside the project folders, so every
// project and every avatar sees the same poses. A project names the one it
// shows by id, so a rename rewrites only the name inside the file. The
// built-in pose is never in it. Main thread only.
class CustomPoseLibrary
{
public:
	explicit CustomPoseLibrary(std::filesystem::path directory= getDefaultDirectory());

	static std::filesystem::path getDefaultDirectory();
	const std::filesystem::path& getDirectory() const { return m_directory; }

	// Rereads the folder. A file that does not load is logged and skipped.
	void refresh();

	// Sorted by name, then id
	const std::vector<CustomPose>& getPoses() const { return m_poses; }
	const CustomPose* find(const std::string& id) const;

	// Writes the pose's file (creating the folder) and updates the list.
	// False for an id that is not a plain UUID-like name, which the built-in
	// pose's is not, or a write failure.
	bool save(const CustomPose& pose);
	// Rewrites the name inside the file and nothing else
	bool rename(const std::string& id, const std::string& name);
	// A copy under a fresh id and the given name, saved; its id, or "" on
	// failure
	std::string duplicate(const CustomPose& source, const std::string& name);
	// A new pose at the default, saved; its id, or "" on failure
	std::string create(const std::string& name);
	// Deletes the pose's file
	bool remove(const std::string& id);

	// Whether an id can name a file in the library: hex digits and dashes
	static bool isValidId(const std::string& id);

private:
	std::filesystem::path getPosePath(const std::string& id) const;
	void sortPoses();

	std::filesystem::path m_directory;
	std::vector<CustomPose> m_poses;
};
