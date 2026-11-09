#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "FaceProfile.h"

// The global library of face profiles: one JSON file per profile,
// `<id>.json`, in `Documents/MikanTrack/FaceProfiles/` beside the project
// folders. Avatars name the profiles they use by id in their rig sidecar, so
// a rename rewrites only the name inside the file. Main thread only.
class FaceProfileLibrary
{
public:
	explicit FaceProfileLibrary(std::filesystem::path directory= getDefaultDirectory());

	static std::filesystem::path getDefaultDirectory();
	const std::filesystem::path& getDirectory() const { return m_directory; }

	// Rereads the folder. A file that does not load is logged and skipped.
	void refresh();

	// Sorted by name, then id
	const std::vector<FaceProfile>& getProfiles() const { return m_profiles; }
	const FaceProfile* find(const std::string& id) const;

	// Writes the profile's file (creating the folder) and updates the list.
	// False for an id that is not a plain UUID-like name, or a write failure.
	bool save(const FaceProfile& profile);
	// Rewrites the name inside the file and nothing else
	bool rename(const std::string& id, const std::string& name);
	// A copy under a fresh id and the given name, saved; its id, or "" on
	// failure
	std::string duplicate(const FaceProfile& source, const std::string& name);
	// A new profile with no outputs, saved; its id, or "" on failure
	std::string create(const std::string& name);
	// Deletes the profile's file
	bool remove(const std::string& id);

	// Whether an id can name a file in the library: hex digits and dashes
	static bool isValidId(const std::string& id);

private:
	std::filesystem::path getProfilePath(const std::string& id) const;
	void sortProfiles();

	std::filesystem::path m_directory;
	std::vector<FaceProfile> m_profiles;
};
