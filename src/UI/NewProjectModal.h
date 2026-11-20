#pragma once

#include <string>

// The project-name prompt New Project opens, from the startup menu and from
// the File menu. The name is checked as it is typed against the project name
// rules and the folders that already exist.
class NewProjectModal
{
public:
	// Opens the prompt on the next draw, with an empty name
	void request();
	// Draws the prompt while it is open, in the window that is current, which
	// must be the same window every frame. True on the frame a valid name is
	// confirmed, with the name in outProjectName.
	bool draw(std::string& outProjectName);

private:
	char m_nameBuffer[64]= {};
	bool m_bRequested= false;
};
