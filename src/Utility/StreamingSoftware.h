#pragma once

#include <chrono>
#include <string>

// The streaming program an executable's file name belongs to ("obs64.exe" is
// "OBS Studio"), nullptr for any other program. Letter case is ignored.
const char* matchStreamingProcessName(const std::string& exeName);

// Whether a streaming program is running, from the system's process list.
// The list is read at most once per rescan interval, and only when asked, so
// a caller polls it on the frames it has a warning to show.
class StreamingSoftwareWatch
{
public:
	// The running streaming program's name, empty for none
	const std::string& poll();

private:
	std::string m_detectedName;
	std::chrono::steady_clock::time_point m_lastScanTime;
	bool m_bScanned= false;
};
