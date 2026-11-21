#include "StreamingSoftware.h"

#include <algorithm>
#include <cctype>

#include <windows.h>
#include <tlhelp32.h>

namespace
{
struct StreamingProgram
{
	const char* exeName; // lower case
	const char* displayName;
};

// Streamlabs Desktop runs its own launcher beside an obs64.exe of its own, so
// it is listed first and wins over the OBS name
const StreamingProgram k_streamingPrograms[]= {
	{"streamlabs obs.exe", "Streamlabs Desktop"},
	{"streamlabs desktop.exe", "Streamlabs Desktop"},
	{"obs64.exe", "OBS Studio"},
	{"obs32.exe", "OBS Studio"},
	{"obs.exe", "OBS Studio"},
};

constexpr std::chrono::milliseconds k_rescanInterval(2000);

// The table's index for an executable name, -1 for none
int findStreamingProgram(const std::string& exeName)
{
	std::string lowerName= exeName;
	std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
				   [](unsigned char character) { return (char)std::tolower(character); });
	for (int programIndex= 0; programIndex < (int)std::size(k_streamingPrograms); ++programIndex)
	{
		if (lowerName == k_streamingPrograms[programIndex].exeName)
			return programIndex;
	}
	return -1;
}

std::string wideToUtf8(const wchar_t* wide)
{
	const int byteCount= WideCharToMultiByte(CP_UTF8, 0, wide, -1, nullptr, 0, nullptr, nullptr);
	if (byteCount <= 1)
		return std::string();
	std::string text((size_t)byteCount - 1, '\0');
	WideCharToMultiByte(CP_UTF8, 0, wide, -1, text.data(), byteCount, nullptr, nullptr);
	return text;
}

// The first table entry any running process matches, empty for none
std::string scanForStreamingProgram()
{
	const HANDLE snapshot= CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snapshot == INVALID_HANDLE_VALUE)
		return std::string();

	int bestProgram= -1;
	PROCESSENTRY32W entry{};
	entry.dwSize= sizeof(entry);
	if (Process32FirstW(snapshot, &entry))
	{
		do
		{
			const int programIndex= findStreamingProgram(wideToUtf8(entry.szExeFile));
			if (programIndex >= 0 && (bestProgram < 0 || programIndex < bestProgram))
				bestProgram= programIndex;
		} while (Process32NextW(snapshot, &entry));
	}
	CloseHandle(snapshot);

	return bestProgram >= 0 ? std::string(k_streamingPrograms[bestProgram].displayName) : std::string();
}
} // namespace

const char* matchStreamingProcessName(const std::string& exeName)
{
	const int programIndex= findStreamingProgram(exeName);
	return programIndex >= 0 ? k_streamingPrograms[programIndex].displayName : nullptr;
}

const std::string& StreamingSoftwareWatch::poll()
{
	const std::chrono::steady_clock::time_point now= std::chrono::steady_clock::now();
	if (!m_bScanned || now - m_lastScanTime >= k_rescanInterval)
	{
		m_detectedName= scanForStreamingProgram();
		m_lastScanTime= now;
		m_bScanned= true;
	}
	return m_detectedName;
}
