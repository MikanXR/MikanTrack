#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <vector>

// Thread-safe ring buffer of log lines fed from the Logger callback,
// rendered as a collapsible ImGui panel.
class LogPanel
{
public:
	static LogPanel& getInstance();

	// Registered as the Logger's t_logCallback (may be called from any thread)
	static void logCallback(int level, const char* line);

	void draw(bool* pOpen);

	// The newest lineCount lines at or above minLevel, oldest first, for the
	// automation server's log tail
	void copyTail(size_t lineCount, int minLevel, std::vector<std::string>& outLines);

private:
	struct LogLine
	{
		int level;
		std::string text;
	};

	void addLine(int level, const char* line);

	std::mutex m_mutex;
	std::deque<LogLine> m_lines;
	bool m_bAutoScroll= true;

	static constexpr size_t k_maxLines= 2000;
};
