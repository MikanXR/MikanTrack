#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "IterationHistory.h"
#include "PerformanceSeries.h"

struct RecordedTelemetry;
class VisionThread;

// Which frame loop the Performance panel shows
enum class ePerformanceSource : int
{
	// The vision thread now
	Live,
	// The shown take's frames, up to the playhead
	Recording,
};

// The Performance panel: a frame loop iteration by phase, the hitch count,
// and a chart of the iterations before it with the phases stacked under
// each total. Live it shows the vision thread's last iteration and recent
// history. While a replay is shown whose take carries its stage state it
// can show the take instead: the playhead's frame and the frames before it.
class PerformancePanel
{
public:
	// The shown take, read while a replay is shown
	struct RecordingInput
	{
		// A replay is the shown frame
		bool bShown= false;
		// Changes with each load
		int takeId= -1;
		// Every frame's recorded stage state, null where a frame has none
		const std::vector<const RecordedTelemetry*>* frames= nullptr;
		int playheadFrame= 0;
		// Whether any frame carries its stage state
		bool bHasTelemetry= false;
	};

	// Every frame, drawn or not: fetches the live history, follows the shown
	// frame's source, and builds the chart's series
	void update(const VisionThread& visionThread, const RecordingInput& recording);
	void draw(bool* pOpen, const VisionThread& visionThread);

	// The source follows the shown frame: Recording when a replay whose take
	// carries stage state starts being shown, Live when the view returns to
	// live. A choice made in between holds until the shown source changes.
	ePerformanceSource getSource() const { return m_source; }
	bool isRecordingAvailable() const { return m_recording.bShown && m_recording.bHasTelemetry; }
	// False, leaving the source as it was, for Recording while it is not
	// available
	bool setSource(ePerformanceSource source);
	// The series the chart draws, as the last update built it
	const PerformanceSeries& getSeries() const { return m_series; }
	// The marked sample's total (the newest live, the playhead's frame in a
	// recording); false when there is none
	bool getMarkedTotal(float& outMs) const;

	static const char* getSourceName(ePerformanceSource source);
	static bool parseSource(const std::string& text, ePerformanceSource& outSource);

private:
	// Appends the iterations the vision thread published since the last
	// fetch, keeping at most the ring's capacity
	void fetchHistory(const VisionThread& visionThread);
	void drawSourceCombo();
	void drawLive(const VisionThread& visionThread);
	void drawRecording();
	// The one chart over a series, whichever source filled it. xAxisKey names
	// what X counts; bMarkPlayhead marks the marked sample as the playhead.
	void drawChart(const PerformanceSeries& series, const char* xAxisKey, bool bMarkPlayhead);
	// The time per phase, phase 0 (none) left out
	static void drawPhaseTable(const float* phaseMs);
	// The playhead's frame's stage state, null when it has none
	const RecordedTelemetry* getPlayheadTelemetry() const;

	// Oldest first, numbered without gaps up to m_newestSequence
	std::vector<IterationHistory::Entry> m_history;
	uint64_t m_newestSequence= 0;
	std::vector<IterationHistory::Entry> m_fetched;

	ePerformanceSource m_source= ePerformanceSource::Live;
	// The shown source the choice last followed
	bool m_bFollowedRecording= false;
	int m_followedTakeId= -1;
	RecordingInput m_recording;
	// The take's hitches, counted once per take
	int m_recordedHitchCount= 0;
	int m_hitchCountTakeId= -1;

	PerformanceSeries m_series;
	// The chart's arrays, rebuilt per draw: each phase has its lower and
	// upper edge in the stack
	std::vector<float> m_xs;
	std::vector<std::vector<float>> m_stackEdges;
	std::vector<float> m_totals;
	std::vector<float> m_hitchXs;
	std::vector<float> m_hitchYs;
};
