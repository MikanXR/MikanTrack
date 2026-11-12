#include "PerformancePanel.h"

#include <algorithm>

#include "imgui.h"
#include "implot.h"

#include "LocText.h"
#include "TrackingRecording.h"
#include "VisionThread.h"

void PerformancePanel::fetchHistory(const VisionThread& visionThread)
{
	m_fetched.clear();
	const uint64_t newest= visionThread.fetchIterationHistory(m_newestSequence, m_fetched);
	if (newest < m_newestSequence)
	{
		// The vision thread's history started over, so the copy is of another
		m_history.clear();
		m_fetched.clear();
		m_newestSequence= 0;
		visionThread.fetchIterationHistory(0, m_fetched);
	}
	if (m_fetched.empty())
		return;

	// The ring overwrote entries this copy never saw, so what it holds is
	// older than the gap and is dropped: the history stays without gaps
	if (m_fetched.front().sequence != m_newestSequence + 1)
		m_history.clear();
	m_history.insert(m_history.end(), m_fetched.begin(), m_fetched.end());
	m_newestSequence= m_history.back().sequence;
	const size_t capacity= (size_t)IterationHistory::k_defaultCapacity;
	if (m_history.size() > capacity)
		m_history.erase(m_history.begin(), m_history.begin() + (std::ptrdiff_t)(m_history.size() - capacity));
}

void PerformancePanel::update(const VisionThread& visionThread, const RecordingInput& recording)
{
	// Fetched whether or not the window shows this frame, so a tab brought to
	// the front has a current chart
	fetchHistory(visionThread);
	m_recording= recording;
	if (m_recording.frames == nullptr)
		m_recording.bHasTelemetry= false;

	// The source follows the shown frame when it changes, and holds the
	// user's choice until it does
	const bool bRecordingShown= isRecordingAvailable();
	if (bRecordingShown != m_bFollowedRecording || (bRecordingShown && m_recording.takeId != m_followedTakeId))
	{
		m_bFollowedRecording= bRecordingShown;
		m_followedTakeId= m_recording.takeId;
		m_source= bRecordingShown ? ePerformanceSource::Recording : ePerformanceSource::Live;
	}
	if (!bRecordingShown)
		m_source= ePerformanceSource::Live;

	if (m_source == ePerformanceSource::Recording)
	{
		const double hitchMs= VisionThread::getHitchThresholdMs();
		if (m_hitchCountTakeId != m_recording.takeId)
		{
			m_hitchCountTakeId= m_recording.takeId;
			m_recordedHitchCount= countRecordedHitches(*m_recording.frames, hitchMs);
		}
		// The window ends at the playhead, the way the live chart ends at the
		// newest iteration, so playing the take scrolls the chart the way the
		// live frame loop does
		buildRecordedSeries(*m_recording.frames, m_recording.playheadFrame, IterationHistory::k_defaultCapacity,
							hitchMs, m_series);
	}
	else
	{
		buildLiveSeries(m_history, m_newestSequence, m_series);
	}
}

bool PerformancePanel::setSource(ePerformanceSource source)
{
	if (source == ePerformanceSource::Recording && !isRecordingAvailable())
		return false;
	m_source= source;
	return true;
}

bool PerformancePanel::getMarkedTotal(float& outMs) const
{
	if (m_series.markedIndex < 0 || m_series.markedIndex >= (int)m_series.samples.size())
		return false;
	outMs= m_series.samples[m_series.markedIndex].totalMs;
	return true;
}

const char* PerformancePanel::getSourceName(ePerformanceSource source)
{
	return source == ePerformanceSource::Recording ? "recording" : "live";
}

bool PerformancePanel::parseSource(const std::string& text, ePerformanceSource& outSource)
{
	for (const ePerformanceSource source : {ePerformanceSource::Live, ePerformanceSource::Recording})
	{
		if (text == getSourceName(source))
		{
			outSource= source;
			return true;
		}
	}
	return false;
}

const RecordedTelemetry* PerformancePanel::getPlayheadTelemetry() const
{
	if (m_recording.frames == nullptr || m_recording.playheadFrame < 0 ||
		m_recording.playheadFrame >= (int)m_recording.frames->size())
		return nullptr;
	const RecordedTelemetry* telemetry= (*m_recording.frames)[m_recording.playheadFrame];
	return telemetry != nullptr && telemetry->present ? telemetry : nullptr;
}

void PerformancePanel::draw(bool* pOpen, const VisionThread& visionThread)
{
	if (!ImGui::Begin(locWindowTitle("windows.performance"), pOpen))
	{
		ImGui::End();
		return;
	}
	drawSourceCombo();
	if (m_source == ePerformanceSource::Recording)
		drawRecording();
	else
		drawLive(visionThread);
	ImGui::End();
}

void PerformancePanel::drawSourceCombo()
{
	static const char* k_sourceKeys[]= {"performancePanel.sourceLive", "performancePanel.sourceRecording"};
	const bool bRecordingAvailable= isRecordingAvailable();
	ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8.f);
	if (ImGui::BeginCombo("##performanceSource", locText(k_sourceKeys[(int)m_source])))
	{
		if (ImGui::Selectable(locLabel(k_sourceKeys[0]), m_source == ePerformanceSource::Live))
			setSource(ePerformanceSource::Live);
		if (ImGui::Selectable(locLabel(k_sourceKeys[1]), m_source == ePerformanceSource::Recording,
							  bRecordingAvailable ? ImGuiSelectableFlags_None : ImGuiSelectableFlags_Disabled))
			setSource(ePerformanceSource::Recording);
		if (!bRecordingAvailable)
			ImGui::SetItemTooltip("%s", locText("performancePanel.recordingUnavailable"));
		ImGui::EndCombo();
	}
	ImGui::SetItemTooltip("%s", locText("performancePanel.sourceTooltip"));
}

void PerformancePanel::drawLive(const VisionThread& visionThread)
{
	const VisionThread::Telemetry telemetry= visionThread.getTelemetry();
	if (!telemetry.bValid)
	{
		ImGui::TextDisabled("%s", locText("performancePanel.noTelemetry"));
		return;
	}

	ImGui::TextUnformatted(locFormat("performancePanel.totalFmt", (double)telemetry.totalMs).c_str());
	ImGui::SameLine(0.f, 24.f);
	const int hitchCount= visionThread.getHitchCount();
	if (hitchCount == 0)
	{
		ImGui::TextDisabled("%s", locText("performancePanel.hitchesNone"));
	}
	else
	{
		ImGui::TextUnformatted(
			locFormat("performancePanel.hitchesFmt", hitchCount, (double)visionThread.getLastHitchMs()).c_str());
	}
	if (m_recording.bShown)
		ImGui::TextDisabled("%s", locText("performancePanel.replayNote"));

	drawChart(m_series, "performancePanel.iterationsAgoAxis", false);
	drawPhaseTable(telemetry.phaseMs);
}

void PerformancePanel::drawRecording()
{
	const RecordedTelemetry* shown= getPlayheadTelemetry();
	if (shown != nullptr)
		ImGui::TextUnformatted(locFormat("performancePanel.totalFmt", (double)shown->totalMs).c_str());
	else
		ImGui::TextDisabled("%s", locText("performancePanel.frameNotRecorded"));
	ImGui::SameLine(0.f, 24.f);
	if (m_recordedHitchCount == 0)
		ImGui::TextDisabled("%s", locText("performancePanel.hitchesNone"));
	else
		ImGui::TextUnformatted(locFormat("performancePanel.takeHitchesFmt", m_recordedHitchCount).c_str());
	// A recorded total is the sum of the recorded phases, and the diagnostics
	// phase is still running when its frame is recorded
	ImGui::TextDisabled("%s", locText("performancePanel.recordedNote"));

	if (m_series.samples.empty())
		ImGui::TextDisabled("%s", locText("performancePanel.windowNotRecorded"));
	else
		drawChart(m_series, "performancePanel.framesBeforePlayheadAxis", true);
	if (shown != nullptr)
		drawPhaseTable(shown->phaseMs);
}

void PerformancePanel::drawPhaseTable(const float* phaseMs)
{
	if (ImGui::BeginTable("##phases", 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn(locText("performancePanel.phaseColumn"));
		ImGui::TableSetupColumn(locText("performancePanel.msColumn"));
		ImGui::TableHeadersRow();
		for (int phaseIndex= 1; phaseIndex < (int)VisionThread::eVisionPhase::Count; ++phaseIndex)
		{
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(VisionThread::getPhaseName((VisionThread::eVisionPhase)phaseIndex));
			ImGui::TableNextColumn();
			ImGui::Text("%.2f", phaseMs[phaseIndex]);
		}
		ImGui::EndTable();
	}
}

void PerformancePanel::drawChart(const PerformanceSeries& series, const char* xAxisKey, bool bMarkPlayhead)
{
	// The phases the table lists, stacked in its order: each phase's band
	// runs from the sum of the phases before it to the sum through it
	const int phaseCount= (int)VisionThread::eVisionPhase::Count;
	const int count= (int)series.samples.size();
	m_xs.resize(count);
	m_totals.resize(count);
	m_stackEdges.resize(phaseCount);
	for (std::vector<float>& edge : m_stackEdges)
		edge.resize(count);
	m_hitchXs.clear();
	m_hitchYs.clear();
	float maxMs= 0.f;
	for (int index= 0; index < count; ++index)
	{
		const PerformanceSample& sample= series.samples[index];
		m_xs[index]= sample.x;
		float sum= 0.f;
		m_stackEdges[0][index]= 0.f;
		for (int phaseIndex= 1; phaseIndex < phaseCount; ++phaseIndex)
		{
			sum+= sample.phaseMs[phaseIndex];
			m_stackEdges[phaseIndex][index]= sum;
		}
		m_totals[index]= sample.totalMs;
		maxMs= std::max(maxMs, std::max(sum, sample.totalMs));
		if (sample.bHitch)
		{
			m_hitchXs.push_back(sample.x);
			m_hitchYs.push_back(sample.totalMs);
		}
	}

	// The range never shrinks below the hitch line and a margin above it, so
	// a quiet history does not zoom into noise and the line stays in view
	const double hitchMs= VisionThread::getHitchThresholdMs();
	const double yMax= std::max(hitchMs * 1.2, (double)maxMs * 1.1);
	const double xMax= (double)(IterationHistory::k_defaultCapacity - 1);

	constexpr ImPlotFlags k_plotFlags= ImPlotFlags_NoTitle | ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect |
									   ImPlotFlags_NoMouseText;
	constexpr ImPlotAxisFlags k_axisFlags=
		ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoHighlight | ImPlotAxisFlags_NoSideSwitch;
	const ImVec2 plotSize(-1.f, ImGui::GetFontSize() * 14.f);
	if (!ImPlot::BeginPlot("##frameLoop", plotSize, k_plotFlags))
		return;
	// X counts back from the newest (or the playhead) right to left, which
	// sits at the right edge
	ImPlot::SetupAxes(locText(xAxisKey), locText("performancePanel.msColumn"), k_axisFlags | ImPlotAxisFlags_Invert,
					  k_axisFlags);
	ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, xMax, ImPlotCond_Always);
	ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, yMax, ImPlotCond_Always);
	ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Outside);

	for (int phaseIndex= 1; phaseIndex < phaseCount; ++phaseIndex)
	{
		ImPlotSpec bandSpec;
		bandSpec.FillAlpha= 0.85f;
		ImPlot::PlotShaded(VisionThread::getPhaseName((VisionThread::eVisionPhase)phaseIndex), m_xs.data(),
						   m_stackEdges[phaseIndex - 1].data(), m_stackEdges[phaseIndex].data(), count, bandSpec);
	}

	// The total outlines the stack: whatever it holds above the phases' sum
	// is time no phase measured
	ImPlotSpec totalSpec;
	totalSpec.LineColor= ImVec4(0.92f, 0.92f, 0.92f, 1.f);
	totalSpec.LineWeight= 1.5f;
	ImPlot::PlotLine(locText("performancePanel.chartTotal"), m_xs.data(), m_totals.data(), count, totalSpec);

	ImPlotSpec limitSpec;
	limitSpec.LineColor= ImVec4(1.f, 0.35f, 0.35f, 0.9f);
	limitSpec.Flags= ImPlotInfLinesFlags_Horizontal;
	ImPlot::PlotInfLines(locText("performancePanel.chartHitchLimit"), &hitchMs, 1, limitSpec);

	ImPlotSpec hitchSpec;
	hitchSpec.Marker= ImPlotMarker_Circle;
	hitchSpec.MarkerSize= 4.f;
	hitchSpec.MarkerFillColor= ImVec4(1.f, 0.35f, 0.35f, 1.f);
	hitchSpec.MarkerLineColor= ImVec4(1.f, 0.35f, 0.35f, 1.f);
	ImPlot::PlotScatter(locText("performancePanel.chartHitches"), m_hitchXs.data(), m_hitchYs.data(),
						(int)m_hitchXs.size(), hitchSpec);

	// The playhead: a ring on its frame's total, at the right edge
	if (bMarkPlayhead && series.markedIndex >= 0 && series.markedIndex < count)
	{
		const float playheadX= m_xs[series.markedIndex];
		const float playheadY= m_totals[series.markedIndex];
		ImPlotSpec playheadSpec;
		playheadSpec.Marker= ImPlotMarker_Circle;
		playheadSpec.MarkerSize= 6.f;
		playheadSpec.MarkerFillColor= ImVec4(0.f, 0.f, 0.f, 0.f);
		playheadSpec.MarkerLineColor= ImVec4(0.4f, 0.85f, 1.f, 1.f);
		playheadSpec.LineWeight= 2.f;
		ImPlot::PlotScatter(locText("performancePanel.chartPlayhead"), &playheadX, &playheadY, 1, playheadSpec);
	}

	ImPlot::EndPlot();
}
