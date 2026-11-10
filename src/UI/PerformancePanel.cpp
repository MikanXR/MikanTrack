#include "PerformancePanel.h"

#include <algorithm>

#include "imgui.h"
#include "implot.h"

#include "LocText.h"
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

void PerformancePanel::draw(bool* pOpen, const VisionThread& visionThread, bool bReplayShown)
{
	// Fetched whether or not the window shows this frame, so a tab brought to
	// the front has a current chart
	fetchHistory(visionThread);

	if (!ImGui::Begin(locWindowTitle("windows.performance"), pOpen))
	{
		ImGui::End();
		return;
	}
	const VisionThread::Telemetry telemetry= visionThread.getTelemetry();
	if (!telemetry.bValid)
	{
		ImGui::TextDisabled("%s", locText("performancePanel.noTelemetry"));
		ImGui::End();
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
	if (bReplayShown)
		ImGui::TextDisabled("%s", locText("performancePanel.replayNote"));

	drawChart();

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
			ImGui::Text("%.2f", telemetry.phaseMs[phaseIndex]);
		}
		ImGui::EndTable();
	}
	ImGui::End();
}

void PerformancePanel::drawChart()
{
	// The phases the table lists, stacked in its order: each phase's band
	// runs from the sum of the phases before it to the sum through it
	const int phaseCount= (int)VisionThread::eVisionPhase::Count;
	const int count= (int)m_history.size();
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
		const IterationHistory::Entry& entry= m_history[index];
		m_xs[index]= (float)(m_newestSequence - entry.sequence);
		float sum= 0.f;
		m_stackEdges[0][index]= 0.f;
		for (int phaseIndex= 1; phaseIndex < phaseCount; ++phaseIndex)
		{
			sum+= entry.phaseMs[phaseIndex];
			m_stackEdges[phaseIndex][index]= sum;
		}
		m_totals[index]= entry.totalMs;
		maxMs= std::max(maxMs, std::max(sum, entry.totalMs));
		if (entry.bHitch)
		{
			m_hitchXs.push_back(m_xs[index]);
			m_hitchYs.push_back(entry.totalMs);
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
	// Iterations ago run right to left, the newest at the right edge
	ImPlot::SetupAxes(locText("performancePanel.iterationsAgoAxis"), locText("performancePanel.msColumn"),
					  k_axisFlags | ImPlotAxisFlags_Invert, k_axisFlags);
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

	ImPlot::EndPlot();
}
