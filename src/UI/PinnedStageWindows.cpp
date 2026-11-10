#include "PinnedStageWindows.h"

#include <algorithm>

#include "imgui.h"

#include "PipelineText.h"
#include "Scene3dPanel.h"
#include "StageInspector.h"
#include "VideoPreviewPanel.h"

namespace
{
// The inspector's width in font heights; the view takes the rest
constexpr float kInspectorEms= 24.f;
// A window pinned for the first time floats at this size, centered on the
// main window, each one more pinned beside it stepped down and right
const ImVec2 kFirstSize(900.f, 520.f);
constexpr float kCascadeStep= 24.f;
} // namespace

PinnedStageWindows::PinnedStageWindows(VideoPreviewPanel* previewPanel, Scene3dPanel* scenePanel)
	: m_previewPanel(previewPanel)
	, m_scenePanel(scenePanel)
{
}

PinnedStageWindows::~PinnedStageWindows()= default;

std::string PinnedStageWindows::getWindowId(const PipelineStageId& stage)
{
	return "pin." + PipelineModel::toString(stage);
}

void PinnedStageWindows::pin(const PipelineStageId& stage)
{
	if (!stage.isValid())
		return;
	for (const std::unique_ptr<Entry>& entry : m_entries)
	{
		if (entry->stage == stage)
		{
			entry->bFocus= true;
			return;
		}
	}
	auto entry= std::make_unique<Entry>();
	entry->stage= stage;
	m_entries.push_back(std::move(entry));
}

bool PinnedStageWindows::unpin(const PipelineStageId& stage)
{
	const auto found= std::find_if(m_entries.begin(), m_entries.end(),
								   [&](const std::unique_ptr<Entry>& entry) { return entry->stage == stage; });
	if (found == m_entries.end())
		return false;
	m_entries.erase(found);
	return true;
}

void PinnedStageWindows::unpinAll()
{
	m_entries.clear();
}

std::vector<PipelineStageId> PinnedStageWindows::getPinned() const
{
	std::vector<PipelineStageId> stages;
	for (const std::unique_ptr<Entry>& entry : m_entries)
		stages.push_back(entry->stage);
	return stages;
}

PipelineStageId PinnedStageWindows::draw(const PipelineStatusInputs& inputs, const StageView::Frame& frame,
										 IStageContent* content)
{
	// A stage the project no longer has (a camera removed, the pipeline
	// switched) is unpinned before anything draws it
	const std::vector<PipelineStageId> all= PipelineModel::getAllStages(inputs.bDirect, inputs.cameraCount);
	m_entries.erase(std::remove_if(m_entries.begin(), m_entries.end(),
								   [&](const std::unique_ptr<Entry>& entry) {
									   return std::find(all.begin(), all.end(), entry->stage) == all.end();
								   }),
					m_entries.end());

	PipelineStageId request;
	const ImGuiViewport* mainViewport= ImGui::GetMainViewport();
	for (size_t entryIndex= 0; entryIndex < m_entries.size();)
	{
		Entry& entry= *m_entries[entryIndex];
		const std::string title= PipelineText::stageDisplayName(entry.stage) + "###" + getWindowId(entry.stage);
		const float cascade= kCascadeStep * (float)(entryIndex % 8);
		ImGui::SetNextWindowPos(ImVec2(mainViewport->GetCenter().x + cascade, mainViewport->GetCenter().y + cascade),
								ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
		ImGui::SetNextWindowSize(kFirstSize, ImGuiCond_FirstUseEver);
		if (entry.bFocus)
		{
			entry.bFocus= false;
			ImGui::SetNextWindowFocus();
		}

		// A window closed this frame draws nothing, so its draw data holds
		// no image of the framebuffer released with it
		bool bOpen= true;
		const bool bVisible= ImGui::Begin(title.c_str(), &bOpen);
		if (bVisible && bOpen)
		{
			// The view on the left, the inspector at a fixed width on the
			// right, each scrolling on its own
			const float inspectorWidth= ImGui::GetFontSize() * kInspectorEms;
			const float viewWidth= std::max(ImGui::GetContentRegionAvail().x - inspectorWidth -
												ImGui::GetStyle().ItemSpacing.x,
											64.f);
			// The wheel orbits the scene, so the view never scrolls
			ImGui::BeginChild("##view", ImVec2(viewWidth, 0.f), ImGuiChildFlags_None,
							  ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
			drawView(entry, inputs, frame);
			ImGui::EndChild();
			ImGui::SameLine();
			ImGui::BeginChild("##inspector", ImVec2(0.f, 0.f), ImGuiChildFlags_None);
			const PipelineStageId rowRequest= StageInspector::drawStageBody(inputs, entry.stage, content);
			if (rowRequest.isValid())
				request= rowRequest;
			ImGui::EndChild();
		}
		ImGui::End();

		if (bOpen)
			++entryIndex;
		else
			m_entries.erase(m_entries.begin() + (std::ptrdiff_t)entryIndex);
	}
	return request;
}

void PinnedStageWindows::drawView(Entry& entry, const PipelineStatusInputs& inputs, const StageView::Frame& frame)
{
	const StageViewContent content= decideStageViewContent(entry.stage, inputs.bDirect, inputs.directCameraIndex,
														   (int)m_previewPanel->getCameraCount());
	if (content.bCameras)
	{
		m_previewPanel->drawSecondaryBody(*frame.cameraResults, *frame.executionProviders, frame.forearms,
										  content.cameraIndex);
		return;
	}

	if (entry.sceneView == nullptr)
		entry.sceneView= std::make_unique<SceneViewState>();
	const SceneLayers layers= makeStageSceneLayers(content.sceneStage, m_scenePanel->getShowAvatar(),
												   m_scenePanel->getShowPerCameraSkeletons());
	m_scenePanel->drawViewBody(*entry.sceneView, *frame.fused, *frame.sceneCameras, *frame.scenePerCamera, layers);
}
