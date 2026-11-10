#pragma once

#include <memory>
#include <string>
#include <vector>

#include "PipelineModel.h"
#include "StageView.h" // StageView::Frame

class IStageContent;
class Scene3dPanel;
class VideoPreviewPanel;
struct SceneViewState;

// The stages pinned into windows of their own: each window shows the
// stage's view on the left, chosen the way the Stage View follows a
// selection, and its inspector on the right, both on the shown frame.
// Several can be open, each docks like any panel, and its close box unpins
// it. A window's ImGui id is fixed per stage (pin.<stage id>), so a stage
// pinned again returns where its window was. The pinned set is not
// persisted.
class PinnedStageWindows
{
public:
	PinnedStageWindows(VideoPreviewPanel* previewPanel, Scene3dPanel* scenePanel);
	~PinnedStageWindows();

	// Opens the stage's window, or brings it to the front when it is open
	void pin(const PipelineStageId& stage);
	// False when the stage was not pinned
	bool unpin(const PipelineStageId& stage);
	void unpinAll();
	// In the order they were pinned
	std::vector<PipelineStageId> getPinned() const;

	// The stable part of a pinned stage's window name, the part after "###",
	// which the gui automation namespace addresses it by
	static std::string getWindowId(const PipelineStageId& stage);

	// Draws every pinned stage's window, after unpinning the stages the
	// project no longer has. The camera views draw nothing the wizards read,
	// and the scene views draw no gizmos and never look through the camera.
	// Returns the stage a sub-stage row asked to select, an invalid id
	// otherwise.
	PipelineStageId draw(const PipelineStatusInputs& inputs, const StageView::Frame& frame, IStageContent* content);

private:
	struct Entry
	{
		PipelineStageId stage;
		// Front the window on its next draw
		bool bFocus= true;
		// Created the first time the window shows the scene
		std::unique_ptr<SceneViewState> sceneView;
	};

	// The window's view: the cameras or the scene, as the stage asks
	void drawView(Entry& entry, const PipelineStatusInputs& inputs, const StageView::Frame& frame);

	VideoPreviewPanel* m_previewPanel;
	Scene3dPanel* m_scenePanel;
	std::vector<std::unique_ptr<Entry>> m_entries;
};
