#pragma once

#include <cstdint>
#include <string>
#include <vector>

// One ImGui viewport of the main window's context, in plain types so the
// automation server can list and address the OS windows panels are torn out into
struct GuiViewportInfo
{
	uint32_t viewportId= 0;
	// The SDL window id the viewport renders into
	uint32_t platformWindowId= 0;
	bool bIsMainViewport= false;
	// Desktop pixels when multi-viewports are on, window pixels otherwise
	int x= 0;
	int y= 0;
	int width= 0;
	int height= 0;
	// The stable id of each window drawn into the viewport last frame (the part
	// of its ImGui name after "###", or the whole name without one)
	std::vector<std::string> windowIds;
};

// The ImGui internals the automation server's gui namespace needs, kept here so
// imgui_internal.h stays out of the server. Everything runs on the main thread
// against the one ImGui context.
namespace GuiViewports
{
// The context's viewports, the main viewport first. Empty with no ImGui context.
std::vector<GuiViewportInfo> getViewports();

// Undocks the window whose ImGui name is windowId or ends in "###windowId",
// drawn last frame, and queues a move to a desktop position and an optional
// size (zero keeps the size). With multi-viewports on, a position outside the
// main window gives the panel its own OS window.
bool requestFloatWindow(const std::string& windowId, int x, int y, int width, int height, std::string& outError);

// Applies the queued requestFloatWindow moves. Called right after NewFrame,
// before any window begins, so a moved window is placed by the Begin that follows.
void applyPendingFloatRequests();
} // namespace GuiViewports
