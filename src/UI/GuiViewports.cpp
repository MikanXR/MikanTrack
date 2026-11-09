#include "GuiViewports.h"

#include <cstring>

#include "imgui.h"
#include "imgui_internal.h"

namespace
{
struct PendingFloatRequest
{
	std::string windowName;
	int x= 0;
	int y= 0;
	int width= 0;
	int height= 0;
};

std::vector<PendingFloatRequest> g_pendingFloatRequests;

// The part of a window's ImGui name its ID hashes from: what follows "###" when
// the name carries one, the whole name otherwise
const char* getStableWindowId(const ImGuiWindow* window) { return ImHashSkipUncontributingPrefix(window->Name); }

// A window the user sees as a panel, drawn last frame: not a child region, dock
// node host or popup, not a fullscreen host drawn without a title bar (the
// dockspace host, the main menu screen), and not one of the "##" names that
// carry no title
bool isListedWindow(const ImGuiWindow* window)
{
	if (!window->WasActive)
		return false;
	if ((window->Flags & ImGuiWindowFlags_ChildWindow) && !window->DockIsActive)
		return false;
	if (window->Flags
		& (ImGuiWindowFlags_DockNodeHost | ImGuiWindowFlags_Popup | ImGuiWindowFlags_Tooltip
		   | ImGuiWindowFlags_ChildMenu | ImGuiWindowFlags_NoTitleBar))
		return false;

	const char* stableId= getStableWindowId(window);
	return stableId[0] != 0 && strncmp(stableId, "##", 2) != 0;
}
} // namespace

namespace GuiViewports
{
std::vector<GuiViewportInfo> getViewports()
{
	std::vector<GuiViewportInfo> viewportInfos;
	ImGuiContext* context= ImGui::GetCurrentContext();
	if (context == nullptr)
		return viewportInfos;

	const ImGuiViewport* mainViewport= ImGui::GetMainViewport();
	for (ImGuiViewportP* viewport : context->Viewports)
	{
		GuiViewportInfo info;
		info.viewportId= viewport->ID;
		info.platformWindowId= (uint32_t)(intptr_t)viewport->PlatformHandle;
		info.bIsMainViewport= viewport == mainViewport;
		info.x= (int)viewport->Pos.x;
		info.y= (int)viewport->Pos.y;
		info.width= (int)viewport->Size.x;
		info.height= (int)viewport->Size.y;

		for (ImGuiWindow* window : context->Windows)
		{
			if (window->Viewport == viewport && isListedWindow(window))
				info.windowIds.push_back(getStableWindowId(window));
		}

		// The main viewport leads, whatever order ImGui keeps them in
		if (info.bIsMainViewport)
			viewportInfos.insert(viewportInfos.begin(), info);
		else
			viewportInfos.push_back(info);
	}

	return viewportInfos;
}

bool requestFloatWindow(const std::string& windowId, int x, int y, int width, int height, std::string& outError)
{
	ImGuiContext* context= ImGui::GetCurrentContext();
	if (context == nullptr)
	{
		outError= "no GUI";
		return false;
	}

	const std::string stableSuffix= "###" + windowId;
	ImGuiWindow* matchedWindow= nullptr;
	for (ImGuiWindow* window : context->Windows)
	{
		if (!isListedWindow(window))
			continue;

		const std::string windowName= window->Name;
		const bool bEndsWithStableId=
			windowName.size() >= stableSuffix.size()
			&& windowName.compare(windowName.size() - stableSuffix.size(), stableSuffix.size(), stableSuffix) == 0;
		if (windowName == windowId || bEndsWithStableId)
		{
			matchedWindow= window;
			break;
		}
	}

	if (matchedWindow == nullptr)
	{
		outError= "no open window '" + windowId + "'";
		return false;
	}

	PendingFloatRequest request;
	request.windowName= matchedWindow->Name;
	request.x= x;
	request.y= y;
	request.width= width;
	request.height= height;
	g_pendingFloatRequests.push_back(request);

	return true;
}

void applyPendingFloatRequests()
{
	if (g_pendingFloatRequests.empty())
		return;

	ImGuiContext* context= ImGui::GetCurrentContext();
	if (context == nullptr)
	{
		g_pendingFloatRequests.clear();
		return;
	}

	const bool bViewportsEnabled= (context->IO.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) != 0;

	for (const PendingFloatRequest& request : g_pendingFloatRequests)
	{
		ImGuiWindow* window= ImGui::FindWindowByName(request.windowName.c_str());
		if (window == nullptr)
			continue;

		// The same undock a tab dragged out of its node gets
		ImGui::DockContextProcessUndockWindow(context, window, true);

		const ImVec2 desktopPos((float)request.x, (float)request.y);
		ImGui::SetWindowPos(window, desktopPos, ImGuiCond_Always);
		if (request.width > 0 && request.height > 0)
			ImGui::SetWindowSize(window, ImVec2((float)request.width, (float)request.height), ImGuiCond_Always);

		// With multi-viewports on, a window that already owns its viewport carries
		// it to the new position. One still in the main viewport would only be
		// clamped back to that viewport's edge, so it drops the viewport and takes
		// the path a window restored from the ini takes: its next Begin creates a
		// viewport at the saved position, which stays its own unless the window
		// fits inside the main window.
		if (bViewportsEnabled)
		{
			const bool bOwnsViewport= window->Viewport != nullptr && window->Viewport->Window == window;
			if (!bOwnsViewport)
			{
				window->Viewport= nullptr;
				window->ViewportId= window->ID;
				window->ViewportPos= desktopPos;
			}
		}
	}

	g_pendingFloatRequests.clear();
}
} // namespace GuiViewports
