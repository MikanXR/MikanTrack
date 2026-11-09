# Proposal: tear-out panels and the automation server

Design decided and built, 2026-11-08. Panels of the main window drag out into their own OS windows and dock back, as MikanXR's do since its pull request 79, and a loopback automation server drives and captures the UI by script. The work landed as two pull requests off `main`, the server first (MikanTrack pull requests 9 and 10). This document is the record of the design: what was already in place, the decisions, the pieces, and what verified them. The living descriptions are in [architecture.md](../reference/architecture.md) (the UI and Render section) and [automation.md](../reference/automation.md), with the flags and the test in [commands.md](../reference/commands.md), [testing.md](../reference/testing.md), and [debugging.md](../reference/debugging.md).

## Goal

A user with two monitors puts the 3D Scene or the Video Preview on the second one while the tracking panels stay in the main window. Any panel tears out by dragging its tab past the main window's edge, lives in its own OS window with its own taskbar entry, and docks back by dragging it onto the dockspace. Closing a torn-out window hides the panel, and the View menu brings it back. A drive script can list the torn-out windows, float a panel by command, click into it, and capture it.

## What was already in place

MikanTrack carried the pieces MikanXR had to build, which is why its port was the smaller one.

- Dear ImGui is the docking branch at 1.92.9, the same submodule commit MikanXR uses, with the stock SDL2 and OpenGL3 backends compiled into the exe and called directly from `App::startup` and `App::tick`. `ImGuiConfigFlags_DockingEnable` was set, `ImGuiConfigFlags_ViewportsEnable` was not.
- The SDL2 backend creates each torn-out window's GL context sharing objects with the main window's (`SDL_GL_SHARE_WITH_CURRENT_CONTEXT`) and sets its swap interval to 0, so the panels' texture ids resolve there and the 90 Hz frame cap is not multiplied by one vsync wait per window.
- `Scene3dPanel` renders into a `GlFrameBuffer` and presents it with `ImGui::Image`, reading the mouse through `io.MouseDelta`, `io.MousePos`, and `IsItemHovered`, which are desktop-space when viewports are on. `VideoPreviewPanel` presents `GlTexture`s through the window draw list. The wizards draw their overlays into the preview's draw list. The modals center on the main viewport. The hotkeys read `ImGui::IsKeyPressed`. None of that changed.
- Every SDL event already reached `ImGui_ImplSDL2_ProcessEvent`, and the quit check is keyed to the main window's id. MikanXR's event-routing work (the enter events its window wrapper swallowed, the child-window claim) has no counterpart here because there is no wrapper.

What was missing was the viewports flag, the platform-window pass in the frame loop, close buttons on seven panels, and the tool that drives it.

## Decisions

- Every panel tears out. Multi-viewports are a property of the ImGui context, so every floating window, the wizards included, becomes an OS window when it leaves the main window. ImGui offers no per-window opt-out, and none is wanted.
- Every panel gets a View menu toggle and a close flag. Seven were new: Video Preview, 3D Scene, Device, Tracking, Calibration, OSC Output, Timeline. A torn-out window's close request is acted on by `ImGui::Begin` only for a window drawn with `p_open`, and a window drawn without one has no close button in its chrome at all. Visibility is not persisted and every panel defaults to shown, as the four existing toggles (Log, Settings, Avatar, Audio) did.
- Window chrome stays at ImGui's defaults, as in MikanXR: ImGui draws the title bar on a borderless OS window, each torn-out window gets a taskbar entry, and a floating panel becomes an OS window only once it crosses the main window's edge. No `ConfigViewports*` overrides.
- The layout stays in `imgui.ini` next to the exe (`io.IniFilename` untouched). Torn-out positions persist there in desktop pixels. A position saved on a monitor that is no longer present is clamped by ImGui onto its fallback monitor, the main viewport's. Untracking the root copy stays on the pre-open-sourcing cleanup item in plan.md.
- Per-monitor DPI is deferred, as a plan.md Later item. MikanTrack has no UI scaling at all (a fixed 16 px font, no content-scale query), so a torn-out window on a monitor with a different scale draws at the main window's pixel size, the same limitation MikanXR carries.
- Verification is the automation server, ported from MikanXR as a loopback TCP text channel. The socket and the protocol framing are copied code (NOTICE.md entries). The log ring is the existing `LogPanel` buffer rather than a copy of MikanXR's.
- The first namespace set is the drive-and-verify set: `help`, `app`, `gui`, `input`, `screenshot`, `log`. No config introspection in this round.
- The listener is enabled by flags only: `--automation-server` opens it for that launch and `--automation-port=<n>` overrides the port and implies it. Off otherwise, nothing persisted, no panel section. The default port is 21140, clear of MikanXR's 21120 and the VNyan automation plugin's 21130, so all three listen on one machine.
- Two pull requests off `main`, the automation server first, so the tear-out branch was driven with the tool from its first build.

## Pieces

### Pull request 1: the automation server

1. **The sources.** `src/Automation/`, a new source directory. `AutomationSocket` and `AutomationProtocol` are copied from MikanXR's `src/Editor/Interprocess/AutomationSocket.*` and `src/Editor/Server/AutomationProtocol.*`, with NOTICE.md entries.
2. **The server.** `AutomationServer`: the namespace registry, the framed reply, `poll`, the listener, and the deferred reply a capture answers with a frame later. The protocol is MikanXR's: newline-terminated `<namespace> [verb] [args...]` lines, quoted spans, one framed reply per command (a line count, then that many lines), errors as `<namespace> <verb>: <message>`.
3. **The `app` namespace.** `info`, `open`, `new`, `resume`, `close`, and `quit`, routed to the app's own project and shutdown requests, which land at the top of the next tick.
4. **The `gui` namespace.** `viewports` lists each viewport with its desktop rect and the stable ids of the panels drawn in it, and `float` undocks a panel and queues a move to a desktop position, applied right after the next `NewFrame`. Both need `imgui_internal.h`, so they live in the `GuiViewports` helper in `src/UI`, keeping the internal API out of the server.
5. **The `input` namespace.** `move`, `click`, `press`, `release`, `wheel`, `key`, `keydown`, `keyup`, `text`, each aimed at the main window or at `viewport <viewportId>`. Every verb raises the target window, warps the real cursor, and pushes an SDL event stamped with that window's id, so the input travels the queue a user's does.
6. **The `screenshot` namespace.** `window` reads the main window's back buffer after `RenderDrawData` and before the swap, `viewport` a torn-out window's between its render and its swap. PNG through `cv::imwrite`, the absolute path as the reply.
7. **The `log` namespace.** `tail` from `LogPanel`'s ring, oldest first, filtered by severity name.
8. **The flags.** `--automation-server` and `--automation-port=<n>`, read in `App::exec`, the listener opened after `MainWindow` exists, a failed bind logged with the app running on. The server is polled after the SDL events and before `NewFrame`.
9. **The client.** `tools/automate.py`, copied from MikanXR with the default port 21140: positional commands, `ready`, `sleep:<s>`, `until:<command>:<expected>`.
10. **The test.** `--test-automation`: the tokenizer and the framing round trip, and a loopback session on an ephemeral port against a server polled in-process.
11. **Docs.** `docs/reference/automation.md`, listed in CLAUDE.md's doc table, with the flags and the test in commands.md, testing.md, and debugging.md.

### Pull request 2: tear-out panels

1. **The flag.** `App::startup` sets `ImGuiConfigFlags_ViewportsEnable` beside the docking flag, before the first `NewFrame`, which is when the ini restores the torn-out windows' positions.
2. **The pass.** `App::renderPlatformWindows`, after `ImGui_ImplOpenGL3_RenderDrawData` and before the main window's swap: `UpdatePlatformWindows`, then the viewports walked as MikanXR's `MkGuiContext::renderPlatformWindows` does (render each, service a pending `screenshot viewport` capture with that context current, then swap each), then `SDL_GL_MakeCurrent` back to the main window and context.
3. **The close flags.** The seven panels take a `bool* pOpen` first, as Avatar, Audio, and Log already did, and the Settings panel's existing toggle reaches its `Begin` too. `MainWindow` holds the show flags with the View menu items. A hidden panel's draw is skipped, with these consequences handled: the Video Preview is shown again as well as focused when a camera calibration wizard starts, and `VideoPreviewPanel`'s last draw list reads null while it is hidden so the wizard overlays skip rather than draw into a stale list.
4. **The strings.** Seven `mainWindow.view*Panel` strings in `en.json` and `ja.json`, the Japanese machine-translated and listed for human review in plan.md's Later group.
5. **Docs.** The paragraph in architecture.md's UI and Render section, the README feature bullet, automation.md's viewport semantics and tear-out round trip, and plan.md's per-monitor DPI and Japanese review items.

## Verification

Pull request 1: `--test-automation`, the full self-test batch, and one live session (`app info`, `log tail`, `screenshot window`, and the Log panel floated to a position and captured).

Pull request 2, driven through the automation server on 2026-11-08 and then checked by hand the same day.

- The drive: `gui float "3D Scene"` to a position outside the main window produced a second viewport, `screenshot viewport` captured its client area with the scene rendering, `input click viewport` landed, the close box clicked through `input click viewport` cleared the panel, the View menu item brought it back at its torn-out position, a relaunch restored the torn-out layout from `imgui.ini`, and the panel dragged back onto the center dock with `input press viewport`, `input move` steps into the main window, and `input release` (automation.md's tear-out round trip). Docking back was expected to stay a hand check, since `gui float` only undocks, and turned out to script.
- The hand checks: each of the eleven panels dragged out and back, Alt+F4 on a torn-out window closing the panel and not the app, a wizard window torn out keeping its overlay on the video preview, and the frame rate holding with two torn-out windows.
