# Proposal: tear-out panels and the automation server

Panels of the main window drag out into their own OS windows and dock back, as MikanXR's do since its pull request 79. A loopback automation server lands first so the tear-out is driven and captured by script rather than verified by hand alone. This document records the design decided on 2026-11-08: what is already in place, the decisions, the pieces in order, and what verifies each one. The work lands as two pull requests off `main`, the server first. The reference docs it touches are [architecture.md](../reference/architecture.md), [commands.md](../reference/commands.md), [testing.md](../reference/testing.md), [debugging.md](../reference/debugging.md), and a new `docs/reference/automation.md`.

## Goal

A user with two monitors puts the 3D Scene or the Video Preview on the second one while the tracking panels stay in the main window. Any panel tears out by dragging its tab past the main window's edge, lives in its own OS window with its own taskbar entry, and docks back by dragging it onto the dockspace. Closing a torn-out window hides the panel, and the View menu brings it back. A drive script can list the torn-out windows, float a panel by command, click into it, and capture it.

## What is already in place

MikanTrack carries the pieces MikanXR had to build, which is why its port is the smaller one.

- Dear ImGui is the docking branch at 1.92.9, the same submodule commit MikanXR uses, with the stock SDL2 and OpenGL3 backends compiled into the exe and called directly from `App::startup` and `App::tick`. `ImGuiConfigFlags_DockingEnable` is set, `ImGuiConfigFlags_ViewportsEnable` is not.
- The SDL2 backend creates each torn-out window's GL context sharing objects with the main window's (`SDL_GL_SHARE_WITH_CURRENT_CONTEXT`) and sets its swap interval to 0, so the panels' texture ids resolve there and the 90 Hz frame cap is not multiplied by one vsync wait per window.
- `Scene3dPanel` renders into a `GlFrameBuffer` and presents it with `ImGui::Image`, reading the mouse through `io.MouseDelta`, `io.MousePos`, and `IsItemHovered`, which are desktop-space when viewports are on. `VideoPreviewPanel` presents `GlTexture`s through the window draw list. The wizards draw their overlays into the preview's draw list. The modals center on the main viewport. The hotkeys read `ImGui::IsKeyPressed`. None of that changes.
- Every SDL event already reaches `ImGui_ImplSDL2_ProcessEvent`, and the quit check is keyed to the main window's id. MikanXR's event-routing work (the enter events its window wrapper swallowed, the child-window claim) has no counterpart here because there is no wrapper.

What is missing is the viewports flag, the platform-window pass in the frame loop, close buttons on seven panels, and the tool that drives it.

## Decisions

- Every panel tears out. Multi-viewports are a property of the ImGui context, so every floating window, the wizards included, becomes an OS window when it leaves the main window. ImGui offers no per-window opt-out, and none is wanted.
- Every panel gets a View menu toggle and a close flag. Seven are new: Video Preview, 3D Scene, Device, Tracking, Calibration, OSC Output, Timeline. A torn-out window's close request is acted on by `ImGui::Begin` only for a window drawn with `p_open`, and a window drawn without one has no close button in its chrome at all. Visibility is not persisted and every panel defaults to shown, as the four existing toggles (Log, Settings, Avatar, Audio) do today.
- Window chrome stays at ImGui's defaults, as in MikanXR: ImGui draws the title bar on a borderless OS window, each torn-out window gets a taskbar entry, and a floating panel becomes an OS window only once it crosses the main window's edge. No `ConfigViewports*` overrides.
- The layout stays in `imgui.ini` next to the exe (`io.IniFilename` untouched). Torn-out positions persist there in desktop pixels. A position saved on a monitor that is no longer present is clamped by ImGui onto its fallback monitor, the main viewport's. Untracking the root copy stays on the pre-open-sourcing cleanup item in plan.md.
- Per-monitor DPI is deferred to plan.md's Later group at the tear-out's doc pass. MikanTrack has no UI scaling at all (a fixed 16 px font, no content-scale query), so a torn-out window on a monitor with a different scale draws at the main window's pixel size, the same limitation MikanXR carries.
- Verification is the automation server, ported from MikanXR as a loopback TCP text channel. The socket and the protocol framing are copied code (NOTICE.md entries). The log ring is the existing `LogPanel` buffer rather than a copy of MikanXR's.
- The first namespace set is the drive-and-verify set: `help`, `app`, `gui`, `input`, `screenshot`, `log`. No config introspection in this round.
- The listener is enabled by flags only: `--automation-server` opens it for that launch and `--automation-port=<n>` overrides the port and implies it. Off otherwise, nothing persisted, no panel section. The default port is 21140, clear of MikanXR's 21120 and the VNyan automation plugin's 21130, so all three listen on one machine.
- Two pull requests off `main`, the automation server first, so the tear-out branch is driven with the tool from its first build.

## Pieces in order

### Pull request 1: the automation server

1. [x] `src/Automation/`, a new source directory (CMake glob, sources, source_group, include dir, per the build gotcha). `AutomationSocket` and `AutomationProtocol` are copied from MikanXR's `src/Editor/Interprocess/AutomationSocket.*` and `src/Editor/Server/AutomationProtocol.*` with NOTICE.md entries. Winsock is already linked (`ws2_32`).
2. [x] `AutomationServer` (`src/Automation/AutomationServer.h`/`.cpp`): the namespace registry, the framed reply, `poll`, `initialize(App*, MainWindow*)`, `startListener(port)`, `shutdown`. The protocol is MikanXR's: newline-terminated `<namespace> [verb] [args...]` lines, quoted spans, one framed reply per command (a line count, then that many lines), errors as `<namespace> <verb>: <message>`.
3. [x] The `app` namespace: `info` replies the app state (`MainMenu` or `Project`) and the project path or `none`; `open <projectPath>`, `new <projectName>`, `resume`, `close`, and `quit` route to `App::requestLoadProject`, `activateNewProject`, the main menu's resume path, `requestCloseProject`, and `requestShutdown`. Project actions land at the top of the next tick, so a drive polls `app info` after them. `app new` starts the guided setup flow's modal, as the menu does.
4. [x] The `gui` namespace: `viewports` replies `<viewportId> <x>,<y> <width>x<height> <windowIds...>` per viewport, the main one first, each window id the stable part of a panel's ImGui name (the English title after `###`, quoted when it holds a space). `float <windowId> <x> <y> [<width> <height>]` undocks a panel drawn last frame and queues a move to a desktop position, applied right after the next `NewFrame`. Both need `imgui_internal.h`, so they live in a small `GuiViewports` helper in `src/UI` that the server calls, keeping the internal API out of the server.
5. [x] The `input` namespace: `move`, `click`, `press`, `release`, `wheel`, `key`, `keydown`, `keyup`, `text`, each aimed at the main window or at `viewport <viewportId>`. Every verb raises the target window, warps the real cursor with `SDL_WarpMouseInWindow`, and pushes an SDL event stamped with that window's id through `SDL_PushEvent`, so the input travels the queue a user's does and the backend routes it. MikanXR's caveats carry: a warp often reports a zero relative delta, a `key` press and release land in one frame, and a click into a different OS window needs two moves there first.
6. [x] The `screenshot` namespace: `window [path]` reads the main window's back buffer after `RenderDrawData` and before the swap; `viewport <viewportId> [path]` reads a torn-out window's back buffer between its render and its swap in the platform-window pass (pull request 2 adds the pass, so in this request the verb answers an error for every id but the main viewport's). PNG through `cv::imwrite`, the absolute path as the reply, delivered the frame the capture lands.
7. [x] The `log` namespace: `tail <lineCount> [minLevel]` from `LogPanel`'s ring (2000 lines today), oldest first, with the `LogSeverityLevel` names (`trace` to `fatal`). `LogPanel` gains a locked copy-out of its last lines.
8. [x] The flags: `App::exec` reads `--automation-server` and `--automation-port=<n>` from `argv`, and `startup` opens the listener after `MainWindow` exists. A failed bind logs an error and the app runs on. `App::tick` polls the server after the SDL events and before `NewFrame`, so a queued `gui float` applies this frame and an injected event is consumed by the next tick's event loop.
9. [x] `tools/automate.py`, copied from MikanXR with the default port 21140 and a MikanTrack docstring: positional commands, `ready`, `sleep:<s>`, `until:<command>:<expected>`.
10. [x] `--test-automation`: the tokenizer and the framing round trip, and a loopback session on an ephemeral port against a server polled in-process (`help` and an unknown namespace), deterministic and hardware-free.
11. [x] Docs: a new `docs/reference/automation.md` (enabling, protocol, the six namespaces, the client helper, the drive-and-verify loop) listed in CLAUDE.md's doc table, the flags and the test in commands.md, the test in testing.md's App plumbing group, a pointer in debugging.md, the copied sources in NOTICE.md.

### Pull request 2: tear-out panels

1. [ ] `App::startup` sets `ImGuiConfigFlags_ViewportsEnable` beside the docking flag, before the first `NewFrame`, which is when the ini restores the torn-out windows' positions.
2. [ ] `App::tick` runs the platform-window pass after `ImGui_ImplOpenGL3_RenderDrawData`: `UpdatePlatformWindows`, then the viewports walked as MikanXR's `MkGuiContext::renderPlatformWindows` does (render each, service a pending `screenshot viewport` capture with that context current, then swap each), then `SDL_GL_MakeCurrent` back to the main window and context. The pass runs every frame `NewFrame` ran, since ImGui asserts otherwise.
3. [ ] The seven panels take a `bool* pOpen` (`DevicePanel::draw`, `SettingsPanels::drawTrackingPanel` and `drawOscPanel`, `CalibrationPanel::draw`, `TimelinePanel::draw`, `Scene3dPanel::draw`, `VideoPreviewPanel::draw`) and `MainWindow` holds their show flags with seven View menu items. A hidden panel's draw is skipped, with these consequences handled: the Video Preview is shown again as well as focused when a camera calibration wizard starts, `VideoPreviewPanel`'s last draw list reads null while it is hidden so the wizard overlays and the hand overlay skip rather than draw into a stale list, and `Scene3dPanel`'s capture path keeps working without the panel drawn, as `--pose-report` already relies on.
4. [ ] Seven `mainWindow.view*Panel` strings in `en.json` and `ja.json`, the Japanese machine-translated and listed for human review in plan.md's Later group with the other such items.
5. [ ] The `screenshot viewport` verb from pull request 1 is serviced by the pass in piece 2, and `gui viewports` lists the torn-out windows.
6. [ ] Docs: a paragraph in architecture.md's UI and Render section (viewports, the platform pass and its shared GL contexts, the close semantics, where the layout persists), a sentence in README.md's usage, automation.md's `gui`, `input ... viewport`, and `screenshot viewport` sections, and plan.md: the per-monitor DPI item in Later, the Japanese review item in Later, and the live-verify item in Now.

## Verification

Pull request 1 is verified by `--test-automation` and one live session: launch with `--automation-server`, then `python tools/automate.py ready "app info" "log tail 5" "screenshot window"`. The full self-test batch (testing.md) runs before each commit.

Pull request 2 is driven and then checked by hand.

- The drive: `app resume`, `gui viewports` (one viewport), `gui float "3D Scene" <x> <y>` with a position outside the main window, `until:gui viewports:` showing two, `screenshot viewport <id>`, `input click viewport <id> <x> <y>` on the scene image, `log tail`. Docking back is a drag onto a dock target, which `gui float` does not perform, so it stays a hand check.
- The hand checks, entering plan.md as a live-verify item: drag each panel out and back, close a torn-out window with its X and reopen it from View, Alt+F4 on a torn-out window closes the panel and not the app, a torn-out layout is restored after a restart, a wizard window torn out keeps its overlay on the video preview, and the frame rate holds with two torn-out windows.

## Open questions

None. The questions raised in the interview are settled above.
