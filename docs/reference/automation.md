# Automation server

How to drive and inspect a running `MikanTrack.exe` over the automation server: a loopback TCP text channel for test scripts and AI tooling. It complements the headless self-tests and tools ([testing.md](./testing.md), [commands.md](./commands.md)), which cover code paths that need no running app. The server lives in `src/Automation/AutomationServer` over the line socket in `src/Automation/AutomationSocket`, with the pure text protocol in `src/Automation/AutomationProtocol`. `App::tick` polls it once per frame on the main thread, between frames, so commands run synchronously against app state. The protocol and the client script are MikanXR's, so a drive written for one app reads the same in the other (`NOTICE.md`).

---

## Enabling and port

The server is off by default. The channel drives and inspects the app, so any local process that reaches it owns the session, and a shipped session has no reason to be listening. It binds 127.0.0.1 only, and nothing persists it: there is no setting and no panel.

- `--automation-server` on the command line opens the listener for that launch
- `--automation-port=<n>` overrides the port and implies `--automation-server`, since a port with nothing listening is a trap

The default port is 21140, clear of MikanXR's 21120 and the VNyan automation plugin's 21130, so all three listen on one machine. A failed bind logs an error and the app runs on without the channel.

## Protocol

One client at a time. Commands are newline-terminated text lines (an optional trailing `\r` is stripped). Tokens split on spaces and tabs; a double-quoted span groups spaces into one token, with `\"` and `\\` escapes inside quotes.

Every command answers with one framed reply: a line holding the reply's content line count, then exactly that many content lines. A single value is `1` then the value. A success with nothing to say is just `0`. An error is `1` then one line `<namespace> <verb>: <message>`. An unknown namespace replies naming the valid namespaces. Nothing falls through silently, so a client awaits each reply and stays in sync.

Project actions land at the top of the next tick, the same deferral the UI's own Load Project and Close Project take, so a drive that opens or closes a project polls `app info` before acting on the new state.

## Commands

`help` lists every registered command.

### App control (app)

- `app info` replies two lines: `state MainMenu` or `state Project`, then `project <path>` for the loaded project file or `project none`
- `app open <projectPath>` loads a `project.json` on the next tick, as the File menu's Load Project does, and answers an error for a path that does not exist
- `app new <projectName>` creates a project folder under the projects root, loads it, and starts the guided setup flow's first prompt, as the main menu's New Project does
- `app resume` loads the last project, as the main menu's Resume does, and answers an error when there is none or its file is gone
- `app close` returns to the main menu on the next tick
- `app quit` requests a clean shutdown, the same path as the window close button. The reply goes out before the socket closes.

### Panels and torn-out windows (gui)

A panel dragged past the main window's edge becomes its own OS window, which ImGui calls a viewport. These windows are addressed by ImGui viewport id.

- `gui viewports` replies `<viewportId> <x>,<y> <width>x<height> <windowIds...>` per viewport, main viewport first. The rect is in desktop pixels when multi-viewports are on, and each window id is the stable part of a panel's ImGui name (the English title after `###`), quoted when it holds a space, so it pastes straight into `gui float`. Without multi-viewports there is one viewport, the main window, listing every panel drawn last frame.

- `gui float <windowId> <x> <y> [<width> <height>]` undocks a panel drawn last frame and moves it to a position, which is how a drive reaches the state a hand drag would produce. With multi-viewports on, a position outside the main window gives the panel its own OS window. The move applies right after the next frame's `NewFrame`, so poll `gui viewports` after it. The ids are the panel titles:
	- `"Video Preview"`
	- `"3D Scene"`
	- `Device`
	- `Tracking`
	- `Settings`
	- `Audio`
	- `Calibration`
	- `"OSC Output"`
	- `Avatar`
	- `Log`
	- `Timeline`

### Synthetic input (input)

Injected input is posted to SDL's own event queue stamped with the target window's id, so it travels the path a user's input does: the same backend, the same ImGui routing. A driven session is therefore evidence about the shipping input path rather than about a test-only shortcut. Every verb raises the target window first, since input only lands where the OS is delivering it.

Mouse position is warped rather than fabricated, which moves the real cursor. ImGui's backend re-reads the OS cursor every frame whenever the pointer is not over one of our windows, and would overwrite a fabricated position before the widget under it saw the click.

The target is the main window, with positions in its client pixels. `input <verb> viewport <viewportId> ...` is every verb below aimed at a torn-out panel's OS window instead, with `viewport <viewportId>` from `gui viewports` in front of the verb's own arguments and positions in that window's client pixels.

- `input move <x> <y>` moves the cursor to a window client position
- `input click <x> <y> [left|middle|right] [clickCount]` moves, then presses and releases. Pass `2` as the click count for a double click.
- `input press|release <x> <y> [left|middle|right]` are the halves of a click, for drags and for widgets whose popup only survives while the button is held
- `input wheel <x> <y> <scrollY> [scrollX]` scrolls in notches
- `input key <keyName> [modifiers...]` presses and releases a key, and `input keydown` / `input keyup` are its halves, so a drive can hold a modifier across a mouse press, move, and release (`input keydown ctrl ctrl`, then `input keyup ctrl`; the modifier key names are `ctrl`, `shift`, `alt`). Names are `return`/`enter`, `tab`, `escape`, `backspace`, `delete`, `space`, the arrows, `home`, `end`, `pageup`, `pagedown`, `insert`, `f1` to `f12`, or a single printable character. Modifiers are `shift`, `ctrl`, `alt`, `gui`.
- `input text <text...>` commits text the way typing does, taking the raw untokenized rest of the line

Two things a warp cannot fake. A warped move often reports a zero relative delta, so anything driven by relative motion rather than by cursor position is unreliable under synthetic input. The 3D Scene's orbit reads ImGui's per-frame mouse delta, which is computed from successive cursor positions, so a `press`, a `move`, and a `release` across frames do orbit it. A `input key` press and release land in the same frame, so held-key behavior never accumulates.

After a drive works in one OS window (the main window or a torn-out panel), a click into another that follows a single `input move` is lost. Move twice in the new window (`input move`, `sleep:0.5`, `input move` a pixel over, `sleep:0.5`) before the first click there.

One caveat worth knowing when a click seems to vanish: an ImGui window's capture flags are computed at `NewFrame` and so describe the previous frame's cursor position. A click injected in the same command that moved the cursor is judged against where the cursor used to be.

### Screenshots (screenshot)

- `screenshot window [path]` writes the main window's back buffer (the whole UI) at the end of that frame's render, replying the absolute path once the capture lands. Default `mikantrack_window.png` in the working directory, overwritten each capture.
- `screenshot viewport <viewportId> [path]` writes a torn-out panel's OS window the same way, read between its render and its swap. The id comes from `gui viewports`, and the image is that window's client area, so its pixels are the coordinates `input ... viewport` takes. The main viewport's id captures the main window. A viewport that does not render that frame (closed, hidden, or minimized) answers with an error, which is also the answer for every torn-out viewport until the multi-viewport render pass lands. Default `mikantrack_viewport.png`.

### Log access (log)

`log tail <lineCount> [minLevel]` replies the most recent log lines at or above the level, oldest first, from the Log panel's ring of the last 2000 lines. Levels are the logger's names:

- trace
- debug
- info
- warning
- error
- fatal

## Client helper

`tools/automate.py` is the checked-in client: it connects (retrying a refused port until `--wait` seconds, default 20), sends each argument as one command, prints each framed reply, and exits nonzero on a connection failure or timeout. `--port` overrides the default 21140, `--delay` sets the pause between commands (default 0.1 s), and `--timeout` bounds each reply and `until` poll (default 60 s).

```
python tools/automate.py "app info"
python tools/automate.py "app resume" "until:app info:state Project" "screenshot window"
python tools/automate.py "gui viewports" "gui float \"3D Scene\" 2000 100"
```

Three arguments run client-side instead of being sent:

- `ready` waits until the server answers `app info` (the connect retry already covers a fresh launch)
- `sleep:<seconds>` pauses that long instead of the default delay
- `until:<command>:<expected>` polls the command until a reply line starts with the expected text, so a drive waits for a state edge instead of sleeping out a worst-case budget

## The drive and verify loop

The standard way to verify a UI feature objectively: launch `MikanTrack.exe` with the channel on, drive it with commands, then read state back rather than assuming behavior. The exe runs from its own directory, as always.

```
cd build/Release && ./MikanTrack.exe --automation-server &
python tools/automate.py "app resume" "until:app info:state Project" "screenshot window"
python tools/automate.py "input click 400 300" "log tail 20 error" "app quit"
```

`--test-automation` covers the pieces that need no window: the tokenizer, the reply framing, and a loopback session against a server polled in-process on an ephemeral port. The window-bound namespaces (`gui`, `input`, `screenshot`) are verified by the drive above.

A handler that cannot answer straight away (`screenshot` waits for the frame to render) calls `AutomationServer::deferReply`, then answers later through `sendDeferredReply`. Nothing bounds that wait to the current frame, so a handler that defers owns arming its own answer.

Growth convention: when a feature adds automation commands, register the namespace in `AutomationServer::registerCoreNamespaces` (or from the owning subsystem) and add its section here.
