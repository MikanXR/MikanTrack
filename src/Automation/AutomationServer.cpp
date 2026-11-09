#include "AutomationServer.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <map>

#include "GL/glew.h"
#include "SDL.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "App.h"
#include "AutomationProtocol.h"
#include "AutomationSocket.h"
#include "GlobalSettings.h"
#include "GuiViewports.h"
#include "LogPanel.h"
#include "Logger.h"
#include "PathUtils.h"

namespace
{
bool parseIntToken(const std::string& token, int& outValue)
{
	if (token.empty())
		return false;

	char* end= nullptr;
	const long parsed= strtol(token.c_str(), &end, 10);
	if (end == nullptr || *end != '\0')
		return false;

	outValue= (int)parsed;
	return true;
}

// Viewport ids are ImGui ids, listed and taken in hex with or without the 0x prefix
std::string formatViewportId(uint32_t viewportId)
{
	char text[16];
	snprintf(text, sizeof(text), "0x%08X", viewportId);
	return text;
}

bool parseViewportId(const std::string& token, uint32_t& outViewportId)
{
	if (token.empty())
		return false;

	char* end= nullptr;
	const unsigned long parsed= strtoul(token.c_str(), &end, 16);
	if (end == nullptr || *end != '\0')
		return false;

	outViewportId= (uint32_t)parsed;
	return true;
}

// A window id goes out quoted when it holds a space or a quote, so a listed id
// pastes straight back into a command
std::string quoteTokenIfNeeded(const std::string& token)
{
	if (token.find_first_of(" \t\"\\") == std::string::npos)
		return token;

	std::string quoted= "\"";
	for (char c : token)
	{
		if (c == '"' || c == '\\')
			quoted+= '\\';
		quoted+= c;
	}
	quoted+= '"';
	return quoted;
}

// Named keys the automation channel can press. A single printable character is
// its own key, so this covers only the keys that have no character form and the
// ones a drive actually reaches for.
bool parseKeyName(const std::string& name, SDL_Keycode& outKey)
{
	static const std::map<std::string, SDL_Keycode> k_namedKeys= {
		{"return", SDLK_RETURN},  {"enter", SDLK_RETURN},   {"tab", SDLK_TAB},
		{"escape", SDLK_ESCAPE},  {"backspace", SDLK_BACKSPACE}, {"delete", SDLK_DELETE},
		{"space", SDLK_SPACE},    {"up", SDLK_UP},          {"down", SDLK_DOWN},
		{"left", SDLK_LEFT},      {"right", SDLK_RIGHT},    {"home", SDLK_HOME},
		{"end", SDLK_END},        {"pageup", SDLK_PAGEUP},  {"pagedown", SDLK_PAGEDOWN},
		{"insert", SDLK_INSERT},  {"f1", SDLK_F1},          {"f2", SDLK_F2},
		{"f3", SDLK_F3},          {"f4", SDLK_F4},          {"f5", SDLK_F5},
		{"f6", SDLK_F6},          {"f7", SDLK_F7},          {"f8", SDLK_F8},
		{"f9", SDLK_F9},          {"f10", SDLK_F10},        {"f11", SDLK_F11},
		{"f12", SDLK_F12},        {"ctrl", SDLK_LCTRL},     {"shift", SDLK_LSHIFT},
		{"alt", SDLK_LALT},
	};

	std::string lowered= name;
	std::transform(lowered.begin(), lowered.end(), lowered.begin(),
				   [](unsigned char c) { return (char)std::tolower(c); });

	const auto it= k_namedKeys.find(lowered);
	if (it != k_namedKeys.end())
	{
		outKey= it->second;
		return true;
	}

	// A single printable character presses the key that produces it unshifted
	if (lowered.size() == 1 && lowered[0] > 0x20 && lowered[0] < 0x7F)
	{
		outKey= (SDL_Keycode)lowered[0];
		return true;
	}

	return false;
}

bool parseKeyModifier(const std::string& name, uint16_t& outKeyMod)
{
	if (name == "shift")
		outKeyMod= KMOD_LSHIFT;
	else if (name == "ctrl")
		outKeyMod= KMOD_LCTRL;
	else if (name == "alt")
		outKeyMod= KMOD_LALT;
	else if (name == "gui")
		outKeyMod= KMOD_LGUI;
	else
		return false;

	return true;
}

bool parseMouseButtonName(const std::string& name, uint8_t& outButton)
{
	if (name == "left")
		outButton= SDL_BUTTON_LEFT;
	else if (name == "middle")
		outButton= SDL_BUTTON_MIDDLE;
	else if (name == "right")
		outButton= SDL_BUTTON_RIGHT;
	else
		return false;

	return true;
}

// Injected input is posted to SDL's own queue stamped with the target window's
// id, so it reaches ImGui through the backend the way a user's input does

void injectMouseButton(uint32_t windowId, uint8_t button, bool bPressed, int x, int y, int clickCount)
{
	SDL_Event sdlEvent{};
	sdlEvent.type= bPressed ? SDL_MOUSEBUTTONDOWN : SDL_MOUSEBUTTONUP;
	sdlEvent.button.timestamp= SDL_GetTicks();
	sdlEvent.button.windowID= windowId;
	sdlEvent.button.which= 0;
	sdlEvent.button.button= button;
	sdlEvent.button.state= bPressed ? SDL_PRESSED : SDL_RELEASED;
	sdlEvent.button.clicks= (Uint8)std::clamp(clickCount, 1, 255);
	sdlEvent.button.x= x;
	sdlEvent.button.y= y;
	SDL_PushEvent(&sdlEvent);
}

void injectMouseWheel(uint32_t windowId, int x, int y, int scrollX, int scrollY)
{
	SDL_Event sdlEvent{};
	sdlEvent.type= SDL_MOUSEWHEEL;
	sdlEvent.wheel.timestamp= SDL_GetTicks();
	sdlEvent.wheel.windowID= windowId;
	sdlEvent.wheel.which= 0;
	sdlEvent.wheel.x= scrollX;
	sdlEvent.wheel.y= scrollY;
	sdlEvent.wheel.direction= SDL_MOUSEWHEEL_NORMAL;
	sdlEvent.wheel.preciseX= (float)scrollX;
	sdlEvent.wheel.preciseY= (float)scrollY;
	sdlEvent.wheel.mouseX= x;
	sdlEvent.wheel.mouseY= y;
	SDL_PushEvent(&sdlEvent);
}

void injectKey(uint32_t windowId, SDL_Keycode key, uint16_t keyMod, bool bPressed)
{
	SDL_Event sdlEvent{};
	sdlEvent.type= bPressed ? SDL_KEYDOWN : SDL_KEYUP;
	sdlEvent.key.timestamp= SDL_GetTicks();
	sdlEvent.key.windowID= windowId;
	sdlEvent.key.state= bPressed ? SDL_PRESSED : SDL_RELEASED;
	sdlEvent.key.repeat= 0;
	sdlEvent.key.keysym.sym= key;
	sdlEvent.key.keysym.scancode= SDL_GetScancodeFromKey(key);
	sdlEvent.key.keysym.mod= keyMod;
	SDL_PushEvent(&sdlEvent);
}

void injectText(uint32_t windowId, const std::string& utf8Text)
{
	// SDL_TextInputEvent carries a fixed 32 byte buffer, so longer text goes out
	// as several events. Splitting only on a UTF-8 lead byte keeps multi-byte
	// characters intact across the boundary.
	const size_t k_maxChunkBytes= sizeof(SDL_TextInputEvent::text) - 1;

	size_t offset= 0;
	while (offset < utf8Text.size())
	{
		size_t chunkBytes= std::min(k_maxChunkBytes, utf8Text.size() - offset);
		while (chunkBytes > 1 && offset + chunkBytes < utf8Text.size()
			   && ((unsigned char)utf8Text[offset + chunkBytes] & 0xC0) == 0x80)
		{
			--chunkBytes;
		}

		SDL_Event sdlEvent{};
		sdlEvent.type= SDL_TEXTINPUT;
		sdlEvent.text.timestamp= SDL_GetTicks();
		sdlEvent.text.windowID= windowId;
		memcpy(sdlEvent.text.text, utf8Text.data() + offset, chunkBytes);
		sdlEvent.text.text[chunkBytes]= '\0';
		SDL_PushEvent(&sdlEvent);

		offset+= chunkBytes;
	}
}

// The default framebuffer's back buffer, read before it presents, written as a
// PNG. GL reads bottom-up, so the rows flip for the image.
bool saveBackBufferToPng(int width, int height, const std::string& path)
{
	if (width <= 0 || height <= 0)
		return false;

	cv::Mat image(height, width, CV_8UC4);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_BYTE, image.data);
	if (glGetError() != GL_NO_ERROR)
		return false;

	cv::flip(image, image, 0);
	cv::Mat bgr;
	cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);

	return cv::imwrite(path, bgr);
}

std::string absolutePathText(const std::string& path) { return std::filesystem::absolute(path).string(); }
} // namespace

AutomationServer::AutomationServer()= default;

AutomationServer::~AutomationServer() { shutdown(); }

void AutomationServer::initialize(App* app)
{
	m_app= app;

	registerCoreNamespaces();
}

bool AutomationServer::startListener(uint16_t port)
{
	m_socket= std::make_unique<AutomationSocket>(port);
	m_socket->onLineReceived= [this](const std::string& line) { handleCommandLine(line); };

	if (!m_socket->isListening())
	{
		// A failed bind is tolerated: the app runs fine without the automation
		// channel, it just cannot be driven externally
		m_socket= nullptr;
		return false;
	}

	return true;
}

void AutomationServer::stopListener() { m_socket= nullptr; }

uint16_t AutomationServer::getListenPort() const { return m_socket != nullptr ? m_socket->getBoundPort() : 0; }

void AutomationServer::poll()
{
	if (m_socket != nullptr)
		m_socket->poll();
}

void AutomationServer::shutdown()
{
	m_socket= nullptr;
	m_commandProviders.clear();
	m_app= nullptr;
}

void AutomationServer::registerCommandNamespace(const std::string& namespaceName,
												const std::vector<std::string>& helpLines, CommandHandler handler)
{
	m_commandProviders[namespaceName]= {helpLines, handler};
}

// ---- Dispatch ----------------------------------------------------------------

void AutomationServer::handleCommandLine(const std::string& line)
{
	std::vector<std::string> tokens;
	std::string parseError;
	if (!AutomationProtocol::tokenizeCommandLine(line, tokens, parseError))
	{
		sendErrorReply("parse: " + parseError);
		return;
	}

	// An empty line is a no-op that still answers, so a client stays in sync
	if (tokens.empty())
	{
		sendReply({});
		return;
	}

	const std::string namespaceName= tokens[0];
	auto providerIter= m_commandProviders.find(namespaceName);
	if (providerIter == m_commandProviders.end())
	{
		std::string validNames;
		for (const auto& [name, provider] : m_commandProviders)
		{
			if (!validNames.empty())
				validNames+= ", ";
			validNames+= name;
		}
		sendErrorReply("unknown command '" + namespaceName + "'; valid namespaces: " + validNames);
		return;
	}

	const std::vector<std::string> args(tokens.begin() + 1, tokens.end());
	std::vector<std::string> outLines;
	std::string outError;
	m_currentCommandLine= line;
	m_bReplyDeferred= false;
	if (providerIter->second.handler(args, outLines, outError))
	{
		if (!m_bReplyDeferred)
			sendReply(outLines);
	}
	else
	{
		std::string errorLine= namespaceName;
		if (!args.empty())
			errorLine+= " " + args[0];
		errorLine+= ": " + (outError.empty() ? std::string("failed") : outError);
		sendErrorReply(errorLine);
	}
}

void AutomationServer::sendReply(const std::vector<std::string>& contentLines)
{
	if (m_socket != nullptr)
		m_socket->sendText(AutomationProtocol::frameReply(contentLines));
}

void AutomationServer::sendErrorReply(const std::string& errorLine) { sendReply({errorLine}); }

void AutomationServer::sendDeferredReply(const std::vector<std::string>& contentLines, bool bIsError)
{
	if (bIsError)
		sendErrorReply(contentLines.empty() ? std::string("failed") : contentLines.front());
	else
		sendReply(contentLines);
}

// ---- Built-in command namespaces -----------------------------------------------

void AutomationServer::registerCoreNamespaces()
{
	using namespace std::placeholders;

	registerCommandNamespace("help", {"help"}, std::bind(&AutomationServer::handleHelpCommand, this, _1, _2, _3));

	registerCommandNamespace(
		"app", {"app info", "app open <projectPath>", "app new <projectName>", "app resume", "app close", "app quit"},
		std::bind(&AutomationServer::handleAppCommand, this, _1, _2, _3));

	registerCommandNamespace("gui", {"gui viewports", "gui float <windowId> <x> <y> [<width> <height>]"},
							 std::bind(&AutomationServer::handleGuiCommand, this, _1, _2, _3));

	registerCommandNamespace(
		"input",
		{"input move <x> <y>", "input click <x> <y> [left|middle|right] [clickCount]",
		 "input press|release <x> <y> [left|middle|right]", "input wheel <x> <y> <scrollY> [scrollX]",
		 "input key|keydown|keyup <keyName> [modifiers...]", "input text <text...>",
		 "input <verb> viewport <viewportId> ... (any verb above, in a torn-out panel's window)"},
		std::bind(&AutomationServer::handleInputCommand, this, _1, _2, _3));

	registerCommandNamespace("screenshot", {"screenshot window [path]", "screenshot viewport <viewportId> [path]"},
							 std::bind(&AutomationServer::handleScreenshotCommand, this, _1, _2, _3));

	registerCommandNamespace("log", {"log tail <lineCount> [trace|debug|info|warning|error|fatal]"},
							 std::bind(&AutomationServer::handleLogCommand, this, _1, _2, _3));
}

bool AutomationServer::handleHelpCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
										 std::string& outError)
{
	for (const auto& [name, provider] : m_commandProviders)
	{
		for (const std::string& helpLine : provider.helpLines)
			outLines.push_back(helpLine);
	}

	return true;
}

bool AutomationServer::handleAppCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
										std::string& outError)
{
	if (args.empty())
	{
		outError= "usage: app info|open|new|resume|close|quit";
		return false;
	}

	if (m_app == nullptr)
	{
		outError= "no app";
		return false;
	}

	const std::string& verb= args[0];
	const bool bInProject= m_app->getAppState() == App::eAppState::Project;

	if (verb == "info")
	{
		outLines.push_back(std::string("state ") + (bInProject ? "Project" : "MainMenu"));

		// The loaded project is the last-project pointer, which activateProject
		// moves onto whatever it loads
		const GlobalSettings* globalSettings= m_app->getGlobalSettings();
		if (bInProject && globalSettings->hasLastProjectPath())
			outLines.push_back("project " + PathUtils::pathToUtf8(globalSettings->lastProjectPath));
		else
			outLines.push_back("project none");

		return true;
	}
	else if (verb == "open")
	{
		if (args.size() < 2)
		{
			outError= "usage: app open <projectPath>";
			return false;
		}

		// Applied at the top of the next tick, the path the Load Project menu takes
		const std::filesystem::path projectFile= std::filesystem::absolute(PathUtils::utf8ToPath(args[1]));
		if (!std::filesystem::exists(projectFile))
		{
			outError= "no project file '" + args[1] + "'";
			return false;
		}

		m_app->requestLoadProject(projectFile);
		return true;
	}
	else if (verb == "new")
	{
		if (args.size() < 2)
		{
			outError= "usage: app new <projectName>";
			return false;
		}

		// The menu's New Project path: creates the folder, loads it, and starts
		// the guided setup flow on entering the tracking UI
		if (!m_app->activateNewProject(args[1]))
		{
			outError= "failed to create project '" + args[1] + "'";
			return false;
		}

		return true;
	}
	else if (verb == "resume")
	{
		const GlobalSettings* globalSettings= m_app->getGlobalSettings();
		if (!globalSettings->hasLastProjectPath() || !std::filesystem::exists(globalSettings->lastProjectPath))
		{
			outError= "no last project to resume";
			return false;
		}

		m_app->requestLoadProject(globalSettings->lastProjectPath);
		return true;
	}
	else if (verb == "close")
	{
		if (!bInProject)
		{
			outError= "no project is open";
			return false;
		}

		m_app->requestCloseProject();
		return true;
	}
	else if (verb == "quit")
	{
		// The same path as the window close button. The reply goes out before
		// the socket closes at shutdown.
		m_app->requestShutdown();
		return true;
	}

	outError= "unknown verb '" + verb + "'";
	return false;
}

bool AutomationServer::resolveViewport(const std::string& idText, GuiViewportInfo& outViewport,
									   std::string& outError) const
{
	uint32_t viewportId= 0;
	if (!parseViewportId(idText, viewportId))
	{
		outError= "invalid viewport id '" + idText + "'";
		return false;
	}

	for (const GuiViewportInfo& viewport : GuiViewports::getViewports())
	{
		if (viewport.viewportId == viewportId)
		{
			outViewport= viewport;
			return true;
		}
	}

	outError= "no viewport " + formatViewportId(viewportId) + " (see gui viewports)";
	return false;
}

bool AutomationServer::handleGuiCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
										std::string& outError)
{
	if (args.empty())
	{
		outError= "usage: gui viewports|float ...";
		return false;
	}

	const std::string& verb= args[0];

	if (verb == "viewports")
	{
		for (const GuiViewportInfo& viewport : GuiViewports::getViewports())
		{
			std::string line= formatViewportId(viewport.viewportId) + " " + std::to_string(viewport.x) + ","
							  + std::to_string(viewport.y) + " " + std::to_string(viewport.width) + "x"
							  + std::to_string(viewport.height);
			for (const std::string& windowId : viewport.windowIds)
				line+= " " + quoteTokenIfNeeded(windowId);

			outLines.push_back(line);
		}

		return true;
	}
	else if (verb == "float")
	{
		if (args.size() != 4 && args.size() != 6)
		{
			outError= "usage: gui float <windowId> <x> <y> [<width> <height>]";
			return false;
		}

		int x= 0;
		int y= 0;
		int width= 0;
		int height= 0;
		if (!parseIntToken(args[2], x) || !parseIntToken(args[3], y))
		{
			outError= "invalid position '" + args[2] + " " + args[3] + "'";
			return false;
		}
		if (args.size() == 6
			&& (!parseIntToken(args[4], width) || !parseIntToken(args[5], height) || width <= 0 || height <= 0))
		{
			outError= "invalid size '" + args[4] + " " + args[5] + "'";
			return false;
		}

		// Applied right after the next NewFrame, so the window shows up moved in
		// `gui viewports` a frame or two later
		return GuiViewports::requestFloatWindow(args[1], x, y, width, height, outError);
	}

	outError= "unknown verb '" + verb + "'";
	return false;
}

bool AutomationServer::handleInputCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
										  std::string& outError)
{
	if (args.empty())
	{
		outError= "usage: input move|click|press|release|wheel|key|keydown|keyup|text ...";
		return false;
	}

	if (m_app == nullptr || m_app->getSdlWindow() == nullptr)
	{
		outError= "no window";
		return false;
	}

	const std::string& verb= args[0];

	// The target is the main window, or `viewport <viewportId>` for a panel torn
	// out of it into its own OS window. The arguments after the target start at
	// firstArg either way.
	SDL_Window* targetWindow= m_app->getSdlWindow();
	size_t firstArg= 1;
	if (args.size() >= 2 && args[1] == "viewport")
	{
		if (args.size() < 3)
		{
			outError= "usage: input " + verb + " viewport <viewportId> ...";
			return false;
		}

		GuiViewportInfo viewport;
		if (!resolveViewport(args[2], viewport, outError))
			return false;

		if (!viewport.bIsMainViewport)
		{
			targetWindow= SDL_GetWindowFromID(viewport.platformWindowId);
			if (targetWindow == nullptr)
			{
				outError= "viewport " + formatViewportId(viewport.viewportId) + " has no OS window";
				return false;
			}
		}

		firstArg= 3;
	}

	const uint32_t targetWindowId= SDL_GetWindowID(targetWindow);

	// Injected input only reaches ImGui if the window is the one receiving OS
	// input, so every verb raises it first
	SDL_RaiseWindow(targetWindow);

	if (verb == "move" || verb == "click" || verb == "press" || verb == "release" || verb == "wheel")
	{
		if (args.size() < firstArg + 2)
		{
			outError= "usage: input " + verb + " <x> <y> ...";
			return false;
		}

		int windowX= 0;
		int windowY= 0;
		if (!parseIntToken(args[firstArg], windowX) || !parseIntToken(args[firstArg + 1], windowY))
		{
			outError= "invalid position '" + args[firstArg] + " " + args[firstArg + 1] + "'";
			return false;
		}

		// The real cursor moves, deliberately: ImGui's backend re-reads the OS
		// cursor every frame whenever the pointer is not over one of our windows,
		// and would overwrite a fabricated position before the widget under it
		// saw the click
		SDL_WarpMouseInWindow(targetWindow, windowX, windowY);

		if (verb == "move")
			return true;

		const size_t optionArg= firstArg + 2;

		if (verb == "wheel")
		{
			int scrollY= 0;
			int scrollX= 0;
			if (args.size() < optionArg + 1 || !parseIntToken(args[optionArg], scrollY))
			{
				outError= "usage: input wheel <x> <y> <scrollY> [scrollX]";
				return false;
			}
			if (args.size() >= optionArg + 2 && !parseIntToken(args[optionArg + 1], scrollX))
			{
				outError= "invalid scrollX '" + args[optionArg + 1] + "'";
				return false;
			}

			injectMouseWheel(targetWindowId, windowX, windowY, scrollX, scrollY);
			return true;
		}

		uint8_t button= SDL_BUTTON_LEFT;
		if (args.size() >= optionArg + 1 && !parseMouseButtonName(args[optionArg], button))
		{
			outError= "unknown mouse button '" + args[optionArg] + "' (left|middle|right)";
			return false;
		}

		int clickCount= 1;
		if (args.size() >= optionArg + 2 && !parseIntToken(args[optionArg + 1], clickCount))
		{
			outError= "invalid click count '" + args[optionArg + 1] + "'";
			return false;
		}

		// press and release are the halves of click, for drags and for widgets
		// whose popup only survives while the button is held
		if (verb != "release")
			injectMouseButton(targetWindowId, button, true, windowX, windowY, clickCount);
		if (verb != "press")
			injectMouseButton(targetWindowId, button, false, windowX, windowY, clickCount);

		return true;
	}
	else if (verb == "key" || verb == "keydown" || verb == "keyup")
	{
		if (args.size() < firstArg + 1)
		{
			outError= "usage: input " + verb + " <keyName> [modifiers...]";
			return false;
		}

		SDL_Keycode key= SDLK_UNKNOWN;
		if (!parseKeyName(args[firstArg], key))
		{
			outError= "unknown key '" + args[firstArg] + "'";
			return false;
		}

		uint16_t keyMod= KMOD_NONE;
		for (size_t argIndex= firstArg + 1; argIndex < args.size(); ++argIndex)
		{
			uint16_t parsedMod= KMOD_NONE;
			if (!parseKeyModifier(args[argIndex], parsedMod))
			{
				outError= "unknown modifier '" + args[argIndex] + "' (shift|ctrl|alt|gui)";
				return false;
			}
			keyMod|= parsedMod;
		}

		// keydown and keyup are the halves of key, so a drive can hold a modifier
		// (keydown ctrl ctrl) across a mouse press, move, and release
		if (verb != "keyup")
			injectKey(targetWindowId, key, keyMod, true);
		if (verb != "keydown")
			injectKey(targetWindowId, key, keyMod, false);
		return true;
	}
	else if (verb == "text")
	{
		// Free text: take the remainder of the raw line so spacing and quoting
		// survive tokenization. The tokens before it are `input text` or
		// `input text viewport <viewportId>`.
		const std::string text= AutomationProtocol::remainderAfterTokens(getCurrentCommandLine(), firstArg + 1);
		if (text.empty())
		{
			outError= "usage: input text <text...>";
			return false;
		}

		injectText(targetWindowId, text);
		return true;
	}

	outError= "unknown verb '" + verb + "'";
	return false;
}

bool AutomationServer::handleScreenshotCommand(const std::vector<std::string>& args,
											   std::vector<std::string>& outLines, std::string& outError)
{
	if (args.empty())
	{
		outError= "usage: screenshot window|viewport ...";
		return false;
	}

	if (m_bWindowCapturePending || m_bViewportCapturePending)
	{
		outError= "a capture is already pending";
		return false;
	}

	const std::string& verb= args[0];

	if (verb == "window")
	{
		// Parked until the frame renders; the reply is sent from servicePendingWindowCapture
		m_windowCapturePath= args.size() > 1 ? args[1] : "mikantrack_window.png";
		m_bWindowCapturePending= true;
		m_bReplyDeferred= true;
		return true;
	}
	else if (verb == "viewport")
	{
		if (args.size() < 2)
		{
			outError= "usage: screenshot viewport <viewportId> [path]";
			return false;
		}

		GuiViewportInfo viewport;
		if (!resolveViewport(args[1], viewport, outError))
			return false;

		const std::string path= args.size() > 2 ? args[2] : "mikantrack_viewport.png";

		if (viewport.bIsMainViewport)
		{
			// The main viewport is the main window itself
			m_windowCapturePath= path;
			m_bWindowCapturePending= true;
		}
		else
		{
			// Parked until the torn-out viewports render; the reply is sent from
			// servicePendingViewportCapture
			m_viewportCapturePath= path;
			m_viewportCaptureId= viewport.viewportId;
			m_bViewportCapturePending= true;
		}

		m_bReplyDeferred= true;
		return true;
	}

	outError= "unknown verb '" + verb + "'";
	return false;
}

void AutomationServer::servicePendingWindowCapture(int drawableWidth, int drawableHeight)
{
	if (!m_bWindowCapturePending)
		return;

	m_bWindowCapturePending= false;

	if (saveBackBufferToPng(drawableWidth, drawableHeight, m_windowCapturePath))
		sendReply({absolutePathText(m_windowCapturePath)});
	else
		sendErrorReply("screenshot window: failed to write '" + m_windowCapturePath + "'");
}

void AutomationServer::servicePendingViewportCapture(uint32_t viewportId, int drawableWidth, int drawableHeight)
{
	if (!m_bViewportCapturePending || viewportId != m_viewportCaptureId)
		return;

	m_bViewportCapturePending= false;

	if (saveBackBufferToPng(drawableWidth, drawableHeight, m_viewportCapturePath))
		sendReply({absolutePathText(m_viewportCapturePath)});
	else
		sendErrorReply("screenshot viewport: failed to write '" + m_viewportCapturePath + "'");
}

void AutomationServer::failUnservicedViewportCapture()
{
	if (!m_bViewportCapturePending)
		return;

	m_bViewportCapturePending= false;
	sendErrorReply("screenshot viewport: viewport " + formatViewportId(m_viewportCaptureId)
				   + " did not render (closed, hidden, or minimized)");
}

bool AutomationServer::handleLogCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
										std::string& outError)
{
	if (args.size() < 2 || args[0] != "tail")
	{
		outError= "usage: log tail <lineCount> [trace|debug|info|warning|error|fatal]";
		return false;
	}

	int lineCount= 0;
	if (!parseIntToken(args[1], lineCount) || lineCount <= 0)
	{
		outError= "invalid line count '" + args[1] + "'";
		return false;
	}

	int minLevel= (int)LogSeverityLevel::trace;
	if (args.size() >= 3)
	{
		static const std::vector<std::string> k_levelNames= {"trace", "debug", "info", "warning", "error", "fatal"};
		const auto levelIter= std::find(k_levelNames.begin(), k_levelNames.end(), args[2]);
		if (levelIter == k_levelNames.end())
		{
			outError= "unknown log level '" + args[2] + "'";
			return false;
		}
		minLevel= (int)(levelIter - k_levelNames.begin());
	}

	LogPanel::getInstance().copyTail((size_t)lineCount, minLevel, outLines);
	return true;
}
