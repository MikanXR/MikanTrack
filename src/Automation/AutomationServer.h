#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

class App;
class AutomationSocket;
struct GuiViewportInfo;

/// The automation command server: a loopback TCP text channel test scripts
/// and AI tooling use to drive and inspect the running app. Commands are
/// newline-terminated "<namespace> [verb] [args...]" lines, each answered
/// with one framed reply: a line holding the content line count, then
/// exactly that many lines.
///
/// Off unless --automation-server (or --automation-port) is on the command
/// line. It is a local debug surface with no place in a shipped session, so
/// it binds loopback only and nothing persists it.
class AutomationServer
{
public:
	/// Clear of MikanXR's 21120 and the VNyan automation plugin's 21130, so all
	/// three listen on one machine
	static constexpr uint16_t k_defaultPort= 21140;

	/// Handles one command. args holds the tokens after the namespace name
	/// (args[0] is the verb where the namespace has verbs).
	/// On success, fill outLines (empty is a valid reply).
	/// On failure, return false with outError set; the server frames it as
	/// "<namespace> <verb>: <error>".
	using CommandHandler= std::function<bool(const std::vector<std::string>& args, std::vector<std::string>& outLines,
											 std::string& outError)>;

	AutomationServer();
	~AutomationServer();

	// Non-copyable
	AutomationServer(const AutomationServer&)= delete;
	AutomationServer& operator=(const AutomationServer&)= delete;

	/// Bind the server to the app and register the built-in command
	/// namespaces. Called once at startup whether or not the listener opens.
	/// app may be null in a headless test, where only the namespaces that need
	/// no app answer.
	void initialize(App* app);

	/// Open the loopback listener. Port 0 takes an ephemeral port, read back
	/// with getListenPort.
	/// @returns false if the listener socket could not open
	bool startListener(uint16_t port);

	/// Close the listener, leaving the command registry intact.
	void stopListener();

	bool isListening() const { return m_socket != nullptr; }

	/// The port the open listener bound, or 0 when it is closed.
	uint16_t getListenPort() const;

	/// Service socket I/O and dispatch any received commands.
	/// Called once per frame on the main thread, between frames, so handlers
	/// may call app code directly and reply synchronously.
	void poll();

	void shutdown();

	/// Capture the main window's back buffer for a pending screenshot command
	/// and send its deferred reply. Called from App::tick after the GUI renders
	/// and before the frame presents, with the main GL context current.
	void servicePendingWindowCapture(int drawableWidth, int drawableHeight);

	/// Capture a torn-out panel's OS window for a pending screenshot viewport
	/// command. Called for each torn-out viewport, with that viewport's GL
	/// context current, after it renders and before it swaps.
	void servicePendingViewportCapture(uint32_t viewportId, int drawableWidth, int drawableHeight);

	/// Answer a pending screenshot viewport command whose viewport did not
	/// render this frame (closed, hidden, or minimized), so the client is not
	/// left waiting. Called after the torn-out viewports render.
	void failUnservicedViewportCapture();

	/// Register a command namespace. Future features add their surface here.
	void registerCommandNamespace(const std::string& namespaceName, const std::vector<std::string>& helpLines,
								  CommandHandler handler);

	/// Suppress the reply to the command currently dispatching, so a handler
	/// that cannot answer yet can answer later through sendDeferredReply. A
	/// handler that defers owns answering.
	void deferReply() { m_bReplyDeferred= true; }

	/// Answer a command whose handler called deferReply.
	void sendDeferredReply(const std::vector<std::string>& contentLines, bool bIsError= false);

	/// The raw, untokenized line of the command currently dispatching, for
	/// handlers whose final argument is free text. Pair with
	/// AutomationProtocol::remainderAfterTokens.
	const std::string& getCurrentCommandLine() const { return m_currentCommandLine; }

private:
	void handleCommandLine(const std::string& line);
	void sendReply(const std::vector<std::string>& contentLines);
	void sendErrorReply(const std::string& errorLine);

	// Built-in command namespaces
	void registerCoreNamespaces();
	bool handleHelpCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
						   std::string& outError);
	bool handleAppCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
						  std::string& outError);
	bool handleGuiCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
						  std::string& outError);
	bool handleInputCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
							std::string& outError);
	bool handleScreenshotCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
								 std::string& outError);
	bool handleLogCommand(const std::vector<std::string>& args, std::vector<std::string>& outLines,
						  std::string& outError);

	/// Resolve a viewport id from the `gui viewports` listing.
	bool resolveViewport(const std::string& idText, GuiViewportInfo& outViewport, std::string& outError) const;

	struct CommandProvider
	{
		std::vector<std::string> helpLines;
		CommandHandler handler;
	};

	App* m_app= nullptr;
	std::unique_ptr<AutomationSocket> m_socket;
	std::map<std::string, CommandProvider> m_commandProviders;

	// The raw line of the command currently dispatching, for handlers whose
	// final argument is free text that must not go through tokenization
	std::string m_currentCommandLine;

	// A handler that parked work for later in the frame sets this to suppress
	// the immediate reply; the parked work sends the reply when it completes
	bool m_bReplyDeferred= false;

	// Pending screenshot of the main window, serviced before the frame presents
	bool m_bWindowCapturePending= false;
	std::string m_windowCapturePath;

	// Pending screenshot of a torn-out viewport, serviced while it renders
	bool m_bViewportCapturePending= false;
	uint32_t m_viewportCaptureId= 0;
	std::string m_viewportCapturePath;
};
