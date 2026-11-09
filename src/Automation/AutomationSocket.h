#pragma once

#include <cstdint>
#include <functional>
#include <string>

/// Loopback TCP line socket, accepting one client at a time.
/// Incoming bytes are split on '\n' (an optional trailing '\r' is stripped)
/// and each complete line is delivered through onLineReceived.
///
/// All I/O is serviced by poll() on the main thread; no background thread.
/// The listener binds 127.0.0.1 only: the automation channel drives the app,
/// so any process that reaches it owns the session, and it stays local.
class AutomationSocket
{
public:
	using SocketHandle= uintptr_t;
	static constexpr SocketHandle k_invalidSocket= static_cast<SocketHandle>(~0ULL);

	/// Port 0 asks the OS for an ephemeral port, read back with getBoundPort.
	explicit AutomationSocket(uint16_t port);
	~AutomationSocket();

	/// A peer that never sends a newline would otherwise grow the read buffer
	/// without bound.
	static const size_t k_maxBufferedLineBytes= 64 * 1024;

	// Non-copyable
	AutomationSocket(const AutomationSocket&)= delete;
	AutomationSocket& operator=(const AutomationSocket&)= delete;

	// ---- Callbacks ----
	std::function<void(const std::string& line)> onLineReceived; // complete line received
	std::function<void()> onClientConnected;                     // client connected
	std::function<void()> onClientDisconnected;                  // client disconnected

	/// Non-blocking: accept a pending connection and read any available lines.
	void poll();

	/// Send raw bytes to the connected client (no terminator appended).
	/// @returns false if the send failed (client disconnected)
	bool sendText(const std::string& text);

	/// @returns true when the listener socket opened successfully.
	bool isListening() const { return m_listenSocket != k_invalidSocket; }

	/// The port the listener bound, 0 when it is closed.
	uint16_t getBoundPort() const { return m_boundPort; }

	/// @returns true when a client is connected.
	bool isClientConnected() const { return m_clientSocket != k_invalidSocket; }

	/// Drop the connected client but keep listening for the next one.
	void disconnectClient(const char* reason);

	/// Close listener and any connected client socket.
	void close();

private:
	/// Try to accept a pending incoming connection on the listener socket.
	void tryAccept();

	/// Read any available data from the connected client into m_readBuffer.
	/// Emits onLineReceived for each complete newline-terminated line.
	void readAvailable();

	/// Handle client disconnect: close client socket, fire onClientDisconnected,
	/// then resume listening for the next connection.
	void handleDisconnect(const char* reason= nullptr);

	uint16_t m_boundPort= 0;
	bool m_bWinsockStarted= false;
	SocketHandle m_listenSocket= k_invalidSocket;
	SocketHandle m_clientSocket= k_invalidSocket;
	std::string m_readBuffer; // partial line accumulator
};
