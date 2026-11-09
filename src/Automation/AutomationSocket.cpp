#include "AutomationSocket.h"

#include <winsock2.h>
#include <ws2tcpip.h>

#include <cstring>

#include "Logger.h"

// ---- Helpers ---------------------------------------------------------------

static void setNonBlocking(AutomationSocket::SocketHandle sock)
{
	u_long mode= 1;
	ioctlsocket(static_cast<SOCKET>(sock), FIONBIO, &mode);
}

// ---- AutomationSocket ------------------------------------------------------

AutomationSocket::AutomationSocket(uint16_t port)
{
	// WSAStartup/WSACleanup are reference counted by Winsock, so pairing them
	// per socket is safe beside the UDP sockets that do the same
	WSADATA wsaData;
	if (::WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		MIKAN_LOG_ERROR("AutomationSocket") << "WSAStartup failed";
		return;
	}
	m_bWinsockStarted= true;

	// Create TCP listener socket
	SOCKET listener= ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listener == INVALID_SOCKET)
	{
		MIKAN_LOG_ERROR("AutomationSocket") << "failed to create listener socket";
		return;
	}

	// Allow rapid reuse of the port after restart
	int optval= 1;
	::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&optval), sizeof(optval));

	sockaddr_in addr{};
	addr.sin_family= AF_INET;
	addr.sin_addr.s_addr= htonl(INADDR_LOOPBACK);
	addr.sin_port= htons(port);

	if (::bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR
		|| ::listen(listener, 1) == SOCKET_ERROR)
	{
		MIKAN_LOG_ERROR("AutomationSocket") << "failed to bind/listen on port " << port;
		closesocket(listener);
		return;
	}

	// The port actually bound, which differs from the request only for port 0
	sockaddr_in boundAddr{};
	int boundLen= sizeof(boundAddr);
	if (::getsockname(listener, reinterpret_cast<sockaddr*>(&boundAddr), &boundLen) == 0)
		m_boundPort= ntohs(boundAddr.sin_port);
	else
		m_boundPort= port;

	setNonBlocking(listener);
	m_listenSocket= static_cast<SocketHandle>(listener);
	MIKAN_LOG_INFO("AutomationSocket") << "listening on port " << m_boundPort << " (loopback)";
}

AutomationSocket::~AutomationSocket()
{
	close();

	if (m_bWinsockStarted)
		::WSACleanup();
}

void AutomationSocket::close()
{
	if (m_clientSocket != k_invalidSocket)
	{
		// Half-close the send side first so a reply still in flight (the
		// app quit acknowledgement) reaches the client instead of being
		// discarded by a hard reset
		::shutdown(static_cast<SOCKET>(m_clientSocket), SD_SEND);
		closesocket(static_cast<SOCKET>(m_clientSocket));
		m_clientSocket= k_invalidSocket;
	}
	if (m_listenSocket != k_invalidSocket)
	{
		closesocket(static_cast<SOCKET>(m_listenSocket));
		m_listenSocket= k_invalidSocket;
		m_boundPort= 0;
	}
}

// ---- Internal helpers ------------------------------------------------------

void AutomationSocket::tryAccept()
{
	SOCKET listener= static_cast<SOCKET>(m_listenSocket);
	sockaddr_in clientAddr{};
	int addrLen= sizeof(clientAddr);
	SOCKET client= ::accept(listener, reinterpret_cast<sockaddr*>(&clientAddr), &addrLen);
	if (client == INVALID_SOCKET)
		return;

	setNonBlocking(client);
	m_clientSocket= static_cast<SocketHandle>(client);

	MIKAN_LOG_INFO("AutomationSocket") << "client connected";

	if (onClientConnected)
		onClientConnected();
}

void AutomationSocket::readAvailable()
{
	char buf[1024];
	SOCKET client= static_cast<SOCKET>(m_clientSocket);

	while (true)
	{
		int n= ::recv(client, buf, static_cast<int>(sizeof(buf)), 0);
		if (n > 0)
		{
			m_readBuffer.append(buf, static_cast<size_t>(n));

			// Emit onLineReceived for each complete newline-terminated line
			size_t pos;
			while ((pos= m_readBuffer.find('\n')) != std::string::npos)
			{
				std::string line= m_readBuffer.substr(0, pos);
				m_readBuffer.erase(0, pos + 1);

				// Strip trailing '\r' if present
				if (!line.empty() && line.back() == '\r')
					line.pop_back();

				MIKAN_LOG_DEBUG("AutomationSocket") << "RX: " << line;
				if (onLineReceived)
					onLineReceived(line);
			}

			if (m_readBuffer.size() > k_maxBufferedLineBytes)
			{
				handleDisconnect("line exceeded the maximum buffered length");
				return;
			}
		}
		else if (n == 0)
		{
			// Orderly shutdown
			handleDisconnect("client closed connection");
			break;
		}
		else
		{
			const int err= WSAGetLastError();
			if (err == WSAEWOULDBLOCK)
				break; // no more data right now

			handleDisconnect("recv error");
			break;
		}
	}
}

void AutomationSocket::handleDisconnect(const char* reason)
{
	if (m_clientSocket == k_invalidSocket)
		return;

	if (reason)
		MIKAN_LOG_INFO("AutomationSocket") << "client disconnected: " << reason;

	closesocket(static_cast<SOCKET>(m_clientSocket));
	m_clientSocket= k_invalidSocket;
	m_readBuffer.clear();

	if (onClientDisconnected)
		onClientDisconnected();
}

// ---- Public interface ------------------------------------------------------

void AutomationSocket::disconnectClient(const char* reason) { handleDisconnect(reason); }

void AutomationSocket::poll()
{
	if (m_listenSocket == k_invalidSocket)
		return;

	if (m_clientSocket == k_invalidSocket)
		tryAccept();

	if (m_clientSocket != k_invalidSocket)
		readAvailable();
}

bool AutomationSocket::sendText(const std::string& text)
{
	if (!isClientConnected())
		return false;

	SOCKET client= static_cast<SOCKET>(m_clientSocket);

	int sent= ::send(client, text.c_str(), static_cast<int>(text.size()), 0);
	if (sent == SOCKET_ERROR)
	{
		handleDisconnect("send error");
		return false;
	}
	return true;
}
