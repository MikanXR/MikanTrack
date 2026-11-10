// Winsock before the shared include block, so no windows.h gets in first
#include <winsock2.h>
#include <ws2tcpip.h>

#include "TestCommon.h"

#include "AutomationProtocol.h"
#include "AutomationServer.h"
#include "LogPanel.h"

namespace
{
int g_failures= 0;

void check(bool bCondition, const std::string& what)
{
	if (bCondition)
	{
		MIKAN_LOG_INFO("TestAutomation") << "PASS " << what;
	}
	else
	{
		++g_failures;
		MIKAN_LOG_ERROR("TestAutomation") << "FAIL " << what;
	}
}

// A loopback client that polls the server while it waits for each reply, since
// the server has no thread of its own
class TestClient
{
public:
	TestClient(AutomationServer& server, uint16_t port)
		: m_server(server)
	{
		WSADATA wsaData;
		m_bWinsockStarted= ::WSAStartup(MAKEWORD(2, 2), &wsaData) == 0;

		m_socket= ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		sockaddr_in addr{};
		addr.sin_family= AF_INET;
		addr.sin_addr.s_addr= htonl(INADDR_LOOPBACK);
		addr.sin_port= htons(port);
		if (::connect(m_socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
		{
			::closesocket(m_socket);
			m_socket= INVALID_SOCKET;
			return;
		}

		u_long mode= 1;
		::ioctlsocket(m_socket, FIONBIO, &mode);
	}

	~TestClient()
	{
		if (m_socket != INVALID_SOCKET)
			::closesocket(m_socket);
		if (m_bWinsockStarted)
			::WSACleanup();
	}

	bool isConnected() const { return m_socket != INVALID_SOCKET; }

	// Sends one command and reads its framed reply. Returns false on a timeout
	// or a closed connection.
	bool sendCommand(const std::string& command, std::vector<std::string>& outLines)
	{
		outLines.clear();
		const std::string wire= command + "\n";
		if (::send(m_socket, wire.c_str(), (int)wire.size(), 0) != (int)wire.size())
			return false;

		std::string countLine;
		if (!readLine(countLine))
			return false;

		const int count= atoi(countLine.c_str());
		for (int lineIndex= 0; lineIndex < count; ++lineIndex)
		{
			std::string line;
			if (!readLine(line))
				return false;
			outLines.push_back(line);
		}

		return true;
	}

	// Whether the server has closed the connection, pumping the server meanwhile
	bool waitForClose()
	{
		const auto deadline= std::chrono::steady_clock::now() + std::chrono::seconds(2);
		while (std::chrono::steady_clock::now() < deadline)
		{
			m_server.poll();
			char buffer[64];
			const int received= ::recv(m_socket, buffer, sizeof(buffer), 0);
			if (received == 0)
				return true;
			if (received < 0 && WSAGetLastError() != WSAEWOULDBLOCK)
				return true;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		return false;
	}

private:
	bool readLine(std::string& outLine)
	{
		const auto deadline= std::chrono::steady_clock::now() + std::chrono::seconds(2);
		while (std::chrono::steady_clock::now() < deadline)
		{
			const size_t newlinePos= m_buffer.find('\n');
			if (newlinePos != std::string::npos)
			{
				outLine= m_buffer.substr(0, newlinePos);
				m_buffer.erase(0, newlinePos + 1);
				if (!outLine.empty() && outLine.back() == '\r')
					outLine.pop_back();
				return true;
			}

			m_server.poll();

			char buffer[1024];
			const int received= ::recv(m_socket, buffer, sizeof(buffer), 0);
			if (received > 0)
				m_buffer.append(buffer, (size_t)received);
			else if (received == 0)
				return false;
			else if (WSAGetLastError() != WSAEWOULDBLOCK)
				return false;
			else
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		return false;
	}

	AutomationServer& m_server;
	SOCKET m_socket= INVALID_SOCKET;
	std::string m_buffer;
	bool m_bWinsockStarted= false;
};

void testTokenizer()
{
	std::vector<std::string> tokens;
	std::string error;

	check(AutomationProtocol::tokenizeCommandLine("gui  float\t\"Stage View\" 10 20", tokens, error) && tokens.size() == 5
			  && tokens[0] == "gui" && tokens[1] == "float" && tokens[2] == "Stage View" && tokens[3] == "10"
			  && tokens[4] == "20",
		  "tokenizer splits on runs of spaces and tabs and groups a quoted span");

	check(AutomationProtocol::tokenizeCommandLine("say \"a \\\"quoted\\\" \\\\ word\"", tokens, error)
			  && tokens.size() == 2 && tokens[1] == "a \"quoted\" \\ word",
		  "tokenizer unescapes a quote and a backslash inside quotes");

	check(AutomationProtocol::tokenizeCommandLine("", tokens, error) && tokens.empty(),
		  "tokenizer yields no tokens for an empty line");

	check(!AutomationProtocol::tokenizeCommandLine("app \"open", tokens, error) && error == "unterminated quote"
			  && tokens.empty(),
		  "tokenizer rejects an unterminated quote");
}

void testFraming()
{
	check(AutomationProtocol::frameReply({}) == "0\r\n", "frameReply frames an empty reply as a zero count");

	check(AutomationProtocol::frameReply({"one", "two\nthree", "four\r\n"}) == "5\r\none\r\ntwo\r\nthree\r\nfour\r\n\r\n",
		  "frameReply splits embedded newlines so the count matches the lines");

	check(AutomationProtocol::remainderAfterTokens("input text viewport 0x1  hello  world ", 4) == "hello  world ",
		  "remainderAfterTokens keeps the free text's inner spacing");

	check(AutomationProtocol::remainderAfterTokens("input text \"two words\" rest", 3) == "rest",
		  "remainderAfterTokens counts a quoted span as one token");

	check(AutomationProtocol::remainderAfterTokens("input text", 3).empty(),
		  "remainderAfterTokens is empty past the end of the line");
}

void testLoopbackSession()
{
	AutomationServer server;
	server.initialize(nullptr);

	check(server.startListener(0) && server.isListening() && server.getListenPort() != 0,
		  "server listens on an ephemeral loopback port");

	TestClient client(server, server.getListenPort());
	check(client.isConnected(), "client connects to the listener");
	if (!client.isConnected())
		return;

	std::vector<std::string> lines;

	check(client.sendCommand("help", lines) && lines.size() >= 6
			  && std::find(lines.begin(), lines.end(), "help") != lines.end()
			  && std::find(lines.begin(), lines.end(), "gui viewports") != lines.end()
			  && std::find(lines.begin(), lines.end(), "screenshot window [path]") != lines.end(),
		  "help lists every registered namespace's lines");

	check(client.sendCommand("", lines) && lines.empty(), "an empty line answers an empty reply");

	check(client.sendCommand("bogus verb", lines) && lines.size() == 1
			  && lines[0].find("unknown command 'bogus'") == 0 && lines[0].find("valid namespaces: app, gui, help") != std::string::npos,
		  "an unknown namespace answers an error naming the valid ones");

	check(client.sendCommand("app \"open", lines) && lines.size() == 1 && lines[0] == "parse: unterminated quote",
		  "a parse error answers as an error line");

	check(client.sendCommand("app info", lines) && lines.size() == 1 && lines[0] == "app info: no app",
		  "a namespace that needs the app answers an error without one");

	check(client.sendCommand("gui viewports", lines) && lines.empty(),
		  "gui viewports answers no viewports without an ImGui context");

	check(client.sendCommand("gui float Scene 10 20", lines) && lines.size() == 1 && lines[0] == "gui float: no GUI",
		  "gui float answers an error without an ImGui context");

	check(client.sendCommand("log tail 0", lines) && lines.size() == 1 && lines[0] == "log tail: invalid line count '0'",
		  "log tail rejects a zero line count");

	check(client.sendCommand("log tail 2 loud", lines) && lines.size() == 1
			  && lines[0] == "log tail: unknown log level 'loud'",
		  "log tail rejects an unknown level");

	// The log ring is fed through the logger callback in the app; here directly
	LogPanel::logCallback((int)LogSeverityLevel::info, "automation test line one");
	LogPanel::logCallback((int)LogSeverityLevel::warning, "automation test line two");
	LogPanel::logCallback((int)LogSeverityLevel::info, "automation test line three");

	check(client.sendCommand("log tail 2", lines) && lines.size() == 2 && lines[0] == "automation test line two"
			  && lines[1] == "automation test line three",
		  "log tail answers the newest lines oldest first");

	check(client.sendCommand("log tail 5 warning", lines) && lines.size() == 1 && lines[0] == "automation test line two",
		  "log tail filters by the minimum level");

	server.stopListener();
	check(!server.isListening() && server.getListenPort() == 0, "stopListener closes the listener");
	check(client.waitForClose(), "the client sees the connection close when the listener stops");
}

int runAutomationTest(const TestArgs& args)
{
	testTokenizer();
	testFraming();
	testLoopbackSession();

	if (g_failures > 0)
	{
		MIKAN_LOG_ERROR("TestAutomation") << g_failures << " check(s) failed";
		return 1;
	}

	MIKAN_LOG_INFO("TestAutomation") << "All checks passed";
	return 0;
}
} // namespace

MIKAN_REGISTER_TEST("--test-automation",
					"automation server: the command tokenizer and reply framing, and a loopback session on an "
					"ephemeral port",
					eTestCategory::SelfTest, runAutomationTest);
