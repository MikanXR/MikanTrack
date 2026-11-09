#include "AutomationProtocol.h"

namespace AutomationProtocol
{
bool tokenizeCommandLine(const std::string& line, std::vector<std::string>& outTokens, std::string& outError)
{
	outTokens.clear();
	outError.clear();

	std::string token;
	bool bInToken= false;
	bool bInQuotes= false;

	for (size_t i= 0; i < line.size(); ++i)
	{
		const char c= line[i];

		if (bInQuotes)
		{
			if (c == '\\' && i + 1 < line.size() && (line[i + 1] == '"' || line[i + 1] == '\\'))
			{
				token+= line[i + 1];
				++i;
			}
			else if (c == '"')
			{
				bInQuotes= false;
			}
			else
			{
				token+= c;
			}
		}
		else if (c == '"')
		{
			bInQuotes= true;
			bInToken= true;
		}
		else if (c == ' ' || c == '\t')
		{
			if (bInToken)
			{
				outTokens.push_back(token);
				token.clear();
				bInToken= false;
			}
		}
		else
		{
			token+= c;
			bInToken= true;
		}
	}

	if (bInQuotes)
	{
		outTokens.clear();
		outError= "unterminated quote";
		return false;
	}

	if (bInToken)
		outTokens.push_back(token);

	return true;
}

std::string frameReply(const std::vector<std::string>& contentLines)
{
	// Split entries on embedded newlines so the count line always matches
	// the number of lines the client will read
	std::vector<std::string> flatLines;
	for (const std::string& entry : contentLines)
	{
		size_t start= 0;
		while (true)
		{
			const size_t newlinePos= entry.find('\n', start);
			std::string lineText=
				(newlinePos != std::string::npos) ? entry.substr(start, newlinePos - start) : entry.substr(start);
			if (!lineText.empty() && lineText.back() == '\r')
				lineText.pop_back();
			flatLines.push_back(lineText);

			if (newlinePos == std::string::npos)
				break;
			start= newlinePos + 1;
		}
	}

	std::string reply= std::to_string(flatLines.size()) + "\r\n";
	for (const std::string& lineText : flatLines)
	{
		reply+= lineText;
		reply+= "\r\n";
	}

	return reply;
}

std::string remainderAfterTokens(const std::string& line, size_t tokenCount)
{
	size_t pos= 0;
	size_t tokensSkipped= 0;

	while (tokensSkipped < tokenCount && pos < line.size())
	{
		// Skip leading whitespace
		while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t'))
			++pos;
		if (pos >= line.size())
			break;

		// Skip one token, honoring quoted spans and their escapes
		bool bInQuotes= false;
		while (pos < line.size())
		{
			const char c= line[pos];
			if (bInQuotes)
			{
				if (c == '\\' && pos + 1 < line.size() && (line[pos + 1] == '"' || line[pos + 1] == '\\'))
					++pos;
				else if (c == '"')
					bInQuotes= false;
			}
			else if (c == '"')
				bInQuotes= true;
			else if (c == ' ' || c == '\t')
				break;

			++pos;
		}

		++tokensSkipped;
	}

	// Trim leading whitespace off the remainder
	while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t'))
		++pos;

	return line.substr(pos);
}
} // namespace AutomationProtocol
