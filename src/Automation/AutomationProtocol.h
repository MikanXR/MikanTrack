#pragma once

#include <string>
#include <vector>

/// Pure text-protocol helpers for the automation command server.
/// No app dependencies so the functions are unit-testable headlessly.
namespace AutomationProtocol
{
/// Split a command line into tokens.
/// Tokens are separated by runs of spaces/tabs. A double-quoted span groups
/// spaces into one token; inside quotes, \" yields a literal quote and
/// \\ a literal backslash.
/// @returns false (with outError set) on an unterminated quote
bool tokenizeCommandLine(const std::string& line, std::vector<std::string>& outTokens, std::string& outError);

/// Build one framed reply: a line holding the content line count, then
/// exactly that many content lines, all "\r\n" terminated.
/// Entries containing embedded newlines are split into multiple lines
/// so the count always matches what the client reads.
std::string frameReply(const std::vector<std::string>& contentLines);

/// Return the raw text after the first tokenCount tokens of the line,
/// with leading whitespace trimmed. Quoted spans count as one token.
/// Used by commands whose final argument is free text that must not go
/// through tokenization.
std::string remainderAfterTokens(const std::string& line, size_t tokenCount);
} // namespace AutomationProtocol
