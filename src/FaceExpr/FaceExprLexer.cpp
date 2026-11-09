#include "FaceExprLexer.h"

#include <cctype>
#include <cstdlib>

namespace
{
bool isNameStart(char character)
{
	return std::isalpha((unsigned char)character) || character == '_';
}

bool isNameChar(char character)
{
	return std::isalnum((unsigned char)character) || character == '_';
}

bool isDigit(char character)
{
	return std::isdigit((unsigned char)character) != 0;
}
} // namespace

bool lexFaceExpr(const std::string& text, std::vector<FaceExprToken>& outTokens, FaceExprError& outError)
{
	outTokens.clear();
	const int length= (int)text.size();
	int position= 0;
	while (position < length)
	{
		const char character= text[position];
		if (std::isspace((unsigned char)character))
		{
			++position;
			continue;
		}

		FaceExprToken token;
		token.span.begin= position;

		if (isDigit(character) || (character == '.' && position + 1 < length && isDigit(text[position + 1])))
		{
			int end= position;
			while (end < length && isDigit(text[end]))
				++end;
			if (end < length && text[end] == '.')
			{
				++end;
				while (end < length && isDigit(text[end]))
					++end;
			}
			if (end < length && (text[end] == 'e' || text[end] == 'E'))
			{
				int exponentEnd= end + 1;
				if (exponentEnd < length && (text[exponentEnd] == '+' || text[exponentEnd] == '-'))
					++exponentEnd;
				if (exponentEnd < length && isDigit(text[exponentEnd]))
				{
					while (exponentEnd < length && isDigit(text[exponentEnd]))
						++exponentEnd;
					end= exponentEnd;
				}
			}
			token.kind= eFaceExprToken::Number;
			token.text= text.substr(position, end - position);
			token.number= (float)std::strtod(token.text.c_str(), nullptr);
			position= end;
		}
		else if (isNameStart(character))
		{
			int end= position;
			for (;;)
			{
				while (end < length && isNameChar(text[end]))
					++end;
				// A dot joins the next name segment: phone.jawOpen
				if (end + 1 < length && text[end] == '.' && isNameStart(text[end + 1]))
				{
					++end;
					continue;
				}
				break;
			}
			token.kind= eFaceExprToken::Identifier;
			token.text= text.substr(position, end - position);
			position= end;
		}
		else if (character == '(' || character == ')' || character == ',')
		{
			token.kind= character == '(' ? eFaceExprToken::LeftParen
					   : character == ')' ? eFaceExprToken::RightParen
										  : eFaceExprToken::Comma;
			token.text= std::string(1, character);
			++position;
		}
		else
		{
			static const char* const k_twoCharOperators[]= {"<=", ">=", "==", "!=", "&&", "||"};
			std::string op;
			if (position + 1 < length)
			{
				const std::string pair= text.substr(position, 2);
				for (const char* candidate : k_twoCharOperators)
				{
					if (pair == candidate)
						op= pair;
				}
			}
			if (op.empty())
			{
				static const std::string k_oneCharOperators= "+-*/<>!";
				if (k_oneCharOperators.find(character) == std::string::npos)
				{
					outError.code= eFaceExprErrorCode::UnexpectedCharacter;
					outError.args= {std::string(1, character)};
					outError.message= std::string("unexpected character '") + character + "'";
					outError.span= FaceExprSpan{position, position + 1};
					return false;
				}
				op= std::string(1, character);
			}
			token.kind= eFaceExprToken::Operator;
			token.text= op;
			position+= (int)op.size();
		}

		token.span.end= position;
		outTokens.push_back(token);
	}

	FaceExprToken end;
	end.kind= eFaceExprToken::End;
	end.span= FaceExprSpan{length, length};
	outTokens.push_back(end);
	return true;
}
