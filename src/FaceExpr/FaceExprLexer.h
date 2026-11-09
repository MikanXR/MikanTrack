#pragma once

#include <string>
#include <vector>

// A character range in an expression's text, [begin, end)
struct FaceExprSpan
{
	int begin= 0;
	int end= 0;
};

enum class eFaceExprToken
{
	Number,
	// A name, dotted or not: "phone.jawOpen", "smooth", "mouthOpen"
	Identifier,
	// An operator: + - * / < <= > >= == != && || !
	Operator,
	LeftParen,
	RightParen,
	Comma,
	End,
};

struct FaceExprToken
{
	eFaceExprToken kind= eFaceExprToken::End;
	std::string text;
	float number= 0.f;
	FaceExprSpan span;
};

// What an expression error is, so a UI can word it in its own language. The
// args each code interpolates, in order:
//   UnexpectedCharacter     the character
//   UnexpectedToken         the token's text
//   ExpressionEndsEarly, ExpectedCloseParen, ExpectedCommaOrParen: none
//   UnknownInput, UnknownName, UnknownFunction: the name as written
//   ArgumentCount           the function, its minimum argument count, its
//                           maximum ("" when unbounded), the count given.
//                           A finite maximum above the minimum takes
//                           exactly one of the two (remap: 3 or 5).
//   InvalidVariableName, VariableIsFunctionName, VariableDefinedTwice,
//   VariableHasError        the variable's name
//   Cycle                   the chain, "a -> b -> a"
enum class eFaceExprErrorCode
{
	UnexpectedCharacter,
	UnexpectedToken,
	ExpressionEndsEarly,
	ExpectedCloseParen,
	ExpectedCommaOrParen,
	UnknownInput,
	UnknownName,
	UnknownFunction,
	ArgumentCount,
	InvalidVariableName,
	VariableIsFunctionName,
	VariableDefinedTwice,
	Cycle,
	VariableHasError,
};

struct FaceExprError
{
	eFaceExprErrorCode code= eFaceExprErrorCode::UnexpectedToken;
	std::vector<std::string> args;
	// The English text, for logs, tools and tests
	std::string message;
	FaceExprSpan span;
};

// Splits an expression into tokens, ending with one End token. False with
// outError set on a character the language has no token for.
bool lexFaceExpr(const std::string& text, std::vector<FaceExprToken>& outTokens, FaceExprError& outError);
