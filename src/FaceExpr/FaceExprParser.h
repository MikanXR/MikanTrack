#pragma once

#include <memory>
#include <string>
#include <vector>

#include "FaceExprLexer.h"

enum class eFaceExprNode
{
	Number,
	// A name, resolved by the compiler: an input or a let
	Identifier,
	// op on children[0]: "-" or "!"
	Unary,
	// op on children[0] and children[1]
	Binary,
	// name(children...)
	Call,
};

// One node of a parsed expression. Names are kept as written; the compiler
// resolves them.
struct FaceExprNode
{
	eFaceExprNode kind= eFaceExprNode::Number;
	FaceExprSpan span;
	float number= 0.f;
	// The identifier, the function name, or the operator
	std::string text;
	std::vector<std::unique_ptr<FaceExprNode>> children;
};

// Parses one expression with a Pratt parser. Precedence, loosest first, all
// left associative:
//   ||
//   &&
//   == !=
//   < <= > >=
//   + -
//   * /
//   unary - !
// Null with outError set on a syntax error.
std::unique_ptr<FaceExprNode> parseFaceExpr(const std::string& text, FaceExprError& outError);
