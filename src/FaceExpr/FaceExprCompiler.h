#pragma once

#include <memory>
#include <string>
#include <vector>

#include "FaceExprLexer.h"
#include "FaceExprProgram.h"
#include "FaceProfile.h"

struct FaceExprCompileError
{
	// Which expression: a let or an output, by profile index
	bool bLet= false;
	int index= -1;
	// The error is about a variable's name rather than its expression: an
	// invalid name, a function's name, a name defined twice. span then
	// indexes the name.
	bool bOnName= false;
	// The characters of that expression's text (or name) the error is about
	FaceExprSpan span;
	eFaceExprErrorCode code= eFaceExprErrorCode::UnexpectedToken;
	// What the code interpolates, see eFaceExprErrorCode
	std::vector<std::string> args;
	// The English text, for logs, tools and tests
	std::string message;
};

struct FaceExprCompileResult
{
	// Null when any expression failed: a profile compiles whole or not at all
	std::shared_ptr<const FaceExprProgram> program;
	// One per failed expression, at most
	std::vector<FaceExprCompileError> errors;
};

// Compiles a profile's lets and outputs into one program. Names compare
// without case, the input namespaces and the let names alike. Unknown names,
// wrong argument counts, a let named twice or after a function, and let
// cycles are errors.
//
// The functions:
//   min(a, b, ...), max(a, b, ...), clamp(x, lo, hi), abs(x), pow(x, y)
//   lerp(a, b, t), remap(x, inLo, inHi[, outLo, outHi]) (clamped to the
//   input range), smoothstep(edge0, edge1, x), select(c, a, b)
//   smooth(x, tauSeconds), hysteresis(x, on, off): stateful, one slot per
//   call site, advanced every frame whichever select branch reads them
// A value counts as true above 0.5; comparisons and logic yield 0 or 1.
FaceExprCompileResult compileFaceProfile(const FaceProfile& profile);

// Whether a name is a built-in function (case ignored)
bool isFaceExprFunctionName(const std::string& name);
