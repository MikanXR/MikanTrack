#pragma once

#include <climits>
#include <string>
#include <vector>

#include "FaceExprProgram.h"

// One function of the expression language: its name, its opcode, its arity,
// whether it carries state across frames, and the argument names an editor
// inserts as placeholders
struct FaceExprFunctionInfo
{
	const char* name;
	eFaceExprOp op;
	int minArgs;
	int maxArgs; // INT_MAX when unbounded
	bool bStateful;
	std::vector<const char*> placeholders;
};

// Every function, in a fixed order an editor can list them in
const std::vector<FaceExprFunctionInfo>& faceExprFunctions();
// Compared without case; null when the table lacks the name
const FaceExprFunctionInfo* findFaceExprFunction(const std::string& name);
