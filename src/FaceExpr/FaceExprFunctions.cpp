#include "FaceExprFunctions.h"

#include <cctype>
#include <cstring>

const std::vector<FaceExprFunctionInfo>& faceExprFunctions()
{
	static const std::vector<FaceExprFunctionInfo> k_functions= {
		{"min", eFaceExprOp::Min, 2, INT_MAX, false, {"a", "b"}},
		{"max", eFaceExprOp::Max, 2, INT_MAX, false, {"a", "b"}},
		{"clamp", eFaceExprOp::Clamp, 3, 3, false, {"x", "lo", "hi"}},
		{"abs", eFaceExprOp::Abs, 1, 1, false, {"x"}},
		{"pow", eFaceExprOp::Pow, 2, 2, false, {"x", "y"}},
		{"lerp", eFaceExprOp::Lerp, 3, 3, false, {"a", "b", "t"}},
		{"remap", eFaceExprOp::Remap, 3, 5, false, {"x", "inLo", "inHi"}},
		{"smoothstep", eFaceExprOp::Smoothstep, 3, 3, false, {"edge0", "edge1", "x"}},
		{"select", eFaceExprOp::Select, 3, 3, false, {"c", "a", "b"}},
		{"smooth", eFaceExprOp::Smooth, 2, 2, true, {"x", "tauSeconds"}},
		{"hysteresis", eFaceExprOp::Hysteresis, 3, 3, true, {"x", "on", "off"}},
	};
	return k_functions;
}

const FaceExprFunctionInfo* findFaceExprFunction(const std::string& name)
{
	for (const FaceExprFunctionInfo& function : faceExprFunctions())
	{
		const size_t length= strlen(function.name);
		if (name.size() != length)
			continue;
		bool bMatch= true;
		for (size_t index= 0; bMatch && index < length; ++index)
			bMatch= std::tolower((unsigned char)name[index]) == (unsigned char)function.name[index];
		if (bMatch)
			return &function;
	}
	return nullptr;
}
