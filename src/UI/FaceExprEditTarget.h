#pragma once

// The row of the active face profile an expression editor is on: a variable
// (let) or a VRM expression binding (output), by profile index
struct FaceExprEditTarget
{
	bool bLet= false;
	int index= -1;

	bool operator==(const FaceExprEditTarget& other) const { return bLet == other.bLet && index == other.index; }
	bool operator!=(const FaceExprEditTarget& other) const { return !(*this == other); }
};
