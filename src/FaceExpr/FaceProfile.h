#pragma once

#include <string>
#include <vector>

// What a profile output drives
enum class eFaceTargetKind
{
	// A VRM expression, by its name in the file
	Expression,
	// A morph target, by name, directly
	Morph,
};

const char* faceTargetKindName(eFaceTargetKind kind);

// A named intermediate, evaluated once per frame before the outputs
struct FaceProfileLet
{
	std::string name;
	std::string expr;
};

struct FaceProfileOutput
{
	eFaceTargetKind targetKind= eFaceTargetKind::Expression;
	std::string target;
	// A morph target's mesh by name; empty means that morph on every mesh
	std::string mesh;
	// The function of the sources that sets the target's weight
	std::string vrmExpressionFunction;
};

// A user-authored face mapping: expressions over the raw face sources, each
// driving a VRM expression or a morph target. id never changes once minted
// (a new profile, a duplicate); name is the display name and is free to
// change.
struct FaceProfile
{
	static constexpr int k_version= 1;

	std::string id;
	std::string name;
	std::vector<FaceProfileLet> lets;
	std::vector<FaceProfileOutput> outputs;
};

// A fresh random UUID string (version 4), lowercase
std::string makeFaceProfileId();

// JSON round trip, separate from the file so it can be tested in memory
std::string faceProfileToJson(const FaceProfile& profile);
bool faceProfileFromJson(const std::string& text, FaceProfile& outProfile, std::string& outError);
