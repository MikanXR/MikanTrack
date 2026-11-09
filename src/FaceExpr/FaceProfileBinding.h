#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "FaceExprCompiler.h"
#include "FaceExprProgram.h"
#include "FaceProfile.h"

struct AvatarModel;

// Why a profile output has no target. The names a problem is about are the
// output's own: its target for NoExpression and NoMorph, its mesh for NoMesh.
enum class eFaceTargetProblem
{
	None,
	NoTarget,
	NoExpression,
	NoMesh,
	NoMorph,
};

// A face profile resolved against one avatar (or none): the compiled
// program, the targets its outputs drive, and the morph binds the targets
// reach. Immutable once built and shared between the vision thread and the
// UI; a profile edit, a profile switch or an avatar change builds a new one
// with a new generation.
//
// The combine rules:
// - An output's value is clamped to 0..1.
// - Outputs sharing a target combine with max into the target's value.
// - An expression target drives its morph binds times the bind weight, a
//   morph target its morphs at weight 1, and a morph driven by several
//   targets takes their max. (The preview ignores an expression's isBinary;
//   a receiver applies it.)
// - On the wire each name goes out once: targets that share a name (an
//   expression and a morph spelled alike) send their max.
// Which output won a target and which target won a morph are per-frame
// facts, computed from the values by whoever evaluates.
class FaceProfileBinding
{
public:
	struct MorphBind
	{
		int mesh= -1;
		int morph= -1;
		float weight= 1.f;
	};
	struct Target
	{
		eFaceTargetKind kind= eFaceTargetKind::Expression;
		// The avatar's spelling, which is what goes on the wire
		std::string name;
		// A morph target's mesh as the outputs named it; empty for every mesh
		std::string mesh;
		// The profile outputs driving it, in profile order
		std::vector<int> outputs;
		std::vector<MorphBind> morphBinds;
		// Index into getWireNames()
		int wireIndex= -1;
	};

	// Builds a binding of a compiled profile against model (null: no avatar,
	// where every target is sent and none is bound). A target the avatar
	// lacks is reported as unresolved and skipped.
	static std::shared_ptr<const FaceProfileBinding> build(const FaceProfile& profile,
														   std::shared_ptr<const FaceExprProgram> program,
														   const AvatarModel* model);

	// Unique across every binding built in the process, never 0
	uint64_t getGeneration() const { return m_generation; }
	const FaceProfile& getProfile() const { return m_profile; }
	const std::shared_ptr<const FaceExprProgram>& getProgram() const { return m_program; }
	const std::vector<Target>& getTargets() const { return m_targets; }
	// Per profile output: its target, or -1 when unresolved
	const std::vector<int>& getOutputTargets() const { return m_outputTargets; }
	// Per profile output: why it has no target in English, or "" when it has
	// one
	const std::vector<std::string>& getOutputProblems() const { return m_outputProblems; }
	// Per profile output: why it has no target, None when it has one
	const std::vector<eFaceTargetProblem>& getOutputProblemCodes() const { return m_outputProblemCodes; }
	const std::vector<std::string>& getWireNames() const { return m_wireNames; }
	const std::vector<size_t>& getMeshMorphCounts() const { return m_meshMorphCounts; }
	// Whether the morph weights this binding evaluates fit model: one entry
	// per mesh, each the size of that mesh's morph targets
	bool fitsModel(const AvatarModel& model) const;
	// The profile outputs reading a let, directly or through other lets,
	// ascending; empty without a program
	std::vector<int> outputsReadingLet(int letIndex) const;

	// Per target, the max of its outputs' values. outWinners, when given,
	// gets the output that gave each target its value (-1 for a target whose
	// outputs are all zero).
	void combineTargets(const std::vector<float>& outputValues, std::vector<float>& outTargetValues,
						std::vector<int>* outWinners= nullptr) const;
	// Per wire name, the max of its targets' values
	void combineWire(const std::vector<float>& targetValues, std::vector<float>& outWireValues) const;
	// Per mesh, one weight per morph. outWinners, when given, gets the target
	// that gave each morph its weight (-1 where nothing drives it).
	void evaluateMorphs(const std::vector<float>* targetValues, std::vector<std::vector<float>>& outMeshWeights,
						std::vector<std::vector<int>>* outWinners= nullptr) const;

private:
	uint64_t m_generation= 0;
	FaceProfile m_profile;
	std::shared_ptr<const FaceExprProgram> m_program;
	std::vector<Target> m_targets;
	std::vector<int> m_outputTargets;
	std::vector<std::string> m_outputProblems;
	std::vector<eFaceTargetProblem> m_outputProblemCodes;
	std::vector<std::string> m_wireNames;
	std::vector<size_t> m_meshMorphCounts;
};

// Compiles a profile and binds it to model. Null, with the errors when asked
// for, when the profile does not compile.
std::shared_ptr<const FaceProfileBinding> compileFaceBinding(const FaceProfile& profile, const AvatarModel* model,
															 std::vector<FaceExprCompileError>* outErrors= nullptr);
