#include "FaceProfileBinding.h"

#include <algorithm>
#include <atomic>
#include <cctype>

#include "AvatarTypes.h"

namespace
{
std::atomic<uint64_t> g_nextGeneration{1};

bool equalsIgnoringCase(const std::string& a, const std::string& b)
{
	if (a.size() != b.size())
		return false;
	for (size_t index= 0; index < a.size(); ++index)
	{
		if (std::tolower((unsigned char)a[index]) != std::tolower((unsigned char)b[index]))
			return false;
	}
	return true;
}

// The index of name in names, exact first and then without case; -1 absent
template <typename T, typename GetName>
int findName(const std::vector<T>& items, const std::string& name, GetName getName)
{
	for (size_t index= 0; index < items.size(); ++index)
	{
		if (getName(items[index]) == name)
			return (int)index;
	}
	for (size_t index= 0; index < items.size(); ++index)
	{
		if (equalsIgnoringCase(getName(items[index]), name))
			return (int)index;
	}
	return -1;
}
} // namespace

std::shared_ptr<const FaceProfileBinding> FaceProfileBinding::build(const FaceProfile& profile,
																	 std::shared_ptr<const FaceExprProgram> program,
																	 const AvatarModel* model)
{
	auto binding= std::make_shared<FaceProfileBinding>();
	binding->m_generation= g_nextGeneration++;
	binding->m_profile= profile;
	binding->m_program= std::move(program);
	binding->m_outputTargets.assign(profile.outputs.size(), -1);
	binding->m_outputProblems.assign(profile.outputs.size(), std::string());
	binding->m_outputProblemCodes.assign(profile.outputs.size(), eFaceTargetProblem::None);

	if (model != nullptr)
	{
		for (const AvatarMesh& mesh : model->meshes)
			binding->m_meshMorphCounts.push_back(mesh.morphTargetNames.size());
	}

	for (size_t outputIndex= 0; outputIndex < profile.outputs.size(); ++outputIndex)
	{
		const FaceProfileOutput& output= profile.outputs[outputIndex];
		std::string& problem= binding->m_outputProblems[outputIndex];
		eFaceTargetProblem& problemCode= binding->m_outputProblemCodes[outputIndex];
		if (output.target.empty())
		{
			problem= "no target";
			problemCode= eFaceTargetProblem::NoTarget;
			continue;
		}

		Target resolved;
		resolved.kind= output.targetKind;
		resolved.name= output.target;
		resolved.mesh= output.targetKind == eFaceTargetKind::Morph ? output.mesh : std::string();
		if (model != nullptr && output.targetKind == eFaceTargetKind::Expression)
		{
			const int expressionIndex= findName(model->expressions, output.target,
												[](const AvatarExpression& expression) { return expression.name; });
			if (expressionIndex < 0)
			{
				problem= "the avatar has no expression named '" + output.target + "'";
				problemCode= eFaceTargetProblem::NoExpression;
				continue;
			}
			const AvatarExpression& expression= model->expressions[(size_t)expressionIndex];
			resolved.name= expression.name;
			for (const AvatarMorphBind& bind : expression.morphBinds)
			{
				if (bind.mesh >= 0 && bind.mesh < (int)model->meshes.size() && bind.morphIndex >= 0 &&
					bind.morphIndex < (int)model->meshes[(size_t)bind.mesh].morphTargetNames.size())
				{
					resolved.morphBinds.push_back(MorphBind{bind.mesh, bind.morphIndex, bind.weight});
				}
			}
		}
		else if (model != nullptr)
		{
			int onlyMesh= -1;
			if (!output.mesh.empty())
			{
				onlyMesh= findName(model->meshes, output.mesh, [](const AvatarMesh& mesh) { return mesh.name; });
				if (onlyMesh < 0)
				{
					problem= "the avatar has no mesh named '" + output.mesh + "'";
					problemCode= eFaceTargetProblem::NoMesh;
					continue;
				}
				resolved.mesh= model->meshes[(size_t)onlyMesh].name;
			}
			bool bNamed= false;
			for (size_t meshIndex= 0; meshIndex < model->meshes.size(); ++meshIndex)
			{
				if (onlyMesh >= 0 && (int)meshIndex != onlyMesh)
					continue;
				const std::vector<std::string>& morphNames= model->meshes[meshIndex].morphTargetNames;
				const int morphIndex= findName(morphNames, output.target, [](const std::string& name) { return name; });
				if (morphIndex < 0)
					continue;
				if (!bNamed)
				{
					resolved.name= morphNames[(size_t)morphIndex];
					bNamed= true;
				}
				resolved.morphBinds.push_back(MorphBind{(int)meshIndex, morphIndex, 1.f});
			}
			if (resolved.morphBinds.empty())
			{
				problem= "the avatar has no morph target named '" + output.target + "'";
				problemCode= eFaceTargetProblem::NoMorph;
				continue;
			}
		}

		// Outputs naming the same target share it
		int targetIndex= -1;
		for (size_t index= 0; index < binding->m_targets.size(); ++index)
		{
			const Target& existing= binding->m_targets[index];
			if (existing.kind == resolved.kind && existing.name == resolved.name && existing.mesh == resolved.mesh)
				targetIndex= (int)index;
		}
		if (targetIndex < 0)
		{
			targetIndex= (int)binding->m_targets.size();
			const auto wire= std::find(binding->m_wireNames.begin(), binding->m_wireNames.end(), resolved.name);
			resolved.wireIndex= (int)(wire - binding->m_wireNames.begin());
			if (wire == binding->m_wireNames.end())
				binding->m_wireNames.push_back(resolved.name);
			binding->m_targets.push_back(resolved);
		}
		binding->m_targets[(size_t)targetIndex].outputs.push_back((int)outputIndex);
		binding->m_outputTargets[outputIndex]= targetIndex;
	}
	return binding;
}

bool FaceProfileBinding::fitsModel(const AvatarModel& model) const
{
	if (m_meshMorphCounts.size() != model.meshes.size())
		return false;
	for (size_t meshIndex= 0; meshIndex < model.meshes.size(); ++meshIndex)
	{
		if (m_meshMorphCounts[meshIndex] != model.meshes[meshIndex].morphTargetNames.size())
			return false;
	}
	return true;
}

std::vector<int> FaceProfileBinding::outputsReadingLet(int letIndex) const
{
	std::vector<int> outputs;
	if (m_program == nullptr)
		return outputs;
	for (size_t outputIndex= 0; outputIndex < m_program->outputs.size(); ++outputIndex)
	{
		const std::vector<int>& lets= m_program->outputs[outputIndex].lets;
		if (std::find(lets.begin(), lets.end(), letIndex) != lets.end())
			outputs.push_back((int)outputIndex);
	}
	return outputs;
}

void FaceProfileBinding::combineTargets(const std::vector<float>& outputValues, std::vector<float>& outTargetValues,
										std::vector<int>* outWinners) const
{
	outTargetValues.assign(m_targets.size(), 0.f);
	if (outWinners != nullptr)
		outWinners->assign(m_targets.size(), -1);
	for (size_t targetIndex= 0; targetIndex < m_targets.size(); ++targetIndex)
	{
		for (int outputIndex : m_targets[targetIndex].outputs)
		{
			if (outputIndex >= (int)outputValues.size())
				continue;
			const float value= std::clamp(outputValues[(size_t)outputIndex], 0.f, 1.f);
			if (value > outTargetValues[targetIndex])
			{
				outTargetValues[targetIndex]= value;
				if (outWinners != nullptr)
					(*outWinners)[targetIndex]= outputIndex;
			}
		}
	}
}

void FaceProfileBinding::combineWire(const std::vector<float>& targetValues, std::vector<float>& outWireValues) const
{
	outWireValues.assign(m_wireNames.size(), 0.f);
	for (size_t targetIndex= 0; targetIndex < m_targets.size() && targetIndex < targetValues.size(); ++targetIndex)
	{
		float& wireValue= outWireValues[(size_t)m_targets[targetIndex].wireIndex];
		wireValue= std::max(wireValue, targetValues[targetIndex]);
	}
}

void FaceProfileBinding::evaluateMorphs(const std::vector<float>* targetValues,
										std::vector<std::vector<float>>& outMeshWeights,
										std::vector<std::vector<int>>* outWinners) const
{
	outMeshWeights.resize(m_meshMorphCounts.size());
	for (size_t meshIndex= 0; meshIndex < m_meshMorphCounts.size(); ++meshIndex)
		outMeshWeights[meshIndex].assign(m_meshMorphCounts[meshIndex], 0.f);
	if (outWinners != nullptr)
	{
		outWinners->resize(m_meshMorphCounts.size());
		for (size_t meshIndex= 0; meshIndex < m_meshMorphCounts.size(); ++meshIndex)
			(*outWinners)[meshIndex].assign(m_meshMorphCounts[meshIndex], -1);
	}
	if (targetValues == nullptr)
		return;

	for (size_t targetIndex= 0; targetIndex < m_targets.size() && targetIndex < targetValues->size(); ++targetIndex)
	{
		for (const MorphBind& bind : m_targets[targetIndex].morphBinds)
		{
			const float value= (*targetValues)[targetIndex] * bind.weight;
			float& weight= outMeshWeights[(size_t)bind.mesh][(size_t)bind.morph];
			if (value > weight)
			{
				weight= value;
				if (outWinners != nullptr)
					(*outWinners)[(size_t)bind.mesh][(size_t)bind.morph]= (int)targetIndex;
			}
		}
	}
}

std::shared_ptr<const FaceProfileBinding> compileFaceBinding(const FaceProfile& profile, const AvatarModel* model,
															 std::vector<FaceExprCompileError>* outErrors)
{
	FaceExprCompileResult result= compileFaceProfile(profile);
	if (outErrors != nullptr)
		*outErrors= result.errors;
	if (result.program == nullptr)
		return nullptr;
	return FaceProfileBinding::build(profile, result.program, model);
}
