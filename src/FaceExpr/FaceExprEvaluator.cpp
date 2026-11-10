#include "FaceExprEvaluator.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "FaceExprInputs.h"

namespace
{
bool truthy(float value)
{
	return value > 0.5f;
}

float boolValue(bool bValue)
{
	return bValue ? 1.f : 0.f;
}
} // namespace

void FaceExprState::reset()
{
	for (Slot& slot : slots)
		slot= Slot();
	bHasTime= false;
	lastTimeSeconds= 0.0;
}

void FaceExprState::bindTo(const std::shared_ptr<const FaceExprProgram>& newProgram)
{
	if (newProgram == program)
		return;

	std::vector<Slot> newSlots(newProgram != nullptr ? newProgram->stateSlotKeys.size() : 0);
	if (program != nullptr && newProgram != nullptr)
	{
		std::unordered_map<std::string, size_t> oldSlots;
		for (size_t slotIndex= 0; slotIndex < program->stateSlotKeys.size() && slotIndex < slots.size(); ++slotIndex)
			oldSlots[program->stateSlotKeys[slotIndex]]= slotIndex;
		for (size_t slotIndex= 0; slotIndex < newSlots.size(); ++slotIndex)
		{
			const auto found= oldSlots.find(newProgram->stateSlotKeys[slotIndex]);
			if (found != oldSlots.end())
				newSlots[slotIndex]= slots[found->second];
		}
	}
	program= newProgram;
	slots= std::move(newSlots);
}

void FaceExprEvaluator::evaluate(const FaceSources& sources, double timeSeconds, FaceExprState& state,
								 float* outValues)
{
	float inputs[FaceExprInputs::k_count];
	FaceExprInputs::fill(sources, inputs);
	evaluate(inputs, FaceExprInputs::k_count, timeSeconds, state, outValues);
}

bool FaceExprEvaluator::evaluate(const float* inputs, int inputCount, double timeSeconds, FaceExprState& state,
								 float* outValues)
{
	if (m_program == nullptr)
		return false;
	const FaceExprProgram& program= *m_program;
	if (inputCount != program.inputCount || (inputs == nullptr && inputCount > 0))
	{
		m_registers.assign((size_t)program.registerCount, 0.f);
		std::fill(outValues, outValues + program.outputs.size(), 0.f);
		return false;
	}
	state.bindTo(m_program);

	// The time rule: forward steps clamp to the cap, small regressions step
	// nothing, a large regression starts over
	double stepSeconds= 0.0;
	if (state.bHasTime)
	{
		const double delta= timeSeconds - state.lastTimeSeconds;
		if (delta < -k_maxStepSeconds)
			state.reset();
		else
			stepSeconds= std::clamp(delta, 0.0, k_maxStepSeconds);
	}
	if (!state.bHasTime || timeSeconds > state.lastTimeSeconds)
		state.lastTimeSeconds= timeSeconds;
	state.bHasTime= true;

	m_registers.resize((size_t)program.registerCount);
	std::copy(inputs, inputs + inputCount, m_registers.begin());
	std::copy(program.constants.begin(), program.constants.end(), m_registers.begin() + program.constantBase);

	float* reg= m_registers.data();
	for (const FaceExprInstruction& instruction : program.instructions)
	{
		const std::array<int32_t, 5>& a= instruction.args;
		float result= 0.f;
		switch (instruction.op)
		{
		case eFaceExprOp::Add: result= reg[a[0]] + reg[a[1]]; break;
		case eFaceExprOp::Sub: result= reg[a[0]] - reg[a[1]]; break;
		case eFaceExprOp::Mul: result= reg[a[0]] * reg[a[1]]; break;
		case eFaceExprOp::Div: result= reg[a[1]] != 0.f ? reg[a[0]] / reg[a[1]] : 0.f; break;
		case eFaceExprOp::Neg: result= -reg[a[0]]; break;
		case eFaceExprOp::Not: result= boolValue(!truthy(reg[a[0]])); break;
		case eFaceExprOp::Less: result= boolValue(reg[a[0]] < reg[a[1]]); break;
		case eFaceExprOp::LessEqual: result= boolValue(reg[a[0]] <= reg[a[1]]); break;
		case eFaceExprOp::Greater: result= boolValue(reg[a[0]] > reg[a[1]]); break;
		case eFaceExprOp::GreaterEqual: result= boolValue(reg[a[0]] >= reg[a[1]]); break;
		case eFaceExprOp::Equal: result= boolValue(reg[a[0]] == reg[a[1]]); break;
		case eFaceExprOp::NotEqual: result= boolValue(reg[a[0]] != reg[a[1]]); break;
		case eFaceExprOp::And: result= boolValue(truthy(reg[a[0]]) && truthy(reg[a[1]])); break;
		case eFaceExprOp::Or: result= boolValue(truthy(reg[a[0]]) || truthy(reg[a[1]])); break;
		case eFaceExprOp::Min: result= std::min(reg[a[0]], reg[a[1]]); break;
		case eFaceExprOp::Max: result= std::max(reg[a[0]], reg[a[1]]); break;
		case eFaceExprOp::Clamp:
			// A reversed range clamps to its first bound rather than asserting
			result= std::max(reg[a[1]], std::min(reg[a[0]], reg[a[2]]));
			break;
		case eFaceExprOp::Abs: result= std::fabs(reg[a[0]]); break;
		case eFaceExprOp::Pow: result= std::pow(reg[a[0]], reg[a[1]]); break;
		case eFaceExprOp::Lerp: result= reg[a[0]] + (reg[a[1]] - reg[a[0]]) * reg[a[2]]; break;
		case eFaceExprOp::Remap:
		{
			const float x= reg[a[0]];
			const float inLo= reg[a[1]];
			const float inHi= reg[a[2]];
			float t;
			if (inHi == inLo)
				t= x >= inHi ? 1.f : 0.f;
			else
				t= std::clamp((x - inLo) / (inHi - inLo), 0.f, 1.f);
			result= reg[a[3]] + (reg[a[4]] - reg[a[3]]) * t;
			break;
		}
		case eFaceExprOp::Smoothstep:
		{
			const float edge0= reg[a[0]];
			const float edge1= reg[a[1]];
			const float x= reg[a[2]];
			float t;
			if (edge1 == edge0)
				t= x >= edge1 ? 1.f : 0.f;
			else
				t= std::clamp((x - edge0) / (edge1 - edge0), 0.f, 1.f);
			result= t * t * (3.f - 2.f * t);
			break;
		}
		case eFaceExprOp::Select: result= truthy(reg[a[0]]) ? reg[a[1]] : reg[a[2]]; break;
		case eFaceExprOp::Smooth:
		{
			FaceExprState::Slot& slot= state.slots[(size_t)instruction.stateSlot];
			const float x= reg[a[0]];
			const float tau= reg[a[1]];
			if (!slot.bInitialized || tau <= 0.f)
			{
				slot.value= x;
				slot.bInitialized= true;
			}
			else
			{
				const float alpha= 1.f - std::exp(-(float)stepSeconds / tau);
				slot.value+= alpha * (x - slot.value);
			}
			result= slot.value;
			break;
		}
		case eFaceExprOp::Hysteresis:
		{
			FaceExprState::Slot& slot= state.slots[(size_t)instruction.stateSlot];
			const float x= reg[a[0]];
			bool bLatched= slot.bInitialized && slot.value > 0.5f;
			if (!bLatched && x >= reg[a[1]])
				bLatched= true;
			else if (bLatched && x <= reg[a[2]])
				bLatched= false;
			slot.value= boolValue(bLatched);
			slot.bInitialized= true;
			result= slot.value;
			break;
		}
		}
		if (!std::isfinite(result))
			result= 0.f;
		reg[instruction.dst]= result;
	}

	for (size_t outputIndex= 0; outputIndex < program.outputs.size(); ++outputIndex)
		outValues[outputIndex]= reg[program.outputs[outputIndex].resultRegister];
	return true;
}

float FaceExprEvaluator::getInputValue(int slot) const
{
	if (m_program == nullptr || slot < 0 || slot >= m_program->inputCount || slot >= (int)m_registers.size())
		return 0.f;
	return m_registers[(size_t)slot];
}

float FaceExprEvaluator::getLetValue(int letIndex) const
{
	if (m_program == nullptr || letIndex < 0 || letIndex >= (int)m_program->lets.size())
		return 0.f;
	const int resultRegister= m_program->lets[(size_t)letIndex].resultRegister;
	return resultRegister >= 0 && resultRegister < (int)m_registers.size() ? m_registers[(size_t)resultRegister] : 0.f;
}
