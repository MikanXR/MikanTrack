#pragma once

#include <memory>
#include <string>
#include <vector>

#include "FaceExprProgram.h"
#include "FaceSources.h"

// The state of a program's stateful call sites (smooth, hysteresis) for one
// evaluator, plus the clock they step on. Kept apart from the program so one
// compiled program can drive several evaluators (the live stream, a replay).
struct FaceExprState
{
	struct Slot
	{
		float value= 0.f;
		bool bInitialized= false;
	};

	// Reinitializes every slot and forgets the clock: the next evaluate
	// starts each stateful call from its input. For a seek or a source jump.
	void reset();

	// Adopts a program, carrying over the slots whose key it still has and
	// starting the rest fresh. A no-op for the program already bound.
	void bindTo(const std::shared_ptr<const FaceExprProgram>& program);

	std::shared_ptr<const FaceExprProgram> program;
	std::vector<Slot> slots;
	bool bHasTime= false;
	double lastTimeSeconds= 0.0;
};

// Runs a compiled program over a frame's sources.
//
// Time: each stateful call steps by the frame timestamp's advance, clamped to
// [0, k_maxStepSeconds]. A small regression (interleaved camera and face
// frames arriving a few milliseconds out of order) steps nothing and keeps
// the clock where it was. Only a regression of more than k_maxStepSeconds, or
// an explicit FaceExprState::reset, reinitializes the state.
class FaceExprEvaluator
{
public:
	static constexpr double k_maxStepSeconds= 1.0;

	void setProgram(std::shared_ptr<const FaceExprProgram> program) { m_program= std::move(program); }
	const std::shared_ptr<const FaceExprProgram>& getProgram() const { return m_program; }

	// outValues receives one raw (unclamped) value per program output.
	// Non-finite intermediate results are replaced with zero.
	//
	// inputs holds one value per slot of the table the program was compiled
	// against, in slot order, and inputCount says how many. A count other
	// than the program's inputCount (or null inputs) runs nothing: every
	// output and every inspector value reads 0, the state does not step, and
	// it returns false. Without a program it writes nothing and returns false.
	bool evaluate(const float* inputs, int inputCount, double timeSeconds, FaceExprState& state, float* outValues);

	// The face table's form: fills the inputs from the sources
	void evaluate(const FaceSources& sources, double timeSeconds, FaceExprState& state, float* outValues);

	// The last evaluate's values, for the inspector
	float getInputValue(int slot) const;
	float getLetValue(int letIndex) const;

private:
	std::shared_ptr<const FaceExprProgram> m_program;
	std::vector<float> m_registers;
};
