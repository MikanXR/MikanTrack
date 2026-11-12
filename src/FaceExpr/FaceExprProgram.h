#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

enum class eFaceExprOp : uint8_t
{
	Add,
	Sub,
	Mul,
	Div,
	Neg,
	Not,
	Less,
	LessEqual,
	Greater,
	GreaterEqual,
	Equal,
	NotEqual,
	And,
	Or,
	Min,
	Max,
	Clamp,
	Abs,
	Pow,
	Lerp,
	// remap(x, inLo, inHi, outLo, outHi): the three-argument form gets 0 and
	// 1 for the output range as constants
	Remap,
	Smoothstep,
	Select,
	// Stateful: read and write state slot stateSlot
	Smooth,
	Hysteresis,
};

// One register-machine instruction: dst = op(args...). Every operand and the
// destination index the program's one register file.
struct FaceExprInstruction
{
	eFaceExprOp op= eFaceExprOp::Add;
	int32_t dst= -1;
	std::array<int32_t, 5> args{-1, -1, -1, -1, -1};
	int32_t stateSlot= -1;
};

// An immutable compiled face profile. The register file is laid out as the
// input slots of the table it was compiled against (inputCount of them), then
// the constants, then the temporaries;
// the instructions run the lets in dependency order and then the outputs in
// profile order. A program carries no state: the stateful call sites name
// slots in a FaceExprState, so one program can drive several evaluators.
struct FaceExprProgram
{
	// One compiled let or output
	struct Expression
	{
		// Where its value is after a run (an input or constant register when
		// the expression is just that)
		int resultRegister= -1;
		// The input slots it reads itself, and through its lets, sorted
		std::vector<int> directInputs;
		std::vector<int> inputs;
		// The lets it names itself, and through other lets, by profile index,
		// sorted
		std::vector<int> directLets;
		std::vector<int> lets;
	};

	int registerCount= 0;
	// The input table's slot count: the inputs an evaluate must supply
	int inputCount= 0;
	int constantBase= 0;
	std::vector<float> constants;
	std::vector<FaceExprInstruction> instructions;
	// In profile order
	std::vector<Expression> lets;
	std::vector<Expression> outputs;
	// The let indices in the order they run: each after every let it reads
	std::vector<int> letOrder;
	// One key per state slot: the let or output it belongs to, its text, and
	// the call site's ordinal inside it. A recompile carries a slot's state
	// over when its key still exists.
	std::vector<std::string> stateSlotKeys;
};
