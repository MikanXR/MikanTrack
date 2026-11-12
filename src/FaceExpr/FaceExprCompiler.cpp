#include "FaceExprCompiler.h"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cstring>
#include <map>
#include <set>

#include "FaceExprFunctions.h"
#include "FaceExprInputs.h"
#include "FaceExprParser.h"

namespace
{
std::string toLower(const std::string& text)
{
	std::string lower= text;
	for (char& character : lower)
		character= (char)std::tolower((unsigned char)character);
	return lower;
}

bool binaryOpFromText(const std::string& text, eFaceExprOp& outOp)
{
	static const std::pair<const char*, eFaceExprOp> k_ops[]= {
		{"+", eFaceExprOp::Add},        {"-", eFaceExprOp::Sub},           {"*", eFaceExprOp::Mul},
		{"/", eFaceExprOp::Div},        {"<", eFaceExprOp::Less},          {"<=", eFaceExprOp::LessEqual},
		{">", eFaceExprOp::Greater},    {">=", eFaceExprOp::GreaterEqual}, {"==", eFaceExprOp::Equal},
		{"!=", eFaceExprOp::NotEqual},  {"&&", eFaceExprOp::And},          {"||", eFaceExprOp::Or},
	};
	for (const auto& entry : k_ops)
	{
		if (text == entry.first)
		{
			outOp= entry.second;
			return true;
		}
	}
	return false;
}

bool isValidLetName(const std::string& name)
{
	if (name.empty() || !(std::isalpha((unsigned char)name[0]) || name[0] == '_'))
		return false;
	return std::all_of(name.begin(), name.end(),
					   [](char character) { return std::isalnum((unsigned char)character) || character == '_'; });
}

// One let or output on its way through the compiler
struct Unit
{
	bool bLet= false;
	int index= -1;
	std::string text;
	// The let or output it belongs to, for the state slot keys
	std::string owner;
	std::unique_ptr<FaceExprNode> root;
	bool bFailed= false;
	// The lets it names, with where it names them
	std::vector<std::pair<int, FaceExprSpan>> letReferences;
	std::set<int> directInputs;
	std::set<int> directLets;
	int stateOrdinal= 0;
	int resultRegister= -1;
};

class Compiler
{
public:
	Compiler(const FaceProfile& profile, const ExprInputTable& table)
		: m_profile(profile)
		, m_table(table)
	{
		m_program.inputCount= table.getCount();
	}

	FaceExprCompileResult run()
	{
		buildUnits();
		for (Unit& unit : m_units)
		{
			if (!unit.bFailed)
				collectReferences(*unit.root, unit);
		}
		const std::vector<int> letOrder= orderLets();

		collectConstants();
		m_program.constantBase= m_program.inputCount;
		m_program.registerCount= m_program.constantBase + (int)m_program.constants.size();

		for (int letIndex : letOrder)
			emitUnit(m_units[(size_t)letIndex]);
		for (size_t unitIndex= m_profile.lets.size(); unitIndex < m_units.size(); ++unitIndex)
			emitUnit(m_units[unitIndex]);

		FaceExprCompileResult result;
		result.errors= m_errors;
		if (!m_errors.empty())
			return result;

		finishExpressions(letOrder);
		result.program= std::make_shared<const FaceExprProgram>(std::move(m_program));
		return result;
	}

private:
	void fail(Unit& unit, const FaceExprSpan& span, eFaceExprErrorCode code, std::vector<std::string> args,
			  const std::string& message, bool bOnName= false)
	{
		if (unit.bFailed)
			return;
		unit.bFailed= true;
		FaceExprCompileError error;
		error.bLet= unit.bLet;
		error.index= unit.index;
		error.bOnName= bOnName;
		error.span= span;
		error.code= code;
		error.args= std::move(args);
		error.message= message;
		m_errors.push_back(error);
	}

	void buildUnits()
	{
		for (size_t letIndex= 0; letIndex < m_profile.lets.size(); ++letIndex)
		{
			const FaceProfileLet& let= m_profile.lets[letIndex];
			Unit unit;
			unit.bLet= true;
			unit.index= (int)letIndex;
			unit.text= let.expr;
			unit.owner= "let:" + toLower(let.name);
			m_units.push_back(std::move(unit));
		}
		std::map<std::string, int> ownerCounts;
		for (size_t outputIndex= 0; outputIndex < m_profile.outputs.size(); ++outputIndex)
		{
			const FaceProfileOutput& output= m_profile.outputs[outputIndex];
			Unit unit;
			unit.index= (int)outputIndex;
			unit.text= output.vrmExpressionFunction;
			const std::string owner=
				std::string("out:") + faceTargetKindName(output.targetKind) + ":" + output.mesh + ":" + output.target;
			unit.owner= owner + "#" + std::to_string(ownerCounts[owner]++);
			m_units.push_back(std::move(unit));
		}

		// The let names: unique without case, plain identifiers, no function
		for (size_t letIndex= 0; letIndex < m_profile.lets.size(); ++letIndex)
		{
			Unit& unit= m_units[letIndex];
			const std::string& name= m_profile.lets[letIndex].name;
			const std::string lower= toLower(name);
			const FaceExprSpan nameSpan{0, (int)name.size()};
			if (!isValidLetName(name))
				fail(unit, nameSpan, eFaceExprErrorCode::InvalidVariableName, {name},
					 "'" + name + "' is not a valid variable name", true);
			else if (findFaceExprFunction(name) != nullptr)
				fail(unit, nameSpan, eFaceExprErrorCode::VariableIsFunctionName, {name}, "'" + name + "' is a function name",
					 true);
			else if (m_letsByName.count(lower) != 0)
				fail(unit, nameSpan, eFaceExprErrorCode::VariableDefinedTwice, {name},
					 "variable '" + name + "' is defined twice", true);
			else
				m_letsByName[lower]= (int)letIndex;
		}

		for (Unit& unit : m_units)
		{
			FaceExprError parseError;
			unit.root= parseFaceExpr(unit.text, parseError);
			if (unit.root == nullptr)
				fail(unit, parseError.span, parseError.code, parseError.args, parseError.message);
		}
	}

	// Resolves every name, recording the inputs and lets a unit reads
	void collectReferences(const FaceExprNode& node, Unit& unit)
	{
		if (node.kind == eFaceExprNode::Identifier)
		{
			if (node.text.find('.') != std::string::npos)
			{
				const int slot= m_table.findSlot(node.text);
				if (slot < 0)
					fail(unit, node.span, eFaceExprErrorCode::UnknownInput, {node.text}, "unknown input '" + node.text + "'");
				else
					unit.directInputs.insert(slot);
			}
			else
			{
				const auto found= m_letsByName.find(toLower(node.text));
				if (found == m_letsByName.end())
					fail(unit, node.span, eFaceExprErrorCode::UnknownName, {node.text}, "unknown name '" + node.text + "'");
				else
				{
					unit.directLets.insert(found->second);
					unit.letReferences.emplace_back(found->second, node.span);
				}
			}
		}
		for (const std::unique_ptr<FaceExprNode>& child : node.children)
			collectReferences(*child, unit);
	}

	// The lets in dependency order. A cycle fails the let that closes it.
	std::vector<int> orderLets()
	{
		std::vector<int> order;
		std::vector<int> visitState(m_profile.lets.size(), 0); // 0 new, 1 in progress, 2 done
		std::vector<int> stack;
		for (int letIndex= 0; letIndex < (int)m_profile.lets.size(); ++letIndex)
			visitLet(letIndex, visitState, stack, order);
		return order;
	}

	void visitLet(int letIndex, std::vector<int>& visitState, std::vector<int>& stack, std::vector<int>& order)
	{
		if (visitState[(size_t)letIndex] != 0)
			return;
		visitState[(size_t)letIndex]= 1;
		stack.push_back(letIndex);
		Unit& unit= m_units[(size_t)letIndex];
		for (const auto& reference : unit.letReferences)
		{
			const int dependency= reference.first;
			if (visitState[(size_t)dependency] == 1)
			{
				std::string cycle;
				const auto start= std::find(stack.begin(), stack.end(), dependency);
				for (auto it= start; it != stack.end(); ++it)
					cycle+= m_profile.lets[(size_t)*it].name + " -> ";
				cycle+= m_profile.lets[(size_t)dependency].name;
				fail(unit, reference.second, eFaceExprErrorCode::Cycle, {cycle}, "cycle: " + cycle);
				continue;
			}
			visitLet(dependency, visitState, stack, order);
			if (m_units[(size_t)dependency].bFailed)
			{
				const std::string& dependencyName= m_profile.lets[(size_t)dependency].name;
				fail(unit, reference.second, eFaceExprErrorCode::VariableHasError, {dependencyName},
					 "variable '" + dependencyName + "' has an error");
			}
		}
		stack.pop_back();
		visitState[(size_t)letIndex]= 2;
		order.push_back(letIndex);
	}

	void collectConstants()
	{
		// remap's default output range
		constantRegister(0.f);
		constantRegister(1.f);
		for (const Unit& unit : m_units)
		{
			if (unit.root != nullptr)
				collectConstants(*unit.root);
		}
	}

	void collectConstants(const FaceExprNode& node)
	{
		if (node.kind == eFaceExprNode::Number)
			constantRegister(node.number);
		for (const std::unique_ptr<FaceExprNode>& child : node.children)
			collectConstants(*child);
	}

	// The register holding a constant, added while collecting
	int constantRegister(float value)
	{
		for (size_t index= 0; index < m_program.constants.size(); ++index)
		{
			if (memcmp(&m_program.constants[index], &value, sizeof(float)) == 0)
				return m_program.inputCount + (int)index;
		}
		m_program.constants.push_back(value);
		return m_program.inputCount + (int)m_program.constants.size() - 1;
	}

	void emitUnit(Unit& unit)
	{
		if (unit.bFailed || !m_errors.empty())
		{
			// A failed program is discarded whole, so the rest only needs
			// checking, not emitting: keep collecting errors
			if (!unit.bFailed && unit.root != nullptr)
				emit(*unit.root, unit);
			return;
		}
		unit.resultRegister= emit(*unit.root, unit);
	}

	int newRegister() { return m_program.registerCount++; }

	int addInstruction(eFaceExprOp op, std::initializer_list<int> args, int stateSlot= -1)
	{
		FaceExprInstruction instruction;
		instruction.op= op;
		instruction.dst= newRegister();
		int argIndex= 0;
		for (int arg : args)
			instruction.args[(size_t)argIndex++]= arg;
		instruction.stateSlot= stateSlot;
		m_program.instructions.push_back(instruction);
		return instruction.dst;
	}

	// The register of a node's value; -1 after an error
	int emit(const FaceExprNode& node, Unit& unit)
	{
		switch (node.kind)
		{
		case eFaceExprNode::Number:
			return constantRegister(node.number);
		case eFaceExprNode::Identifier:
		{
			if (node.text.find('.') != std::string::npos)
				return m_table.findSlot(node.text);
			const auto found= m_letsByName.find(toLower(node.text));
			if (found == m_letsByName.end())
				return -1;
			return m_units[(size_t)found->second].resultRegister;
		}
		case eFaceExprNode::Unary:
		{
			const int operand= emit(*node.children[0], unit);
			if (operand < 0)
				return -1;
			return addInstruction(node.text == "!" ? eFaceExprOp::Not : eFaceExprOp::Neg, {operand});
		}
		case eFaceExprNode::Binary:
		{
			const int left= emit(*node.children[0], unit);
			const int right= emit(*node.children[1], unit);
			eFaceExprOp op;
			if (left < 0 || right < 0 || !binaryOpFromText(node.text, op))
				return -1;
			return addInstruction(op, {left, right});
		}
		case eFaceExprNode::Call:
			return emitCall(node, unit);
		}
		return -1;
	}

	int emitCall(const FaceExprNode& node, Unit& unit)
	{
		const FaceExprSpan nameSpan{node.span.begin, node.span.begin + (int)node.text.size()};
		const FaceExprFunctionInfo* function= findFaceExprFunction(node.text);
		if (function == nullptr)
		{
			fail(unit, nameSpan, eFaceExprErrorCode::UnknownFunction, {node.text}, "unknown function '" + node.text + "'");
			return -1;
		}
		const int argCount= (int)node.children.size();
		const bool bBadRemap= function->op == eFaceExprOp::Remap && argCount == 4;
		if (argCount < function->minArgs || argCount > function->maxArgs || bBadRemap)
		{
			std::string expected;
			if (function->op == eFaceExprOp::Remap)
				expected= "3 or 5";
			else if (function->maxArgs == INT_MAX)
				expected= "at least " + std::to_string(function->minArgs);
			else
				expected= std::to_string(function->minArgs);
			const std::string maxText= function->maxArgs == INT_MAX ? std::string() : std::to_string(function->maxArgs);
			fail(unit, node.span, eFaceExprErrorCode::ArgumentCount,
				 {function->name, std::to_string(function->minArgs), maxText, std::to_string(argCount)},
				 std::string(function->name) + " takes " + expected + " arguments, not " + std::to_string(argCount));
			return -1;
		}

		std::vector<int> args;
		for (const std::unique_ptr<FaceExprNode>& child : node.children)
			args.push_back(emit(*child, unit));
		if (std::any_of(args.begin(), args.end(), [](int arg) { return arg < 0; }))
			return -1;

		switch (function->op)
		{
		case eFaceExprOp::Min:
		case eFaceExprOp::Max:
		{
			int result= addInstruction(function->op, {args[0], args[1]});
			for (size_t argIndex= 2; argIndex < args.size(); ++argIndex)
				result= addInstruction(function->op, {result, args[argIndex]});
			return result;
		}
		case eFaceExprOp::Abs:
			return addInstruction(function->op, {args[0]});
		case eFaceExprOp::Pow:
			return addInstruction(function->op, {args[0], args[1]});
		case eFaceExprOp::Remap:
			if (args.size() == 3)
				return addInstruction(function->op, {args[0], args[1], args[2], constantRegister(0.f), constantRegister(1.f)});
			return addInstruction(function->op, {args[0], args[1], args[2], args[3], args[4]});
		case eFaceExprOp::Smooth:
		case eFaceExprOp::Hysteresis:
		{
			const int stateSlot= (int)m_program.stateSlotKeys.size();
			m_program.stateSlotKeys.push_back(unit.owner + "|" + unit.text + "|" + std::to_string(unit.stateOrdinal++));
			if (function->op == eFaceExprOp::Smooth)
				return addInstruction(function->op, {args[0], args[1]}, stateSlot);
			return addInstruction(function->op, {args[0], args[1], args[2]}, stateSlot);
		}
		default:
			// clamp, lerp, smoothstep, select
			return addInstruction(function->op, {args[0], args[1], args[2]});
		}
	}

	void finishExpressions(const std::vector<int>& letOrder)
	{
		const size_t letCount= m_profile.lets.size();
		m_program.lets.resize(letCount);
		m_program.letOrder= letOrder;
		// Dependency order, so a let's lets are finished before it
		for (int letIndex : letOrder)
			finishExpression(m_units[(size_t)letIndex], m_program.lets[(size_t)letIndex]);
		m_program.outputs.resize(m_profile.outputs.size());
		for (size_t outputIndex= 0; outputIndex < m_profile.outputs.size(); ++outputIndex)
			finishExpression(m_units[letCount + outputIndex], m_program.outputs[outputIndex]);
	}

	void finishExpression(const Unit& unit, FaceExprProgram::Expression& outExpression)
	{
		outExpression.resultRegister= unit.resultRegister;
		outExpression.directInputs.assign(unit.directInputs.begin(), unit.directInputs.end());
		outExpression.directLets.assign(unit.directLets.begin(), unit.directLets.end());
		std::set<int> inputs= unit.directInputs;
		std::set<int> lets= unit.directLets;
		for (int letIndex : unit.directLets)
		{
			const FaceExprProgram::Expression& let= m_program.lets[(size_t)letIndex];
			inputs.insert(let.inputs.begin(), let.inputs.end());
			lets.insert(let.lets.begin(), let.lets.end());
		}
		outExpression.inputs.assign(inputs.begin(), inputs.end());
		outExpression.lets.assign(lets.begin(), lets.end());
	}

	const FaceProfile& m_profile;
	const ExprInputTable& m_table;
	std::vector<Unit> m_units;
	std::map<std::string, int> m_letsByName;
	std::vector<FaceExprCompileError> m_errors;
	FaceExprProgram m_program;
};
} // namespace

FaceExprCompileResult compileFaceProfile(const FaceProfile& profile, const ExprInputTable& table)
{
	Compiler compiler(profile, table);
	return compiler.run();
}

FaceExprCompileResult compileFaceProfile(const FaceProfile& profile)
{
	return compileFaceProfile(profile, FaceExprInputs::table());
}

bool isFaceExprFunctionName(const std::string& name)
{
	return findFaceExprFunction(name) != nullptr;
}
