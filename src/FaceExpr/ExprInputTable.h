#pragma once

#include <string>
#include <unordered_map>
#include <vector>

// The input slots an expression reads: the dotted names the compiler
// resolves and the order a caller fills their values in. The compiler and the
// evaluator take a table rather than knowing one, so the same language runs
// over the face sources and over other inputs.
class ExprInputTable
{
public:
	virtual ~ExprInputTable()= default;

	virtual int getCount() const= 0;
	// The slot of a dotted name, compared without case; -1 for a name the
	// table lacks
	virtual int findSlot(const std::string& name) const= 0;
	// The canonical spelling of a slot; "" out of range
	virtual std::string getName(int slot) const= 0;
};

// A table built from a list of dotted names, one slot per name in list
// order. A name repeated without case keeps its first slot.
class ExprNameTable final : public ExprInputTable
{
public:
	explicit ExprNameTable(std::vector<std::string> names);

	int getCount() const override { return (int)m_names.size(); }
	int findSlot(const std::string& name) const override;
	std::string getName(int slot) const override;

private:
	std::vector<std::string> m_names;
	std::unordered_map<std::string, int> m_slotsByLowerName;
};
