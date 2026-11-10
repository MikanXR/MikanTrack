#include "ExprInputTable.h"

#include <cctype>
#include <utility>

namespace
{
std::string toLower(const std::string& text)
{
	std::string lower= text;
	for (char& character : lower)
		character= (char)std::tolower((unsigned char)character);
	return lower;
}
} // namespace

ExprNameTable::ExprNameTable(std::vector<std::string> names)
	: m_names(std::move(names))
{
	for (int slot= 0; slot < (int)m_names.size(); ++slot)
		m_slotsByLowerName.emplace(toLower(m_names[(size_t)slot]), slot);
}

int ExprNameTable::findSlot(const std::string& name) const
{
	const auto found= m_slotsByLowerName.find(toLower(name));
	return found != m_slotsByLowerName.end() ? found->second : -1;
}

std::string ExprNameTable::getName(int slot) const
{
	return slot >= 0 && slot < (int)m_names.size() ? m_names[(size_t)slot] : std::string();
}
