#include "argument.h"

namespace argparse
{
    
// Argument class definitions
Argument::Argument(
    const std::string&  name,
    const std::string&  desc,
    const ArgumentType  type) noexcept
    : name(name),
    desc(desc),
    type(type),  
    m_value(""),
    m_value_changed(false)
{
}

Argument::Argument(
    const std::string&  name,
    const std::string&  desc,
    const ArgumentType  type,
    const std::string&  value) noexcept
    : name(name),
    desc(desc),
    type(type),
    m_value(value),
    m_value_changed(false)
{
}

bool Argument::has_value() const
{
    return m_value.size() > 0;
}

bool Argument::value_changed() const
{
    return m_value_changed;
}

const std::string& Argument::value() const
{
    return m_value;
}

bool Argument::set_value(const std::string& value)
{
    m_value = value;
    m_value_changed = true;
    return check_value();
}

bool Argument::check_value() const
{
    return has_value() == false || can_parse(m_value, type);
} 

// Positional argument definitions
PositionalArgument::PositionalArgument(
        const std::string&  name,
        const std::string&  desc,
        const ArgumentType  type,
        const size_t        position) noexcept
        : Argument(name, desc, type, desc),
        position(position)
{
}

//Keyword argument definitions
KeywordArgument::KeywordArgument(
        const std::string&  name,
        const std::string&  desc,
        const ArgumentType  type,
        const std::string&  short_name,
        const std::string&  default_value) noexcept
        : Argument(name, desc, type, default_value),
        short_name(short_name),
        default_value(default_value)
{
}

bool KeywordArgument::has_short_name() const
{
    return short_name.size() > 0;
}

bool KeywordArgument::has_default_value() const
{
    return default_value.size() > 0;
}

} // end of namespace argparse
