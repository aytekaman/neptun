#pragma once

#include <string>
#include <typeinfo>

#include "util.h"

namespace argparse
{
    
class Argument
{
public:
    const std::string name;
    const std::string desc;
    const ArgumentType type;
    
    explicit Argument(
        const std::string&  name,
        const std::string&  desc,
        const ArgumentType  type) noexcept;

    explicit Argument(
        const std::string&  name,
        const std::string&  desc,
        const ArgumentType  type,
        const std::string&  value) noexcept;

    bool has_value() const;
    bool value_changed() const;

    const std::string& value() const;
    bool set_value(const std::string& value);

    template <typename T>
    T cast() const
    {
        T t;
        const bool s = parse_string<T>(m_value, t);
        if (s == false)
            throw std::bad_cast();

        return t;
    }

    template <typename T>
    bool can_cast() const
    {
        return can_cast<T>(m_value);
    }
    
private:
    std::string m_value;
    bool m_value_changed;

    bool check_value() const;
};

class PositionalArgument : public Argument
{
public:
    const size_t position;
    explicit PositionalArgument(
        const std::string&  name,
        const std::string&  desc,
        const ArgumentType  type,
        const size_t        position) noexcept;
};

class KeywordArgument : public Argument
{
public:
    const std::string short_name;
    const std::string default_value;
    
    explicit KeywordArgument(
        const std::string&  name,
        const std::string&  desc,
        const ArgumentType  type,
        const std::string&  short_name,
        const std::string&  default_value) noexcept;

    bool has_short_name() const;
    bool has_default_value() const;
};

} // end of namespace argparse
