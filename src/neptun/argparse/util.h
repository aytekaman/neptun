#pragma once

#include <sstream>
#include <string>

namespace argparse
{

// Type of the argument
enum class ArgumentType
{
    STRING,
    BOOL,
    INTEGER,
    FLOAT
};

template <typename T>
inline bool parse_string(const std::string& value, T& t)
{
    std::stringstream ss(value);
    ss >> t;
    if (ss.fail())
        return false;
        
    if (ss.peek() == EOF)
        return true;
    
    return ((ss >> std::ws).fail() == false) && (ss.peek() == EOF);
}

// Special parsing for boolean values 
template <>
inline bool parse_string<bool>(const std::string& value, bool& t)
{
    if (value == "true" || value == "1")
        t = true;
    else if (value == "false" || value == "0" || value.size() == 0)
        t = false;
    else
        return false;

    return true;
}

template <typename T>
inline bool can_parse(const std::string& value)
{
    T t;
    return parse_string<T>(value, t);
}

inline bool can_parse(const std::string& value, const ArgumentType& type)
{
    switch(type)
    {
        case ArgumentType::BOOL:    return can_parse<bool>(value);
        case ArgumentType::FLOAT:   return can_parse<float>(value);
        case ArgumentType::INTEGER: return can_parse<int>(value);
        case ArgumentType::STRING:  return true;
    }
    return false;
}

struct ArgumentError
{
    enum class Type
    {
        SUCCESS,
        ERROR,
        WARNING
    };

    explicit ArgumentError(const Type& type, const std::string& desc = "") noexcept
        : type(type),
            desc(desc)
    {
    }

    bool is_error() const
    {
        return type == Type::ERROR;
    }

    bool is_warning() const
    {
        return type == Type::WARNING;
    }

    bool is_success() const
    {
        return type == Type::SUCCESS;
    }

    const Type          type;
    const std::string   desc;
};

// Convenient functions for throwing errors
inline ArgumentError ARG_SUCCESS()
{
    return ArgumentError(ArgumentError::Type::SUCCESS);
}

inline ArgumentError ARG_ERROR(const std::string& desc)
{
    return ArgumentError(ArgumentError::Type::ERROR, desc);
}

inline ArgumentError ARG_WARNING(const std::string& desc)
{
    return ArgumentError(ArgumentError::Type::WARNING, desc);
}

} // end of namespace argparse
