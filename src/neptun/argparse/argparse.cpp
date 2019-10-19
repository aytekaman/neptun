#include "argparse.h"

#include <iostream>
#include <string>
#include <utility>

#include "argument.h"
#include "util.h"

namespace argparse
{
    
// ArgumentData
// Accessors
const Argument* ArgumentData::get_argument(
    const std::string& arg_name) const
{
    auto it = m_arguments.find(arg_name);
    if (it == m_arguments.end())
    {
        return nullptr;
    }
    
    return it->second.get();
}   

Argument* ArgumentData::get_argument(
    const std::string& arg_name)
{
    auto it = m_arguments.find(arg_name);
    if (it == m_arguments.end())
    {
        return nullptr;
    }
    
    return it->second.get();
}

const PositionalArgument* ArgumentData::get_positional_argument(
    const std::size_t arg_position) const
{
    if (arg_position < m_positional_arguments.size())
        return m_positional_arguments[arg_position];
    return nullptr;
}

PositionalArgument* ArgumentData::get_positional_argument(
    const std::size_t arg_position)
{
    if (arg_position < m_positional_arguments.size())
        return m_positional_arguments[arg_position];
    return nullptr;
}

const KeywordArgument* ArgumentData::get_keyword_argument_by_name(
    const std::string& arg_name) const
{
    auto it = m_keyword_arguments_by_name.find(arg_name);
    
    if (it == m_keyword_arguments_by_name.end())
        return nullptr;
    
    return it->second;
}

KeywordArgument* ArgumentData::get_keyword_argument_by_name(
    const std::string& arg_name)
{
    auto it = m_keyword_arguments_by_name.find(arg_name);
    
    if (it == m_keyword_arguments_by_name.end())
        return nullptr;
    
    return it->second;
}

const KeywordArgument* ArgumentData::get_keyword_argument_by_short_name(
    const std::string& arg_short_name) const
{
    auto it = m_keyword_arguments_by_short_name.find(arg_short_name);
    
    if (it == m_keyword_arguments_by_short_name.end())
        return nullptr;
    
    return it->second;
}

KeywordArgument* ArgumentData::get_keyword_argument_by_short_name(
    const std::string& arg_short_name)
{
    auto it = m_keyword_arguments_by_short_name.find(arg_short_name);
    
    if (it == m_keyword_arguments_by_short_name.end())
        return nullptr;
    
    return it->second;
}

// Easy access operator overloads
const Argument* ArgumentData::operator[](
    const std::string& arg_name) const
{
    return get_argument(arg_name);
}

Argument* ArgumentData::operator[](
    const std::string& arg_name)
{
    return get_argument(arg_name);
}

const PositionalArgument* ArgumentData::operator[](
    const std::size_t arg_position) const
{
    return get_positional_argument(arg_position);
}

PositionalArgument* ArgumentData::operator[](
    const std::size_t arg_position)
{
    return get_positional_argument(arg_position);
}

// Util 
bool ArgumentData::has_argument(
    const std::string& arg_name) const
{
    auto it = m_arguments.find(arg_name);
    return it != m_arguments.end();
}

std::size_t ArgumentData::positional_argument_size() const
{
    return m_positional_arguments.size();
}

std::size_t ArgumentData::keyword_argument_size() const
{
    return m_keyword_arguments_by_name.size();
}

// Error handling
bool ArgumentData::has_errors() const
{
    return m_errors.size() > 0;
}

bool ArgumentData::has_warnings() const
{
    return m_warnings.size() > 0;
}

const std::set<std::string>& ArgumentData::get_errors() const
{
    return m_errors;
}

const std::set<std::string>& ArgumentData::get_warnings() const
{
    return m_warnings;
}

// Print functions
std::ostream& ArgumentData::print_errors(std::ostream& os) const
{
    if (has_errors() == false)
        return os;

    os << m_errors.size() << " errors:\n";
    for (const auto& err : m_errors)
        os << " - " << err << "\n";
    return os;
}

std::ostream& ArgumentData::print_warnings(std::ostream& os) const
{
    if (has_warnings() == false)
        return os;

    os << m_warnings.size() << " warnings:\n";
    for (const auto& warn : m_warnings)
        os << " - " << warn << "\n";
    return os;
}

std::ostream& ArgumentData::print_usage(std::ostream& os) const
{ 
    constexpr std::size_t column_starts[2] = {2, 25};
    constexpr std::size_t max_line_length = 80;

    std::size_t line_pos = 0; // Current line position
    auto print_arg_name  = [&os, &line_pos, &column_starts](const std::string& name){
        os << std::string(column_starts[0], ' ');
        os << name;

        line_pos += name.size() + 2;
    };

    auto print_arg_description = [&os, &line_pos, &column_starts, &max_line_length](const std::string& desc){
        size_t space = 0;

        if (line_pos > column_starts[1])
        {
            space = 4;    
        }
        else
        {
            space = column_starts[1] - line_pos;
        }

        os << std::string(space, ' ');
        
        // Print description
        for (size_t i = 0; i < desc.size();)
        {
            const std::size_t characters_left = desc.size() - i;
            const std::size_t space_left = max_line_length - line_pos; 
            if (characters_left <= space_left)
            {
                os << desc.substr(i) << "\n";
                line_pos = 0;
                return;
            }
            else
            {
                // Find nearest end of word
                size_t print_idx = i + space_left;
                while (desc[print_idx] != ' ' && print_idx > i + 1)
                {
                    print_idx--;
                }

                // Print until nearest end of word
                os << desc.substr(i, print_idx) << "\n";
                os << std::string(column_starts[1], ' ');
                line_pos = column_starts[1];
                i = print_idx + 1;
            }
        }
    }; 

    // Print: Usage: prog_name [Options] args...
    os << "Usage: " << m_parser->program_name << " ";

    if (keyword_argument_size() > 0)
    {
        os << "[Options]";
    }

    for (const auto& it : m_positional_arguments)
    {
        os << " " << it->name;
    }
    os << "\n";

    // Print program description
    if (m_parser->program_desc.size() > 0)
    {
        os << std::string(column_starts[0], ' ') << m_parser->program_desc << "\n";
    }
    
    // Print positional arguments
    if (positional_argument_size() > 0)
    {
        os << "Positional Arguments:\n";

        for (const auto& arg : m_positional_arguments)
        {
            print_arg_name(arg->name);
            print_arg_description(arg->desc);
        }
    }
    
    // Print keyword args
    if (keyword_argument_size() > 0)
    {
        os << "Options:\n";
        
        for (const auto& it : m_keyword_arguments_by_name)
        {
            KeywordArgument* arg = it.second;
            const std::string name = "--" + arg->name;
            const std::string short_name = arg->has_short_name() ? (", -" + arg->short_name) : "";

            print_arg_name(name + short_name);
            print_arg_description(arg->desc);
        }
    }

    return os;
}

// ArgumentData private functions
ArgumentData::ArgumentData(
    const ArgumentParser* const parser)
    : m_parser(parser)
{
}

bool ArgumentData::throw_error(const ArgumentError& err)
{
    if (err.is_error())
    {
        m_errors.emplace(err.desc);
    }
    else if (err.is_warning())
    {
        m_warnings.emplace(err.desc);
    } // else success
    
    return err.is_error() == false; // return true if we do not have an error
}

ArgumentError ArgumentData::add_positional_argument(
    const std::string& name,
    const std::string& desc,
    const ArgumentType type)
{
    if (get_argument(name) != nullptr)
    {
        return ARG_ERROR("Argument " + name + " is already exists in argument list.");
    }

    size_t position = m_positional_arguments.size();
    PositionalArgument* arg = new PositionalArgument(name, desc, type, position);
    const auto it = m_arguments.emplace(name, std::move(std::unique_ptr<PositionalArgument>(arg)));

    if (it.second == false) // Something went wrong
    {
        return ARG_ERROR("Argument Error: Cannot create argument " + name);
    }
    else
    {
        m_positional_arguments.push_back(arg);
        return ARG_SUCCESS();
    }
}

ArgumentError ArgumentData::add_keyword_argument(
    const std::string& name,
    const std::string& desc,
    const ArgumentType type,
    const std::string& short_name,
    const std::string& default_value)
{
    if (get_argument(name) != nullptr)
    {
        return ARG_ERROR("Argument " + name + " is already exists in argument list.");
    }

    if (short_name.size() > 0 && get_keyword_argument_by_short_name(short_name) != nullptr)
    {
        return ARG_ERROR("Duplicate short names (" + short_name + ") in argument list.");
    }

    if (default_value != "" && can_parse(default_value, type) == false)
    {
        return ARG_ERROR("Cannot parse default value of argument " + name);
    }

    KeywordArgument* arg = new KeywordArgument(name, desc, type, short_name, default_value);
    const auto it = m_arguments.emplace(name, std::move(std::unique_ptr<KeywordArgument>(arg)));

    if (it.second == false) // Something went wrong
    {
        return ARG_ERROR("Argument Error: Cannot create argument " + name);
    } 
    else
    {
        m_keyword_arguments_by_name.emplace(name, arg);

        if (short_name.size() > 0)
            m_keyword_arguments_by_short_name.emplace(short_name, arg);
        return ARG_SUCCESS();
    }
}

// ArgumentParser
ArgumentParser::ArgumentParser(
    const std::string&      program_name,
    const std::string&      program_desc,
    const callback_type&    callback) noexcept
    : program_name(program_name),
        program_desc(program_desc),
        callback_fun(callback),
        m_arg_data(this)
{   
}

// Add arguments
ArgumentParser& ArgumentParser::add_positional_argument(
    const std::string& name,
    const std::string& desc,
    const ArgumentType type)
{
    throw_error(m_arg_data.add_positional_argument(name, desc, type));
    return *this;
}

ArgumentParser& ArgumentParser::add_keyword_argument(
    const std::string& name,
    const std::string& desc,
    const ArgumentType type,
    const std::string& short_name,
    const std::string& default_value)
{
    throw_error(m_arg_data.add_keyword_argument(name, desc, type, short_name, default_value));
    return *this;
}

// Parse
int ArgumentParser::parse(const int argc, char const* argv[])
{
    parse_args(argc, argv);
    return callback_fun(m_arg_data);
}

int ArgumentParser::operator()(const int argc, char const* argv[])
{
    return parse(argc, argv);
}

// Error handling
bool ArgumentParser::has_errors() const
{
    return m_arg_data.has_errors();
}

bool ArgumentParser::has_warnings() const
{
    return m_arg_data.has_warnings();
}

const std::set<std::string>& ArgumentParser::get_errors() const
{
    return m_arg_data.get_errors();
}

const std::set<std::string>& ArgumentParser::get_warnings() const
{
    return m_arg_data.get_errors();
}

std::ostream& ArgumentParser::print_usage(std::ostream& os) const
{
    return m_arg_data.print_usage(os);
}

// ArgumentParser private members
bool ArgumentParser::throw_error(const ArgumentError& err)
{
    return m_arg_data.throw_error(err);
}

bool ArgumentParser::parse_args(const int argc, char const* argv[])
{
    Argument* arg = nullptr;
    size_t positional_arg_index = 0;
    std::string cur_value;

    for (int i = 0; i < argc; i++)
    {   
        cur_value = std::string(argv[i]);

        if (arg == nullptr && cur_value[0] == '-') // Might be keyword arg (or a value starts with -)
        {
            if (cur_value.size() >= 2 && cur_value[1] == '-') // might be by name
            {
                arg = m_arg_data.get_keyword_argument_by_name(cur_value.substr(2));
            } else // by short name
            {
                arg = m_arg_data.get_keyword_argument_by_short_name(cur_value.substr(1));
            }
            
            if (arg == nullptr) // Argument not found maybe it is a positional argument starting with -?
            {
                arg = m_arg_data.get_positional_argument(positional_arg_index);
                
                if (arg != nullptr) // Add as keyword argument 
                {
                    const bool success = arg->set_value(cur_value);

                    if (success == false)
                        throw_error(ARG_ERROR("Illegal value \""+ cur_value +"\" for argument: " + arg->name));

                    positional_arg_index++;
                    arg = nullptr;
                } else 
                {
                    // Keyword argument not found 
                    throw_error(ARG_ERROR("Unrecognized argument: " + cur_value));
                }
            }
            else // Keyword argument
            {
                if (arg->value_changed())
                    throw_error(ARG_WARNING("Duplicate definition of keyword: " + arg->name));

                if (arg->type == ArgumentType::BOOL) // boolean keyword argument
                {
                    arg->set_value("true");
                    arg = nullptr;
                }
            } 
        }
        else // Positional argument or value of the keyword argument
        {
            if (arg == nullptr) // is positional argument
            {
                arg = m_arg_data.get_positional_argument(positional_arg_index);
                
                if (arg == nullptr)
                    throw_error(ARG_ERROR("Illegal value \"" + cur_value + "\": "
                        + "Number of positional arguments cannot be greater than " + std::to_string(m_arg_data.positional_argument_size())));
                
                positional_arg_index++;
            } 

            if (arg != nullptr)
            {
                const bool success = arg->set_value(cur_value);

                if (!success)
                    throw_error(ARG_ERROR("Illegal value \""+ cur_value +"\" for argument: " + arg->name));
                arg = nullptr;
            }
        }
    }

    // If we do get all of the positional args
    if (m_arg_data.positional_argument_size() != positional_arg_index)
        throw_error(ARG_ERROR("Incorrect number of arguments entered. ( required = " + std::to_string(m_arg_data.positional_argument_size()) +
            ", entered = " + std::to_string(positional_arg_index) + " )"));

    // If we have a leftover arg
    if (arg != nullptr)
        throw_error(ARG_ERROR("Unrecognized argument: " + cur_value));
    
    return m_arg_data.has_errors();
}

} // end of namespace argparse
