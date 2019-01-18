#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "argument.h"
#include "util.h"

namespace argparse
{
    // Forward decleration
    class ArgumentParser;
    class ArgumentData;

    class ArgumentData
    {
    public:
        template <typename T>
        using map_type = std::map<std::string, T>;

        ArgumentData(const ArgumentData&) = delete;
        ArgumentData(ArgumentData&&) = default;
        ArgumentData& operator=(const ArgumentData&) = delete;
        ArgumentData& operator=(ArgumentData&&) = default;

        // Argument getters
        const Argument* get_argument(const std::string& arg_name) const;
        Argument* get_argument(const std::string& arg_name);

        const PositionalArgument* get_positional_argument(const size_t arg_position) const;
        PositionalArgument* get_positional_argument(const size_t arg_position);

        const KeywordArgument* get_keyword_argument_by_name(const std::string& arg_name) const;
        KeywordArgument* get_keyword_argument_by_name(const std::string& arg_name);
        
        const KeywordArgument* get_keyword_argument_by_short_name(const std::string& arg_short_name) const;
        KeywordArgument* get_keyword_argument_by_short_name(const std::string& arg_short_name);

        // Easy access operator overloads
        const Argument* operator[](const std::string& arg_name) const;
        Argument* operator[](const std::string& arg_name);
        const PositionalArgument* operator[](const size_t arg_position) const;
        PositionalArgument* operator[](const size_t arg_position);

        // Util 
        bool has_argument(const std::string& arg_name) const;
        std::size_t positional_argument_size() const;
        std::size_t keyword_argument_size() const;

        // Error handling
        bool has_errors() const;
        bool has_warnings() const;

        const std::set<std::string>& get_errors() const;
        const std::set<std::string>& get_warnings() const;

        std::ostream& print_errors(std::ostream& os = std::cerr) const;
        std::ostream& print_warnings(std::ostream& os = std::cerr) const;
        std::ostream& print_usage(std::ostream& os = std::cout) const;
    private:
        friend class ArgumentParser;

        map_type<std::unique_ptr<Argument>> m_arguments;
        std::vector<PositionalArgument*> m_positional_arguments;
        map_type<KeywordArgument*> m_keyword_arguments_by_name;
        map_type<KeywordArgument*> m_keyword_arguments_by_short_name;

        // Error handling
        std::set<std::string> m_warnings;
        std::set<std::string> m_errors;

        // ArgumentParser reference
        const ArgumentParser* const m_parser;

        // Private functions 
        explicit ArgumentData(const ArgumentParser* const parser);
        bool throw_error(const ArgumentError& err);

        ArgumentError add_positional_argument(
            const std::string& name,
            const std::string& desc,
            const ArgumentType type = ArgumentType::STRING);

        ArgumentError add_keyword_argument(
            const std::string& name,
            const std::string& desc,
            const ArgumentType type = ArgumentType::STRING,
            const std::string& short_name = "",
            const std::string& default_value = "");
    }; // end of class ArgumentData

    class ArgumentParser
    {
    public:
        using callback_type = std::function<int(const ArgumentData&)>;
        
        const std::string program_name;
        const std::string program_desc;
        const callback_type callback_fun; 

        explicit ArgumentParser(
            const std::string&      program_name,
            const std::string&      program_desc,
            const callback_type&    callback) noexcept;

        ArgumentParser(const ArgumentParser&) = delete;
        ArgumentParser(ArgumentParser&&) = default;
        ArgumentParser& operator=(const ArgumentParser&) = delete;
        ArgumentParser& operator=(ArgumentParser&&) = default;

        ArgumentParser& add_positional_argument(
            const std::string& name,
            const std::string& desc,
            const ArgumentType type = ArgumentType::STRING);

        ArgumentParser& add_keyword_argument(
            const std::string& name,
            const std::string& desc,
            const ArgumentType type = ArgumentType::STRING,
            const std::string& short_name = "",
            const std::string& default_value = "");

        int parse(const int argc, char const* argv[]);
        int operator()(const int argc, char const* argv[]);

        // Error handling
        bool has_errors() const;
        bool has_warnings() const;
        const std::set<std::string>& get_errors() const;
        const std::set<std::string>& get_warnings() const;
        std::ostream& print_usage(std::ostream& os = std::cout) const;

    private:
        ArgumentData m_arg_data;

        bool throw_error(const ArgumentError& err);
        bool parse_args(const int argc, char const* argv[]);
    }; // end of class ArgumentParser
    
}// end of namespace argparse 
