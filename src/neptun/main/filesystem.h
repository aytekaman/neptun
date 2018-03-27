#pragma once
#ifndef FILESYSTEM_HPP
#define FILESYSTEM_HPP
// Can be replaced with with c++17 filesystem library
//   signatures are similar with filesystem library
// note fs namespace is added to get rid of cases where path is a variable

#include <string>

#if defined(_WIN32) || defined(_WIN64)

#else
#include <sys/stat.h>
#endif

namespace fs
{
class path
{
public:
    path()
    {

    }

    path(const std::string &str) : path_str(str) {}

    path(const char* c_str) : path_str(std::string(c_str)) {}


    path filename() const
    {
        const std::string::size_type index = path_str.find_last_of("/\\");
        if (index == std::string::npos)
            return path();
        else
            return path(path_str.substr(index + 1));
    }

    path stem() const
    {
        const std::string file_name = filename();
        const std::string::size_type index = file_name.find_last_of(".");

        if (index == std::string::npos)
            return path();
        else
            return path(file_name.substr(0, index));
    }

    path extension()
    {
        const std::string::size_type index = path_str.find_last_of(".");

        if (index == std::string::npos)
            return path();
        else
            return path(path_str.substr(index));
    }

    operator std::string() const { return path_str; }
private:
    std::string path_str;
};

inline bool create_directory( const path& p )
{
    std::string path_str = p;
#if defined(_WIN32) || defined(_WIN64)
    return _mkdir(path_str.c_str()) == 0;
#else
    return mkdir(path_str.c_str(), 0777) == 0;
#endif
}
}

#endif
