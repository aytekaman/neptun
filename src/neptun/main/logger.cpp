#include "logger.h"

void Logger::Clear()
{
    logs.clear();
}

void Logger::Log(const char * fmt, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 1024, fmt, args);

    Msg msg;
    msg.text = buffer;
    msg.type = 0;

    logs.push_back(msg);

    va_end(args);
}

void Logger::LogWarning(const char * fmt, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 1024, fmt, args);

    Msg msg;
    msg.text = buffer;
    msg.type = 1;

    logs.push_back(msg);

    va_end(args);
}

void Logger::LogError(const char * fmt, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 1024, fmt, args);

    Msg msg;
    msg.text = buffer;
    msg.type = 2;

    logs.push_back(msg);

    va_end(args);
}

std::vector<Msg> Logger::logs;