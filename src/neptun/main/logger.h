#pragma once

#include <string>
#include <vector>

#include <stdarg.h>

struct Msg
{
	std::string text;
	int type;
};

class Logger
{
public:
	static void Clear();
	static void Log(const char *fmt, ...);
	static void LogWarning(const char *fmt, ...);
	static void LogError(const char *fmt, ...);

	static std::vector<Msg> logs;
};