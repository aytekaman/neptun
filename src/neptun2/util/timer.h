#pragma once

#include <chrono>
#include <type_traits>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#endif

namespace neptun
{

class Timer
{
private:
    // Type definitions
#ifdef _WIN32
    using time_point_t = LARGE_INTEGER;
#else
    using clock_type = typename std::conditional<
        std::chrono::high_resolution_clock::is_steady,
        std::chrono::high_resolution_clock,
        std::chrono::steady_clock>::type;
    using time_point_t = decltype(clock_type::now());
#endif

    static void get_time(time_point_t* tp)
    {
#ifdef _WIN32
        QueryPerformanceCounter(tp);
#else
        * tp = clock_type::now();
#endif
    }

public:

    Timer()
    {
        m_running = false;

#ifdef _WIN32
        QueryPerformanceFrequency(&m_performance_frequency);
#endif
    }

    void start()
    {
        m_running = true;
        get_time(&m_start_time);
    }

    void stop()
    {
        m_running = false;
        get_time(&m_stop_time);
    }

    template <typename Type, typename Period>
    Type time_passed()
    {
        if (m_running)
        {
            get_time(&m_stop_time);
        }

#ifdef _WIN32
        double num = Period::den * double(m_stop_time.QuadPart - m_start_time.QuadPart);
        double den = Period::num * double(m_performance_frequency.QuadPart);
        return static_cast<Type>(num / den);
#else
        return std::chrono::duration<Type, Period>(m_stop_time - m_start_time).count();
#endif
    }

    double milliseconds()
    {
        return time_passed<double, std::milli>();
    }

    double seconds()
    {
        return time_passed<double, std::ratio<1, 1>>();
    }

    double nanoseconds()
    {
        return time_passed<double, std::nano>();
    }

private:
    time_point_t m_start_time, m_stop_time;
    bool         m_running;

#ifdef _WIN32
    LARGE_INTEGER m_performance_frequency;
#endif
};

// Quickly measure time
template <typename Func, typename... FuncArgs>
inline double measure_time(Func func, FuncArgs&&... args)
{
    Timer timer;
    timer.start();
    func(std::forward<FuncArgs>(args)...);
    timer.stop();
    return timer.milliseconds();
}

} // end of namespace neptun
