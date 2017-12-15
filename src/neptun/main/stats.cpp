#include "stats.h"

void Stats::add_render_time(float render_time)
{
    last_render_time = render_time;

    s_render_times.push_back(render_time);
}

float Stats::get_avg_render_time(int frame_count)
{
    frame_count = s_render_times.size() < frame_count ? s_render_times.size() : frame_count;

    float render_time_sum = 0;

    for (int i = 0; i < frame_count; i++)
    {
        render_time_sum += s_render_times[s_render_times.size() - 1 - i];
    }

    return render_time_sum / frame_count;
}

void Stats::add_build_time(float build_time)
{
    last_build_time = build_time;
}

float Stats::last_build_time;
float Stats::last_render_time;

std::vector<float> Stats::s_render_times;