#include "stats.h"

void Stats::add_render_time(float render_time)
{
    last_render_time = render_time;

    s_render_times.push_back(render_time);

    if (render_time < best_render_time)
        best_render_time = render_time;
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

int Stats::last_accelerator_size_in_bytes;

float Stats::best_render_time = 1024.0f;

std::vector<float> Stats::s_render_times;