#pragma once

#include <vector>

class Stats
{
public:
    static void  add_render_time(float render_time);
    static float get_avg_render_time(int frame_count);

    static void  add_build_time(float build_time);

    static float last_render_time;
    static float last_build_time;

    static float best_render_time;

    static float ray_prep_time;
    static float gpu_kernel_time;
    static float draw_time;

private:
    static std::vector<float> s_render_times;
};
