#include "accelerator.h"

namespace neptun
{

namespace detail
{
    template <size_t N>
    inline void intersect_all(const Accelerator* accel, RayHit_<N>& ray_hit)
    {
        for (int i = 0; i < N; ++i)
        {
            RayHit rt;
            ray_cpy(ray_hit.ray, rt.ray, i);

            accel->intersect1(rt);

            ray_hit.ray.max_t[i] = rt.ray.max_t;
            ray_hit.ray.tet_id[i] = rt.ray.tet_id;

            hit_cpy(rt.hit, ray_hit.hit, i);
        }
    }

    template <size_t N>
    inline void intersect_all_stats(const Accelerator* accel, RayHit_<N>& ray_hit, Stats_<N>& stats)
    {
        for (int i = 0; i < N; ++i)
        {
            RayHit rt;
            Stats st;
            ray_cpy(ray_hit.ray, rt.ray, i);

            accel->intersect1_stats(rt, st);

            ray_hit.ray.max_t[i] = rt.ray.max_t;
            ray_hit.ray.tet_id[i] = rt.ray.tet_id;

            hit_cpy(rt.hit, ray_hit.hit, i);
            stats_cpy(st, stats, i);
        }
    }
}

void Accelerator::intersect4(RayHit4& ray_hit4) const
{
    detail::intersect_all(this, ray_hit4);
}

void Accelerator::intersect8(RayHit8& ray_hit8) const
{
    detail::intersect_all(this, ray_hit8);    
}

void Accelerator::intersect16(RayHit16& ray_hit16) const
{
    detail::intersect_all(this, ray_hit16);    
}

void Accelerator::intersect1_stats(RayHit& ray_hit, Stats& stats) const
{
    intersect1(ray_hit); //intersect stats defaults to non stats version
}

void Accelerator::intersect4_stats(RayHit4& ray_hit4, Stats4& stats4) const
{
    detail::intersect_all_stats(this, ray_hit4, stats4);
}

void Accelerator::intersect8_stats(RayHit8& ray_hit8, Stats8& stats8) const
{
    detail::intersect_all_stats(this, ray_hit8, stats8);
}

void Accelerator::intersect16_stats(RayHit16& ray_hit16, Stats16& stats16) const
{
    detail::intersect_all_stats(this, ray_hit16, stats16);
}

size_t Accelerator::get_size_in_bytes() const
{
    return 0;
}

size_t Accelerator::get_max_supported_packet_size() const
{
    return 1;
}

} // end of namespace neptun
