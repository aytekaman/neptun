#include "accelerator.h"

namespace neptun
{
    
void Accelerator::intersect4(RayHit4& ray_hit4) const
{
    for (int i = 0; i < 4; ++i)
    {
        RayHit rt;
        rayCpy(ray_hit4.ray, rt.ray, i);

        intersect(rt);

        ray_hit4.ray.max_t[i] = rt.ray.max_t;
        ray_hit4.ray.tet_id[i] = rt.ray.tet_id;

        hitCpy(rt.hit, ray_hit4.hit, i);
    }
}

void Accelerator::intersect8(RayHit8& ray_hit8) const
{
    for (int i = 0; i < 8; ++i)
    {
        RayHit rt;
        rayCpy(ray_hit8.ray, rt.ray, i);

        intersect(rt);

        ray_hit8.ray.max_t[i] = rt.ray.max_t;
        ray_hit8.ray.tet_id[i] = rt.ray.tet_id;

        hitCpy(rt.hit, ray_hit8.hit, i);
    }
}

void Accelerator::intersect16(RayHit16& ray_hit16) const
{
    for (int i = 0; i < 16; ++i)
    {
        RayHit rt;
        rayCpy(ray_hit16.ray, rt.ray, i);

        intersect(rt);

        ray_hit16.ray.max_t[i] = rt.ray.max_t;
        ray_hit16.ray.tet_id[i] = rt.ray.tet_id;

        hitCpy(rt.hit, ray_hit16.hit, i);
    }
}

void Accelerator::intersect_stats(RayHit& ray_hit, Stats& stats) const
{
    // intersect stats defaults to non stats version
    intersect(ray_hit);
}

void Accelerator::intersect4_stats(RayHit4& ray_hit4, Stats4& stats4) const
{
    for (int i = 0; i < 4; ++i)
    {
        RayHit rt;
        Stats st;

        rayCpy(ray_hit4.ray, rt.ray, i);

        intersect(rt);

        ray_hit4.ray.max_t[i] = rt.ray.max_t;
        ray_hit4.ray.tet_id[i] = rt.ray.tet_id;

        hitCpy(rt.hit, ray_hit4.hit, i);
        statsCpy(st, stats4, i);
    }
}

void Accelerator::intersect8_stats(RayHit8& ray_hit8, Stats8& stats8) const
{
    for (int i = 0; i < 8; ++i)
    {
        RayHit rt;
        Stats st;

        rayCpy(ray_hit8.ray, rt.ray, i);

        intersect(rt);

        ray_hit8.ray.max_t[i] = rt.ray.max_t;
        ray_hit8.ray.tet_id[i] = rt.ray.tet_id;

        hitCpy(rt.hit, ray_hit8.hit, i);
        statsCpy(st, stats8, i);
    }
}

void Accelerator::intersect16_stats(RayHit16& ray_hit16, Stats16& stats16) const
{
    for (int i = 0; i < 16; ++i)
    {
        RayHit rt;
        Stats st;
        rayCpy(ray_hit16.ray, rt.ray, i);

        intersect(rt);

        ray_hit16.ray.max_t[i] = rt.ray.max_t;
        ray_hit16.ray.tet_id[i] = rt.ray.tet_id;

        hitCpy(rt.hit, ray_hit16.hit, i);
        statsCpy(st, stats16, i);
    }
}

size_t Accelerator::get_size_in_bytes() const
{
    return 0;
}

} // end of namespace neptun
