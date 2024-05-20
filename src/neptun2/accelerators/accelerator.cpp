#include "accelerator.h"

namespace neptun
{

void Accelerator::intersect1_stats(RayHit& ray_hit, Stats& stats) const
{
    intersect1(ray_hit);
}

size_t Accelerator::get_size_in_bytes() const
{
    return 0;
}

} // end of namespace neptun
