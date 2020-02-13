#pragma once

#include "ray.h"

#include <glm/glm.hpp>

#include <string>

namespace neptun
{

struct Triangle
{
    glm::vec3 v[3];
    unsigned int geometry_id;
    unsigned int primitive_id;
};

class Accelerator
{
public:
    virtual ~Accelerator() = default;

    virtual bool build(const Triangle* primitives, size_t primitive_count) = 0;

    virtual void intersect1(RayHit& ray_hit) const = 0;
    //virtual void intersectN(RayHitN& ray_hit) const; // Not implemented

    virtual void intersect1_stats(RayHit& ray_hit, Stats& stats) const;
    //virtual void intersectN_stats(RayHitN& ray_hit, StatsN& stats) const; // Not implemented
    
    // Uniquely identifies acceleration structure.
    virtual const char* name() const = 0;

    virtual size_t get_size_in_bytes() const;

    //virtual size_t get_max_supported_packet_size() const;
private:
};

}// end of namespace neptun
