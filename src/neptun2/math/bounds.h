#pragma once

#include <neptun2/accelerators/ray.h>

#include <glm/glm.hpp>

namespace neptun
{
constexpr float MACHINE_EPSILON = std::numeric_limits<float>::epsilon() * 0.5;

class Bounds3
{
public:
    Bounds3();
    Bounds3(const glm::vec3 min_p, const glm::vec3 max_p);

    bool intersect_p(const Ray& ray, float* hitt0, float* hitt1) const;
    inline bool intersect_p(const Ray& ray, const glm::vec3& inv_dir, const int dir_is_neg[3]) const;

    inline const glm::vec3& operator[](int i) const;

    glm::vec3 diagonal() const;
    float surface_area() const;
    float volume() const;
    int max_extent() const;
    glm::vec3 offset(const glm::vec3& p) const;
    
    void add(const Bounds3& bounds);
    void add(const glm::vec3& point);
    static Bounds3 add(const Bounds3& b1, const Bounds3& b2);

    glm::vec3 m_min_p;
    glm::vec3 m_max_p;
};

// Inline members
inline bool Bounds3::intersect_p(const Ray& ray, const glm::vec3& inv_dir, const int dir_is_neg[3]) const
{
    const Bounds3& bounds = *this;

    // Check for ray intersection against $x$ and $y$ slabs
    float t_min = (bounds[dir_is_neg[0]].x - ray.org.x) * inv_dir.x;
    float t_max = (bounds[1 - dir_is_neg[0]].x - ray.org.x) * inv_dir.x;
    float ty_min = (bounds[dir_is_neg[1]].y - ray.org.y) * inv_dir.y;
    float ty_max = (bounds[1 - dir_is_neg[1]].y - ray.org.y) * inv_dir.y;

    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    t_max *= 1 + 2 * ((3 * MACHINE_EPSILON) / (1 - 3 * MACHINE_EPSILON));
    ty_max *= 1 + 2 * ((3 * MACHINE_EPSILON) / (1 - 3 * MACHINE_EPSILON));
    if (t_min > ty_max || ty_min > t_max) return false;
    if (ty_min > t_min) t_min = ty_min;
    if (ty_max < t_max) t_max = ty_max;

    // Check for ray intersection against $z$ slab
    float tz_min = (bounds[dir_is_neg[2]].z - ray.org.z) * inv_dir.z;
    float tz_max = (bounds[1 - dir_is_neg[2]].z - ray.org.z) * inv_dir.z;

    // Update _tzMax_ to ensure robust bounds intersection
    tz_max *= 1 + 2 * ((3 * MACHINE_EPSILON) / (1 - 3 * MACHINE_EPSILON));
    if (t_min > tz_max || tz_min > t_max) return false;
    if (tz_min > t_min) t_min = tz_min;
    if (tz_max < t_max) t_max = tz_max;

    return (t_min < ray.max_t) && (t_max > 0);
}

inline const glm::vec3& Bounds3::operator[](int i) const
{
    return (i == 0) ? m_min_p : m_max_p;
}

} // end of namespace neptun
