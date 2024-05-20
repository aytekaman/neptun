#include "bounds.h"

namespace neptun
{

Bounds3::Bounds3()
{
    m_min_p = glm::vec3(1000000, 1000000, 1000000);
    m_max_p = glm::vec3(-1000000, -1000000, -1000000);
}

Bounds3::Bounds3(const glm::vec3 min_p, const glm::vec3 max_p)
    : m_min_p(min_p)
    , m_max_p(max_p)
{
}

bool Bounds3::intersect_p(const Ray& ray, float* hitt0, float* hitt1) const
{
    float t0 = 0, t1 = ray.max_t;

    // For each axis
    for (int i = 0; i < 3; ++i)
    {
        // Update interval for _i_th bounding box slab
        const float inv_ray_dir = 1 / ray.dir[i];
        float t_near = (m_min_p[i] - ray.org[i]) * inv_ray_dir;
        float t_far = (m_max_p[i] - ray.org[i]) * inv_ray_dir;

        // Update parametric interval from slab intersection $t$ values
        if (t_near > t_far) std::swap(t_near, t_far);

        // Update _tFar_ to ensure robust ray--bounds intersection
        t_far *= 1 + 2 * ((3 * MACHINE_EPSILON) / (1 - 3 * MACHINE_EPSILON));
        
        t0 = t_near > t0 ? t_near : t0;
        t1 = t_far < t1 ? t_far : t1;

        if (t0 > t1) return false;
    }

    if (hitt0)
        *hitt0 = t0;

    if (hitt1)
        *hitt1 = t1;

    return true;
}

glm::vec3 Bounds3::diagonal() const
{
    return m_max_p - m_min_p;
}

float Bounds3::surface_area() const
{
    glm::vec3 diag = diagonal();
    return 2 * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z);
}

float Bounds3::volume() const
{
    glm::vec3 diag = diagonal();
    return diag.x * diag.y * diag.z;
}

int Bounds3::max_extent() const
{
    glm::vec3 d = diagonal();
    if (d.x > d.y&& d.x > d.z)
        return 0;
    else if (d.y > d.z)
        return 1;
    else
        return 2;
}

glm::vec3 Bounds3::offset(const glm::vec3& p) const
{
    glm::vec3 o = p - m_min_p;
    if (m_max_p.x > m_min_p.x) o.x /= m_max_p.x - m_min_p.x;
    if (m_max_p.y > m_min_p.y) o.y /= m_max_p.y - m_min_p.y;
    if (m_max_p.z > m_min_p.z) o.z /= m_max_p.z - m_min_p.z;
    return o;
}

void Bounds3::add(const Bounds3& bounds)
{
    m_min_p = glm::min(m_min_p, bounds.m_min_p);
    m_max_p = glm::max(m_max_p, bounds.m_max_p);
}

void Bounds3::add(const glm::vec3& point)
{
    m_min_p = glm::min(m_min_p, point);
    m_max_p = glm::max(m_max_p, point);
}

Bounds3 Bounds3::add(const Bounds3& b1, const Bounds3& b2)
{
    glm::vec3 min = glm::min(b1.m_min_p, b2.m_min_p);
    glm::vec3 max = glm::max(b1.m_max_p, b2.m_max_p);

    return Bounds3(min, max);
}

} // end of namespace neptun
