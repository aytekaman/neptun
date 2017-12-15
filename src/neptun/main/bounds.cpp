#include "bounds.h"

glm::vec3 Bounds3::diagonal() const
{
    return max - min;
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

void Bounds3::add(const Bounds3& bounds)
{
    min = glm::min(min, bounds.min);
    max = glm::max(max, bounds.max);
}

void Bounds3::add(const glm::vec3 & point)
{
    min = glm::min(min, point);
    max = glm::max(max, point);
}

Bounds3 Bounds3::add(const Bounds3 & b1, const Bounds3 & b2)
{
    glm::vec3 min = glm::min(b1.min, b2.min);
    glm::vec3 max = glm::max(b1.max, b2.max);

    return Bounds3(min, max);
}
