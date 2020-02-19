#include "transform.h"

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>

#include <limits>


namespace neptun
{

// Constructors
Transform::Transform() : m_tr(), m_inv_tr()
{
}

Transform::Transform(const glm::mat4& t, const glm::mat4& inv_t)
    : m_tr(t)
    , m_inv_tr(inv_t)
{
}

// Operations
Transform Transform::inverse() const
{
    return Transform(m_inv_tr, m_tr);
}

Transform Transform::operator*(const Transform& other) const
{
    return Transform(m_tr * other.m_tr, other.m_inv_tr * m_inv_tr);
}

// Transformations
glm::vec4 Transform::transform(const glm::vec4& v) const
{
    return m_tr * v;
}

glm::vec4 Transform::inv_transform(const glm::vec4& v) const
{
    return m_inv_tr * v;
}

glm::vec3 Transform::transform_point(const glm::vec3& pt) const
{
    const glm::vec4 v = transform(glm::vec4(pt, 1.f));

    if (v.w == 1)
        return glm::vec3(v);
    else
        return glm::vec3(v) / v.w;
}

glm::vec3 Transform::inv_transform_point(const glm::vec3& pt) const
{
    const glm::vec4 v = inv_transform(glm::vec4(pt, 1.f));

    if (v.w == 1)
        return glm::vec3(v);
    else
        return glm::vec3(v) / v.w;
}

glm::vec3 Transform::transform_vector(const glm::vec3& v) const
{
    return glm::mat3(m_tr) * v;
}

glm::vec3 Transform::inv_transform_vector(const glm::vec3& v) const
{
    return glm::mat3(m_inv_tr) * v;
}

glm::vec3 Transform::transform_normal(const glm::vec3& n) const
{
    return glm::transpose(glm::mat4(m_inv_tr)) * glm::vec4(n, 0.f);
}

glm::vec3 Transform::inv_transform_normal(const glm::vec3& n) const
{
    return glm::transpose(glm::mat4(m_tr)) * glm::vec4(n, 0.f);
}


// Static members
Transform Transform::translate(const glm::vec3& t)
{
    glm::mat4 tr(
        1, 0, 0, t.x,
        0, 1, 0, t.y,
        0, 0, 1, t.z,
        0, 0, 0, 1);

    glm::mat4 inv_tr(
        1, 0, 0, -t.x,
        0, 1, 0, -t.y,
        0, 0, 1, -t.z,
        0, 0, 0, 1);

    return Transform(glm::transpose(tr), glm::transpose(inv_tr));
}

Transform Transform::scale(const glm::vec3& s)
{
    glm::mat4 tr(
        s.x, 0, 0, 0,
        0, s.y, 0, 0,
        0, 0, s.z, 0,
        0, 0, 0, 1);

    glm::mat4 inv_tr(
        1 / s.x, 0, 0, 0,
        0, 1 / s.y, 0, 0,
        0, 0, 1 / s.z, 0,
        0, 0, 0, 1);

    return Transform(glm::transpose(tr), glm::transpose(inv_tr));
}

Transform Transform::rotate(const glm::vec3& r)
{
    glm::mat4 tr = glm::eulerAngleYXZ(glm::radians(r.y), glm::radians(r.x), glm::radians(r.z));
    glm::mat4 inv_tr = glm::transpose(tr);

    return Transform(tr, inv_tr);
}

} // end of namespace neptun
