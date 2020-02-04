#pragma once

#include <glm/glm.hpp>

// https://github.com/tunabrain/tungsten/blob/master/src/core/math/TangentFrame.hpp
// Similar to basis?
class TangentFrame
{
public:

    explicit TangentFrame(const glm::vec3& normal) : m_normal(normal)
    {
        float sign = copysignf(1.0f, normal.z);
        const float a = -1.0f / (sign + normal.z);
        const float b = normal.x * normal.y * a;
        m_tangent = glm::vec3(1.0f + sign * normal.x * normal.x * a, sign * b, -sign * normal.x);
        m_bitangent = glm::vec3(b, sign + normal.y * normal.y * a, -normal.y);
    }

    glm::vec3 to_local(const glm::vec3& p) const
    {
        return glm::vec3(
            glm::dot(m_tangent, p),
            glm::dot(m_bitangent, p),
            glm::dot(m_normal, p)
        );
    }

    glm::vec3 to_global(const glm::vec3& p) const
    {
        return m_tangent * p.x + m_bitangent * p.y + m_normal * p.z;
    }

    glm::vec2 to_angles(const glm::vec3 w) const
    {
        const float r = glm::length(w); // Not necessary

        const float theta = std::acos(w.z / r);
        const float phi = std::atan2(w.y, w.x);

        return glm::vec2(theta, phi);
    }

    static glm::vec2 to_ang(const glm::vec3 w)
    {
        const float r = glm::length(w); // Not necessary

        const float theta = std::acos(glm::clamp(w.z / r, -1.f, 1.f));
        const float phi = [&w]() {
            const auto a = std::atan2(w.y, w.x);
            return a < 0 ? a + glm::two_pi<float>() : a;
        }();

        return glm::vec2(theta, phi);
    }

private:
    glm::vec3 m_normal;
    glm::vec3 m_tangent;
    glm::vec3 m_bitangent;
};