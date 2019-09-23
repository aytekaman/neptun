#pragma once

#include <glm/glm.hpp>

#include "neptun/main/ray.h"

class Transform
{
public:
    // Constructors
    Transform();
    explicit Transform(const glm::mat4& t, const glm::mat4& inv_t);

    // Operations
    Transform inverse() const;
    Transform operator*(const Transform &other) const;

    // Transformations
    glm::vec4 transform(const glm::vec4& p)     const;
    glm::vec4 inv_transform(const glm::vec4& p) const;

    glm::vec3 transform_point(const glm::vec3& p)  const;
    glm::vec3 inv_transform_point(const glm::vec3& p)  const;

    glm::vec3 transform_vector(const glm::vec3& v) const;
    glm::vec3 inv_transform_vector(const glm::vec3& v) const;

    glm::vec3 transform_normal(const glm::vec3& n) const;
    glm::vec3 inv_transform_normal(const glm::vec3& n) const;

    Ray transform_ray(const Ray& ray) const;
    Ray inv_transform_ray(const Ray& ray) const;

    // Static members
    static Transform translate(const glm::vec3& t);
    static Transform scale(const glm::vec3& s);
    static Transform rotate(const glm::vec3& r);
private:
    glm::mat4 m_tr;
    glm::mat4 m_inv_tr;
};
