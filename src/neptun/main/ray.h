
#pragma once

#include "glm/glm.hpp"

struct Ray
{
    glm::vec3 origin;
    glm::vec3 dir;

    union
    {
        mutable float tMax;
        int tet_idx;
    };

    Ray(glm::vec3 origin_ = glm::vec3(0, 0, 0), glm::vec3 dir_ = glm::vec3(0, 0, 0))
    {
        origin = origin_;
        dir = dir_;
    }
};
