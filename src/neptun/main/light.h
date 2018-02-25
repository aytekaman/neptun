#pragma once

#include <glm\glm.hpp>

class Light
{
public:
    Light();
    ~Light();

    bool isDirty = true;

    glm::vec3 color;

    float intensity;

    int point_index;
};