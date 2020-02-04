#pragma once

#include "glm/glm.hpp"

class Texture;

class Material
{
public:
    glm::vec3 diffuse = glm::vec3(1,1,1);
    glm::vec3 le = glm::vec3(0.f);

    Texture *texture = nullptr;
};
