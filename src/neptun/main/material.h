#pragma once

#include "glm\glm.hpp"

class Texture;

class Material
{
public:
    glm::vec3 diffuse = glm::vec3(1,1,1);

    Texture *texture = nullptr;
};