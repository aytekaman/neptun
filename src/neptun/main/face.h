
#pragma once

#include <glm\glm.hpp>

#include "material.h"

struct Face
{
    glm::vec3 vertices[3];
    glm::vec3 normals[3];
    glm::vec2 uvs[3];
    Material *material = nullptr;
};