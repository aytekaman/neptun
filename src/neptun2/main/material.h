#pragma once

#include <glm/glm.hpp>

namespace neptun
{

class Texture;

class Material
{
public:
    glm::vec3 diffuse = glm::vec3(1.f);
    Texture* texture = nullptr;
};

class Light
{
public:
    glm::vec3 color = glm::vec3(1.f);
    float intensity = 1.f;
};

}// end of namespace neptun
