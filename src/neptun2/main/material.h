#pragma once

#include <glm/glm.hpp>

namespace neptun
{

class Texture;

class Material
{
public:
    glm::vec3 m_diffuse = glm::vec3(1.f);
    Texture* m_texture = nullptr;
};

class Light
{
public:
    glm::vec3 m_color = glm::vec3(1.f);
    float m_intensity = 1.f;
};

}// end of namespace neptun
