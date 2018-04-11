#pragma once

#include "glm/glm.hpp"

struct Color
{
    static glm::vec4 jet(const float t);
    static glm::vec4 lerp(const glm::vec4& a, const glm::vec4& b, const glm::vec4& c, float t);

    static glm::vec4 red;
    static glm::vec4 green;
    static glm::vec4 blue;
    static glm::vec4 yellow;
    static glm::vec4 orange;

    static glm::vec4 turquoise;
    static glm::vec4 orchid;
    static glm::vec4 yolk;
    static glm::vec4 melon;
    static glm::vec4 berry;
};
