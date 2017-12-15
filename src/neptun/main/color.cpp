#include "color.h"

glm::vec4 Color::red(1, 0, 0, 1);
glm::vec4 Color::green(0, 1, 0, 1);
glm::vec4 Color::blue(0, 0, 1, 1);
glm::vec4 Color::yellow(1, 0.92, 0.016, 1);
glm::vec4 Color::orange(1, 0.6, 0, 1);

glm::vec4 Color::turquoise	(1 / 255.0f, 185 / 255.0f, 187 / 255.0f, 1);
glm::vec4 Color::orchid		(97 / 255.0f, 125 / 255.0f, 198 / 255.0f, 1);
glm::vec4 Color::yolk		(247 / 255.0f, 210 / 255.0f, 43 / 255.0f, 1);
glm::vec4 Color::melon		(242 / 255.0f, 94 / 255.0f, 66 / 255.0f, 1);
glm::vec4 Color::berry		(226 / 255.0f, 108 / 255.0f, 122 / 255.0f, 1);

glm::vec4 Color::lerp(const glm::vec4& a, const glm::vec4& b, const glm::vec4& c, float t)
{
    if (t < 0.5f)
    {
        t *= 2.0f;

        return glm::mix(a, b, t);
    }
    else
    {
        t = glm::min(t, 1.0f);
        t = 2.0f * (t - 0.5f);

        return glm::mix(b, c, t);
    }
}
