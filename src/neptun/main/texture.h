#pragma once

#include <glm\glm.hpp>

class Texture
{
public:
	glm::vec3 GetPixelBilinear(float u, float v);

	bool is_dirty = true;

	unsigned char *pixels;
    int w, h;
};