#include "texture.h"

glm::vec3 Texture::GetPixelBilinear(float u, float v)
{
	//v = 1 - v;
	u *= w;
	v *= h;

	glm::vec3 pixel;

	for (int i = 0; i < 3; i++)
		pixel[i] = pixels[int((int)v * w + (int)u) * 3 + i] / 255.0f;

	return pixel;
}
