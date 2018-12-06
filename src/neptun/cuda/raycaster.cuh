#include "cuda_runtime.h"

#include "device_launch_parameters.h"

#include "neptun/main/tet_mesh.h"
#include "neptun/main/ray_tracer.h"
#include "neptun/main/scene.h"

#define GLM_FORCE_CUDA
#include "glm/glm.hpp"
#include "glm/gtc/constants.hpp"

#include <iostream>


__device__
inline void swapvec2(glm::vec2 &a, glm::vec2 &b)
{
	glm::vec2 t = a;
	a = b;
	b = t;
}

__device__
inline void swap(unsigned int &a, unsigned int &b)
{
	unsigned int t = a;
	a = b;
	b = t;
}

__device__
inline float crossv(const glm::vec2& a, const glm::vec2& b) { return a.x * b.y - a.y * b.x; }

__device__
inline float crossv(const glm::vec3& a, const glm::vec3& b) { return a.x * b.y - a.y * b.x; }