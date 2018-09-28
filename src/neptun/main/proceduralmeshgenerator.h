
#pragma once

#include "glm/glm.hpp"

class Mesh;

class ProceduralMeshGenerator
{
public:
  static Mesh* create_cube(const glm::vec3 &size = glm::vec3(1.0f, 1.0f, 1.0f),
                           const glm::ivec3 &step_count = glm::vec3(4, 4, 4));

  static Mesh* create_icosphere(const float radius = 1.0f, const int n = 2);
};
