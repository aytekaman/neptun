
#pragma once

#include "glm/glm.hpp"

class Ray;
class Material;

struct DiagnosticData
{
    int visited_node_count;
    int total_tet_distance;
    int L1_hit_count;
};

struct IntersectionData
{
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;

    Material* material;

    bool hit;

    int tet_idx;
    int neighbor_tet_idx;
};

class Accelerator
{
public:
    Accelerator();
    ~Accelerator();

    virtual bool intersect(const Ray& ray, IntersectionData& intersection_data) = 0;
    virtual bool intersect(const Ray& ray) = 0;

    //virtual bool intersect_stats(const Ray& ray, IntersectionData& intersection_data, DiagnosticData& diagnostic_data);

    // Total number of bytes required for storing
    // this acceleration structure in memory.
    virtual int get_size_in_bytes() = 0;

private:
    //void init_faces();
};
