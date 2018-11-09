#pragma once

#include <glm/glm.hpp>

#include <vector>
#include <string>

class Mesh
{
public:
    Mesh();
    ~Mesh();

    void center_pivot();
    void calculate_bounds();

    bool m_is_dirty = true;
    bool m_structure_mesh = false; 
    bool m_ignore_tetrahedralization = false;

    int m_face_count;
    int m_vertex_count;

    glm::vec3 m_bounds_max;
    glm::vec3 m_bounds_min;

    std::vector<glm::vec3> m_vertices;
    std::vector<glm::vec3> m_normals;
    std::vector<glm::vec3> m_colors;
    std::vector<glm::vec2> m_uvs;

    std::string m_file_name;
};
