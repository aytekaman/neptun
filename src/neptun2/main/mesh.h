#pragma once

#include <glm/glm.hpp>

#include <vector>
#include <string>

namespace neptun
{

class Mesh
{
public:
    explicit Mesh(std::string file_name);

    size_t face_count() const;
    size_t vertex_count() const;

    // Dirty functions
    bool is_dirty() const;
    void clean();
    void mark_dirty();

    std::string m_file_name;

    std::vector<glm::vec3> m_vertices;
    std::vector<glm::vec3> m_normals;
    std::vector<glm::vec3> m_colors;
    std::vector<glm::vec2> m_uvs;

    bool m_structure_mesh = false;
    bool m_ignore_tetrahedralization = false;

    bool m_dirty = true;
    glm::vec3 m_bounds_max;
    glm::vec3 m_bounds_min;
};

} // end of namespace neptun
