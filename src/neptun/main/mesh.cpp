#include "mesh.h"

#include <vector>

Mesh::Mesh()
{
    m_face_count = 0;
}

//Mesh::Mesh(const Mesh & mesh)
//{
//}

Mesh::~Mesh()
{
}

void Mesh::center_pivot()
{
    glm::vec3 center(0, 0, 0);

    float r = 1.0f / m_vertex_count;

    for (int i = 0; i < m_vertex_count; i++)
        center += m_vertices[i] * r;

    for (int i = 0; i < m_vertex_count; i++)
        m_vertices[i] -= center;

    m_bounds_min -= center;
    m_bounds_max -= center;
}
