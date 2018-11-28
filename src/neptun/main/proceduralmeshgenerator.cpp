
#include <glm/gtc/noise.hpp>

#include "proceduralmeshgenerator.h"
#include "mesh.h"

Mesh* ProceduralMeshGenerator::create_cube(const glm::vec3& size, const glm::ivec3& step_count)
{
    Mesh* mesh = new Mesh();

    const glm::vec3 min = -size * 0.5f;
    const glm::vec3 max = +size * 0.5f;
    const glm::vec3 step_size = size / (glm::vec3)step_count;

    for (size_t axis = 0; axis < 3; ++axis)
    {
        for (size_t i = 0; i < step_count[(axis + 1) % 3]; ++i)
        {
            for (size_t j = 0; j < step_count[(axis + 2) % 3]; ++j)
            {
                glm::vec3 step_first = step_size;
                step_first[axis] = 0.0f;
                step_first[(axis + 2) % 3] = 0.0f;

                glm::vec3 step_second = step_size;
                step_second[axis] = 0.0f;
                step_second[(axis + 1) % 3] = 0.0f;

                glm::vec3 a = min + (float)i * step_first + (float)j * step_second;
                glm::vec3 b = a + step_first;
                glm::vec3 c = a + step_first + step_second;
                glm::vec3 d = a + step_second;

                mesh->m_vertices.push_back(a);
                mesh->m_vertices.push_back(c);
                mesh->m_vertices.push_back(b);

                mesh->m_vertices.push_back(a);
                mesh->m_vertices.push_back(d);
                mesh->m_vertices.push_back(c);

                glm::vec3 normal;
                normal[axis] = -1.0f;

                mesh->m_normals.push_back(normal);
                mesh->m_normals.push_back(normal);
                mesh->m_normals.push_back(normal);

                mesh->m_normals.push_back(normal);
                mesh->m_normals.push_back(normal);
                mesh->m_normals.push_back(normal);

                a[axis] += size[axis];
                b[axis] += size[axis];
                c[axis] += size[axis];
                d[axis] += size[axis];

                normal[axis] = 1.0f;

                mesh->m_vertices.push_back(a);
                mesh->m_vertices.push_back(b);
                mesh->m_vertices.push_back(c);

                mesh->m_vertices.push_back(a);
                mesh->m_vertices.push_back(c);
                mesh->m_vertices.push_back(d);

                mesh->m_normals.push_back(normal);
                mesh->m_normals.push_back(normal);
                mesh->m_normals.push_back(normal);

                mesh->m_normals.push_back(normal);
                mesh->m_normals.push_back(normal);
                mesh->m_normals.push_back(normal);
            }
        }
    }

    mesh->m_vertex_count = mesh->m_vertices.size();
    mesh->m_face_count = mesh->m_vertex_count / 3;

    return mesh;
}

void subdivide_triangle(Mesh* mesh, const int n, glm::vec3 a, glm::vec3 b, glm::vec3 c)
{
    if (n == 0)
    {
        mesh->m_vertices.push_back(a);
        mesh->m_normals.push_back(a);

        mesh->m_vertices.push_back(b);
        mesh->m_normals.push_back(b);

        mesh->m_vertices.push_back(c);
        mesh->m_normals.push_back(c);
    }
    else
    {
        glm::vec3 ab = glm::normalize((a + b) * 0.5f);
        glm::vec3 bc = glm::normalize((b + c) * 0.5f);
        glm::vec3 ca = glm::normalize((c + a) * 0.5f);

        subdivide_triangle(mesh, n - 1, a, ab, ca);
        subdivide_triangle(mesh, n - 1, b, bc, ab);
        subdivide_triangle(mesh, n - 1, c, ca, bc);
        subdivide_triangle(mesh, n - 1, ab, bc, ca);
    }
}

Mesh* ProceduralMeshGenerator::create_icosphere(const float radius, const int n)
{
    // Creating an icosphere mesh in code
    // http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
    // Accessed on 20.03.2018

    Mesh* mesh = new Mesh();

    std::vector<glm::vec3> vertices;

    const float t = (1.0f + glm::sqrt(5.0f)) / 2.0f;

    vertices.push_back(glm::normalize(glm::vec3(-1, +t, 0)));
    vertices.push_back(glm::normalize(glm::vec3(+1, +t, 0)));
    vertices.push_back(glm::normalize(glm::vec3(-1, -t, 0)));
    vertices.push_back(glm::normalize(glm::vec3(+1, -t, 0)));

    vertices.push_back(glm::normalize(glm::vec3(0, -1, +t)));
    vertices.push_back(glm::normalize(glm::vec3(0, +1, +t)));
    vertices.push_back(glm::normalize(glm::vec3(0, -1, -t)));
    vertices.push_back(glm::normalize(glm::vec3(0, +1, -t)));

    vertices.push_back(glm::normalize(glm::vec3(+t, 0, -1)));
    vertices.push_back(glm::normalize(glm::vec3(+t, 0, +1)));
    vertices.push_back(glm::normalize(glm::vec3(-t, 0, -1)));
    vertices.push_back(glm::normalize(glm::vec3(-t, 0, +1)));

    subdivide_triangle(mesh, n, vertices[ 0], vertices[11], vertices[ 5]);
    subdivide_triangle(mesh, n, vertices[ 0], vertices[ 5], vertices[ 1]);
    subdivide_triangle(mesh, n, vertices[ 0], vertices[ 1], vertices[ 7]);
    subdivide_triangle(mesh, n, vertices[ 0], vertices[ 7], vertices[10]);
    subdivide_triangle(mesh, n, vertices[ 0], vertices[10], vertices[11]);

    subdivide_triangle(mesh, n, vertices[ 1], vertices[ 5], vertices[ 9]);
    subdivide_triangle(mesh, n, vertices[ 5], vertices[11], vertices[ 4]);
    subdivide_triangle(mesh, n, vertices[11], vertices[10], vertices[ 2]);
    subdivide_triangle(mesh, n, vertices[10], vertices[ 7], vertices[ 6]);
    subdivide_triangle(mesh, n, vertices[ 7], vertices[ 1], vertices[ 8]);

    subdivide_triangle(mesh, n, vertices[ 3], vertices[ 9], vertices[ 4]);
    subdivide_triangle(mesh, n, vertices[ 3], vertices[ 4], vertices[ 2]);
    subdivide_triangle(mesh, n, vertices[ 3], vertices[ 2], vertices[ 6]);
    subdivide_triangle(mesh, n, vertices[ 3], vertices[ 6], vertices[ 8]);
    subdivide_triangle(mesh, n, vertices[ 3], vertices[ 8], vertices[ 9]);

    subdivide_triangle(mesh, n, vertices[ 4], vertices[ 9], vertices[ 5]);
    subdivide_triangle(mesh, n, vertices[ 2], vertices[ 4], vertices[11]);
    subdivide_triangle(mesh, n, vertices[ 6], vertices[ 2], vertices[10]);
    subdivide_triangle(mesh, n, vertices[ 8], vertices[ 6], vertices[ 7]);
    subdivide_triangle(mesh, n, vertices[ 9], vertices[ 8], vertices[ 1]);

    mesh->m_vertex_count = mesh->m_vertices.size();
    mesh->m_face_count = mesh->m_vertex_count / 3;

    return mesh;
}

glm::vec3 perlin2d(const float x, const float y, const float multiplier = 1)
{
    const float z = glm::perlin(glm::vec2(x * multiplier, y * multiplier));
    return glm::vec3( x - 0.5, (z + 0.5) / 2, y - 0.5 );
}

Mesh* ProceduralMeshGenerator::create_terrain()
{
    const float perlin_multiplier = 3;
 
    const glm::ivec2 step_count(30, 30);
    assert(step_count.x > 0 && step_count.y > 0);
    const glm::vec2 step_size = 1.0f / glm::vec2(step_count);
    
    Mesh* mesh = new Mesh;
    int index = 0;
    for (int i = 1; i < step_count.x; i++)
    {
        for (int j = 1; j < step_count.y; j++)
        {
            const glm::vec2 bounds_coords[2] = { step_size * glm::vec2(i - 1, j - 1),
                                                 step_size * glm::vec2(i, j)};

            const glm::vec3 v[4] = { perlin2d(bounds_coords[0].x, bounds_coords[0].y, perlin_multiplier),
                                     perlin2d(bounds_coords[0].x, bounds_coords[1].y, perlin_multiplier),
                                     perlin2d(bounds_coords[1].x, bounds_coords[1].y, perlin_multiplier),
                                     perlin2d(bounds_coords[1].x, bounds_coords[0].y, perlin_multiplier)
                                     
            };
 
            mesh->m_vertices.push_back(v[0]);
            mesh->m_vertices.push_back(v[1]);
            mesh->m_vertices.push_back(v[2]);

            glm::vec3 normal = glm::normalize(glm::cross(v[1] - v[0], v[2] - v[0]));
            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
            
            
            mesh->m_vertices.push_back(v[0]);
            mesh->m_vertices.push_back(v[2]);
            mesh->m_vertices.push_back(v[3]);

            normal = glm::normalize(glm::cross(v[2] - v[0], v[3] - v[0]));
            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
        }
    }
    
    mesh->m_vertex_count = mesh->m_vertices.size();
    return mesh;
}
