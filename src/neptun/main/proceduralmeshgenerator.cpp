
#include <limits>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/noise.hpp>
#include <glm/gtc/random.hpp>

#include "mesh.h"
#include "proceduralmeshgenerator.h"

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

glm::vec3 noise2d(const float x, const float y, const float shift_x = 0, const float shift_y = 0)
{
    float a = 6;
    float f = 1/8.0f;
    
    float da = 0.8;
    const float df = 4;
    const int num_octaves = 10;

    float z = 0;
    
    for (int i = 0; i < num_octaves; i++)
    {
        z += glm::perlin(glm::vec2(x * f + shift_x, y * f + shift_y)) * a;
        a *= da;
        da *= 0.4;
        f *= df;
    }

    return glm::vec3( x, (z + 0.5) / 4, y);
}

glm::vec3 noise2d(const glm::vec2& v, const glm::vec2& shift = glm::vec2(0.f))
{
    return noise2d(v.x, v.y, shift.x, shift.y);
}

Mesh* ProceduralMeshGenerator::create_terrain(const glm::vec2& terrain_size)
{
    const glm::vec2 shift = glm::linearRand(glm::vec2(-100, -100), glm::vec2(100, 100));

    const glm::ivec2 step_count(100, 100);
    assert(step_count.x > 0 && step_count.y > 0);
    const glm::vec2 step_size = terrain_size / glm::vec2(step_count);
    
    Mesh* mesh = new Mesh;
    mesh->m_vertices.reserve(step_count.x * step_count.y * 6);
    mesh->m_normals.reserve(step_count.x * step_count.y * 6);

    int index = 0;
    for (int i = 1; i < step_count.x; i++)
    {
        for (int j = 1; j < step_count.y; j++)
        {
            const glm::vec2 bounds_coords[2] = {step_size * glm::vec2(i - 1, j - 1) - (terrain_size / 2.0f),
                                                step_size * glm::vec2(i, j) - (terrain_size / 2.0f)};

            const glm::vec2 coords[4] = {
                glm::vec2(bounds_coords[0].x, bounds_coords[0].y),
                glm::vec2(bounds_coords[0].x, bounds_coords[1].y),
                glm::vec2(bounds_coords[1].x, bounds_coords[1].y),
                glm::vec2(bounds_coords[1].x, bounds_coords[0].y)};

            // Positions of the vertices of the current quad
            const glm::vec3 v[4] = {noise2d(coords[0], shift),
                                    noise2d(coords[1], shift),
                                    noise2d(coords[2], shift),
                                    noise2d(coords[3], shift)};

            // Find normals of the vertices
            const glm::vec2 cx = glm::vec2(step_size.x / 2.0f, 0);
            const glm::vec2 cy = glm::vec2(0, step_size.y / 2.0f);

            const glm::vec3 dx[4] = {
                (noise2d(coords[0] + cx, shift) - noise2d(coords[0] - cx, shift)) / 2.0f,
                (noise2d(coords[1] + cx, shift) - noise2d(coords[1] - cx, shift)) / 2.0f,
                (noise2d(coords[2] + cx, shift) - noise2d(coords[2] - cx, shift)) / 2.0f,
                (noise2d(coords[3] + cx, shift) - noise2d(coords[3] - cx, shift)) / 2.0f};

            const glm::vec3 dy[4] = {
                (noise2d(coords[0] + cy, shift) - noise2d(coords[0] - cy, shift)) / 2.0f,
                (noise2d(coords[1] + cy, shift) - noise2d(coords[1] - cy, shift)) / 2.0f,
                (noise2d(coords[2] + cy, shift) - noise2d(coords[2] - cy, shift)) / 2.0f,
                (noise2d(coords[3] + cy, shift) - noise2d(coords[3] - cy, shift)) / 2.0f};

            const glm::vec3 n[4] = {
                -glm::normalize(glm::cross(dx[0], dy[0])),
                -glm::normalize(glm::cross(dx[1], dy[1])),
                -glm::normalize(glm::cross(dx[2], dy[2])),
                -glm::normalize(glm::cross(dx[3], dy[3])),
            };

            mesh->m_vertices.push_back(v[0]);
            mesh->m_vertices.push_back(v[1]);
            mesh->m_vertices.push_back(v[2]);

            mesh->m_normals.push_back(n[0]);
            mesh->m_normals.push_back(n[1]);
            mesh->m_normals.push_back(n[2]);

            mesh->m_vertices.push_back(v[0]);
            mesh->m_vertices.push_back(v[2]);
            mesh->m_vertices.push_back(v[3]);

            mesh->m_normals.push_back(n[0]);
            mesh->m_normals.push_back(n[2]);
            mesh->m_normals.push_back(n[3]);
        }
    }
    
    mesh->m_vertex_count = mesh->m_vertices.size();
    return mesh;
}

Mesh* ProceduralMeshGenerator::create_torus(const glm::ivec2& step_size)
{
    Mesh* mesh = new Mesh;
    constexpr float r_out = 2; // Radius of big circle
    constexpr float r_in = 1; // Radius of inner circle

    auto find_coord = [&r_out, &r_in]( const glm::vec2& coord) 
    {
        const glm::vec2 sin_coord = glm::sin(coord);
        const glm::vec2 cos_coord = glm::cos(coord);
        const float t = r_out + (r_in * cos_coord.x);

        const glm::vec3 p(t * cos_coord.y, r_in * sin_coord.x, t * sin_coord.y); // Coord

        // Find normal 
        // Tangent wrt big circle
        const glm::vec3 big_tan(-sin_coord.y, 0, cos_coord.y);

        // Tangent wrt small circle
        const glm::vec3 small_tan(-sin_coord.x * cos_coord.y, cos_coord.x, -sin_coord.x * sin_coord.y);

        // Normal 
        const glm::vec3 n = glm::cross(big_tan, small_tan);
        constexpr float epsilon = std::numeric_limits<float>::epsilon();
        return std::make_pair(p + epsilon, glm::normalize(glm::vec3(-n)));
    };

    constexpr float TWO_PI = glm::two_pi<float>();
    const glm::vec2 step_angle = glm::vec2(TWO_PI) / glm::vec2(step_size);
    for (int si = 0; si < step_size.x; si++)
    {
        for (int sj = 0; sj < step_size.y; sj++)
        {
            const float i1 = (si % step_size.x) * step_angle.x;
            const float j1 = (sj % step_size.y) * step_angle.y;

            const float i2 = ((si + 1) % step_size.x) * step_angle.x;
            const float j2 = ((sj + 1) % step_size.y) * step_angle.y;

            const auto c1 = find_coord(glm::vec2(i1, j1));
            const auto c2 = find_coord(glm::vec2(i1, j2));
            const auto c3 = find_coord(glm::vec2(i2, j1));
            const auto c4 = find_coord(glm::vec2(i2, j2));
            
            // First triangle
            mesh->m_vertices.push_back(c1.first);
            mesh->m_vertices.push_back(c3.first);
            mesh->m_vertices.push_back(c2.first);

            mesh->m_normals.push_back(c1.second);
            mesh->m_normals.push_back(c3.second);
            mesh->m_normals.push_back(c2.second);

            // Second triangle
            mesh->m_vertices.push_back(c2.first);
            mesh->m_vertices.push_back(c3.first);
            mesh->m_vertices.push_back(c4.first);

            mesh->m_normals.push_back(c2.second);
            mesh->m_normals.push_back(c3.second);
            mesh->m_normals.push_back(c4.second);
        }
    }

    mesh->m_vertex_count = mesh->m_vertices.size();
    return mesh;
}


