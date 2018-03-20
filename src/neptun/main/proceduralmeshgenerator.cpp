
#include "proceduralmeshgenerator.h"

#include "mesh.h"

Mesh* ProceduralMeshGenerator::create_cube(const glm::vec3& size)
{
    Mesh* mesh = new Mesh();

    const glm::vec3 min = -size * 0.5f;
    const glm::vec3 max = +size * 0.5f;
    const glm::ivec3 step_count(4, 4, 4);
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

                mesh->vertices.push_back(a);
                mesh->vertices.push_back(c);
                mesh->vertices.push_back(b);

                mesh->vertices.push_back(a);
                mesh->vertices.push_back(d);
                mesh->vertices.push_back(c);

                glm::vec3 normal;
                normal[axis] = -1.0f;

                mesh->normals.push_back(normal);
                mesh->normals.push_back(normal);
                mesh->normals.push_back(normal);

                mesh->normals.push_back(normal);
                mesh->normals.push_back(normal);
                mesh->normals.push_back(normal);

                a[axis] += size[axis];
                b[axis] += size[axis];
                c[axis] += size[axis];
                d[axis] += size[axis];

                normal[axis] = 1.0f;

                mesh->vertices.push_back(a);
                mesh->vertices.push_back(b);
                mesh->vertices.push_back(c);

                mesh->vertices.push_back(a);
                mesh->vertices.push_back(c);
                mesh->vertices.push_back(d);

                mesh->normals.push_back(normal);
                mesh->normals.push_back(normal);
                mesh->normals.push_back(normal);

                mesh->normals.push_back(normal);
                mesh->normals.push_back(normal);
                mesh->normals.push_back(normal);
            }
        }
    }

    mesh->vertexCount = mesh->vertices.size();

    return mesh;
}

//Mesh* ProceduralMeshGenerator::create_icosphere(const float radius, const int n)
//{
//    Mesh* mesh = new Mesh();
//
//    return mesh;
//}
