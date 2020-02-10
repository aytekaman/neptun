#include "scene.h"

#include <neptun2/main/mesh.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>


namespace neptun
{

// Camera
void Camera::update()
{
    glm::vec3 dir = glm::vec3(glm::cos(orbit.y), 0, glm::sin(orbit.y));
    dir = dir * glm::cos(orbit.x);
    dir.y = glm::sin(orbit.x);

    m_cam_pos = target + dir * dist;

    const glm::vec3 forward = glm::normalize(target - m_cam_pos);
    const glm::vec3 right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), forward));
    const glm::vec3 down = glm::cross(forward, right);

    const float aspect = (float)resolution.x / resolution.y;
    const float scale_y = glm::tan(glm::pi<float>() / 8);

    m_top_left = m_cam_pos + forward - down * scale_y - right * scale_y * aspect;
    m_right_step = (right * scale_y * 2.0f * aspect) / (float)resolution.x;
    m_down_step = (down * scale_y * 2.0f) / (float)resolution.y;
}

// Generates neptun::Ray from camera
Ray Camera::generate_ray(const glm::vec2& pixel) const
{
    const glm::vec3 origin = m_cam_pos;
    const glm::vec3 dir = glm::normalize(m_top_left + m_right_step * pixel.x + m_down_step * pixel.y - origin);
    Ray ray;

    ray.org_x = origin.x;
    ray.org_y = origin.y;
    ray.org_z = origin.z;

    ray.dir_x = dir.x;
    ray.dir_y = dir.y;
    ray.dir_z = dir.z;

    ray.max_t = 1000000000.f;

    return ray;
}

// Scene
    
void Scene::build_accelerator(Accelerator* accelerator)
{
    // Create triangle array
    std::vector<Triangle> triangles;

    for (const auto& obj : scene_objects)
    {
        obj->clean();

        if (obj->has_mesh())
        {
            const Mesh& mesh = *obj->m_mesh;

            const Transform& tr = obj->m_transform;

            for (size_t i = 0; i < mesh.m_vertices.size(); i += 3)
            {
                Triangle triangle;
                triangle.geometry_id = size_t(obj.get());
                triangle.primitive_id = i / 3;

                triangle.v[0] = tr.transform_point(mesh.m_vertices[i]);
                triangle.v[1] = tr.transform_point(mesh.m_vertices[i + 1]);
                triangle.v[2] = tr.transform_point(mesh.m_vertices[i + 2]);

                triangles.push_back(triangle);
            }
        }
    }
}

} // end of namespace neptun