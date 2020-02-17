#include "scene.h"

#include <neptun2/main/mesh.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <iostream>

namespace neptun
{

// Camera
void Camera::update()
{
    glm::vec3 dir = glm::vec3(glm::cos(m_orbit.y), 0, glm::sin(m_orbit.y));
    dir = dir * glm::cos(m_orbit.x);
    dir.y = glm::sin(m_orbit.x);

    m_cam_pos = m_target + dir * m_dist;

    const glm::vec3 forward = glm::normalize(m_target - m_cam_pos);
    const glm::vec3 right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), forward));
    const glm::vec3 down = glm::cross(forward, right);

    const float aspect = (float)m_resolution.x / m_resolution.y;
    const float scale_y = glm::tan(glm::pi<float>() / 8);

    m_top_left = m_cam_pos + forward - down * scale_y - right * scale_y * aspect;
    m_right_step = (right * scale_y * 2.0f * aspect) / (float)m_resolution.x;
    m_down_step = (down * scale_y * 2.0f) / (float)m_resolution.y;
}

// Generates neptun::Ray from camera
Ray Camera::generate_ray(const glm::vec2& pixel) const
{
    const glm::vec3 origin = m_cam_pos;
    const glm::vec3 dir = glm::normalize(m_top_left + m_right_step * pixel.x + m_down_step * pixel.y - origin);
    Ray ray;

    ray.org = origin;
    ray.dir = dir;

    ray.max_t = 1000000000.f;
    ray.tet_id = -1;

    return ray;
}

std::ostream& operator<<(std::ostream& os, const Camera& cam)
{
    os << "Camera: \n"
        << "camera_target=" << cam.m_target.x << " " << cam.m_target.y << " " << cam.m_target.z << "\n"
        << "camera_distance=" << cam.m_dist << "\n"
        << "camera_orbit=" << cam.m_orbit.x << " " << cam.m_orbit.y << "\n"
        << "image_resolution=" << cam.m_resolution.x << " " << cam.m_resolution.y << "\n";

    return os;
}

// Scene
    
void Scene::build_accelerator(Accelerator* accelerator)
{
    // Create triangle array
    std::vector<Triangle> triangles;

    size_t obj_index = 0;
    for (const auto& obj : m_scene_objects)
    {
        obj->clean();

        if (obj->has_mesh())
        {
            const Mesh& mesh = *obj->m_mesh;

            const Transform& tr = obj->m_transform;

            for (size_t i = 0; i < mesh.m_vertices.size(); i += 3)
            {
                Triangle triangle;
                triangle.geometry_id = obj_index;
                triangle.primitive_id = i / 3;

                triangle.v[0] = tr.transform_point(mesh.m_vertices[i]);
                triangle.v[1] = tr.transform_point(mesh.m_vertices[i + 1]);
                triangle.v[2] = tr.transform_point(mesh.m_vertices[i + 2]);

                triangles.push_back(triangle);
            }
        }

        obj_index++;
    }

    const bool build_success = accelerator->build(&triangles[0], triangles.size());

    if (build_success)
        m_accelerator.reset(accelerator);
}

Mesh* Scene::get_mesh(const std::string& mesh_file_name)
{
    Mesh* mesh = nullptr;

    for (const auto& m : m_meshes)
        if (m->m_file_name == mesh_file_name)
            mesh = m.get();

    return mesh;
}

SceneObject* Scene::get_scene_object(const std::string& scene_object_name)
{
    SceneObject* obj = nullptr;

    for (const auto& m : m_scene_objects)
        if (m->m_name == scene_object_name)
            obj = m.get();

    return obj;
}

void Scene::clear()
{
    m_meshes.clear();
    m_accelerator.reset();
    m_scene_objects.clear();
    m_camera = Camera();
}

std::ostream& operator<<(std::ostream& os, const Scene& scene)
{
    os << "Scene (" << scene.m_file_name << "):\n"
        << "scene_path=" << scene.m_path << "\n"
        << scene.m_camera << "\n";
    
    os << "Scene Objects:\n";
    for (const auto& obj : scene.m_scene_objects)
        os << *obj << "\n";

    os << "Imported Meshes:\n";
    for (const auto& obj : scene.m_meshes)
        os << obj->m_file_name << " [#f=" << obj->face_count() << "]\n";

    return os;
}

} // end of namespace neptun