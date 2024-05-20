#pragma once

#include <neptun2/main/material.h>
#include <neptun2/main/mesh.h>
#include <neptun2/accelerators/ray.h>
#include <neptun2/accelerators/accelerator.h>
#include <neptun2/main/scene_object.h>
#include <neptun2/integrators/integrator.h>
#include <neptun2/main/image.h>

#include <glm/fwd.hpp>

#include <memory>
#include <vector>
#include <iosfwd>

namespace neptun
{

class Camera
{
public:
    glm::vec3 m_target = glm::vec3(0.f);
    glm::vec2 m_orbit = glm::vec2(0.f);
    float m_dist = 1.f;
    glm::u64vec2 m_resolution = glm::u64vec2(640, 480); // Image resolution, not sure it should be in here

    // Updates private camera parameters 
    // Call update after parameter changes
    void update();

    // Generates neptun::Ray from camera
    Ray generate_ray(const glm::vec2& pixel) const;

    friend std::ostream& operator<<(std::ostream& os, const Camera& cam);

private:
    glm::vec3 m_top_left;
    glm::vec3 m_right_step, m_down_step;
    glm::vec3 m_cam_pos;
};

class Scene
{
public:
    void build_accelerator(Accelerator* accelerator);

    Mesh* get_mesh(const std::string& mesh_file_name);
    SceneObject* get_scene_object(const std::string& scene_object_name);

    void clear();

    friend std::ostream& operator<<(std::ostream& os, const Scene& scene);

    // Fields
    Camera m_camera;
    std::vector<std::unique_ptr<SceneObject>> m_scene_objects;
    std::vector<std::unique_ptr<Mesh>> m_meshes;

    std::string m_path;
    std::string m_file_name;

    std::unique_ptr<Accelerator> m_accelerator; // TODO: Add support for multiple accelerators in one scene
};

} // end of namespace neptun
