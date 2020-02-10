#pragma once

#include <neptun2/accelerators/ray.h>
#include <neptun2/accelerators/accelerator.h>
#include <neptun2/main/scene_object.h>

#include <glm/fwd.hpp>

#include <memory>
#include <vector>

namespace neptun
{

class Camera
{
public:
    glm::vec3 target;
    glm::vec2 orbit;
    float dist;
    glm::vec2 resolution; // Image resolution, not sure it should be in here

    // Updates private camera parameters 
    // Call update after parameter changes
    void update();

    // Generates neptun::Ray from camera
    Ray generate_ray(const glm::vec2& pixel) const;
private:
    glm::vec3 m_top_left;
    glm::vec3 m_right_step, m_down_step;
    glm::vec3 m_cam_pos;
};

class Scene
{
    void build_accelerator(Accelerator* accelerator);

public:
    Camera camera;
    std::vector<std::unique_ptr<SceneObject>> scene_objects;

    std::unique_ptr<Accelerator> accelerator; // TODO: Add support for multiple accelerators in one scene
};

} // end of namespace neptun