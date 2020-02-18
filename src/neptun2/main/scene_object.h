#pragma once

#include <neptun2/math/transform.h>

#include <glm/glm.hpp>

#include <string>
#include <memory>
#include <iosfwd>

namespace neptun
{

class Light;
class Material;
class Mesh;

class SceneObject
{
public:
    explicit SceneObject(std::string name);
    
    // Dirty functions
    bool is_dirty() const;
    void clean();
    void mark_dirty();

    bool is_light() const;
    bool has_mesh() const;

    friend std::ostream& operator<<(std::ostream& os, const SceneObject& obj);

    std::string m_name;
    glm::vec4 m_color;

    // Point light
    std::unique_ptr<Light>      m_light = nullptr;

    // Primitive
    std::unique_ptr<Material>   m_material = nullptr;
    Mesh* m_mesh = nullptr;

    glm::vec3 m_pos = glm::vec3(0.f);
    glm::vec3 m_rot = glm::vec3(0.f);
    glm::vec3 m_scale = glm::vec3(1.f);

    bool m_hide_in_editor = false;
    bool m_is_visible = true;
    bool m_is_pickable = true;

    bool m_dirty = true;
    Transform m_transform;
};

} // end of namespace neptun
