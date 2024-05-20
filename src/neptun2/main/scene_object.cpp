#include "scene_object.h"

#include <neptun2/main/material.h>
#include <neptun2/main/mesh.h>


#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/color_space.hpp>

#include <iostream>

namespace neptun
{

// Utility functions
glm::vec4 random_color()
{
    float hue = glm::linearRand(0.0f, 360.0f);
    return glm::vec4(glm::rgbColor(glm::vec3(hue, 1.f, 1.f)), 1.f);
}

//SceneObject
SceneObject::SceneObject(std::string name) : m_name(name), m_color(random_color())
{
}


// Dirty functions
bool SceneObject::is_dirty() const
{
    return m_dirty;
}

void SceneObject::clean()
{
    if (is_dirty())
    {
        m_transform =Transform::translate(m_pos) *
            Transform::rotate(glm::radians(m_rot)) *
            Transform::scale(m_scale);
        
        m_dirty = false;
    }
}

void SceneObject::mark_dirty()
{
    m_dirty = true;
}

bool SceneObject::is_light() const
{
    return m_light != nullptr;
}

bool SceneObject::has_mesh() const
{
    return m_mesh != nullptr;
}

std::ostream& operator<<(std::ostream& os, const SceneObject& obj)
{
    os << obj.m_name << " [type=" << (obj.is_light() ? "light" : "mesh")
        << ", pos=(" << obj.m_pos.x << " " << obj.m_pos.y << " " << obj.m_pos.z 
        << "), scale=(" << obj.m_scale.x << " " << obj.m_scale.y << " " << obj.m_scale.z 
        << "), rot=(" << obj.m_rot.x << " " << obj.m_rot.y << " " << obj.m_rot.z << ")]";
    
    if (obj.m_mesh != nullptr)
        os << "\nmesh=" << obj.m_mesh->m_file_name;

    if (obj.m_light)
    {
        os << "\nlight=[intensity" << obj.m_light->m_intensity
            << ", color=("<< obj.m_light->m_color.x << ", " << obj.m_light->m_color.y << ", " << obj.m_light->m_color.z << ")]";
    }

    os << "\n";
    return os;
}

} // end of namespace neptun
