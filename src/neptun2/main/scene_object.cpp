#include "scene_object.h"

#include <neptun2/main/material.h>
#include <neptun2/main/mesh.h>


#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/color_space.hpp>

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

} // end of namespace neptun