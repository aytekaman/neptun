#include "sceneObject.h"

#include "material.h"

#include <glm\gtc\random.hpp>
#include <glm\gtx\color_space.hpp>

SceneObject::SceneObject(std::string name_) : name(name_), scale(1), mesh(nullptr), light(nullptr)
{
	float hue = glm::linearRand(0.0f, 360.0f);

	color = glm::vec4(glm::rgbColor(glm::vec3(hue, 1, 1)), 1);

	material = new Material();
}

SceneObject::~SceneObject()
{
    delete light;
    delete material;
    delete mesh;
}

void SceneObject::LookAt(glm::vec3 target, glm::vec3 up)
{
	
}