#pragma once

#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class Light;
class Material;
class Mesh;

class SceneObject
{
public:

    SceneObject(std::string name);
    ~SceneObject();

    void randomize_color();

    void look_at(glm::vec3 target, glm::vec3 up = glm::vec3(0, 1, 0));

    Light *light;
    Material *material;
    Mesh *mesh;
    

    bool isVisible = true;
    bool hide_in_editor = false;
    bool pickable = true;

    glm::vec4 color;

    std::string name;

    glm::vec3 pos;
    glm::vec3 rot;
    glm::vec3 scale;
};
