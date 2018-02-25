#pragma once

#include <glm\glm.hpp>

#include <vector>
//#include <string>

class Mesh
{
public:
    Mesh();
    //Mesh(const Mesh& mesh);
    ~Mesh();

    void CenterPivot();

    void SetVertices();
    void SetNormals();
    void SetColors();

    bool isDirty = true;

    int faceCount;
    int vertexCount;

    glm::vec3 min, max;

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec3> colors;
    std::vector<glm::vec2> uvs;

    std::string file_name;
};