#pragma once

#include <fstream>
#include <vector>
#include <unordered_map>

#include "glm/glm.hpp"

class Mesh;
class Scene;
class Texture;
struct GLFWwindow;

#include "gl3w/include/GL/gl3w.h"

struct RenderData
{
	GLuint vertexArrayID;
	GLuint vertexBufferIDs[4];
};

enum class RenderingMode
{
    Solid,
    WireFrame,
    SolidWireframe
};

class Graphics
{
public:
	Graphics();

	void Init();
	void Render(Scene *scene, bool show_tetrahedrons, RenderingMode rendering_mode);

	void DrawLine(glm::vec3 start, glm::vec3 end, glm::vec4 color = glm::vec4(1.0, 1.0, 1.0, 1.0));
	void DrawTri(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec4 color = glm::vec4(1.0, 1.0, 1.0, 1.0));
	void DrawTet(glm::vec3 p[4], glm::vec4 color = glm::vec4(1.0, 1.0, 1.0, 1.0));

    void UpdateTetMeshData(const Scene* scene);

	void DrawCube(glm::vec3 min = -glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3 max = glm::vec3(0.5f, 0.5f, 0.5f), glm::vec4 color = glm::vec4(1.0, 1.0, 1.0, 1.0));

	void CreateRenderData(Mesh *mesh);
	void SendMeshData(Mesh *mesh);

	void CreateTextureData(Texture *texture);
	void SendTextureData(Texture *texture);

	GLuint LoadShaders(std::string vertexFilePath, std::string fragmentFilePath);

	GLFWwindow *window;
	glm::mat4 p, v;


	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> normals;

	// Line drawing
	std::vector<glm::vec3> lineVertices;
	std::vector<glm::vec4> lineColors;

    struct TetData
    {
        int tet_idx;
        float z;

        bool operator<(const TetData& other) const
        {
            return z < other.z;
        }
    };
	// Tri drawing
	std::vector<glm::vec3> triVertices;
	std::vector<glm::vec3> triNormals;
	std::vector<glm::vec4> triColors;
    std::vector<TetData> tet_datas;
	
	std::unordered_map<Mesh*, RenderData> renderDatas;
	std::unordered_map<Texture*, GLuint> texture_handles;

	GLuint programID;

	GLuint linesProgramID;
	GLuint linesVertexArrayID;
	GLuint linesVertexBufferIDs[2];

	GLuint trisProgramID;
	GLuint trisVertexArrayID;
	GLuint trisVertexBufferIDs[3];

	GLuint textureID;
};