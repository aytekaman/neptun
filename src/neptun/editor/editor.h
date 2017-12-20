#pragma once



#include "glm\glm.hpp"

#include <vector>

class Graphics;
class RayTracer;
class Scene;
class SceneObject;
class TetMesh;

struct GLFWwindow;

class Editor
{
public:
	Editor(Scene *scene, Graphics *graphics, RayTracer *ray_tracer);

	void Run();

	void Draw();
	void DrawMainMenuBar();
	void DrawInspector();
	void DrawHierarchy();
	void DrawTetGen();
	void DrawScene();

	void DrawRenderedFrame();
	void DrawConsole();

	void HandleSelectionGizmos();
	void HandleTransformGizmos();

	static void DropCallback(GLFWwindow *window, int count, const char** paths);

	void InitSkin();

	bool show_rendered_frame_window;
	bool show_console_window;
	bool show_grid = true;

	std::vector<SceneObject*> selected_scene_objects;

	SceneObject *selected_scene_object = nullptr;

	GLFWwindow *window;
	Scene *scene;
	//TetMesh *tet_mesh;
	Graphics *graphics;
	RayTracer *ray_tracer;

	glm::vec3 cam_target;

    bool show_tetrahedrons = false;

	unsigned int rendered_frame_texture_id;
    unsigned int visited_tets_texture_id;
    unsigned int locality_texture_id;

	std::vector<std::string> asset_file_names;
    std::vector<std::string> scene_file_names;

	static Editor *instance;
};