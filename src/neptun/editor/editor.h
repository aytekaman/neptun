#pragma once

#include "glm/glm.hpp"

#include <vector>
#include <string>

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
    void DrawBoundingBoxMenu();


    void HandleSelectionGizmos();
    void HandleTransformGizmos();

    static void DropCallback(GLFWwindow *window, int count, const char** paths);

    void InitSkin();

    bool show_rendered_frame_window = false;
    bool show_console_window = false;
    bool show_bounding_box_window = false;
    
    bool show_grid = true;
    bool edged_faces = false;

    //RenderingMode m_rendering_mode;

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
