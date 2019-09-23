#include "editor.h"

#define NOMINMAX

//#include "tinydir/tinydir.h"
#include "noc/noc_file_dialog.h"

#include "imgui/imgui.h"
#include "imgui_impl_glfw_gl3.h"

#include "ImGuizmo/ImGuizmo.h"

#include "gl3w/include/GL/gl3w.h"
#include "GLFW/glfw3.h"

#include "glm/gtc/random.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/transform.hpp"

#include "neptun/editor/graphics.h"
#include "neptun/main/asset_importer.h"
#include "neptun/main/bvh.h"
#include "neptun/main/color.h"
#include "neptun/main/filesystem.h"
#include "neptun/main/hilbert.h"
#include "neptun/main/image.h"
#include "neptun/main/kd_tree.h"
#include "neptun/main/light.h"
#include "neptun/main/logger.h"
#include "neptun/main/material.h"
#include "neptun/main/mesh.h"
#include "neptun/main/procedural_mesh_generator.h"
#include "neptun/main/ray_tracer.h"
#include "neptun/main/scene.h"
#include "neptun/main/sceneobject.h"
#include "neptun/main/stats.h"
#include "neptun/main/tet_mesh.h"
#include "neptun/main/texture.h"

void Editor::DropCallback(GLFWwindow* window, int count, const char** paths)
{
    for (int i = 0; i < count; i++)
    {
        fs::path file_path(paths[i]);
        std::string name = file_path.stem();
        //SceneObject *sceneObject = new SceneObject(name);

        //sceneObject->mesh = AssetImporter::ImportMesh(paths[i]);

        //instance->scene->add_scene_object(sceneObject);

        SceneObject *sceneObject = AssetImporter::CreateFromObj(paths[i]);
        instance->scene->add_scene_object(sceneObject);
    }
}

void Editor::InitSkin()
{
    ImGui::StyleColorsDark();



    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowRounding = 0.0f;
    //style.FramePadding = ImVec2(4, style.FramePadding.y);
 //   style.WindowBorderSize = 1;
 //   style.FrameBorderSize = 1;

    return;

    style.Colors[ImGuiCol_Text] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
    style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.2f, 0.2f, 0.2f, 1.00f);
    style.Colors[ImGuiCol_ChildWindowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    style.Colors[ImGuiCol_PopupBg] = ImVec4(0.2f, 0.2f, 0.2f, 1.00f);
    style.Colors[ImGuiCol_Border] = ImVec4(0.00f, 0.00f, 0.00f, 0.39f);
    style.Colors[ImGuiCol_BorderShadow] = ImVec4(1.00f, 1.00f, 1.00f, 0.10f);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
    style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.4f, 0.4f, 0.4f, 1.00f);
    style.Colors[ImGuiCol_TitleBg] = ImVec4(0.2f, 0.2f, 0.2f, 1.00f);
    style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.2f, 0.2f, 0.2f, 1.00f);
    style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.2f, 0.2f, 0.2f, 1.00f);
    style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.2f, 0.2f, 0.2f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.4f, 0.4f, 0.4f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.45f, 0.45f, 0.45f, 1.00f);
    //style.Colors[ImGuiCol_PopupBg] = ImVec4(0.3f, 0.3f, 0.3f, 0.99f);
    style.Colors[ImGuiCol_CheckMark] = ImVec4(0.6f, 0.6f, 0.6f, 0.99f);
    style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.6f, 0.6f, 0.6f, 0.99f);
    style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.7f, 0.7f, 0.7f, 0.99f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.35f, 0.35f, 0.35f, 1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.35f, 0.35f, 0.35f, 1.0f);

    style.Colors[ImGuiCol_Header] = ImVec4(0.9f, 0.5f, 0.0f, 1.0f);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.9f, 0.5f, 0.0f, 1.0f);
    style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.9f, 0.5f, 0.0f, 1.0f);

    style.Colors[ImGuiCol_Header] = ImVec4(0.17f, 0.57f, 0.69f, 1.0f);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.17f, 0.57f, 0.69f, 1.0f);
    style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.17f, 0.57f, 0.69f, 1.0f);

    style.Colors[ImGuiCol_Column] = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
    style.Colors[ImGuiCol_ColumnHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.78f);
    style.Colors[ImGuiCol_ColumnActive] = ImVec4(0.8f, 0.4f, 0.0f, 1.0f);
    style.Colors[ImGuiCol_ResizeGrip] = ImVec4(1.00f, 1.00f, 1.00f, 0.00f);
    style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
    style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
    style.Colors[ImGuiCol_CloseButton] = ImVec4(0.59f, 0.59f, 0.59f, 0.50f);
    style.Colors[ImGuiCol_CloseButtonHovered] = ImVec4(0.98f, 0.39f, 0.36f, 1.00f);
    style.Colors[ImGuiCol_CloseButtonActive] = ImVec4(0.98f, 0.39f, 0.36f, 1.00f);
    style.Colors[ImGuiCol_PlotLines] = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
    style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
    style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    style.Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
    style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.8f, 0.4f, 0.0f, 1.0f);
    style.Colors[ImGuiCol_ModalWindowDarkening] = ImVec4(0.20f, 0.20f, 0.20f, 0.35f);
}

Editor::Editor(Scene * scene_, Graphics * graphics_, RayTracer *ray_tracer_) : scene(scene_), graphics(graphics_), ray_tracer(ray_tracer_)
{
    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return;
    }

    glfwWindowHint(GLFW_MAXIMIZED, GL_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    window = glfwCreateWindow(1200, 600, "neptun", NULL, NULL);

    if (window == NULL)
    {
        fprintf(stderr, "Failed to open GLFW window.");
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);

    if (gl3wInit() != GL3W_OK)
    {
        fprintf(stderr, "Failed to initialize GLEW.");
        return;
    }

    glfwSetDropCallback(window, Editor::DropCallback);
    glfwSwapInterval(1);

    ImGui::CreateContext();

    ImGui_ImplGlfwGL3_Init(window, true);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glGenTextures(1, &rendered_frame_texture_id);
    glBindTexture(GL_TEXTURE_2D, rendered_frame_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, 0, GL_BGR, GL_UNSIGNED_BYTE, ray_tracer->m_rendered_image->get_pixels());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glGenTextures(1, &visited_tets_texture_id);
    glBindTexture(GL_TEXTURE_2D, visited_tets_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, 0, GL_BGR, GL_UNSIGNED_BYTE, ray_tracer->m_visited_tets_image->get_pixels());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glGenTextures(1, &locality_texture_id);
    glBindTexture(GL_TEXTURE_2D, locality_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, 0, GL_BGR, GL_UNSIGNED_BYTE, ray_tracer->m_locality_image->get_pixels());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glm::vec4 clear_color(0.1, 0.1, 0.1, 1.0);

    glClearColor(clear_color.r, clear_color.g, clear_color.b, clear_color.a);

    glEnable(GL_DEPTH_TEST);

    ImGuiIO& io = ImGui::GetIO();

    io.Fonts->AddFontFromFileTTF("../../fonts/DroidSans.ttf", 14);
    io.Fonts->AddFontFromFileTTF("../../fonts/Consolas.ttf", 12);


    glEnable(GL_MULTISAMPLE);

    InitSkin();

    graphics->Init();

    graphics->window = window;

    Logger::Log("Tetrosity is started.");
    Logger::Log("Welcome.");

    instance = this;
}

void Editor::Run()
{
    do
    {
        Draw();
    } while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);
}

void Editor::Draw()
{
    glfwPollEvents();
    ImGui_ImplGlfwGL3_NewFrame();

    ImGuizmo::BeginFrame();

    HandleTransformGizmos();

    DrawMainMenuBar();
    DrawHierarchy();
    DrawInspector();
    DrawTetGen();
    DrawScene();

    DrawRenderedFrame();
    DrawConsole();
    DrawBoundingBoxMenu();

    HandleSelectionGizmos();
    ImGui::Render();
    glfwSwapBuffers(window);
}

void Editor::DrawMainMenuBar()
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Save scene"))
            {
                scene->save_to_file();
            }

            if (ImGui::MenuItem("Save scene as"))
            {
                const char* file_name = noc_file_dialog_open(NOC_FILE_DIALOG_SAVE, "scene\0*.scene\0", nullptr, nullptr);
                Logger::Log("scene is saved as %s", file_name);
            }

            if (ImGui::MenuItem("Load scene"))
            {
                const char* file_name = noc_file_dialog_open(NOC_FILE_DIALOG_OPEN, "scene\0*.scene\0", ".\\Assets\\", nullptr);
                if (file_name)
                {
                    scene->load_from_file(file_name);
                    selected_scene_object = nullptr;
                    cam_target = scene->camTarget;
                }
            }

            if (ImGui::MenuItem("Reset"))
            {
                scene->clear();
                selected_scene_object = nullptr;
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Import OBJ file"))
            {
                const char* file_name = noc_file_dialog_open(NOC_FILE_DIALOG_OPEN, "obj\0*.obj\0", ".\\Assets\\", nullptr);
                if (file_name)
                {
                    SceneObject *scene_object = AssetImporter::CreateFromObj(file_name);
                    scene->add_scene_object(scene_object);

                    if (scene->sceneObjects.size() > 0)
                        selected_scene_object = scene->sceneObjects.back();
                }
            }

            if (ImGui::MenuItem("Exit", "ALT+F4"))
            {
                glfwSetWindowShouldClose(graphics->window, true);
            }

            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Edit"))
        {
            if (ImGui::MenuItem("Perturb objects"))
                for (SceneObject* scene_object : scene->sceneObjects)
                    scene_object->pos += glm::sphericalRand(0.5f);

            ImGui::Separator();

            if (ImGui::MenuItem("Preferences")) {}

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Create"))
        {
            if (ImGui::MenuItem("Cube"))
            {
                SceneObject* cube = new SceneObject("Cube");

                cube->mesh = ProceduralMeshGenerator::create_cube();
                scene->add_scene_object(cube);
                if (scene->sceneObjects.size() > 0)
                    selected_scene_object = scene->sceneObjects.back();
            }

            if (ImGui::MenuItem("Icosphere"))
            {
                SceneObject* icosphere = new SceneObject("Icosphere");

                icosphere->mesh = ProceduralMeshGenerator::create_icosphere();
                scene->add_scene_object(icosphere);
                if (scene->sceneObjects.size() > 0)
                    selected_scene_object = scene->sceneObjects.back();
            }

            if (ImGui::MenuItem("Torus"))
            {
                SceneObject* object = new SceneObject("Torus");

                object->mesh = ProceduralMeshGenerator::create_torus();
                scene->add_scene_object(object);
                if (scene->sceneObjects.size() > 0)
                    selected_scene_object = scene->sceneObjects.back();
            }

            if (ImGui::MenuItem("Terrain"))
            {
                SceneObject* terrain = new SceneObject("Terrain");

                terrain->mesh = ProceduralMeshGenerator::create_terrain();
                scene->add_scene_object(terrain);
                if (scene->sceneObjects.size() > 0)
                    selected_scene_object = scene->sceneObjects.back();
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Light"))
            {
                SceneObject *light = new SceneObject("Light");
                light->light = new Light();
                scene->add_scene_object(light);
                light->pos = glm::vec3(0, 10, 0);
                light->light->intensity = 1.0f;

                if (scene->sceneObjects.size() > 0)
                    selected_scene_object = scene->sceneObjects.back();
            }

            if (ImGui::MenuItem("Sky Dome"))
            {
                for (int i = 0; i < 256; i++)
                {
                    SceneObject *light = new SceneObject("Light");
                    light->light = new Light();
                    scene->add_scene_object(light);
                    light->pos = glm::sphericalRand(49.0f);
                    light->pos.y = glm::abs(light->pos.y);
                    light->light->intensity = 1.0f / (256.0f / (3.14f / 2.0f));
                }
            }

            ImGui::Separator();

            ImGui::MenuItem("Bounding Box", 0, &show_bounding_box_window, true);
            if (ImGui::MenuItem("Steiner Point"))
            {
                SceneObject* steiner_point = new SceneObject("Steiner point");
                scene->add_scene_object(steiner_point);
                if (scene->sceneObjects.size() > 0)
                    selected_scene_object = scene->sceneObjects.back();
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Rendered Frame", 0, &show_rendered_frame_window, true);
            ImGui::MenuItem("Console", 0, &show_console_window, true);
            ImGui::MenuItem("Visualization");

            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}



void Editor::DrawInspector()
{
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    //ImGui::GetStyle().WindowTitleAlign = ImVec2(0.025f, 0.5f);
    ImGui::Begin("Inspector", 0, flags);

    ImGui::SetWindowSize(ImVec2(240, (display_h - 20) / 2));
    ImGui::SetWindowPos(ImVec2(display_w - 240, 20));

    if (selected_scene_object)
    {
        ImGui::Checkbox("", &selected_scene_object->m_is_visible);

        ImGui::SameLine();

        ImGui::Text("%s", selected_scene_object->name.c_str());

        ImGui::Separator();

        ImGui::Text("Transform");

        if (ImGui::Button("P##1"))selected_scene_object->pos = glm::vec3();
        ImGui::SameLine();
        ImGui::DragFloat3("##4", &(selected_scene_object->pos.x), 0.05f, 0, 0, "%.3f");

        if (ImGui::Button("R##2"))selected_scene_object->rot = glm::vec3();
        ImGui::SameLine();
        ImGui::DragFloat3("##5", &(selected_scene_object->rot.x), 0.25f, 0, 0, "%.3f");

        if (ImGui::Button("S##3"))selected_scene_object->scale = glm::vec3(1,1,1);
        ImGui::SameLine();
        ImGui::DragFloat3("##6", &(selected_scene_object->scale.x), 0.05f, 0, 0, "%.3f");

        ImGui::Separator();

        if (selected_scene_object->light)
        {
            ImGui::Text("Light");
            ImGui::ColorEdit3("Color", &selected_scene_object->light->color.x);
            ImGui::DragFloat("Intensity", &selected_scene_object->light->intensity);
            ImGui::Separator();
        }

        if (selected_scene_object->mesh)
        {
            ImGui::Text("Mesh");

            ImGui::Text("Vertex count: %d", selected_scene_object->mesh->m_vertex_count);

            //if (ImGui::Button("Center Pivot Point"))
            //{
            //	selected_scene_object->mesh->CenterPivot();
            //	selected_scene_object->mesh->isDirty = true;
            //}
                
            ImGui::Checkbox("Ignore Tetrahedralization", &selected_scene_object->mesh->m_ignore_tetrahedralization);
            ImGui::Checkbox("Structure Mesh", &selected_scene_object->mesh->m_structure_mesh);

            ImGui::Separator();
        }

        if (selected_scene_object->material)
        {
            ImGui::Text("Material");
            ImGui::ColorEdit3("Diffuse", &selected_scene_object->material->diffuse.r);

            ImGui::Separator();

            Material *material = selected_scene_object->material;

            if (material->texture)
            {
                std::unordered_map<Texture*, GLuint>::const_iterator result = graphics->texture_handles.find(material->texture);

                if (result != graphics->texture_handles.end())
                {
                    float aspect = (float)material->texture->w / material->texture->h;

                    ImVec2 size(ImGui::GetContentRegionAvailWidth(), ImGui::GetContentRegionAvailWidth() / aspect);

                    ImGui::Image((ImTextureID)(intptr_t)result->second, size);
                }
            }
        }

        //ImGui::ShowStyleEditor();

        //ImGui::Button("Add Component");
    }

    ImGui::End();
}

void Editor::DrawHierarchy()
{
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    //ImGui::GetStyle().WindowTitleAlign = ImVec2(0.1f, 0.5f);
    ImGui::Begin("Scene Objects", 0, flags);
    ImGui::SetWindowSize(ImVec2(120.0f, (display_h - 20) / 2.0f));
    ImGui::SetWindowPos(ImVec2(display_w - 360.0f, 20.0f));

    static ImVec2 rect_min, rect_max;

    for (int i = 0; i < scene->sceneObjects.size(); i++)
    {
        if (scene->sceneObjects[i]->m_hide_in_editor)
            continue;

        ImGui::PushID(i);

        if (ImGui::Selectable(scene->sceneObjects[i]->name.c_str(), selected_scene_object == scene->sceneObjects[i]))
        {
            selected_scene_object = scene->sceneObjects[i];
            rect_min = ImGui::GetItemRectMin();
            rect_max = ImGui::GetItemRectMax();
        }

        ImGui::PopID();
    }

    if (ImGui::IsMouseDoubleClicked(0) && ImGui::IsMouseHoveringRect(rect_min, rect_max) && selected_scene_object)
    {
        cam_target = selected_scene_object->pos;
    }

    if (ImGui::GetIO().KeysDown[GLFW_KEY_F])
    {
        if (selected_scene_object)
            cam_target = selected_scene_object->pos;
        else
            cam_target = glm::vec3();
    }

    if (ImGui::GetIO().KeysDown[GLFW_KEY_DELETE])
    {
        for (int i = 0; i < scene->sceneObjects.size(); i++)
        {
            if (selected_scene_object == scene->sceneObjects[i])
            {
                scene->sceneObjects.erase(scene->sceneObjects.begin() + i);
                selected_scene_object = NULL;
                break;
            }
        }
    }

    scene->camTarget = glm::mix(scene->camTarget, cam_target, 0.1);

    ImGui::End();
}

void Editor::DrawTetGen()
{
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImGui::GetStyle().Colors[ImGuiCol_TitleBg]);

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse;


    //ImGui::OpenPopup("Modal window");

    //bool open = true;
    //if (ImGui::BeginPopupModal("Modal window", &open))
    //{
    //	ImGui::Text("Hello dsjfhds fhjs hfj dshfj hds");
    //	/*if (ImGui::Button("Close"))
    //		ImGui::CloseCurrentPopup();*/
    //	ImGui::EndPopup();
    //}

    ImGui::Begin("TetGen", 0, flags);

    static bool preserveTriangles = true;
    static bool half_space_optimization = false;
    static bool process_light_sources = false;
    static bool create_bounding_box = true;
    static float quality = 5.0f;

    ImGui::Checkbox("Preserve Triangles", &preserveTriangles);
    ImGui::Checkbox("Half-space Optimization", &half_space_optimization);
    ImGui::Checkbox("Create Bounding Box", &create_bounding_box);
    ImGui::SliderFloat("Quality", &quality, 1.0, 10.0, "%.2f");


    static int current_tet_mesh_type = 0;
    const char* reps[] = { "TetMesh32", "TetMesh20", "TetMesh16", "TetMesh80" };
    ImGui::Combo("Type", &current_tet_mesh_type, reps, 4);

    static bool show_points = false;
    //static

    //std::thread *t = nullptr;

    if (ImGui::Button("Build"))
    {
        //t = new std::thread(&TetMesh::BuildFromScene, tet_mesh, *scene, preserveTriangles, create_bounding_box);
        //t->detach();


        //t.join();
        if (scene->tet_mesh)
            delete scene->tet_mesh;

        if (current_tet_mesh_type == 0)
        {
            scene->tet_mesh = new TetMesh32(*scene, preserveTriangles, create_bounding_box, quality, half_space_optimization);
            Logger::Log("TetMesh32 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
        }
        else if (current_tet_mesh_type == 1)
        {
            scene->tet_mesh = new TetMesh20(*scene, preserveTriangles, create_bounding_box, quality);
            Logger::Log("TetMesh20 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
        }
        else if (current_tet_mesh_type == 2)
        {
            scene->tet_mesh = new TetMesh16(*scene, preserveTriangles, create_bounding_box, quality);
            Logger::Log("TetMesh16 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
        }
        else if (current_tet_mesh_type == 3)
        {
            scene->tet_mesh = new TetMesh80(*scene, preserveTriangles, create_bounding_box, quality);
            Logger::Log("TetMesh80 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
        }
    }

    ImGui::SameLine();

    if (ImGui::Button("Save"))
    {
        scene->tet_mesh->write_to_file();
    }

    ImGui::SameLine();

    if (ImGui::Button("Load"))
    {
        if (current_tet_mesh_type == 0)
        {
            scene->tet_mesh = new TetMesh32(*scene);
            Logger::Log("TetMesh32 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
        }
        else if (current_tet_mesh_type == 1)
        {
            scene->tet_mesh = new TetMesh20(*scene);
            Logger::Log("TetMesh20 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
        }
        else if (current_tet_mesh_type == 2)
        {
            scene->tet_mesh = new TetMesh16(*scene);
            Logger::Log("TetMesh16 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
        }
        else if (current_tet_mesh_type == 3)
        {
            scene->tet_mesh = new TetMesh80(*scene);
            Logger::Log("TetMesh80 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
        }

        //Logger::Log("TetMesh32 size: %d MB", scene->tet_mesh->get_size_in_bytes() / (1024 * 1024));
    }

    ImGui::Separator();

    static bool show_ffd = false;

    //static float slicer = 0.0f;

    if (scene->tet_mesh)
    {
        ImGui::Text("Statistics:");
        ImGui::Text("Number of points:");
        ImGui::SameLine(180);
        ImGui::Text("%zu", scene->tet_mesh->m_points.size());

        ImGui::Text("Number of tetrahedrons:");
        ImGui::SameLine(180);
        ImGui::Text("%zu", scene->tet_mesh->m_tets.size());

        ImGui::Text("Weight:");
        ImGui::SameLine(180);
        ImGui::Text("%.2f", scene->tet_mesh->m_weight);

        //ImGui::Text("Number of triangles:");
        //ImGui::SameLine(180);
        //ImGui::Text("%d", out.numberoffacets);

        static int bit_count = 16U;
        static bool use_regions = false;
        static bool swap = false;
        ImGui::SliderInt("Bits", &bit_count, 1U, 21U);
        ImGui::Checkbox("Regions", &use_regions);
        ImGui::Checkbox("Swap", &swap);

        if (ImGui::Button("Sort (Hilbert)"))
            scene->tet_mesh->sort(SortingMethod::Hilbert, bit_count, use_regions, swap);

        ImGui::SameLine();

        if (ImGui::Button("Sort (Morton)"))
            scene->tet_mesh->sort(SortingMethod::Morton, bit_count, use_regions);


        ImGui::Separator();
        ImGui::Checkbox("Show points", &show_points);
        ImGui::Checkbox("Show tetrahedrons", &show_tetrahedrons);
        ImGui::SameLine();
        ImGui::SliderFloat("Slice", &scene->tet_mesh->slice, -51, 51);

        //ImGui::Checkbox("Deformation", &deformation);
        ImGui::Checkbox("FFD", &show_ffd);
    }

    //if (ray_tracer->kd_tree)
    //{
    //    graphics->DrawCube(ray_tracer->kd_tree->bounds.min, ray_tracer->kd_tree->bounds.max);

    //    //std::vector<Bounds3>& bounds = ray_tracer->kd_tree->leafBounds;

    //    //for (int i = 0; i < bounds.size(); i++)
    //    //{
    //    //    graphics->DrawCube(bounds[i].min, bounds[i].max);
    //    //}
    //}

    if (ImGui::Button("Build kd-tree"))
        scene->build_kd_tree();

    if (ImGui::Button("Build BVH (pbrt)"))
        scene->build_bvh();

    if (ImGui::Button("Build BVH (embree)"))
        scene->build_bvh_embree();

    ImGui::SetWindowSize(ImVec2(360, (display_h - 20) / 2));
    ImGui::SetWindowPos(ImVec2(display_w - 360, 20 + (display_h - 20) / 2));
    ImGui::End();

    // Free Form Deformation
    /*static bool ffd_created = false;
    static SceneObject *ffd[4][4][4];

    if (!ffd_created)
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                {
                    ffd[i][j][k] = new SceneObject("FFD Node");
                    ffd[i][j][k]->pos = glm::vec3(i, j, k);
                    ffd[i][j][k]->m_hide_in_editor = true;
                    ffd[i][j][k]->color = Color::orange;
                    scene->add_scene_object(ffd[i][j][k]);
                }

        ffd_created = true;
    }
    else
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                {
                    ffd[i][j][k]->m_is_pickable = show_ffd;

                    if (show_ffd && i < 3) graphics->DrawLine(ffd[i][j][k]->pos, ffd[i + 1][j][k]->pos, Color::orange);
                    if (show_ffd && j < 3) graphics->DrawLine(ffd[i][j][k]->pos, ffd[i][j + 1][k]->pos, Color::orange);
                    if (show_ffd && k < 3) graphics->DrawLine(ffd[i][j][k]->pos, ffd[i][j][k + 1]->pos, Color::orange);

                    if (!show_ffd && selected_scene_object == ffd[i][j][k])
                        selected_scene_object = false;
                }
    }*/
    glm::vec4 colors[5] = { Color::turquoise, Color::orchid, Color::yolk, Color::melon, Color::berry };

    //if (show_points)
    //{
    //    glm::vec3 up(1, 1, 1);
    //    float scale = 0.01f;

    //    bool *is_reachable = new bool[scene->tet_mesh->num_points]();

    //    for (int i = 0; i < scene->tet_mesh->num_tets; i++)
    //    {
    //        if (scene->tet_mesh->tets[i].region_id == scene->tet_mesh->air_region_id)
    //        {
    //            for (int j = 0; j < 4; j++)
    //            {
    //                is_reachable[scene->tet_mesh->tets[i].vertices[j]] = true;
    //            }
    //        }
    //    }

    //    for (int i = 0; i < scene->tet_mesh->num_points; i++)
    //    {

    //        glm::vec3 center = scene->tet_mesh->points[i];
    //        glm::vec3 min = center + up * scale;
    //        glm::vec3 max = center - up * scale;

    //        glm::vec4 color = i < scene->tet_mesh->reachable_point_count ? Color::turquoise : Color::melon;

    //        graphics->DrawCube(min, max, color);
    //    }
    //}




    if (scene->tet_mesh)
    {
        static bool dummy_is_created = false;
        static SceneObject* dummy = nullptr;

        if (!dummy_is_created)
        {
            dummy = new SceneObject("Dummy");
            scene->add_scene_object(dummy);
            dummy_is_created = true;
        }

        //glm::vec3 p[4];
        //
        //SourceTet tet = scene->tet_mesh->m_source_tet;

        //for (int i = 0; i < 4; i++)
        //{
        //    p[i] = scene->tet_mesh->m_points[tet.v[i]];
        //}

        //for (int i = 0; i < 3; i++)
        //	for (int j = i + 1; j < 4; j++)
        //		graphics->DrawLine(p[i], p[j]);

        //SourceTet dummy_src;

        //int tet_index = scene->tet_mesh->find_tet(dummy->pos, dummy_src);

        //for (int i = 0; i < 4; i++)
        //{
        //    p[i] = scene->tet_mesh->m_points[scene->tet_mesh->m_tets[tet_index].v[i]];
        //}

        //for (int i = 0; i < 3; i++)
        //    for (int j = i + 1; j < 4; j++)
        //        graphics->DrawLine(p[i], p[j]);

        //graphics->DrawTet(p);

        /*SourceTet source_tet;

        scene->tet_mesh32->find_tet(dummy->pos, source_tet);

        std::cout << std::endl;
        for (int i = 0; i < 4; ++i)
            std::cout << source_tet.n[i] << " ";

        scene->tet_mesh20->find_tet(dummy->pos, source_tet);

        std::cout << std::endl;
        for (int i = 0; i < 4; ++i)
            std::cout << source_tet.n[i] << " ";

        scene->tet_mesh->find_tet(dummy->pos, source_tet);

        glm::vec3 v[4];

        for (int i = 0; i < 4; ++i)
        {
            v[i] = scene->tet_mesh->m_points[source_tet.v[i]];
        }

        for (int i = 0; i < 3; i++)
            for (int j = i + 1; j < 4; j++)
                graphics->DrawLine(v[i], v[j]);

        std::cout << std::endl;
        for (int i = 0; i < 4; ++i)
            std::cout << source_tet.n[i] << " ";

        std::cout << std::endl;*/

        //std::cout << iss_point_inside_tet(v, dummy->pos) << std::endl;
        //std::cout << std::endl;

        //int tet_idx = scene->tet_mesh32->find_tet(dummy->pos, source_tet);

        //std::cout << "fast search: " << tet_idx << std::endl;

        //tet_idx = scene->tet_mesh32->find_tet_brute_force(dummy->pos);

        //std::cout << "brute force search: " << tet_idx << std::endl;

    }

    if (show_tetrahedrons)
    {

        //for (int index = 0; index < scene->tet_mesh->num_tets; index++)
        //{
        //	glm::vec3 c;

        //	for (int i = 0; i < 4; i++)
        //	{
        //		c += scene->tet_mesh->points[scene->tet_mesh->tets[index].vertices[i]] * 0.25f;
        //	}

        //	glm::vec4 color = colors[(scene->tet_mesh->tets[index].region_id + 1) % 5];

        //	glm::vec3 p[4];

        //	for (int i = 0; i < 4; i++)
        //	{
        //		p[i] = scene->tet_mesh->points[scene->tet_mesh->tets[index].vertices[i]];
        //	}

        //	//if (tet_mesh->tets[index].region_id == tet_mesh->air_region_id && c.z < slicer)
        //	//	graphics->DrawTet(p, glm::vec4(1,1,1,1));

        //	//if (tet_mesh->tets[index].region_id != tet_mesh->air_region_id)
        //	//	//DrawTetrahedron(index, graphics, color);
        //	//	graphics->DrawTet(p, color);

  //          if (c.z < slicer)
  //          {
  //              graphics->DrawTet(p, Color::lerp(Color::turquoise, Color::yellow, Color::melon, (float)index/ scene->tet_mesh->num_tets));
  //          }
        //}
    }

    ImGui::PopStyleColor();
}

void Editor::DrawScene()
{
    static bool KeysDownPrev[512];

    if (!KeysDownPrev[GLFW_KEY_G] && ImGui::GetIO().KeysDown[GLFW_KEY_G])
        show_grid = !show_grid;

    static RenderingMode renderingMode = RenderingMode::Solid;

    if (!KeysDownPrev[GLFW_KEY_W] && ImGui::GetIO().KeysDown[GLFW_KEY_W])
    {
        renderingMode = renderingMode == RenderingMode::Solid ? RenderingMode::SolidWireframe : RenderingMode::Solid;
    }

    if (!KeysDownPrev[GLFW_KEY_H] && ImGui::GetIO().KeysDown[GLFW_KEY_H])
    {
        for (int i = 0; i < scene->sceneObjects.size(); i++)
            scene->sceneObjects[i]->m_is_visible = false;
    }


    if (!KeysDownPrev[GLFW_KEY_D] && ImGui::GetIO().KeysDown[GLFW_KEY_D] && selected_scene_object)
    {
        SceneObject *clone = new SceneObject(*selected_scene_object);

        clone->randomize_color();

        if (selected_scene_object->mesh)
        {
            Mesh *mesh = new Mesh(*clone->mesh);
            clone->mesh = mesh;
            mesh->m_is_dirty = true;
        }

        if (selected_scene_object->light)
        {
            Light *light = new Light(*clone->light);
            clone->light = light;
            light->isDirty = true;
        }

        scene->add_scene_object(clone);
        selected_scene_object = clone;
    }


    float grid_scale = 1;

    static ImVec2 lastMousePos[5];

    if(!ImGui::GetIO().WantCaptureMouse)
        scene->camDist -= ImGui::GetIO().MouseWheel * scene->camDist * 0.05;
    scene->camDist = glm::clamp(scene->camDist, 1.0f, 10000.0f);

    for (int i = 0; i < 5; i++)
        if (ImGui::GetIO().MouseClicked[i])
            lastMousePos[i] = ImGui::GetIO().MousePos;

    if (ImGui::GetIO().KeysDown[GLFW_KEY_LEFT_ALT])
        ImGuizmo::Enable(false);

    if (ImGui::GetIO().KeysDown[GLFW_KEY_LEFT_ALT] && !ImGuizmo::IsUsing() && ImGui::GetIO().MouseDown[0] && !ImGui::GetIO().WantCaptureMouse)
    {
        ImVec2 mousePos = ImGui::GetIO().MousePos;

        float xDelta = mousePos.x - lastMousePos[0].x;
        float yDelta = mousePos.y - lastMousePos[0].y;

        scene->camOrbitY += xDelta * 0.01f;
        scene->camOrbitX += yDelta * 0.01f;

        scene->camOrbitX = glm::clamp(scene->camOrbitX, -glm::half_pi<float>() * 0.99f, glm::half_pi<float>() * 0.99f);

        lastMousePos[0] = mousePos;
    }

    if (ImGui::GetIO().MouseDown[2] && !ImGui::GetIO().WantCaptureMouse)
    {
        ImVec2 mousePos = ImGui::GetIO().MousePos;

        float xDelta = mousePos.x - lastMousePos[2].x;
        float yDelta = mousePos.y - lastMousePos[2].y;

        glm::vec3 dir = glm::vec3(glm::cos(scene->camOrbitY), 0, glm::sin(scene->camOrbitY));

        dir = dir * glm::cos(scene->camOrbitX);

        dir.y = glm::sin(scene->camOrbitX);

        glm::vec3 right = glm::cross(dir, glm::vec3(0, 1, 0));
        glm::vec3 up = glm::cross(right, dir);

        cam_target += (right * xDelta + up * yDelta) * 0.01f;
        scene->camTarget = cam_target;

        lastMousePos[2] = mousePos;
    }

    //if(scene->tet_mesh)
    //    for (size_t i = 0; i < scene->tet_mesh->m_optimized_faces.size(); i += 3)
    //    {
    //        graphics->DrawLine(scene->tet_mesh->m_optimized_faces[i + 0], scene->tet_mesh->m_optimized_faces[i + 1]);
    //        graphics->DrawLine(scene->tet_mesh->m_optimized_faces[i + 1], scene->tet_mesh->m_optimized_faces[i + 2]);
    //        graphics->DrawLine(scene->tet_mesh->m_optimized_faces[i + 2], scene->tet_mesh->m_optimized_faces[i + 0]);
    //    }


    //if (show_grid)
    //	DrawLights(graphics);

    if (show_grid)
    {
        glm::vec3 grid_color(0.5, 0.5, 0.5);

        for (int i = -50; i <= 50; i++)
            graphics->DrawLine(glm::vec3(i, 0, -50) * grid_scale, glm::vec3(i, 0, 50) * grid_scale, glm::vec4(grid_color * (float)(i % 10 != 0), 0.5f));

        for (int i = -50; i <= 50; i++)
            graphics->DrawLine(glm::vec3(-50, 0, i) * grid_scale, glm::vec3(50, 0, i) * grid_scale, glm::vec4(grid_color * (float)(i % 10 != 0), 0.5f));
    }

    if(show_grid)
        for (int i = 0; i < scene->sceneObjects.size(); i++)
        {
            if (selected_scene_object == scene->sceneObjects[i] && selected_scene_object->mesh)
            {
                glm::mat4 t = glm::translate(glm::mat4(1.0f), scene->sceneObjects[i]->pos);
                glm::vec3 rot = glm::radians(scene->sceneObjects[i]->rot);
                glm::mat4 r = glm::eulerAngleYXZ(rot.y, rot.x, rot.z);
                glm::mat4 s = glm::scale(glm::mat4(1.0), scene->sceneObjects[i]->scale);

                s[3][3] = 1;

                glm::mat4 m = t * r * s;

                glm::vec3 min = selected_scene_object->mesh->m_bounds_min;
                glm::vec3 max = selected_scene_object->mesh->m_bounds_max;

                //glm::vec3 a = glm::vec3(m * glm::vec4(selected_scene_object->mesh->min, 1));
                //glm::vec3 b = glm::vec3(m * glm::vec4(selected_scene_object->mesh->max, 1));

                glm::vec4 color = selected_scene_object->color * 0.4f;
                color = Color::orange;

                glm::vec3 a(min.x, min.y, min.z);
                glm::vec3 b(min.x, min.y, max.z);
                glm::vec3 c(max.x, min.y, max.z);
                glm::vec3 d(max.x, min.y, min.z);

                glm::vec3 e(min.x, max.y, min.z);
                glm::vec3 f(min.x, max.y, max.z);
                glm::vec3 g(max.x, max.y, max.z);
                glm::vec3 h(max.x, max.y, min.z);

                a = glm::vec3(m * glm::vec4(a, 1));
                b = glm::vec3(m * glm::vec4(b, 1));
                c = glm::vec3(m * glm::vec4(c, 1));
                d = glm::vec3(m * glm::vec4(d, 1));
                e = glm::vec3(m * glm::vec4(e, 1));
                f = glm::vec3(m * glm::vec4(f, 1));
                g = glm::vec3(m * glm::vec4(g, 1));
                h = glm::vec3(m * glm::vec4(h, 1));

                graphics->DrawLine(a, b, color);
                graphics->DrawLine(b, c, color);
                graphics->DrawLine(c, d, color);
                graphics->DrawLine(d, a, color);

                graphics->DrawLine(e, f, color);
                graphics->DrawLine(f, g, color);
                graphics->DrawLine(g, h, color);
                graphics->DrawLine(h, e, color);

                graphics->DrawLine(a, e, color);
                graphics->DrawLine(b, f, color);
                graphics->DrawLine(c, g, color);
                graphics->DrawLine(d, h, color);
            }
        }

 /*   const int bits = 3;
    const int dim = 8;

    glm::vec3 sorted[dim * dim * dim];

    for (int i = 0; i < dim; i++)
        for (int j = 0; j < dim; j++)
            for (int k = 0; k < dim; k++)
            {
                bitmask_t coords[3] = { i,j,k };

                bitmask_t index = 0;

                for (unsigned int j = 0; j < bits; ++j)
                {
                    bitmask_t mask = 1ULL << (bits - j - 1);

                    index |= ((coords[0] & mask) << (bits * 2 - 0 - 2 * j));
                    index |= ((coords[1] & mask) << (bits * 2 - 1 - 2 * j));
                    index |= ((coords[2] & mask) << (bits * 2 - 2 - 2 * j));
                }

                sorted[index] = glm::vec3(i, j, k);
            }

    for (int i = 0; i < dim * dim * dim - 1; i++)
    {
        graphics->DrawLine(sorted[i], sorted[i+1]);
    }*/

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    glViewport(0, 0, display_w - 360, display_h - 20);

    if (edged_faces)
        renderingMode = RenderingMode::SolidWireframe;
    else
        renderingMode = RenderingMode::Solid;

    graphics->Render(scene, show_tetrahedrons, renderingMode);

    glViewport(0, 0, display_w, display_h);
    //glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);

    for (int i = 0; i < 512; i++)
        KeysDownPrev[i] = ImGui::GetIO().KeysDown[i];

    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4());
    ImGui::SetNextWindowPos(ImVec2(0, 20));
    ImGui::Begin("Scene Options", NULL, ImVec2(-1, 0), 0, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::Checkbox("Grid", &show_grid);
    ImGui::Checkbox("Edged Faces", &edged_faces);
    ImGui::End();
    ImGui::PopStyleColor();

    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4());
    ImGui::SetNextWindowPos(ImVec2(display_w - 520, 20));
    ImGui::Begin("Scene Stats", NULL, ImVec2(-1, 0), 0, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus);
    
    const size_t scene_object_count = scene->sceneObjects.size();
    ImGui::Text("Number of objects: %zd", scene_object_count);
    const size_t total_triangle_count = scene->get_triangle_count();
    ImGui::Text("Triangle count: %zd", total_triangle_count);
    ImGui::End();
    ImGui::PopStyleColor();
}

void Editor::DrawRenderedFrame()
{
    if (!show_rendered_frame_window)
        return;

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;// = ImGuiWindowFlags_ShowBorders;

    ImGui::Begin("Rendered Frame", &show_rendered_frame_window, flags);

    float cw = ImGui::GetContentRegionAvailWidth() * 0.65f;

    float image_scale = 1;

    ImGui::Image((ImTextureID)(intptr_t)rendered_frame_texture_id, ImVec2(cw, cw * (float)ray_tracer->m_resolution.y / ray_tracer->m_resolution.x));
    ImGui::SameLine();
    ImGui::BeginChild("asd", ImVec2(0, 0));

    //(printf("Rendered in %.3f seconds. (%.1f FPS)", ray_tracer->last_render_time, 1 / ray_tracer->last_render_time);
    ImGui::Text("Rendered in %.4f seconds. (%.2f FPS)", Stats::get_avg_render_time(10), 1 / Stats::get_avg_render_time(10));


    //ImGui::Text("Triangle count: %d", scene->tet_mesh->face_count);
    ImGui::InputInt("Thread count", &ray_tracer->thread_count);

    //ImGui::Checkbox("Multi-threading", &ray_tracer->multi_threading);

    static glm::ivec2 res = ray_tracer->m_resolution;

    ImGui::InputInt2("Resolution", &res.x);
    //ImGui::InputInt("Size", &ray_tracer->tile_size);
    ImGui::Checkbox("Shadows", &ray_tracer->shadows);

    const char* reps[] = { "Default", "Default (SIMD)", "ScTP", "Fast Basis", "kd-tree", "BVH (pbrt)", "BVH (embree)" };
    ImGui::Combo("Method", (int*)&ray_tracer->method, reps, 7);

    if (res != ray_tracer->m_resolution)
    {
        ray_tracer->set_resolution(res);

        //glGenTextures(1, &rendered_frame_texture_id);
        glBindTexture(GL_TEXTURE_2D, rendered_frame_texture_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, 0, GL_BGR, GL_UNSIGNED_BYTE, ray_tracer->m_rendered_image->get_pixels());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

        //glGenTextures(1, &visited_tets_texture_id);
        glBindTexture(GL_TEXTURE_2D, visited_tets_texture_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, 0, GL_BGR, GL_UNSIGNED_BYTE, ray_tracer->m_visited_tets_image->get_pixels());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

        //glGenTextures(1, &locality_texture_id);
        glBindTexture(GL_TEXTURE_2D, locality_texture_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, 0, GL_BGR, GL_UNSIGNED_BYTE, ray_tracer->m_locality_image->get_pixels());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }

    static bool render = false;
    static bool diagnostics = false;
    ImGui::Checkbox("Render", &render);

    if (render && !scene->has_accelerator())
    {
        Logger::LogWarning("can't render because there is no accelerator in the scene.");
        render = false;
    }

    ImGui::Separator();
    ImGui::Checkbox("Diagnostics", &diagnostics);
    ImGui::Text("Avg. intersection test count:");
    ImGui::SameLine(180);
    ImGui::Text("%.1f", ray_tracer->avg_test_count);

    ImGui::Text("L1 hit count:");
    ImGui::SameLine(180);
    ImGui::Text("%d", ray_tracer->L1_count);

    //ImGui::Checkbox("Visualize cost", &ray_tracer.show_traversed_tet_count);
    //ImGui::SliderInt("Scale: ", &cost_scale, 64, 512);
    if (ImGui::Button("Save"))
        ray_tracer->save_to_disk("last.png");
    ImGui::EndChild();

    if (render && scene->has_accelerator())
    {
        ray_tracer->Render(*scene, diagnostics);
        glBindTexture(GL_TEXTURE_2D, rendered_frame_texture_id);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, GL_BGR, GL_UNSIGNED_BYTE, ray_tracer->m_rendered_image->get_pixels());

        glBindTexture(GL_TEXTURE_2D, visited_tets_texture_id);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, GL_RGB, GL_UNSIGNED_BYTE, ray_tracer->m_visited_tets_image->get_pixels());

        glBindTexture(GL_TEXTURE_2D, locality_texture_id);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, ray_tracer->m_resolution.x, ray_tracer->m_resolution.y, GL_RGB, GL_UNSIGNED_BYTE, ray_tracer->m_locality_image->get_pixels());
    }

    ImGui::End();

    ImGui::Begin("Diag");

    glm::vec2 image_size(cw, cw * (float)ray_tracer->m_resolution.y / ray_tracer->m_resolution.x);

    ImGui::Image((ImTextureID)(intptr_t)visited_tets_texture_id, ImVec2(cw, cw * (float)ray_tracer->m_resolution.y / ray_tracer->m_resolution.x));

    if (ImGui::IsItemHovered())
    {
        const ImVec2 mouse_pos = ImGui::GetMousePos();

        glm::vec2 pixel_coords(
            int(ImGui::GetMousePos().x - ImGui::GetItemRectMin().x),
            int(ImGui::GetMousePos().y - ImGui::GetItemRectMin().y));

        pixel_coords /= image_size;
        pixel_coords *= ray_tracer->m_resolution;

        const int visited_node_count = ray_tracer->stats.get((glm::ivec2)pixel_coords);

        const std::string text = std::string("Pixel: (") + std::to_string((int)pixel_coords.x) + ", " + std::to_string((int)pixel_coords.y) + ")\n" +
            "Visited nodes: " + std::to_string(visited_node_count);

        const ImVec2 text_size = ImGui::CalcTextSize(text.c_str());

        const int padding = 5;

        ImVec2 text_pos = mouse_pos;
        text_pos.x += 20;

        ImVec2 popup_top_left = text_pos;
        popup_top_left.x -= padding;
        popup_top_left.y -= padding;

        ImVec2 popup_bottom_right = popup_top_left;
        popup_bottom_right.x += text_size.x + padding * 2;
        popup_bottom_right.y += text_size.y + padding * 2;

        ImGui::GetOverlayDrawList()->AddRectFilled(popup_top_left, popup_bottom_right, ImColor(0, 0, 0, 200));
        ImGui::GetOverlayDrawList()->AddRect(popup_top_left, popup_bottom_right, ImColor(128, 128, 128, 255));
        ImGui::GetOverlayDrawList()->AddText(text_pos, ImColor(255, 255, 255, 255), text.c_str());
    }

    ImGui::SameLine();
    ImGui::Image((ImTextureID)(intptr_t)locality_texture_id, ImVec2(cw, cw * (float)ray_tracer->m_resolution.y / ray_tracer->m_resolution.x));
    
    static std::string save_filename = "";
    if (ImGui::Button("Save"))
    {
        save_filename = "tet_cost_" + std::to_string(clock()) + ".png";

        ray_tracer->save_to_disk(save_filename.c_str(), Tet_cost);
    }

    if (save_filename.empty() == false)
    {
        ImGui::SameLine();
        ImGui::Text("Saved: %s", save_filename.c_str());
    }

    ImGui::End();

}

void Editor::DrawConsole()
{
    if (!show_console_window)
        return;

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;// = ImGuiWindowFlags_ShowBorders;

    ImGui::Begin("Console", &show_console_window, flags);

    if (ImGui::Button("Clear"))
        Logger::Clear();

    static bool show_logs = true, show_warnings = true, show_errors = true;

    ImGui::SameLine();
    ImGui::Checkbox("Show Logs", &show_logs);
    ImGui::SameLine();
    ImGui::Checkbox("Show Warnings", &show_warnings);
    ImGui::SameLine();
    ImGui::Checkbox("Show Errors", &show_errors);

    float h = ImGui::GetContentRegionAvail().y  - 100;

    ImGui::PushFont(ImGui::GetIO().Fonts->Fonts[1]);
    ImGui::PushStyleColor(ImGuiCol_ChildWindowBg, ImVec4(0.1f, 0.1f, 0.1f, 1.0f));

    ImGui::BeginChild("Message List", ImVec2(0, h), false, flags);

    const std::vector<Msg> &logs = Logger::logs;

    static int selected_msg_id = -1;

    for (size_t i = 0; i < logs.size(); i++)
    {
        if (!show_logs && logs[i].type == 0)
            continue;
        if (!show_warnings && logs[i].type == 1)
            continue;
        if (!show_errors && logs[i].type == 2)
            continue;

        ImGui::PushID(i);

        if (logs[i].type == 1)
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.92f, 0.016f, 1.0f));
        if (logs[i].type == 2)
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));

        if (ImGui::Selectable(logs[i].text.c_str(), selected_msg_id == i))
        {
            selected_msg_id = i;
        }

        if (logs[i].type > 0)
            ImGui::PopStyleColor();

        ImGui::PopID();

        //ImGui::PushStyleColor(ImGuiCol_ChildWindowBg, ImColor(0.1f, 0.1f, 0.1f, 0.1f));
        //ImGui::BeginChild(msgs[i].c_str(), ImVec2(0, 30), false);
        //ImGui::Text(msgs[i].c_str());
        //ImGui::EndChild();
        //ImGui::PopStyleColor();
    }

    ImGui::EndChild();

    ImGui::BeginChild("Message Window", ImVec2(0, 0), false, flags);
    if(selected_msg_id != -1)
        ImGui::Text("%s", logs[selected_msg_id].text.c_str());
    ImGui::End();

    ImGui::PopFont();

    ImGui::PopStyleColor();

    ImGui::End();
}

void Editor::DrawBoundingBoxMenu()
{
    if (show_bounding_box_window == false)
        return;

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;// = ImGuiWindowFlags_ShowBorders;

    ImGui::SetNextWindowSizeConstraints({200, 200}, {10000, 10000});
    if (ImGui::Begin("Bounding Box", &show_bounding_box_window, flags))
    {
        ImGui::Separator();
        ImGui::Text("Bounding Box Settings");

        static float bounding_box_padding_size = 0.01;
        ImGui::DragFloat("Padding", &bounding_box_padding_size, 0.01f);
        
        // DragFloat negative bug fix
        if (bounding_box_padding_size < 0)
            bounding_box_padding_size = 0;

        static int bounding_box_step_count[3] = {4, 4, 4};
        ImGui::SliderInt3("Subdivision Count", bounding_box_step_count, 1, 50);
        
        ImGui::NewLine();
        ImGui::Separator();
        ImGui::NewLine();

        const ImVec2 create_button_size = {50, 30};
        ImGui::Indent(ImGui::GetWindowWidth() - create_button_size.x - 30);
        
        if (ImGui::Button("Create", create_button_size))
        {
            for (SceneObject* obj : scene->sceneObjects)
            {
                Mesh* mesh = obj->mesh;
                if (mesh == nullptr || mesh->m_ignore_tetrahedralization || mesh->m_structure_mesh)
                    continue;

                SceneObject* bounding_box_object = new SceneObject(obj->name + "_BB");
                
                bounding_box_object->rot = obj->rot;
                bounding_box_object->pos = obj->pos;
                bounding_box_object->scale = obj->scale;
                bounding_box_object->m_is_visible = false;
                bounding_box_object->m_is_pickable = false;

                mesh->calculate_bounds();
                const glm::vec3 mesh_size = mesh->m_bounds_max - mesh->m_bounds_min + (bounding_box_padding_size * 2);

                bounding_box_object->mesh = ProceduralMeshGenerator::create_cube(mesh_size, glm::make_vec3(bounding_box_step_count));
                bounding_box_object->mesh->m_structure_mesh = true;

                scene->add_scene_object(bounding_box_object); 
            }

            show_bounding_box_window = false; // Close bounding box window ?
        }
    }
    ImGui::End();
}

void Editor::HandleSelectionGizmos()
{
    if (!show_grid)
        return;

    ImGuiIO& io = ImGui::GetIO();

    ImGui::Begin("pick", NULL, io.DisplaySize, 0, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus);

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    glm::mat4 p = glm::perspective(glm::radians(45.0f), (float)(display_w - 360) / (display_h - 20), 0.1f, 1000.0f);
    glm::vec3 camTarget = scene->camTarget;
    glm::vec3 dir = glm::vec3(glm::cos(scene->camOrbitY), 0, glm::sin(scene->camOrbitY));
    dir = dir * glm::cos(scene->camOrbitX);
    dir.y = glm::sin(scene->camOrbitX);
    glm::vec3 camPos = camTarget + dir * scene->camDist;
    glm::mat4 v = glm::lookAt(camPos, camTarget, glm::vec3(0, 1, 0));

    int miss_count = 0;

    for (int i = 0; i < scene->sceneObjects.size(); i++)
    {
        if (!scene->sceneObjects[i]->m_is_pickable)
            continue;

        glm::mat4 t = glm::translate(glm::mat4(1.0f), scene->sceneObjects[i]->pos);
        glm::vec3 rot = glm::radians(scene->sceneObjects[i]->rot);
        glm::mat4 r = glm::eulerAngleYXZ(rot.y, rot.x, rot.z);
        glm::mat4 mvp = p * v * t;

        float x = (display_w - 360) * (mvp[3][0] / mvp[3][3] + 1.0f) * 0.5f;
        float y = display_h - (display_h - 20) * (mvp[3][1] / mvp[3][3] + 1.0f) * 0.5f;

        //std::cout << x << " "  << y << std::endl;

        glm::vec2 mouse(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y);
        glm::vec2 point(x, y);

        glm::vec4 color = scene->sceneObjects[i]->color;

        if (glm::distance(mouse, point) < 10 && !ImGuizmo::IsOver())
        {
            //color |= 0xFF000000;
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(x - 5, y - 5), ImVec2(x + 5, y + 5), ImColor(color.r, color.g, color.b, 1.0f), 2);

            //ImGui::GetWindowDrawList()->AddCircle(ImVec2(x, y), 5, ImColor(color.r, color.g, color.b, 1.0f), 12, 6);

            if (ImGui::IsMouseClicked(0))
            {
                selected_scene_object = scene->sceneObjects[i];
            }
        }
        else
        {
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(x - 5, y - 5), ImVec2(x + 5, y + 5), ImColor(color.r, color.g, color.b, 0.6f), 2);
        }

        //if (glm::distance(mouse, point) > 5 && !ImGuizmo::IsOver() && ImGui::IsMouseClicked(0))
        //	miss_count++;

        //if (ImGui::IsMouseClicked(0) && miss_count == scene->sceneObjects.size())
        //	selected_scene_object = NULL;

        bool sth_is_selected = false;
    }

    if (ImGui::IsMouseClicked(1))
        selected_scene_object = NULL;

    ImGui::End();
}

void Editor::HandleTransformGizmos()
{
    if (!show_grid)
        return;

    static bool can_use_gizmo = false;

    if (ImGui::GetIO().MouseClicked[0] && ImGuizmo::IsOver())
        can_use_gizmo = true;

    glm::mat4 m = glm::mat4(1.0f);
    if (selected_scene_object)
    {
        ImGuizmo::Enable(true);

        m[3] = glm::vec4(selected_scene_object->pos, 1);

        float *snap = ImGui::GetIO().KeysDown[GLFW_KEY_LEFT_CONTROL] ? new float[3]{ 1,1,1 } : NULL;

        ImGuizmo::Manipulate(&graphics->v[0][0], &graphics->p[0][0], ImGuizmo::OPERATION::TRANSLATE, ImGuizmo::MODE::WORLD, &m[0][0], 0, snap);

        delete[] snap;

        if (can_use_gizmo)
        {
            selected_scene_object->pos = glm::vec3(m[3]);
        }
    }
    else
    {
        ImGuizmo::Enable(false);
    }

    if (!ImGuizmo::IsUsing())
        can_use_gizmo = false;
}

Editor *Editor::instance = nullptr;
