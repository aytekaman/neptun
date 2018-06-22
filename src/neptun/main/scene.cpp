#include "scene.h"

#include "asset_importer.h"
#include "bvh.h"
#include "kd_tree.h"
#include "light.h"
#include "mesh.h"
#include "tet_mesh.h"
#include "utils.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <ctime>

Scene::Scene() : camOrbitX(0), camOrbitY(glm::pi<float>() / 2.0f), camDist(20)
{
    for (int i = 0; i < 1; i++)
    {
        SceneObject *light = new SceneObject("Light");
        light->light = new Light();
        add_scene_object(light); 
        //light->pos = glm::sphericalRand(32.0f);
        //light->pos.y = glm::abs(light->pos.y);
        light->pos = glm::vec3(6, 10, 8);
        light->light->intensity = 1.0f;
    }
}

Scene::~Scene()
{
}

void Scene::load_from_file(const std::string& scene_file_path)
{
    clear();

    std::ifstream ifs(scene_file_path);

    std::string line;

    std::string tet_mesh_file_name;
    ifs >> tet_mesh_file_name;

    tet_mesh_file_path =
        scene_file_path.substr(0, scene_file_path.find_last_of("\\") + 1) + tet_mesh_file_name;

    ifs >> camOrbitX >> camOrbitY >> camDist;
    ifs >> camTarget.x >> camTarget.y >> camTarget.z;

    int scene_object_count;
    ifs >> scene_object_count;

    //std::cout << "num: " << scene_object_count << std::endl;

    for (int i = 0; i < scene_object_count; i++)
    {
        std::string scene_object_name;
        
        ifs >> scene_object_name;

        std::replace(scene_object_name.begin(), scene_object_name.end(), '#', ' '); // replace all 'x' to 'y'

        SceneObject* scene_object = new SceneObject(scene_object_name);

        //ifs >> line;

        //std::cout << line << std::endl;

        ifs >> scene_object->pos.x >> scene_object->pos.y >> scene_object->pos.z;
        ifs >> scene_object->rot.x >> scene_object->rot.y >> scene_object->rot.z;
        ifs >> scene_object->scale.x >> scene_object->scale.y >> scene_object->scale.z;

        ifs >> line;

        if (line != "#") // Light
        {
            Light* light = new Light();
            ifs >> light->color.r >> light->color.g >> light->color.b;
            ifs >> light->intensity;

            scene_object->light = light;
        }

        ifs >> line;

        if (line != "#") // Material
        {
            Material* material = new Material();
            ifs >> material->diffuse.r >> material->diffuse.g >> material->diffuse.b;

            scene_object->material = material;
        }

        ifs >> line;

        if (line != "#") // Mesh
        {
            std::string mesh_file_path;

            ifs >> mesh_file_path;

            std::string absolute_mesh_file_path = 
                scene_file_path.substr(0, scene_file_path.find_last_of("\\") + 1) + mesh_file_path;

            Mesh* mesh = AssetImporter::ImportMesh(absolute_mesh_file_path.c_str());

            scene_object->mesh = mesh;
        }

        sceneObjects.push_back(scene_object);
    }

    ifs.close();
}

void Scene::save_to_file(const std::string & file_name)
{
    std::string inputfile = AssetImporter::assets_folder_path;
    inputfile.append(file_name);
    std::ofstream o(inputfile);

    o << "MyScene" << std::endl;
    o << camOrbitX << " " << camOrbitY << " " << camDist << std::endl;
    o << camTarget.x << " " << camTarget.y << " " << camTarget.z << std::endl;

    o << sceneObjects.size() << std::endl;

    for (const SceneObject* scene_object : sceneObjects)
    {
        std::string name = scene_object->name;
        std::replace(name.begin(), name.end(), ' ', '#'); // replace all 'x' to 'y'

        o << name << std::endl;
        o << scene_object->pos.x << " " << scene_object->pos.y << " " << scene_object->pos.z << std::endl;
        o << scene_object->rot.x << " " << scene_object->rot.y << " " << scene_object->rot.z << std::endl;
        o << scene_object->scale.x << " " << scene_object->scale.y << " " << scene_object->scale.z << std::endl;

        if (scene_object->light)
        {
            o << "#Light" << std::endl;
            glm::vec3 color = scene_object->light->color;
            o << color.r << " " << color.g << " " << color.b << std::endl;
            o << scene_object->light->intensity << std::endl;
        }
        else
            o << "#" << std::endl;

        if (scene_object->material)
        {
            o << "#Material" << std::endl;
            glm::vec3 diffuse = scene_object->material->diffuse;
            o << diffuse.r << " " << diffuse.g << " " << diffuse.b << std::endl;
        }
        else
            o << "#" << std::endl;

        if (scene_object->mesh)
        {
            o << "#Mesh" << std::endl;
            o << scene_object->mesh->m_file_name << std::endl;
        }
        else
            o << "#" << std::endl;
    }

    o.close();
}

void Scene::save_to_file()
{
    save_to_file(Utils::get_timestamp() + ".scene");
}

void Scene::add_scene_object(SceneObject *sceneObject)
{
    sceneObjects.push_back(sceneObject);
}

void Scene::clear()
{
    for (const SceneObject* scene_object : sceneObjects)
        delete scene_object;

    sceneObjects.clear();
}

bool Scene::has_accelerator() const
{
    return kd_tree || bvh || bvh_embree || tet_mesh;
}

int Scene::get_triangle_count(bool ignore_hidden_scene_objects)
{
    int triangle_count = 0;

    for (SceneObject* scene_object : sceneObjects)
    {
        if (ignore_hidden_scene_objects && scene_object->m_hide_in_editor)
            continue;

        if (scene_object->mesh)
            triangle_count += scene_object->mesh->m_face_count;
    }

    return triangle_count;
}

void Scene::build_bvh(int maxPrimsInNode, SplitMethod splitMethod)
{
    if (bvh)
        delete bvh;

    bvh = new Bvh(*this, maxPrimsInNode, splitMethod);
}

void Scene::build_bvh_embree()
{
    if (bvh_embree)
        delete bvh_embree;

    bvh_embree = new BvhEmbree(*this);

    std::cout << "embree tree is built." << std::endl;
}

void Scene::build_kd_tree(int isectCost, int traversalCost, float emptyBonus, int maxPrims, int maxDepth)
{
    if (kd_tree)
        delete kd_tree;

    kd_tree = new KdTree(*this, isectCost, traversalCost, emptyBonus, maxPrims, maxDepth);
}

void Scene::build_tet_mesh(bool preserve_triangles, bool create_bounding_box, float quality)
{
    if (tet_mesh)
        delete tet_mesh;

    tet_mesh = new TetMesh32(*this, preserve_triangles, create_bounding_box, quality);
}
