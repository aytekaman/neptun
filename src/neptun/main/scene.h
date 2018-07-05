#pragma once

#include "sceneobject.h"

#include <iostream>
#include <fstream>
#include <vector>

class Bvh;
class BvhEmbree;

enum class SplitMethod;
class KdTree;
class TetMesh;

class Scene
{
public:
    Scene();
    ~Scene();

    void load_from_file(const std::string& file_name);
    void save_to_file(const std::string& file_name);
    void save_to_file();

    void add_scene_object(SceneObject *sceneObject);
    void clear();

    bool has_accelerator() const;

    int  get_triangle_count(bool ignore_hidden_scene_objects = true);

    void build_bvh(int maxPrimsInNode = 1, SplitMethod splitMethod = (SplitMethod)0);
    void build_bvh_embree();
    void build_kd_tree(int isectCost = 80, int traversalCost = 1, float emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);
    void build_tet_mesh(bool preserve_triangles, bool create_bounding_box, float quality = 5.0f);

    // Scene objects.
    std::vector<SceneObject*> sceneObjects;

    // Camera params
    glm::vec3 camTarget;
    float camOrbitX, camOrbitY, camDist;

    std::string name;
    std::string tet_mesh_file_path;

    // Accelerators.
    Bvh*        bvh         = nullptr;
    BvhEmbree*  bvh_embree  = nullptr;
    KdTree*     kd_tree     = nullptr;
    TetMesh*    tet_mesh    = nullptr;
};