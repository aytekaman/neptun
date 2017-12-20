#pragma once

#include <vector>


#include "sceneObject.h"

#include <iostream>
#include <fstream>

class Bvh;

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

    int  get_triangle_count(bool ignore_hidden_scene_objects = true);

    void build_bvh(int maxPrimsInNode = 1, SplitMethod splitMethod = (SplitMethod)0);
    void build_kd_tree(int isectCost = 80, int traversalCost = 1, float emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);
    void build_tet_mesh(bool preserve_triangles, bool create_bounding_box, float quality = 5.0f);

    // Scene objects.
	std::vector<SceneObject*> sceneObjects;

	// Camera params
	glm::vec3 camTarget;
	float camOrbitX, camOrbitY, camDist;

    std::string name;
    std::string tet_mesh_file_name;

    // Accelerators.
    Bvh*     bvh      = nullptr;
    KdTree*  kd_tree  = nullptr;
    TetMesh* tet_mesh = nullptr;
    TetMesh* tet_mesh20 = nullptr;
    TetMesh* tet_mesh32 = nullptr;
};