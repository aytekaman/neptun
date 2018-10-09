#pragma once

#include "neptun/math/matrix.h"

#include <atomic>
#include <vector>

#include "glm/glm.hpp"

class Bvh;
class Image;
class KdTree;
class Scene;
struct SourceTet;
class TetMesh;
class Texture;

enum Method
{
    Default,
    DefaultSimd,
    ScTP,
    Fast_basis,
    Kd_tree,
    BVH_pbrt,
    BVH_embree,
    Method_count
};

//enum class AcceleratorType
//{
//    TetMesh32,
//    TetMesh32,
//    TetMesh20,
//    TetMesh16a,
//    TetMesh16b,
//    Kd_tree,
//    BVH,
//    AcceleratorTypeCount
//};

enum class TraversalBasis
{
    Default,
    Optimized,
    TraversalBasisCount
};

//enum class AcceleratorType
//{
//    TetMesh32,
//    TetMesh32,
//    TetMesh20,
//    TetMesh16a,
//    TetMesh16b,
//    Kd_tree,
//    BVH,
//    AcceleratorTypeCount
//};

struct LightInfo
{
    glm::vec3 pos;
    glm::vec3 color;
    int tet_index;
    int point_index;
    float intensity;
};

enum ImageType
{
    Render,
    Tet_cost,
    Locality
};

class RayTracer
{
public:
    RayTracer();

    void Render(Scene &scene, const bool is_diagnostic = false);

    void Raytrace_worker(
        Scene &scene,
        SourceTet source_tet,
        int thread_idx,
        std::vector<LightInfo> lightInfos,
        bool is_diagnostic);


    void save_to_disk(const char* file_name, ImageType image_type = ImageType::Render);
    void set_resoultion(const glm::ivec2& resolution);


    bool multi_threading = false;
    bool shadows = false;

    // stats
    float last_render_time = 0;
    float avg_test_count;
    int L1_count;
    float traversed_tetra_count[32] = { 0 };
    int L1_hit_count[32] = { 0 };

    Image* m_visited_tets_image;
    Image* m_locality_image;
    Image* m_rendered_image;

    Matrix<int> stats;

    std::atomic<int> job_index{0};
    glm::ivec2 m_resolution = glm::ivec2(640, 480);
    int tile_size = 16;
    //glm::u8vec3 *pixels;
    int thread_count;

    //KdTree *kd_tree = nullptr;
    //Bvh *bvh = nullptr;

    Method method = Method::Default;
};
