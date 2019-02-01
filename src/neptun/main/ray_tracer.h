#pragma once

#include "neptun/math/matrix.h"

#include <atomic>
#include <vector>

#include "glm/glm.hpp"
#include "ray.h"
#include "tet_mesh.h"

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

extern void Traverse_rays_gpu(Ray* rays, unsigned int rays_size, unsigned int tet_mesh_type, IntersectionData* output);
extern void Traverse_rays_gpu(Ray* rays, unsigned int rays_size, int num_streams, unsigned int tet_mesh_type, int idt, IntersectionData* output);
extern void Traverse_rays_gpu(Scene & scene, SourceTet source_tet, glm::ivec2 resolution, unsigned int rays_size, unsigned int tet_mesh_type, IntersectionData* output);

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

    void Raytrace_worker_gpu(
        Scene & scene,
        SourceTet source_tet,
        int thread_idx,
        std::vector<LightInfo> lightInfos,
        bool is_diagnostic);

    void Prepare_rays_gpu(Scene & scene,
        SourceTet source_tet,
        int thread_idx,
        bool is_diagnostic);

    void Draw_intersectiondata(int thread_idx, std::vector<LightInfo> lightInfos);
    void Draw_intersectiondata(int set_start, int set_end, std::vector<LightInfo> lightInfos);

    void Render_gpu(Scene & scene, const bool is_diagnostic = false);

    void Ray_caster(Scene & scene, std::vector<Ray> rays, std::vector<IntersectionData>& output);

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

    std::atomic<int> job_index{ 0 };
    glm::ivec2 m_resolution = glm::ivec2(640, 480);
    glm::ivec2 m_old_res = glm::ivec2();
    IntersectionData* m_intersect_data;
    Ray* m_rays;
    int tile_size = 8;
    //glm::u8vec3 *pixels;
    int thread_count;
    int m_stream_count;

    //KdTree *kd_tree = nullptr;
    //Bvh *bvh = nullptr;

    Method method = Method::Default;
};
