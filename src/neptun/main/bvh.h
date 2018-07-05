#pragma once

#include <atomic>

#include "accelerator.h"
#include "bounds.h"
#include "face.h"
#include "memory.h"

#include "embree3/rtcore.h"

#include <memory>
#include <vector>

class Scene;

struct BVHBuildNode;

// Bvh Forward Declarations
struct BVHPrimitiveInfo;
struct MortonPrimitive;
struct LinearBVHNode;

enum class SplitMethod { SAH, HLBVH, Middle, EqualCounts };
// Bvh Declarations
class Bvh{
public:
    // Bvh Public Types
    

    // Bvh Public Methods
    Bvh(Scene& scene,
        int maxPrimsInNode = 1,
        SplitMethod splitMethod = SplitMethod::SAH);
    Bounds3 WorldBound() const;
    ~Bvh();
    bool Intersect(const Ray &ray, IntersectionData& intersection_data) const;
    bool Intersect_stats(const Ray &ray, IntersectionData& intersection_data, DiagnosticData& diagnostic_data) const;
    //bool IntersectP(const Ray &ray) const;

    void initFaces();
    // Bvh Private Methods
    BVHBuildNode *recursiveBuild(
        MemoryArena &arena, std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int start, int end, int *totalNodes,
        std::vector<Face> &orderedPrims);
    BVHBuildNode *HLBVHBuild(
        MemoryArena &arena, const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int *totalNodes,
        std::vector<Face> &orderedPrims) const;
    BVHBuildNode *emitLBVH(
        BVHBuildNode *&buildNodes,
        const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
        std::vector<Face> &orderedPrims,
        std::atomic<int> *orderedPrimsOffset, int bitIndex) const;
    BVHBuildNode *buildUpperSAH(MemoryArena &arena,
        std::vector<BVHBuildNode *> &treeletRoots,
        int start, int end, int *totalNodes) const;
    int flattenBVHTree(BVHBuildNode *node, int *offset);

    std::vector<Bounds3> primBounds, leafBounds;
    Scene* scene_ptr;
    std::vector<Face> faces;

    // Bvh Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    //std::vector<Face> faces;
    LinearBVHNode *nodes = nullptr;
};

class BvhEmbree
{
public:
    BvhEmbree(Scene& scene);
    ~BvhEmbree();

    void initFaces();

    bool Intersect(const Ray &ray, IntersectionData& intersection_data) const;

    RTCScene rtc_scene;

    std::vector<Face> faces;
    Scene* scene_ptr;
};

//std::shared_ptr<Bvh> CreateBvherator(
//    const std::vector<std::shared_ptr<Primitive>> &prims, const ParamSet &ps);