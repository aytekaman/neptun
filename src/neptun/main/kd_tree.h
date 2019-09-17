
#pragma once

#include "accelerator.h"
#include "bounds.h"
#include "face.h"
#include "scene.h"

#include <vector>

struct KdNode
{
    void InitLeaf(int *primNums, int np, std::vector<int> *primitiveIndices);
    void InitInterior(int axis, int ac, float s) {
        split = s;
        flags = axis;
        aboveChild |= (ac << 2);
    }
    float SplitPos() const { return split; }
    int nPrimitives() const { return nPrims >> 2; }
    int SplitAxis() const { return flags & 3; }
    bool IsLeaf() const { return (flags & 3) == 3; }
    int AboveChild() const { return aboveChild >> 2; }
    union {
        float split;                 // Interior
        int onePrimitive;            // Leaf
        int primitiveIndicesOffset;  // Leaf
    };

private:
    union {
        int flags;       // Both
        int nPrims;      // Leaf
        int aboveChild;  // Interior
    };
};

enum class EdgeType { Start, End };
struct BoundEdge {
    // BoundEdge Public Methods
    BoundEdge() {}
    BoundEdge(float t, int primNum, bool starting) : t(t), primNum(primNum) {
        type = starting ? EdgeType::Start : EdgeType::End;
    }
    float t;
    int primNum;
    EdgeType type;
};

struct KdToDo {
    const KdNode *node;
    float tMin, tMax;
};

class KdTree : public Accelerator
{
public:
    KdTree(Scene& scene,
        int isectCost = 80, int traversalCost = 1,
        float emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);

	~KdTree();

    Bounds3 WorldBound() const { return bounds; }

    bool Intersect(const Ray &ray, IntersectionData& intersection_data) const;
    bool Intersect_stats(const Ray &ray, IntersectionData& intersection_data, DiagnosticData& diagnostic_data) const;

    //bool IntersectP(const Ray &ray) const;


    void initFaces();
    // KdTree Private Methods
    void buildTree(int nodeNum, const Bounds3 &bounds,
        const std::vector<Bounds3> &primBounds, int *primNums,
        int nprims, int depth,
        BoundEdge* edges[3], int *prims0,
        int *prims1, int badRefines = 0);

    int get_size_in_bytes();

    // KdTree Private Data
    const int isectCost, traversalCost, maxPrims;
    const float emptyBonus;
    //std::vector<std::shared_ptr<Primitive>> primitives;

    std::vector<Bounds3> primBounds, leafBounds;
    Scene* scene_ptr;
    std::vector<Face> faces;

    std::vector<int> primitiveIndices;
    KdNode *nodes;
    int nAllocedNodes, nextFreeNode;
    Bounds3 bounds;
};

inline int Log2Int(uint32_t v) {
#if defined(_MSC_VER)
    unsigned long lz = 0;
    if (_BitScanReverse(&lz, v)) return lz;
    return 0;
#else
    return 31 - __builtin_clz(v);
#endif
}
