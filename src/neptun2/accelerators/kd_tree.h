#pragma once

#include <neptun2/accelerators/ray.h>
#include <neptun2/accelerators/accelerator.h>
#include <neptun2/math/bounds.h>

#include <vector>

namespace neptun
{

struct KdNode
{
    void InitLeaf(int* primNums, int np, std::vector<int>* primitiveIndices);
    void InitInterior(int axis, int ac, float s) 
    {
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
    union
    {
        int flags;       // Both
        int nPrims;      // Leaf
        int aboveChild;  // Interior
    };
};

enum class EdgeType { Start, End };
struct BoundEdge
{
    // BoundEdge Public Methods
    BoundEdge() {}
    BoundEdge(float t, int primNum, bool starting) : t(t), primNum(primNum)
    {
        type = starting ? EdgeType::Start : EdgeType::End;
    }
    float t;
    int primNum;
    EdgeType type;
};

struct KdToDo
{
    const KdNode* node;
    float tMin, tMax;
};

class KdTree final : public Accelerator
{
public:
    KdTree() = default;
    ~KdTree() override;

    virtual bool build(const Triangle * primitives, size_t primitive_count) override final;
    virtual void intersect1(RayHit & ray_hit) const override final;
    virtual const char* name() const override final;
    virtual size_t get_size_in_bytes() const override final;

    // KdTree Private Methods
    
    void buildTree(int nodeNum, const Bounds3& bounds,
                   const std::vector<Bounds3>& primBounds, int* primNums,
                   int nprims, int depth,
                   BoundEdge* edges[3], int* prims0,
                   int* prims1, int badRefines = 0);


    // KdTree Private Data
    int isectCost = 80;
    int traversalCost = 1;
    int maxPrims = 1;
    const float emptyBonus = 0.5f;
    int maxDepth = -1;

    std::vector<Bounds3> primBounds, leafBounds;

    std::vector<int> primitiveIndices;
    
    int nAllocedNodes, nextFreeNode;
    Bounds3 bounds;

    KdNode* nodes;
    std::vector<Triangle> m_primitives;
};

inline int Log2Int(uint32_t v) 
{
#if defined(_MSC_VER)
    unsigned long lz = 0;
    if (_BitScanReverse(&lz, v)) return lz;
    return 0;
#else
    return 31 - __builtin_clz(v);
#endif
}

} // end of namespace neptun