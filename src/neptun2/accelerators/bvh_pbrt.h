#pragma once

#include <neptun2/accelerators/accelerator.h>
#include <neptun2/math/bounds.h>

#include <atomic>

namespace neptun
{

struct MemoryArena;

class BvhPbrt final : public Accelerator
{
public:
    // Common interface
    ~BvhPbrt() override;
    virtual bool build(const Triangle* primitives, size_t primitive_count) override final;
    virtual void intersect1(RayHit& ray_hit) const override final;
    virtual const char* name() const override final;
    virtual size_t get_size_in_bytes() const override final;

    struct BVHPrimitiveInfo;
    struct MortonPrimitive;
    struct LinearBVHNode;
    struct BVHBuildNode;

    // Bvh Parameters
    enum class SplitMethod { SAH, HLBVH, Middle, EqualCounts };

    SplitMethod m_split_method = SplitMethod::SAH;
    int m_max_prims_in_node = 1;

    std::vector<Bounds3> m_prim_bounds, m_leaf_bounds;
    LinearBVHNode* nodes = nullptr;
    std::vector<Triangle> m_primitives;

private:
    BVHBuildNode* recursive_build(
        MemoryArena& arena,
        std::vector<BVHPrimitiveInfo>& primitive_info,
        int start,
        int end,
        int* total_nodes,
        std::vector<Triangle>& ordered_prims);

    BVHBuildNode* HLBVH_build(
        MemoryArena& arena,
        const std::vector<BVHPrimitiveInfo>& primitive_info,
        int* total_nodes,
        std::vector<Triangle>& ordered_prims) const;

    BVHBuildNode* emit_LBVH(
        BVHBuildNode*& build_nodes,
        const std::vector<BVHPrimitiveInfo>& primitive_info,
        MortonPrimitive* morton_prims,
        int num_primitives,
        int* total_nodes,
        std::vector<Triangle>& ordered_prims,
        std::atomic<int>* ordered_prims_offset,
        int bit_index) const;

    BVHBuildNode* build_upper_SAH(
        MemoryArena& arena,
        std::vector<BVHBuildNode*>& treelet_roots,
        int start,
        int end,
        int* total_nodes) const;

    int flatten_BVH_tree(BVHBuildNode* node, int* offset);
};

} // end of namespace neptun
