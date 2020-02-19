#include "bvh_pbrt.h"

#include <neptun2/util/memory.h>

#include <glm/gtx/intersect.hpp>

namespace neptun
{

struct BvhPbrt::BVHPrimitiveInfo 
{
    BVHPrimitiveInfo() = default;
    BVHPrimitiveInfo(size_t primitive_number, const Bounds3& bounds)
        : m_primitive_number(primitive_number)
        , m_bounds(bounds)
        , m_centroid(.5f * bounds.m_min_p + .5f * bounds.m_max_p) 
    {
    }

    size_t    m_primitive_number;
    Bounds3   m_bounds;
    glm::vec3 m_centroid;
};

struct BvhPbrt::BVHBuildNode
{
    // BVHBuildNode Public Methods
    void init_leaf(int first, int n, const Bounds3& b) 
    {
        m_first_prim_offset = first;
        m_num_primitives = n;
        m_bounds = b;
        m_children[0] = m_children[1] = nullptr;
    }

    void init_interior(int axis, BVHBuildNode* c0, BVHBuildNode* c1)
    {
        m_children[0] = c0;
        m_children[1] = c1;
        m_bounds = Bounds3::add(c0->m_bounds, c1->m_bounds);
        m_split_axis = axis;
        m_num_primitives = 0;
    }

    Bounds3 m_bounds;
    BVHBuildNode* m_children[2];
    int m_split_axis, m_first_prim_offset, m_num_primitives;
};

struct BvhPbrt::MortonPrimitive
{
    int m_primitive_index;
    uint32_t m_morton_code;
};

struct LBVHTreelet
{
    int m_start_index, m_num_primitives;
    BvhPbrt::BVHBuildNode* m_build_nodes;
};

struct BvhPbrt::LinearBVHNode
{
    Bounds3 m_bounds;

    union
    {
        int m_primitives_offset;   // leaf
        int m_second_child_offset;  // interior
    };

    uint16_t m_num_primitives;  // 0 -> interior node
    uint8_t m_axis;          // interior node: xyz
    uint8_t m_pad[1];        // ensure 32 byte total size
};

// Bvh Utility Functions
inline uint32_t left_shift3(uint32_t x)
{
    if (x == (1 << 10)) --x;

    x = (x | (x << 16)) & 0x30000ff;
    // x = ---- --98 ---- ---- ---- ---- 7654 3210
    x = (x | (x << 8)) & 0x300f00f;
    // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    x = (x | (x << 4)) & 0x30c30c3;
    // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    x = (x | (x << 2)) & 0x9249249;
    // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    
    return x;
}

inline uint32_t encode_morton3(const glm::vec3& v)
{
    return (left_shift3(v.z) << 2) | (left_shift3(v.y) << 1) | left_shift3(v.x);
}

static void radix_sort(std::vector<BvhPbrt::MortonPrimitive>* v)
{
    constexpr int bits_per_pass = 6;
    constexpr int num_bits = 30;
    static_assert((num_bits % bits_per_pass) == 0,
                  "Radix sort bitsPerPass must evenly divide nBits");
    constexpr int num_passes = num_bits / bits_per_pass;

    std::vector<BvhPbrt::MortonPrimitive> temp_vector(v->size());

    for (int pass = 0; pass < num_passes; ++pass)
    {
        // Perform one pass of radix sort, sorting _bitsPerPass_ bits
        int low_bit = pass * bits_per_pass;

        // Set in and out vector pointers for radix sort pass
        std::vector<BvhPbrt::MortonPrimitive>& in = (pass & 1) ? temp_vector : *v;
        std::vector<BvhPbrt::MortonPrimitive>& out = (pass & 1) ? *v : temp_vector;

        // Count number of zero bits in array for current radix sort bit
        constexpr int num_buckets = 1 << bits_per_pass;
        int bucket_count[num_buckets] = { 0 };
        constexpr int bit_mask = (1 << bits_per_pass) - 1;

        for (const BvhPbrt::MortonPrimitive& mp : in)
        {
            int bucket = (mp.m_morton_code >> low_bit) & bit_mask;
            ++bucket_count[bucket];
        }

        // Compute starting index in output array for each bucket
        int outIndex[num_buckets];
        outIndex[0] = 0;
        for (int i = 1; i < num_buckets; ++i)
            outIndex[i] = outIndex[i - 1] + bucket_count[i - 1];

        // Store sorted values in output array
        for (const BvhPbrt::MortonPrimitive& mp : in)
        {
            int bucket = (mp.m_morton_code >> low_bit) & bit_mask;
            out[outIndex[bucket]++] = mp;
        }
    }
    // Copy final result from _tempVector_, if needed
    if (num_passes & 1) std::swap(*v, temp_vector);
}

// BvhPbrt Helper member functions
BvhPbrt::BVHBuildNode* BvhPbrt::recursive_build(
    MemoryArena& arena,
    std::vector<BVHPrimitiveInfo>& primitive_info,
    int start,
    int end,
    int* total_nodes,
    std::vector<Triangle>& ordered_prims)
{
    BVHBuildNode* node = arena.Alloc<BVHBuildNode>();
    (*total_nodes)++;

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = start; i < end; ++i)
        bounds.add(primitive_info[i].m_bounds);

    int num_primitives = end - start;
    if (num_primitives == 1)
    {
        // Create leaf _BVHBuildNode_
        const int first_prim_offset = ordered_prims.size();

        for (int i = start; i < end; ++i) 
        {
            const int prim_num= primitive_info[i].m_primitive_number;
            ordered_prims.push_back(m_primitives[prim_num]);
        }

        node->init_leaf(first_prim_offset, num_primitives, bounds);
        
        return node;
    }
    else // num_primitives > 1
    {
        // Compute bound of primitive centroids, choose split dimension _dim_
        Bounds3 centroid_bounds;
        for (int i = start; i < end; ++i)
            centroid_bounds.add(primitive_info[i].m_centroid);
        const int dim = centroid_bounds.max_extent();

        // Partition primitives into two sets and build children
        int mid = (start + end) / 2;
        if (centroid_bounds.m_max_p[dim] == centroid_bounds.m_min_p[dim])
        {
            // Create leaf _BVHBuildNode_
            const int first_prim_offset = ordered_prims.size();

            for (int i = start; i < end; ++i)
            {
                const int prim_num = primitive_info[i].m_primitive_number;
                ordered_prims.push_back(m_primitives[prim_num]);
            }

            node->init_leaf(first_prim_offset, num_primitives, bounds);

            return node;
        }
        else // bounds is not empty
        {
            // Partition primitives based on _splitMethod_
            switch (m_split_method) 
            {
            case SplitMethod::Middle: {
                // Partition primitives through node's midpoint
                float pmid =
                    (centroid_bounds.m_min_p[dim] + centroid_bounds.m_max_p[dim]) / 2;
                
                BVHPrimitiveInfo* mid_ptr = std::partition(
                    &primitive_info[start], &primitive_info[end - 1] + 1,
                    [dim, pmid](const BVHPrimitiveInfo& pi) {
                        return pi.m_centroid[dim] < pmid;
                    });

                mid = mid_ptr - &primitive_info[0];

                // For lots of prims with large overlapping bounding boxes, this
                // may fail to partition; in that case don't break and fall
                // through
                // to EqualCounts.
                if (mid != start && mid != end) break;
            }
            case SplitMethod::EqualCounts:
            {
                // Partition primitives into equally-sized subsets
                mid = (start + end) / 2;
                std::nth_element(&primitive_info[start], &primitive_info[mid],
                                 &primitive_info[end - 1] + 1,
                                 [dim](const BVHPrimitiveInfo& a,
                                 const BVHPrimitiveInfo& b) {
                                     return a.m_centroid[dim] < b.m_centroid[dim];
                                 });
                break;
            }
            case SplitMethod::SAH:
            default: 
            {
                // Partition primitives using approximate SAH
                if (num_primitives <= 2)
                {
                    // Partition primitives into equally-sized subsets
                    mid = (start + end) / 2;
                    std::nth_element(&primitive_info[start], &primitive_info[mid],
                                     &primitive_info[end - 1] + 1,
                                     [dim](const BVHPrimitiveInfo& a,
                                     const BVHPrimitiveInfo& b) {
                                         return a.m_centroid[dim] <
                                             b.m_centroid[dim];
                                     });
                }
                else // num_primitives > 2
                {
                    // Allocate _BucketInfo_ for SAH partition buckets
                    struct BucketInfo {
                        int count = 0;
                        Bounds3 bounds;
                    };

                    constexpr int num_buckets = 12;
                    BucketInfo buckets[num_buckets];

                    // Initialize _BucketInfo_ for SAH partition buckets
                    for (int i = start; i < end; ++i)
                    {
                        int b = num_buckets * centroid_bounds.offset(primitive_info[i].m_centroid)[dim];
                        if (b == num_buckets) b = num_buckets - 1;
                        buckets[b].count++;
                        buckets[b].bounds.add(primitive_info[i].m_bounds);
                    }

                    // Compute costs for splitting after each bucket
                    float cost[num_buckets - 1];
                    for (int i = 0; i < num_buckets - 1; ++i)
                    {
                        Bounds3 b0, b1;
                        int count0 = 0, count1 = 0;
                        for (int j = 0; j <= i; ++j)
                        {
                            b0.add(buckets[j].bounds);
                            count0 += buckets[j].count;
                        }

                        for (int j = i + 1; j < num_buckets; ++j)
                        {
                            b1.add(buckets[j].bounds);
                            count1 += buckets[j].count;
                        }

                        cost[i] = 1 +
                            (count0 * b0.surface_area() +
                             count1 * b1.surface_area()) /
                            bounds.surface_area();
                    }

                    // Find bucket to split at that minimizes SAH metric
                    float min_cost = cost[0];
                    int min_cost_split_bucket = 0;
                    for (int i = 1; i < num_buckets - 1; ++i)
                    {
                        if (cost[i] < min_cost)
                        {
                            min_cost = cost[i];
                            min_cost_split_bucket = i;
                        }
                    }

                    // Either create leaf or split primitives at selected SAH
                    // bucket
                    float leaf_cost = num_primitives;

                    if (num_primitives > m_max_prims_in_node || min_cost < leaf_cost)
                    {
                        BVHPrimitiveInfo* pmid = std::partition(
                            &primitive_info[start], &primitive_info[end - 1] + 1,
                            [=](const BVHPrimitiveInfo& pi)
                            {
                                int b = num_buckets * centroid_bounds.offset(pi.m_centroid)[dim];
                                if (b == num_buckets) b = num_buckets - 1;
                                return b <= min_cost_split_bucket;
                            });
                        mid = pmid - &primitive_info[0];
                    }
                    else 
                    {
                        // Create leaf _BVHBuildNode_
                        int first_prim_offset = ordered_prims.size();
                        for (int i = start; i < end; ++i)
                        {
                            const int prim_num = primitive_info[i].m_primitive_number;
                            ordered_prims.push_back(m_primitives[prim_num]);
                        }

                        node->init_leaf(first_prim_offset, num_primitives, bounds);
                        
                        return node;
                    }
                }
                break;
            }
            }

            node->init_interior(dim,
                               recursive_build(arena, primitive_info, start, mid,
                               total_nodes, ordered_prims),
                               recursive_build(arena, primitive_info, mid, end,
                               total_nodes, ordered_prims));
        }
    }
    return node;
}

BvhPbrt::BVHBuildNode* BvhPbrt::HLBVH_build(
    MemoryArena& arena,
    const std::vector<BVHPrimitiveInfo>& primitive_info,
    int* total_nodes,
    std::vector<Triangle>& ordered_prims) const
{
    // Compute bounding box of all primitive centroids
    Bounds3 bounds;
    for (const BVHPrimitiveInfo& pi : primitive_info)
        bounds.add(pi.m_centroid);

    // Compute Morton indices of primitives
    std::vector<MortonPrimitive> morton_prims(primitive_info.size());

    for (int i = 0; i < primitive_info.size(); i++)
    {
        // Initialize _mortonPrims[i]_ for _i_th primitive
        constexpr int morton_bits = 10;
        constexpr int morton_scale = 1 << morton_bits;
        morton_prims[i].m_primitive_index = primitive_info[i].m_primitive_number;
        const glm::vec3 centroid_offset = bounds.offset(primitive_info[i].m_centroid);
        morton_prims[i].m_morton_code = encode_morton3(centroid_offset * (float)morton_scale);
    }

    // Radix sort primitive Morton indices
    radix_sort(&morton_prims);

    // Create LBVH treelets at bottom of BVH

    // Find intervals of primitives for each treelet
    std::vector<LBVHTreelet> treelets_to_build;
    for (int start = 0, end = 1; end <= (int)morton_prims.size(); ++end)
    {
        uint32_t mask = 0x3ffc0000;

        if (end == (int)morton_prims.size() ||
            ((morton_prims[start].m_morton_code & mask) !=
            (morton_prims[end].m_morton_code & mask)))
        {
            // Add entry to _treeletsToBuild_ for this treelet
            int num_primitives = end - start;
            int max_bvh_nodes = 2 * num_primitives;
            BVHBuildNode* nodes = arena.Alloc<BVHBuildNode>(max_bvh_nodes, false);
            treelets_to_build.push_back({ start, num_primitives, nodes });
            start = end;
        }
    }

    // Create LBVHs for treelets in parallel
    std::atomic<int> atomic_total(0), ordered_prims_offset(0);
    ordered_prims.resize(m_primitives.size());
    for (int i = 0; i < treelets_to_build.size(); i++)
    {
        // Generate _i_th LBVH treelet
        int nodes_created = 0;
        const int first_bit_index = 29 - 12;
        LBVHTreelet& tr = treelets_to_build[i];
        tr.m_build_nodes =
            emit_LBVH(tr.m_build_nodes, primitive_info, &morton_prims[tr.m_start_index],
                     tr.m_num_primitives, &nodes_created, ordered_prims,
                     &ordered_prims_offset, first_bit_index);
        atomic_total += nodes_created;
    }
    
    *total_nodes = atomic_total;

    // Create and return SAH BVH from LBVH treelets
    std::vector<BVHBuildNode*> finished_treelets;
    finished_treelets.reserve(treelets_to_build.size());
    for (LBVHTreelet& treelet : treelets_to_build)
        finished_treelets.push_back(treelet.m_build_nodes);
    return build_upper_SAH(arena, finished_treelets, 0, finished_treelets.size(),
                         total_nodes);
}

BvhPbrt::BVHBuildNode* BvhPbrt::emit_LBVH(
    BVHBuildNode*& build_nodes,
    const std::vector<BVHPrimitiveInfo>& primitive_info,
    MortonPrimitive* morton_prims,
    int num_primitives,
    int* total_nodes,
    std::vector<Triangle>& ordered_prims,
    std::atomic<int>* ordered_prims_offset,
    int bit_index) const
{
    if (bit_index == -1 || num_primitives < m_max_prims_in_node)
    {
        // Create and return leaf node of LBVH treelet
        (*total_nodes)++;
        BVHBuildNode* node = build_nodes++;

        Bounds3 bounds;
        int first_prim_offset = ordered_prims_offset->fetch_add(num_primitives);
        for (int i = 0; i < num_primitives; ++i)
        {
            int primitive_index = morton_prims[i].m_primitive_index;
            ordered_prims[first_prim_offset + i] = m_primitives[primitive_index];
            bounds.add(primitive_info[primitive_index].m_bounds);
        }
        node->init_leaf(first_prim_offset, num_primitives, bounds);
        return node;
    }
    else 
    {
        int mask = 1 << bit_index;

        // Advance to next subtree level if there's no LBVH split for this bit
        if ((morton_prims[0].m_morton_code & mask) ==
            (morton_prims[num_primitives - 1].m_morton_code & mask))
            return emit_LBVH(build_nodes, primitive_info, morton_prims, num_primitives,
                            total_nodes, ordered_prims, ordered_prims_offset,
                            bit_index - 1);

        // Find LBVH split point for this dimension
        int search_start = 0, search_end = num_primitives - 1;
        while (search_start + 1 != search_end) 
        {
            int mid = (search_start + search_end) / 2;
            if ((morton_prims[search_start].m_morton_code & mask) ==
                (morton_prims[mid].m_morton_code & mask))
                search_start = mid;
            else 
            {
                search_end = mid;
            }
        }
        int split_offset = search_end;

        // Create and return interior LBVH node
        (*total_nodes)++;
        BVHBuildNode* node = build_nodes++;
        BVHBuildNode* lbvh[2] =
        {
            emit_LBVH(build_nodes, primitive_info, morton_prims, split_offset,
                total_nodes, ordered_prims, ordered_prims_offset, bit_index - 1),
            emit_LBVH(build_nodes, primitive_info, &morton_prims[split_offset],
                num_primitives - split_offset, total_nodes, ordered_prims,
                ordered_prims_offset, bit_index - 1)
        };

        int axis = bit_index % 3;
        node->init_interior(axis, lbvh[0], lbvh[1]);
        return node;
    }
}

BvhPbrt::BVHBuildNode* BvhPbrt::build_upper_SAH(
    MemoryArena& arena,
    std::vector<BVHBuildNode*>& treelet_roots,
    int start,
    int end,
    int* total_nodes) const
{
    int num_nodes = end - start;
    if (num_nodes == 1) return treelet_roots[start];
    (*total_nodes)++;
    BVHBuildNode* node = arena.Alloc<BVHBuildNode>();

    // Compute bounds of all nodes under this HLBVH node
    Bounds3 bounds;
    for (int i = start; i < end; ++i)
        bounds.add(treelet_roots[i]->m_bounds);

    // Compute bound of HLBVH node centroids, choose split dimension _dim_
    Bounds3 centroid_bounds;
    for (int i = start; i < end; ++i) {
        glm::vec3 centroid =
            (treelet_roots[i]->m_bounds.m_min_p + treelet_roots[i]->m_bounds.m_max_p) * 0.5f;
        centroid_bounds.add(centroid);
    }

    int dim = centroid_bounds.max_extent();
    // FIXME: if this hits, what do we need to do?
    // Make sure the SAH split below does something... ?
    //CHECK_NE(centroidBounds.max[dim], centroidBounds.min[dim]);

    // Allocate _BucketInfo_ for SAH partition buckets
    constexpr int num_buckets = 12;
    struct BucketInfo
    {
        int count = 0;
        Bounds3 bounds;
    };
    BucketInfo buckets[num_buckets];

    // Initialize _BucketInfo_ for HLBVH SAH partition buckets
    for (int i = start; i < end; ++i)
    {
        float centroid = (treelet_roots[i]->m_bounds.m_min_p[dim] +
                          treelet_roots[i]->m_bounds.m_max_p[dim]) *
            0.5f;

        int b =
            num_buckets * ((centroid - centroid_bounds.m_min_p[dim]) /
            (centroid_bounds.m_max_p[dim] - centroid_bounds.m_min_p[dim]));
        if (b == num_buckets) b = num_buckets - 1;
        buckets[b].count++;
        buckets[b].bounds.add(treelet_roots[i]->m_bounds);
    }

    // Compute costs for splitting after each bucket
    float cost[num_buckets - 1];
    for (int i = 0; i < num_buckets - 1; ++i)
    {
        Bounds3 b0, b1;
        int count0 = 0, count1 = 0;
        
        for (int j = 0; j <= i; ++j)
        {
            b0.add(buckets[j].bounds);
            count0 += buckets[j].count;
        }
        for (int j = i + 1; j < num_buckets; ++j)
        {
            b1.add(buckets[j].bounds);
            count1 += buckets[j].count;
        }
        cost[i] = .125f +
            (count0 * b0.surface_area() + count1 * b1.surface_area()) /
            bounds.surface_area();
    }

    // Find bucket to split at that minimizes SAH metric
    float min_cost = cost[0];
    int min_cost_split_bucket = 0;
    for (int i = 1; i < num_buckets - 1; ++i) {
        if (cost[i] < min_cost) {
            min_cost = cost[i];
            min_cost_split_bucket = i;
        }
    }

    // Split nodes and create interior HLBVH SAH node
    BVHBuildNode** pmid = std::partition(
        &treelet_roots[start], &treelet_roots[end - 1] + 1,
        [=](const BVHBuildNode* node) 
        {
            float centroid =
                (node->m_bounds.m_min_p[dim] + node->m_bounds.m_max_p[dim]) * 0.5f;
            int b = num_buckets *
                ((centroid - centroid_bounds.m_min_p[dim]) /
                (centroid_bounds.m_max_p[dim] - centroid_bounds.m_min_p[dim]));
            if (b == num_buckets) b = num_buckets - 1;
            return b <= min_cost_split_bucket;
        });
    int mid = pmid - &treelet_roots[0];
    
    node->init_interior(
        dim,
        this->build_upper_SAH(arena, treelet_roots, start, mid, total_nodes),
        this->build_upper_SAH(arena, treelet_roots, mid, end, total_nodes));
    return node;
}

int BvhPbrt::flatten_BVH_tree(BVHBuildNode* node, int* offset)
{
    LinearBVHNode* linear_node = &nodes[*offset];
    linear_node->m_bounds = node->m_bounds;
    int my_offset = (*offset)++;
    
    if (node->m_num_primitives > 0) 
    {
        linear_node->m_primitives_offset = node->m_first_prim_offset;
        linear_node->m_num_primitives = node->m_num_primitives;
    }
    else 
    {
        // Create interior flattened BVH node
        linear_node->m_axis = node->m_split_axis;
        linear_node->m_num_primitives = 0;
        flatten_BVH_tree(node->m_children[0], offset);
        linear_node->m_second_child_offset =
            flatten_BVH_tree(node->m_children[1], offset);
    }
    return my_offset;
}


// Interface
BvhPbrt::~BvhPbrt()
{
    free_aligned(nodes);
}

bool BvhPbrt::build(const Triangle* primitives, size_t primitive_count)
{
    std::vector<BVHPrimitiveInfo> primitive_info(primitive_count);
    m_primitives.resize(primitive_count);

    for (size_t i = 0; i < primitive_count; ++i)
    {
        m_primitives[i] = primitives[i];

        glm::vec3 b_min = primitives[i].v[0];
        glm::vec3 b_max = primitives[i].v[0];

        b_min = glm::min(b_min, primitives[i].v[1]);
        b_min = glm::min(b_min, primitives[i].v[2]);

        b_max = glm::max(b_max, primitives[i].v[1]);
        b_max = glm::max(b_max, primitives[i].v[2]);

        primitive_info[i] = { i, Bounds3(b_min, b_max) };
    }

    // Build BVH tree for primitives using _primitiveInfo_
    MemoryArena arena(1024 * 1024);
    int total_nodes = 0;
    std::vector<Triangle> ordered_prims;
    ordered_prims.reserve(m_primitives.size());
    BVHBuildNode* root;

    if (m_split_method == SplitMethod::HLBVH)
        root = HLBVH_build(arena, primitive_info, &total_nodes, ordered_prims);
    else
        root = recursive_build(arena, primitive_info, 0, m_primitives.size(),
                              &total_nodes, ordered_prims);
    m_primitives.swap(ordered_prims);

    nodes = alloc_aligned<LinearBVHNode>(total_nodes);
    int offset = 0;
    flatten_BVH_tree(root, &offset);

    return true;
}

void BvhPbrt::intersect1(RayHit& ray_hit) const
{
    Ray& ray = ray_hit.ray;
    Hit& hit = ray_hit.hit;

    if (!nodes)
    {
        hit.geometry_id = INVALID_GEOMETRY_ID;
    };

    size_t hit_index = m_primitives.size();
    glm::vec3 inv_dir(1 / ray.dir.x, 1 / ray.dir.y, 1 / ray.dir.z);
    int dir_is_neg[3] = { inv_dir.x < 0, inv_dir.y < 0, inv_dir.z < 0 };

    // Follow ray through BVH nodes to find primitive intersections
    int to_visit_offset = 0, current_node_index = 0;
    int nodes_to_visit[64];

    while (true)
    {
        const LinearBVHNode* node = &nodes[current_node_index];

        // Check ray against BVH node
        if (node->m_bounds.intersect_p(ray, inv_dir, dir_is_neg))
        {
            if (node->m_num_primitives > 0)
            {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->m_num_primitives; ++i)
                {
                    const Triangle& t = m_primitives[node->m_primitives_offset + i];
                    glm::vec3 bary;
                    if (glm::intersectRayTriangle(ray.org, ray.dir, t.v[0], t.v[1], t.v[2], bary))
                    {
                        if (bary.z < ray.max_t)
                        {
                            ray.max_t = bary.z;

                            hit.bary.x = (1 - bary.x - bary.y);
                            hit.bary.y = bary.x;

                            hit_index = node->m_primitives_offset + i;
                        }
                    }
                }
                if (to_visit_offset == 0) break;
                current_node_index = nodes_to_visit[--to_visit_offset];
            }
            else
            {
                // Put far BVH node on _nodesToVisit_ stack, advance to near
                // node
                if (dir_is_neg[node->m_axis])
                {
                    nodes_to_visit[to_visit_offset++] = current_node_index + 1;
                    current_node_index = node->m_second_child_offset;
                }
                else 
                {
                    nodes_to_visit[to_visit_offset++] = node->m_second_child_offset;
                    current_node_index = current_node_index + 1;
                }
            }
        }
        else 
        {
            if (to_visit_offset == 0) break;
            current_node_index = nodes_to_visit[--to_visit_offset];
        }
    }

    if (hit_index != m_primitives.size())
    {
        const Triangle& t = m_primitives[hit_index];
        hit.geometry_id = t.geometry_id;
        hit.primitive_id = t.primitive_id;
        hit.n = glm::normalize(glm::cross(t.v[1] - t.v[0], t.v[2] - t.v[1]));
    }
    else
    {
        hit.geometry_id = INVALID_GEOMETRY_ID;
        hit.primitive_id = INVALID_PRIMITIVE_ID;
    }
}

const char* BvhPbrt::name() const
{
    return "Bvh Pbrt";
}

size_t BvhPbrt::get_size_in_bytes() const
{
    return 0;
}

} // end of namespace neptun
