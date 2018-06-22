#include "bvh.h"

#include <algorithm>
#include <ctime>
#include <iostream>



#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


#include "mesh.h"
#include "scene.h"
#include "stats.h"

struct BVHPrimitiveInfo {
    BVHPrimitiveInfo() {}
    BVHPrimitiveInfo(size_t primitiveNumber, const Bounds3 &bounds)
        : primitiveNumber(primitiveNumber),
        bounds(bounds),
        centroid(.5f * bounds.min + .5f * bounds.max) {}
    size_t primitiveNumber;
    Bounds3 bounds;
    glm::vec3 centroid;
};

struct BVHBuildNode {
    // BVHBuildNode Public Methods
    void InitLeaf(int first, int n, const Bounds3 &b) {
        firstPrimOffset = first;
        nPrimitives = n;
        bounds = b;
        children[0] = children[1] = nullptr;
    }
    void InitInterior(int axis, BVHBuildNode *c0, BVHBuildNode *c1) {
        children[0] = c0;
        children[1] = c1;
        bounds = Bounds3::add(c0->bounds, c1->bounds);
        splitAxis = axis;
        nPrimitives = 0;
    }
    Bounds3 bounds;
    BVHBuildNode *children[2];
    int splitAxis, firstPrimOffset, nPrimitives;
};

struct MortonPrimitive {
    int primitiveIndex;
    uint32_t mortonCode;
};

struct LBVHTreelet {
    int startIndex, nPrimitives;
    BVHBuildNode *buildNodes;
};

struct LinearBVHNode {
    Bounds3 bounds;
    union {
        int primitivesOffset;   // leaf
        int secondChildOffset;  // interior
    };
    uint16_t nPrimitives;  // 0 -> interior node
    uint8_t axis;          // interior node: xyz
    uint8_t pad[1];        // ensure 32 byte total size
};

// Bvh Utility Functions
inline uint32_t LeftShift3(uint32_t x) {
    //CHECK_LE(x, (1 << 10));
    if (x == (1 << 10)) --x;
#ifdef PBRT_HAVE_BINARY_CONSTANTS
    x = (x | (x << 16)) & 0b00000011000000000000000011111111;
    // x = ---- --98 ---- ---- ---- ---- 7654 3210
    x = (x | (x << 8)) & 0b00000011000000001111000000001111;
    // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    x = (x | (x << 4)) & 0b00000011000011000011000011000011;
    // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    x = (x | (x << 2)) & 0b00001001001001001001001001001001;
    // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
#else
    x = (x | (x << 16)) & 0x30000ff;
    // x = ---- --98 ---- ---- ---- ---- 7654 3210
    x = (x | (x << 8)) & 0x300f00f;
    // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    x = (x | (x << 4)) & 0x30c30c3;
    // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    x = (x | (x << 2)) & 0x9249249;
    // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
#endif // PBRT_HAVE_BINARY_CONSTANTS
    return x;
}

inline uint32_t EncodeMorton3(const glm::vec3 &v) {
    //CHECK_GE(v.x, 0);
    //CHECK_GE(v.y, 0);
    //CHECK_GE(v.z, 0);
    return (LeftShift3(v.z) << 2) | (LeftShift3(v.y) << 1) | LeftShift3(v.x);
}

static void RadixSort(std::vector<MortonPrimitive> *v) {
    std::vector<MortonPrimitive> tempVector(v->size());
    constexpr int bitsPerPass = 6;
    constexpr int nBits = 30;
    static_assert((nBits % bitsPerPass) == 0,
        "Radix sort bitsPerPass must evenly divide nBits");
    constexpr int nPasses = nBits / bitsPerPass;

    for (int pass = 0; pass < nPasses; ++pass) {
        // Perform one pass of radix sort, sorting _bitsPerPass_ bits
        int lowBit = pass * bitsPerPass;

        // Set in and out vector pointers for radix sort pass
        std::vector<MortonPrimitive> &in = (pass & 1) ? tempVector : *v;
        std::vector<MortonPrimitive> &out = (pass & 1) ? *v : tempVector;

        // Count number of zero bits in array for current radix sort bit
        constexpr int nBuckets = 1 << bitsPerPass;
        int bucketCount[nBuckets] = { 0 };
        constexpr int bitMask = (1 << bitsPerPass) - 1;
        for (const MortonPrimitive &mp : in) {
            int bucket = (mp.mortonCode >> lowBit) & bitMask;
            //CHECK_GE(bucket, 0);
            //CHECK_LT(bucket, nBuckets);
            ++bucketCount[bucket];
        }

        // Compute starting index in output array for each bucket
        int outIndex[nBuckets];
        outIndex[0] = 0;
        for (int i = 1; i < nBuckets; ++i)
            outIndex[i] = outIndex[i - 1] + bucketCount[i - 1];

        // Store sorted values in output array
        for (const MortonPrimitive &mp : in) {
            int bucket = (mp.mortonCode >> lowBit) & bitMask;
            out[outIndex[bucket]++] = mp;
        }
    }
    // Copy final result from _tempVector_, if needed
    if (nPasses & 1) std::swap(*v, tempVector);
}


Bvh::Bvh(Scene &scene,
    int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)),
    splitMethod(splitMethod) {

    clock_t start_time = clock();
    //ProfilePhase _(Prof::AccelConstruction);

    // Build BVH from _primitives_

    // Initialize _primitiveInfo_ array for primitives
    scene_ptr = &scene;

    initFaces();

    std::vector<BVHPrimitiveInfo> primitiveInfo(faces.size());

    for (size_t i = 0; i < faces.size(); ++i)
    {
        const Face& face = faces[i];

        glm::vec3 b_min = face.vertices[0];
        glm::vec3 b_max = face.vertices[0];

        b_min = glm::min(b_min, face.vertices[1]);
        b_min = glm::min(b_min, face.vertices[2]);

        b_max = glm::max(b_max, face.vertices[1]);
        b_max = glm::max(b_max, face.vertices[2]);

        primitiveInfo[i] = { i, Bounds3(b_min, b_max)};
    }

    // Build BVH tree for primitives using _primitiveInfo_
    MemoryArena arena(1024 * 1024);
    int totalNodes = 0;
    std::vector<Face> orderedPrims;
    orderedPrims.reserve(faces.size());
    BVHBuildNode *root;
    if (splitMethod == SplitMethod::HLBVH)
        root = HLBVHBuild(arena, primitiveInfo, &totalNodes, orderedPrims);
    else
        root = recursiveBuild(arena, primitiveInfo, 0, faces.size(),
            &totalNodes, orderedPrims);
    faces.swap(orderedPrims);
    /*LOG(INFO) << StringPrintf("BVH created with %d nodes for %d "
        "primitives (%.2f MB)", totalNodes,
        (int)primitives.size(),
        float(totalNodes * sizeof(LinearBVHNode)) /
        (1024.f * 1024.f));*/

    //// Compute representation of depth-first traversal of BVH tree
    //treeBytes += totalNodes * sizeof(LinearBVHNode) + sizeof(*this) +
    //    primitives.size() * sizeof(primitives[0]);
    nodes = AllocAligned<LinearBVHNode>(totalNodes);
    int offset = 0;
    flattenBVHTree(root, &offset);
    //CHECK_EQ(totalNodes, offset);

    printf("Node count: %d", totalNodes);
    printf("Node size:  %zu", sizeof(LinearBVHNode));

    printf("BVH size:  %zu MB", (totalNodes * sizeof(LinearBVHNode)) / (1024 * 1024));

    clock_t elapsed = clock() - start_time;

    Stats::add_build_time(elapsed / (float)CLOCKS_PER_SEC);
}

Bounds3 Bvh::WorldBound() const {
    return nodes ? nodes[0].bounds : Bounds3();
}

struct BucketInfo {
    int count = 0;
    Bounds3 bounds;
};

BVHBuildNode *Bvh::recursiveBuild(
    MemoryArena &arena, std::vector<BVHPrimitiveInfo> &primitiveInfo, int start,
    int end, int *totalNodes,
    std::vector<Face> &orderedPrims) {
    //CHECK_NE(start, end);
    BVHBuildNode *node = arena.Alloc<BVHBuildNode>();
    (*totalNodes)++;
    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = start; i < end; ++i)
        bounds.add(primitiveInfo[i].bounds);
    int nPrimitives = end - start;
    if (nPrimitives == 1) {
        // Create leaf _BVHBuildNode_
        int firstPrimOffset = orderedPrims.size();
        for (int i = start; i < end; ++i) {
            int primNum = primitiveInfo[i].primitiveNumber;
            orderedPrims.push_back(faces[primNum]);
        }
        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
        return node;
    }
    else {
        // Compute bound of primitive centroids, choose split dimension _dim_
        Bounds3 centroidBounds;
        for (int i = start; i < end; ++i)
            centroidBounds.add(primitiveInfo[i].centroid);
        int dim = centroidBounds.max_extent();

        // Partition primitives into two sets and build children
        int mid = (start + end) / 2;
        if (centroidBounds.max[dim] == centroidBounds.min[dim]) {
            // Create leaf _BVHBuildNode_
            int firstPrimOffset = orderedPrims.size();
            for (int i = start; i < end; ++i) {
                int primNum = primitiveInfo[i].primitiveNumber;
                orderedPrims.push_back(faces[primNum]);
            }
            node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
            return node;
        }
        else {
            // Partition primitives based on _splitMethod_
            switch (splitMethod) {
            case SplitMethod::Middle: {
                // Partition primitives through node's midpoint
                float pmid =
                    (centroidBounds.min[dim] + centroidBounds.max[dim]) / 2;
                BVHPrimitiveInfo *midPtr = std::partition(
                    &primitiveInfo[start], &primitiveInfo[end - 1] + 1,
                    [dim, pmid](const BVHPrimitiveInfo &pi) {
                    return pi.centroid[dim] < pmid;
                });
                mid = midPtr - &primitiveInfo[0];
                // For lots of prims with large overlapping bounding boxes, this
                // may fail to partition; in that case don't break and fall
                // through
                // to EqualCounts.
                if (mid != start && mid != end) break;
            }
            case SplitMethod::EqualCounts: {
                // Partition primitives into equally-sized subsets
                mid = (start + end) / 2;
                std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
                    &primitiveInfo[end - 1] + 1,
                    [dim](const BVHPrimitiveInfo &a,
                        const BVHPrimitiveInfo &b) {
                    return a.centroid[dim] < b.centroid[dim];
                });
                break;
            }
            case SplitMethod::SAH:
            default: {
                // Partition primitives using approximate SAH
                if (nPrimitives <= 2) {
                    // Partition primitives into equally-sized subsets
                    mid = (start + end) / 2;
                    std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
                        &primitiveInfo[end - 1] + 1,
                        [dim](const BVHPrimitiveInfo &a,
                            const BVHPrimitiveInfo &b) {
                        return a.centroid[dim] <
                            b.centroid[dim];
                    });
                }
                else {
                    // Allocate _BucketInfo_ for SAH partition buckets
                    constexpr int nBuckets = 12;
                    BucketInfo buckets[nBuckets];

                    // Initialize _BucketInfo_ for SAH partition buckets
                    for (int i = start; i < end; ++i) {
                        int b = nBuckets *
                            centroidBounds.offset(
                                primitiveInfo[i].centroid)[dim];
                        if (b == nBuckets) b = nBuckets - 1;
                        //CHECK_GE(b, 0);
                        //CHECK_LT(b, nBuckets);
                        buckets[b].count++;
                        buckets[b].bounds.add(primitiveInfo[i].bounds);
                    }

                    // Compute costs for splitting after each bucket
                    float cost[nBuckets - 1];
                    for (int i = 0; i < nBuckets - 1; ++i) {
                        Bounds3 b0, b1;
                        int count0 = 0, count1 = 0;
                        for (int j = 0; j <= i; ++j) {
                            b0.add(buckets[j].bounds);
                            count0 += buckets[j].count;
                        }
                        for (int j = i + 1; j < nBuckets; ++j) {
                            b1.add(buckets[j].bounds);
                            count1 += buckets[j].count;
                        }
                        cost[i] = 1 +
                            (count0 * b0.surface_area() +
                                count1 * b1.surface_area()) /
                            bounds.surface_area();
                    }

                    // Find bucket to split at that minimizes SAH metric
                    float minCost = cost[0];
                    int minCostSplitBucket = 0;
                    for (int i = 1; i < nBuckets - 1; ++i) {
                        if (cost[i] < minCost) {
                            minCost = cost[i];
                            minCostSplitBucket = i;
                        }
                    }

                    // Either create leaf or split primitives at selected SAH
                    // bucket
                    float leafCost = nPrimitives;
                    if (nPrimitives > maxPrimsInNode || minCost < leafCost) {
                        BVHPrimitiveInfo *pmid = std::partition(
                            &primitiveInfo[start], &primitiveInfo[end - 1] + 1,
                            [=](const BVHPrimitiveInfo &pi) {
                            int b = nBuckets *
                                centroidBounds.offset(pi.centroid)[dim];
                            if (b == nBuckets) b = nBuckets - 1;
                            //CHECK_GE(b, 0);
                            //CHECK_LT(b, nBuckets);
                            return b <= minCostSplitBucket;
                        });
                        mid = pmid - &primitiveInfo[0];
                    }
                    else {
                        // Create leaf _BVHBuildNode_
                        int firstPrimOffset = orderedPrims.size();
                        for (int i = start; i < end; ++i) {
                            int primNum = primitiveInfo[i].primitiveNumber;
                            orderedPrims.push_back(faces[primNum]);
                        }
                        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
                        return node;
                    }
                }
                break;
            }
            }
            node->InitInterior(dim,
                recursiveBuild(arena, primitiveInfo, start, mid,
                    totalNodes, orderedPrims),
                recursiveBuild(arena, primitiveInfo, mid, end,
                    totalNodes, orderedPrims));
        }
    }
    return node;
}

BVHBuildNode *Bvh::HLBVHBuild(
    MemoryArena &arena, const std::vector<BVHPrimitiveInfo> &primitiveInfo,
    int *totalNodes,
    std::vector<Face> &orderedPrims) const {
    // Compute bounding box of all primitive centroids
    Bounds3 bounds;
    for (const BVHPrimitiveInfo &pi : primitiveInfo)
        bounds.add(pi.centroid);

    // Compute Morton indices of primitives
    std::vector<MortonPrimitive> mortonPrims(primitiveInfo.size());
    for (int i = 0; i < primitiveInfo.size(); i++) {
        // Initialize _mortonPrims[i]_ for _i_th primitive
        constexpr int mortonBits = 10;
        constexpr int mortonScale = 1 << mortonBits;
        mortonPrims[i].primitiveIndex = primitiveInfo[i].primitiveNumber;
        glm::vec3 centroidOffset = bounds.offset(primitiveInfo[i].centroid);
        mortonPrims[i].mortonCode = EncodeMorton3(centroidOffset * (float)mortonScale);
    }//, primitiveInfo.size(), 512);

    // Radix sort primitive Morton indices
    RadixSort(&mortonPrims);

    // Create LBVH treelets at bottom of BVH

    // Find intervals of primitives for each treelet
    std::vector<LBVHTreelet> treeletsToBuild;
    for (int start = 0, end = 1; end <= (int)mortonPrims.size(); ++end) {
#ifdef PBRT_HAVE_BINARY_CONSTANTS
        uint32_t mask = 0b00111111111111000000000000000000;
#else
        uint32_t mask = 0x3ffc0000;
#endif
        if (end == (int)mortonPrims.size() ||
            ((mortonPrims[start].mortonCode & mask) !=
            (mortonPrims[end].mortonCode & mask))) {
            // Add entry to _treeletsToBuild_ for this treelet
            int nPrimitives = end - start;
            int maxBVHNodes = 2 * nPrimitives;
            BVHBuildNode *nodes = arena.Alloc<BVHBuildNode>(maxBVHNodes, false);
            treeletsToBuild.push_back({ start, nPrimitives, nodes });
            start = end;
        }
    }

    // Create LBVHs for treelets in parallel
    std::atomic<int> atomicTotal(0), orderedPrimsOffset(0);
    orderedPrims.resize(faces.size());
    for (int i = 0; i < treeletsToBuild.size(); i++) {
        // Generate _i_th LBVH treelet
        int nodesCreated = 0;
        const int firstBitIndex = 29 - 12;
        LBVHTreelet &tr = treeletsToBuild[i];
        tr.buildNodes =
            emitLBVH(tr.buildNodes, primitiveInfo, &mortonPrims[tr.startIndex],
                tr.nPrimitives, &nodesCreated, orderedPrims,
                &orderedPrimsOffset, firstBitIndex);
        atomicTotal += nodesCreated;
    }//, treeletsToBuild.size());
    *totalNodes = atomicTotal;

    // Create and return SAH BVH from LBVH treelets
    std::vector<BVHBuildNode *> finishedTreelets;
    finishedTreelets.reserve(treeletsToBuild.size());
    for (LBVHTreelet &treelet : treeletsToBuild)
        finishedTreelets.push_back(treelet.buildNodes);
    return buildUpperSAH(arena, finishedTreelets, 0, finishedTreelets.size(),
        totalNodes);
}

BVHBuildNode *Bvh::emitLBVH(
    BVHBuildNode *&buildNodes,
    const std::vector<BVHPrimitiveInfo> &primitiveInfo,
    MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
    std::vector<Face> &orderedPrims,
    std::atomic<int> *orderedPrimsOffset, int bitIndex) const {
    //CHECK_GT(nPrimitives, 0);
    if (bitIndex == -1 || nPrimitives < maxPrimsInNode) {
        // Create and return leaf node of LBVH treelet
        (*totalNodes)++;
        BVHBuildNode *node = buildNodes++;
        Bounds3 bounds;
        int firstPrimOffset = orderedPrimsOffset->fetch_add(nPrimitives);
        for (int i = 0; i < nPrimitives; ++i) {
            int primitiveIndex = mortonPrims[i].primitiveIndex;
            orderedPrims[firstPrimOffset + i] = faces[primitiveIndex];
            bounds.add(primitiveInfo[primitiveIndex].bounds);
        }
        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
        return node;
    }
    else {
        int mask = 1 << bitIndex;
        // Advance to next subtree level if there's no LBVH split for this bit
        if ((mortonPrims[0].mortonCode & mask) ==
            (mortonPrims[nPrimitives - 1].mortonCode & mask))
            return emitLBVH(buildNodes, primitiveInfo, mortonPrims, nPrimitives,
                totalNodes, orderedPrims, orderedPrimsOffset,
                bitIndex - 1);

        // Find LBVH split point for this dimension
        int searchStart = 0, searchEnd = nPrimitives - 1;
        while (searchStart + 1 != searchEnd) {
            //CHECK_NE(searchStart, searchEnd);
            int mid = (searchStart + searchEnd) / 2;
            if ((mortonPrims[searchStart].mortonCode & mask) ==
                (mortonPrims[mid].mortonCode & mask))
                searchStart = mid;
            else {
                //CHECK_EQ(mortonPrims[mid].mortonCode & mask,
                   // mortonPrims[searchEnd].mortonCode & mask);
                searchEnd = mid;
            }
        }
        int splitOffset = searchEnd;
        //CHECK_LE(splitOffset, nPrimitives - 1);
        //CHECK_NE(mortonPrims[splitOffset - 1].mortonCode & mask,
        //    mortonPrims[splitOffset].mortonCode & mask);

        // Create and return interior LBVH node
        (*totalNodes)++;
        BVHBuildNode *node = buildNodes++;
        BVHBuildNode *lbvh[2] = {
            emitLBVH(buildNodes, primitiveInfo, mortonPrims, splitOffset,
            totalNodes, orderedPrims, orderedPrimsOffset,
            bitIndex - 1),
            emitLBVH(buildNodes, primitiveInfo, &mortonPrims[splitOffset],
            nPrimitives - splitOffset, totalNodes, orderedPrims,
            orderedPrimsOffset, bitIndex - 1) };
        int axis = bitIndex % 3;
        node->InitInterior(axis, lbvh[0], lbvh[1]);
        return node;
    }
}

BVHBuildNode *Bvh::buildUpperSAH(MemoryArena &arena,
    std::vector<BVHBuildNode *> &treeletRoots,
    int start, int end,
    int *totalNodes) const {
    //CHECK_LT(start, end);
    int nNodes = end - start;
    if (nNodes == 1) return treeletRoots[start];
    (*totalNodes)++;
    BVHBuildNode *node = arena.Alloc<BVHBuildNode>();

    // Compute bounds of all nodes under this HLBVH node
    Bounds3 bounds;
    for (int i = start; i < end; ++i)
        bounds.add(treeletRoots[i]->bounds);

    // Compute bound of HLBVH node centroids, choose split dimension _dim_
    Bounds3 centroidBounds;
    for (int i = start; i < end; ++i) {
        glm::vec3 centroid =
            (treeletRoots[i]->bounds.min + treeletRoots[i]->bounds.max) *
            0.5f;
        centroidBounds.add(centroid);
    }
    int dim = centroidBounds.max_extent();
    // FIXME: if this hits, what do we need to do?
    // Make sure the SAH split below does something... ?
    //CHECK_NE(centroidBounds.max[dim], centroidBounds.min[dim]);

    // Allocate _BucketInfo_ for SAH partition buckets
    constexpr int nBuckets = 12;
    struct BucketInfo {
        int count = 0;
        Bounds3 bounds;
    };
    BucketInfo buckets[nBuckets];

    // Initialize _BucketInfo_ for HLBVH SAH partition buckets
    for (int i = start; i < end; ++i) {
        float centroid = (treeletRoots[i]->bounds.min[dim] +
            treeletRoots[i]->bounds.max[dim]) *
            0.5f;
        int b =
            nBuckets * ((centroid - centroidBounds.min[dim]) /
            (centroidBounds.max[dim] - centroidBounds.min[dim]));
        if (b == nBuckets) b = nBuckets - 1;
        /*CHECK_GE(b, 0);
        CHECK_LT(b, nBuckets);*/
        buckets[b].count++;
        buckets[b].bounds.add(treeletRoots[i]->bounds);
    }

    // Compute costs for splitting after each bucket
    float cost[nBuckets - 1];
    for (int i = 0; i < nBuckets - 1; ++i) {
        Bounds3 b0, b1;
        int count0 = 0, count1 = 0;
        for (int j = 0; j <= i; ++j) {
            b0.add(buckets[j].bounds);
            count0 += buckets[j].count;
        }
        for (int j = i + 1; j < nBuckets; ++j) {
            b1.add(buckets[j].bounds);
            count1 += buckets[j].count;
        }
        cost[i] = .125f +
            (count0 * b0.surface_area() + count1 * b1.surface_area()) /
            bounds.surface_area();
    }

    // Find bucket to split at that minimizes SAH metric
    float minCost = cost[0];
    int minCostSplitBucket = 0;
    for (int i = 1; i < nBuckets - 1; ++i) {
        if (cost[i] < minCost) {
            minCost = cost[i];
            minCostSplitBucket = i;
        }
    }

    // Split nodes and create interior HLBVH SAH node
    BVHBuildNode **pmid = std::partition(
        &treeletRoots[start], &treeletRoots[end - 1] + 1,
        [=](const BVHBuildNode *node) {
        float centroid =
            (node->bounds.min[dim] + node->bounds.max[dim]) * 0.5f;
        int b = nBuckets *
            ((centroid - centroidBounds.min[dim]) /
            (centroidBounds.max[dim] - centroidBounds.min[dim]));
        if (b == nBuckets) b = nBuckets - 1;
        //CHECK_GE(b, 0);
        //CHECK_LT(b, nBuckets);
        return b <= minCostSplitBucket;
    });
    int mid = pmid - &treeletRoots[0];
    //CHECK_GT(mid, start);
    //CHECK_LT(mid, end);
    node->InitInterior(
        dim, this->buildUpperSAH(arena, treeletRoots, start, mid, totalNodes),
        this->buildUpperSAH(arena, treeletRoots, mid, end, totalNodes));
    return node;
}

int Bvh::flattenBVHTree(BVHBuildNode *node, int *offset) {
    LinearBVHNode *linearNode = &nodes[*offset];
    linearNode->bounds = node->bounds;
    int myOffset = (*offset)++;
    if (node->nPrimitives > 0) {
        //CHECK(!node->children[0] && !node->children[1]);
        //CHECK_LT(node->nPrimitives, 65536);
        linearNode->primitivesOffset = node->firstPrimOffset;
        linearNode->nPrimitives = node->nPrimitives;
    }
    else {
        // Create interior flattened BVH node
        linearNode->axis = node->splitAxis;
        linearNode->nPrimitives = 0;
        flattenBVHTree(node->children[0], offset);
        linearNode->secondChildOffset =
            flattenBVHTree(node->children[1], offset);
    }
    return myOffset;
}

Bvh::~Bvh()
{ 
    FreeAligned(nodes);

    //for (Face face : faces)
    //{
    //	face.reset();
    //}
}

bool Bvh::Intersect(const Ray &ray, IntersectionData& intersection_data) const {
    if (!nodes) return false;
    //ProfilePhase p(Prof::AccelIntersect);
    bool hit = false;
    glm::vec3 invDir(1 / ray.dir.x, 1 / ray.dir.y, 1 / ray.dir.z);
    int dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    // Follow ray through BVH nodes to find primitive intersections
    int toVisitOffset = 0, currentNodeIndex = 0;
    int nodesToVisit[64];
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        // Check ray against BVH node
        if (node->bounds.intersectP(ray, invDir, dirIsNeg)) {
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                {
                    
                        const Face& p =
                        faces[node->primitivesOffset + i];
                    glm::vec3 bary;
                    // Check one primitive inside leaf node
                    if (glm::intersectRayTriangle(ray.origin, ray.dir, p.vertices[0], p.vertices[1], p.vertices[2], bary))
                    {
                        if (bary.z < ray.tMax)
                        {
                            ray.tMax = bary.z;

                            intersection_data.position = ray.origin + bary.z * ray.dir;
                            intersection_data.normal =	(bary.x * p.normals[1] + bary.y * p.normals[2] + (1 - bary.x - bary.y) * p.normals[0]);
                            //uv =		(bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0]);
                        }
                            
                        hit = true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            }
            else {
                // Put far BVH node on _nodesToVisit_ stack, advance to near
                // node
                if (dirIsNeg[node->axis]) {
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                }
                else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        }
        else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

bool Bvh::Intersect_stats(const Ray &ray, IntersectionData& intersection_data, DiagnosticData& diagnostic_data) const {
    if (!nodes) return false;
    //ProfilePhase p(Prof::AccelIntersect);
    bool hit = false;
    glm::vec3 invDir(1 / ray.dir.x, 1 / ray.dir.y, 1 / ray.dir.z);
    int dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    // Follow ray through BVH nodes to find primitive intersections
    int toVisitOffset = 0, currentNodeIndex = 0;
    int nodesToVisit[64];
    while (true) {
        diagnostic_data.visited_node_count++;
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        // Check ray against BVH node
        if (node->bounds.intersectP(ray, invDir, dirIsNeg)) {
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                {

                    const Face& p =
                        faces[node->primitivesOffset + i];
                    glm::vec3 bary;
                    // Check one primitive inside leaf node
                    if (glm::intersectRayTriangle(ray.origin, ray.dir, p.vertices[0], p.vertices[1], p.vertices[2], bary))
                    {
                        if (bary.z < ray.tMax)
                        {
                            ray.tMax = bary.z;

                            intersection_data.position = ray.origin + bary.z * ray.dir;
                            intersection_data.normal = (bary.x * p.normals[1] + bary.y * p.normals[2] + (1 - bary.x - bary.y) * p.normals[0]);
                            //uv =		(bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0]);
                        }

                        hit = true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            }
            else {
                // Put far BVH node on _nodesToVisit_ stack, advance to near
                // node
                if (dirIsNeg[node->axis]) {
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                }
                else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        }
        else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

//bool Bvh::IntersectP(const Ray &ray) const {
//    if (!nodes) return false;
//    ProfilePhase p(Prof::AccelIntersectP);
//    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
//    int dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
//    int nodesToVisit[64];
//    int toVisitOffset = 0, currentNodeIndex = 0;
//    while (true) {
//        const LinearBVHNode *node = &nodes[currentNodeIndex];
//        if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
//            // Process BVH node _node_ for traversal
//            if (node->nPrimitives > 0) {
//                for (int i = 0; i < node->nPrimitives; ++i) {
//                    if (primitives[node->primitivesOffset + i]->IntersectP(
//                        ray)) {
//                        return true;
//                    }
//                }
//                if (toVisitOffset == 0) break;
//                currentNodeIndex = nodesToVisit[--toVisitOffset];
//            }
//            else {
//                if (dirIsNeg[node->axis]) {
//                    /// second child first
//                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
//                    currentNodeIndex = node->secondChildOffset;
//                }
//                else {
//                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
//                    currentNodeIndex = currentNodeIndex + 1;
//                }
//            }
//        }
//        else {
//            if (toVisitOffset == 0) break;
//            currentNodeIndex = nodesToVisit[--toVisitOffset];
//        }
//    }
//    return false;
//}

void Bvh::initFaces()
{
    Scene &scene = *scene_ptr;

    std::vector<SceneObject*> &scene_objects = scene.sceneObjects;

    for (int i = 0, face_idx = 0; i < scene_objects.size(); i++)
    {
        Mesh *mesh = scene.sceneObjects[i]->mesh;

        if (mesh == nullptr)
            continue;

        glm::mat4 t = glm::translate(glm::mat4(1.0f), scene_objects[i]->pos);
        glm::vec3 rot = glm::radians(scene_objects[i]->rot);
        glm::mat4 r = glm::eulerAngleYXZ(rot.y, rot.x, rot.z);
        glm::mat4 s = glm::scale(glm::mat4(1.0), scene_objects[i]->scale);
        
        s[3][3] = 1;

        glm::mat4 m = t * r * s;

        for (int j = 0; j < mesh->m_vertex_count; j += 3)
        {
            glm::vec3 vertex = glm::vec3(m * glm::vec4(mesh->m_vertices[j], 1));

            Face face;;

            //face.material = scene.sceneObjects[i]->material;

            face.vertices[0] = glm::vec3(m * glm::vec4(mesh->m_vertices[j + 0], 1));
            face.vertices[1] = glm::vec3(m * glm::vec4(mesh->m_vertices[j + 1], 1));
            face.vertices[2] = glm::vec3(m * glm::vec4(mesh->m_vertices[j + 2], 1));
                
            face.normals[0] = glm::vec3(r * glm::vec4(mesh->m_normals[j + 0], 1));
            face.normals[1] = glm::vec3(r * glm::vec4(mesh->m_normals[j + 1], 1));
            face.normals[2] = glm::vec3(r * glm::vec4(mesh->m_normals[j + 2], 1));

            //if (mesh->uvs.size() > 0)
            //{
            //    face.uvs[0] = mesh->uvs[j + 0];
            //    face.uvs[1] = mesh->uvs[j + 1];
            //    face.uvs[2] = mesh->uvs[j + 2];
            //}

            faces.push_back(face);
        }
    }
}

BvhEmbree::BvhEmbree(Scene& scene)
{
    scene_ptr = &scene;

    initFaces();

    std::cout << "a" << std::endl;

    RTCDevice rtc_device = rtcNewDevice("isa=sse2");
    rtc_scene = rtcNewScene(rtc_device);

    std::cout << "b" << std::endl;

    RTCGeometry mesh = rtcNewGeometry(rtc_device, RTC_GEOMETRY_TYPE_TRIANGLE);

    std::cout << "c" << std::endl;

    glm::vec3* vertices = (glm::vec3*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(glm::vec3), faces.size() * 3);

    for (int i = 0; i < faces.size(); ++i)
    {
        vertices[3 * i + 0].x = faces[i].vertices[0].x;
        vertices[3 * i + 0].y = faces[i].vertices[0].y;
        vertices[3 * i + 0].z = faces[i].vertices[0].z;
                                      
        vertices[3 * i + 1].x = faces[i].vertices[1].x;
        vertices[3 * i + 1].y = faces[i].vertices[1].y;
        vertices[3 * i + 1].z = faces[i].vertices[1].z;
                                      
        vertices[3 * i + 2].x = faces[i].vertices[2].x;
        vertices[3 * i + 2].y = faces[i].vertices[2].y;
        vertices[3 * i + 2].z = faces[i].vertices[2].z;
    }

    struct Tri
    {
        int v0;
        int v1;
        int v2;
    };

    Tri* triangles = (Tri*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Tri), faces.size());

    for (int i = 0; i < faces.size(); ++i)
    {
        triangles[i].v0 = 3 * i + 0;
        triangles[i].v1 = 3 * i + 1;
        triangles[i].v2 = 3 * i + 2;
    }

    std::cout << "e" << std::endl;

    rtcCommitGeometry(mesh);
    unsigned int geomID = rtcAttachGeometry(rtc_scene, mesh);
    rtcReleaseGeometry(mesh);
    //return geomID;
    rtcCommitScene(rtc_scene);
}

BvhEmbree::~BvhEmbree()
{

}

void BvhEmbree::initFaces()
{
    Scene &scene = *scene_ptr;

    std::vector<SceneObject*> &scene_objects = scene.sceneObjects;

    for (int i = 0, face_idx = 0; i < scene_objects.size(); i++)
    {
        Mesh *mesh = scene.sceneObjects[i]->mesh;

        if (mesh == nullptr)
            continue;

        glm::mat4 t = glm::translate(glm::mat4(1.0f), scene_objects[i]->pos);
        glm::vec3 rot = glm::radians(scene_objects[i]->rot);
        glm::mat4 r = glm::eulerAngleYXZ(rot.y, rot.x, rot.z);
        glm::mat4 s = glm::scale(glm::mat4(1.0), scene_objects[i]->scale);

        s[3][3] = 1;

        glm::mat4 m = t * r * s;

        for (int j = 0; j < mesh->m_vertex_count; j += 3)
        {
            glm::vec3 vertex = glm::vec3(m * glm::vec4(mesh->m_vertices[j], 1));

            Face face;;

            //face.material = scene.sceneObjects[i]->material;

            face.vertices[0] = glm::vec3(m * glm::vec4(mesh->m_vertices[j + 0], 1));
            face.vertices[1] = glm::vec3(m * glm::vec4(mesh->m_vertices[j + 1], 1));
            face.vertices[2] = glm::vec3(m * glm::vec4(mesh->m_vertices[j + 2], 1));

            face.normals[0] = glm::vec3(r * glm::vec4(mesh->m_normals[j + 0], 1));
            face.normals[1] = glm::vec3(r * glm::vec4(mesh->m_normals[j + 1], 1));
            face.normals[2] = glm::vec3(r * glm::vec4(mesh->m_normals[j + 2], 1));

            //if (mesh->uvs.size() > 0)
            //{
            //    face.uvs[0] = mesh->uvs[j + 0];
            //    face.uvs[1] = mesh->uvs[j + 1];
            //    face.uvs[2] = mesh->uvs[j + 2];
            //}

            faces.push_back(face);
        }
    }
}

bool BvhEmbree::Intersect(const Ray& ray, IntersectionData& intersection_data) const
{
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);


    //RTCRay rtc_ray;
    RTCRayHit rtc_hit;

    rtc_hit.ray.org_x = ray.origin.x;
    rtc_hit.ray.org_y = ray.origin.y;
    rtc_hit.ray.org_z = ray.origin.z;

    rtc_hit.ray.dir_x = ray.dir.x;
    rtc_hit.ray.dir_y = ray.dir.y;
    rtc_hit.ray.dir_z = ray.dir.z;

    rtc_hit.ray.tnear = 0.0000001f;
    rtc_hit.ray.tfar = 10000000.0f;
    //rtc_hit.ray.id = rand();

    rtc_hit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtc_hit.hit.primID = RTC_INVALID_GEOMETRY_ID;
    
    rtcIntersect1(rtc_scene, &context, &rtc_hit);

    //intersection_data.position = rtc_hit.ray.

    return rtc_hit.hit.geomID != RTC_INVALID_GEOMETRY_ID;
}