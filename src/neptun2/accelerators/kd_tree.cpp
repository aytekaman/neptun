#include "kd_tree.h"

#include <neptun2/util/memory.h>
#include <neptun2/accelerators/ray.h>
#include <neptun2/accelerators/accelerator.h>

#include <glm/gtx/intersect.hpp>

#include <algorithm>
#include <ctime>
#include <iostream>

namespace neptun
{

void KdNode::InitLeaf(
    int*              primNums,
    int               np,
    std::vector<int>* primitiveIndices)
{
    flags = 3;
    nPrims |= (np << 2);
    // Store primitive ids for leaf node
    if (np == 0)
        onePrimitive = 0;
    else if (np == 1)
        onePrimitive = primNums[0];
    else {
        primitiveIndicesOffset = primitiveIndices->size();
        for (int i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
    }
}

bool KdTree::build(const Triangle* primitives, size_t primitive_count)
{
    m_primitives.resize(primitive_count);
    for (size_t i = 0; i < primitive_count; ++i)
    {
        m_primitives[i] = primitives[i];
    }

    nextFreeNode = nAllocedNodes = 0;
    if (maxDepth <= 0)
    {
        // !!! Log2Int
        unsigned long lz = Log2Int(int64_t(m_primitives.size()));

        maxDepth = std::round(8 + 1.3f * lz);
    }

    // Compute bounds for kd-tree construction

    primBounds.reserve(m_primitives.size());

    //std::cout << "faces size: " << faces.size() << std::endl;

    for (const Triangle& t : m_primitives) {

        glm::vec3 b_min = t.v[0];
        glm::vec3 b_max = t.v[0];

        b_min = (glm::min)(b_min, t.v[1]);
        b_min = (glm::min)(b_min, t.v[2]);

        b_max = (glm::max)(b_max, t.v[1]);
        b_max = (glm::max)(b_max, t.v[2]);

        Bounds3 b(b_min, b_max);
        bounds.add(b);
        primBounds.push_back(b);
    }

    // Allocate working memory for kd-tree construction
    BoundEdge* edges[3];
    for (int i = 0; i < 3; ++i)
        edges[i] = new BoundEdge[2 * m_primitives.size()];

    int* prims0 = new int[m_primitives.size()];
    int* prims1 = new int[(maxDepth + 1) * m_primitives.size()];

    // Initialize _primNums_ for kd-tree construction
    int* primNums = new int[m_primitives.size()];
    for (size_t i = 0; i < m_primitives.size(); ++i) primNums[i] = i;

    // Start recursive construction of kd-tree
    buildTree(0, bounds, primBounds, primNums, m_primitives.size(),
              maxDepth, edges, prims0, prims1);


    for (int i = 0; i < 3; ++i)
        delete[] edges[i];// = new BoundEdge[2 * faces.size()];

    delete[] prims0;
    delete[] prims1;
    delete[] primNums;

    return true;
}

KdTree::~KdTree()
{
    free_aligned(nodes);
}

void KdTree::buildTree(int nodeNum, const Bounds3& nodeBounds,
                       const std::vector<Bounds3>& allPrimBounds,
                       int* primNums, int nPrimitives, int depth,
                       BoundEdge* edges[3],
                       int* prims0, int* prims1, int badRefines) {

    if (nextFreeNode == nAllocedNodes) {
        int nNewAllocNodes = std::max(2 * nAllocedNodes, 512);
        KdNode* n = alloc_aligned<KdNode>(nNewAllocNodes);
        if (nAllocedNodes > 0)
        {
            memcpy(n, nodes, nAllocedNodes * sizeof(KdNode));
            free_aligned(nodes);
        }
        nodes = n;
        nAllocedNodes = nNewAllocNodes;
    }
    ++nextFreeNode;

    // Initialize leaf node if termination criteria met
    if (nPrimitives <= maxPrims || depth == 0)
    {
        leafBounds.push_back(nodeBounds);
        nodes[nodeNum].InitLeaf(primNums, nPrimitives, &primitiveIndices);
        return;
    }

    //std::cout << "done" << std::endl;
    //
        // Initialize interior node and continue recursion

        // Choose split axis position for interior node
    int bestAxis = -1, bestOffset = -1;
    float bestCost = std::numeric_limits<float>::infinity();
    float oldCost = isectCost * float(nPrimitives);
    float totalSA = nodeBounds.surface_area();
    float invTotalSA = 1 / totalSA;
    glm::vec3 d = nodeBounds.m_max_p - nodeBounds.m_min_p;
    //
        // Choose which axis to split along
    int axis = nodeBounds.max_extent();
    int retries = 0;
retrySplit:
    //
        // Initialize edges for _axis_
    for (int i = 0; i < nPrimitives; ++i) {
        int pn = primNums[i];
        const Bounds3& bounds = allPrimBounds[pn];
        edges[axis][2 * i] = BoundEdge(bounds.m_min_p[axis], pn, true);
        edges[axis][2 * i + 1] = BoundEdge(bounds.m_max_p[axis], pn, false);
    }
    //
        // Sort _edges_ for _axis_
    std::sort(&edges[axis][0], &edges[axis][2 * nPrimitives],
              [](const BoundEdge& e0, const BoundEdge& e1) -> bool {
                  if (e0.t == e1.t)
                      return (int)e0.type < (int)e1.type;
                  else
                      return e0.t < e1.t;
              });
    //
        // Compute cost of all splits for _axis_ to find best
    int nBelow = 0, nAbove = nPrimitives;
    for (int i = 0; i < 2 * nPrimitives; ++i) {
        if (edges[axis][i].type == EdgeType::End) --nAbove;
        float edgeT = edges[axis][i].t;
        if (edgeT > nodeBounds.m_min_p[axis] && edgeT < nodeBounds.m_max_p[axis]) {
            // Compute cost for split at _i_th edge

            // Compute child surface areas for split at _edgeT_
            int otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
            float belowSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                (edgeT - nodeBounds.m_min_p[axis]) *
                                 (d[otherAxis0] + d[otherAxis1]));
            float aboveSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                (nodeBounds.m_max_p[axis] - edgeT) *
                                 (d[otherAxis0] + d[otherAxis1]));
            float pBelow = belowSA * invTotalSA;
            float pAbove = aboveSA * invTotalSA;
            float eb = (nAbove == 0 || nBelow == 0) ? emptyBonus : 0;
            float cost =
                traversalCost +
                isectCost * (1 - eb) * (pBelow * nBelow + pAbove * nAbove);

            // Update best split if this is lowest cost so far
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestOffset = i;
            }
        }
        if (edges[axis][i].type == EdgeType::Start) ++nBelow;
    }
    //    CHECK(nBelow == nPrimitives && nAbove == 0);
    //
        // Create leaf if no good splits were found
    if (bestAxis == -1 && retries < 2) {
        ++retries;
        axis = (axis + 1) % 3;
        goto retrySplit;
    }
    if (bestCost > oldCost) ++badRefines;
    if ((bestCost > 4 * oldCost && nPrimitives < 16) || bestAxis == -1 ||
        badRefines == 3) {
        leafBounds.push_back(nodeBounds);
        nodes[nodeNum].InitLeaf(primNums, nPrimitives, &primitiveIndices);
        return;
    }



    //
        // Classify primitives with respect to split
    int n0 = 0, n1 = 0;
    for (int i = 0; i < bestOffset; ++i)
        if (edges[bestAxis][i].type == EdgeType::Start)
            prims0[n0++] = edges[bestAxis][i].primNum;
    for (int i = bestOffset + 1; i < 2 * nPrimitives; ++i)
        if (edges[bestAxis][i].type == EdgeType::End)
            prims1[n1++] = edges[bestAxis][i].primNum;

    // Recursively initialize children nodes
    float tSplit = edges[bestAxis][bestOffset].t;
    Bounds3 bounds0 = nodeBounds, bounds1 = nodeBounds;
    bounds0.m_max_p[bestAxis] = bounds1.m_min_p[bestAxis] = tSplit;
    buildTree(nodeNum + 1, bounds0, allPrimBounds, prims0, n0, depth - 1, edges,
              prims0, prims1 + nPrimitives, badRefines);
    int aboveChild = nextFreeNode;
    nodes[nodeNum].InitInterior(bestAxis, aboveChild, tSplit);
    buildTree(aboveChild, bounds1, allPrimBounds, prims1, n1, depth - 1, edges,
              prims0, prims1 + nPrimitives, badRefines);
}

void KdTree::intersect1(RayHit& rayhit) const 
{
    Ray& ray = rayhit.ray;
    Hit& hit = rayhit.hit;

    size_t hit_index = m_primitives.size();

    //ProfilePhase p(Prof::AccelIntersect);
    // Compute initial parametric range of ray inside kd-tree extent
    float tMin, tMax;
    if (!bounds.intersect_p(ray, &tMin, &tMax)) 
    {
        hit.geometry_id = INVALID_GEOMETRY_ID;
        return;
    }

    // Prepare to traverse kd-tree for ray
    glm::vec3 invDir(1 / ray.dir.x, 1 / ray.dir.y, 1 / ray.dir.z);
    constexpr int maxTodo = 64;
    KdToDo todo[maxTodo];
    int todoPos = 0;

    // Traverse kd-tree nodes in order for ray
    const KdNode* node = &nodes[0];
    while (node != nullptr)
    {
        // Bail out if we found a hit closer than the current node
        //!!!!!!!!!!!!!!!!!
        if (ray.max_t < tMin) break;
        if (!node->IsLeaf()) {
            // Process kd-tree interior node

            // Compute parametric distance along ray to split plane
            int axis = node->SplitAxis();
            float tPlane = (node->SplitPos() - ray.org[axis]) * invDir[axis];

            // Get node children pointers for ray
            const KdNode* firstChild, * secondChild;
            int belowFirst =
                (ray.org[axis] < node->SplitPos()) ||
                (ray.org[axis] == node->SplitPos() && ray.dir[axis] <= 0);
            if (belowFirst) {
                firstChild = node + 1;
                secondChild = &nodes[node->AboveChild()];
            }
            else {
                firstChild = &nodes[node->AboveChild()];
                secondChild = node + 1;
            }

            // Advance to next child node, possibly enqueue other child
            if (tPlane > tMax || tPlane <= 0)
                node = firstChild;
            else if (tPlane < tMin)
                node = secondChild;
            else {
                // Enqueue _secondChild_ in todo list
                todo[todoPos].node = secondChild;
                todo[todoPos].tMin = tPlane;
                todo[todoPos].tMax = tMax;
                ++todoPos;
                node = firstChild;
                tMax = tPlane;
            }
        }
        else 
        {
            // Check for intersections inside leaf node
            int nPrimitives = node->nPrimitives();
            if (nPrimitives == 1) 
            {
                const Triangle& t = m_primitives[node->onePrimitive];

                glm::vec3 bary;
                // Check one primitive inside leaf node
                if (glm::intersectRayTriangle(ray.org, ray.dir, t.v[0], t.v[1], t.v[2], bary))
                {
                    if (bary.z < ray.max_t)
                    {
                        ray.max_t = bary.z;
                        hit.bary.x = 1.f - bary.x - bary.y;
                        hit.bary.y = bary.x;
                        hit_index = node->onePrimitive;
                    }

                }
            }
            else {
                for (int i = 0; i < nPrimitives; ++i) {
                    int index = primitiveIndices[node->primitiveIndicesOffset + i];

                    const Triangle& t = m_primitives[index];
                    glm::vec3 bary;
                    // Check one primitive inside leaf node
                    if (glm::intersectRayTriangle(ray.org, ray.dir, t.v[0], t.v[1], t.v[2], bary))
                    {
                        if (bary.z < ray.max_t)
                        {
                            ray.max_t = bary.z;
                            hit.bary.x = 1.f - bary.x - bary.y;
                            hit.bary.y = bary.x;
                            hit_index = index;
                        }

                    }
                }
            }

            // Grab next node to process from todo list
            if (todoPos > 0) {
                --todoPos;
                node = todo[todoPos].node;
                tMin = todo[todoPos].tMin;
                tMax = todo[todoPos].tMax;
            }
            else
                break;
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
    }
}

const char* KdTree::name() const
{
    return "Kd Tree";
}

size_t KdTree::get_size_in_bytes() const
{
    return 0;
}

} // end of namespace neptun