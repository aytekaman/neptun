#include "kd_tree.h"

#include <algorithm>
#include <ctime>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/norm.hpp>

#include "memory.h"
#include "mesh.h"
#include "neptun/util/timer.h"
#include "stats.h"

void KdNode::InitLeaf(int *primNums, int np,
    std::vector<int> *primitiveIndices) {
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

KdTree::KdTree(Scene& scene,
    int isectCost_, int traversalCost_, float emptyBonus_,
    int maxPrims_, int maxDepth)
    : isectCost(isectCost_),
    traversalCost(traversalCost_),
    maxPrims(maxPrims_),
    emptyBonus(emptyBonus_)
    {
        Timer timer;

        scene_ptr = &scene;

        initFaces();

    nextFreeNode = nAllocedNodes = 0;
    if (maxDepth <= 0)
    {
        // !!! Log2Int
        unsigned long lz = Log2Int(int64_t(faces.size()));

        maxDepth = std::round(8 + 1.3f * lz);
    }
    //std::cout << maxDepth << std::endl;
    //maxDepth = 50;

    // Compute bounds for kd-tree construction

    primBounds.reserve(faces.size());

    //std::cout << "faces size: " << faces.size() << std::endl;

    for (const Face& face : faces) {

        glm::vec3 b_min = face.vertices[0];
        glm::vec3 b_max = face.vertices[0];

        b_min = glm::min(b_min, face.vertices[1]);
        b_min = glm::min(b_min, face.vertices[2]);

        b_max = glm::max(b_max, face.vertices[1]);
        b_max = glm::max(b_max, face.vertices[2]);

        Bounds3 b(b_min, b_max);
        bounds.add(b);
        primBounds.push_back(b);
    }



    // Allocate working memory for kd-tree construction
    BoundEdge *edges[3];
    for (int i = 0; i < 3; ++i)
        edges[i] = new BoundEdge[2 * faces.size()];

    int* prims0 = new int[faces.size()];
    int* prims1 = new int[(maxDepth + 1) * faces.size()];

    // Initialize _primNums_ for kd-tree construction
    int* primNums = new int[faces.size()];
    for (size_t i = 0; i < faces.size(); ++i) primNums[i] = i;

    // Start recursive construction of kd-tree
    buildTree(0, bounds, primBounds, primNums, faces.size(),
        maxDepth, edges, prims0, prims1);

    timer.stop();

    for (int i = 0; i < 3; ++i)
	    delete[] edges[i];// = new BoundEdge[2 * faces.size()];

	delete[] prims0;
	delete[] prims1;
	delete[] primNums;

    Stats::add_build_time(timer.seconds());
}

KdTree::~KdTree()
{
	FreeAligned(nodes);
}

void KdTree::initFaces()
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

            Face face;

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



//
//void KdAccelNode::InitLeaf(int *primNums, int np,
//    std::vector<int> *primitiveIndices) {
//    flags = 3;
//    nPrims |= (np << 2);
//    // Store primitive ids for leaf node
//    if (np == 0)
//        onePrimitive = 0;
//    else if (np == 1)
//        onePrimitive = primNums[0];
//    else {
//        primitiveIndicesOffset = primitiveIndices->size();
//        for (int i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
//    }
//}
//
//KdTree::~KdTree() { FreeAligned(nodes); }
//
void KdTree::buildTree(int nodeNum, const Bounds3 &nodeBounds,
    const std::vector<Bounds3> &allPrimBounds,
    int *primNums, int nPrimitives, int depth,
    BoundEdge* edges[3],
    int *prims0, int *prims1, int badRefines) {

    //std::cout << "!" << std::endl;

    //    CHECK_EQ(nodeNum, nextFreeNode);
    //    // Get next free node from _nodes_ array
    if (nextFreeNode == nAllocedNodes) {
        int nNewAllocNodes = std::max(2 * nAllocedNodes, 512);
        KdNode *n = AllocAligned<KdNode>(nNewAllocNodes);
        if (nAllocedNodes > 0) {
            memcpy(n, nodes, nAllocedNodes * sizeof(KdNode));
            FreeAligned(nodes);
        }
        nodes = n;
        nAllocedNodes = nNewAllocNodes;
    }
    ++nextFreeNode;

    // Initialize leaf node if termination criteria met
    if (nPrimitives <= maxPrims || depth == 0) {
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
    glm::vec3 d = nodeBounds.max - nodeBounds.min;
    //
        // Choose which axis to split along
    int axis = nodeBounds.max_extent();
    int retries = 0;
retrySplit:
    //
        // Initialize edges for _axis_
    for (int i = 0; i < nPrimitives; ++i) {
        int pn = primNums[i];
        const Bounds3 &bounds = allPrimBounds[pn];
        edges[axis][2 * i] = BoundEdge(bounds.min[axis], pn, true);
        edges[axis][2 * i + 1] = BoundEdge(bounds.max[axis], pn, false);
    }
    //
        // Sort _edges_ for _axis_
    std::sort(&edges[axis][0], &edges[axis][2 * nPrimitives],
        [](const BoundEdge &e0, const BoundEdge &e1) -> bool {
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
        if (edgeT > nodeBounds.min[axis] && edgeT < nodeBounds.max[axis]) {
            // Compute cost for split at _i_th edge

            // Compute child surface areas for split at _edgeT_
            int otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
            float belowSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                (edgeT - nodeBounds.min[axis]) *
                (d[otherAxis0] + d[otherAxis1]));
            float aboveSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                (nodeBounds.max[axis] - edgeT) *
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
    bounds0.max[bestAxis] = bounds1.min[bestAxis] = tSplit;
    buildTree(nodeNum + 1, bounds0, allPrimBounds, prims0, n0, depth - 1, edges,
        prims0, prims1 + nPrimitives, badRefines);
    int aboveChild = nextFreeNode;
    nodes[nodeNum].InitInterior(bestAxis, aboveChild, tSplit);
    buildTree(aboveChild, bounds1, allPrimBounds, prims1, n1, depth - 1, edges,
        prims0, prims1 + nPrimitives, badRefines);
}
int KdTree::get_size_in_bytes()
{
    return 0;
}
//}
//
bool KdTree::Intersect(const Ray &ray, IntersectionData& intersection_data) const {


    //ProfilePhase p(Prof::AccelIntersect);
    // Compute initial parametric range of ray inside kd-tree extent
    float tMin, tMax;
    if (!bounds.intersectP(ray, &tMin, &tMax)) {
        return false;
    }


    // Prepare to traverse kd-tree for ray
    glm::vec3 invDir(1 / ray.dir.x, 1 / ray.dir.y, 1 / ray.dir.z);
    constexpr int maxTodo = 64;
    KdToDo todo[maxTodo];
    int todoPos = 0;

    // Traverse kd-tree nodes in order for ray
    bool hit = false;
    const KdNode *node = &nodes[0];
    while (node != nullptr) {
        // Bail out if we found a hit closer than the current node
        //!!!!!!!!!!!!!!!!!
        if (ray.tMax < tMin) break;
        if (!node->IsLeaf()) {
            // Process kd-tree interior node

            // Compute parametric distance along ray to split plane
            int axis = node->SplitAxis();
            float tPlane = (node->SplitPos() - ray.origin[axis]) * invDir[axis];

            // Get node children pointers for ray
            const KdNode *firstChild, *secondChild;
            int belowFirst =
                (ray.origin[axis] < node->SplitPos()) ||
                (ray.origin[axis] == node->SplitPos() && ray.dir[axis] <= 0);
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
        else {
            // Check for intersections inside leaf node
            int nPrimitives = node->nPrimitives();
            if (nPrimitives == 1) {
                const Face& p =
                    faces[node->onePrimitive];

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
            else {
                for (int i = 0; i < nPrimitives; ++i) {
                    int index =
                        primitiveIndices[node->primitiveIndicesOffset + i];

                    const Face& p =
                        faces[index];
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
    return hit;
}

bool KdTree::Intersect_stats(const Ray &ray, IntersectionData& intersection_data, DiagnosticData& diagnostic_data) const {

    diagnostic_data.visited_node_count++;
    //ProfilePhase p(Prof::AccelIntersect);
    // Compute initial parametric range of ray inside kd-tree extent
    float tMin, tMax;
    if (!bounds.intersectP(ray, &tMin, &tMax)) {
        return false;
    }


    // Prepare to traverse kd-tree for ray
    glm::vec3 invDir(1 / ray.dir.x, 1 / ray.dir.y, 1 / ray.dir.z);
    constexpr int maxTodo = 64;
    KdToDo todo[maxTodo];
    int todoPos = 0;

    // Traverse kd-tree nodes in order for ray
    bool hit = false;
    const KdNode *node = &nodes[0];
    while (node != nullptr) {
        diagnostic_data.visited_node_count++;
        // Bail out if we found a hit closer than the current node
        //!!!!!!!!!!!!!!!!!
        if (ray.tMax < tMin) break;
        if (!node->IsLeaf()) {
            // Process kd-tree interior node

            // Compute parametric distance along ray to split plane
            int axis = node->SplitAxis();
            float tPlane = (node->SplitPos() - ray.origin[axis]) * invDir[axis];

            // Get node children pointers for ray
            const KdNode *firstChild, *secondChild;
            int belowFirst =
                (ray.origin[axis] < node->SplitPos()) ||
                (ray.origin[axis] == node->SplitPos() && ray.dir[axis] <= 0);
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
        else {
            // Check for intersections inside leaf node
            int nPrimitives = node->nPrimitives();
            if (nPrimitives == 1) {
                const Face& p =
                    faces[node->onePrimitive];

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
            else {
                for (int i = 0; i < nPrimitives; ++i) {
                    int index =
                        primitiveIndices[node->primitiveIndicesOffset + i];

                    const Face& p =
                        faces[index];
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
    return hit;
}
//
//bool KdTree::IntersectP(const Ray &ray) const {
//    // Compute initial parametric range of ray inside kd-tree extent
//    float tMin, tMax;
//    if (!bounds.IntersectP(ray, &tMin, &tMax)) {
//        return false;
//    }
//
//    // Prepare to traverse kd-tree for ray
//    glm::vec3 invDir(1 / ray.dir.x, 1 / ray.dir.y, 1 / ray.dir.z);
//    constexpr int maxTodo = 64;
//    KdToDo todo[maxTodo];
//    int todoPos = 0;
//    const KdNode *node = &nodes[0];
//    while (node != nullptr) {
//        if (node->IsLeaf()) {
//            // Check for shadow ray intersections inside leaf node
//            int nPrimitives = node->nPrimitives();
//            if (nPrimitives == 1) {
//                const std::shared_ptr<Primitive> &p =
//                    primitives[node->onePrimitive];
//                if (p->IntersectP(ray)) {
//                    return true;
//                }
//            }
//            else {
//                for (int i = 0; i < nPrimitives; ++i) {
//                    int primitiveIndex =
//                        primitiveIndices[node->primitiveIndicesOffset + i];
//                    const std::shared_ptr<Primitive> &prim =
//                        primitives[primitiveIndex];
//                    if (prim->IntersectP(ray)) {
//                        return true;
//                    }
//                }
//            }
//
//            // Grab next node to process from todo list
//            if (todoPos > 0) {
//                --todoPos;
//                node = todo[todoPos].node;
//                tMin = todo[todoPos].tMin;
//                tMax = todo[todoPos].tMax;
//            }
//            else
//                break;
//        }
//        else {
//            // Process kd-tree interior node
//
//            // Compute parametric distance along ray to split plane
//            int axis = node->SplitAxis();
//            float tPlane = (node->SplitPos() - ray.origin[axis]) * invDir[axis];
//
//            // Get node children pointers for ray
//            const KdNode *firstChild, *secondChild;
//            int belowFirst =
//                (ray.origin[axis] < node->SplitPos()) ||
//                (ray.origin[axis] == node->SplitPos() && ray.dir[axis] <= 0);
//            if (belowFirst) {
//                firstChild = node + 1;
//                secondChild = &nodes[node->AboveChild()];
//            }
//            else {
//                firstChild = &nodes[node->AboveChild()];
//                secondChild = node + 1;
//            }
//
//            // Advance to next child node, possibly enqueue other child
//            if (tPlane > tMax || tPlane <= 0)
//                node = firstChild;
//            else if (tPlane < tMin)
//                node = secondChild;
//            else {
//                // Enqueue _secondChild_ in todo list
//                todo[todoPos].node = secondChild;
//                todo[todoPos].tMin = tPlane;
//                todo[todoPos].tMax = tMax;
//                ++todoPos;
//                node = firstChild;
//                tMax = tPlane;
//            }
//        }
//    }
//    return false;
//}
//
//std::shared_ptr<KdTree> CreateKdTreeerator(
//    const std::vector<std::shared_ptr<Primitive>> &prims, const ParamSet &ps) {
//    int isectCost = ps.FindOneInt("intersectcost", 80);
//    int travCost = ps.FindOneInt("traversalcost", 1);
//    Float emptyBonus = ps.FindOneFloat("emptybonus", 0.5f);
//    int maxPrims = ps.FindOneInt("maxprims", 1);
//    int maxDepth = ps.FindOneInt("maxdepth", -1);
//    return std::make_shared<KdTree>(prims, isectCost, travCost, emptyBonus,
//        maxPrims, maxDepth);
