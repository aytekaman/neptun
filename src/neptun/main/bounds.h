
#pragma once

#include <algorithm>

#include <glm/glm.hpp>

#include "ray.h"

#define MachineEpsilon (std::numeric_limits<float>::epsilon() * 0.5)



struct Bounds3
{
    Bounds3()
    {
        min = glm::vec3(1000000, 1000000, 1000000);
        max = glm::vec3(-1000000, -1000000, -1000000);
    }

    Bounds3(glm::vec3 min_, glm::vec3 max_)
    {
        min = min_;
        max = max_;
    }


    glm::vec3 diagonal() const;
    float surface_area() const;
    float volume() const;

    int max_extent() const {
        glm::vec3 d = diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    bool intersectP(const Ray &ray, float* hitt0, float* hitt1) const
    {

        // !!!!!!!!!!!!!!!!
        float t0 = 0, t1 = ray.tMax;
        for (int i = 0; i < 3; ++i) {
            // Update interval for _i_th bounding box slab
            float invRayDir = 1 / ray.dir[i];
            float tNear = (min[i] - ray.origin[i]) * invRayDir;
            float tFar = (max[i] - ray.origin[i]) * invRayDir;

            // Update parametric interval from slab intersection $t$ values
            if (tNear > tFar) std::swap(tNear, tFar);

            // Update _tFar_ to ensure robust ray--bounds intersection
            tFar *= 1 + 2 * ((3 * MachineEpsilon) / (1 - 3 * MachineEpsilon));
            t0 = tNear > t0 ? tNear : t0;
            t1 = tFar < t1 ? tFar : t1;
            if (t0 > t1) return false;
        }

        if (hitt0)
            *hitt0 = t0;

        if (hitt1)
            *hitt1 = t1;
        return true;
    }

    inline const glm::vec3 &operator[](int i) const {
        //DCHECK(i == 0 || i == 1);
        return (i == 0) ? min : max;
    }

    bool intersectP(const Ray &ray, const glm::vec3 &invDir,
        const int dirIsNeg[3]) const {
        const Bounds3 &bounds = *this;
        // Check for ray intersection against $x$ and $y$ slabs
        float tMin = (bounds[dirIsNeg[0]].x - ray.origin.x) * invDir.x;
        float tMax = (bounds[1 - dirIsNeg[0]].x - ray.origin.x) * invDir.x;
        float tyMin = (bounds[dirIsNeg[1]].y - ray.origin.y) * invDir.y;
        float tyMax = (bounds[1 - dirIsNeg[1]].y - ray.origin.y) * invDir.y;

        // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
        tMax *= 1 + 2 * ((3 * MachineEpsilon) / (1 - 3 * MachineEpsilon));
        tyMax *= 1 + 2 * ((3 * MachineEpsilon) / (1 - 3 * MachineEpsilon));
        if (tMin > tyMax || tyMin > tMax) return false;
        if (tyMin > tMin) tMin = tyMin;
        if (tyMax < tMax) tMax = tyMax;

        // Check for ray intersection against $z$ slab
        float tzMin = (bounds[dirIsNeg[2]].z - ray.origin.z) * invDir.z;
        float tzMax = (bounds[1 - dirIsNeg[2]].z - ray.origin.z) * invDir.z;

        // Update _tzMax_ to ensure robust bounds intersection
        tzMax *= 1 + 2 * ((3 * MachineEpsilon) / (1 - 3 * MachineEpsilon));
        if (tMin > tzMax || tzMin > tMax) return false;
        if (tzMin > tMin) tMin = tzMin;
        if (tzMax < tMax) tMax = tzMax;
        return (tMin < ray.tMax) && (tMax > 0);
    }


    void add(const Bounds3& bounds);
    void add(const glm::vec3& point);

    glm::vec3 offset(const glm::vec3 &p) const {
        glm::vec3 o = p - min;
        if (max.x > min.x) o.x /= max.x - min.x;
        if (max.y > min.y) o.y /= max.y - min.y;
        if (max.z > min.z) o.z /= max.z - min.z;
        return o;
    }

    static Bounds3 add(const Bounds3& b1, const Bounds3& b2);

    glm::vec3 min;
    glm::vec3 max;
};
