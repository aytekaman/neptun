#pragma once

#include <cstddef>
#include <vector>
#include <limits>
#include <cassert>

namespace neptun
{

// hit results usually return this when ray does not intersect anything
constexpr unsigned int INVALID_PRIMITIVE_ID = (std::numeric_limits<unsigned int>::max());
constexpr unsigned int INVALID_GEOMETRY_ID = (std::numeric_limits<unsigned int>::max());

// Forward declarations
template <size_t N>
struct Ray_;

template <size_t N>
struct Hit_;

template <size_t N>
struct RayHit_;

template <size_t N>
struct Stats_;

// Typedefs
using Ray       = Ray_<1>;
using Ray4      = Ray_<4>;
using Ray8      = Ray_<8>;
using Ray16     = Ray_<16>;

using Hit       = Hit_<1>;
using Hit4      = Hit_<4>;
using Hit8      = Hit_<8>;
using Hit16     = Hit_<16>;

using RayHit    = RayHit_<1>;
using RayHit4   = RayHit_<4>;
using RayHit8   = RayHit_<8>;
using RayHit16  = RayHit_<16>;

using Stats     = Stats_<1>;
using Stats4    = Stats_<4>;
using Stats8    = Stats_<8>;
using Stats16   = Stats_<16>;

// Used for intersection queries with dynamic size.
// TODO: Implement
struct RayN;
struct HitN;
struct RayHitN;
struct StatsN;

// Definitions
template <size_t N> 
struct alignas(4 * N) Ray_
{
public:
    float org_x[N];
    float org_y[N];
    float org_z[N];

    float dir_x[N];
    float dir_y[N];
    float dir_z[N];

    // Union?
    float max_t[N];
    int   tet_id[N];
};

template <>
struct alignas(4) Ray_<1>
{
public:
    float org_x;
    float org_y;
    float org_z;

    float dir_x;
    float dir_y;
    float dir_z;

    // Union?
    float max_t;
    int   tet_id;
};

template <size_t N> 
struct alignas(4 * N) Hit_
{
public:
    float u[N];
    float v[N];

    // Not needed?
    float nx[N];
    float ny[N];
    float nz[N];

    unsigned int primitive_id[N];
    unsigned int geometry_id[N];
};

template <>
struct alignas(4) Hit_<1>
{
public:
    float u;
    float v;

    // Not needed?
    float nx;
    float ny;
    float nz;

    unsigned int primitive_id;
    unsigned int geometry_id;
};

template <size_t N>
struct RayHit_
{
    Ray_<N> ray;
    Hit_<N> hit;
};

template <size_t N>
struct Stats_
{
    unsigned int traversal_count[N];
    unsigned int hit_test_count[N];
};

template <>
struct Stats_<1>
{
    unsigned int traversal_count;
    unsigned int hit_test_count;
};

// Make sure that alignment of the types are correct. 
static_assert(alignof(Ray) == 4, "Alignment error");
static_assert(alignof(Hit) == 4, "Alignment error");
static_assert(alignof(Ray4) == 16, "Alignment error");
static_assert(alignof(Hit4) == 16, "Alignment error");
static_assert(alignof(Ray8) == 32, "Alignment error");
static_assert(alignof(Hit8) == 32, "Alignment error");
static_assert(alignof(Ray16) == 64, "Alignment error");
static_assert(alignof(Hit16) == 64, "Alignment error");

// Utility functions

// Copy functions is used to extract/store information for packed ray, hit, and stats types.
template <size_t N>
inline void ray_cpy(const Ray_<N>& src, Ray& dst, size_t src_index)
{
    assert(src_index < N);

    dst.org_x = src.org_x[src_index];
    dst.org_y = src.org_y[src_index];
    dst.org_z = src.org_z[src_index];

    dst.dir_x = src.dir_x[src_index];
    dst.dir_y = src.dir_y[src_index];
    dst.dir_z = src.dir_z[src_index];

    dst.max_t = src.max_t[src_index];
    dst.tet_id = src.tet_id[src_index];
}

template <size_t N>
inline void ray_cpy(const Ray& src, Ray_<N>& dst, size_t dst_index)
{
    assert(dst_index < N);

    dst.org_x[dst_index] = src.org_x;
    dst.org_y[dst_index] = src.org_y;
    dst.org_z[dst_index] = src.org_z;

    dst.dir_x[dst_index] = src.dir_x;
    dst.dir_y[dst_index] = src.dir_y;
    dst.dir_z[dst_index] = src.dir_z;

    dst.max_t[dst_index] = src.max_t;
    dst.tet_id[dst_index] = src.tet_id;
}

template <size_t N>
inline void hit_cpy(const Hit_<N>& src, Hit& dst, size_t src_index)
{
    assert(src_index < N);

    dst.u = src.u[src_index];
    dst.v = src.v[src_index];

    dst.nx = src.nx[src_index];
    dst.ny = src.ny[src_index];
    dst.nz = src.nz[src_index];

    dst.primitive_id = src.primitive_id[src_index];
    dst.geometry_id = src.geometry_id[src_index];
}

template <size_t N>
inline void hit_cpy(const Hit& src, Hit_<N>& dst, size_t dst_index)
{
    assert(dst_index < N);

    dst.u[dst_index] = src.u;
    dst.v[dst_index] = src.v;

    dst.nx[dst_index] = src.nx;
    dst.ny[dst_index] = src.ny;
    dst.nz[dst_index] = src.nz;

    dst.primitive_id[dst_index] = src.primitive_id;
    dst.geometry_id[dst_index] = src.geometry_id;
}

template <size_t N>
inline void stats_cpy(const Stats_<N>& src, Stats& dst, size_t src_index)
{
    assert(src_index < N);

    dst.traversal_count = src.traversal_count[src_index];
    dst.hit_test_count = src.hit_test_count[src_index];
}

template <size_t N>
inline void stats_cpy(const Stats& src, Stats_<N>& dst, size_t dst_index)
{
    assert(dst_index < N);

    dst.traversal_count[dst_index] = src.traversal_count;
    dst.hit_test_count[dst_index] = src.hit_test_count;
}
} // end of namespace neptun
