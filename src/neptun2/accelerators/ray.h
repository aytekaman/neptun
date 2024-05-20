#pragma once

#include <glm/glm.hpp>

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
    float max_t[N];

    float dir_x[N];
    float dir_y[N];
    float dir_z[N];
    int   tet_id[N];
};

template <>
struct alignas(4) Ray_<1>
{
public:
    glm::vec3 org;
    float max_t;

    glm::vec3 dir;
    int   tet_id;
};

template <size_t N> 
struct alignas(4 * N) Hit_
{
public:
    float nx[N];
    float ny[N];
    float nz[N];

    float bary_x[N];
    float bary_y[N];

    unsigned int primitive_id[N];
    unsigned int geometry_id[N];
};

template <>
struct alignas(4) Hit_<1>
{
public:
    glm::vec3 n;
    glm::vec2 bary;
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
} // end of namespace neptun
