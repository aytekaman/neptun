#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "accelerator.h"
#include "face.h"
#include "ray.h"

class Material;
class Scene;

int* raycast_gpu();


struct Tet
{
    unsigned int v[4];
    int n[4];
	int face_idx[4];
    int region_id;
};

struct SourceTet
{
    unsigned int v[4];
    int n[4];
    int idx;
};

// Represents an oriented face inside a tet.
struct TetFace
{
    unsigned int id[3];
    unsigned int tet_idx;
};

// Represents a shared face between two tets.
// id[3] are oriented w.r.t the tet_idx'th tet.
struct SharedFace
{
    unsigned int id[3];
    unsigned int tet_idx;
    unsigned int other_tet_idx;
};

struct ConstrainedFace
{
    unsigned int tet_idx;
    unsigned int other_tet_idx;
    int n;
    Face* face;
};

enum class SortingMethod
{
	Default, // no sorting.
    Hilbert,
    Morton
};

class TetMesh
{
public:
    TetMesh(const Scene &scene);
    TetMesh(
        const Scene &scene,
        const bool preserve_triangles,
        const bool create_bbox,
        const float quality = 1.0f);

    TetMesh(const TetMesh& tet_mesh)
    {
        m_tets = tet_mesh.m_tets;
        m_points = tet_mesh.m_points;
    }

	~TetMesh();

    void clear();

    void read_from_file(std::string file_name);
    void write_to_file();

    void build_from_scene(
        const Scene &scene,
        bool preserve_triangles,
        bool create_bbox,
        float quality = 1.0f);

    void sort(
        const SortingMethod sorting_method,
        const unsigned int bit_count = 16U,
        const bool use_regions = false,
        const bool swap = false);

    void sort_tets(
        const SortingMethod sorting_method,
        const unsigned int bit_count = 16U,
        const bool use_regions = false,
        const bool swap = false);

    void sort_points(
        const SortingMethod sorting_method,
        const unsigned int bit_count = 16U,
        const bool use_regions = false);

    void init_faces(const Scene& scene);

    virtual void init_acceleration_data() = 0;

    void compute_weight();

    virtual int get_size_in_bytes() = 0;

    int find_tet_brute_force(const glm::vec3& point);

    // Returns the index of the tet that contains the 'point'.
    // Otherwise, returns -1. Output 'tet' is filled with this tetrahedron data.
    virtual int find_tet(const glm::vec3& point, SourceTet& tet) = 0;

    // Returns the index of th tet that contains the 'point'.
    // int  find_tet_idx(const glm::vec3& point);

    // Casts a ray from a point inside the 'tet'.
    virtual bool raycast(
        const Ray& ray, 
        const SourceTet& tet, 
        IntersectionData& intersection_data) = 0;

    // Casts a ray from a point on the 'tet_face'
    virtual bool raycast(
        const Ray& ray, 
        const TetFace& tet_face, 
        IntersectionData& intersection_data) = 0;

    // Returns true if a ray can reach to a target_tet_idx'th tet.
    virtual bool raycast(
        const Ray& ray, 
        const TetFace& tet_face, 
        const int& target_tet_idx = 0) = 0;

    bool raycast_stats(
        const Ray& ray, 
        IntersectionData& intersection_data, 
        DiagnosticData& diagnostic_data);

    bool raycast_optimized_basis(const Ray& ray, IntersectionData& intersection_data);

    std::vector<Tet>        m_tets;
    std::vector<glm::vec3>  m_points;

    std::vector<ConstrainedFace> m_constrained_faces;
    std::vector<Face>            faces;

    SourceTet m_source_tet;

    int     m_air_region_id;
    int     m_constrained_face_count = 0;
    float   m_weight = 0;

    // Used for rendering the tet mesh
    bool is_dirty = true;
    float slice = 0.0f;

    // this is used to perturb input points slightly to avoid bug in TetGen.
    const bool m_perturb_points = true;
    const bool m_empty_volume_optimization = false;
};

class TetMesh32 : public TetMesh
{
public:
    struct Tet32
    {
        unsigned int x;
        unsigned int v[3];
        int n[4];
    };

    TetMesh32(
        const Scene &scene,
        const bool preserve_triangles,
        const bool create_bbox, 
        const float quality = 1.0f);

    TetMesh32(const Scene &scene);

    virtual int get_size_in_bytes();

    void init_acceleration_data() override;

    int  find_tet(const glm::vec3& point, SourceTet& tet) override;

    // Casts a ray from a point inside the 'tet'.
    bool raycast(
        const Ray& ray,
        const SourceTet& tet,
        IntersectionData& intersection_data) override;

    // Casts a ray from a point on the 'tet_face'
    bool raycast(
        const Ray& ray,
        const TetFace& tet_face,
        IntersectionData& intersection_data) override;

    // Returns true if a ray can reach to a target_tet_idx'th tet.
    bool raycast(
        const Ray& ray,
        const TetFace& tet_face,
        const int& target_tet_idx = 0) override;

    std::vector<Tet32> m_tet32s;
};

class TetMesh16 : public TetMesh
{
public:
    struct Tet16
    {
        unsigned int x;
        int n[3];
    };
    
    TetMesh16(
        const Scene &scene,
        const bool preserve_triangles,
        const bool create_bbox,
        const float quality = 1.0f);

    TetMesh16(const Scene &scene);

    virtual int get_size_in_bytes();

    void init_acceleration_data() override;

    int find_tet(
        const glm::vec3& point,
        SourceTet& tet) override;

    // Returns the index of th tet that contains the 'point'.
    // int  find_tet_idx(const glm::vec3& point);

    // Casts a ray from a point inside the 'tet'.
    bool raycast(
        const Ray& ray,
        const SourceTet& tet,
        IntersectionData& intersection_data) override;

    // Casts a ray from a point on the 'tet_face'
    bool raycast(
        const Ray& ray,
        const TetFace& tet_face,
        IntersectionData& intersection_data) override;

    // Returns true if a ray can reach to a target_tet_idx'th tet.
    bool raycast(
        const Ray& ray,
        const TetFace& tet_face,
        const int& target_tet_idx) override;

    std::vector<Tet16> m_tet16s;
};

class TetMesh20 : public TetMesh
{
public:
    // Tet data of 20 bytes. Neighbors are sorted
    // using their corresponding vertex ids as keys.
    struct Tet20
    {
        unsigned int x;
        int n[4];
    };

    TetMesh20(
        const Scene &scene,
        const bool preserve_triangles,
        const bool create_bbox,
        const float quality = 1.0f);

    TetMesh20(const Scene &scene);

    virtual int get_size_in_bytes();

    void init_acceleration_data() override;

    int  find_tet(const glm::vec3& point, SourceTet& tet) override;

    // Casts a ray from a point inside the 'tet'.
    bool raycast(
        const Ray& ray,
        const SourceTet& tet,
        IntersectionData& intersection_data) override;

    // Casts a ray from a point on the 'tet_face'
    bool raycast(
        const Ray& ray,
        const TetFace& tet_face,
        IntersectionData& intersection_data) override;

    // Returns true if a ray can reach to a target_tet_idx'th tet.
    bool raycast(
        const Ray& ray,
        const TetFace& tet_face,
        const int& target_tet_idx = 0) override;

    std::vector<Tet20> m_tet20s;
};

void send_to_gpu();