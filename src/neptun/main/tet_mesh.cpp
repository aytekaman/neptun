#include "tet_mesh.h"

#include <pmmintrin.h>

#include <algorithm>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "basis.h"
#include "logger.h"
#include "memory.h"
#include "mesh.h"
#include "neptun/tet_mesh/tet_mesh_builder_factory.h"
#include "scene.h"
#include "sfc_utils.h"
#include "stats.h"

TetMesh::TetMesh(
    const Scene& scene,
    const bool preserve_triangles,
    const bool create_bbox,
    const float quality)
{
    clock_t start_time = clock();

    build_from_scene(scene, preserve_triangles, create_bbox, quality);

    //init_acceleration_data();

    clock_t elapsed = clock() - start_time;

    Stats::add_build_time(elapsed / (float)CLOCKS_PER_SEC);
}

TetMesh::TetMesh(const Scene& scene)
{
    init_faces(scene);
    Logger::LogWarning(scene.tet_mesh_file_path.c_str());
    read_from_file(scene.tet_mesh_file_path);

    Logger::Log("Tet Mesh is read from the disk.");
    printf("Tet Mesh is read from the disk.\n");
}

TetMesh::~TetMesh()
{
    clear();
} 

void TetMesh::read_from_file(std::string file_name)
{
    //const char* FILENAME = "tet_mesh.txt";
    std::ifstream i(file_name, std::ios::binary);

    // read points
    size_t point_count = m_points.size();
    i.read((char*)&point_count, sizeof(point_count));
    m_points.resize(point_count);
    i.read((char*)&m_points[0], sizeof(glm::vec3) * point_count);

    // read tets;
    size_t tet_count = m_tets.size();
    i.read((char*)&tet_count, sizeof(tet_count));
    m_tets.resize(tet_count);
    i.read((char*)&m_tets[0], sizeof(Tet) * tet_count);

    // read other data
    i.read((char*)&m_air_region_id, sizeof(m_air_region_id));
    i.read((char*)&m_constrained_face_count, sizeof(m_constrained_face_count));

    i.close();
}

void TetMesh::write_to_file()
{
    const char* FILENAME = "./Assets/tet_mesh.tetmesh";
    std::ofstream o(FILENAME, std::ios::binary);

    // write points
    size_t point_count = m_points.size();
    o.write((char*)&point_count, sizeof(point_count));
    o.write((char*)&m_points[0], sizeof(glm::vec3) * point_count);

    // write tets
    size_t tet_count = m_tets.size();
    o.write((char*)&tet_count, sizeof(tet_count));
    o.write((char*)&m_tets[0], sizeof(Tet) * tet_count);

    // write other data
    o.write((char*)&m_air_region_id, sizeof(m_air_region_id));
    o.write((char*)&m_constrained_face_count, sizeof(m_constrained_face_count));

    o.close();
}

void TetMesh::build_from_scene(
    const Scene& scene,
    const bool preserve_triangles,
    const bool create_bbox,
    const float quality)
{
    init_faces(scene);

    struct VertexId
    {
        glm::vec3 vertex;
        int id;
    };

    std::vector<glm::vec3> vertices;
    std::vector<int> triangles;

    int cell_count = 20;
    float cell_size = 1;
    float grid_size = cell_count * cell_size;

    std::vector<VertexId> *vertexIds = new std::vector<VertexId>[cell_count * cell_count * cell_count];

    clock_t start = clock();

    int last_given_id = -1;

    for (int i = 0; i < faces.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            glm::vec3 vertex = faces[i].vertices[j];
            bool found = false;

            glm::ivec3 idx = ((glm::mod(vertex, grid_size)) / grid_size) * (float)cell_count;

            std::vector<VertexId> &cell = vertexIds[idx.x * cell_count * cell_count + idx.y * cell_count + idx.z];

            for (int k = 0; k < cell.size(); k++)
            {
                if (vertex == cell[k].vertex)
                {
                    triangles.push_back(cell[k].id);
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                //glm::ivec3 idx = ((glm::mod(vertex, grid_size)) / grid_size) * (float)cell_count;

                VertexId vid;
                vid.vertex = vertex;
                vid.id = ++last_given_id;
                vertexIds[idx.x * cell_count * cell_count + idx.y * cell_count + idx.z].push_back(vid);

                vertices.push_back(vertex);
                triangles.push_back(last_given_id);
            }
        }
    }

    delete[] vertexIds;

    clock_t end = clock();

    Logger::Log("Duplicate vertices are merged in %.2f seconds.", (float)(end - start) / CLOCKS_PER_SEC);

    //Logger::LogError("This is an error message");

    int num_lights = 0;

    std::vector<glm::vec3> light_positions;

    //for (int i = 0; i < scene.sceneObjects.size(); i++)
    //{
    //	if (scene.sceneObjects[i]->light)
    //	{
    //		light_positions.push_back(scene.sceneObjects[i]->pos);
    //		scene.sceneObjects[i]->light->point_index = vertices.size() + num_lights;
    //		num_lights++;
    //	}
    //}

    // Get tet mesh builder
    TetMeshBuilder* tetmesh_builder = TetMeshBuilderFactory::default_builder(); 
    TetMeshBuilder::TetMeshOut out_data;
    TetMeshBuilder::TetMeshIn in_data((int)vertices.size(), (int)triangles.size() / 3, triangles.size());

    // Fill TetMeshIn struct
    in_data.preserve_triangles = preserve_triangles;
    in_data.quality = quality;

    in_data.points = vertices;
    in_data.facets = triangles;
    for (int i = 0; i < in_data.num_facets(); i++){
        in_data.facet_indices[i] = 3 * i;
        in_data.facet_markerlist[i] = i + 1;
        in_data.is_face_visible[i] = faces[i].is_visible;
    }

    if (create_bbox){
        constexpr int face_indices[6][4] = {{0,1,3,2},  // bottom
                                            {4,5,7,6},  // top
                                            {1,3,7,5},  // right
                                            {2,6,7,3},  // back
                                            {0,4,6,2},  // left
                                            {0,1,5,4}}; // front

        const glm::vec3 bbox[2] = { glm::vec3(-50, -50, -50), // min point
                                    glm::vec3(50, 50, 50)};   // max point

        const int point_index_start = in_data.points.size();
        const int facet_index_start = in_data.facet_indices.size();
        const int facets_size_start = in_data.facets.size();

        in_data.points.resize(in_data.points.size() + 8);
        in_data.facet_markerlist.resize(in_data.facet_markerlist.size() + 6);
        in_data.facet_indices.resize(in_data.facet_indices.size() + 6);
        in_data.facets.resize(in_data.facets.size() + (6 * 4));

        // Perturb points
        const glm::vec3 rot = glm::radians((float)m_perturb_points * glm::vec3(0.01f, 0.01f, 0.01f));
        const glm::mat4 r = glm::eulerAngleYXZ(rot.y, rot.x, rot.z);

        // Create bbox vertices
        for (size_t i = 0; i < 8; i++){
            const float x = bbox[(i & 0x1)].x;
            const float y = bbox[(i >> 1) & 0x1].y;
            const float z = bbox[(i >> 2) & 0x1].z;
            glm::vec3 p(x, y, z);

            // Apply perturbation
            if (m_perturb_points){
                p = glm::vec3(r * glm::vec4(p, 1));
            }
            in_data.points[i + point_index_start] = p;
        }

        // Add facet indices
        int facet_index = facet_index_start;
        int facet_pointer = facets_size_start;
        
        for (int i = 0; i < 6; i++){
            in_data.facet_indices[facet_index] = facet_pointer;
            in_data.facet_markerlist[facet_index] = 0;

            for (int j = 0; j < 4; j++){
                in_data.facets[facet_pointer] = face_indices[i][j] + point_index_start;

                facet_pointer++;
            }
            facet_index++;
        }
    }
    
    const int success = tetmesh_builder->tetrahedralize(in_data, out_data);
    if (success != 0){
        Logger::LogError("TetGen error: %d", success);
        return;
    }

    m_points = std::move(out_data.points);
    m_tets = std::move(out_data.tets);
    m_air_region_id = out_data.air_region_id;
    m_constrained_face_count = out_data.constrained_face_count;

    end = clock();

    Logger::Log("Tet Mesh is generated in %f seconds.", float(end - start) / CLOCKS_PER_SEC);
    Logger::Log("Air region ID: %d", m_air_region_id);
    Logger::Log("Constrained face count: %d", m_constrained_face_count);
}

void TetMesh::sort(const SortingMethod sorting_method, const unsigned int bit_count, const bool use_regions, const bool swap)
{
    if (sorting_method == SortingMethod::Default)
        return;

    sort_points(sorting_method, bit_count, use_regions);
    sort_tets(sorting_method, bit_count, use_regions, swap);

    is_dirty = true;
}

void TetMesh::sort_tets(const SortingMethod sorting_method, const unsigned int bit_count, const bool use_regions, const bool swap)
{
    //SortTets();

    glm::vec3 bb_min(-51, -51, -51);
    glm::vec3 bb_max(51, 51, 51);

    std::vector<std::pair<bitmask_t, int>> pairs;

    for (int i = 0; i < m_tets.size(); i++)
    {
        glm::vec3 center(0.0f, 0.0f, 0.0f);

        for (int j = 0; j < 4; j++)
            center += m_points[m_tets[i].v[j]];

        center /= 4.0f;

        bitmask_t index;

        if (sorting_method == SortingMethod::Hilbert)
            index = SfcUtils::hilbert_idx(bb_min, bb_max, center, bit_count);
        else if (sorting_method == SortingMethod::Morton)
            index = SfcUtils::morton_idx(bb_min, bb_max, center, bit_count);

        if (use_regions)
        {
            bitmask_t region_id = m_tets[i].region_id;
            index |= (region_id << (3 * bit_count));
        }

        std::pair<bitmask_t, int> pair(index, i);

        pairs.push_back(pair);
    }

    std::sort(pairs.begin(), pairs.end());

    //for (int i = 0; i < pairs.size(); i++)
        //std::cout << pairs[i].second << std::endl;

    int *sort_idx = new int[m_tets.size()];
    Tet* sorted_tets = new Tet[m_tets.size()];

    for (int i = 0; i < m_tets.size(); i++)
    {
        sorted_tets[i] = m_tets[pairs[i].second];
        sort_idx[pairs[i].second] = i;
    }

    for (int i = 0; i < m_tets.size(); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            int n_idx = sorted_tets[i].n[j];

            if (n_idx != -1)
            {
                sorted_tets[i].n[j] = sort_idx[n_idx];
            }
        }
    }

    for (int i = 0; i < m_tets.size(); i++)
    {
        m_tets[i] = sorted_tets[i];
    }

    delete[] sorted_tets;
    delete[] sort_idx;

    init_acceleration_data();
}

void TetMesh::sort_points(const SortingMethod sorting_method, const unsigned int bit_count, const bool use_regions)
{
    bool* is_reachable = new bool[m_points.size()]();

    for (int i = 0; i < m_tets.size(); i++)
    {
        if (m_tets[i].region_id == m_air_region_id)
        {
            for (int j = 0; j < 4; j++)
            {
                is_reachable[m_tets[i].v[j]] = true;
            }
        }
    }

    glm::vec3 bb_min(-51, -51, -51);
    glm::vec3 bb_max(51, 51, 51);

    std::vector<std::pair<bitmask_t, int>> pairs;

    for (int i = 0; i < m_points.size(); i++)
    {
        bitmask_t index;

        if (sorting_method == SortingMethod::Hilbert)
            index = SfcUtils::hilbert_idx(bb_min, bb_max, m_points[i], bit_count);
        else if (sorting_method == SortingMethod::Morton)
            index = SfcUtils::morton_idx(bb_min, bb_max, m_points[i], bit_count);

        if (use_regions)
        {
            if (!is_reachable[i])
            {
                bitmask_t region_id = 1;
                index |= (region_id << (bit_count * 3));
            }
        }

        std::pair<bitmask_t, int> pair(index, i);

        pairs.push_back(pair);
    }

    std::sort(pairs.begin(), pairs.end());

    int *sort_idx = new int[m_points.size()];
    glm::vec3* sorted_points = new glm::vec3[m_points.size()];

    for (int i = 0; i < m_points.size(); i++)
    {
        sorted_points[i] = m_points[pairs[i].second];
        sort_idx[pairs[i].second] = i;
    }

    for (int i = 0; i < m_tets.size(); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            m_tets[i].v[j] = sort_idx[m_tets[i].v[j]];
        }
    }

    for (int i = 0; i < m_points.size(); i++)
    {
        m_points[i] = sorted_points[i];
    }

    delete[] sorted_points;
    delete[] sort_idx;
    delete[] is_reachable;

    init_acceleration_data();
}

void TetMesh::init_faces(const Scene& scene)
{
    constexpr float epsilon = std::numeric_limits<float>::epsilon();
    const std::vector<SceneObject*> &scene_objects = scene.sceneObjects;

    for (int i = 0, face_idx = 0; i < scene_objects.size(); i++)
    {
        Mesh *mesh = scene.sceneObjects[i]->mesh;

        if (mesh == nullptr || mesh->m_ignore_tetrahedralization)
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

            if (m_perturb_points)
            {
                face.vertices[0] += epsilon;
                face.vertices[1] += epsilon;
                face.vertices[2] += epsilon;

                face.normals[0] += epsilon;
                face.normals[1] += epsilon;
                face.normals[2] += epsilon;
            }

            //if (mesh->uvs.size() > 0)
            //{
            //    face.uvs[0] = mesh->uvs[j + 0];
            //    face.uvs[1] = mesh->uvs[j + 1];
            //    face.uvs[2] = mesh->uvs[j + 2];
            //}

            face.is_visible = (mesh->m_structure_mesh == false);
            faces.push_back(face);
        }
    }
}

void TetMesh::compute_weight()
{
    m_weight = 0;

    for (auto& tet : m_tets)
    {
        glm::vec3 v[4];

        for (int j = 0; j < 4; j++)
        {
            v[j] = m_points[tet.v[j]];
        }

        for (int j = 0; j < 4; j++)
        {
            const float area = glm::length(glm::cross(v[(j + 2) % 4] - v[(j + 1) % 4], v[(j + 3) % 4] - v[(j + 1) % 4])) / 2;

            m_weight += area * ((tet.n[j] == -1) + 1);
        }
    }
}

int TetMesh::find_tet_brute_force(const glm::vec3& point)
{
    for (int i = 0; i < m_tets.size(); ++i)
    {
        bool flag = true;

        glm::vec3 v[4];

        for (int j = 0; j < 4; j++)
            v[j] = m_points[m_tets[i].v[j]];

        for (int j = 0; j < 4; j++)
        {
            glm::vec3 a = v[(j + 2) % 4] - v[(j + 1) % 4];
            glm::vec3 b = v[(j + 3) % 4] - v[(j + 1) % 4];
            glm::vec3 c = glm::cross(a, b);
            glm::vec3 d = v[j] - v[(j + 1) % 4];

            bool result = glm::dot(c, d) > 0;

            glm::vec3 e = point - v[(j + 1) % 4];

            if ((glm::dot(c, e) > 0) != result)
            {
                flag = false;
                break;
            }
        }

        if(flag)
            return i;
    }

    return -1;
}

void TetMesh::intersect4(const Ray rays[4], const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh::intersect4_common_origin(const glm::vec3 dirs[4], const glm::vec3& origin, const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh::intersect4_common_origin_soa(const glm::vec3 dirs[4], const glm::vec3 & origin, const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh::intersect16_common_origin_soa(const glm::vec3 dirs[16], const glm::vec3 & origin, const SourceTet & tet, IntersectionData intersection_data[16])
{
}

bool TetMesh::intersect_simd(const Ray & ray, const SourceTet & tet, IntersectionData & intersection_data)
{
    return false;
}

bool TetMesh::intersect_simd_b(const Ray& ray, const SourceTet& tet, IntersectionData& intersection_data)
{
    return false;
}

bool TetMesh::intersect_stats(const Ray & ray, const SourceTet& tet, IntersectionData& intersection_data, DiagnosticData & diagnostic_data)
{
    return false;
}

bool is_point_inside_tet(glm::vec3 v[4], glm::vec3 point)
{
    bool flag = true;

    for (int j = 0; j < 4; j++)
    {
        glm::vec3 a = v[(j + 2) % 4] - v[(j + 1) % 4];
        glm::vec3 b = v[(j + 3) % 4] - v[(j + 1) % 4];
        glm::vec3 c = glm::cross(a, b);
        glm::vec3 d = v[j] - v[(j + 1) % 4];

        bool result = glm::dot(c, d) > 0;

        glm::vec3 e = point - v[(j + 1) % 4];

        if ((glm::dot(c, e) > 0) != result)
        {
            flag = false;
            break;
        }
    }

    return flag;
};

void TetMesh::clear()
{
    m_tets.clear();
    m_points.clear();
}

TetMesh32::TetMesh32(
    const Scene & scene,
    const bool preserve_triangles,
    const bool create_bbox,
    const float quality) :
    TetMesh(scene, preserve_triangles, create_bbox, quality)
{
    init_acceleration_data();
    compute_weight();
}

TetMesh32::TetMesh32(const Scene& scene) : TetMesh(scene)
{
    init_acceleration_data();
    compute_weight();
}

int TetMesh32::get_size_in_bytes()
{
    int size_in_bytes = 0;

    size_in_bytes += m_tets.size() * sizeof(Tet32);
    size_in_bytes += m_points.size() * sizeof(glm::vec3);
    size_in_bytes += m_constrained_faces.size() * sizeof(ConstrainedFace);

    return size_in_bytes;
}

void TetMesh32::init_acceleration_data()
{
    if (m_tets.size() == 0)
        return;

    clock_t start = clock();

    FreeAligned(m_tet32s);
    m_tet32s = (Tet32*)AllocAligned(m_tets.size() * 32);
    //m_tet32s = new Tet32[m_tets.size()];

    for (int i = 0; i < m_tets.size(); i++)
    {
        const unsigned* v = m_tets[i].v;

        for (int j = 0; j < 3; ++j)
            m_tet32s[i].v[j] = v[j];

        m_tet32s[i].x = v[0] ^ v[1] ^ v[2] ^ v[3];

        for (int j = 0; j < 4; ++j)
        {
            const int n = m_tets[i].n[j];

            if (m_tets[i].face_idx[j] > 0)
            {
                ConstrainedFace cf;
                cf.face = &faces[m_tets[i].face_idx[j] - 1];
                cf.tet_idx = i;
                cf.other_tet_idx = n;
                m_constrained_faces.push_back(cf);

                m_tet32s[i].n[j] = (m_constrained_faces.size() - 1) | (1 << 31);
            }
            else
                m_tet32s[i].n[j] = n;
        }
    }

    for (int i = 0; i < 4; ++i)
    {
        m_source_tet.v[i] = m_tets[0].v[i];
        m_source_tet.n[i] = m_tet32s[0].n[i];
    }

    clock_t end = clock();

    Logger::Log("Acceleration data is initialized in %.2f seconds.", float(end - start) / CLOCKS_PER_SEC);

    //for (int i = 0; i < m_tet32s.size(); i++)
    //{
    //    int a = rand() % 3;
    //    int b = rand() % 3;

    //    std::swap(m_tet32s[i].v[a], m_tet32s[i].v[b]);
    //    std::swap(m_tet32s[i].n[a], m_tet32s[i].n[b]);
    //}

    Logger::LogWarning("constrained face count: %d", m_constrained_face_count);

    FreeAligned(m_padded_points);
    m_padded_points = (glm::vec4*)AllocAligned(m_points.size() * 16);

    for (int i = 0; i < m_points.size(); ++i)
    {
        m_padded_points[i].x = 0.0f;
        m_padded_points[i].y = m_points[i].z;
        m_padded_points[i].z = m_points[i].y;
        m_padded_points[i].w = m_points[i].x;
    }
         
}

inline float cross(const glm::vec2& a, const glm::vec2& b) { return a.x * b.y - a.y * b.x; };

int TetMesh32::find_tet(const glm::vec3& point, SourceTet& tet)
{
    int index = 0;
    Ray ray;

    glm::vec3 v[4];

    ray.origin = glm::vec3(0, 0, 0);

    for (int i = 0; i < 4; i++)
    {
        int t = m_source_tet.v[i];
        v[i] = m_points[t];
        ray.origin += v[i];
    }

    if (is_point_inside_tet(v, point))
    {
        // point is in the source tet.
        tet = m_source_tet;
        tet.idx = 0;

        //std::cout << index << std::endl;

        return tet.idx;
    }


    ray.origin *= 0.25;

    ray.dir = glm::normalize(point - ray.origin);

    glm::vec2 p[4];
    unsigned int id[4];
    int outIdx = -1;

    const Basis basis(ray.dir);

    for (int i = 0; i < 4; i++)
    {
        id[i] = m_source_tet.v[i];
        v[i] = m_points[id[i]];
        p[i].x = glm::dot(basis.right, v[i] - ray.origin);
        p[i].y = glm::dot(basis.up, v[i] - ray.origin);
    }

    if (cross(p[2], p[1]) <= 0.0 && cross(p[1], p[3]) <= 0.0 && cross(p[3], p[2]) <= 0.0)
        outIdx = 0;
    else if (cross(p[2], p[3]) <= 0.0 && cross(p[3], p[0]) <= 0.0 && cross(p[0], p[2]) <= 0.0)
        outIdx = 1;
    else if (cross(p[0], p[3]) <= 0.0 && cross(p[3], p[1]) <= 0.0 && cross(p[1], p[0]) <= 0.0)
        outIdx = 2;
    else if (cross(p[0], p[1]) <= 0.0 && cross(p[1], p[2]) <= 0.0 && cross(p[2], p[0]) <= 0.0)
    {
        outIdx = 3;
        std::swap(id[0], id[1]);
        std::swap(p[0], p[1]);
        std::swap(v[0], v[1]);
    }
    else
        return false;

    index = m_source_tet.n[outIdx];

    if (index < -1)
    {
        index = index & 0x7FFFFFFF;
        index = m_constrained_faces[index].other_tet_idx;
    }

    //int prev_index = -1;

    while (index > -1)
    {
        id[outIdx] = id[3];
        id[3] = m_tet32s[index].x ^ id[0] ^ id[1] ^ id[2]; // cache total sum for 1, 2 and 3

        v[outIdx] = v[3];
        v[3] = m_points[id[3]];

        glm::vec3 newPoint = v[3] - ray.origin;

        p[outIdx] = p[3];
        p[3].x = glm::dot(basis.right, newPoint);
        p[3].y = glm::dot(basis.up, newPoint);

        // make right or up zero
        outIdx = 0;

        if (cross(p[3], p[0]) < 0)
        {
            if ((cross(p[3], p[2]) >= 0))
                outIdx = 1;
        }
        else if (cross(p[3], p[1]) < 0)
            outIdx = 2;

        glm::vec3 n = glm::cross(v[(outIdx + 1) % 3] - v[3], v[(outIdx + 2) % 3] - v[3]);
        n = glm::normalize(n);
        float dot = glm::dot(glm::normalize(point - v[3]), n);

        //std::cout << dot << std::endl;

        if (dot < 0)
        {
            for (int i = 0; i < 4; ++i)
            {
                tet.v[i] = id[i];

                //for (int j = 0; j < 4; ++j)
                {
                    if (id[i] == m_tet32s[index].v[0])
                        tet.n[i] = m_tet32s[index].n[0];
                    else if (id[i] == m_tet32s[index].v[1])
                        tet.n[i] = m_tet32s[index].n[1];
                    else if (id[i] == m_tet32s[index].v[2])
                        tet.n[i] = m_tet32s[index].n[2];
                    else
                        tet.n[i] = m_tet32s[index].n[3];
                }
            }

            tet.idx = index;

            return index;
        }

        //prev_index = index;

        if (m_tet32s[index].v[0] == id[outIdx])
            index = m_tet32s[index].n[0];
        else if (m_tet32s[index].v[1] == id[outIdx])
            index = m_tet32s[index].n[1];
        else if (m_tet32s[index].v[2] == id[outIdx])
            index = m_tet32s[index].n[2];
        else
            index = m_tet32s[index].n[3];

        if (index < -1)
        {
            index = index & 0x7FFFFFFF;
            index = m_constrained_faces[index].other_tet_idx;
        }
    }

    return -1;
}

bool TetMesh32::intersect(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data)
{
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    unsigned int id[4];
    glm::vec2 p[4];

    const float sign = copysignf(1.0f, ray.dir.z);

    const float a = -1.0f / (sign + ray.dir.z);
    const float b = ray.dir.x * ray.dir.y * a;

    const glm::vec3 right(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x);
    const glm::vec3 up(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y);

    for (int i = 0; i < 4; i++)
    {
        id[i] = source_tet.v[i];
        const glm::vec3 point = m_points[id[i]] - ray.origin;
        p[i].x = glm::dot(right, point);
        p[i].y = glm::dot(up, point);
    }

    signed short outIdx = -1;

    if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
        outIdx = 0;
    else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
        outIdx = 1;
    else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
        outIdx = 2;
    else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
    {
        outIdx = 3;
        std::swap(id[0], id[1]);
        std::swap(p[0], p[1]);
    }
    else
        return false;

    int index = source_tet.n[outIdx];

    while (index >= 0)
    {
        id[outIdx] = id[3];
        id[3] = m_tet32s[index].x ^ id[0] ^ id[1] ^ id[2];
        const glm::vec3 newPoint = m_points[id[3]] - ray.origin;

        p[outIdx] = p[3];
        p[3].x = glm::dot(right, newPoint);
        p[3].y = glm::dot(up, newPoint);

        //p[3] = basis.project(newPoint);

        if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
        {
            if (p[3].x * p[2].y >= p[3].y * p[2].x)
                outIdx = 1;
            else
                outIdx = 0;
        }
        else if (p[3].x * p[1].y < p[3].y * p[1].x)
            outIdx = 2;
        else
            outIdx = 0;

        //prev_index = index;

        if (id[outIdx] == m_tet32s[index].v[0])
            index = m_tet32s[index].n[0];
        else if (id[outIdx] == m_tet32s[index].v[1])
            index = m_tet32s[index].n[1];
        else if (id[outIdx] == m_tet32s[index].v[2])
            index = m_tet32s[index].n[2];
        else
            index = m_tet32s[index].n[3];
    }

    if (index != -1)
    {
        index = (index & 0x7FFFFFFF);
        const Face& face = *m_constrained_faces[index].face;

        const glm::vec3 *v = face.vertices;
        const glm::vec3 *n = face.normals;
        const glm::vec2 *t = face.uvs;

        const glm::vec3 e1 = v[1] - v[0];
        const glm::vec3 e2 = v[2] - v[0];
        const glm::vec3 s = ray.origin - v[0];
        const glm::vec3 q = glm::cross(s, e1);
        const glm::vec3 p = glm::cross(ray.dir, e2);
        const float f = 1.0f / glm::dot(e1, p);
        const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));

        intersection_data.position = ray.origin + f * glm::dot(e2, q) * ray.dir;
        intersection_data.normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
        intersection_data.uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
        intersection_data.tet_idx = m_constrained_faces[index].tet_idx;
        intersection_data.neighbor_tet_idx = m_constrained_faces[index].other_tet_idx;
        intersection_data.hit = true;

        return true;
    }
    else
    {
        intersection_data.hit = false;
        return false;
    }
}

void TetMesh32::intersect4(const Ray rays[4], const SourceTet & tet, IntersectionData intersection_data[4])
{
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    //for (int i = 0; i < 4; i++)
    //{
    //    intersect(rays[i], tet, intersection_data[i]);
    //}

    unsigned int id[4][4];
    glm::vec2 p[4][4];

    const Basis basis[4] = {
        Basis(rays[0].dir),
        Basis(rays[1].dir),
        Basis(rays[2].dir),
        Basis(rays[3].dir) };

    int index[4];

    for (int k = 0; k < 4; ++k)
    {
        for (int i = 0; i < 4; i++)
        {
            id[k][i] = tet.v[i];
            const glm::vec3 point = m_points[id[k][i]] - rays[0].origin;
            p[k][i].x = glm::dot(basis[k].right, point);
            p[k][i].y = glm::dot(basis[k].up, point);
        }
    }

    int outIdx[4] = { -1 };

    for (int k = 0; k < 4; ++k)
    {
        if (cross(p[k][2], p[k][1]) <= 0.0f && cross(p[k][1], p[k][3]) <= 0.0f && cross(p[k][3], p[k][2]) <= 0.0f)
        {
            outIdx[k] = 0;
        }
        else if (cross(p[k][2], p[k][3]) <= 0.0f && cross(p[k][3], p[k][0]) <= 0.0f && cross(p[k][0], p[k][2]) <= 0.0f)
        {
            outIdx[k] = 1;
        }
        else if (cross(p[k][0], p[k][3]) <= 0.0f && cross(p[k][3], p[k][1]) <= 0.0f && cross(p[k][1], p[k][0]) <= 0.0f)
        {
            outIdx[k] = 2;
        }
        else if (cross(p[k][0], p[k][1]) <= 0.0f && cross(p[k][1], p[k][2]) <= 0.0f && cross(p[k][2], p[k][0]) <= 0.0f)
        {
            outIdx[k] = 3;

            std::swap(id[k][0], id[k][1]);
            std::swap(p[k][0], p[k][1]);
        }
    }

    for (int k = 0; k < 4; ++k)
        index[k] = tet.n[outIdx[k]];

    int mask = 0;

    while (mask != 15)
    {
#pragma loop( ivdep )
        for (int k = 0; k < 4; ++k)
        {
            if (index[k] < 0)
            {
                mask |= 1 << k;

                continue;
            }

            id[k][outIdx[k]] = id[k][3];
            id[k][3] = m_tet32s[index[k]].x ^ id[k][0] ^ id[k][1] ^ id[k][2];
            const glm::vec3 newPoint = m_points[id[k][3]] - rays[0].origin;

            p[k][outIdx[k]] = p[k][3];
            p[k][3].x = glm::dot(basis[k].right, newPoint);
            p[k][3].y = glm::dot(basis[k].up, newPoint);
        }

#pragma loop( ivdep )
        for (int k = 0; k < 4; ++k)
        {
            if (index[k] < 0)
                continue;

            if (p[k][3].x * p[k][0].y < p[k][3].y * p[k][0].x) // copysignf here?
            {
                if (p[k][3].x * p[k][2].y >= p[k][3].y * p[k][2].x)
                    outIdx[k] = 1;
                else
                    outIdx[k] = 0;
            }
            else if (p[k][3].x * p[k][1].y < p[k][3].y * p[k][1].x)
                outIdx[k] = 2;
            else
                outIdx[k] = 0;
        }

#pragma loop( ivdep )
        for (int k = 0; k < 4; ++k)
        {
            if (index[k] < 0)
            {
                continue;
            }

            if (id[k][outIdx[k]] == m_tet32s[index[k]].v[0])
                index[k] = m_tet32s[index[k]].n[0];
            else if (id[k][outIdx[k]] == m_tet32s[index[k]].v[1])
                index[k] = m_tet32s[index[k]].n[1];
            else if (id[k][outIdx[k]] == m_tet32s[index[k]].v[2])
                index[k] = m_tet32s[index[k]].n[2];
            else
                index[k] = m_tet32s[index[k]].n[3];
        }
    }

#pragma loop( ivdep )
    for (int k = 0; k < 4; ++k)
    {
        if (index[k] != -1)
        {
            index[k] = (index[k] & 0x7FFFFFFF);
            const Face& face = *m_constrained_faces[index[k]].face;

            const glm::vec3 *v = face.vertices;
            const glm::vec3 *n = face.normals;
            const glm::vec2 *t = face.uvs;

            const glm::vec3 e1 = v[1] - v[0];
            const glm::vec3 e2 = v[2] - v[0];
            const glm::vec3 s = rays[k].origin - v[0];
            const glm::vec3 q = glm::cross(s, e1);
            const glm::vec3 p = glm::cross(rays[k].dir, e2);
            const float f = 1.0f / glm::dot(e1, p);
            const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(rays[k].dir, q));

            intersection_data[k].position = rays[k].origin + f * glm::dot(e2, q) * rays[k].dir;
            intersection_data[k].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
            intersection_data[k].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
            intersection_data[k].tet_idx = m_constrained_faces[index[k]].tet_idx;
            intersection_data[k].neighbor_tet_idx = m_constrained_faces[index[k]].other_tet_idx;
            intersection_data[k].hit = true;
        }
        else
            intersection_data[k].hit = false;
    }
}

void TetMesh32::intersect4_common_origin(const glm::vec3 dirs[4], const glm::vec3& origin, const SourceTet & tet, IntersectionData intersection_data[4])
{
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    unsigned int id[4][4];
    glm::vec2 p[4][4];

    const Basis basis[4] = {
        Basis(dirs[0]),
        Basis(dirs[1]),
        Basis(dirs[2]),
        Basis(dirs[3]) };

    for (int k = 0; k < 4; ++k)
    {
        for (int i = 0; i < 4; i++)
        {
            id[k][i] = tet.v[i];
            const glm::vec3 point = m_points[id[k][i]] - origin;
            p[k][i].x = glm::dot(basis[k].right, point);
            p[k][i].y = glm::dot(basis[k].up, point);
        }
    }

    int outIdx[4] = { -1 };

    for (int k = 0; k < 4; ++k)
    {
        if (cross(p[k][2], p[k][1]) <= 0.0f && cross(p[k][1], p[k][3]) <= 0.0f && cross(p[k][3], p[k][2]) <= 0.0f)
        {
            outIdx[k] = 0;
        }
        else if (cross(p[k][2], p[k][3]) <= 0.0f && cross(p[k][3], p[k][0]) <= 0.0f && cross(p[k][0], p[k][2]) <= 0.0f)
        {
            outIdx[k] = 1;
        }
        else if (cross(p[k][0], p[k][3]) <= 0.0f && cross(p[k][3], p[k][1]) <= 0.0f && cross(p[k][1], p[k][0]) <= 0.0f)
        {
            outIdx[k] = 2;
        }
        else if (cross(p[k][0], p[k][1]) <= 0.0f && cross(p[k][1], p[k][2]) <= 0.0f && cross(p[k][2], p[k][0]) <= 0.0f)
        {
            outIdx[k] = 3;

            std::swap(id[k][0], id[k][1]);
            std::swap(p[k][0], p[k][1]);
        }
    }

    int index[4];

    for (int k = 0; k < 4; ++k)
        index[k] = tet.n[outIdx[k]];

    bool diverged = false;

    while (!diverged)
    {

        for (int k = 0; k < 4; ++k)
        {
            id[k][outIdx[k]] = id[k][3];
            id[k][3] = m_tet32s[index[k]].x ^ id[k][0] ^ id[k][1] ^ id[k][2];
        }


        for (int k = 0; k < 4; ++k)
        {
            const glm::vec3 newPoint = m_points[id[k][3]] - origin;

            p[k][outIdx[k]] = p[k][3];
            p[k][3].x = glm::dot(basis[k].right, newPoint);
            p[k][3].y = glm::dot(basis[k].up, newPoint);
        }


        for (int k = 0; k < 4; ++k)
        {
            if (p[k][3].x * p[k][0].y < p[k][3].y * p[k][0].x) // copysignf here?
            {
                if (p[k][3].x * p[k][2].y >= p[k][3].y * p[k][2].x)
                    outIdx[k] = 1;
                else
                    outIdx[k] = 0;
            }
            else if (p[k][3].x * p[k][1].y < p[k][3].y * p[k][1].x)
                outIdx[k] = 2;
            else
                outIdx[k] = 0;
        }

        for (int k = 0; k < 4; ++k)
        {
            if (id[k][outIdx[k]] == m_tet32s[index[k]].v[0])
                index[k] = m_tet32s[index[k]].n[0];
            else if (id[k][outIdx[k]] == m_tet32s[index[k]].v[1])
                index[k] = m_tet32s[index[k]].n[1];
            else if (id[k][outIdx[k]] == m_tet32s[index[k]].v[2])
                index[k] = m_tet32s[index[k]].n[2];
            else
                index[k] = m_tet32s[index[k]].n[3];

        }

        for (int k = 0; k < 4; ++k)
        {
            if (index[k] < 0)
                diverged = true;
        }
    }

    for (int k = 0; k < 4; ++k)
    {
        while (index[k] >= 0)
        {
            id[k][outIdx[k]] = id[k][3];
            id[k][3] = m_tet32s[index[k]].x ^ id[k][0] ^ id[k][1] ^ id[k][2];
            const glm::vec3 newPoint = m_points[id[k][3]] - origin;

            p[k][outIdx[k]] = p[k][3];
            p[k][3].x = glm::dot(basis[k].right, newPoint);
            p[k][3].y = glm::dot(basis[k].up, newPoint);

            if (p[k][3].x * p[k][0].y < p[k][3].y * p[k][0].x) // copysignf here?
            {
                if (p[k][3].x * p[k][2].y >= p[k][3].y * p[k][2].x)
                    outIdx[k] = 1;
                else
                    outIdx[k] = 0;
            }
            else if (p[k][3].x * p[k][1].y < p[k][3].y * p[k][1].x)
                outIdx[k] = 2;
            else
                outIdx[k] = 0;

            if (id[k][outIdx[k]] == m_tet32s[index[k]].v[0])
                index[k] = m_tet32s[index[k]].n[0];
            else if (id[k][outIdx[k]] == m_tet32s[index[k]].v[1])
                index[k] = m_tet32s[index[k]].n[1];
            else if (id[k][outIdx[k]] == m_tet32s[index[k]].v[2])
                index[k] = m_tet32s[index[k]].n[2];
            else
                index[k] = m_tet32s[index[k]].n[3];
        }
    }

    for (int k = 0; k < 4; ++k)
    {
        if (index[k] != -1)
        {
            index[k] = (index[k] & 0x7FFFFFFF);
            const Face& face = *m_constrained_faces[index[k]].face;

            const glm::vec3 *v = face.vertices;
            const glm::vec3 *n = face.normals;
            const glm::vec2 *t = face.uvs;

            const glm::vec3 e1 = v[1] - v[0];
            const glm::vec3 e2 = v[2] - v[0];
            const glm::vec3 s = origin - v[0];
            const glm::vec3 q = glm::cross(s, e1);
            const glm::vec3 p = glm::cross(dirs[k], e2);
            const float f = 1.0f / glm::dot(e1, p);
            const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(dirs[k], q));

            intersection_data[k].position = origin + f * glm::dot(e2, q) * dirs[k];
            intersection_data[k].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
            intersection_data[k].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
            intersection_data[k].tet_idx = m_constrained_faces[index[k]].tet_idx;
            intersection_data[k].neighbor_tet_idx = m_constrained_faces[index[k]].other_tet_idx;
            intersection_data[k].hit = true;
        }
        else
            intersection_data[k].hit = false;
    }
}

void TetMesh32::intersect4_common_origin_soa(const glm::vec3 dirs[4], const glm::vec3& origin, const SourceTet& tet, IntersectionData intersection_data[4])
{
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    float right[3][4];
    float up   [3][4]; 

    unsigned int id[4][4];
    float p[2][4][4];

    for (int k = 0; k < 4; ++k)
    {
        const float sign = copysignf(1.0f, dirs[k].z);

        const float a = -1.0f / (sign + dirs[k].z);
        const float b = dirs[k].x * dirs[k].y * a;

        right[0][k] = 1.0f + sign * dirs[k].x * dirs[k].x * a;
        right[1][k] = sign * b;
        right[2][k] = -sign * dirs[k].x;

        up[0][k] = b;
        up[1][k] = sign + dirs[k].y * dirs[k].y * a;
        up[2][k] = -dirs[k].y;
    }

    int index[4];


    for (int i = 0; i < 4; i++)
    {
        const glm::vec3 point = m_points[tet.v[i]] - origin;

        for (int k = 0; k < 4; ++k)
            id[i][k] = tet.v[i];

        for (int k = 0; k < 4; ++k)
        {
            p[0][i][k] = right[0][k] * point[0];
            p[1][i][k] = up   [0][k] * point[0];
        }

        for (int axis = 1; axis < 3; ++axis)
        {
            for (int k = 0; k < 4; ++k)
            {
                p[0][i][k] += right[axis][k] * point[axis];
                p[1][i][k] += up   [axis][k] * point[axis];
            }
        }
    }


    int outIdx[4] = { -1 };

    for (int k = 0; k < 4; ++k)
    {
        if      (p[0][2][k] * p[1][1][k] <= p[1][2][k] * p[0][1][k] && p[0][1][k] * p[1][3][k] <= p[1][1][k] * p[0][3][k] && p[0][3][k] * p[1][2][k] <= p[1][3][k] * p[0][2][k])
            outIdx[k] = 0;
        else if (p[0][2][k] * p[1][3][k] <= p[1][2][k] * p[0][3][k] && p[0][3][k] * p[1][0][k] <= p[1][3][k] * p[0][0][k] && p[0][0][k] * p[1][2][k] <= p[1][0][k] * p[0][2][k])
            outIdx[k] = 1;
        else if (p[0][0][k] * p[1][3][k] <= p[1][0][k] * p[0][3][k] && p[0][3][k] * p[1][1][k] <= p[1][3][k] * p[0][1][k] && p[0][1][k] * p[1][0][k] <= p[1][1][k] * p[0][0][k])
            outIdx[k] = 2;
        else if (p[0][0][k] * p[1][1][k] <= p[1][0][k] * p[0][1][k] && p[0][1][k] * p[1][2][k] <= p[1][1][k] * p[0][2][k] && p[0][2][k] * p[1][0][k] <= p[1][2][k] * p[0][0][k])
        {
            outIdx[k] = 3;
            std::swap(id[0][k], id[1][k]);
            std::swap(p[0][0][k], p[0][1][k]);
            std::swap(p[1][0][k], p[1][1][k]);
        }
    }

    for (int k = 0; k < 4; ++k)
        index[k] = tet.n[outIdx[k]];

    //int diverged = false;

    while (index[0] >= 0 || index[1] >= 0 || index[2] >= 0 || index[3] >= 0)
    {
        for (int k = 0; k < 4; ++k)
        {
            if (index[k] < 0)
                continue;

            id[outIdx[k]][k] = id[3][k];
            id[3][k] = m_tet32s[index[k]].x;// ^ id[0][k] ^ id[1][k] ^ id[2][k];
        }

        for (int k = 0; k < 4; ++k)
        {
            if (index[k] < 0)
                continue;

            id[3][k] ^= id[0][k];

            id[3][k] ^= id[1][k];


            id[3][k] ^= id[2][k];
        }
            


        {
            float new_points[3][4];

            for (int k = 0; k < 4; ++k)
            {
                const glm::vec3 new_point = m_points[id[3][k]] - origin;

                for (int axis = 0; axis < 3; ++axis)
                    new_points[axis][k] = new_point[axis];
            }

            for (int axis = 0; axis < 2; ++axis)
                for (int k = 0; k < 4; ++k)
                    p[axis][outIdx[k]][k] = p[axis][3][k];

            for (int k = 0; k < 4; ++k)
                p[0][3][k] = right[0][k] * new_points[0][k];

            //for (int axis = 1; axis < 3; ++axis)
                for (int k = 0; k < 4; ++k)
                    p[0][3][k] += right[1][k] * new_points[1][k];

                for (int k = 0; k < 4; ++k)
                    p[0][3][k] += right[2][k] * new_points[2][k];

            for (int k = 0; k < 4; ++k)
                p[1][3][k] = up[0][k] * new_points[0][k];

            //for (int axis = 1; axis < 3; ++axis)
                for (int k = 0; k < 4; ++k)
                    p[1][3][k] += up[1][k] * new_points[1][k];

                for (int k = 0; k < 4; ++k)
                    p[1][3][k] += up[2][k] * new_points[2][k];
        }

        int r0[4], r1[4], r2[4];

        // (p[3][k].x * p[0][k].y < p[3][k].y * p[0][k].x)
        {
            float a[4], b[4];

            for (int k = 0; k < 4; ++k)
                a[k] = p[0][3][k] * p[1][0][k];

            for (int k = 0; k < 4; ++k)
                b[k] = p[1][3][k] * p[0][0][k];

            for (int k = 0; k < 4; ++k)
                r0[k] = a[k] < b[k];
        }

        // (p[3][k].x * p[2][k].y >= p[3][k].y * p[2][k].x)
        {
            float a[4], b[4];

            for (int k = 0; k < 4; ++k)
                a[k] = p[0][3][k] * p[1][2][k];

            for (int k = 0; k < 4; ++k)
                b[k] = p[1][3][k] * p[0][2][k];

            for (int k = 0; k < 4; ++k)
                r1[k] = a[k] >= b[k];
        }

        // (p[3][k].x * p[1][k].y < p[3][k].y * p[1][k].x)
        {
            float a[4], b[4];

            for (int k = 0; k < 4; ++k)
                a[k] = p[0][3][k] * p[1][1][k];

            for (int k = 0; k < 4; ++k)
                b[k] = p[1][3][k] * p[0][1][k];

            for (int k = 0; k < 4; ++k)
                r2[k] = a[k] < b[k];
        }

        for (int k = 0; k < 4; ++k)
            outIdx[k] = r0[k] * r1[k] + (!r0[k]) * r2[k] * 2;

        for (int k = 0; k < 4; ++k)
        {
            if (index[k] < 0)
                continue;

            if (id[outIdx[k]][k] == m_tet32s[index[k]].v[0])
                index[k] = m_tet32s[index[k]].n[0];
            else if (id[outIdx[k]][k] == m_tet32s[index[k]].v[1])
                index[k] = m_tet32s[index[k]].n[1];
            else if (id[outIdx[k]][k] == m_tet32s[index[k]].v[2])
                index[k] = m_tet32s[index[k]].n[2];
            else
                index[k] = m_tet32s[index[k]].n[3];
        }

        //for (int k = 0; k < 4; ++k)
        //{
        //    if (index[k] < 0)
        //        diverged = true;
        //}
    }

    //for (int k = 0; k < 4; ++k)
    //{
    //    while (index[k] >= 0)
    //    {
    //        id[outIdx[k]][k] = id[3][k];
    //        id[3][k] = m_tet32s[index[k]].x ^ id[0][k] ^ id[1][k] ^ id[2][k];
    //        const glm::vec3 newPoint = m_points[id[3][k]] - origin;

    //        p[outIdx[k]][k] = p[3][k];
    //        p[3][k].x = glm::dot(right[k], newPoint);
    //        p[3][k].y = glm::dot(up[k], newPoint);

    //        if (p[3][k].x * p[0][k].y < p[3][k].y * p[0][k].x) // copysignf here?
    //        {
    //            if (p[3][k].x * p[2][k].y >= p[3][k].y * p[2][k].x)
    //                outIdx[k] = 1;
    //            else
    //                outIdx[k] = 0;
    //        }
    //        else if (p[3][k].x * p[1][k].y < p[3][k].y * p[1][k].x)
    //            outIdx[k] = 2;
    //        else
    //            outIdx[k] = 0;

    //        if (id[outIdx[k]][k] == m_tet32s[index[k]].v[0])
    //            index[k] = m_tet32s[index[k]].n[0];
    //        else if (id[outIdx[k]][k] == m_tet32s[index[k]].v[1])
    //            index[k] = m_tet32s[index[k]].n[1];
    //        else if (id[outIdx[k]][k] == m_tet32s[index[k]].v[2])
    //            index[k] = m_tet32s[index[k]].n[2];
    //        else
    //            index[k] = m_tet32s[index[k]].n[3];
    //    }
    //}

    for (int k = 0; k < 4; ++k)
    {
        if (index[k] != -1)
        {
            index[k] = (index[k] & 0x7FFFFFFF);
            const Face& face = *m_constrained_faces[index[k]].face;

            const glm::vec3 *v = face.vertices;
            const glm::vec3 *n = face.normals;
            const glm::vec2 *t = face.uvs;

            const glm::vec3 e1 = v[1] - v[0];
            const glm::vec3 e2 = v[2] - v[0];
            const glm::vec3 s = origin - v[0];
            const glm::vec3 q = glm::cross(s, e1);
            const glm::vec3 p = glm::cross(dirs[k], e2);
            const float f = 1.0f / glm::dot(e1, p);
            const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(dirs[k], q));

            intersection_data[k].position = origin + f * glm::dot(e2, q) * dirs[k];
            intersection_data[k].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
            intersection_data[k].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
            intersection_data[k].tet_idx = m_constrained_faces[index[k]].tet_idx;
            intersection_data[k].neighbor_tet_idx = m_constrained_faces[index[k]].other_tet_idx;
            intersection_data[k].hit = true;
        }
        else
            intersection_data[k].hit = false;
    }
}

void TetMesh32::intersect16_common_origin_soa(const glm::vec3 dirs[16], const glm::vec3 & origin, const SourceTet & tet, IntersectionData intersection_data[16])
{
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    unsigned int id[4][16];
    glm::vec2 p[4][16];

    glm::vec3 right[16];
    glm::vec3 up[16];

    for (int k = 0; k < 16; ++k)
    {
        const float sign = copysignf(1.0f, dirs[k].z);

        const float a = -1.0f / (sign + dirs[k].z);
        const float b = dirs[k].x * dirs[k].y * a;

        right[k] = glm::vec3(1.0f + sign * dirs[k].x * dirs[k].x * a, sign * b, -sign * dirs[k].x);
        up[k] = glm::vec3(b, sign + dirs[k].y * dirs[k].y * a, -dirs[k].y);
    }

    int index[16];

    for (int k = 0; k < 16; ++k)
    {
        for (int i = 0; i < 4; i++)
        {
            id[i][k] = tet.v[i];
            const glm::vec3 point = m_points[id[i][k]] - origin;
            p[i][k].x = glm::dot(right[k], point);
            p[i][k].y = glm::dot(up[k], point);
        }
    }

    int outIdx[16] = { -1 };

    for (int k = 0; k < 16; ++k)
    {
        if (cross(p[2][k], p[1][k]) <= 0.0f && cross(p[1][k], p[3][k]) <= 0.0f && cross(p[3][k], p[2][k]) <= 0.0f)
        {
            outIdx[k] = 0;
        }
        else if (cross(p[2][k], p[3][k]) <= 0.0f && cross(p[3][k], p[0][k]) <= 0.0f && cross(p[0][k], p[2][k]) <= 0.0f)
        {
            outIdx[k] = 1;
        }
        else if (cross(p[0][k], p[3][k]) <= 0.0f && cross(p[3][k], p[1][k]) <= 0.0f && cross(p[1][k], p[0][k]) <= 0.0f)
        {
            outIdx[k] = 2;
        }
        else if (cross(p[0][k], p[1][k]) <= 0.0f && cross(p[1][k], p[2][k]) <= 0.0f && cross(p[2][k], p[0][k]) <= 0.0f)
        {
            outIdx[k] = 3;

            std::swap(id[0][k], id[1][k]);
            std::swap(p[0][k], p[1][k]);
        }
    }

    for (int k = 0; k < 16; ++k)
        index[k] = tet.n[outIdx[k]];

    bool diverged = false;

    while (!diverged)
    {
#pragma loop( ivdep )
        for (int k = 0; k < 16; ++k)
        {
            id[outIdx[k]][k] = id[3][k];
            id[3][k] = m_tet32s[index[k]].x ^ id[0][k] ^ id[1][k] ^ id[2][k];
        }

        for (int k = 0; k < 16; ++k)
        {
            const glm::vec3 newPoint = m_points[id[3][k]] - origin;

            p[outIdx[k]][k] = p[3][k];
            p[3][k].x = glm::dot(right[k], newPoint);
            p[3][k].y = glm::dot(up[k], newPoint);
        }

#pragma loop( ivdep )
        for (int k = 0; k < 16; ++k)
        {
            if (p[3][k].x * p[0][k].y < p[3][k].y * p[0][k].x) // copysignf here?
            {
                if (p[3][k].x * p[2][k].y >= p[3][k].y * p[2][k].x)
                    outIdx[k] = 1;
                else
                    outIdx[k] = 0;
            }
            else if (p[3][k].x * p[1][k].y < p[3][k].y * p[1][k].x)
                outIdx[k] = 2;
            else
                outIdx[k] = 0;
        }

#pragma loop( ivdep )
        for (int k = 0; k < 16; ++k)
        {
            if (id[outIdx[k]][k] == m_tet32s[index[k]].v[0])
                index[k] = m_tet32s[index[k]].n[0];
            else if (id[outIdx[k]][k] == m_tet32s[index[k]].v[1])
                index[k] = m_tet32s[index[k]].n[1];
            else if (id[outIdx[k]][k] == m_tet32s[index[k]].v[2])
                index[k] = m_tet32s[index[k]].n[2];
            else
                index[k] = m_tet32s[index[k]].n[3];
        }

        for (int k = 0; k < 16; ++k)
        {
            if (index[k] < 0)
                diverged = true;
        }
    }

    for (int k = 0; k < 16; ++k)
    {
        while (index[k] >= 0)
        {
            id[outIdx[k]][k] = id[3][k];
            id[3][k] = m_tet32s[index[k]].x ^ id[0][k] ^ id[1][k] ^ id[2][k];
            const glm::vec3 newPoint = m_points[id[3][k]] - origin;

            p[outIdx[k]][k] = p[3][k];
            p[3][k].x = glm::dot(right[k], newPoint);
            p[3][k].y = glm::dot(up[k], newPoint);

            if (p[3][k].x * p[0][k].y < p[3][k].y * p[0][k].x) // copysignf here?
            {
                if (p[3][k].x * p[2][k].y >= p[3][k].y * p[2][k].x)
                    outIdx[k] = 1;
                else
                    outIdx[k] = 0;
            }
            else if (p[3][k].x * p[1][k].y < p[3][k].y * p[1][k].x)
                outIdx[k] = 2;
            else
                outIdx[k] = 0;

            if (id[outIdx[k]][k] == m_tet32s[index[k]].v[0])
                index[k] = m_tet32s[index[k]].n[0];
            else if (id[outIdx[k]][k] == m_tet32s[index[k]].v[1])
                index[k] = m_tet32s[index[k]].n[1];
            else if (id[outIdx[k]][k] == m_tet32s[index[k]].v[2])
                index[k] = m_tet32s[index[k]].n[2];
            else
                index[k] = m_tet32s[index[k]].n[3];
        }
    }

#pragma loop( ivdep )
    for (int k = 0; k < 16; ++k)
    {
        if (index[k] != -1)
        {
            index[k] = (index[k] & 0x7FFFFFFF);
            const Face& face = *m_constrained_faces[index[k]].face;

            const glm::vec3 *v = face.vertices;
            const glm::vec3 *n = face.normals;
            const glm::vec2 *t = face.uvs;

            const glm::vec3 e1 = v[1] - v[0];
            const glm::vec3 e2 = v[2] - v[0];
            const glm::vec3 s = origin - v[0];
            const glm::vec3 q = glm::cross(s, e1);
            const glm::vec3 p = glm::cross(dirs[k], e2);
            const float f = 1.0f / glm::dot(e1, p);
            const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(dirs[k], q));

            intersection_data[k].position = origin + f * glm::dot(e2, q) * dirs[k];
            intersection_data[k].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
            intersection_data[k].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
            intersection_data[k].tet_idx = m_constrained_faces[index[k]].tet_idx;
            intersection_data[k].neighbor_tet_idx = m_constrained_faces[index[k]].other_tet_idx;
            intersection_data[k].hit = true;
        }
        else
            intersection_data[k].hit = false;
    }
}

bool TetMesh32::intersect_stats(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data, DiagnosticData& diagnostic_data)
{
    unsigned int id[4];
    glm::vec2 p[4];

    ++diagnostic_data.visited_node_count;
    //int prev_index;
    signed short outIdx = -1;

    const Basis basis(ray.dir);

    for (int i = 0; i < 4; i++)
    {
        id[i] = source_tet.v[i];
        const glm::vec3 point = m_points[id[i]] - ray.origin;
        p[i].x = glm::dot(basis.right, point);
        p[i].y = glm::dot(basis.up, point);
    }

    if (cross(p[2], p[1]) <= 0.0 && cross(p[1], p[3]) <= 0.0 && cross(p[3], p[2]) <= 0.0)
        outIdx = 0;
    else if (cross(p[2], p[3]) <= 0.0 && cross(p[3], p[0]) <= 0.0 && cross(p[0], p[2]) <= 0.0)
        outIdx = 1;
    else if (cross(p[0], p[3]) <= 0.0 && cross(p[3], p[1]) <= 0.0 && cross(p[1], p[0]) <= 0.0)
        outIdx = 2;
    else if (cross(p[0], p[1]) <= 0.0 && cross(p[1], p[2]) <= 0.0 && cross(p[2], p[0]) <= 0.0)
    {
        outIdx = 3;
        std::swap(id[0], id[1]);
        std::swap(p[0], p[1]);
    }
    else
        return false;

    int index = source_tet.n[outIdx];

    while (index >= 0)
    {
        ++diagnostic_data.visited_node_count;

        id[outIdx] = id[3];
        id[3] = m_tet32s[index].x ^ id[0] ^ id[1] ^ id[2];
        const glm::vec3 newPoint = m_points[id[3]] - ray.origin;

        p[outIdx] = p[3];
        p[3].x = glm::dot(basis.right, newPoint);
        p[3].y = glm::dot(basis.up, newPoint);

        //p[3] = basis.project(newPoint);

        if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
        {
            if (p[3].x * p[2].y >= p[3].y * p[2].x)
                outIdx = 1;
            else
                outIdx = 0;
        }
        else if (p[3].x * p[1].y < p[3].y * p[1].x)
            outIdx = 2;
        else
            outIdx = 0;

        //prev_index = index;

        if (id[outIdx] == m_tet32s[index].v[0])
            index = m_tet32s[index].n[0];
        else if (id[outIdx] == m_tet32s[index].v[1])
            index = m_tet32s[index].n[1];
        else if (id[outIdx] == m_tet32s[index].v[2])
            index = m_tet32s[index].n[2];
        else
            index = m_tet32s[index].n[3];
    }

    if (index != -1)
    {
        index = (index & 0x7FFFFFFF);
        const Face& face = *m_constrained_faces[index].face;

        const glm::vec3 *v = face.vertices;
        const glm::vec3 *n = face.normals;
        const glm::vec2 *t = face.uvs;

        const glm::vec3 e1 = v[1] - v[0];
        const glm::vec3 e2 = v[2] - v[0];
        const glm::vec3 s = ray.origin - v[0];
        const glm::vec3 q = glm::cross(s, e1);
        const glm::vec3 p = glm::cross(ray.dir, e2);
        const float f = 1.0f / glm::dot(e1, p);
        const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));

        intersection_data.position = ray.origin + f * glm::dot(e2, q) * ray.dir;
        intersection_data.normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
        intersection_data.uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
        intersection_data.tet_idx = m_constrained_faces[index].tet_idx;
        intersection_data.neighbor_tet_idx = m_constrained_faces[index].other_tet_idx;

        return true;
    }
    else
        return false;
}

bool TetMesh32::intersect(const Ray& ray, const TetFace& tet_face, IntersectionData& intersection_data)
{
    return false;
}

bool TetMesh32::intersect(const Ray& ray, const TetFace& tet_face, const int& target_tet_idx)
{
    return false;
}

bool TetMesh32::intersect_simd(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data)
{
    unsigned int id[4];
    glm::vec2 p[4];

    const float sign = copysignf(1.0f, ray.dir.z);

    const float a = -1.0f / (sign + ray.dir.z);
    const float b = ray.dir.x * ray.dir.y * a;

    const __m128 right = _mm_set_ps(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x, 0.0f);
    const __m128 up = _mm_set_ps(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y, 0.0f);
    const __m128 origin = _mm_set_ps(ray.origin.x, ray.origin.y, ray.origin.z, 0.0f);

    const int mask = 0xE1;

    for (int i = 0; i < 4; i++)
    {
        id[i] = source_tet.v[i];

        const __m128 point = (_mm_load_ps(&m_padded_points[id[i]].x));
        const __m128 new_point = _mm_sub_ps(point, origin);

        p[i].x = _mm_cvtss_f32(_mm_dp_ps(new_point, right, mask));
        p[i].y = _mm_cvtss_f32(_mm_dp_ps(new_point, up,    mask));
    }

    signed short outIdx = -1;

    if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
        outIdx = 0;
    else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
        outIdx = 1;
    else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
        outIdx = 2;
    else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
    {
        outIdx = 3;
        std::swap(id[0], id[1]);
        std::swap(p[0], p[1]);
    }
    else
        return false;

    int index = source_tet.n[outIdx];

    while (index >= 0)
    {
        id[outIdx] = id[3];
        id[3] = m_tet32s[index].x ^ id[0] ^ id[1] ^ id[2];
        //const glm::vec3 newPoint = (glm::vec3)m_padded_points[id[3]] - ray.origin;

        const __m128 point = _mm_sub_ps(_mm_load_ps(&m_padded_points[id[3]].x), origin);
        //const __m128 newPoint_simd = _mm_set_ps(newPoint.x, newPoint.y, newPoint.z, 0.0f);

        p[outIdx] = p[3];



        //p[3].x = right.x * newPoint.x + right.y * newPoint.y + right.z * newPoint.z;
        p[3].x = _mm_cvtss_f32(_mm_dp_ps(point, right, mask));
        p[3].y = _mm_cvtss_f32(_mm_dp_ps(point, up, mask));

        //p[3] = basis.project(newPoint);

        if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
        {
            if (p[3].x * p[2].y >= p[3].y * p[2].x)
                outIdx = 1;
            else
                outIdx = 0;
        }
        else if (p[3].x * p[1].y < p[3].y * p[1].x)
            outIdx = 2;
        else
            outIdx = 0;

        //prev_index = index;

        if (id[outIdx] == m_tet32s[index].v[0])
            index = m_tet32s[index].n[0];
        else if (id[outIdx] == m_tet32s[index].v[1])
            index = m_tet32s[index].n[1];
        else if (id[outIdx] == m_tet32s[index].v[2])
            index = m_tet32s[index].n[2];
        else
            index = m_tet32s[index].n[3];
    }

    if (index != -1)
    {
        index = (index & 0x7FFFFFFF);
        const Face& face = *m_constrained_faces[index].face;

        const glm::vec3 *v = face.vertices;
        const glm::vec3 *n = face.normals;
        const glm::vec2 *t = face.uvs;

        const glm::vec3 e1 = v[1] - v[0];
        const glm::vec3 e2 = v[2] - v[0];
        const glm::vec3 s = ray.origin - v[0];
        const glm::vec3 q = glm::cross(s, e1);
        const glm::vec3 p = glm::cross(ray.dir, e2);
        const float f = 1.0f / glm::dot(e1, p);
        const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));

        intersection_data.position = ray.origin + f * glm::dot(e2, q) * ray.dir;
        intersection_data.normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
        //intersection_data.uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
        intersection_data.tet_idx = m_constrained_faces[index].tet_idx;
        intersection_data.neighbor_tet_idx = m_constrained_faces[index].other_tet_idx;

        return true;
    }
    else
        return false;
}

//bool TetMesh32::intersect_simd_b(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data)
//{
//    unsigned int id[4];
//    glm::vec2 p[4];
//
//    const float sign = copysignf(1.0f, ray.dir.z);
//
//    const float a = -1.0f / (sign + ray.dir.z);
//    const float b = ray.dir.x * ray.dir.y * a;
//
//    //const __m128 right = _mm_set_ps(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x, 0.0f);
//    //const __m128 up = _mm_set_ps(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y, 0.0f);
//    const __m128 origin = _mm_set_ps(ray.origin.x, ray.origin.y, ray.origin.z, 0.0f);
//
//    const __m256 basis = _mm256_set_ps( 1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x, 0.0f, b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y, 0.0f );
//    //const __m256 origin2 = _mm256_set_ps( ray.origin.x, ray.origin.y, ray.origin.z, 0.0f , ray.origin.x, ray.origin.y, ray.origin.z, 0.0f );
//
//    for (int i = 0; i < 4; i++)
//    {
//        id[i] = source_tet.v[i];
//
//        const __m128 point = _mm_sub_ps(_mm_load_ps(&m_padded_points[id[i]].x), origin);
//
//        const __m256 point2 = _mm256_broadcast_ps(&point);
//        __m256 dot = _mm256_mul_ps(point2, basis);
//        dot = _mm256_hadd_ps(dot, dot);
//        dot = _mm256_hadd_ps(dot, dot);
//
//        p[i].x = dot.m256_f32[4];
//        p[i].y = dot.m256_f32[0];
//    }
//
//    signed short outIdx = -1;
//
//    if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
//        outIdx = 0;
//    else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
//        outIdx = 1;
//    else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
//        outIdx = 2;
//    else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
//    {
//        outIdx = 3;
//        std::swap(id[0], id[1]);
//        std::swap(p[0], p[1]);
//    }
//    else
//        return false;
//
//    int index = source_tet.n[outIdx];
//
//    while (index >= 0)
//    {
//        id[outIdx] = id[3];
//        id[3] = m_tet32s[index].x ^ id[0] ^ id[1] ^ id[2];
//        //const glm::vec3 newPoint = (glm::vec3)m_padded_points[id[3]] - ray.origin;
//
//        const __m128 point = _mm_sub_ps(_mm_load_ps(&m_padded_points[id[3]].x), origin);
//        //const __m128 newPoint_simd = _mm_set_ps(newPoint.x, newPoint.y, newPoint.z, 0.0f);
//
//        p[outIdx] = p[3];
//        //_mm256_broadcast_ps((__m128*)&m_padded_points[id[3]].x);
//        const __m256 point2 = _mm256_broadcast_ps(&point);
//        __m256 dot = _mm256_mul_ps(point2, basis);
//
//        //_mm256_dp_ps
//
//        dot = _mm256_hadd_ps(dot, dot);
//        dot = _mm256_hadd_ps(dot, dot);
//
//        //p[3].x = right.x * newPoint.x + right.y * newPoint.y + right.z * newPoint.z;
//        //p[3].x = _mm_dp_ps(point, right, mask).m128_f32[0];
//        //p[3].y = _mm_dp_ps(point, up, mask).m128_f32[0];
//
//        p[3].x = dot.m256_f32[4];
//        p[3].y = dot.m256_f32[0];
//
//        //p[3] = basis.project(newPoint);
//
//        if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
//        {
//            if (p[3].x * p[2].y >= p[3].y * p[2].x)
//                outIdx = 1;
//            else
//                outIdx = 0;
//        }
//        else if (p[3].x * p[1].y < p[3].y * p[1].x)
//            outIdx = 2;
//        else
//            outIdx = 0;
//
//        //prev_index = index;
//
//        if (id[outIdx] == m_tet32s[index].v[0])
//            index = m_tet32s[index].n[0];
//        else if (id[outIdx] == m_tet32s[index].v[1])
//            index = m_tet32s[index].n[1];
//        else if (id[outIdx] == m_tet32s[index].v[2])
//            index = m_tet32s[index].n[2];
//        else
//            index = m_tet32s[index].n[3];
//    }
//
//    if (index != -1)
//    {
//        index = (index & 0x7FFFFFFF);
//        const Face& face = *m_constrained_faces[index].face;
//
//        const glm::vec3 *v = face.vertices;
//        const glm::vec3 *n = face.normals;
//        const glm::vec2 *t = face.uvs;
//
//        const glm::vec3 e1 = v[1] - v[0];
//        const glm::vec3 e2 = v[2] - v[0];
//        const glm::vec3 s = ray.origin - v[0];
//        const glm::vec3 q = glm::cross(s, e1);
//        const glm::vec3 p = glm::cross(ray.dir, e2);
//        const float f = 1.0f / glm::dot(e1, p);
//        const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));
//
//        intersection_data.position = ray.origin + f * glm::dot(e2, q) * ray.dir;
//        intersection_data.normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
//        //intersection_data.uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
//        intersection_data.tet_idx = m_constrained_faces[index].tet_idx;
//        intersection_data.neighbor_tet_idx = m_constrained_faces[index].other_tet_idx;
//
//        return true;
//    }
//    else
//        return false;
//}

TetMesh20::TetMesh20(
    const Scene & scene,
    const bool preserve_triangles,
    const bool create_bbox,
    const float quality) :
    TetMesh(scene, preserve_triangles, create_bbox, quality)
{
    init_acceleration_data();
    compute_weight();
}

TetMesh20::TetMesh20(const Scene& scene) : TetMesh(scene)
{
    init_acceleration_data();
    compute_weight();
}

int TetMesh20::get_size_in_bytes()
{
    int size_in_bytes = 0;

    size_in_bytes += m_tet20s.size() * sizeof(Tet20);
    size_in_bytes += m_points.size() * sizeof(glm::vec3);
    size_in_bytes += m_constrained_faces.size() * sizeof(ConstrainedFace);

    return size_in_bytes;
}

void TetMesh20::init_acceleration_data()
{
    m_tet20s.resize(m_tets.size());

    for (int i = 0; i < m_tets.size(); i++)
    {
        const unsigned* v = m_tets[i].v;

        m_tet20s[i].x = v[0] ^ v[1] ^ v[2] ^ v[3];

        for (int j = 0; j < 4; ++j)
        {
            const int n = m_tets[i].n[j];

            if (m_tets[i].face_idx[j] > 0)
            {
                ConstrainedFace cf;
                cf.face = &faces[m_tets[i].face_idx[j] - 1];
                cf.tet_idx = i;
                cf.other_tet_idx = n;
                m_constrained_faces.push_back(cf);

                m_tet20s[i].n[j] = (m_constrained_faces.size() - 1) | (1 << 31);
            }
            else
                m_tet20s[i].n[j] = n;
        }
    }

    for (int i = 0; i < 4; ++i)
    {
        m_source_tet.v[i] = m_tets[0].v[i];
        m_source_tet.n[i] = m_tet20s[0].n[i];
    }

    for (int i = 0; i < m_tet20s.size(); i++)
    {
        int n[4];

        for (int j = 0; j < 4; ++j)
            n[j] = m_tet20s[i].n[j];

        for (int j = 0; j < 4; ++j)
        {
            int idx = 0;

            for (int k = 0; k < 4; ++k)
            {
                if (m_tets[i].v[j] > m_tets[i].v[k])
                    ++idx;
            }

            m_tet20s[i].n[idx] = n[j];
        }
    }

    Logger::LogWarning("constrained face count: %d", m_constrained_face_count);

    FreeAligned(m_padded_points);
    m_padded_points = (glm::vec4*)AllocAligned(m_points.size() * 16);

    for (int i = 0; i < m_points.size(); ++i)
    {
        m_padded_points[i].x = 0.0f;
        m_padded_points[i].y = m_points[i].z;
        m_padded_points[i].z = m_points[i].y;
        m_padded_points[i].w = m_points[i].x;
    }
}

int TetMesh20::find_tet(const glm::vec3& point, SourceTet & tet)
{
    int index = 0;
    Ray ray;

    glm::vec3 v[4];

    ray.origin = glm::vec3(0, 0, 0);

    for (int i = 0; i < 4; i++)
    {
        int t = m_source_tet.v[i];
        v[i] = m_points[t];
        ray.origin += v[i];
    }

    if (is_point_inside_tet(v, point))
    {
        // point is in the source tet.
        tet = m_source_tet;
        tet.idx = 0;

        //std::cout << index << std::endl;

        return tet.idx;
    }


    ray.origin *= 0.25;

    ray.dir = glm::normalize(point - ray.origin);

    glm::vec2 p[4];
    unsigned int id[4];
    int outIdx = -1;

    const Basis basis(ray.dir);

    for (int i = 0; i < 4; i++)
    {
        id[i] = m_source_tet.v[i];
        v[i] = m_points[id[i]];
        p[i].x = glm::dot(basis.right, v[i] - ray.origin);
        p[i].y = glm::dot(basis.up, v[i] - ray.origin);
    }

    if (cross(p[2], p[1]) <= 0.0 && cross(p[1], p[3]) <= 0.0 && cross(p[3], p[2]) <= 0.0)
        outIdx = 0;
    else if (cross(p[2], p[3]) <= 0.0 && cross(p[3], p[0]) <= 0.0 && cross(p[0], p[2]) <= 0.0)
        outIdx = 1;
    else if (cross(p[0], p[3]) <= 0.0 && cross(p[3], p[1]) <= 0.0 && cross(p[1], p[0]) <= 0.0)
        outIdx = 2;
    else if (cross(p[0], p[1]) <= 0.0 && cross(p[1], p[2]) <= 0.0 && cross(p[2], p[0]) <= 0.0)
    {
        outIdx = 3;
        std::swap(id[0], id[1]);
        std::swap(p[0], p[1]);
        std::swap(v[0], v[1]);
    }
    else
        return false;

    index = m_source_tet.n[outIdx];

    if (index < -1)
    {
        index = index & 0x7FFFFFFF;
        index = m_constrained_faces[index].other_tet_idx;
    }

    //int prev_index = -1;

    while (index > -1)
    {
        id[outIdx] = id[3];
        id[3] = m_tet20s[index].x ^ id[0] ^ id[1] ^ id[2]; // cache total sum for 1, 2 and 3

        v[outIdx] = v[3];
        v[3] = m_points[id[3]];

        glm::vec3 newPoint = v[3] - ray.origin;

        p[outIdx] = p[3];
        p[3].x = glm::dot(basis.right, newPoint);
        p[3].y = glm::dot(basis.up, newPoint);

        // make right or up zero
        outIdx = 0;

        if (cross(p[3], p[0]) < 0)
        {
            if ((cross(p[3], p[2]) >= 0))
                outIdx = 1;
        }
        else if (cross(p[3], p[1]) < 0)
            outIdx = 2;

        glm::vec3 n = glm::cross(v[(outIdx + 1) % 3] - v[3], v[(outIdx + 2) % 3] - v[3]);
        n = glm::normalize(n);
        float dot = glm::dot(glm::normalize(point - v[3]), n);

        //std::cout << dot << std::endl;

        if (dot < 0)
        {
            for (int i = 0; i < 4; ++i)
            {
                tet.v[i] = id[i];

                int idx = 0;
                for (int j = 0; j < 4; ++j)
                {
                    if (id[i] > id[j])
                        idx++;
                }

                tet.n[i] = m_tet20s[index].n[idx];
            }

            tet.idx = index;

            return index;
        }

        int idx = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (id[outIdx] > id[i])
                idx++;
        }

        index = m_tet20s[index].n[idx];

        if (index < -1)
        {
            index = index & 0x7FFFFFFF;
            index = m_constrained_faces[index].other_tet_idx;
        }
    }

    return -1;
}

bool TetMesh20::intersect(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data)
{
    unsigned int id[4];
    glm::vec2 p[4];

    //int prev_index;
    signed short outIdx = -1;

    const float sign = copysignf(1.0f, ray.dir.z);

    const float a = -1.0f / (sign + ray.dir.z);
    const float b = ray.dir.x * ray.dir.y * a;

    const glm::vec3 right(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x);
    const glm::vec3 up(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y);
    const glm::vec3 origin(ray.origin.x, ray.origin.y, ray.origin.z);

    const int mask = 0xE1;

    for (int i = 0; i < 4; i++)
    {
        id[i] = source_tet.v[i];
        const glm::vec3 point = m_points[id[i]] - ray.origin;
        p[i].x = glm::dot(right, point);
        p[i].y = glm::dot(up, point);
    }

    if      (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
        outIdx = 0;
    else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
        outIdx = 1;
    else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
        outIdx = 2;
    else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
    {
        outIdx = 3;
        std::swap(id[0], id[1]);
        std::swap(p[0], p[1]);
    }
    else
        return false;

    int index = source_tet.n[outIdx];
    id[outIdx] = id[3];
    p[outIdx] = p[3];

    while (index >= 0)
    {
        id[3] = m_tet20s[index].x ^ id[0] ^ id[1] ^ id[2];
        const glm::vec3 newPoint = m_points[id[3]] - ray.origin;

        p[3].x = glm::dot(right, newPoint);
        p[3].y = glm::dot(up, newPoint);

        //p[3] = basis.project(newPoint);

        if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
        {
            if (p[3].x * p[2].y >= p[3].y * p[2].x)
            {
                index = m_tet20s[index].n[(id[1] > id[0]) + (id[1] > id[2]) + (id[1] > id[3])];
                //outIdx = 1;
                id[1] = id[3];
                p[1] = p[3];
            }
            else
            {
                index = m_tet20s[index].n[(id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3])];
                //outIdx = 0;
                id[0] = id[3];
                p[0] = p[3];
            }
        }
        else if (p[3].x * p[1].y < p[3].y * p[1].x)
        {
            index = m_tet20s[index].n[(id[2] > id[0]) + (id[2] > id[1]) + (id[2] > id[3])];
            //outIdx = 2;
            id[2] = id[3];
            p[2] = p[3];
        }
        else
        {
            index = m_tet20s[index].n[(id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3])];
            //outIdx = 0;
            id[0] = id[3];
            p[0] = p[3];
        }

        //prev_index = index;

        //int idx = 0;
        //for (int i = 0; i < 4; ++i)
        //{
        //    if (id[outIdx] > id[i])
        //        idx++;
        //}

        //index = m_tet20s[index].n[idx];
    }

    if (index != -1)
    {
        index = (index & 0x7FFFFFFF);
        const Face& face = *m_constrained_faces[index].face;

        const glm::vec3 *v = face.vertices;
        const glm::vec3 *n = face.normals;
        const glm::vec2 *t = face.uvs;

        const glm::vec3 e1 = v[1] - v[0];
        const glm::vec3 e2 = v[2] - v[0];
        const glm::vec3 s = ray.origin - v[0];
        const glm::vec3 q = glm::cross(s, e1);
        const glm::vec3 p = glm::cross(ray.dir, e2);
        const float f = 1.0f / glm::dot(e1, p);
        const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));
        intersection_data.position = ray.origin + f * glm::dot(e2, q) * ray.dir;
        intersection_data.normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
        intersection_data.uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
        intersection_data.tet_idx = m_constrained_faces[index].tet_idx;
        intersection_data.neighbor_tet_idx = m_constrained_faces[index].other_tet_idx;

        return true;
    }
    else
        return false;
}

void TetMesh20::intersect4(const Ray rays[4], const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh20::intersect4_common_origin(const glm::vec3 dirs[4], const glm::vec3& origin, const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh20::intersect4_common_origin_soa(const glm::vec3 dirs[4], const glm::vec3 & origin, const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh20::intersect16_common_origin_soa(const glm::vec3 dirs[16], const glm::vec3 & origin, const SourceTet & tet, IntersectionData intersection_data[16])
{
}

bool TetMesh20::intersect(const Ray& ray, const TetFace& tet_face, IntersectionData& intersection_data)
{
    return false;
}

bool TetMesh20::intersect(const Ray& ray, const TetFace& tet_face, const int& target_tet_idx)
{
    return false;
}

TetMesh16::TetMesh16(
    const Scene& scene,
    const bool preserve_triangles,
    const bool create_bbox,
    const float quality) :
    TetMesh(scene, preserve_triangles, create_bbox, quality)
{
    init_acceleration_data();
    compute_weight();
}

TetMesh16::TetMesh16(const Scene& scene) : TetMesh(scene)
{
    init_acceleration_data();
    compute_weight();
}

int TetMesh16::get_size_in_bytes()
{
    int size_in_bytes = 0;

    size_in_bytes += m_tets.size() * sizeof(Tet16);
    size_in_bytes += m_points.size() * sizeof(glm::vec3);
    size_in_bytes += m_constrained_faces.size() * sizeof(ConstrainedFace);

    return size_in_bytes;
}

void TetMesh16::init_acceleration_data()
{
    FreeAligned(m_tet16s);
    m_tet16s = (Tet16*)AllocAligned(m_tets.size() * 16);

    std::vector<std::pair<unsigned int, int>> tet_data;
    tet_data.resize(4);

    // std::vector<bool> is_processed(m_tets.size(), false);

    std::vector<int> n(m_tets.size() * 4);

    for (int i = 0; i < m_tets.size(); i++)
    {
        const unsigned* v = m_tets[i].v;

        m_tet16s[i].x = v[0] ^ v[1] ^ v[2] ^ v[3];

        for (int j = 0; j < 4; ++j)
        {
            const int other_tet_idx = m_tets[i].n[j];

            if (m_tets[i].face_idx[j] > 0 && (other_tet_idx > i || other_tet_idx == -1))
            {
                ConstrainedFace cf;
                cf.face = &faces[m_tets[i].face_idx[j] - 1];
                cf.tet_idx = i;
                cf.other_tet_idx = other_tet_idx;
                cf.n = (m_constrained_faces.size() + 1) | (1 << 31);
                m_constrained_faces.push_back(cf);
                n[4 * i + j] = (m_constrained_faces.size() - 1) | (1 << 31);

                if (other_tet_idx > i)
                {
                    const Tet& other_tet = m_tets[other_tet_idx];

                    int k = 0;

                    for (; k < 4; ++k)
                    {
                        if (other_tet.n[k] == i)
                            break;
                    }

                    ConstrainedFace cf2;
                    cf2.face = &faces[m_tets[i].face_idx[j] - 1];
                    cf2.tet_idx = cf.other_tet_idx;
                    cf2.other_tet_idx = cf.tet_idx;
                    cf2.n = (m_constrained_faces.size() - 1) | (1 << 31);

                    m_constrained_faces.push_back(cf2);

                    n[4 * other_tet_idx + k] = (m_constrained_faces.size() - 1) | (1 << 31);

                    //cf.n = (m_constrained_faces.size()-1) | (1 << 31);

                }

                //n[j] = (m_constrained_faces.size() - 1) | (1 << 31);
            }

            if (m_tets[i].face_idx[j] < 1)
                n[4 * i + j] = m_tets[i].n[j];

            tet_data[j].first  = v[j];
            tet_data[j].second = n[4 * i + j];
        }

        if (i == 0) // init the source tet.
        {
            for (int j = 0; j < 4; ++j)
            {
                m_source_tet.v[j] = tet_data[j].first;
                m_source_tet.n[j] = tet_data[j].second;
            }
        }

        std::sort(tet_data.begin(), tet_data.end());

        for (int j = 0; j < 3; ++j)
        {
            m_tet16s[i].n[j] = tet_data[j].second ^ tet_data[j + 1].second;
        }

        // 
        m_tet16s[i].n[1] = m_tet16s[i].n[0] ^ m_tet16s[i].n[1];
        m_tet16s[i].n[2] = m_tet16s[i].n[1] ^ m_tet16s[i].n[2];
    }

    Logger::LogWarning("constrained face count: %d", m_constrained_face_count);
}

int TetMesh16::find_tet(const glm::vec3& point, SourceTet& tet)
{
    int index = 0;
    Ray ray;

    glm::vec3 v[4];

    ray.origin = glm::vec3(0, 0, 0);

    for (int i = 0; i < 4; i++)
    {
        int t = m_source_tet.v[i];
        v[i] = m_points[t];
        ray.origin += v[i];
    }

    if (is_point_inside_tet(v, point))
    {
        // point is in the source tet.
        tet = m_source_tet;
        tet.idx = 0;

        //std::cout << index << std::endl;

        return tet.idx;
    }


    ray.origin *= 0.25;

    ray.dir = glm::normalize(point - ray.origin);

    glm::vec2 p[4];
    unsigned int id[4];
    int outIdx = -1;

    const Basis basis(ray.dir);

    for (int i = 0; i < 4; i++)
    {
        id[i] = m_source_tet.v[i];
        v[i] = m_points[id[i]];
        p[i].x = glm::dot(basis.right, v[i] - ray.origin);
        p[i].y = glm::dot(basis.up, v[i] - ray.origin);
    }

    if (cross(p[2], p[1]) <= 0.0 && cross(p[1], p[3]) <= 0.0 && cross(p[3], p[2]) <= 0.0)
        outIdx = 0;
    else if (cross(p[2], p[3]) <= 0.0 && cross(p[3], p[0]) <= 0.0 && cross(p[0], p[2]) <= 0.0)
        outIdx = 1;
    else if (cross(p[0], p[3]) <= 0.0 && cross(p[3], p[1]) <= 0.0 && cross(p[1], p[0]) <= 0.0)
        outIdx = 2;
    else if (cross(p[0], p[1]) <= 0.0 && cross(p[1], p[2]) <= 0.0 && cross(p[2], p[0]) <= 0.0)
    {
        outIdx = 3;
        std::swap(id[0], id[1]);
        std::swap(p[0], p[1]);
        std::swap(v[0], v[1]);
    }
    else
        return false;

    index = m_source_tet.n[outIdx];

    int nx = 0;

    if (index < -1)
    {
        index = index & 0x7FFFFFFF;
        ConstrainedFace& cf = m_constrained_faces[index];
        index = cf.other_tet_idx;
        nx = cf.n;
    }

    int prev_index = -1;

    while (index > -1)
    {
        id[outIdx] = id[3];
        id[3] = m_tet16s[index].x ^ id[0] ^ id[1] ^ id[2]; // cache total sum for 1, 2 and 3

        v[outIdx] = v[3];
        v[3] = m_points[id[3]];

        glm::vec3 newPoint = v[3] - ray.origin;

        p[outIdx] = p[3];
        p[3].x = glm::dot(basis.right, newPoint);
        p[3].y = glm::dot(basis.up, newPoint);

        // make right or up zero
        outIdx = 0;

        if (cross(p[3], p[0]) < 0)
        {
            if ((cross(p[3], p[2]) >= 0))
                outIdx = 1;
        }
        else if (cross(p[3], p[1]) < 0)
            outIdx = 2;

        glm::vec3 n = glm::cross(v[(outIdx + 1) % 3] - v[3], v[(outIdx + 2) % 3] - v[3]);
        n = glm::normalize(n);
        float dot = glm::dot(glm::normalize(point - v[3]), n);

        //std::cout << dot << std::endl;

        if (dot < 0)
        {
            for (int i = 0; i < 4; ++i)
            {
                tet.v[i] = id[i];

                int idx2 = 0;
                for (int j = 0; j < 4; ++j)
                {
                    if (id[i] > id[j])
                        idx2++;
                }

                int idx = 0;
                for (int j = 0; j < 4; ++j)
                {
                    if (id[3] > id[j])
                        idx++;
                }

                int nxx = nx;

                //if (idx2 > idx)
                //{
                //    for (int j = idx; j < idx2; j++)
                //        nxx ^= m_tet16s[index].n[j];
                //}
                //else if (idx2 < idx)
                //{
                //    for (int j = idx2; j < idx; j++)
                //        nxx ^= m_tet16s[index].n[j];
                //}

                nxx ^= idx == 0 ? 0 : m_tet16s[index].n[idx - 1];
                nxx ^= idx2 == 0 ? 0 : m_tet16s[index].n[idx2 - 1];

                tet.n[i] = nxx;
            }

            tet.idx = index;

            return index;
        }

        int idx2 = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (id[outIdx] > id[i])
                idx2++;
        }

        int idx = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (id[3] > id[i])
                idx++;
        }

        //if (idx2 > idx)
        //{
        //    for (int i = idx; i < idx2; i++)
        //        nx ^= m_tet16s[index].n[i];
        //}
        //else
        //{
        //    for (int i = idx2; i < idx; i++)
        //        nx ^= m_tet16s[index].n[i];
        //}

        nx ^= idx == 0 ? 0 : m_tet16s[index].n[idx - 1];
        nx ^= idx2 == 0 ? 0 : m_tet16s[index].n[idx2 - 1];

        prev_index = index;

        index = nx;

        if (index < -1)
        {
            index = index & 0x7FFFFFFF;
            ConstrainedFace& cf = m_constrained_faces[index];
            index = cf.other_tet_idx;
            nx = cf.n;
        }
        else
        {
            nx = prev_index;
        }
    }

    return -1;
}

bool TetMesh16::intersect(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data)
{
    unsigned int id[4];
    glm::vec2 p[4];

    const Basis basis(ray.dir);
    int index;

    for (int i = 0; i < 4; i++)
    {
        id[i] = source_tet.v[i];
        const glm::vec3 point = m_points[id[i]] - ray.origin;
        p[i].x = glm::dot(basis.right, point);
        p[i].y = glm::dot(basis.up, point);
    }

    if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
    {
        index = source_tet.n[0];
        id[0] = id[3];
        p[0] = p[3];
    }
    else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
    {
        index = source_tet.n[1];
        id[1] = id[3];
        p[1] = p[3];
    }
    else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
    {
        index = source_tet.n[2];
        id[2] = id[3];
        p[2] = p[3];
    }
    else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
    {
        std::swap(id[0], id[1]);
        std::swap(p[0], p[1]);

        index = source_tet.n[3];
    }
    else
        return false;

    int nx = source_tet.idx;

    while (index >= 0)
    {
        id[3] = m_tet16s[index].x ^ id[0] ^ id[1] ^ id[2];
        const glm::vec3 newPoint = m_points[id[3]] - ray.origin;

        p[3].x = glm::dot(basis.right, newPoint);
        p[3].y = glm::dot(basis.up, newPoint);

        const int idx = (id[3] > id[0]) + (id[3] > id[1]) + (id[3] > id[2]);

        if(idx != 0)
            nx ^= m_tet16s[index].n[idx - 1];

        if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
        {
            if (p[3].x * p[2].y >= p[3].y * p[2].x)
            {
                const int idx2 = (id[1] > id[0]) + (id[1] > id[2]) + (id[1] > id[3]);

                if (idx2 != 0)
                    nx ^= m_tet16s[index].n[idx2 - 1];

                id[1] = id[3];
                p[1] = p[3];
            }
            else
            {
                const int idx2 = (id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3]);

                if (idx2 != 0)
                    nx ^= m_tet16s[index].n[idx2 - 1];

                id[0] = id[3];
                p[0] = p[3];
            }
        }
        else if (p[3].x * p[1].y < p[3].y * p[1].x)
        {
            const int idx2 = (id[2] > id[0]) + (id[2] > id[1]) + (id[2] > id[3]);

            if (idx2 != 0)
                nx ^= m_tet16s[index].n[idx2 - 1];

            id[2] = id[3];
            p[2] = p[3];
        }
        else
        {
            const int idx2 = (id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3]);

            if (idx2 != 0)
                nx ^= m_tet16s[index].n[idx2 - 1];

            id[0] = id[3];
            p[0] = p[3];
        }

        std::swap(nx, index);
    }

    if (index != -1)
    {
        index = (index & 0x7FFFFFFF);
        const Face& face = *m_constrained_faces[index].face;

        const glm::vec3 *v = face.vertices;
        const glm::vec3 *n = face.normals;
        const glm::vec2 *t = face.uvs;

        const glm::vec3 e1 = v[1] - v[0];
        const glm::vec3 e2 = v[2] - v[0];
        const glm::vec3 s = ray.origin - v[0];
        const glm::vec3 q = glm::cross(s, e1);
        const glm::vec3 p = glm::cross(ray.dir, e2);
        const float f = 1.0f / glm::dot(e1, p);
        const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));
        intersection_data.position = ray.origin + f * glm::dot(e2, q) * ray.dir;
        intersection_data.normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
        intersection_data.uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
        intersection_data.tet_idx = m_constrained_faces[index].tet_idx;
        intersection_data.neighbor_tet_idx = m_constrained_faces[index].other_tet_idx;

        return true;
    }
    else
        return false;
}

void TetMesh16::intersect4(const Ray rays[4], const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh16::intersect4_common_origin(const glm::vec3 dirs[4], const glm::vec3& origin, const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh16::intersect4_common_origin_soa(const glm::vec3 dirs[4], const glm::vec3 & origin, const SourceTet & tet, IntersectionData intersection_data[4])
{
}

void TetMesh16::intersect16_common_origin_soa(const glm::vec3 dirs[16], const glm::vec3 & origin, const SourceTet & tet, IntersectionData intersection_data[16])
{
}

bool TetMesh16::intersect(const Ray& ray, const TetFace& tet_face, IntersectionData& intersection_data)
{
    return false;
}

bool TetMesh16::intersect(const Ray& ray, const TetFace& tet_face, const int& target_tet_idx)
{
    return false;
}

void TetMesh16::intersect4(TetRayHit4& tet_ray_hit)
{
    unsigned int id[4][4];
    glm::vec2 p[4][4];

    const Basis basis[4] = {
        Basis(tet_ray_hit.tet_ray[0].dir),
        Basis(tet_ray_hit.tet_ray[1].dir),
        Basis(tet_ray_hit.tet_ray[2].dir),
        Basis(tet_ray_hit.tet_ray[3].dir) };

    int index[4];


    
    for (int k = 0; k < 4; ++k)
    {
        for (int i = 0; i < 4; i++)
        {
            id[k][i] = tet_ray_hit.tet_ray[k].source_tet.v[i];
            const glm::vec3 point = m_points[id[k][i]] - tet_ray_hit.tet_ray[k].origin;
            p[k][i].x = glm::dot(basis[k].right, point);
            p[k][i].y = glm::dot(basis[k].up, point);
        }
    }

    for (int k = 0; k < 4; ++k)
    {
        if (cross(p[k][2], p[k][1]) <= 0.0f && cross(p[k][1], p[k][3]) <= 0.0f && cross(p[k][3], p[k][2]) <= 0.0f)
        {
            index[k] = tet_ray_hit.tet_ray[k].source_tet.n[0];
            id[k][0] = id[k][3];
            p[k][0] = p[k][3];
        }
        else if (cross(p[k][2], p[k][3]) <= 0.0f && cross(p[k][3], p[k][0]) <= 0.0f && cross(p[k][0], p[k][2]) <= 0.0f)
        {
            index[k] = tet_ray_hit.tet_ray[k].source_tet.n[1];
            id[k][1] = id[k][3];
            p[k][1] = p[k][3];
        }
        else if (cross(p[k][0], p[k][3]) <= 0.0f && cross(p[k][3], p[k][1]) <= 0.0f && cross(p[k][1], p[k][0]) <= 0.0f)
        {
            index[k] = tet_ray_hit.tet_ray[k].source_tet.n[2];
            id[k][2] = id[k][3];
            p[k][2] = p[k][3];
        }
        else if (cross(p[k][0], p[k][1]) <= 0.0f && cross(p[k][1], p[k][2]) <= 0.0f && cross(p[k][2], p[k][0]) <= 0.0f)
        {
            std::swap(id[k][0], id[k][1]);
            std::swap(p[k][0], p[k][1]);

            index[k] = tet_ray_hit.tet_ray[k].source_tet.n[3];
        }
    }


}
