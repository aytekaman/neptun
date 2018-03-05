#include "tet_mesh.h"

#include <algorithm>
#include <ctime>
#include <iostream>
#include <fstream>
#include <set>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "basis.h"
#include "tetgen/tetgen.h"
#include "logger.h"
#include "memory.h"
#include "mesh.h"
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

    tetgenio data;

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

    int bbox_vertex_count = 8 * create_bbox;
    int bbox_facet_count = 6 * create_bbox;

    glm::vec3 bb_min(-500, -500, -500);
    glm::vec3 bb_max(500, 500, 500);

    glm::vec3 bbox_vertices[8];

    glm::vec3 rot = glm::radians((float)m_perturb_points * glm::vec3(0.01f, 0.01f, 0.01f));
    glm::mat4 r = glm::eulerAngleYXZ(rot.y, rot.x, rot.z);

    for (int i = 0; i < 8; i++)
    {
        bbox_vertices[i].x = (i / 4) % 2 ? bb_min.x : bb_max.x;
        bbox_vertices[i].y = (i / 2) % 2 ? bb_min.y : bb_max.y;
        bbox_vertices[i].z = (i / 1) % 2 ? bb_min.z : bb_max.z;

        bbox_vertices[i] = glm::vec3(r * glm::vec4(bbox_vertices[i], 1));
    }

    data.numberofpoints = (int)vertices.size() + bbox_vertex_count;
    data.pointlist = new REAL[data.numberofpoints * 3];
    data.numberoffacets = (int)triangles.size() / 3 + bbox_facet_count;
    data.facetlist = new tetgenio::facet[data.numberoffacets];

    for (size_t i = 0; i < vertices.size(); i++)
    {
        data.pointlist[3 * i + 0] = vertices[i].x;
        data.pointlist[3 * i + 1] = vertices[i].y;
        data.pointlist[3 * i + 2] = vertices[i].z;
    }

    for (size_t i = vertices.size(), j = 0; i < vertices.size() + bbox_vertex_count; i++, j++)
    {
        data.pointlist[3 * i + 0] = bbox_vertices[j].x;
        data.pointlist[3 * i + 1] = bbox_vertices[j].y;
        data.pointlist[3 * i + 2] = bbox_vertices[j].z;
    }

    //int j = 0;

    //for (int i = vertices.size(); i < vertices.size() + 8; i++)
    //{
    //	data.pointlist[3 * i + 0] = ((j % 8) > 4) ? 20 : -20;
    //	data.pointlist[3 * i + 1] = ((j % 4) > 2) ? 20 : -20;
    //	data.pointlist[3 * i + 2] = ((j % 2) > 1) ? 20 : -20;

    //	j++;
    //}


    //for (int i = 0; i < num_lights; i++)
    //{
    //	data.pointlist[3 * (i + vertices.size()) + 0] = light_positions[i].x;
    //	data.pointlist[3 * (i + vertices.size()) + 1] = light_positions[i].y;
    //	data.pointlist[3 * (i + vertices.size()) + 2] = light_positions[i].z;
    //}

    data.facetmarkerlist = new int[data.numberoffacets];



    for (int i = 0; i < data.numberoffacets - bbox_facet_count; i++)
    {
        data.facetmarkerlist[i] = i + 1;
        tetgenio::facet *f = &data.facetlist[i];
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = NULL;
        tetgenio::polygon *p = &f->polygonlist[0];
        p->numberofvertices = 3;
        p->vertexlist = new int[p->numberofvertices];
        p->vertexlist[0] = triangles[3 * i + 2];
        p->vertexlist[1] = triangles[3 * i + 1];
        p->vertexlist[2] = triangles[3 * i + 0];
    }

    for (int i = data.numberoffacets - bbox_facet_count, j = 0; i < data.numberoffacets; i++, j++)
    {
        data.facetmarkerlist[i] = 0;

        tetgenio::facet *f = &data.facetlist[i];
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = NULL;
        tetgenio::polygon *p = &f->polygonlist[0];
        p->numberofvertices = 4;
        p->vertexlist = new int[p->numberofvertices];

        int axis = j / 2;
        int side = j % 2;
        if (axis == 0)
        {
            p->vertexlist[0] = (int)vertices.size() + 0 + side * 4;
            p->vertexlist[1] = (int)vertices.size() + 1 + side * 4;
            p->vertexlist[2] = (int)vertices.size() + 3 + side * 4;
            p->vertexlist[3] = (int)vertices.size() + 2 + side * 4;
        }					    			   
        else if (axis == 1)	    			   
        {					    			   
            p->vertexlist[0] = (int)vertices.size() + 0 + side * 2;
            p->vertexlist[1] = (int)vertices.size() + 1 + side * 2;
            p->vertexlist[2] = (int)vertices.size() + 5 + side * 2;
            p->vertexlist[3] = (int)vertices.size() + 4 + side * 2;
        }					   				   
        else if (axis == 2)	   				   
        {					   				   
            p->vertexlist[0] = (int)vertices.size() + 0 + side * 1;
            p->vertexlist[1] = (int)vertices.size() + 4 + side * 1;
            p->vertexlist[2] = (int)vertices.size() + 6 + side * 1;
            p->vertexlist[3] = (int)vertices.size() + 2 + side * 1;
        }
    }

    tetgenio out;

    try
    {
        if (preserve_triangles)
        {
            char str[128];

            sprintf_s(str, 128, "q%.2fYnAf", quality);
            
            tetrahedralize(str, &data, &out);
        }	
        else
            tetrahedralize("qnnAf", &data, &out);
    }
    catch (int a)
    {
        std::cout << "Error: " << a << std::endl;
    }

    m_points.resize(out.numberofpoints);
    m_tets.resize(out.numberoftetrahedra);

    for (int i = 0; i < out.numberofpoints; i++)
    {
        m_points[i] = glm::make_vec3(&out.pointlist[3 * i]);
    }

    m_constrained_face_count = 0;

    for (int i = 0; i < out.numberoftetrahedra; i++)
    {
        m_tets[i].region_id = (int)out.tetrahedronattributelist[i];

        for (int j = 0; j < 4; j++)
        {
            m_tets[i].v[j] = out.tetrahedronlist[4 * i + j];
            m_tets[i].n[j] = out.neighborlist[4 * i + j];
            m_tets[i].face_idx[j] = out.trifacemarkerlist[out.tet2facelist[4*i+j]];

            if (m_tets[i].face_idx[j] > 0)
                ++m_constrained_face_count;

            if (m_tets[i].n[j] == -1)
                m_air_region_id = m_tets[i].region_id;
        }
    }

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
    const std::vector<SceneObject*> &scene_objects = scene.sceneObjects;

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

        for (int j = 0; j < mesh->vertexCount; j += 3)
        {
            glm::vec3 vertex = glm::vec3(m * glm::vec4(mesh->vertices[j], 1));

            Face face;

            //face.material = scene.sceneObjects[i]->material;

            face.vertices[0] = glm::vec3(m * glm::vec4(mesh->vertices[j + 0], 1));
            face.vertices[1] = glm::vec3(m * glm::vec4(mesh->vertices[j + 1], 1));
            face.vertices[2] = glm::vec3(m * glm::vec4(mesh->vertices[j + 2], 1));

            face.normals[0] = glm::vec3(r * glm::vec4(mesh->normals[j + 0], 1));
            face.normals[1] = glm::vec3(r * glm::vec4(mesh->normals[j + 1], 1));
            face.normals[2] = glm::vec3(r * glm::vec4(mesh->normals[j + 2], 1));

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

    size_in_bytes += m_tet32s.size() * sizeof(Tet32);
    size_in_bytes += m_points.size() * sizeof(glm::vec3);
    size_in_bytes += m_constrained_faces.size() * sizeof(ConstrainedFace);

    return size_in_bytes;
}

void TetMesh32::init_acceleration_data()
{
    clock_t start = clock();

    m_tet32s.resize(m_tets.size());

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
                cf.face = &faces[m_tets[i].face_idx[j]-1];
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
}

float cross(const glm::vec2& a, const glm::vec2& b) { return a.x * b.y - a.y * b.x; };

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

bool TetMesh32::raycast(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data)
{
    unsigned int id[4];
    glm::vec2 p[4];
    
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
    }
    else
        return false;
}

bool TetMesh32::raycast(const Ray& ray, const TetFace& tet_face, IntersectionData& intersection_data)
{
    return false;
}

bool TetMesh32::raycast(const Ray& ray, const TetFace& tet_face, const int& target_tet_idx)
{
    return false;
}

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

bool TetMesh20::raycast(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data)
{
    unsigned int id[4];
    glm::vec2 p[4];

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
    id[outIdx] = id[3];
    p[outIdx] = p[3];

    while (index >= 0)
    {
        
        id[3] = m_tet20s[index].x ^ id[0] ^ id[1] ^ id[2];
        const glm::vec3 newPoint = m_points[id[3]] - ray.origin;

        
        p[3].x = glm::dot(basis.right, newPoint);
        p[3].y = glm::dot(basis.up, newPoint);

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
    }
    else
        return false;
}

bool TetMesh20::raycast(const Ray& ray, const TetFace& tet_face, IntersectionData& intersection_data)
{
    return false;
}

bool TetMesh20::raycast(const Ray& ray, const TetFace& tet_face, const int& target_tet_idx)
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

    size_in_bytes += m_tet16s.size() * sizeof(Tet16);
    size_in_bytes += m_points.size() * sizeof(glm::vec3);
    size_in_bytes += m_constrained_faces.size() * sizeof(ConstrainedFace);

    return size_in_bytes;
}

void TetMesh16::init_acceleration_data()
{
    m_tet16s.resize(m_tets.size());

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

                if (idx2 > idx)
                {
                    for (int j = idx; j < idx2; j++)
                        nxx ^= m_tet16s[index].n[j];
                }
                else if (idx2 < idx)
                {
                    for (int j = idx2; j < idx; j++)
                        nxx ^= m_tet16s[index].n[j];
                }

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

        if (idx2 > idx)
        {
            for (int i = idx; i < idx2; i++)
                nx ^= m_tet16s[index].n[i];
        }
        else
        {
            for (int i = idx2; i < idx; i++)
                nx ^= m_tet16s[index].n[i];
        }

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

bool TetMesh16::raycast(const Ray& ray, const SourceTet& source_tet, IntersectionData& intersection_data)
{
    unsigned int id[4];
    glm::vec2 p[4];

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
    
    //nx = 0
    int index = source_tet.n[outIdx];
    int nx = source_tet.idx;
    

    while (index >= 0)
    {
        id[outIdx] = id[3];
        id[3] = m_tet16s[index].x ^ id[0] ^ id[1] ^ id[2];
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

        int prev_index = index;

        if (idx2 > idx)
        {
            for (int i = idx; i < idx2; i++)
                nx ^= m_tet16s[index].n[i];
        }
        else
        {
            for (int i = idx2; i < idx; i++)
                nx ^= m_tet16s[index].n[i];
        }

        index = nx;
        nx = prev_index;
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

bool TetMesh16::raycast(const Ray& ray, const TetFace& tet_face, IntersectionData& intersection_data)
{
    return false;
}

bool TetMesh16::raycast(const Ray& ray, const TetFace& tet_face, const int& target_tet_idx)
{
    return false;
}
