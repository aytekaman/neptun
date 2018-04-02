#include "asset_importer.h"

#include <iostream>

#include "material.h"
#include "mesh.h"
#include "sceneObject.h"
#include "texture.h"
#include "filesystem.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

Mesh * AssetImporter::ImportMesh(const char *file_name)
{
    //std::string inputfile = assets_folder_path;
    //inputfile.append(file_name);

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, file_name);

    if (!err.empty()) { // `err` may contain warning message.
        std::cerr << err << std::endl;
    }

    if (!ret) {
        exit(1);
    }

    // std::cout << "Vertex count: " << attrib.vertices.size() << std::endl;

    Mesh* mesh = new Mesh();


    //std::string tmp(wstr.begin(), wstr.end());

    mesh->m_file_name = file_name;
    mesh->m_vertex_count = 0;

    mesh->m_bounds_min = glm::vec3(1000000, 1000000, 1000000);
    mesh->m_bounds_max = glm::vec3(-1000000, -1000000, -1000000);

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                float vx = attrib.vertices[3 * idx.vertex_index + 0];
                float vy = attrib.vertices[3 * idx.vertex_index + 1];
                float vz = attrib.vertices[3 * idx.vertex_index + 2];

                glm::vec3 vertex(vx, vy, vz);

                mesh->m_bounds_min = glm::min(mesh->m_bounds_min, vertex);
                mesh->m_bounds_max = glm::max(mesh->m_bounds_max, vertex);

                mesh->m_vertices.push_back(vertex);

                if (attrib.normals.size() > 0)
                {
                    float nx = attrib.normals[3 * idx.normal_index + 0];
                    float ny = attrib.normals[3 * idx.normal_index + 1];
                    float nz = attrib.normals[3 * idx.normal_index + 2];

                    mesh->m_normals.push_back(glm::vec3(nx, ny, nz));
                }

                if (attrib.texcoords.size() > 0)
                {
                    float tx = attrib.texcoords[2 * idx.texcoord_index + 0];
                    float ty = 1 - attrib.texcoords[2 * idx.texcoord_index + 1];

                    mesh->m_uvs.push_back(glm::vec2(tx, ty));
                }

                mesh->m_vertex_count++;
            }

            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];
        }
    }

    mesh->m_face_count = mesh->m_vertex_count / 3;

    if (attrib.normals.size() == 0)
    {
        std::cout << "Calculating normals" << std::endl;

        std::vector<glm::vec3> &vertices = mesh->m_vertices;

        for (int i = 0; i < mesh->m_vertex_count; i += 3)
        {
            glm::vec3 normal = glm::cross(vertices[i + 1] - vertices[i], vertices[i + 2] - vertices[i]);
            normal = glm::normalize(normal);

            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
        }
    }

    //materials[0].te

    std::cout << file_name << " is loaded." << std::endl;

    return mesh;
}

Texture * AssetImporter::ImportTexture(const char * file_name)
{
    int w, h, comp;

    unsigned char *pixels = stbi_load(file_name, &w, &h, &comp, 3);

    if (pixels)
    {
        Texture *texture = new Texture();
        texture->pixels = pixels;
        texture->w = w;
        texture->h = h;

        return texture;
    }
    else
        return nullptr;
}

SceneObject * AssetImporter::CreateFromObj(const char * file_name)
{
    //std::string inputfile = assets_folder_path;
    //inputfile.append(file_name);
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, file_name, assets_folder_path.c_str());

    if (!err.empty()) { // `err` may contain warning message.
        std::cerr << err << std::endl;
    }

    if (!ret) {
        exit(1);
    }

    // std::cout << "Vertex count: " << attrib.vertices.size() << std::endl;

    Mesh* mesh = new Mesh();

    mesh->m_file_name = file_name;

    mesh->m_vertex_count = 0;

    mesh->m_bounds_min = glm::vec3(1000000, 1000000, 1000000);
    mesh->m_bounds_max = glm::vec3(-1000000, -1000000, -1000000);

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                float vx = attrib.vertices[3 * idx.vertex_index + 0];
                float vy = attrib.vertices[3 * idx.vertex_index + 1];
                float vz = attrib.vertices[3 * idx.vertex_index + 2];

                glm::vec3 vertex(vx, vy, vz);

                mesh->m_bounds_min = glm::min(mesh->m_bounds_min, vertex);
                mesh->m_bounds_max = glm::max(mesh->m_bounds_max, vertex);

                mesh->m_vertices.push_back(vertex);

                if (attrib.normals.size() > 0)
                {
                    float nx = attrib.normals[3 * idx.normal_index + 0];
                    float ny = attrib.normals[3 * idx.normal_index + 1];
                    float nz = attrib.normals[3 * idx.normal_index + 2];

                    mesh->m_normals.push_back(glm::vec3(nx, ny, nz));
                }

                if (attrib.texcoords.size() > 0)
                {
                    float tx = attrib.texcoords[2 * idx.texcoord_index + 0];
                    float ty = 1 - attrib.texcoords[2 * idx.texcoord_index + 1];

                    mesh->m_uvs.push_back(glm::vec2(tx, ty));
                }

                mesh->m_vertex_count++;
            }

            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];
        }
    }

    mesh->m_face_count = mesh->m_vertex_count / 3;

    if (attrib.normals.size() == 0)
    {
        std::cout << "Calculating normals" << std::endl;

        std::vector<glm::vec3> &vertices = mesh->m_vertices;

        for (int i = 0; i < mesh->m_vertex_count; i += 3)
        {
            glm::vec3 normal = glm::cross(vertices[i + 1] - vertices[i], vertices[i + 2] - vertices[i]);
            normal = glm::normalize(normal);

            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
        }
    }

    std::cout << file_name << " is loaded." << std::endl;

    std::string name = fs::path(file_name).stem();

    SceneObject *scene_object = new SceneObject(name);

    scene_object->mesh = mesh;
    Material *material = new Material();
    scene_object->material = material;

    std::cout << materials.size() << std::endl;

    if (materials.size() > 0)
    {
        fs::path tex_path(materials[0].diffuse_texname.c_str());
        name = tex_path.stem();
        std::string ext = tex_path.extension();

        std::string texture_file_name = assets_folder_path;
        texture_file_name.append(name).append(ext);

        material->texture = ImportTexture(texture_file_name.c_str());

        std::cout << "Dif: " << texture_file_name << std::endl;
    }

    return scene_object;
}

std::string AssetImporter::assets_folder_path = ".\\Assets\\";
