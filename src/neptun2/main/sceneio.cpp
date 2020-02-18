#include "sceneio.h"

#include <neptun2/util/filesystem.h>

#include <tinyxml2/tinyxml2.h>

//#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

#include <cstdio>
#include <iostream>
#include <cstring>
#include <memory>

namespace neptun
{

std::unique_ptr<Mesh> load_mesh(const char* mesh_path, const char* file_name)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, mesh_path);

    if (!err.empty()) 
    {
        std::cerr << err << std::endl;
    }

    if (!ret) 
    {
        return nullptr;
    }

    std::unique_ptr<Mesh> mesh(new Mesh(file_name));

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) 
    {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            int fv = shapes[s].mesh.num_face_vertices[f];

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++)
            {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                float vx = attrib.vertices[3ull * idx.vertex_index + 0];
                float vy = attrib.vertices[3ull * idx.vertex_index + 1];
                float vz = attrib.vertices[3ull * idx.vertex_index + 2];

                glm::vec3 vertex(vx, vy, vz);

                mesh->m_bounds_min = glm::min(mesh->m_bounds_min, vertex);
                mesh->m_bounds_max = glm::max(mesh->m_bounds_max, vertex);

                mesh->m_vertices.push_back(vertex);

                if (attrib.normals.size() > 0)
                {
                    float nx = attrib.normals[3ull * idx.normal_index + 0];
                    float ny = attrib.normals[3ull * idx.normal_index + 1];
                    float nz = attrib.normals[3ull * idx.normal_index + 2];

                    mesh->m_normals.push_back(glm::vec3(nx, ny, nz));
                }

                if (attrib.texcoords.size() > 0)
                {
                    float tx = attrib.texcoords[2ull * idx.texcoord_index + 0];
                    float ty = 1 - attrib.texcoords[2ull * idx.texcoord_index + 1];

                    mesh->m_uvs.push_back(glm::vec2(tx, ty));
                }
            }

            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];
        }
    }

    if (attrib.normals.size() == 0)
    {
        std::vector<glm::vec3>& vertices = mesh->m_vertices;

        for (size_t i = 0; i < vertices.size(); i += 3)
        {
            glm::vec3 normal = glm::cross(vertices[i + 1] - vertices[i], vertices[i + 2] - vertices[i]);
            normal = glm::normalize(normal);

            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
            mesh->m_normals.push_back(normal);
        }
    }

    return mesh;
}

// Scene Parser

SceneParser::SceneParser() : m_error_code(0), m_error_line(0)
{
}

bool SceneParser::parse(const char* file, Scene& scene)
{
    tinyxml2::XMLDocument scene_xml;
    scene_xml.LoadFile(file);

    scene.m_file_name = std::string(fs::path(file).filename());
    scene.m_path = std::string(fs::path(file).parent_path());
    
    if (scene_xml.Error())
    {
        return error(scene_xml.ErrorID(), scene_xml.ErrorStr(), scene_xml.ErrorLineNum());
    }

    // Find root element
    // Root element must be scene
    const tinyxml2::XMLElement* root_el = scene_xml.RootElement();

    if (std::strcmp(root_el->Name(), "scene") != 0)
    {
        return error(1, "Root element must be scene", root_el->GetLineNum());
    }

    // Loop over elements
    const tinyxml2::XMLElement* element = root_el->FirstChildElement();
    const tinyxml2::XMLElement* camera_element = nullptr;

    while (element != nullptr)
    {	
        if (std::strcmp(element->Name(), "camera") == 0)
        {
            if (camera_element != nullptr)
                return error(1, "Scene cannot contain more than one cameras", element->GetLineNum());

            camera_element = element;

            if (!parse_camera(camera_element, scene.m_camera))
                return false;
        }
        else if (std::strcmp(element->Name(), "scene_object") == 0) 
        {
            if (!parse_scene_object(element, scene))
                return false;
        }
        else
        {
            std::cout << "Parser warning: Unknown tag \"" << element->Name() << "\"\n";
        }
        
        element = element->NextSiblingElement();
    }

    if (camera_element == nullptr)
        return error(1, "Scene must contain a camera", root_el->GetLineNum());
    return true;
}

bool SceneParser::parse_camera(const tinyxml2::XMLElement* camera_element, Camera& camera)
{
    if (!parse_attribute(camera_element, "target", "%f %f %f", &camera.m_target.x, &camera.m_target.y, &camera.m_target.z))
        return false;

    if (!parse_attribute(camera_element, "distance", "%f", &camera.m_dist))
        return false;

    if (!parse_attribute(camera_element, "orbit", "%f %f", &camera.m_orbit.x, &camera.m_orbit.y))
        return false;

    if (!parse_attribute(camera_element, "resolution", "%zu %zu", &camera.m_resolution.x, &camera.m_resolution.y))
        return false;

    camera.update();
    return true;
}

bool SceneParser::parse_scene_object(const tinyxml2::XMLElement* element, Scene& scene)
{
    char obj_name[64];
    if (!parse_attribute(element, "name", "%64s", obj_name))
        return false;

    if (scene.get_scene_object(obj_name) != nullptr)
        return error(1, std::string("Duplicate scene objects with name ") + obj_name, element->GetLineNum());
    
    std::unique_ptr<SceneObject> obj(new SceneObject(obj_name));
    if (!parse_attribute_optional(element, "position", "%f %f %f", "0 0 0", &obj->m_pos.x, &obj->m_pos.y, &obj->m_pos.z))
        return false;

    if (!parse_attribute_optional(element, "rot", "%f %f %f", "0 0 0", &obj->m_rot.x, &obj->m_rot.y, &obj->m_rot.z))
        return false;

    if (!parse_attribute_optional(element, "scale", "%f %f %f", "1 1 1", &obj->m_scale.x, &obj->m_scale.y, &obj->m_scale.z))
        return false;

    const tinyxml2::XMLElement* material_xml = nullptr;
    const tinyxml2::XMLElement* mesh_xml = nullptr;
    const tinyxml2::XMLElement* light_xml = nullptr;

    // Loop elements
    for (const tinyxml2::XMLElement* el = element->FirstChildElement(); el != nullptr; el = el->NextSiblingElement())
    {
        if (std::strcmp(el->Name(), "light") == 0)
        {
            if (light_xml != nullptr)
                return error(1, std::string("Light component of scene object must be unique"), el->GetLineNum());

            std::unique_ptr<Light> light(new Light);

            if (!parse_attribute_optional(el, "intensity", "%f", "1", &light->m_intensity))
                return false;

            if (!parse_attribute_optional(el, "color", "%f %f %f", "1 1 1", &light->m_color.x, &light->m_color.y, &light->m_color.z))
                return false;

            obj->m_light = std::move(light);
            light_xml = el;
        }
        else if (std::strcmp(el->Name(), "material") == 0)
        {
            if (material_xml != nullptr)
                return error(1, "Material component of scene object must be unique", el->GetLineNum());

            std::unique_ptr<Material> mat(new Material);
            if (!parse_attribute_optional(el, "color", "%f %f %f", "1 1 1", &mat->m_diffuse.x, &mat->m_diffuse.y, &mat->m_diffuse.z))
                return false;

            obj->m_material = std::move(mat);
            material_xml = el;
        }
        else if (std::strcmp(el->Name(), "mesh") == 0)
        {
            if (mesh_xml != nullptr)
                return error(1, "Mesh component of scene object must be unique", el->GetLineNum());

            char mesh_filename[64];
            if (!parse_attribute(el, "filename", "%64s", mesh_filename))
                return false;

            Mesh* mesh = scene.get_mesh(mesh_filename);

            if (mesh == nullptr)
            {
                // Load mesh
                std::string mesh_path = scene.m_path + mesh_filename;
                std::unique_ptr<Mesh> new_mesh = load_mesh(mesh_path.c_str(), mesh_filename);
                mesh = new_mesh.get();

                scene.m_meshes.push_back(std::move(new_mesh));
            }

            mesh->clean();

            obj->m_mesh = mesh;
            mesh_xml = el;
        }
        else
        {
            return error(1, std::string("Invalid element \"") + el->Name() + "\" in scene object " + obj_name);
        }
    }

    scene.m_scene_objects.push_back(std::move(obj));

    return true;
}

// Helpers
bool SceneParser::error(size_t error_code, std::string error_msg, int line)
{
    m_error_code = error_code;
    m_error_line = line;
    m_error_msg = std::move(error_msg);

    return false;
}

bool SceneParser::has_attribute(const tinyxml2::XMLElement* element, const char* attr_name, const char* attr_value, bool& result)
{
    const tinyxml2::XMLAttribute* attr = element->FindAttribute(attr_name);

    if (attr == nullptr)
    {
        return error(1, std::string("Cannot found attribute ") + attr_name + " in " + element->Name(), element->GetLineNum());
    }

    result = std::strcmp(attr->Value(), attr_value) == 0;

    return true;
}
} // end of namespace neptun
