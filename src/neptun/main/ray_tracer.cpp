#define NOMINMAX
#include "ray_tracer.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <thread>

#include "gl3w/include/GL/gl3w.h"
#include "GLFW/glfw3.h"

#include "glm/trigonometric.hpp"
#include "glm/gtc/constants.hpp"

//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "stb_image_write.h"

#include "asset_importer.h"
#include "bvh.h"
#include "color.h"
#include "image.h"
#include "kd_tree.h"
#include "light.h"
#include "logger.h"
#include "material.h"
#include "neptun/util/timer.h"
#include "ray.h"
#include "scene.h"
#include "stats.h"
#include "tet_mesh.h"
#include "texture.h"
#include <neptun/math/tangent_frame.h>

RayTracer::RayTracer()
{
    m_rendered_image = new Image(m_resolution.x, m_resolution.y);
    m_visited_tets_image = new Image(m_resolution.x, m_resolution.y);
    m_locality_image     = new Image(m_resolution.x, m_resolution.y);
    stats.set_size(m_resolution);

    thread_count = std::thread::hardware_concurrency();
    Logger::Log("Number of threads: %d", thread_count);


    m_samplers.clear();

    for (size_t i = 0; i < m_resolution.x * m_resolution.y; ++i)
    {
        m_samplers.emplace_back(i);
    }
}

void RayTracer::Render(Scene & scene, const bool is_diagnostic)
{
    //TetMesh& tet_mesh = *scene.tet_mesh;

    glm::vec3 camTarget = scene.camTarget;
    glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));

    dir = dir * glm::cos(scene.camOrbitX);

    dir.y = glm::sin(scene.camOrbitX);

    glm::vec3 cam_pos = camTarget + dir * scene.camDist;

    Ray ray;
    ray.origin = cam_pos;
    SourceTet source_tet;

    int tet_index = 0;

    if(scene.tet_mesh)
        tet_index = scene.tet_mesh->find_tet(cam_pos, source_tet);

    if (tet_index < 0)
        return;

    std::vector<LightInfo> lightInfos;

    for (int i = 0; i < scene.sceneObjects.size(); i++)
    {
        if (scene.sceneObjects[i]->light)
        {
            LightInfo li;
            li.pos = scene.sceneObjects[i]->pos;
            li.color = scene.sceneObjects[i]->light->color;
            li.intensity = scene.sceneObjects[i]->light->intensity;

            SourceTet dummy_source_tet;

            if (scene.tet_mesh)
                li.tet_index = scene.tet_mesh->find_tet(li.pos, dummy_source_tet);
            li.point_index = scene.sceneObjects[i]->light->point_index;
            lightInfos.push_back(li);
        }
    }

    std::thread **threads = new std::thread*[thread_count];

    job_index = thread_count;


    Timer timer;
    timer.start();

    for (int i = 0; i < thread_count; i++)
    {
        if (integrator == Integrator::RayCasting)
            threads[i] = new std::thread(&RayTracer::Raytrace_worker, this, std::ref(scene), source_tet, i, lightInfos, is_diagnostic);
        else 
            threads[i] = new std::thread(&RayTracer::Pathtrace_worker, this, std::ref(scene), source_tet, i, lightInfos, is_diagnostic);
    }

    for (int i = 0; i < thread_count; i++)
    {
        threads[i]->join();
        delete threads[i];
    }

    timer.stop();

    delete[] threads;

    last_render_time = timer.seconds();

    Stats::add_render_time(last_render_time);

    avg_test_count = 0;
    L1_count = 0;

    for (int i = 0; i < thread_count; i++)
    {
        avg_test_count += traversed_tetra_count[i] / (float)thread_count;
        L1_count += L1_hit_count[i];
    }
}

//#define NOMINMAX

void RayTracer::Raytrace_worker(Scene& scene, SourceTet source_tet, int thread_idx, std::vector<LightInfo> lightInfos, bool is_diagnostic)
{
    //TetMesh& tet_mesh = *scene.tet_mesh;

    const glm::vec3 camTarget = scene.camTarget;
    glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));

    dir = dir * glm::cos(scene.camOrbitX);
    dir.y = glm::sin(scene.camOrbitX);

    glm::vec3 cam_pos = camTarget + dir * scene.camDist;

    const glm::vec3 forward = glm::normalize(scene.camTarget - cam_pos);
    const glm::vec3 right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), forward));
    const glm::vec3 down = glm::cross(forward, right);

    const float aspect = (float)m_resolution.x / m_resolution.y;
    const float scale_y = glm::tan(glm::pi<float>() / 8);

    const glm::vec3 top_left = cam_pos + forward - down * scale_y - right * scale_y * aspect;
    const glm::vec3 right_step = (right * scale_y * 2.0f * aspect) / (float)m_resolution.x;
    const glm::vec3 down_step = (down * scale_y * 2.0f) / (float)m_resolution.y;

    Ray ray(cam_pos);
    ray.tet_idx = source_tet.idx;

    int total_test_count = 0;
    int total_L1_hit_count = 0;

    const int tile_count_x = (m_resolution.x + tile_size - 1) / tile_size;
    const int tile_count_y = (m_resolution.y + tile_size - 1) / tile_size;
    const int max_job_index = tile_count_x * tile_count_y;

    int idx = thread_idx;

    while (idx < max_job_index)
    {
        glm::ivec2 rect_min = glm::ivec2((idx % tile_count_x) * tile_size, (idx / tile_count_x) * tile_size);
        glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);

        rect_max = (glm::min)(rect_max, m_resolution);

        for (int j = rect_min.y; j < rect_max.y; j++)
        {
            for (int i = rect_min.x; i < rect_max.x; i++)
            {
                ray.dir = glm::normalize(top_left + right_step * (float)i + down_step * (float)j - ray.origin);

                glm::vec3 pos, normal, color;
                glm::vec2 uv;
                Face face;

                DiagnosticData diagnostic_data;
                diagnostic_data.total_tet_distance = 0;
                diagnostic_data.visited_node_count = 0;
                diagnostic_data.L1_hit_count = 0;

                // int tet_index_copy = tet_index;

                bool hit = false;

                IntersectionData intersection_data;

                if (method == Method::Default ||
                    method == Method::Fast_basis ||
                    method == Method::ScTP);
                    //ray.tet_idx = 0;
                else
                    ray.tMax = 100000000;

                if (is_diagnostic)
                {
                    if (method == Method::Default || method == Method::Fast_basis || method == Method::ScTP)
                        hit = scene.tet_mesh->intersect_stats(ray, source_tet, intersection_data, diagnostic_data);
                    else if (method == Method::Kd_tree)
                        hit = scene.kd_tree->Intersect_stats(ray, intersection_data, diagnostic_data);
                    else if (method == Method::BVH_pbrt)
                        hit = scene.bvh->Intersect_stats(ray, intersection_data, diagnostic_data);
                }

                else
                {
                    if (method == Method::Default)
                        hit = scene.tet_mesh->intersect(ray, source_tet, intersection_data);
                    else if (method == Method::DefaultSimd)
                        hit = scene.tet_mesh->intersect_simd(ray, source_tet, intersection_data);
                    //else if (method == Method::ScTP)
                    //    hit = scene.tet_mesh->intersect_sctp(ray, intersection_data);
                    //else if (method == Method::Fast_basis)
                    //    hit = scene.tet_mesh->intersect_optimized_basis(ray, intersection_data);
                    else if (method == Method::Kd_tree)
                        hit = scene.kd_tree->Intersect(ray, intersection_data);
                    else if (method == Method::BVH_embree)
                        hit = scene.bvh_embree->Intersect(ray, intersection_data);
                    else if (method == Method::BVH_pbrt)
                        hit = scene.bvh->Intersect(ray, intersection_data);
                }

                if (hit)
                {
                    
                    color = glm::vec3();

                    for (int light_idx = 0; light_idx < lightInfos.size(); light_idx++)
                    {
                        Ray shadow_ray(intersection_data.position, glm::normalize(lightInfos[light_idx].pos - intersection_data.position));

                        // check normals before shooting
                        //if (!shadows || scene.tet_mesh->intersect(shadow_ray, intersection_data.tet_idx, lightInfos[light_idx].tet_index))
                        {
                            glm::vec3 to_light = glm::normalize(lightInfos[light_idx].pos - intersection_data.position);
                            float diffuse = glm::clamp(glm::dot(intersection_data.normal, to_light), 0.0f, 1.0f);
                            //diffuse = 1.0f;
                            color += lightInfos[light_idx].color * diffuse * lightInfos[light_idx].intensity;
                        }
                    }

                    //color = glm::clamp(color, 0.0f, 1.0f);

                    //float u = (glm::atan(pos.x, pos.z) / glm::pi<float>()) * 0.5f + 0.5f;
                    //float v = 0.5 - (glm::atan(pos.y, glm::sqrt(pos.x * pos.x + pos.z * pos.z)) / glm::pi<float>());

                    //glm::vec3 d(face.material->diffuse);

                    //if (face.material->texture)
                    //	d = face.material->texture->GetPixelBilinear(uv.x, uv.y) * face.material->diffuse;

                    //color.r *= d.r;
                    //color.g *= d.g;
                    //color.b *= d.b;
                }
                else
                    color = glm::vec3(0.1, 0.1, 0.1);

                glm::ivec2 p_idx(i, j);

                if (is_diagnostic)
                {
                    if (scene.tet_mesh)
                    {
                        total_L1_hit_count += diagnostic_data.L1_hit_count;

                        stats.set(p_idx, diagnostic_data.visited_node_count);

                        float avg_locality = diagnostic_data.total_tet_distance / diagnostic_data.visited_node_count;
                        float scaled_avg_locality = (avg_locality / scene.tet_mesh->m_tets.size()) * 2.0f;
                        glm::vec3 avg_locality_color = Color::jet(scaled_avg_locality);
                        //m_locality_image->set_pixel((m_resolution.y - i - 1), (m_resolution.x - j - 1), avg_locality_color * 255.0f);
                    }

                    float scaled_visited_tet_count = diagnostic_data.visited_node_count / 256.0f;
                    glm::vec3 visited_tet_count_color = Color::jet(scaled_visited_tet_count);
                    m_visited_tets_image->set_pixel(i, j, visited_tet_count_color * 255.0f);

                    total_test_count += diagnostic_data.visited_node_count;
                }

                m_rendered_image->set_pixel(p_idx.x, p_idx.y, glm::vec3(color.z, color.y, color.x) * 255.0f);
            }
        }

        idx = job_index++;
    }

    traversed_tetra_count[thread_idx] = total_test_count / ((m_resolution.x * m_resolution.y) / (float)thread_count);
    L1_hit_count[thread_idx] = total_L1_hit_count;
}

void RayTracer::Pathtrace_worker(
    Scene& scene,
    SourceTet source_tet,
    int thread_idx,
    std::vector<LightInfo> lightInfos,
    bool is_diagnostic)
{
    // Init camera
    const glm::vec3 camTarget = scene.camTarget;
    glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));

    dir = dir * glm::cos(scene.camOrbitX);
    dir.y = glm::sin(scene.camOrbitX);

    glm::vec3 cam_pos = camTarget + dir * scene.camDist;

    const glm::vec3 forward = glm::normalize(scene.camTarget - cam_pos);
    const glm::vec3 right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), forward));
    const glm::vec3 down = glm::cross(forward, right);

    const float aspect = (float)m_resolution.x / m_resolution.y;
    const float scale_y = glm::tan(glm::pi<float>() / 8);

    const glm::vec3 top_left = cam_pos + forward - down * scale_y - right * scale_y * aspect;
    const glm::vec3 right_step = (right * scale_y * 2.0f * aspect) / (float)m_resolution.x;
    const glm::vec3 down_step = (down * scale_y * 2.0f) / (float)m_resolution.y;

    const int tile_count_x = (m_resolution.x + tile_size - 1) / tile_size;
    const int tile_count_y = (m_resolution.y + tile_size - 1) / tile_size;
    const int max_job_index = tile_count_x * tile_count_y;

    int idx = thread_idx;

    size_t total_visited_tet_count = 0;

    // For each tile
    while (idx < max_job_index)
    {
        Sampler& sampler = m_samplers[idx];

        glm::ivec2 rect_min = glm::ivec2((idx % tile_count_x) * tile_size, (idx / tile_count_x) * tile_size);
        glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);

        rect_max = (glm::min)(rect_max, m_resolution);

        // For each pixel
        for (size_t j = rect_min.y; j < rect_max.y; j++)
        {
            for (size_t i = rect_min.x; i < rect_max.x; i++)
            {
                glm::u64vec2 pixel(i, j);
                
                glm::vec3 total_radiance(0.f);
                float radiance_weight = 0.f;
                size_t pixel_visited_tet_count = 0;

                for (size_t s = 0; s < m_sample_per_pixel; ++s)
                {
                    Sampler& sampler = m_samplers[j * m_resolution.x + i];
                    const glm::vec2 pixel_sample = sampler.next_vec2() + glm::vec2(pixel);
                    //const glm::vec2 pixel_sample = glm::vec2(pixel);

                    // Path tracing :D
                    glm::vec3 radiance(0.f);
                    glm::vec3 beta(1.f);  // Radiance contribution (pbrt calls it beta)
                    size_t cur_depth = 0;

                    SourceTet path_source_tet = source_tet;
                    Ray ray(cam_pos);
                    ray.dir = glm::normalize(
                        top_left + right_step * pixel_sample.x + down_step * pixel_sample.y - ray.origin);
            
           
                    if (method == Method::Default)
                    {
                        ray.tet_idx = source_tet.idx;
                    }
                    else
                    {
                        ray.tMax = 100000000;
                    }

                    // Path trace loop
                    while (cur_depth < m_max_depth) 
                    {
                        IntersectionData isect;
                        bool hit = false;

                        if (is_diagnostic)
                        {
                            DiagnosticData diag;
                            diag.visited_node_count = 0;

                            hit = scene.bvh->Intersect_stats(ray, isect, diag);

                            pixel_visited_tet_count += diag.visited_node_count;
                        }
                        else // not diagnostic
                        {
                            if (method == Method::Default)
                                hit = scene.tet_mesh->intersect(ray, path_source_tet, isect);
                            else if (method == Method::DefaultSimd)
                                hit = scene.tet_mesh->intersect_simd(ray, path_source_tet, isect);
                            else if (method == Method::BVH_embree)
                                hit = scene.bvh_embree->Intersect(ray, isect);
                            else if (method == Method::BVH_pbrt)        
                                hit = scene.bvh->Intersect(ray, isect);
                            else if (method == Method::Kd_tree)
                                hit = scene.kd_tree->Intersect(ray, isect);
                        }

                        if (hit == false)
                        {
                            //radiance += beta * glm::vec3(0.1f); // background radiance
                            radiance += beta * glm::vec3(1.f);
                            
                            break;
                        }
                        

                        radiance += beta * isect.material->le;
                        
                        glm::vec3 wo = -ray.dir;
                        glm::vec3 normal = glm::normalize(isect.normal);
                        if (glm::dot(wo, normal) < 0)
                            normal = -normal;

                        // Create tangent frame
                        TangentFrame frame(normal);
                        const glm::vec3 t_wo = frame.to_local(wo);

                        // Evaluate brdf
                        float pdf;
                        glm::vec3 t_wi;
                        t_wi = sampler.sample_hemisphere(pdf);
                        const glm::vec3 wi = frame.to_global(t_wi);
                        const auto f = glm::vec3(glm::one_over_pi<float>());

                        if (pdf == 0.f)
                            break;

                        const glm::vec3 diff = isect.material->diffuse;
                        beta *= diff * f * (glm::abs(glm::dot(normal, glm::normalize(wi)))) / pdf;
                       
                        // ROULETTE
                        //https://computergraphics.stackexchange.com/questions/2316/is-russian-roulette-really-the-answer
                        //https://github.com/tunabrain/tungsten/blob/bda107574eb82edaef34ce0174c981340f5c1b0f/src/core/integrators/path_tracer/PathTracer.cpp
                   
                        const float roulette_pdf = glm::max(beta.x, glm::max(beta.y, beta.z));

                        if (cur_depth > 2 && roulette_pdf < 0.1f)
                        {
                            if (sampler.next_float() < roulette_pdf)
                            {
                                beta /= roulette_pdf;
                            }
                            else
                            {
                                break;
                            }
                        }

                        // Create new ray
                        Ray new_ray;
                        new_ray.origin = isect.position + (normal * 0.00001f);
                        new_ray.dir = glm::normalize(wi);
                        
                        if (method == Method::Default || method == Method::DefaultSimd)
                        {
                            new_ray.tet_idx = isect.tet_idx;

                            path_source_tet.idx = isect.neighbor_tet_idx;
                            TetMesh32* tm = dynamic_cast<TetMesh32*>(scene.tet_mesh);
                            for (int i = 0; i < 4; ++i)
                            {
                                path_source_tet.v[i] = scene.tet_mesh->m_tets[isect.tet_idx].v[i];
                                //path_source_tet.n[i] = scene.tet_mesh->m_tets[isect.tet_idx].n[i];
                                path_source_tet.n[i] = tm->m_tet32s[isect.tet_idx].n[i];
                            }
                        }
                        else
                        {
                            new_ray.tMax = 100000000;
                        }

                        ray = new_ray;
                        ++cur_depth;
                    } // end of while (cur_dept < MAX_DEPTH)

                    total_radiance += radiance;
                    radiance_weight += 1.f;
                } 

                const glm::vec3 clamped_color = glm::clamp((total_radiance / radiance_weight), { 0.f }, { 1.f });
             
                m_rendered_image->set_pixel(i, j, clamped_color * 255.f);

                if (is_diagnostic)
                {
                    float scaled_visited_tet_count = pixel_visited_tet_count / 256.0f;
                    scaled_visited_tet_count /= float(m_sample_per_pixel);
                    
                    stats.set(pixel, pixel_visited_tet_count / m_sample_per_pixel);

                    glm::vec3 visited_tet_count_color = Color::jet(scaled_visited_tet_count);
                    m_visited_tets_image->set_pixel(i, j, visited_tet_count_color * 255.0f);

                    total_visited_tet_count += pixel_visited_tet_count;
                }
            }
        }

        idx = job_index++;
    }

    traversed_tetra_count[thread_idx] = total_visited_tet_count / (((m_resolution.x * m_resolution.y * float(m_sample_per_pixel)) / (float)thread_count));
}

void RayTracer::save_to_disk(const char * file_name, ImageType image_type)
{
    if (image_type == ImageType::Locality)
        m_locality_image->save_to_disk(file_name);
    else if (image_type == ImageType::Tet_cost)
        m_visited_tets_image->save_to_disk(file_name);
    else if (image_type == ImageType::Render)
        m_rendered_image->save_to_disk(file_name);
}

void RayTracer::set_resolution(const glm::ivec2& resolution)
{
    if (m_resolution == resolution)
        return;

    m_resolution = resolution;

    delete m_rendered_image;
    delete m_locality_image;
    delete m_visited_tets_image;

    m_rendered_image = new Image(m_resolution.x, m_resolution.y);
    m_locality_image = new Image(m_resolution.x, m_resolution.y);
    m_visited_tets_image = new Image(m_resolution.x, m_resolution.y);

    stats.set_size(resolution);

    m_samplers.clear();

    for (size_t i = 0; i < m_resolution.x * m_resolution.y; ++i)
    {
        m_samplers.emplace_back(i);
    }
}
