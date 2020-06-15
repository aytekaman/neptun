#include "ray_tracer.h"

#include "gl3w/include/GL/gl3w.h"
#include "GLFW/glfw3.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <thread>

#include "glm/trigonometric.hpp"

//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "stb_image_write.h"

#include "asset_importer.h"
#include "bvh.h"
#include "image.h"
#include "kd_tree.h"
#include "light.h"
#include "logger.h"
#include "material.h"
#include "scene.h"
#include "stats.h"
#include "texture.h"

#include "color.h"

#include "glm/gtc/constants.hpp"

RayTracer::RayTracer()
{
    m_rendered_image = new Image(m_resolution.x, m_resolution.y);
    m_visited_tets_image = new Image(m_resolution.x, m_resolution.y);
    m_locality_image = new Image(m_resolution.x, m_resolution.y);
    stats.set_size(m_resolution);

    thread_count = std::thread::hardware_concurrency();
    Logger::Log("Number of threads: %d", thread_count);
    m_stream_count = thread_count;
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

    if (scene.tet_mesh)
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

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    for (int i = 0; i < thread_count; i++)
        threads[i] = new std::thread(&RayTracer::Raytrace_worker, this, std::ref(scene), source_tet, i, lightInfos, is_diagnostic);

    for (int i = 0; i < thread_count; i++)
    {
        threads[i]->join();
        delete threads[i];
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    delete[] threads;

    last_render_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1e3f;

    Stats::add_render_time(last_render_time);

    printf("Rendered in %.3f seconds. (%.1f FPS)\n", last_render_time, 1 / last_render_time);

    avg_test_count = 0;
    L1_count = 0;

    for (int i = 0; i < thread_count; i++)
    {
        avg_test_count += traversed_tetra_count[i] / (float)thread_count;
        L1_count += L1_hit_count[i];
    }

    //glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, resolution.x, resolution.y, GL_BGR, GL_UNSIGNED_BYTE, pixels);
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
                    method == Method::ScTP)
                    ray.tet_idx = 0;
                else
                    ray.tMax = 100000000;

                if (is_diagnostic)
                {
                    if (method == Method::Default || method == Method::Fast_basis || method == Method::ScTP)
                        hit = scene.tet_mesh->intersect_stats(ray, source_tet, intersection_data, diagnostic_data);
                    else if (method == Method::ScTP)
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
                    else if (method == Method::ScTP)
                        hit = scene.tet_mesh->intersect(ray, source_tet, intersection_data);
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
                    //color = { 1.0, 0.0, 1.0 };

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


//void RayTracer::asynch_ray_traverse_worker_gpu(Scene & scene, SourceTet source_tet, int thread_idx, std::vector<LightInfo> lightInfos, bool is_diagnostic)
//{
//    //TetMesh& tet_mesh = *scene.tet_mesh;
//	//Init rays
//    const glm::vec3 camTarget = scene.camTarget;
//    glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));
//
//    dir = dir * glm::cos(scene.camOrbitX);
//    dir.y = glm::sin(scene.camOrbitX);
//
//    glm::vec3 cam_pos = camTarget + dir * scene.camDist;
//
//    const glm::vec3 forward = glm::normalize(scene.camTarget - cam_pos);
//    const glm::vec3 right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), forward));
//    const glm::vec3 down = glm::cross(forward, right);
//
//    const float aspect = (float)m_resolution.x / m_resolution.y;
//    const float scale_y = glm::tan(glm::pi<float>() / 8);
//
//    const glm::vec3 top_left = cam_pos + forward - down * scale_y - right * scale_y * aspect;
//    const glm::vec3 right_step = (right * scale_y * 2.0f * aspect) / (float)m_resolution.x;
//    const glm::vec3 down_step = (down * scale_y * 2.0f) / (float)m_resolution.y;
//
//    Ray ray(cam_pos);
//
//    int total_test_count = 0;
//    int total_L1_hit_count = 0;
//
//
//    const int tile_count_x = (m_resolution.x + tile_size - 1) / tile_size;
//    const int tile_count_y = (m_resolution.y + tile_size - 1) / tile_size;
//    const int max_job_index = tile_count_x * tile_count_y;
//
//    int idx = thread_idx;
//    unsigned rays_index = 0;
//
//    while (idx < max_job_index)
//    {
//        glm::ivec2 rect_min = glm::ivec2((idx % tile_count_x) * tile_size, (idx / tile_count_x) * tile_size);
//        glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);
//
//        rect_max = (glm::min)(rect_max, m_resolution);
//
//        rays_index = 0;
//
//        for (int j = rect_min.y; j < rect_max.y; j++)
//        {
//            for (int i = rect_min.x; i < rect_max.x; i++)
//            {
//                ray.dir = glm::normalize(top_left + right_step * (float)i + down_step * (float)j - ray.origin);
//                ray.source_tet = source_tet;
//
//                glm::vec3 pos, normal;
//                glm::vec2 uv;
//                Face face;
//
//                DiagnosticData diagnostic_data;
//                diagnostic_data.total_tet_distance = 0;
//                diagnostic_data.visited_node_count = 0;
//                diagnostic_data.L1_hit_count = 0;
//
//                // int tet_index_copy = tet_index;
//
//                if (method == Method::Default ||
//                    method == Method::Fast_basis ||
//                    method == Method::ScTP)
//                    ray.tet_idx = 0;
//                else
//                    ray.tMax = 100000000;
//
//                m_rays[rays_index++ + idx * tile_size * tile_size] = ray;
//				//End of Ray init.
//
//                /*if (is_diagnostic)
//                {
//                    if (method == Method::Default || method == Method::Fast_basis || method == Method::ScTP)
//                        hit = scene.tet_mesh->intersect_stats(ray, source_tet, intersection_data, diagnostic_data);
//                    else if (method == Method::Kd_tree)
//                        hit = scene.kd_tree->Intersect_stats(ray, intersection_data, diagnostic_data);
//                    else if (method == Method::BVH_pbrt)
//                        hit = scene.bvh->Intersect_stats(ray, intersection_data, diagnostic_data);
//                }
//
//                else
//                {
//                    if (method == Method::Default)
//                        hit = scene.tet_mesh->intersect(ray, source_tet, intersection_data);
//                    else if (method == Method::DefaultSimd)
//                        hit = scene.tet_mesh->intersect_simd(ray, source_tet, intersection_data);
//                    else if (method == Method::Kd_tree)
//                        hit = scene.kd_tree->Intersect(ray, intersection_data);
//                    else if (method == Method::BVH_embree)
//                        hit = scene.bvh_embree->Intersect(ray, intersection_data);
//                    else if (method == Method::BVH_pbrt)
//                        hit = scene.bvh->Intersect(ray, intersection_data);
//                }*/
//            }
//        }
//        if (idx % (max_job_index / m_stream_count) == 0 && idx != 0)//Ray traversal
//        {
//            int tetmesh_type = 0;
//            if (dynamic_cast<TetMesh20 *>(scene.tet_mesh) != nullptr)
//                tetmesh_type = 1;
//            else if (dynamic_cast<TetMesh16 *>(scene.tet_mesh) != nullptr)
//                tetmesh_type = 2;
//            //printf("idt-call: %d\n", idx / (max_job_index / thread_count) - 1);
//            int stream_num = idx / (max_job_index / m_stream_count) - 1;
//            traverse_rays_gpu(m_rays, m_resolution.x * m_resolution.y, m_stream_count, tetmesh_type, stream_num, m_intersect_data);
//            draw_intersectiondata(stream_num * (max_job_index / m_stream_count), idx, lightInfos);
//        }
//        idx = job_index++;
//    }
//
//    if (idx == max_job_index)
//    {
//        int tetmesh_type = 0;
//        if (dynamic_cast<TetMesh20 *>(scene.tet_mesh) != nullptr)
//            tetmesh_type = 1;
//        else if (dynamic_cast<TetMesh16 *>(scene.tet_mesh) != nullptr)
//            tetmesh_type = 2;
//        //printf("idt-call: %d\n", idx / (max_job_index / thread_count) - 1);
//        int stream_num = idx / (max_job_index / m_stream_count) - 1;
//        traverse_rays_gpu(m_rays, m_resolution.x * m_resolution.y, m_stream_count, tetmesh_type, stream_num, m_intersect_data);//End of Ray traversal
//        draw_intersectiondata(stream_num * (max_job_index / m_stream_count), idx, lightInfos);//intersection data is drawn
//    }
//}
////DOES NOT WORK !!!
//void RayTracer::asynch_ray_cast_worker_gpu(Scene& scene, SourceTet source_tet, int thread_idx, std::vector<LightInfo> lightInfos, bool is_diagnostic)
//{
//	const int tile_count_x = (m_resolution.x + tile_size - 1) / tile_size;
//	const int tile_count_y = (m_resolution.y + tile_size - 1) / tile_size;
//	const int max_job_index = tile_count_x * tile_count_y;
//
//	int idx = thread_idx;
//
//	while (idx < max_job_index)
//	{
//		if (idx % (max_job_index / m_stream_count) == 0 && idx != 0)//Ray traversal
//		{
//			int tetmesh_type = 0;
//			if (dynamic_cast<TetMesh20*>(scene.tet_mesh) != nullptr)
//				tetmesh_type = 1;
//			else if (dynamic_cast<TetMesh16*>(scene.tet_mesh) != nullptr)
//				tetmesh_type = 2;
//			//printf("idt-call: %d\n", idx / (max_job_index / thread_count) - 1);
//			int stream_num = idx / (max_job_index / m_stream_count) - 1;
//			cast_rays_gpu(scene, source_tet, m_resolution, tile_size, m_stream_count, stream_num, tetmesh_type, m_intersect_data);
//			draw_intersectiondata(stream_num * (max_job_index / m_stream_count), idx, lightInfos);
//		}
//		idx = job_index++;
//	}
//	if (idx == max_job_index)
//	{
//		int tetmesh_type = 0;
//		if (dynamic_cast<TetMesh20*>(scene.tet_mesh) != nullptr)
//			tetmesh_type = 1;
//		else if (dynamic_cast<TetMesh16*>(scene.tet_mesh) != nullptr)
//			tetmesh_type = 2;
//		//printf("idt-call: %d\n", idx / (max_job_index / thread_count) - 1);
//		int stream_num = idx / (max_job_index / m_stream_count) - 1;
//		cast_rays_gpu(scene, source_tet, m_resolution, tile_size, m_stream_count, tetmesh_type, stream_num, m_intersect_data);
//		draw_intersectiondata_rowmajor(stream_num * (max_job_index / m_stream_count), idx, lightInfos);
//	}
//}
void RayTracer::prepare_rays_gpu(Scene & scene, SourceTet source_tet, int thread_idx, bool is_diagnostic)
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

    int total_test_count = 0;
    int total_L1_hit_count = 0;


    const int tile_count_x = (m_resolution.x + tile_size - 1) / tile_size;
    const int tile_count_y = (m_resolution.y + tile_size - 1) / tile_size;
    const int max_job_index = tile_count_x * tile_count_y;

    int idx = thread_idx;
    unsigned rays_index = 0;

    while (idx < max_job_index)
    {
        glm::ivec2 rect_min = glm::ivec2((idx % tile_count_x) * tile_size, (idx / tile_count_x) * tile_size);
        glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);

        rect_max = (glm::min)(rect_max, m_resolution);

        rays_index = 0;

        for (int j = rect_min.y; j < rect_max.y; j++)
        {
            for (int i = rect_min.x; i < rect_max.x; i++)
            {
                ray.dir = glm::normalize(top_left + right_step * (float)i + down_step * (float)j - ray.origin);
                ray.source_tet = source_tet;

                DiagnosticData diagnostic_data;
                diagnostic_data.total_tet_distance = 0;
                diagnostic_data.visited_node_count = 0;
                diagnostic_data.L1_hit_count = 0;

                // int tet_index_copy = tet_index;

                if (method == Method::Default ||
                    method == Method::Fast_basis ||
                    method == Method::ScTP)
                    ray.tet_idx = 0;
                else
                    ray.tMax = 100000000;

                m_rays[rays_index++ + idx * tile_size * tile_size] = ray;

                /*if (is_diagnostic)
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
                    else if (method == Method::Kd_tree)
                        hit = scene.kd_tree->Intersect(ray, intersection_data);
                    else if (method == Method::BVH_embree)
                        hit = scene.bvh_embree->Intersect(ray, intersection_data);
                    else if (method == Method::BVH_pbrt)
                        hit = scene.bvh->Intersect(ray, intersection_data);
                }*/
            }
        }
        idx = job_index++;
    }
}

void RayTracer::draw_intersectiondata(Scene& scene, int thread_idx, std::vector<LightInfo> lightInfos)
{

    const int tile_count_x = (m_resolution.x + tile_size - 1) / tile_size;
    const int tile_count_y = (m_resolution.y + tile_size - 1) / tile_size;
    const int max_job_index = tile_count_x * tile_count_y;

    int idx = thread_idx;
    unsigned int rays_index = 0;

    glm::ivec2 rect_min = glm::ivec2((idx % tile_count_x) * tile_size, (idx / tile_count_x) * tile_size);
    glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);
    rect_max = (glm::min)(rect_max, m_resolution);

    while (idx < max_job_index)
    {
        rect_min = glm::ivec2((idx % tile_count_x) * tile_size, (idx / tile_count_x) * tile_size);
        rect_max = rect_min + glm::ivec2(tile_size, tile_size);

        rect_max = (glm::min)(rect_max, m_resolution);
        rays_index = 0;

        for (int j = rect_min.y; j < rect_max.y; j++)
        {
            for (int i = rect_min.x; i < rect_max.x; i++)
            {
				glm::vec3 color;
				bool hit = method != Method::Tet96_gpu ? (m_gpu_face_indices[rays_index + idx * tile_size * tile_size] != -1) :
					(m_gpu_face_indices[rays_index + idx * tile_size * tile_size] > 0);
				if (hit)
				{
					m_gpu_face_indices[rays_index + idx * tile_size * tile_size] = 
						method != Method::Tet96_gpu ? (m_gpu_face_indices[rays_index + idx * tile_size * tile_size] & 0x7FFFFFFF) :
						(m_gpu_face_indices[rays_index + idx * tile_size * tile_size]) - 1;

					//---------------Get Ray-------------
					Ray ray = m_rays[rays_index + idx * tile_size * tile_size];

					//calculate face data
					const Face& face = method != Method::Tet96_gpu ?
						*(scene.tet_mesh->m_constrained_faces[m_gpu_face_indices[rays_index + idx * tile_size * tile_size]].face) :
						scene.tet_mesh->faces[m_gpu_face_indices[rays_index + idx * tile_size * tile_size]];
					const glm::vec3* v = face.vertices;
					const glm::vec3* n = face.normals;
					const glm::vec2* t = face.uvs;

					const glm::vec3 e1 = v[1] - v[0];
					const glm::vec3 e2 = v[2] - v[0];
					const glm::vec3 s = ray.origin - v[0];
					const glm::vec3 q = glm::cross(s, e1);
					const glm::vec3 p = glm::cross(ray.dir, e2);
					const float f = 1.0f / glm::dot(e1, p);
					const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));

					m_intersect_data[rays_index + idx * tile_size * tile_size].position = ray.origin + f * glm::dot(e2, q) * ray.dir;
					m_intersect_data[rays_index + idx * tile_size * tile_size].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
					m_intersect_data[rays_index + idx * tile_size * tile_size].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
					m_intersect_data[rays_index + idx * tile_size * tile_size].tet_idx = 
						scene.tet_mesh->m_constrained_faces[m_gpu_face_indices[rays_index + idx * tile_size * tile_size]].tet_idx;
					m_intersect_data[rays_index + idx * tile_size * tile_size].neighbor_tet_idx = 
						scene.tet_mesh->m_constrained_faces[m_gpu_face_indices[rays_index + idx * tile_size * tile_size]].other_tet_idx;
                    color = glm::vec3();
                    //color = glm::vec3(1.0f, 1.0f, 1.0f);

                    for (int light_idx = 0; light_idx < lightInfos.size(); light_idx++)
                    {
                        Ray shadow_ray(m_intersect_data[rays_index + idx * tile_size * tile_size].position,
                            glm::normalize(lightInfos[light_idx].pos - m_intersect_data[rays_index + idx * tile_size * tile_size].position));
                        {
                            glm::vec3 to_light = glm::normalize(lightInfos[light_idx].pos - m_intersect_data[rays_index + idx * tile_size * tile_size].position);
                            float diffuse = glm::clamp(glm::dot(m_intersect_data[rays_index + idx * tile_size * tile_size].normal, to_light), 0.0f, 1.0f);
                            color += lightInfos[light_idx].color * diffuse * lightInfos[light_idx].intensity;
                        }
                    }

                }
                else
                    color = glm::vec3(0.1, 0.1, 0.1);

                glm::ivec2 p_idx(i, j);

                m_rendered_image->set_pixel(p_idx.x, p_idx.y, glm::vec3(color.z, color.y, color.x) * 255.0f);
                rays_index++;
            }
        }
        idx = job_index++;
    }
}

void RayTracer::draw_intersectiondata(int set_start, int set_end, std::vector<LightInfo> lightInfos)
{
	const int tile_count_x = (m_resolution.x + tile_size - 1) / tile_size;
	const int tile_count_y = (m_resolution.y + tile_size - 1) / tile_size;
	const int max_job_index = tile_count_x * tile_count_y;

	unsigned int rays_index = 0;
	int idx = set_start;

	glm::ivec2 rect_min;
	glm::ivec2 rect_max;
	rect_max = (glm::min)(rect_max, m_resolution);

	while (idx < max_job_index || idx < set_end)
	{
		rect_min = glm::ivec2((idx % tile_count_x) * tile_size, (idx / tile_count_x) * tile_size);
		rect_max = rect_min + glm::ivec2(tile_size, tile_size);

		rect_max = (glm::min)(rect_max, m_resolution);
		rays_index = 0;

		for (int j = rect_min.y; j < rect_max.y; j++)
		{
			for (int i = rect_min.x; i < rect_max.x; i++)
			{
				glm::vec3 color; 
				bool hit = method != Method::Tet96_gpu ? (m_gpu_face_indices[idx] != -1) : (m_gpu_face_indices[idx] > 0);
				if (hit)
				{
					m_gpu_face_indices[idx] = method != Method::Tet96_gpu ? (m_gpu_face_indices[idx] & 0x7FFFFFFF) : (m_gpu_face_indices[idx]) - 1;
					color = glm::vec3();
					//color = glm::vec3(1.0f, 1.0f, 1.0f);

					for (int light_idx = 0; light_idx < lightInfos.size(); light_idx++)
					{
						Ray shadow_ray(m_intersect_data[rays_index + idx * tile_size * tile_size].position,
							glm::normalize(lightInfos[light_idx].pos - m_intersect_data[rays_index + idx * tile_size * tile_size].position));
						{
							glm::vec3 to_light = glm::normalize(lightInfos[light_idx].pos - m_intersect_data[rays_index + idx * tile_size * tile_size].position);
							float diffuse = glm::clamp(glm::dot(m_intersect_data[rays_index + idx * tile_size * tile_size].normal, to_light), 0.0f, 1.0f);
							color += lightInfos[light_idx].color * diffuse * lightInfos[light_idx].intensity;
						}
					}

				}
				else
					color = glm::vec3(0.1, 0.1, 0.1);

				glm::ivec2 p_idx(i, j);


				m_rendered_image->set_pixel(p_idx.x, p_idx.y, glm::vec3(color.z, color.y, color.x) * 255.0f);
				rays_index++;
			}
		}
		idx++;
	}
}

void RayTracer::draw_intersectiondata_rowmajor(Scene& scene, int thread_idx, std::vector<LightInfo> lightInfos)
{
    int idx = ((m_resolution.x * m_resolution.y) / thread_count) * thread_idx;
    int upperbound = min( ( (m_resolution.x * m_resolution.y) / thread_count ) * (thread_idx + 1), m_resolution.x * m_resolution.y);
	bool hit = false;

    while (idx < upperbound)
    {
        glm::vec3 color;
		hit = method != Method::Tet96_gpu ? (m_gpu_face_indices[idx] != -1) : (m_gpu_face_indices[idx] > 0);
		if(hit)
		{
			m_gpu_face_indices[idx] = method != Method::Tet96_gpu ? (m_gpu_face_indices[idx] & 0x7FFFFFFF) : (m_gpu_face_indices[idx]) - 1;

			color = glm::vec3();

			//---------------Generate Ray-------------
			const glm::vec3 camTarget = scene.camTarget;
			glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));

			dir = dir * glm::cos(scene.camOrbitX);
			dir.y = glm::sin(scene.camOrbitX);

			glm::vec3 cam_pos = camTarget + dir * scene.camDist;

			const glm::vec3 cam_forward = glm::normalize(scene.camTarget - cam_pos);
			const glm::vec3 cam_right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), cam_forward));
			const glm::vec3 cam_down = glm::cross(cam_forward, cam_right);

			const glm::vec3 cam_up = glm::cross(cam_forward, cam_right);

			const float aspect = (float)m_resolution.x / m_resolution.y;
			const float scale_y = glm::tan(glm::pi<float>() / 8);

			const glm::vec3 bottom_left = cam_pos + cam_forward - cam_up * scale_y - cam_right * scale_y * aspect;
			const glm::vec3 up_step = (cam_up * scale_y * 2.0f) / (float)m_resolution.y;

			const glm::vec3 top_left = cam_pos + cam_forward - cam_down * scale_y - cam_right * scale_y * aspect;
			const glm::vec3 right_step = (cam_right * scale_y * 2.0f * aspect) / (float)m_resolution.x;
			const glm::vec3 down_step = (cam_down * scale_y * 2.0f) / (float)m_resolution.y;

			glm::ivec2 pixel_coords = glm::ivec2(idx%(m_resolution.x), idx / (m_resolution.x));

			Ray ray;
			ray.origin = cam_pos;
			ray.dir = glm::normalize(top_left + right_step * (float)pixel_coords.x + down_step * (float)pixel_coords.y - ray.origin);

			//calculate face data
			const Face& face = method != Method::Tet96_gpu ? 
				*(scene.tet_mesh->m_constrained_faces[m_gpu_face_indices[idx]].face) : scene.tet_mesh->faces[m_gpu_face_indices[idx]];
			const glm::vec3* v = face.vertices;
			const glm::vec3* n = face.normals;
			const glm::vec2* t = face.uvs;

			const glm::vec3 e1 = v[1] - v[0];
			const glm::vec3 e2 = v[2] - v[0];
			const glm::vec3 s = ray.origin - v[0];
			const glm::vec3 q = glm::cross(s, e1);
			const glm::vec3 p = glm::cross(ray.dir, e2);
			const float f = 1.0f / glm::dot(e1, p);
			const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));

			m_intersect_data[idx].position = ray.origin + f * glm::dot(e2, q) * ray.dir;
			m_intersect_data[idx].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
			m_intersect_data[idx].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
			m_intersect_data[idx].tet_idx = scene.tet_mesh->m_constrained_faces[m_gpu_face_indices[idx]].tet_idx;
			m_intersect_data[idx].neighbor_tet_idx = scene.tet_mesh->m_constrained_faces[m_gpu_face_indices[idx]].other_tet_idx;

			//color = glm::vec3(1.0f, 1.0f, 1.0f);
			for (int light_idx = 0; light_idx < lightInfos.size(); light_idx++)
			{
				Ray shadow_ray(m_intersect_data[idx].position,
					glm::normalize(lightInfos[light_idx].pos - m_intersect_data[idx].position));
				{
					glm::vec3 to_light = glm::normalize(lightInfos[light_idx].pos - m_intersect_data[idx].position);
					float diffuse = glm::clamp(glm::dot(m_intersect_data[idx].normal, to_light), 0.0f, 1.0f);
					color += lightInfos[light_idx].color * diffuse * lightInfos[light_idx].intensity;
				}
			}
		}
        else
            color = glm::vec3(0.1, 0.1, 0.1);

        glm::ivec2 p_idx(idx % m_resolution.x, idx / m_resolution.x);
        m_rendered_image->set_pixel(p_idx.x, p_idx.y, glm::vec3(color.z, color.y, color.x) * 255.0f);
                
        idx++;
    }
}

void RayTracer::draw_intersectiondata_rowmajor(int set_start, int set_end, std::vector<LightInfo> lightInfos)
{
	int idx = set_start;
	//int upperbound = min(((m_resolution.x * m_resolution.y) / thread_count) * (thread_idx + 1), m_resolution.x * m_resolution.y);

	while (idx < set_end || idx < m_resolution.x * m_resolution.y)
	{
		//rays_index = 0;
		glm::vec3 color;
		if (m_intersect_data[idx].hit)
		{
			color = glm::vec3();
			//color = glm::vec3(1.0f, 1.0f, 1.0f);
			for (int light_idx = 0; light_idx < lightInfos.size(); light_idx++)
			{
				Ray shadow_ray(m_intersect_data[idx].position,
					glm::normalize(lightInfos[light_idx].pos - m_intersect_data[idx].position));
				{
					glm::vec3 to_light = glm::normalize(lightInfos[light_idx].pos - m_intersect_data[idx].position);
					float diffuse = glm::clamp(glm::dot(m_intersect_data[idx].normal, to_light), 0.0f, 1.0f);
					color += lightInfos[light_idx].color * diffuse * lightInfos[light_idx].intensity;
				}
			}
		}
		else
			color = glm::vec3(0.1, 0.1, 0.1);

		glm::ivec2 p_idx(idx % m_resolution.x, idx / m_resolution.x);
		m_rendered_image->set_pixel(p_idx.x, p_idx.y, glm::vec3(color.z, color.y, color.x) * 255.0f);

		idx++;
	}
}

void RayTracer::render_gpu(Scene & scene, const bool is_diagnostic)
{
    glm::vec3 camTarget = scene.camTarget;
    glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));

    dir = dir * glm::cos(scene.camOrbitX);

    dir.y = glm::sin(scene.camOrbitX);

    glm::vec3 cam_pos = camTarget + dir * scene.camDist;

    Ray ray;
    ray.origin = cam_pos;
    SourceTet source_tet;

    int tet_index = 0;

    if (scene.tet_mesh)
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

    if (m_old_res != m_resolution)
    {
		if (!m_intersect_data)
            delete[] m_intersect_data;
		if (!m_gpu_face_indices)
			delete[] m_gpu_face_indices;
		if (!m_rays)
		{
            delete[] m_rays;
		}

        m_rays = new Ray[m_resolution.x * m_resolution.y];
        m_intersect_data = new IntersectionData[m_resolution.x * m_resolution.y];
		m_gpu_face_indices = new int[m_resolution.x * m_resolution.y];
        m_old_res = m_resolution;
    }
    //--------------------------------------------
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(), end, start_2, end_2;
    //--------------------------------------------
    std::thread **threads = new std::thread*[thread_count];

    job_index = thread_count;
    
    if (method == Method::Default_gpu)
    {
        if (gpu_init_rays)
        {
			/*for (int i = 0; i < thread_count; i++)
				threads[i] = new std::thread(&RayTracer::asynch_ray_cast_worker_gpu, this, std::ref(scene), source_tet, i, lightInfos, is_diagnostic);

			for (int i = 0; i < thread_count; i++)
			{
				threads[i]->join();
				delete threads[i];
			}*/
            //--------------------------------------------
            Stats::ray_prep_time = 0;
            //--------------------------------------------

            int tetmesh_type = 0;
            if (dynamic_cast<TetMesh20 *>(scene.tet_mesh) != nullptr)
                tetmesh_type = 1;
            else if (dynamic_cast<TetMesh16 *>(scene.tet_mesh) != nullptr)
                tetmesh_type = 2;

			cast_rays_gpu(scene, source_tet, m_resolution, tile_size, tetmesh_type, m_gpu_face_indices/*m_intersect_data*/);


            threads = new std::thread*[thread_count];
            job_index = thread_count;

            //--------------------------------------------
            start_2 = std::chrono::steady_clock::now();
            //--------------------------------------------
            for (int i = 0; i < thread_count; i++)
            {
                void (RayTracer::*mem_funct)(Scene&, int, std::vector<LightInfo>) = &RayTracer::draw_intersectiondata_rowmajor;
                threads[i] = new std::thread(mem_funct, this, scene, i, lightInfos);
            }

            for (int i = 0; i < thread_count; i++)
            {
                threads[i]->join();
                delete threads[i];
            }
            //--------------------------------------------
            end_2 = std::chrono::steady_clock::now();
            Stats::draw_time = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count() / 1e3;
            //--------------------------------------------
        }
        else
        {
            /*for (int i = 0; i < thread_count; i++)
            threads[i] = new std::thread(&RayTracer::raytrace_worker_gpu, this, std::ref(scene), source_tet, i, lightInfos, is_diagnostic);

            for (int i = 0; i < thread_count; i++)
            {
                threads[i]->join();
                delete threads[i];
            }*/

            //--------------------------------------------
            std::chrono::steady_clock::time_point start_2 = std::chrono::steady_clock::now();
            //--------------------------------------------
            for (int i = 0; i < thread_count; i++)
                threads[i] = new std::thread(&RayTracer::prepare_rays_gpu, this, std::ref(scene), source_tet, i, is_diagnostic);

            for (int i = 0; i < thread_count; i++)
            {
                threads[i]->join();
                delete threads[i];
            }
            //--------------------------------------------
            std::chrono::steady_clock::time_point end_2 = std::chrono::steady_clock::now();
            Stats::ray_prep_time = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count() / 1e3;
            //--------------------------------------------

            int tetmesh_type = 0;
            if (dynamic_cast<TetMesh20 *>(scene.tet_mesh) != nullptr)
                tetmesh_type = 1;
            else if (dynamic_cast<TetMesh16 *>(scene.tet_mesh) != nullptr)
                tetmesh_type = 2;

            traverse_rays_gpu(m_rays, m_resolution.x * m_resolution.y, tetmesh_type, m_gpu_face_indices);

            threads = new std::thread*[thread_count];
            job_index = thread_count;

            //--------------------------------------------
            start_2 = std::chrono::steady_clock::now();
            //--------------------------------------------
            for (int i = 0; i < thread_count; i++)
            {
                void (RayTracer::*mem_funct)(Scene&, int, std::vector<LightInfo>) = &RayTracer::draw_intersectiondata;
                threads[i] = new std::thread(mem_funct, this, scene, i, lightInfos);
            }

            for (int i = 0; i < thread_count; i++)
            {
                threads[i]->join();
                delete threads[i];
            }
            //--------------------------------------------
            end_2 = std::chrono::steady_clock::now();
            Stats::draw_time = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count() / 1e3;
            //--------------------------------------------

        }
    }
    else if(method == Method::ScTP_gpu)
    {
        if (gpu_init_rays)
        {
            //--------------------------------------------
            Stats::ray_prep_time = 0;
            //--------------------------------------------
            if (dynamic_cast<TetMeshSctp *>(scene.tet_mesh) != nullptr)
                cast_rays_gpu(scene, source_tet, m_resolution, tile_size, 3, m_gpu_face_indices);

            threads = new std::thread*[thread_count];
            job_index = thread_count;

            //--------------------------------------------
            start_2 = std::chrono::steady_clock::now();
            //--------------------------------------------
            for (int i = 0; i < thread_count; i++)
            {
				void (RayTracer::* mem_funct)(Scene&, int, std::vector<LightInfo>) = &RayTracer::draw_intersectiondata_rowmajor;
				threads[i] = new std::thread(mem_funct, this, scene, i, lightInfos);
            }

            for (int i = 0; i < thread_count; i++)
            {
                threads[i]->join();
                delete threads[i];
            }
            //--------------------------------------------
            end_2 = std::chrono::steady_clock::now();
            Stats::draw_time = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count() / 1e3;
            //--------------------------------------------
        }
    }
	else if (method == Method::Tet96_gpu)
	{
		if (gpu_init_rays)
		{
			//--------------------------------------------
			Stats::ray_prep_time = 0;
			//--------------------------------------------
			if (dynamic_cast<TetMesh80*>(scene.tet_mesh) != nullptr)
				cast_rays_gpu(scene, source_tet, m_resolution, tile_size, 4, m_gpu_face_indices);

			threads = new std::thread * [thread_count];
			job_index = thread_count;

			//--------------------------------------------
			start_2 = std::chrono::steady_clock::now();
			//--------------------------------------------
			for (int i = 0; i < thread_count; i++)
			{
				void (RayTracer::* mem_funct)(Scene&, int, std::vector<LightInfo>) = &RayTracer::draw_intersectiondata_rowmajor;
				threads[i] = new std::thread(mem_funct, this, scene, i, lightInfos);
			}

			for (int i = 0; i < thread_count; i++)
			{
				threads[i]->join();
				delete threads[i];
			}
			//--------------------------------------------
			end_2 = std::chrono::steady_clock::now();
			Stats::draw_time = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count() / 1e3;
			//--------------------------------------------
		}
		else
		{  //--------------------------------------------
			std::chrono::steady_clock::time_point start_2 = std::chrono::steady_clock::now();
			//--------------------------------------------
			for (int i = 0; i < thread_count; i++)
				threads[i] = new std::thread(&RayTracer::prepare_rays_gpu, this, std::ref(scene), source_tet, i, is_diagnostic);

			for (int i = 0; i < thread_count; i++)
			{
				threads[i]->join();
				delete threads[i];
			}
			//--------------------------------------------
			std::chrono::steady_clock::time_point end_2 = std::chrono::steady_clock::now();
			Stats::ray_prep_time = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count() / 1e3;
			//--------------------------------------------

			if (dynamic_cast<TetMesh80*>(scene.tet_mesh) != nullptr)
				traverse_rays_gpu(m_rays, m_resolution.x * m_resolution.y, 4, m_gpu_face_indices);

			threads = new std::thread * [thread_count];
			job_index = thread_count;

			//--------------------------------------------
			start_2 = std::chrono::steady_clock::now();
			//--------------------------------------------
			for (int i = 0; i < thread_count; i++)
			{
				void (RayTracer:: * mem_funct)(Scene&, int, std::vector<LightInfo>) = &RayTracer::draw_intersectiondata;
				threads[i] = new std::thread(mem_funct, this, scene, i, lightInfos);
			}

			for (int i = 0; i < thread_count; i++)
			{
				threads[i]->join();
				delete threads[i];
			}
			//--------------------------------------------
			end_2 = std::chrono::steady_clock::now();
			Stats::draw_time = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count() / 1e3;
			//--------------------------------------------
		}
	}

    delete[] threads;
    
    //--------------------------------------------
    end = std::chrono::steady_clock::now();
    last_render_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1e3f;
    Stats::add_render_time(last_render_time);
    //--------------------------------------------

    avg_test_count = 0;
    L1_count = 0;
}

//Unused!!!
void RayTracer::ray_caster(Scene& scene, std::vector<Ray> rays, std::vector<IntersectionData>& output)
{
    for (int i = 0; i < rays.size(); i++)
    {
        IntersectionData i_data;
        output.push_back(i_data);
        output[i].hit = scene.tet_mesh->intersect(rays[i], rays[i].source_tet, output[i]);
    }
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

void RayTracer::set_resoultion(const glm::ivec2& resolution)
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
}
