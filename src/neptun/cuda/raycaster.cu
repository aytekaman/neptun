#include "cuda_runtime.h"

#include "device_launch_parameters.h"

#include "neptun/main/tet_mesh.h"

#define GLM_FORCE_CUDA
#include "glm/glm.hpp"

#include <iostream>

glm::vec3* points;
Tet32* tets;

int* buffer;

int buffer_cpu[640 * 480];

const int tile_size = 16;

__device__
void swapvec2(glm::vec2 &a, glm::vec2 &b)
{
    glm::vec2 t = a;
    a = b;
    b = t;
}

//__device__
//void swap(float &a, float &b)
//{
//    float t = a;
//    a = b;
//    b = t;
//}

__device__
void swap(unsigned int &a, unsigned int &b)
{
    unsigned int t = a;
    a = b;
    b = t;
}

__device__
float crossv(const glm::vec2& a, const glm::vec2& b) { return a.x * b.y - a.y * b.x; }

__global__
void raycast_kernel(int* output, glm::vec3* m_points, Tet32* m_tet32s, CamInfo* cam_info, SourceTet* source_tet)
{
    glm::ivec2 resolution(640, 480);
    glm::ivec2 rect_min = glm::ivec2((blockIdx.x * tile_size) % resolution.x, ((blockIdx.x * tile_size) / resolution.x) * tile_size);
    glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);
    glm::ivec2 pixel_coords = rect_min + glm::ivec2(threadIdx.x, threadIdx.y);

    int outputindex = pixel_coords.x * resolution.y + pixel_coords.y;

    glm::vec3 origin;
    glm::vec3 dir;

    unsigned int id[4];
    glm::vec2 p[4];

    //int prev_index;
    signed short outIdx = -1;

    const float sign = copysignf(1.0f, dir.z);

    const float a = -1.0f / (sign + dir.z);
    const float b = dir.x * dir.y * a;

    glm::vec3 right = glm::vec3(1.0f + sign * dir.x * dir.x * a, sign * b, -sign * dir.x);
    glm::vec3 up = glm::vec3(b, sign + dir.y * dir.y * a, -dir.y);

    for (int i = 0; i < 4; i++)
    {
        id[i] = source_tet->v[i];
        const glm::vec3 point = m_points[id[i]] - origin;
        p[i].x = glm::dot(right, point);
        p[i].y = glm::dot(up, point);
    }

    if (crossv(p[2], p[1]) <= 0.0 && crossv(p[1], p[3]) <= 0.0 && crossv(p[3], p[2]) <= 0.0)
        outIdx = 0;
    else if (crossv(p[2], p[3]) <= 0.0 && crossv(p[3], p[0]) <= 0.0 && crossv(p[0], p[2]) <= 0.0)
        outIdx = 1;
    else if (crossv(p[0], p[3]) <= 0.0 && crossv(p[3], p[1]) <= 0.0 && crossv(p[1], p[0]) <= 0.0)
        outIdx = 2;
    else if (crossv(p[0], p[1]) <= 0.0 && crossv(p[1], p[2]) <= 0.0 && crossv(p[2], p[0]) <= 0.0)
    {
        outIdx = 3;
        swap(id[0], id[1]);
        swapvec2(p[0], p[1]);
    }
    else
    {
        buffer[outputindex];
        return;
    }

    int index = source_tet->n[outIdx];

    while (index >= 0)
    {
        id[outIdx] = id[3];
        id[3] = m_tet32s[index].x ^ id[0] ^ id[1] ^ id[2];
        const glm::vec3 newPoint = m_points[id[3]] - origin;

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

    output[outputindex] = (index != -1);
}

void send_to_gpu(TetMesh32& tet_mesh)
{
    glm::ivec2 resolution(640, 480);
    //buffer_cpu = new bool[resolution.x * resolution.y];

    std::cout << cudaMalloc(&buffer, resolution.x * resolution.y * sizeof(int));
    cudaMemcpy(buffer, buffer_cpu, resolution.x * resolution.y * sizeof(int), cudaMemcpyHostToDevice);

    cudaMalloc(&points, tet_mesh.m_points.size() * sizeof(glm::vec3));
    cudaMemcpy(points, tet_mesh.m_points.data(), tet_mesh.m_points.size() * sizeof(glm::vec3), cudaMemcpyHostToDevice);

    cudaMalloc(&tets, tet_mesh.m_tet32s.size() * sizeof(Tet32));
    cudaMemcpy(tets, tet_mesh.m_tet32s.data(), tet_mesh.m_tet32s.size() * sizeof(Tet32), cudaMemcpyHostToDevice);
}

int* raycast_gpu(SourceTet* source_tet, CamInfo* cam_info)
{
    glm::ivec2 resolution(640, 480);
    dim3 g(16, 16);
    raycast_kernel<<<64, g>>>(buffer, points, tets);

    std::cout << cudaThreadSynchronize();

    cudaMemcpy(buffer_cpu, buffer, resolution.x * resolution.y * sizeof(int), cudaMemcpyDeviceToHost);
 
    return buffer_cpu;
}