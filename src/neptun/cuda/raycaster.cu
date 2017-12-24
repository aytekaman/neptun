#include "cuda_runtime.h"

#include "device_launch_parameters.h"

#include "neptun/main/tet_mesh.h"

#define GLM_FORCE_CUDA
#include "glm/glm.hpp"

#include <iostream>

glm::vec3* points;

int* buffer;

int buffer_cpu[640 * 480];

const int tile_size = 16;

__global__
void raycast_kernel(int* b)
{
    glm::ivec2 resolution(640, 480);
    glm::ivec2 rect_min = glm::ivec2((blockIdx.x * tile_size) % resolution.x, ((blockIdx.x * tile_size) / resolution.x) * tile_size);
    glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);
    glm::ivec2 pixel_coords = rect_min + glm::ivec2(threadIdx.x, threadIdx.y);

    int index = pixel_coords.x * resolution.y + pixel_coords.y;

    b[index] = 7;
}

void send_to_gpu()
{
    glm::ivec2 resolution(640, 480);
    //buffer_cpu = new bool[resolution.x * resolution.y];

    std::cout << cudaMalloc(&buffer, resolution.x * resolution.y * sizeof(int));
    cudaMemcpy(buffer, buffer_cpu, resolution.x * resolution.y * sizeof(int), cudaMemcpyHostToDevice);

    //cudaMalloc(&points, tet_mesh.m_points.size() * sizeof(glm::vec3));
    //cudaMemcpy(points, tet_mesh.m_points.data(), tet_mesh.m_points.size() * sizeof(glm::vec3), cudaMemcpyHostToDevice);
}

int* raycast_gpu()
{
    glm::ivec2 resolution(640, 480);
    dim3 g(16, 16);
    raycast_kernel<<<64, g>>>(buffer);

    std::cout << cudaThreadSynchronize();

    cudaMemcpy(buffer_cpu, buffer, resolution.x * resolution.y * sizeof(int), cudaMemcpyDeviceToHost);
 
    return buffer_cpu;
}