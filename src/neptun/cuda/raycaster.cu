#include "cuda_runtime.h"

#include "device_launch_parameters.h"

#include "neptun/main/tet_mesh.h"

#define GLM_FORCE_CUDA
#include "glm/glm.hpp"

#include <iostream>

__global__
void saxpy(int n, float a, float *x, float *y)
{
    glm::vec3 v;
    glm::normalize(v);
    //int i = blockIdx.x*blockDim.x + threadIdx.x;
    //if (i < n) y[i] = a * x[i] + y[i];
}

void send_to_gpu(const TetMesh& tet_mesh)
{
    glm::vec3* points;

    cudaMalloc(&points, tet_mesh.m_points.size() * sizeof(glm::vec3));
    cudaMemcpy(points, tet_mesh.m_points.data(), tet_mesh.m_points.size() * sizeof(glm::vec3), cudaMemcpyHostToDevice);
}

void raycast()
{
    
    saxpy <<<256, 256>>>(1, 2.0, NULL, NULL);
    //printf("hello\n");
}