#include "raycaster.cuh";
#include <stdio.h>

int N = 640 * 480;

__global__
void raycast_kernel(/*Scene &a,*/ Ray *rays, int rays_size, IntersectionData *output)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < rays_size)
    {
        if ((i / 8) % 2)
            output[i].hit = 1;
        else
            output[i].hit = 0;
    }
}

/*void copy_to_gpu(const std::vector<Ray> rays, Ray* d_rays)
{
    cudaFree(d_rays);
    cudaMalloc(&d_rays, rays.size() * sizeof(Ray));
    cudaMemcpy(d_rays, rays.data(), rays.size() * sizeof(Ray), cudaMemcpyHostToDevice);
}

void copy_from_gpu(const Ray* d_rays, Ray* rays)
{
    cudaMemcpy(rays, d_rays, rays.size() * sizeof(Ray), cudaMemcpyDeviceToHost);
}*/

void ray_caster_gpu(Scene& scene, std::vector<Ray> rays, std::vector<IntersectionData>& output)
{
    Ray *d_rays;
    IntersectionData* c = new IntersectionData[rays.size()];

    IntersectionData *d_intersectdata;
    
    // Allocate space for device copy of Ray
    cudaMalloc(&d_rays, rays.size() * sizeof(Ray));
    cudaMalloc(&d_intersectdata, rays.size() * sizeof(IntersectionData));
    cudaError_t error = cudaGetLastError();
    //printf("CUDA error0: %s\n", cudaGetErrorString(error));

    // Copy inputs to device
    cudaMemcpy(d_rays, rays.data(), rays.size() * sizeof(Ray), cudaMemcpyHostToDevice);
    error = cudaGetLastError();
    printf("CUDA error1: %s\n", cudaGetErrorString(error));

    // Launch kernel on GPU
    raycast_kernel<<< rays.size() / 1024, 1024>>> (d_rays, rays.size(), d_intersectdata);
    //cudaDeviceSynchronize();
    error = cudaGetLastError();
    printf("CUDA error2: %s\n", cudaGetErrorString(error));

    // Copy result back to host
    //cudaMemcpyToArray(c, 0, 0, d_intersectdata, rays.size() * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
    cudaMemcpy(c, d_intersectdata, rays.size() * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
    /*error = cudaGetLastError();
    printf("CUDA error3: %s\n", cudaGetErrorString(error));*/

    // Cleanup
    cudaFree(d_rays);
    cudaFree(d_intersectdata);

    printf("%d\n", (int)c[110].hit);
    printf("%d\n", (int)c[1021].hit);
    output.insert(output.begin(), c, c + rays.size());

    delete[] c;
}
