#include "raycaster.cuh";
#include <stdio.h>

__global__
void raycast_kernel(/*Scene &a,*/ Ray *rays, int rays_size/*, IntersectionData *output*/)
{
    int i = threadIdx.x;
    if(i < rays_size)
    rays[i].dir.x = 111.0f;
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
    Ray* c = new Ray[rays.size()];
    // Allocate space for device copy of Ray
    cudaMalloc(&d_rays, rays.size() * sizeof(Ray));

    // Copy inputs to device
    cudaMemcpy(d_rays, rays.data(), rays.size() * sizeof(Ray), cudaMemcpyHostToDevice);

    // Launch add() kernel on GPU
    raycast_kernel<<<1, 1024>>> (d_rays, rays.size()/*, nullptr*/);

    // Copy result back to host
    cudaMemcpy(c, d_rays, rays.size() * sizeof(Ray), cudaMemcpyDeviceToHost);
    // Cleanup
    cudaFree(d_rays);

    printf("%lf", c[1023].dir.x);

    delete[] c;
}
