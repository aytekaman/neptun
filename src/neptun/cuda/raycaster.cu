#include "raycaster.cuh";
#include <stdio.h>

__global__
void raycast_kernel(/*Scene &a,*/ Ray *rays, int rays_size, IntersectionData *output)
{
    int i = threadIdx.x;
    if (i < rays_size)
    {
        if( i % 2)
            output[i].hit = true;
        else
            output[i].hit = false;
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

    // Copy inputs to device
    cudaMemcpy(d_rays, rays.data(), rays.size() * sizeof(Ray), cudaMemcpyHostToDevice);

    // Launch add() kernel on GPU
    raycast_kernel<<<1, rays.size()>>> (d_rays, rays.size(), d_intersectdata);

    // Copy result back to host
    cudaMemcpy(c, d_intersectdata, rays.size() * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
    //cudaMemcpy(&output, d_intersectdata, rays.size() * sizeof(IntersectionData), cudaMemcpyDeviceToHost);

    // Cleanup
    cudaFree(d_rays);
    cudaFree(d_intersectdata);

    printf("%d", c[1020].hit);
    printf("%d", c[1021].hit);
    output.insert(output.begin(), c, c + rays.size());

    //delete[] c;
}
