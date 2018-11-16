#include "raycaster.cuh";

__global__
void raycast_kernel()
{
    //for (int i = 0; i < rays.size(); i++)
    if()
    {
        IntersectionData i_data;
        output.push_back(i_data);
        output[i].hit = scene.tet_mesh->intersect(rays[i], rays[i].source_tet, output[i]);
    }
}

void ray_caster_gpu(Scene& scene, std::vector<Ray> rays, std::vector<IntersectionData>& output)
{
    
}