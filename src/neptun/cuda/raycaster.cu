#include "raycaster.cuh";
#include <chrono>
#include <ctime>

Ray *d_rays;
IntersectionData* d_intersectdata;
glm::vec3* d_points;
TetMesh32::Tet32* d_tets32;
TetMesh20::Tet20* d_tets20;
TetMesh16::Tet16* d_tets16;
ConstrainedFace* d_cons_faces;
Face* d_faces;
unsigned int old_size = 0;

__global__
void raycast_kernel(Ray *rays, int rays_size, glm::vec3* d_points, TetMesh32::Tet32* d_tets, ConstrainedFace* d_cons_faces, Face* d_faces, IntersectionData *output)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < rays_size)
    {
        Ray ray = rays[i];
        unsigned int id[4];
        glm::vec2 p[4];

        const float sign = copysignf(1.0f, ray.dir.z);

        const float a = -1.0f / (sign + ray.dir.z);
        const float b = ray.dir.x * ray.dir.y * a;

        const glm::vec3 right(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x);
        const glm::vec3 up(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y);

        for (int j = 0; j < 4; j++)
        {
            id[j] = rays[i].source_tet.v[j];
            const glm::vec3 point = d_points[id[j]] - ray.origin;
            p[j].x = glm::dot(right, point);
            p[j].y = glm::dot(up, point);
        }

        signed short outIdx = -1;

        if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
            outIdx = 0;
        else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
            outIdx = 1;
        else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
            outIdx = 2;
        else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
        {
            outIdx = 3;
            swap(id[0], id[1]);
            swapvec2(p[0], p[1]);
        }
        else
        {
            output[i].hit = false;
            return;
        }

        int index = ray.source_tet.n[outIdx];

        while (index >= 0)
        {
            id[outIdx] = id[3];
            id[3] = d_tets[index].x ^ id[0] ^ id[1] ^ id[2];
            const glm::vec3 newPoint = d_points[id[3]] - ray.origin;

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

            if (id[outIdx] == d_tets[index].v[0])
                index = d_tets[index].n[0];
            else if (id[outIdx] == d_tets[index].v[1])
                index = d_tets[index].n[1];
            else if (id[outIdx] == d_tets[index].v[2])
                index = d_tets[index].n[2];
            else
                index = d_tets[index].n[3];
        }

        if (index != -1)
        {
            index = (index & 0x7FFFFFFF);
            const Face& face = d_faces[d_cons_faces[index].face_idx];

            const glm::vec3 *v = face.vertices;
            const glm::vec3 *n = face.normals;
            const glm::vec2 *t = face.uvs;

            const glm::vec3 e1 = v[1] - v[0];
            const glm::vec3 e2 = v[2] - v[0];
            const glm::vec3 s = ray.origin - v[0];
            const glm::vec3 q = glm::cross(s, e1);
            const glm::vec3 p = glm::cross(ray.dir, e2);
            const float f = 1.0f / glm::dot(e1, p);
            const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));

            output[i].position =  ray.origin + f * glm::dot(e2, q) * ray.dir;//***
            output[i].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];//***
            output[i].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0]; //***
            output[i].tet_idx = d_cons_faces[index].tet_idx;
            output[i].neighbor_tet_idx = d_cons_faces[index].other_tet_idx;

            output[i].hit = true;
        }
        else
            output[i].hit = false;
    }
}

//------------------------------------------------o-------------------------------------------------------

__global__
void raycast_kernel(Ray* rays, int rays_size, glm::vec3* d_points, TetMesh20::Tet20* d_tets, ConstrainedFace* d_cons_faces, Face* d_faces, IntersectionData *output)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < rays_size)
    {
        Ray ray = rays[i];
        unsigned int id[4];
        glm::vec2 p[4];

        //int prev_index;
        signed short outIdx = -1;

        const float sign = copysignf(1.0f, ray.dir.z);

        const float a = -1.0f / (sign + ray.dir.z);
        const float b = ray.dir.x * ray.dir.y * a;

        const glm::vec3 right(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x);
        const glm::vec3 up(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y);
        const glm::vec3 origin(ray.origin.x, ray.origin.y, ray.origin.z);

        const int mask = 0xE1;

        for (int j = 0; j < 4; j++)
        {
            id[j] = ray.source_tet.v[j];
            const glm::vec3 point = d_points[id[j]] - ray.origin;
            p[j].x = glm::dot(right, point);
            p[j].y = glm::dot(up, point);
        }

        if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
            outIdx = 0;
        else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
            outIdx = 1;
        else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
            outIdx = 2;
        else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
        {
            outIdx = 3;
            swap(id[0], id[1]);
            swapvec2(p[0], p[1]);
        }
        else
        {
            output[i].hit = false;
            return;
        }

        int index = ray.source_tet.n[outIdx];
        id[outIdx] = id[3];
        p[outIdx] = p[3];

        while (index >= 0)
        {
            id[3] = d_tets[index].x ^ id[0] ^ id[1] ^ id[2];
            const glm::vec3 newPoint = d_points[id[3]] - ray.origin;

            p[3].x = glm::dot(right, newPoint);
            p[3].y = glm::dot(up, newPoint);

            //p[3] = basis.project(newPoint);

            if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
            {
                if (p[3].x * p[2].y >= p[3].y * p[2].x)
                {
                    index = d_tets[index].n[(id[1] > id[0]) + (id[1] > id[2]) + (id[1] > id[3])];
                    //outIdx = 1;
                    id[1] = id[3];
                    p[1] = p[3];
                }
                else
                {
                    index = d_tets[index].n[(id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3])];
                    //outIdx = 0;
                    id[0] = id[3];
                    p[0] = p[3];
                }
            }
            else if (p[3].x * p[1].y < p[3].y * p[1].x)
            {
                index = d_tets[index].n[(id[2] > id[0]) + (id[2] > id[1]) + (id[2] > id[3])];
                //outIdx = 2;
                id[2] = id[3];
                p[2] = p[3];
            }
            else
            {
                index = d_tets[index].n[(id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3])];
                //outIdx = 0;
                id[0] = id[3];
                p[0] = p[3];
            }

            //prev_index = index;

            //int idx = 0;
            //for (int i = 0; i < 4; ++i)
            //{
            //    if (id[outIdx] > id[i])
            //        idx++;
            //}

            //index = m_tet20s[index].n[idx];
        }

        if (index != -1)
        {
            index = (index & 0x7FFFFFFF);
            const Face& face = d_faces[d_cons_faces[index].face_idx];

            const glm::vec3 *v = face.vertices;
            const glm::vec3 *n = face.normals;
            const glm::vec2 *t = face.uvs;

            const glm::vec3 e1 = v[1] - v[0];
            const glm::vec3 e2 = v[2] - v[0];
            const glm::vec3 s = ray.origin - v[0];
            const glm::vec3 q = glm::cross(s, e1);
            const glm::vec3 p = glm::cross(ray.dir, e2);
            const float f = 1.0f / glm::dot(e1, p);
            const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));
            output[i].position = ray.origin + f * glm::dot(e2, q) * ray.dir;
            output[i].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
            output[i].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
            output[i].tet_idx = d_cons_faces[index].tet_idx;
            output[i].neighbor_tet_idx = d_cons_faces[index].other_tet_idx;

            output[i].hit = true;
        }
        else
            output[i].hit = false;
    }
}

//------------------------------------------------o-------------------------------------------------------

__global__
void raycast_kernel(Ray* rays, int rays_size, glm::vec3* d_points, TetMesh16::Tet16* d_tets, ConstrainedFace* d_cons_faces, Face* d_faces, IntersectionData *output)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < rays_size)
    {
        Ray ray = rays[i];
        unsigned int id[4];
        glm::vec2 p[4];

        const float sign = copysignf(1.0f, ray.dir.z);

        const float a = -1.0f / (sign + ray.dir.z);
        const float b = ray.dir.x * ray.dir.y * a;

        const glm::vec3 right(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x);
        const glm::vec3 up(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y);

        int index;

        for (int j = 0; j < 4; j++)
        {
            id[j] = ray.source_tet.v[j];
            const glm::vec3 point = d_points[id[j]] - ray.origin;
            p[j].x = glm::dot(right, point);
            p[j].y = glm::dot(up, point);
        }

        if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
        {
            index = ray.source_tet.n[0];
            id[0] = id[3];
            p[0] = p[3];
        }
        else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
        {
            index = ray.source_tet.n[1];
            id[1] = id[3];
            p[1] = p[3];
        }
        else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
        {
            index = ray.source_tet.n[2];
            id[2] = id[3];
            p[2] = p[3];
        }
        else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
        {
            swap(id[0], id[1]);
            swapvec2(p[0], p[1]);

            index = ray.source_tet.n[3];
        }
        else
        {
            output[i].hit = false;
            return;
        }

        int nx = ray.source_tet.idx;

        while (index >= 0)
        {
            id[3] = d_tets[index].x ^ id[0] ^ id[1] ^ id[2];
            const glm::vec3 newPoint = d_points[id[3]] - ray.origin;

            p[3].x = glm::dot(right, newPoint);
            p[3].y = glm::dot(up, newPoint);

            const int idx = (id[3] > id[0]) + (id[3] > id[1]) + (id[3] > id[2]);

            if (idx != 0)
                nx ^= d_tets[index].n[idx - 1];

            if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
            {
                if (p[3].x * p[2].y >= p[3].y * p[2].x)
                {
                    const int idx2 = (id[1] > id[0]) + (id[1] > id[2]) + (id[1] > id[3]);

                    if (idx2 != 0)
                        nx ^= d_tets[index].n[idx2 - 1];

                    id[1] = id[3];
                    p[1] = p[3];
                }
                else
                {
                    const int idx2 = (id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3]);

                    if (idx2 != 0)
                        nx ^= d_tets[index].n[idx2 - 1];

                    id[0] = id[3];
                    p[0] = p[3];
                }
            }
            else if (p[3].x * p[1].y < p[3].y * p[1].x)
            {
                const int idx2 = (id[2] > id[0]) + (id[2] > id[1]) + (id[2] > id[3]);

                if (idx2 != 0)
                    nx ^= d_tets[index].n[idx2 - 1];

                id[2] = id[3];
                p[2] = p[3];
            }
            else
            {
                const int idx2 = (id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3]);

                if (idx2 != 0)
                    nx ^= d_tets[index].n[idx2 - 1];

                id[0] = id[3];
                p[0] = p[3];
            }

            swap(nx, index);
        }

        if (index != -1)
        {
            index = (index & 0x7FFFFFFF);
            const Face& face = d_faces[d_cons_faces[index].face_idx];

            const glm::vec3 *v = face.vertices;
            const glm::vec3 *n = face.normals;
            const glm::vec2 *t = face.uvs;

            const glm::vec3 e1 = v[1] - v[0];
            const glm::vec3 e2 = v[2] - v[0];
            const glm::vec3 s = ray.origin - v[0];
            const glm::vec3 q = glm::cross(s, e1);
            const glm::vec3 p = glm::cross(ray.dir, e2);
            const float f = 1.0f / glm::dot(e1, p);
            const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));
            output[i].position = ray.origin + f * glm::dot(e2, q) * ray.dir;
            output[i].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
            output[i].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
            output[i].tet_idx = d_cons_faces[index].tet_idx;
            output[i].neighbor_tet_idx = d_cons_faces[index].other_tet_idx;

            output[i].hit = true;
        }
        else
            output[i].hit = false;
    }
}

//------------------------------------------------o-------------------------------------------------------

void copy_to_gpu_helper(TetMesh& tet_mesh)
{
    cudaFree(d_points);
    cudaMalloc(&d_points, tet_mesh.m_points.size() * sizeof(glm::vec3));
    cudaMemcpy(d_points, tet_mesh.m_points.data(), tet_mesh.m_points.size() * sizeof(glm::vec3), cudaMemcpyHostToDevice);

    cudaFree(d_cons_faces);
    cudaMalloc(&d_cons_faces, tet_mesh.m_constrained_faces.size() * sizeof(ConstrainedFace));
    cudaMemcpy(d_cons_faces, tet_mesh.m_constrained_faces.data(), tet_mesh.m_constrained_faces.size() * sizeof(ConstrainedFace), cudaMemcpyHostToDevice);

    cudaFree(d_faces);
    cudaMalloc(&d_faces, tet_mesh.faces.size() * sizeof(Face));
    cudaMemcpy(d_faces, tet_mesh.faces.data(), tet_mesh.faces.size() * sizeof(Face), cudaMemcpyHostToDevice);

    //print_cuda_error("CUDA copy error");
}

//------------------------------------------------o-------------------------------------------------------

void copy_to_gpu(TetMesh32& tet_mesh)
{
    copy_to_gpu_helper(tet_mesh);

    cudaFree(d_tets32);
    cudaMalloc(&d_tets32, tet_mesh.m_tets.size() * sizeof(TetMesh32::Tet32));
    cudaMemcpy(d_tets32, tet_mesh.m_tet32s, tet_mesh.m_tets.size() * sizeof(TetMesh32::Tet32), cudaMemcpyHostToDevice);
    
    print_cuda_error("CUDA copy error32");
}

void copy_to_gpu(TetMesh20& tet_mesh)
{
    copy_to_gpu_helper(tet_mesh);

    cudaFree(d_tets20);
    cudaMalloc(&d_tets20, tet_mesh.m_tets.size() * sizeof(TetMesh20::Tet20));
    cudaMemcpy(d_tets20, tet_mesh.m_tet20s.data(), tet_mesh.m_tets.size() * sizeof(TetMesh20::Tet20), cudaMemcpyHostToDevice);

    print_cuda_error("CUDA copy error20");
}

void copy_to_gpu(TetMesh16& tet_mesh)
{
    copy_to_gpu_helper(tet_mesh);

    cudaFree(d_tets16);
    cudaMalloc(&d_tets16, tet_mesh.m_tets.size() * sizeof(TetMesh16::Tet16));
    cudaMemcpy(d_tets16, tet_mesh.m_tet16s, tet_mesh.m_tets.size() * sizeof(TetMesh16::Tet16), cudaMemcpyHostToDevice);

    print_cuda_error("CUDA copy error16");
}

void ray_caster_gpu(Ray* rays, unsigned int rays_size, unsigned int tet_mesh_type, IntersectionData* output)
{
    // Allocate space for device copy of data
    if (old_size != rays_size)
    {
        cudaFree(d_rays);
        cudaFree(d_intersectdata);
        cudaMalloc(&d_rays, rays_size * sizeof(Ray));
        cudaMalloc(&d_intersectdata, rays_size * sizeof(IntersectionData));
        old_size = rays_size;
    }

    float copy_time = 0, kernel_time=0;

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    // Copy inputs to device
    cudaMemcpy(d_rays, rays, rays_size * sizeof(Ray), cudaMemcpyHostToDevice);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    copy_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
    //printf("Copy time of Rays to GPU: %f ms\n ", copy_time);


    // Launch kernel on GPU
    int t = 512;
    start = std::chrono::steady_clock::now();

    if (tet_mesh_type == 0 )
    {
        raycast_kernel <<< rays_size / t, t >>>(d_rays, rays_size, d_points, d_tets32, d_cons_faces, d_faces, d_intersectdata);
    }
    else if (tet_mesh_type == 1 )
    {
        raycast_kernel <<< rays_size / t, t >>>(d_rays, rays_size, d_points, d_tets20, d_cons_faces, d_faces, d_intersectdata);
    }
    else if (tet_mesh_type == 2 )
    {
        raycast_kernel <<< rays_size / t, t >>>(d_rays, rays_size, d_points, d_tets16, d_cons_faces, d_faces, d_intersectdata);
    }

    cudaDeviceSynchronize();

    //print_cuda_error("kernel error");

    end = std::chrono::steady_clock::now();
    kernel_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
    Stats::gpu_kernel_time = kernel_time;
    
    //printf("Kernel time %f ms\n ", kernel_time);

    // Copy result back to host
    cudaMemcpy(output, d_intersectdata, rays_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
}
