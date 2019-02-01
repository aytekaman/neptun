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

cudaStream_t* streams;

unsigned int old_size = 0;
const int NSTREAMS = 3;

__global__
void Raycast_kernel(Scene & scene, SourceTet source_tet, glm::ivec2 resolution, int offset, int tile_size, Ray *output)
{
    int idx = offset + blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < resolution.x*resolution.y)
    {
        const glm::vec3 camTarget = scene.camTarget;
        glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));

        dir = dir * glm::cos(scene.camOrbitX);
        dir.y = glm::sin(scene.camOrbitX);

        glm::vec3 cam_pos = camTarget + dir * scene.camDist;

        const glm::vec3 forward = glm::normalize(scene.camTarget - cam_pos);
        const glm::vec3 right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), forward));
        const glm::vec3 down = glm::cross(forward, right);

        const float aspect = (float)resolution.x / resolution.y;
        const float scale_y = glm::tan(glm::pi<float>() / 8);

        const glm::vec3 top_left = cam_pos + forward - down * scale_y - right * scale_y * aspect;
        const glm::vec3 right_step = (right * scale_y * 2.0f * aspect) / (float)resolution.x;
        const glm::vec3 down_step = (down * scale_y * 2.0f) / (float)resolution.y;

        Ray ray(cam_pos);

        int total_test_count = 0;
        int total_L1_hit_count = 0;


        const int tile_count_x = (resolution.x + tile_size - 1) / tile_size;
        const int tile_count_y = (resolution.y + tile_size - 1) / tile_size;
        const int max_job_index = tile_count_x * tile_count_y;
            
        glm::ivec2 rect_min = glm::ivec2((idx % tile_count_x) * tile_size, (idx / tile_count_x) * tile_size);
        glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);

        rect_max = (glm::min)(rect_max, resolution);

        //j = rect_min.y
        //i = rect_min.x
        ray.dir = glm::normalize(top_left + right_step * (float)(rect_min.x + idx % tile_size)+ down_step * ((float)(idx/resolution.x))- ray.origin);//!!!!!!!!!!!!!!!!!
        ray.source_tet = source_tet;

        glm::vec3 pos, normal;
        glm::vec2 uv;
        Face face;

        DiagnosticData diagnostic_data;
        diagnostic_data.total_tet_distance = 0;
        diagnostic_data.visited_node_count = 0;
        diagnostic_data.L1_hit_count = 0;

        // int tet_index_copy = tet_index;


        ray.tet_idx = 0;

        output[idx] = ray;
    }
}

__global__
void Ray_traversal_kernel(Ray *rays, int rays_size, int offset, glm::vec3* d_points, TetMesh32::Tet32* d_tets, 
    ConstrainedFace* d_cons_faces, Face* d_faces, IntersectionData *output)
{
    int i = offset + blockIdx.x * blockDim.x + threadIdx.x;
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
void Ray_traversal_kernel(Ray* rays, int rays_size, int offset, glm::vec3* d_points, TetMesh20::Tet20* d_tets,
    ConstrainedFace* d_cons_faces, Face* d_faces, IntersectionData *output)
{
    int i = offset + blockIdx.x * blockDim.x + threadIdx.x;
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
void Ray_traversal_kernel(Ray* rays, int rays_size, int offset, glm::vec3* d_points, TetMesh16::Tet16* d_tets,
    ConstrainedFace* d_cons_faces, Face* d_faces, IntersectionData *output)
{
    int i = offset + blockIdx.x * blockDim.x + threadIdx.x;
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

void Traverse_rays_gpu(Ray* rays, unsigned int rays_size, unsigned int tet_mesh_type, IntersectionData* output)
{
    // Allocate space for device copy of data
    if (old_size != rays_size)
    {
        cudaFree(d_rays);
        cudaFree(d_intersectdata);
        cudaMalloc(&d_rays, rays_size * sizeof(Ray));
        cudaMalloc(&d_intersectdata, rays_size * sizeof(IntersectionData));
        old_size = rays_size;

		streams = new cudaStream_t[NSTREAMS];
		for (int i = 0; i < NSTREAMS; i++)
		{
			cudaStreamCreate(&streams[i]);
			print_cuda_error("Stream creation error");
		}
    }

    unsigned int stream_size = rays_size / NSTREAMS;
    int stream_bytes = stream_size * sizeof(Ray);

    float kernel_time=0;

    std::chrono::steady_clock::time_point start, end;

    // Launch kernel on GPU
    int t = 512;

    start = std::chrono::steady_clock::now();

   // for (int i = 0; i < NSTREAMS; i++) {
        //int offset = i * stream_size;
        //cudaMemcpyAsync(&d_rays[offset], &rays[offset], stream_bytes, cudaMemcpyHostToDevice, streams[i]);
    //cudaMemcpy(d_rays, rays, rays_size * sizeof(Ray), cudaMemcpyHostToDevice);
    if (tet_mesh_type == 0)
    {
        //raycast_kernel <<< stream_size / t, t,  0, streams[i]>>> (d_rays, rays_size, offset, d_points, d_tets32, d_cons_faces, d_faces, d_intersectdata);
        Ray_traversal_kernel <<< rays_size / t, t >>> (d_rays, rays_size, 0, d_points, d_tets32, d_cons_faces, d_faces, d_intersectdata);
    }
    else if (tet_mesh_type == 1)
    {
        Ray_traversal_kernel <<< rays_size / t, t >>> (d_rays, rays_size, 0, d_points, d_tets20, d_cons_faces, d_faces, d_intersectdata);
    }
    else if (tet_mesh_type == 2)
    {
        Ray_traversal_kernel <<< rays_size / t, t >>> (d_rays, rays_size, 0, d_points, d_tets16, d_cons_faces, d_faces, d_intersectdata);
    }

        //print_cuda_error("kernel");
        //cudaMemcpyAsync(&output[offset], &d_intersectdata[offset], stream_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost, streams[i]);
    cudaMemcpy(output, d_intersectdata, rays_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
		//print_cuda_error("copyback");
   // }
    //cudaDeviceSynchronize();

    end = std::chrono::steady_clock::now();
    kernel_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
    Stats::gpu_kernel_time = kernel_time;
}

void Traverse_rays_gpu(Scene & scene, SourceTet source_tet, glm::ivec2 resolution, unsigned int rays_size, unsigned int tet_mesh_type, IntersectionData* output)
{
    // Allocate space for device copy of data
    if (old_size != rays_size)
    {
        cudaFree(d_rays);
        cudaFree(d_intersectdata);
        cudaMalloc(&d_rays, rays_size * sizeof(Ray));
        cudaMalloc(&d_intersectdata, rays_size * sizeof(IntersectionData));
        old_size = rays_size;

        streams = new cudaStream_t[NSTREAMS];
        for (int i = 0; i < NSTREAMS; i++)
        {
            cudaStreamCreate(&streams[i]);
            print_cuda_error("Stream creation error");
        }
    }

    unsigned int stream_size = rays_size / NSTREAMS;
    int stream_bytes = stream_size * sizeof(Ray);

    float kernel_time = 0;

    std::chrono::steady_clock::time_point start, end;

    // Launch kernel on GPU
    int t = 512;

    start = std::chrono::steady_clock::now();

    // for (int i = 0; i < NSTREAMS; i++) {
         //int offset = i * stream_size;
         //cudaMemcpyAsync(&d_rays[offset], &rays[offset], stream_bytes, cudaMemcpyHostToDevice, streams[i]);
     //cudaMemcpy(d_rays, rays, rays_size * sizeof(Ray), cudaMemcpyHostToDevice);
    Ray_prepare_kernel <<< rays_size / t, t >>> (scene, source_tet, resolution, 0, 8, d_rays);
        if (tet_mesh_type == 0)
        {
            //raycast_kernel <<< stream_size / t, t,  0, streams[i]>>> (d_rays, rays_size, offset, d_points, d_tets32, d_cons_faces, d_faces, d_intersectdata);
            Ray_traversal_kernel <<< rays_size / t, t >>> (d_rays, rays_size, 0, d_points, d_tets32, d_cons_faces, d_faces, d_intersectdata);
        }
        else if (tet_mesh_type == 1)
        {
            Ray_traversal_kernel << < rays_size / t, t >> > (d_rays, rays_size, 0, d_points, d_tets20, d_cons_faces, d_faces, d_intersectdata);
        }
        else if (tet_mesh_type == 2)
        {
            Ray_traversal_kernel << < rays_size / t, t >> > (d_rays, rays_size, 0, d_points, d_tets16, d_cons_faces, d_faces, d_intersectdata);
        }

    //print_cuda_error("kernel");
    //cudaMemcpyAsync(&output[offset], &d_intersectdata[offset], stream_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost, streams[i]);
    cudaMemcpy(output, d_intersectdata, rays_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
    //print_cuda_error("copyback");
// }
 //cudaDeviceSynchronize();

    end = std::chrono::steady_clock::now();
    kernel_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
    Stats::gpu_kernel_time = kernel_time;
}

void Traverse_rays_gpu(Ray* rays, unsigned int rays_size, int num_streams, unsigned int tet_mesh_type, int ids, IntersectionData* output)
{
    if (!streams)
    {
        streams = new cudaStream_t[num_streams];
    }
    // Allocate space for device copy of data
    if (old_size != rays_size)
    {
        old_size = rays_size;
        cudaFree(d_rays);
        cudaFree(d_intersectdata);
        cudaMalloc(&d_rays, rays_size * sizeof(Ray));
        cudaMalloc(&d_intersectdata, rays_size * sizeof(IntersectionData));
    }
    cudaStreamCreate(&streams[ids]);
    unsigned int stream_size = rays_size / num_streams;
    int stream_bytes = stream_size * sizeof(Ray);

    int t = 512;
    int offset = ids * stream_size;

    if (streams)
    {
        cudaMemcpyAsync(&d_rays[offset], &rays[offset], stream_bytes, cudaMemcpyHostToDevice, streams[ids]);

        if (tet_mesh_type == 0)
        {
            Ray_traversal_kernel <<< stream_size / t, t, 0, streams[ids] >>> (d_rays, rays_size, offset, d_points, d_tets32, d_cons_faces, d_faces, d_intersectdata);
        }
        else if (tet_mesh_type == 1)
        {
            Ray_traversal_kernel <<< stream_size / t, t, 0, streams[ids] >>> (d_rays, rays_size, offset, d_points, d_tets20, d_cons_faces, d_faces, d_intersectdata);
        }
        else if (tet_mesh_type == 2)
        {
            Ray_traversal_kernel <<< stream_size / t, t, 0, streams[ids] >>> (d_rays, rays_size, offset, d_points, d_tets16, d_cons_faces, d_faces, d_intersectdata);
        }

        offset = ids * stream_size;
        cudaMemcpyAsync(&output[offset], &d_intersectdata[offset], stream_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost, streams[ids]);
        cudaStreamDestroy(streams[ids]);
    }
}
