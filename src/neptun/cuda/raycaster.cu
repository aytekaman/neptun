#include "cuda_runtime.h"

#include "device_launch_parameters.h"

#include "neptun/main/tet_mesh.h"

#define GLM_FORCE_CUDA
#include "glm/glm.hpp"

#include <iostream>



glm::vec3* points;
Tet32* tets;
ConstrainedFace* faces;

SourceTet* source_tet_gpu;
CamInfo* cam_info_gpu;

//int* buffer;
HitData* hitdata;

//int buffer_cpu[640 * 480];
HitData hitdata_cpu[640 * 480];

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
void raycast_kernel(HitData* output, glm::vec3* m_points, Tet32* m_tet32s, ConstrainedFace* m_faces, CamInfo* cam_info, SourceTet* source_tet)
{
    glm::ivec2 resolution(640, 480);
    glm::ivec2 rect_min = glm::ivec2((blockIdx.x * tile_size) % resolution.x, ((blockIdx.x * tile_size) / resolution.x) * tile_size);
    glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);
    glm::ivec2 pixel_coords = rect_min + glm::ivec2(threadIdx.x, threadIdx.y);

    int outputindex = pixel_coords.x * resolution.y + pixel_coords.y;

    glm::vec3 origin = cam_info->origin;
    glm::vec3 dir = glm::normalize(cam_info->bottom_left + cam_info->right_step * (float)pixel_coords.x + cam_info->up_step * (float)pixel_coords.y - origin);

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
		output[outputindex].hit = 0;
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

	if (index != -1)
	{
		//index = (index & 0x7FFFFFFF);
		//const Face face = *m_faces[index].face;

		//const glm::vec3 *v = face.vertices;
		//const glm::vec3 *n = face.normals;
		////const glm::vec2 *t = face.uvs;

		//const glm::vec3 e1 = v[1] - v[0];
		//const glm::vec3 e2 = v[2] - v[0];
		//const glm::vec3 s = origin - v[0];
		//const glm::vec3 q = glm::cross(s, e1);
		//const glm::vec3 p = glm::cross(dir, e2);
		//const float f = 1.0f / glm::dot(e1, p);
		//const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(dir, q));
		//output[outputindex].point = origin + f * glm::dot(e2, q) * dir;
		//output[outputindex].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
		//intersection_data.uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
		//intersection_data.tet_idx = m_constrained_faces[index].tet_idx;
		//intersection_data.neighbor_tet_idx = m_constrained_faces[index].other_tet_idx;

		output[outputindex].hit = 1;
	}
	else
		output[outputindex].hit = 0;
}

void send_to_gpu(TetMesh32& tet_mesh)
{
	//hitdata_cpu = new HitData[640 * 480];

    glm::ivec2 resolution(640, 480);
    //buffer_cpu = new bool[resolution.x * resolution.y];

    std::cout << cudaMalloc(&hitdata, resolution.x * resolution.y * sizeof(HitData)) << std::endl;
    //cudaMemcpy(hitdata, hitdata_cpu, resolution.x * resolution.y * sizeof(HitData), cudaMemcpyHostToDevice);

    cudaMalloc(&points, tet_mesh.m_points.size() * sizeof(glm::vec3));
    cudaMemcpy(points, tet_mesh.m_points.data(), tet_mesh.m_points.size() * sizeof(glm::vec3), cudaMemcpyHostToDevice);

    cudaMalloc(&tets, tet_mesh.m_tet32s.size() * sizeof(Tet32));
    cudaMemcpy(tets, tet_mesh.m_tet32s.data(), tet_mesh.m_tet32s.size() * sizeof(Tet32), cudaMemcpyHostToDevice);

	cudaMalloc(&faces, tet_mesh.m_constrained_faces.size() * sizeof(ConstrainedFace));
	cudaMemcpy(faces, tet_mesh.m_constrained_faces.data(), tet_mesh.m_constrained_faces.size() * sizeof(ConstrainedFace), cudaMemcpyHostToDevice);

	cudaMalloc(&source_tet_gpu, sizeof(SourceTet));
	cudaMalloc(&cam_info_gpu, sizeof(CamInfo));
}

HitData* raycast_gpu(SourceTet* source_tet, CamInfo* cam_info)
{
	
	cudaMemcpy(source_tet_gpu, source_tet, sizeof(SourceTet), cudaMemcpyHostToDevice);
	cudaMemcpy(cam_info_gpu, cam_info, sizeof(CamInfo), cudaMemcpyHostToDevice);

    glm::ivec2 resolution(640, 480);
    dim3 g(tile_size, tile_size);
    raycast_kernel<<<1200, g>>>(hitdata, points, tets, faces, cam_info_gpu, source_tet_gpu);

    //std::cout << cudaThreadSynchronize() << std::endl;

    cudaMemcpy(hitdata_cpu, hitdata, resolution.x * resolution.y * sizeof(HitData), cudaMemcpyDeviceToHost);
 
    return hitdata_cpu;
}