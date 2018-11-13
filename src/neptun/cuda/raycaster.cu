#include "raycaster.cuh";

glm::ivec2 m_resolution = glm::ivec2(640, 480); //Hardcoded !!!!


//__device__
//void swap(float &a, float &b)
//{
//    float t = a;
//    a = b;
//    b = t;
//}

__global__
void raycast_kernel(IntersectionData* output, glm::vec3* m_points, TetMesh32::Tet32* m_tet32s, ConstrainedFace* m_faces, Face* m_mesh_faces, 
	Scene& scene, SourceTet source_tet, Ray ray, glm::vec3 top_left, glm::vec3 right_step, glm::vec3 down_step, std::vector<LightInfo> lightInfos, bool is_diagnostic)
{
	int index = blockIdx.x * blockDim.x + threadIdx.x;
	const int max_job_index = m_resolution.x * m_resolution.y;

	if (index < max_job_index)
	{
		int i = index % m_resolution.x;
		int j = (index - i) / m_resolution.x;

		//rect_max = (glm::min)(rect_max, m_resolution);

		ray.dir = glm::normalize(top_left + right_step * (float)i + down_step * (float)j - ray.origin);

		glm::vec3 pos, normal, color;
		glm::vec2 uv;
		Face face;

		DiagnosticData diagnostic_data;
		diagnostic_data.total_tet_distance = 0;
		diagnostic_data.visited_node_count = 0;
		diagnostic_data.L1_hit_count = 0;

		bool hit = false;

		IntersectionData intersection_data;

		/*if (method == Method::Default ||
			method == Method::Fast_basis ||
			method == Method::ScTP)*/
		ray.tet_idx = 0;
		/*else
			ray.tMax = 100000000;*/

		if (is_diagnostic)
		{
			//if (method == Method::Default || method == Method::Fast_basis || method == Method::ScTP)
			hit = scene.tet_mesh->intersect_stats(ray, source_tet, intersection_data, diagnostic_data);
			/*else if (method == Method::Kd_tree)
				hit = scene.kd_tree->Intersect_stats(ray, intersection_data, diagnostic_data);
			else if (method == Method::BVH_pbrt)
				hit = scene.bvh->Intersect_stats(ray, intersection_data, diagnostic_data);*/
		}

		else
		{
			//if (method == Method::Default)
			hit = scene.tet_mesh->intersect(ray, source_tet, intersection_data);
			/*else if (method == Method::DefaultSimd)
				hit = scene.tet_mesh->intersect_simd(ray, source_tet, intersection_data);
			//else if (method == Method::ScTP)
			//    hit = scene.tet_mesh->intersect_sctp(ray, intersection_data);
			//else if (method == Method::Fast_basis)
			//    hit = scene.tet_mesh->intersect_optimized_basis(ray, intersection_data);
			else if (method == Method::Kd_tree)
				hit = scene.kd_tree->Intersect(ray, intersection_data);
			else if (method == Method::BVH_embree)
				hit = scene.bvh_embree->Intersect(ray, intersection_data);
			else if (method == Method::BVH_pbrt)
				hit = scene.bvh->Intersect(ray, intersection_data);*/
		}

		if (hit)
		{

			color = glm::vec3();

			for (int light_idx = 0; light_idx < lightInfos.size(); light_idx++)
			{
				Ray shadow_ray(intersection_data.position, glm::normalize(lightInfos[light_idx].pos - intersection_data.position));

				// check normals before shooting
				//if (!shadows || scene.tet_mesh->intersect(shadow_ray, intersection_data.tet_idx, lightInfos[light_idx].tet_index))
				{
					glm::vec3 to_light = glm::normalize(lightInfos[light_idx].pos - intersection_data.position);
					float diffuse = glm::clamp(glm::dot(intersection_data.normal, to_light), 0.0f, 1.0f);
					//diffuse = 1.0f;
					color += lightInfos[light_idx].color * diffuse * lightInfos[light_idx].intensity;
				}
			}

			//color = glm::clamp(color, 0.0f, 1.0f);

			//float u = (glm::atan(pos.x, pos.z) / glm::pi<float>()) * 0.5f + 0.5f;
			//float v = 0.5 - (glm::atan(pos.y, glm::sqrt(pos.x * pos.x + pos.z * pos.z)) / glm::pi<float>());

			//glm::vec3 d(face.material->diffuse);

			//if (face.material->texture)
			//	d = face.material->texture->GetPixelBilinear(uv.x, uv.y) * face.material->diffuse;

			//color.r *= d.r;
			//color.g *= d.g;
			//color.b *= d.b;
		}
		else
			color = glm::vec3(0.1, 0.1, 0.1);
		
		glm::ivec2 p_idx(i, j);
		if (is_diagnostic)
		{
			if (scene.tet_mesh)
			{
				total_L1_hit_count += diagnostic_data.L1_hit_count;//Shared variable ??

				stats.set(p_idx, diagnostic_data.visited_node_count);

				float avg_locality = diagnostic_data.total_tet_distance / diagnostic_data.visited_node_count;
				float scaled_avg_locality = (avg_locality / scene.tet_mesh->m_tets.size()) * 2.0f;
				glm::vec3 avg_locality_color = Color::jet(scaled_avg_locality);
				//m_locality_image->set_pixel((m_resolution.y - i - 1), (m_resolution.x - j - 1), avg_locality_color * 255.0f);
			}

			float scaled_visited_tet_count = diagnostic_data.visited_node_count / 256.0f;
			glm::vec3 visited_tet_count_color = Color::jet(scaled_visited_tet_count);
			m_visited_tets_image->set_pixel(i, j, visited_tet_count_color * 255.0f);//shared variable ???

			total_test_count += diagnostic_data.visited_node_count; //shared variable ???
		}

		m_rendered_image->set_pixel(p_idx.x, p_idx.y, glm::vec3(color.z, color.y, color.x) * 255.0f); //shared variable???
	}

	/*traversed_tetra_count[thread_idx] = total_test_count / ((m_resolution.x * m_resolution.y) / (float)thread_count);
	L1_hit_count[thread_idx] = total_L1_hit_count;*/ //TODO:: UNCOMMENT AND PARALILIZE
}
__device__
void raycast_gpu(Scene& scene, SourceTet source_tet, int thread_count, std::vector<LightInfo> lightInfos, bool is_diagnostic)
{
	const glm::vec3 camTarget = scene.camTarget;
	glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));

	dir = dir * glm::cos(scene.camOrbitX);
	dir.y = glm::sin(scene.camOrbitX);

	glm::vec3 cam_pos = camTarget + dir * scene.camDist;

	const glm::vec3 forward = glm::normalize(scene.camTarget - cam_pos);
	const glm::vec3 right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), forward));
	const glm::vec3 down = glm::cross(forward, right);

	const float aspect = (float)m_resolution.x / m_resolution.y;
	const float scale_y = glm::tan(glm::pi<float>() / 8);

	const glm::vec3 top_left = cam_pos + forward - down * scale_y - right * scale_y * aspect;
	const glm::vec3 right_step = (right * scale_y * 2.0f * aspect) / (float)m_resolution.x;
	const glm::vec3 down_step = (down * scale_y * 2.0f) / (float)m_resolution.y;

	Ray ray(cam_pos);

	int total_test_count = 0;//TODO: INIT as CUDAmalloc
	int total_L1_hit_count = 0;

	/*const int tile_count_x = (m_resolution.x + tile_size - 1) / tile_size;
	const int tile_count_y = (m_resolution.y + tile_size - 1) / tile_size;*/
}