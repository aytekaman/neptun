#include "cast.h"

#include <neptun2/main/scene.h>
#include <neptun2/util/parallel.h>


#include <glm/glm.hpp>

namespace neptun
{
void CastIntegrator::render(const Scene& scene, Image& image, const RenderingMode& rendering_mode)
{
	image.resize(scene.m_camera.m_resolution);

	parallel_for_2d(m_num_threads, image.resolution(), m_tile_size,
		[&scene, &image, &rendering_mode](size_t thread_id, const glm::u64vec2& min_tile_index, const glm::u64vec2& max_tile_index)
		{
			for (size_t y = min_tile_index.y; y < max_tile_index.y; ++y)
			{
				for (size_t x = min_tile_index.x; x < max_tile_index.x; ++x)
				{
					glm::u64vec2 pixel(x, y);

					RayHit rayhit;
					rayhit.ray = scene.m_camera.generate_ray(pixel);

					scene.m_accelerator->intersect1(rayhit);

					if (rendering_mode == RenderingMode::DEFAULT)
					{
						glm::vec3 radiance(0.f);
						if (rayhit.hit.geometry_id == INVALID_GEOMETRY_ID) // We miss
						{
							radiance = glm::vec3(0.1f);
						}
						else // We hit
						{
							Hit& hit = rayhit.hit;

							SceneObject* hit_object = scene.m_scene_objects[hit.geometry_id].get();
							Mesh* mesh = hit_object->m_mesh;

							const glm::vec3 p[3] = {
								hit_object->m_transform.transform_point(mesh->m_vertices[hit.primitive_id * 3]),
								hit_object->m_transform.transform_point(mesh->m_vertices[hit.primitive_id * 3 + 1]),
								hit_object->m_transform.transform_point(mesh->m_vertices[hit.primitive_id * 3 + 2]),
							};

							const glm::vec3 n[3] = {
								hit_object->m_transform.transform_normal(mesh->m_normals[hit.primitive_id * 3]),
								hit_object->m_transform.transform_normal(mesh->m_normals[hit.primitive_id * 3 + 1]),
								hit_object->m_transform.transform_normal(mesh->m_normals[hit.primitive_id * 3 + 2]),
							};

							const glm::vec3 position = p[0] * hit.bary.x + p[1] * hit.bary.y + p[2] * (1 - hit.bary.x - hit.bary.y);
							const glm::vec3 normal = n[0] * hit.bary.x + n[1] * hit.bary.y + n[2] * (1 - hit.bary.x - hit.bary.y);

							for (const auto& obj : scene.m_scene_objects)
							{
								if (obj->is_light())
								{
									glm::vec3 to_light = glm::normalize(obj->m_pos - position);
									float diffuse = glm::clamp(glm::dot(normal, to_light), 0.0f, 1.0f);
									radiance += obj->m_light->m_color * diffuse * obj->m_light->m_intensity;
								}
							}
						}

						image(x, y) = radiance;
					}
					else if (rendering_mode == RenderingMode::DEPTH)
					{
						float depth = 30.f; // Max depth

						if (rayhit.hit.geometry_id != INVALID_GEOMETRY_ID)
						{
							depth = rayhit.ray.max_t / glm::length(rayhit.ray.dir);
						}

						image(x, y) = glm::vec3(depth);
					}
					else if (rendering_mode == RenderingMode::NORMAL)
					{
						if (rayhit.hit.geometry_id == INVALID_GEOMETRY_ID)
						{
							image(x, y) = glm::vec3(0.1f);
						}
						else
						{
							Hit& hit = rayhit.hit;

							SceneObject* hit_object = scene.m_scene_objects[hit.geometry_id].get();
							Mesh* mesh = hit_object->m_mesh;
							const glm::vec3 n[3] = {
									hit_object->m_transform.transform_normal(mesh->m_normals[hit.primitive_id * 3]),
									hit_object->m_transform.transform_normal(mesh->m_normals[hit.primitive_id * 3 + 1]),
									hit_object->m_transform.transform_normal(mesh->m_normals[hit.primitive_id * 3 + 2]),
							};
							const glm::vec3 normal = n[0] * hit.bary.x + n[1] * hit.bary.y + n[2] * (1 - hit.bary.x - hit.bary.y);
						
							image(x, y) = normal * 0.5f + 0.5f;
						}
					}
					else if (rendering_mode == RenderingMode::VISIBILITY)
					{
						image(x, y) = glm::vec3(rayhit.hit.geometry_id == INVALID_GEOMETRY_ID ? 0.f : 1.f);
					}

					
				}
			}
		});
}


} // end of namespace neptun