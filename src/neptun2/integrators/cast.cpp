#include "cast.h"

#include <neptun2/main/scene.h>
#include <neptun2/util/parallel.h>


#include <glm/glm.hpp>

namespace neptun
{
void CastIntegrator::render(Scene& scene)
{
	Image& rendered_image = *scene.m_rendered_image.get();
	parallel_for_2d(m_num_threads, rendered_image.resolution(), m_tile_size,
					[&scene, &rendered_image](size_t thread_id, const glm::u64vec2& min_tile_index, const glm::u64vec2& max_tile_index)
					{
						for (size_t y = min_tile_index.y; y < max_tile_index.y; ++y)
						{
							for (size_t x = min_tile_index.x; x < max_tile_index.x; ++x)
							{
								glm::u64vec2 pixel(x, y);

								RayHit rayhit;
								rayhit.ray = scene.m_camera.generate_ray(pixel);
								rayhit.hit.geometry_id = INVALID_GEOMETRY_ID;

								scene.m_accelerator->intersect1(rayhit);

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
								
								rendered_image(x, y) = radiance;
							}
						}
					});
}


} // end of namespace neptun