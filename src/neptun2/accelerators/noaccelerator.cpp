#include "noaccelerator.h"

#include <glm/glm.hpp>
#include <glm/gtx/intersect.hpp>

namespace neptun
{

bool NoAccelerator::build(const Triangle* primitives, size_t primitive_count)
{
	m_triangles.resize(primitive_count);
	for (size_t i = 0; i < primitive_count; ++i)
	{
		m_triangles[i] = primitives[i];
	}

	return true;
}

void NoAccelerator::intersect1(RayHit& ray_hit) const
{
	Ray& ray = ray_hit.ray;
	Hit& hit = ray_hit.hit;

	size_t hit_index = m_triangles.size();
	for (size_t i = 0; i < m_triangles.size(); ++i)
	{
		const Triangle& t = m_triangles[i];

		glm::vec3 bary;
		if (glm::intersectRayTriangle(ray.org, ray.dir, t.v[0], t.v[1], t.v[2], bary))
		{
			if (bary.z < ray.max_t)
			{
				ray.max_t = bary.z;

				hit.bary.x = bary.x;
				hit.bary.y = bary.y;
			}
		}
	}

	// If we hit fill hit data
	// We do not fill these each time we hit
	if (hit_index != m_triangles.size())
	{
		const Triangle& t = m_triangles[hit_index];
		hit.geometry_id = t.geometry_id;
		hit.primitive_id = t.primitive_id;
		hit.n = glm::normalize(glm::cross(t.v[1] - t.v[0], t.v[2] - t.v[1]));
	}
}

void NoAccelerator::intersect1_stats(RayHit& ray_hit, Stats& stats) const
{
	stats.traversal_count = 0;
	stats.hit_test_count = m_triangles.size();

	intersect1(ray_hit);
}

const char* NoAccelerator::name() const
{
	return "NoAccelerator";
}

size_t NoAccelerator::get_size_in_bytes() const
{
	return m_triangles.size() * sizeof(Triangle);
}

} // end of namespace neptun