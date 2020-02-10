#include "mesh.h"

namespace neptun
{

size_t Mesh::face_count() const
{
	return m_vertices.size() / 3;
}

size_t Mesh::vertex_count() const
{
	return m_vertices.size();
}

// Dirty functions
bool Mesh::is_dirty() const
{
	return m_dirty;
}

void Mesh::clean()
{
	if (!m_dirty)
		return;

	for (size_t i = 0; i < m_vertices.size(); ++i)
	{
		m_bounds_min = glm::min(m_bounds_min, m_vertices[i]);
		m_bounds_max = glm::max(m_bounds_max, m_vertices[i]);
	}
	
	m_dirty = false;
}

void Mesh::mark_dirty()
{
	m_dirty = true;
}
} // end of namespace neptun