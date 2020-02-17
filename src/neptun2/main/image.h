#pragma once

#include <glm/glm.hpp>

namespace neptun
{

class Image
{
public:
	explicit Image(const glm::u64vec2& resolution);
	~Image();

	glm::vec3& operator()(size_t x, size_t y);
	const glm::vec3& operator()(size_t x, size_t y) const;

	glm::u64vec2 resolution() const;
	size_t width() const;
	size_t height() const;
	glm::vec3* ptr();

	bool save_as_png(const char* fname);

private:
	glm::vec3*	 m_data;
	glm::u64vec2 m_resolution;
};

} // end of namespace neptun