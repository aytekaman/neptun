#include "image.h"

// Add this in future
//#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#include <memory>
#include <iostream>

namespace neptun
{


Image::Image(const glm::u64vec2& resolution)
	: m_data(new glm::vec3[resolution.x * resolution.y])
	, m_resolution(resolution)
{
}

Image::~Image() 
{
	if (m_data)
		delete[] m_data;
}

glm::vec3& Image::operator()(size_t x, size_t y)
{
	return m_data[y * m_resolution.x + x];
}

const glm::vec3& Image::operator()(size_t x, size_t y) const 
{
	return m_data[y * m_resolution.x + x];
}

glm::u64vec2 Image::resolution() const
{
	return m_resolution;
}

size_t Image::width() const 
{
	return m_resolution.x;
}

size_t Image::height() const
{
	return m_resolution.y;
}

glm::vec3* Image::ptr()
{
	return m_data;
}


bool Image::save_as_png(const char* fname) 
{
	// Generate rgb array
	std::unique_ptr<glm::u8vec3[]> pixels(new glm::u8vec3[m_resolution.x * m_resolution.y]);

	size_t index = 0;
	for (size_t y = 0; y < m_resolution.y; ++y)
	{
		for (size_t x = 0; x < m_resolution.x; ++x)
		{
			pixels[index] = glm::u8vec3(glm::clamp(m_data[index], { 0 }, { 1 }) * 255.f);
			++index;
		}
	}

	const int success = stbi_write_png(fname, m_resolution.x, m_resolution.y, 3, pixels.get(), 3 * m_resolution.x);

	return success != 0;
}


} // end of namespace neptun