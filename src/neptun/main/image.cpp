#include "image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

Image::Image(int width, int height) : m_width(width), m_height(height)
{
    m_pixels = new glm::u8vec3[m_width * m_height];
}

Image::~Image()
{
    delete[] m_pixels;
}

glm::u8vec3* Image::get_pixels() const
{
    return m_pixels;
}

void Image::set_pixel(int x, int y, const glm::u8vec3 & pixel)
{
    m_pixels[(x * m_width + y)] = pixel;
}

void Image::save_to_disk(const char * file_name) const
{
    stbi_write_png(file_name, m_width, m_height, 3, m_pixels, 3 * m_width);
}
