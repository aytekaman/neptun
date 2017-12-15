#pragma once

#include <glm/glm.hpp>

class Image
{
public:
    Image(int width, int height);
    ~Image();

    glm::u8vec3* get_pixels() const;

    void set_pixel(int x, int y, const glm::u8vec3& pixel);
    void save_to_disk(const char* file_name) const;



private:
    const int m_width;
    const int m_height;
    glm::u8vec3* m_pixels = nullptr;
};