
#pragma once

#include <glm/vec2.hpp>

#include <vector>

template<class T>
class Matrix
{
public:
    Matrix();
    ~Matrix();

    void set_size(glm::ivec2 size)
    {
        if (size == m_size)
            return;

        m_data.resize(m_size.x * m_size.y);
    }

    T& get(const glm::ivec2& index) const
    {
        return m_data[index.x * m_size.y + index.y];
    }

    size_t get_element_count()
    {
        return m_size.x * m_size.y;
    }

private:
    std::vector<T> m_data;
    glm::ivec2 m_size;
};
