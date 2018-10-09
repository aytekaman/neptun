
#pragma once

#include <glm/vec2.hpp>

#include <vector>

template<class T>
class Matrix
{
public:
    Matrix()
    {

    }

    Matrix(const glm::ivec2& size)
    {

    }

    ~Matrix()
    {

    }

    void set_size(const glm::ivec2& size)
    {
        if (size == m_size)
            return;

        m_size = size;

        m_data.resize(m_size.x * m_size.y);

    }

    T get(const glm::ivec2& index) const
    {
        return m_data[index.x * m_size.y + index.y];
    }

    size_t get_element_count() const
    {
        return m_size.x * m_size.y;
    }

    void set(const glm::ivec2& index, const T& value)
    {
        m_data[index.x * m_size.y + index.y] = value;
    }

private:
    std::vector<T> m_data;
    glm::ivec2 m_size;
};
