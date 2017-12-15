#pragma once

#include "glm/glm.hpp"

struct OptimizedBasis
{
    OptimizedBasis(const glm::vec3& forward)
    {
        idx_of_other = 0;
        float max_val = std::abs(forward[0]);

        for (int i = 1; i < 3; i++)
        {
            const float abs_val = std::abs(forward[i]);

            if (abs_val > max_val)
            {
                idx_of_other = i;
                max_val = abs_val;
            }
        }

        const int prev_idx = idx_of_other == 0 ? 2 : idx_of_other - 1;
        const int next_idx = idx_of_other == 2 ? 0 : idx_of_other + 1;

        const bool is_next_bigger = std::abs(forward[prev_idx]) < std::abs(forward[next_idx]);

        idx_of_one = is_next_bigger ? next_idx : prev_idx;

        m_other = -forward[idx_of_one] / forward[idx_of_other];

        glm::vec3 right(0.0f, 0.0f, 0.0f);

        right[idx_of_other] = m_other;
        right[idx_of_one] = 1.0f;

        m_up = glm::cross(forward, right);

        is_pos = m_up[3 - idx_of_one - idx_of_other] > 0;

        m_up /= std::abs(m_up[3 - idx_of_one - idx_of_other]);
    }

    float m_other;
    glm::vec3 m_up;
    int idx_of_one;
    int idx_of_other;
    bool is_pos;
};

struct Basis
{
    Basis(const glm::vec3& forward)
    {
        const float sign = copysignf(1.0f, forward.z);

        const float a = -1.0f / (sign + forward.z);
        const float b = forward.x * forward.y * a;

        right = glm::vec3(1.0f + sign * forward.x * forward.x * a, sign * b, -sign * forward.x);
        up = glm::vec3(b, sign + forward.y * forward.y * a, -forward.y);
    }

    glm::vec3 right;
    glm::vec3 up;
};