#pragma once

#include <glm/glm.hpp>
#include "hilbert.h"

class SfcUtils
{
public:
    static bitmask_t hilbert_idx(const glm::vec3& min, const glm::vec3& max, const glm::vec3& p, const unsigned int bit_count = 21U);
    static bitmask_t morton_idx (const glm::vec3& min, const glm::vec3& max, const glm::vec3& p, const unsigned int bit_count = 21U);

private:
    //static const unsigned int bit_count;
};