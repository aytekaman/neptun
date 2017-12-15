#include "sfc_utils.h"

bitmask_t SfcUtils::hilbert_idx(const glm::vec3& min, const glm::vec3& max, const glm::vec3& p, const unsigned int bit_count)
{
    glm::uvec3 coord_tmp = ((p - min) / (max - min)) * (float)(1 << bit_count);

    const bitmask_t coord[3] = { coord_tmp[0], coord_tmp[1], coord_tmp[2]};

    return hilbert_c2i(3, bit_count, coord);
}

bitmask_t SfcUtils::morton_idx(const glm::vec3& min, const glm::vec3& max, const glm::vec3& p, const unsigned int bit_count)
{
    glm::uvec3 coord_tmp = ((p - min) / (max - min)) * (float)(1 << bit_count);

    const bitmask_t coord[3] = { coord_tmp[0], coord_tmp[1], coord_tmp[2] };

    bitmask_t index = 0;

    for (unsigned int j = 0; j < bit_count; ++j)
    {
        bitmask_t mask = 1ULL << (bit_count - j - 1);

        index |= ((coord[0] & mask) << (bit_count * 2 - 0 - 2 * j));
        index |= ((coord[1] & mask) << (bit_count * 2 - 1 - 2 * j));
        index |= ((coord[2] & mask) << (bit_count * 2 - 2 - 2 * j));
    }

    return index;
}

//const unsigned int SfcUtils::bit_count = 21U;
