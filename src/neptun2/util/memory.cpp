#include "memory.h"

namespace neptun
{
constexpr size_t ALIGNMENT = 64;

void* alloc_aligned(size_t size)
{
#if defined(HAVE_ALIGNED_MALLOC)
    return _aligned_malloc(size, ALIGNMENT);
#elif defined(HAVE_POSIX_MEMALIGN)
    void* ptr;
    if (posix_memalign(&ptr, ALIGNMENT, size) != 0)
        ptr = nullptr;
    return ptr;
#else
    return memalign(ALIGNMENT, size);
#endif
}

void free_aligned(void* ptr)
{
    if (!ptr) return;
#if defined(HAVE_ALIGNED_MALLOC)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

MemoryArena::MemoryArena(size_t block_size) : m_block_size(block_size) {}

MemoryArena::~MemoryArena() 
{
    free_aligned(m_current_block);
    for (auto& block : m_used_blocks) free_aligned(block.second);
    for (auto& block : m_available_blocks) free_aligned(block.second);
}

void* MemoryArena::alloc(size_t num_bytes)
{
    // Round up _nBytes_ to minimum machine alignment
    num_bytes = ((num_bytes + 15) & (~15));

    if (m_current_block_pos + num_bytes > m_current_alloc_size)
    {
        // Add current block to _usedBlocks_ list
        if (m_current_block)
        {
            m_used_blocks.push_back(std::make_pair(m_current_alloc_size, m_current_block));
            m_current_block = nullptr;
            m_current_alloc_size = 0;
        }

        // Get new block of memory for _MemoryArena_

        // Try to get memory block from _availableBlocks_
        for (auto iter = m_available_blocks.begin(); iter != m_available_blocks.end(); ++iter) 
        {
            if (iter->first >= num_bytes)
            {
                m_current_alloc_size = iter->first;
                m_current_block = iter->second;
                m_available_blocks.erase(iter);
                break;
            }
        }

        // If no block is found create new block
        if (!m_current_block)
        {
            m_current_alloc_size = (std::max)(num_bytes, m_block_size);
            m_current_block = alloc_aligned<uint8_t>(m_current_alloc_size);
        }

        m_current_block_pos = 0;
    }

    void* ret = m_current_block + m_current_block_pos;
    m_current_block_pos += num_bytes;

    return ret;
}

void MemoryArena::reset()
{
    m_current_block_pos = 0;
    m_available_blocks.splice(m_available_blocks.begin(), m_used_blocks);
}

size_t MemoryArena::total_allocated() const
{
    size_t total = m_current_alloc_size;
    for (const auto& alloc : m_used_blocks) total += alloc.first;
    for (const auto& alloc : m_available_blocks) total += alloc.first;
    return total;
}

size_t MemoryArena::total_used() const
{
    size_t total = m_current_block_pos;
    for (const auto& alloc : m_used_blocks) total += alloc.first;
    return total;
}

} // end of namespace neptun

