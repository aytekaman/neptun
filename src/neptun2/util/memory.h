#pragma once

#include <algorithm>
#include <cstdint>
#include <list>

namespace neptun
{

void* alloc_aligned(size_t size);
void  free_aligned(void*);

template <typename T>
T* alloc_aligned(size_t count)
{
    return (T*)alloc_aligned(count * sizeof(T));
}

class alignas(64) MemoryArena
{
public:
    MemoryArena(size_t block_size = 262144);
    ~MemoryArena();

    void* alloc(size_t num_bytes);

    template <typename T>
    T* Alloc(size_t n = 1, bool run_constructor = true) {
        T* ret = (T*)alloc(n * sizeof(T));
        if (run_constructor)
            for (size_t i = 0; i < n; ++i) new (&ret[i]) T();
        return ret;
    }

    void reset();
    size_t total_allocated() const;
    size_t total_used() const;

private:
    MemoryArena(const MemoryArena&) = delete;
    MemoryArena& operator=(const MemoryArena&) = delete;

    const size_t m_block_size;
    size_t m_current_block_pos = 0;
    size_t m_current_alloc_size = 0;
    uint8_t* m_current_block = nullptr;
    std::list<std::pair<size_t, uint8_t*>> m_used_blocks, m_available_blocks;
};

} // end of namespace neptun

// Memory Declarations
#define ARENA_ALLOC(arena, Type) new ((arena).Alloc(sizeof(Type))) Type

