#pragma once

#include <glm/glm.hpp>

#include <iostream>
#include <thread>
#include <mutex>
#include <functional>
#include <atomic>

namespace neptun
{


// Runs func in parallel
// func must be a function that takes (thread_id, min_tile_index, max_tile_index, other args ...)
//
template <typename Func, typename ... FuncArgs>
void parallel_for_2d(size_t num_threads, glm::u64vec2 work_size, size_t tile_size, Func&& func, FuncArgs&& ... args)
{
    constexpr size_t MAX_NUM_THREADS = 32;
    num_threads = glm::min(num_threads, MAX_NUM_THREADS);

    // Ignore incomplete tiles
    const glm::u64vec2 tile_count = (work_size + tile_size - 1ull) / tile_size;
    const size_t total_tile_count = tile_count.x * tile_count.y;

    std::thread threads[MAX_NUM_THREADS];
    std::atomic_size_t tile_index = num_threads;

    for (size_t tid = 0; tid < num_threads; ++tid)
    {
        threads[tid] = std::thread([&](size_t tid) {

            size_t index = tid;
            while (index < total_tile_count)
            {
                const glm::u64vec2 tile_start((index % tile_count.x) * tile_size, (index / tile_count.x) * tile_size);
                const glm::u64vec2 tile_end = glm::min(tile_start + tile_size, work_size);

                func(tid, tile_start, tile_end, args...);

                index = tile_index++;
            }
        }, tid);
    }

    for (size_t i = 0; i < num_threads; ++i)
    {
        threads[i].join();
    }
}

} // end of namespace neptun
