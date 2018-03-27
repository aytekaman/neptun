#include "memory.h"

void *AllocAligned(size_t size)
{
#if defined(HAVE__ALIGNED_MALLOC)
    return _aligned_malloc(size, 64);
#elif defined(HAVE_POSIX_MEMALIGN)
    void *ptr;
    if (posix_memalign(&ptr, 64, size) != 0)
        ptr = nullptr;
    return ptr;
#else
    return memalign(64, size);
#endif
}

void FreeAligned(void *ptr)
{
    if (!ptr) return;
#if defined(HAVE__ALIGNED_MALLOC)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}
