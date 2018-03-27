#include "utils.h"

#include <ctime>

std::string Utils::get_timestamp()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);

    char buf[128];
#ifdef HAVE_SNPRINTF
    snprintf(buf, 128, "%d_%d_%d_%d_%d_%d",
        1900 + ltm->tm_year,
        1 + ltm->tm_mon,
        ltm->tm_mday,
        1 + ltm->tm_hour,
        1 + ltm->tm_min,
        1 + ltm->tm_sec);
#elif
    sprintf_s(buf, 128, "%d_%d_%d_%d_%d_%d",
        1900 + ltm->tm_year,
        1 + ltm->tm_mon,
        ltm->tm_mday,
        1 + ltm->tm_hour,
        1 + ltm->tm_min,
        1 + ltm->tm_sec);
#endif

    return std::string(buf);
}
