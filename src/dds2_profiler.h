#pragma once
#include <atomic>

namespace dds2_profiler
{
    // Starts the sampling profiler when DDS2_PROFILE is set.
    void start(std::atomic<bool> &stop);
}
