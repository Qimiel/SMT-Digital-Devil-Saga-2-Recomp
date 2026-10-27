#pragma once

namespace dds2_platform
{
    // Requests 1 ms OS timer resolution for the run (and restores it).
    void raiseTimerResolution();
    void restoreTimerResolution();
}
