// Host platform setup kept apart from the raylib-facing sources (windows.h
// clashes with raylib's names).
#include "dds2_platform.h"

#include <windows.h>

namespace dds2_platform
{
    namespace
    {
        bool g_timerRaised = false;
    }

    void raiseTimerResolution()
    {
        // 1 ms timer resolution for the scheduler's VBlank and alarm waits.
        g_timerRaised = timeBeginPeriod(1) == TIMERR_NOERROR;
    }

    void restoreTimerResolution()
    {
        if (g_timerRaised)
            timeEndPeriod(1);
        g_timerRaised = false;
    }
}
