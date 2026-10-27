#pragma once
#include <cstdint>

class PS2Runtime;

namespace dds2_debug
{
    // Installs the DDS2_WATCH_WRITE guest memory write watchpoint, if set.
    void installWriteWatch(uint8_t *rdram);

    // Installs the game heap consistency checks selected by DDS2_CHECK_HEAP
    // and DDS2_CHECK_SMALL (nothing is hooked when neither is set).
    void installHeapChecks(PS2Runtime &runtime);

    // Installs DDS2_TRACE_FN entry logging (arguments, VU0 registers and the
    // memory at $a0) for the listed guest functions.
    void installFunctionTrace(PS2Runtime &runtime);
}
