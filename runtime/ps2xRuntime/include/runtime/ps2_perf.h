#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>

// Host time (ns) spent per subsystem on the EE thread, reported every 300
// frames by gs_threaded_backend.cpp when PS2X_TRACE_GS_PERF is set.
extern std::atomic<uint64_t> g_ps2xPerfNs[4];

enum class Ps2xPerfSlot
{
    Vif1 = 0,
    Vu1 = 1,
    GsPackets = 2,
    Vu0 = 3,
};

// Adds the lifetime of the scope to a slot (only with PS2X_TRACE_GS_PERF).
class Ps2xPerfScope
{
public:
    explicit Ps2xPerfScope(Ps2xPerfSlot slot)
        : m_slot(static_cast<int>(slot)), m_active(enabled())
    {
        if (m_active)
            m_start = std::chrono::steady_clock::now();
    }

    ~Ps2xPerfScope()
    {
        if (m_active)
        {
            const auto elapsed = std::chrono::steady_clock::now() - m_start;
            g_ps2xPerfNs[m_slot].fetch_add(
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count()),
                std::memory_order_relaxed);
        }
    }

    Ps2xPerfScope(const Ps2xPerfScope &) = delete;
    Ps2xPerfScope &operator=(const Ps2xPerfScope &) = delete;

    static bool enabled()
    {
        static const bool on = std::getenv("PS2X_TRACE_GS_PERF") != nullptr;
        return on;
    }

private:
    int m_slot;
    bool m_active;
    std::chrono::steady_clock::time_point m_start{};
};
