// DDS2 native launcher: boots the statically recompiled SLUS_211.52 under the
// vendored ps2xRuntime. The game data stays external; point the launcher at the
// directory holding the disc files (SLUS_211.52, DDS3.DDT, DDS3.IMG, ...).
#include "dds2_debug.h"
#include "dds2_platform.h"
#include "dds2_profiler.h"
#include "ps2_runtime.h"
#include "runtime/ee_scheduler.h"
#include "vu1rec/vu1rec.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

// Presentation loop progress, defined in ps2_runtime.cpp.
extern std::atomic<uint32_t> g_ps2xPresentStage;
extern std::atomic<uint64_t> g_ps2xPresentedFrames;

namespace
{
    constexpr const char *kElfName = "SLUS_211.52";

    std::filesystem::path findGameElf(int argc, char *argv[])
    {
        namespace fs = std::filesystem;
        std::error_code ec;
        auto resolve = [&](const fs::path &candidate) -> fs::path
        {
            if (fs::is_regular_file(candidate, ec))
                return candidate;
            if (fs::is_directory(candidate, ec) && fs::is_regular_file(candidate / kElfName, ec))
                return candidate / kElfName;
            return {};
        };

        if (argc >= 2 && argv[1] && argv[1][0] != '\0')
        {
            if (fs::path p = resolve(argv[1]); !p.empty())
                return p;
            throw std::runtime_error(std::string("No ") + kElfName + " at " + argv[1]);
        }
        if (const char *env = std::getenv("DDS2_GAME_DIR"))
        {
            if (fs::path p = resolve(env); !p.empty())
                return p;
        }
        // Search the executable's directory and its parents (the build tree lives
        // inside the extracted disc directory during development).
        fs::path dir = fs::absolute(argv[0], ec).parent_path();
        for (int depth = 0; depth < 6 && !dir.empty(); ++depth)
        {
            if (fs::path p = resolve(dir); !p.empty())
                return p;
            if (fs::path p = resolve(dir / "game"); !p.empty())
                return p;
            fs::path parent = dir.parent_path();
            if (parent == dir)
                break;
            dir = parent;
        }
        throw std::runtime_error(std::string("Cannot find ") + kElfName +
                                 ". Pass the disc directory as the first argument or set DDS2_GAME_DIR.");
    }

    // DDS2_HEARTBEAT=<seconds> prints the guest thread table periodically; used
    // to locate stalls while bringing the port up.
    void startHeartbeat(PS2Runtime &runtime, std::atomic<bool> &stop)
    {
        const char *env = std::getenv("DDS2_HEARTBEAT");
        const int seconds = env ? std::atoi(env) : 0;
        if (seconds <= 0)
            return;
        std::thread([&runtime, &stop, seconds]()
                    {
            while (!stop.load())
            {
                std::this_thread::sleep_for(std::chrono::seconds(seconds));
                const EeKernelSnapshot snap = runtime.eeScheduler().snapshot();
                std::fprintf(stderr, "[heartbeat] eeCycle=%llu (%.1f s emulated) presented=%llu present-stage=%u running=%d seq=%llu slice-end=%llu next-event=%llu\n",
                             static_cast<unsigned long long>(snap.eeCycle), snap.eeCycle / 294912000.0,
                             static_cast<unsigned long long>(g_ps2xPresentedFrames.load()), g_ps2xPresentStage.load(),
                             snap.runningThreadId, static_cast<unsigned long long>(snap.sequence),
                             static_cast<unsigned long long>(snap.sliceEndCycle),
                             static_cast<unsigned long long>(snap.nextEventCycle));
                std::fprintf(stderr, "[heartbeat] vsync=%llu pc=%08x ra=%08x threads=%zu dma=%llu gif=%llu gsw=%llu vif=%llu\n",
                             static_cast<unsigned long long>(runtime.eeScheduler().currentVSyncTick()),
                             runtime.m_debugPc.load(), runtime.m_debugRa.load(), snap.threads.size(),
                             static_cast<unsigned long long>(runtime.memory().dmaStartCount()),
                             static_cast<unsigned long long>(runtime.memory().gifCopyCount()),
                             static_cast<unsigned long long>(runtime.memory().gsWriteCount()),
                             static_cast<unsigned long long>(runtime.memory().vifWriteCount()));
                const GSRegisters &gs = runtime.memory().gs();
                std::fprintf(stderr, "  gs pmode=%llx smode2=%llx dispfb1=%llx display1=%llx dispfb2=%llx display2=%llx\n",
                             static_cast<unsigned long long>(gs.pmode), static_cast<unsigned long long>(gs.smode2),
                             static_cast<unsigned long long>(gs.dispfb1), static_cast<unsigned long long>(gs.display1),
                             static_cast<unsigned long long>(gs.dispfb2), static_cast<unsigned long long>(gs.display2));
                for (const EeThreadSnapshot &t : snap.threads)
                {
                    std::fprintf(stderr, "  thread %d entry=%08x pc=%08x ra=%08x prio=%d status=%d wait=%d/%d\n",
                                 t.id, t.entry, t.pc, t.ra, t.currentPriority, static_cast<int>(t.status),
                                 static_cast<int>(t.waitReason), t.waitId);
                }
                // DDS2_WATCH=addr:len[,addr:len...] dumps guest RAM bytes (hex).
                if (const char *watch = std::getenv("DDS2_WATCH"))
                {
                    const uint8_t *rdram = runtime.memory().getRDRAM();
                    std::string spec = watch;
                    size_t pos = 0;
                    while (rdram && pos < spec.size())
                    {
                        const size_t comma = spec.find(',', pos);
                        const std::string item = spec.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                        pos = comma == std::string::npos ? spec.size() : comma + 1;
                        const size_t colon = item.find(':');
                        const uint32_t addr = static_cast<uint32_t>(std::strtoul(item.c_str(), nullptr, 16)) & 0x1FFFFFFu;
                        const uint32_t len = colon == std::string::npos ? 4u : static_cast<uint32_t>(std::strtoul(item.c_str() + colon + 1, nullptr, 0));
                        std::fprintf(stderr, "  mem %08x:", addr);
                        for (uint32_t i = 0; i < len && addr + i < PS2_RAM_SIZE; ++i)
                            std::fprintf(stderr, " %02x", rdram[addr + i]);
                        std::fprintf(stderr, "\n");
                    }
                }
                // DDS2_WALK=<head pointer address>: dumps the circular block list
                // used by the packet allocator at 0x3292A8 (node: +0 prev,
                // +4 next, +8 address, +C state).
                if (const char *walk = std::getenv("DDS2_WALK"))
                {
                    const uint8_t *rdram = runtime.memory().getRDRAM();
                    auto read32 = [rdram](uint32_t address)
                    {
                        uint32_t value = 0;
                        std::memcpy(&value, rdram + (address & 0x1FFFFFFu), 4);
                        return value;
                    };
                    const uint32_t head = read32(static_cast<uint32_t>(std::strtoul(walk, nullptr, 16)));
                    uint32_t node = head;
                    for (int i = 0; i < 48 && node != 0u; ++i)
                    {
                        const uint32_t next = read32(node + 4u);
                        std::fprintf(stderr, "  node %08x addr=%08x state=%u next=%08x size=%d\n", node, read32(node + 8u),
                                     read32(node + 12u) & 0xFFFFu, next, static_cast<int>(read32(next + 8u) - read32(node + 8u)));
                        node = next;
                        if (node == head)
                            break;
                    }
                }
            } })
            .detach();
    }
}

int main(int argc, char *argv[])
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);
    std::cout << std::unitbuf;
    static std::atomic<bool> heartbeatStop{false};
    try
    {
        const std::filesystem::path elf = findGameElf(argc, argv);
        std::cout << "[dds2] booting " << elf.string() << std::endl;

        PS2Runtime runtime;
        if (!runtime.initialize("Shin Megami Tensei: Digital Devil Saga 2"))
        {
            std::cerr << "[dds2] failed to initialize runtime" << std::endl;
            return 1;
        }
        if (!runtime.loadELF(elf.string()))
        {
            std::cerr << "[dds2] failed to load " << elf.string() << std::endl;
            return 1;
        }
        vu1rec::install();
        startHeartbeat(runtime, heartbeatStop);
        dds2_profiler::start(heartbeatStop);
        dds2_debug::installWriteWatch(runtime.memory().getRDRAM());
        dds2_platform::raiseTimerResolution();
        runtime.run();
        dds2_platform::restoreTimerResolution();
        heartbeatStop = true;
        std::cout.flush();
        std::cerr.flush();
        std::_Exit(0);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[dds2] fatal: " << e.what() << std::endl;
    }
    std::cout.flush();
    std::cerr.flush();
    std::_Exit(1);
}
