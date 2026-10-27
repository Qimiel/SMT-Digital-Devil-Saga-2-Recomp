// Debugging aids, all inactive unless their environment variable is set.
//
// DDS2_WATCH_WRITE=<guest address>: reports every host instruction that writes
// the 16 guest bytes at that address. The host page is made read-only; a write
// fault is logged when it hits the watched bytes, then the page is unprotected
// for one single-stepped instruction and protected again. Map the logged
// offsets (from the executable base) to functions with nm.
//
// DDS2_CHECK_HEAP: reports allocations from the game's main heap that can
// never fit (the allocator then spins forever).
//
// DDS2_CHECK_SMALL: tracks the game's small-object allocator exactly and
// reports the first chunk handed out twice or freed while not allocated.
#include "dds2_debug.h"

#include "ps2_runtime.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <unordered_map>

extern std::atomic<uint64_t> g_ps2xVBlankCount;

namespace dds2_debug
{
    namespace
    {
        uint8_t *g_watchBegin = nullptr;
        uint8_t *g_watchPage = nullptr;
        uintptr_t g_imageBase = 0;
        thread_local bool t_reprotect = false;
        thread_local bool t_watchedWrite = false;

        void protectPage(bool readOnly)
        {
            DWORD old = 0;
            VirtualProtect(g_watchPage, 4096, readOnly ? PAGE_READONLY : PAGE_READWRITE, &old);
        }

        LONG CALLBACK handler(EXCEPTION_POINTERS *info)
        {
            const DWORD code = info->ExceptionRecord->ExceptionCode;
            if (code == EXCEPTION_ACCESS_VIOLATION && info->ExceptionRecord->NumberParameters >= 2 &&
                info->ExceptionRecord->ExceptionInformation[0] == 1)
            {
                auto *address = reinterpret_cast<uint8_t *>(info->ExceptionRecord->ExceptionInformation[1]);
                if (address < g_watchPage || address >= g_watchPage + 4096)
                    return EXCEPTION_CONTINUE_SEARCH;
                t_watchedWrite = address + 16 > g_watchBegin && address < g_watchBegin + 16;
                if (t_watchedWrite)
                    std::fprintf(stderr, "[watch] write at +%llx by rip=exe+%llx (thread %lu)\n",
                                 static_cast<unsigned long long>(address - g_watchBegin),
                                 static_cast<unsigned long long>(info->ContextRecord->Rip - g_imageBase),
                                 GetCurrentThreadId());
                protectPage(false);
                t_reprotect = true;
                info->ContextRecord->EFlags |= 0x100; // trap after this instruction
                return EXCEPTION_CONTINUE_EXECUTION;
            }
            if (code == EXCEPTION_SINGLE_STEP && t_reprotect)
            {
                t_reprotect = false;
                if (t_watchedWrite)
                {
                    uint32_t words[4];
                    std::memcpy(words, g_watchBegin, sizeof(words));
                    std::fprintf(stderr, "[watch]   now %08x %08x %08x %08x\n", words[0], words[1], words[2], words[3]);
                }
                protectPage(true);
                return EXCEPTION_CONTINUE_EXECUTION;
            }
            return EXCEPTION_CONTINUE_SEARCH;
        }
    }

    void installWriteWatch(uint8_t *rdram)
    {
        const char *env = std::getenv("DDS2_WATCH_WRITE");
        if (!env || !rdram)
            return;
        const uint32_t guest = static_cast<uint32_t>(std::strtoul(env, nullptr, 16)) & 0x1FFFFF0u;
        g_watchBegin = rdram + guest;
        g_watchPage = reinterpret_cast<uint8_t *>(reinterpret_cast<uintptr_t>(g_watchBegin) & ~uintptr_t{4095});
        g_imageBase = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
        AddVectoredExceptionHandler(1, handler);
        protectPage(true);
        std::fprintf(stderr, "[watch] watching guest %08x\n", guest);
    }

    namespace
    {
        uint8_t *g_rdram = nullptr;

        uint32_t read32(uint32_t address)
        {
            uint32_t value = 0;
            std::memcpy(&value, g_rdram + (address & PS2_RAM_MASK), sizeof(value));
            return value;
        }

        // 0x3292A8 allocates from the main heap: a circular node list at
        // *0x45F0FC (+4 next, +8 address, +C state 0 free / 1 used / 2 bound).
        PS2Runtime::RecompiledFunction g_heapAlloc = nullptr;

        void checkedHeapAlloc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            const uint32_t request = (getRegU32(ctx, 4) + 0x7Fu) & ~0x7Fu;
            const uint32_t head = read32(0x0045F0FCu);
            uint32_t largest = 0, freeTotal = 0, nodes = 0;
            for (uint32_t node = head; node != 0u && nodes < 100000u;)
            {
                const uint32_t next = read32(node + 4u);
                if ((read32(node + 12u) & 0xFFFFu) == 0u)
                {
                    const uint32_t size = read32(next + 8u) - read32(node + 8u);
                    largest = std::max(largest, size);
                    freeTotal += size;
                }
                ++nodes;
                node = next;
                if (node == head)
                    break;
            }
            static uint32_t reported = 0;
            if (request > largest && reported++ < 8u)
                std::fprintf(stderr, "[dds2-heap] %u-byte allocation from ra=%08x cannot fit: largest free %u, total free %u, %u nodes\n",
                             request, getRegU32(ctx, 31), largest, freeTotal, nodes);
            g_heapAlloc(rdram, ctx, runtime);
        }

        // 0x328D68 / 0x328E48 allocate and free small-object chunks.
        PS2Runtime::RecompiledFunction g_smallAlloc = nullptr;
        PS2Runtime::RecompiledFunction g_smallFree = nullptr;
        std::unordered_map<uint32_t, uint32_t> g_liveChunks;  // chunk -> allocating ra
        std::unordered_map<uint32_t, uint32_t> g_freedChunks; // chunk -> last freeing ra
        uint32_t g_smallReports = 0;

        void checkedSmallAlloc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            const uint32_t ra = getRegU32(ctx, 31);
            const uint32_t size = getRegU32(ctx, 4);
            g_smallAlloc(rdram, ctx, runtime);
            if (ctx->pc != ra)
                return; // yielded at a checkpoint: the result is not known here
            const uint32_t chunk = getRegU32(ctx, 2);
            const auto live = g_liveChunks.find(chunk);
            if (live != g_liveChunks.end() && g_smallReports++ < 16u)
                std::fprintf(stderr, "[dds2-small] chunk %08x (%u bytes) handed out again to ra=%08x; still owned by ra=%08x\n",
                             chunk, size, ra, live->second);
            g_liveChunks[chunk] = ra;
            g_freedChunks.erase(chunk);
        }

        void checkedSmallFree(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            const uint32_t chunk = getRegU32(ctx, 4);
            if (chunk != 0u)
            {
                const uint32_t ra = getRegU32(ctx, 31);
                if (g_liveChunks.erase(chunk) == 0u && g_smallReports++ < 16u)
                {
                    const auto freed = g_freedChunks.find(chunk);
                    std::fprintf(stderr, "[dds2-small] free of chunk %08x by ra=%08x that is not allocated (last freed by ra=%08x)\n",
                                 chunk, ra, freed != g_freedChunks.end() ? freed->second : 0u);
                }
                g_freedChunks[chunk] = ra;
            }
            g_smallFree(rdram, ctx, runtime);
        }
    }

    namespace
    {
        // DDS2_TRACE_FN=addr[:count],...: entries to these functions (count
        // each, default 4) from DDS2_TRACE_FN_AT emulated seconds on.
        std::unordered_map<uint32_t, std::pair<PS2Runtime::RecompiledFunction, uint32_t>> g_tracedFns;
        double g_traceFrom = 0.0;

        void tracedFunction(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            const uint32_t address = ctx->pc;
            auto &entry = g_tracedFns.at(address);
            const bool traced = entry.second != 0u && g_ps2xVBlankCount.load() / 59.94 >= g_traceFrom;
            const uint32_t returnAddress = getRegU32(ctx, 31);
            if (traced)
            {
                --entry.second;
                const uint32_t a0 = getRegU32(ctx, 4);
                std::fprintf(stderr, "[trace-fn] %08x ra=%08x a0=%08x a1=%08x a2=%08x a3=%08x\n", ctx->pc,
                             getRegU32(ctx, 31), a0, getRegU32(ctx, 5), getRegU32(ctx, 6), getRegU32(ctx, 7));
                for (int r = 1; r < 32; ++r)
                {
                    float v[4];
                    std::memcpy(v, &ctx->vu0_vf[r], sizeof(v));
                    if (v[0] != 0.0f || v[1] != 0.0f || v[2] != 0.0f || v[3] != 0.0f)
                        std::fprintf(stderr, "    vf%-2d %12g %12g %12g %12g\n", r, v[0], v[1], v[2], v[3]);
                }
                // DDS2_TRACE_FN_OFFSET=<hex>: dump from $a0 + offset instead.
                static const uint32_t dumpOffset = std::getenv("DDS2_TRACE_FN_OFFSET")
                                                       ? static_cast<uint32_t>(std::strtoul(std::getenv("DDS2_TRACE_FN_OFFSET"), nullptr, 16))
                                                       : 0u;
                static const uint32_t dumpQwords = std::getenv("DDS2_TRACE_FN_QWORDS")
                                                       ? static_cast<uint32_t>(std::strtoul(std::getenv("DDS2_TRACE_FN_QWORDS"), nullptr, 0))
                                                       : 16u;
                for (uint32_t q = 0; q < dumpQwords && a0 >= 0x100000u && a0 < 0x2000000u; ++q)
                {
                    const uint32_t address = a0 + dumpOffset + q * 16u;
                    float v[4];
                    std::memcpy(v, rdram + (address & PS2_RAM_MASK), sizeof(v));
                    std::fprintf(stderr, "    a0+%02x %12g %12g %12g %12g  (%08x %08x %08x %08x)\n", dumpOffset + q * 16u, v[0],
                                 v[1], v[2], v[3], read32(address), read32(address + 4u), read32(address + 8u),
                                 read32(address + 12u));
                }
            }
            entry.first(rdram, ctx, runtime);
            if (traced && ctx->pc == returnAddress)
                std::fprintf(stderr, "[trace-fn] %08x returned v0=%08x\n", address, getRegU32(ctx, 2));
        }
    }

    namespace
    {
        // DDS2_CHECK_HIT: HitPositionInside2 (0x139400, field collision zone
        // test) writes a clearance to *$a0; report calls whose clearance is
        // not finite or huge, with their arguments and the zone's planes.
        PS2Runtime::RecompiledFunction g_hitPositionInside2 = nullptr;

        void checkedHitPositionInside2(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            const uint32_t out = getRegU32(ctx, 4);
            const uint32_t axis = getRegU32(ctx, 5);
            const uint32_t count = getRegU32(ctx, 6);
            const uint32_t pos = getRegU32(ctx, 7);
            const uint32_t zone = getRegU32(ctx, 8);
            const float margin = ctx->f[12];
            g_hitPositionInside2(rdram, ctx, runtime);
            float clearance = 0.0f;
            std::memcpy(&clearance, rdram + (out & PS2_RAM_MASK), 4);
            static uint32_t reports = 0;
            if ((!std::isfinite(clearance) || std::fabs(clearance) > 1e5f) && reports++ < 10u)
            {
                float p[3], z[0x58];
                std::memcpy(p, rdram + (pos & PS2_RAM_MASK), sizeof(p));
                std::memcpy(z, rdram + ((zone + 4u) & PS2_RAM_MASK), sizeof(z));
                std::fprintf(stderr,
                             "[hit] HitPositionInside2 out=%g (%08x) v0=%d axis=%u count=%u margin=%g pos=(%g %g %g) zone=%08x\n",
                             clearance, read32(out), static_cast<int32_t>(getRegU32(ctx, 2)), axis, count, margin, p[0], p[1], p[2],
                             zone);
                for (int i = 0; i < 4; ++i)
                {
                    const float *plane = z + (0x18 - 4) / 4 + i * 4;
                    std::fprintf(stderr, "    plane%d=(%g %g %g) limit=%g bound=%g\n", i, plane[0], plane[1], plane[2],
                                 z[(0x58 - 4) / 4 + i], z[(0x68 - 4) / 4 + i]);
                }
            }
        }

        // DDS2_CHECK_TM: sdf3DPrimSetTmPacket (0x33AEA8) stores the world
        // matrix held in vf28-vf31 into a node's VU1 transform block; report
        // matrices with a degenerate or non-finite basis.
        PS2Runtime::RecompiledFunction g_setTmPacket = nullptr;
        uint32_t g_tmReports = 0;
        uint64_t g_tmChecked = 0, g_tmBad = 0;

        void checkedSetTmPacket(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            ++g_tmChecked;
            float m[4][4];
            std::memcpy(m, &ctx->vu0_vf[28], sizeof(m));
            bool bad = false;
            for (int r = 0; r < 3; ++r)
            {
                const float n = m[r][0] * m[r][0] + m[r][1] * m[r][1] + m[r][2] * m[r][2];
                if (!(n > 1e-12f) || !std::isfinite(n))
                    bad = true;
            }
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    if (!std::isfinite(m[r][c]))
                        bad = true;
            if (bad)
            {
                ++g_tmBad;
                if (g_tmReports++ < 24u)
                {
                    // Walk one frame up: sdfModelEvalNode keeps the node in $s1.
                    std::fprintf(stderr, "[dds2-tm] degenerate matrix for block %08x (ra=%08x node=%08x) t=%.1fs\n",
                                 getRegU32(ctx, 4), getRegU32(ctx, 31), getRegU32(ctx, 17),
                                 g_ps2xVBlankCount.load() / 59.94);
                    for (int r = 0; r < 4; ++r)
                        std::fprintf(stderr, "    %12g %12g %12g %12g\n", m[r][0], m[r][1], m[r][2], m[r][3]);
                }
            }
            if ((g_tmChecked & 0xFFFFFu) == 0u)
                std::fprintf(stderr, "[dds2-tm] %llu matrices, %llu degenerate\n",
                             static_cast<unsigned long long>(g_tmChecked), static_cast<unsigned long long>(g_tmBad));
            g_setTmPacket(rdram, ctx, runtime);
        }
    }

    namespace
    {
        // DDS2_TRACE_MES: itfMesAdvanceEntryText (0x1A6350) per message
        // object: logs its state whenever it changes, and every call made
        // while confirm, cancel or triangle (dds3PadButtonState bank 1) is set.
        PS2Runtime::RecompiledFunction g_mesAdvance = nullptr;

        void tracedMesAdvance(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            static std::unordered_map<uint32_t, uint64_t> lastState;
            const uint32_t mes = getRegU32(ctx, 4);
            const uint32_t flags = read32(mes);
            const uint8_t textState = rdram[(mes + 0x34u) & PS2_RAM_MASK];
            const uint8_t mask = rdram[(mes + 0x35u) & PS2_RAM_MASK];
            const uint32_t items = read32(mes + 0x3Cu); // itemIndex | tableCount << 16
            const uint32_t pad = read32(0x0037F530u);   // bank 1 port 0, buttons 0-3
            const uint64_t state = (static_cast<uint64_t>(flags) << 32) ^ (static_cast<uint64_t>(textState) << 24) ^
                                   (static_cast<uint64_t>(mask) << 16) ^ items;
            auto it = lastState.find(mes);
            const bool changed = it == lastState.end() || it->second != state;
            if (changed || (pad & 0xFFFFFF00u) != 0u)
                std::fprintf(stderr, "[dds2-mes] %.2fs mes=%08x ra=%08x flags=%08x textState=%d mask=%02x item=%d/%d pad=%08x\n",
                             g_ps2xVBlankCount.load() / 59.94, mes, getRegU32(ctx, 31), flags,
                             static_cast<int8_t>(textState), mask, static_cast<int16_t>(items & 0xFFFFu),
                             static_cast<int16_t>(items >> 16), pad);
            lastState[mes] = state;
            g_mesAdvance(rdram, ctx, runtime);
        }
    }

    void installFunctionTrace(PS2Runtime &runtime)
    {
        g_rdram = runtime.memory().getRDRAM();
        if (std::getenv("DDS2_CHECK_TM"))
        {
            g_setTmPacket = runtime.lookupFunction(0x0033AEA8u);
            runtime.replaceFunction(0x0033AEA8u, &checkedSetTmPacket);
        }
        if (std::getenv("DDS2_CHECK_HIT"))
        {
            g_hitPositionInside2 = runtime.lookupFunction(0x00139400u);
            runtime.replaceFunction(0x00139400u, &checkedHitPositionInside2);
        }
        if (std::getenv("DDS2_TRACE_MES"))
        {
            g_mesAdvance = runtime.lookupFunction(0x001A6350u);
            runtime.replaceFunction(0x001A6350u, &tracedMesAdvance);
        }
        // DDS2_TRACE_PLAYER=<emulated s>: from then on, the field area and the
        // player position (fldGlobalBase) four times a second.
        if (const char *playerEnv = std::getenv("DDS2_TRACE_PLAYER"))
        {
            const double from = std::atof(playerEnv);
            const uint8_t *rdram = g_rdram;
            CreateThread(
                nullptr, 0,
                [](LPVOID parameter) -> DWORD
                {
                    const auto *args = static_cast<const std::pair<const uint8_t *, double> *>(parameter);
                    float last[3] = {};
                    for (;;)
                    {
                        Sleep(250);
                        const double now = g_ps2xVBlankCount.load() / 59.94;
                        if (now < args->second)
                            continue;
                        constexpr uint32_t kBase = 0x00389770u; // fldGlobalBase
                        int32_t area = 0;
                        float position[3];
                        std::memcpy(&area, args->first + kBase + 0x10u, 4);
                        std::memcpy(position, args->first + kBase + 0x14Cu, 12);
                        if (std::memcmp(position, last, sizeof(last)) != 0)
                            std::fprintf(stderr, "[player] %.2fs area=%d pos=(%.1f, %.1f, %.1f)\n", now, area, position[0],
                                         position[1], position[2]);
                        std::memcpy(last, position, sizeof(last));
                    }
                },
                new std::pair<const uint8_t *, double>(rdram, from), 0, nullptr);
        }
        // DDS2_CHECK_COLLISION=<emulated s>: then lists field collision records
        // (fldMatterTable) whose normals are not unit length or that hold
        // non-finite or huge values.
        if (const char *collisionEnv = std::getenv("DDS2_CHECK_COLLISION"))
        {
            CreateThread(
                nullptr, 0,
                [](LPVOID parameter) -> DWORD
                {
                    const auto *args = static_cast<const std::pair<const uint8_t *, double> *>(parameter);
                    while (g_ps2xVBlankCount.load() / 59.94 < args->second)
                        Sleep(100);
                    const uint8_t *rdram = args->first;
                    uint32_t table = 0, count = 0;
                    std::memcpy(&table, rdram + 0x00436180u, 4);
                    std::memcpy(&count, rdram + 0x00436184u, 4);
                    std::fprintf(stderr, "[collision] %u records at %08x\n", count, table);
                    auto bad = [](float v) { return !std::isfinite(v) || std::fabs(v) > 1e6f; };
                    uint32_t reported = 0;
                    for (uint32_t i = 0; i < count && i < 4096u && table; ++i)
                    {
                        const uint8_t *r = rdram + ((table + i * 0xE4u) & 0x1FFFFFFu);
                        int16_t mode, planes;
                        float values[0x58];
                        std::memcpy(&mode, r, 2);
                        std::memcpy(&planes, r + 2, 2);
                        std::memcpy(values, r + 4, 0xB4);
                        const float *normal = values;
                        const float normalLength = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
                        bool anomaly = std::fabs(normalLength - 1.0f) > 0.01f;
                        // Records of moving objects are tested relative to the
                        // object's transform (worldNode->inner, position at +0x40).
                        uint32_t worldNode = 0;
                        std::memcpy(&worldNode, r + 0xD0u, 4);
                        if (worldNode)
                        {
                            uint32_t inner = 0;
                            std::memcpy(&inner, rdram + ((worldNode + 0x1Cu) & 0x1FFFFFFu), 4);
                            float transform[3] = {};
                            if (inner)
                                std::memcpy(transform, rdram + ((inner + 0x40u) & 0x1FFFFFFu), 12);
                            std::fprintf(stderr, "[collision] #%u moving: node=%08x inner=%08x position=(%g %g %g) normal=(%g %g %g)\n",
                                         i, worldNode, inner, transform[0], transform[1], transform[2], normal[0], normal[1],
                                         normal[2]);
                        }
                        for (int v = 0; v < 0xB4 / 4; ++v)
                            anomaly |= bad(values[v]);
                        for (int p = 0; p < planes && p < 4; ++p)
                        {
                            const float *plane = values + (0x18 - 4) / 4 + p * 4;
                            const float length = std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
                            anomaly |= std::fabs(length - 1.0f) > 0.01f;
                        }
                        if (anomaly && reported++ < 40)
                        {
                            std::fprintf(stderr, "[collision] #%u mode=%d count=%d normal=(%g %g %g) |n|=%g d=%g\n", i, mode, planes,
                                         normal[0], normal[1], normal[2], normalLength, values[4]);
                            for (int p = 0; p < 4; ++p)
                            {
                                const float *plane = values + (0x18 - 4) / 4 + p * 4;
                                const float *vertex = values + (0x78 - 4) / 4 + p * 4;
                                std::fprintf(stderr, "    plane%d=(%g %g %g) limit=%g  v%d=(%g %g %g)\n", p, plane[0], plane[1],
                                             plane[2], values[(0x58 - 4) / 4 + p], p, vertex[0], vertex[1], vertex[2]);
                            }
                        }
                    }
                    std::fprintf(stderr, "[collision] %u anomalous records\n", reported);
                    return 0;
                },
                new std::pair<const uint8_t *, double>(g_rdram, std::atof(collisionEnv)), 0, nullptr);
        }
        const char *env = std::getenv("DDS2_TRACE_FN");
        if (!env)
            return;
        if (const char *at = std::getenv("DDS2_TRACE_FN_AT"))
            g_traceFrom = std::atof(at);
        for (const char *p = env; *p;)
        {
            char *end = nullptr;
            const uint32_t address = static_cast<uint32_t>(std::strtoul(p, &end, 16));
            uint32_t count = 4;
            if (*end == ':')
                count = static_cast<uint32_t>(std::strtoul(end + 1, &end, 10));
            g_tracedFns[address] = {runtime.lookupFunction(address), count};
            runtime.replaceFunction(address, &tracedFunction);
            p = *end == ',' ? end + 1 : end;
        }
    }

    void installHeapChecks(PS2Runtime &runtime)
    {
        g_rdram = runtime.memory().getRDRAM();
        if (std::getenv("DDS2_CHECK_HEAP"))
        {
            g_heapAlloc = runtime.lookupFunction(0x003292A8u);
            runtime.replaceFunction(0x003292A8u, &checkedHeapAlloc);
        }
        if (std::getenv("DDS2_CHECK_SMALL"))
        {
            g_smallAlloc = runtime.lookupFunction(0x00328D68u);
            runtime.replaceFunction(0x00328D68u, &checkedSmallAlloc);
            g_smallFree = runtime.lookupFunction(0x00328E48u);
            runtime.replaceFunction(0x00328E48u, &checkedSmallFree);
        }
    }
}
