#include "vu1rec/vu1rec.h"
#include "vu1rec/vu1rec_rt.h"

#include "runtime/ps2_memory.h"
#include "runtime/ps2_vu1.h"

#include <xmmintrin.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

// VBlanks delivered so far, defined in EeScheduler.cpp.
extern std::atomic<uint64_t> g_ps2xVBlankCount;

namespace vu1rec
{
    namespace
    {
        // Entry PC and ITOP of the program now running (kick tracing).
        thread_local uint32_t t_programPc = 0;
        thread_local uint32_t t_programItop = 0;

        // Same hash as the harvester in ps2_vu1_core.cpp (registry keys).
        uint64_t hashCodeImage(const uint8_t *data, uint32_t size)
        {
            uint64_t h = 0xCBF29CE484222325ull;
            for (uint32_t i = 0; i + 8u <= size; i += 8u)
            {
                uint64_t word = 0;
                std::memcpy(&word, data + i, sizeof(word));
                h = (h ^ word) * 0x100000001B3ull;
                h ^= h >> 29;
            }
            return h;
        }

        // The translation of the code image now in VU1 (or VU0) micro memory,
        // looked up again whenever that memory changes.
        ProgramFn lookup(const uint8_t *code, const PS2Memory *memory, bool vu0, uint64_t &hash)
        {
            struct Cache
            {
                uint64_t generation = UINT64_MAX;
                uint64_t hash = 0;
                ProgramFn fn = nullptr;
            };
            static Cache caches[2];
            Cache &cache = caches[vu0 ? 1 : 0];
            const uint64_t generation = !memory ? 0u : vu0 ? memory->getVU0CodeGeneration() : memory->getVU1CodeGeneration();
            if (generation != cache.generation)
            {
                cache.generation = generation;
                cache.hash = hashCodeImage(code, vu0 ? 0x1000u : 0x4000u);
                cache.fn = nullptr;
                for (const ProgramEntry *entry = vu0 ? kVu0Programs : kPrograms; entry->fn; ++entry)
                {
                    if (entry->hash == cache.hash)
                    {
                        cache.fn = entry->fn;
                        break;
                    }
                }
                static std::unordered_map<uint64_t, bool> reported;
                if (!cache.fn && !reported[cache.hash])
                {
                    reported[cache.hash] = true;
                    std::fprintf(stderr, "[vu1rec] no translation for %s microcode image %016llx; interpreting it "
                                         "(harvest it with PS2X_%s_HARVEST and run tools/vu1_translate.py%s)\n",
                                 vu0 ? "VU0" : "VU1", static_cast<unsigned long long>(cache.hash), vu0 ? "VU0" : "VU1",
                                 vu0 ? " --vu0" : "");
                }
            }
            hash = cache.hash;
            return cache.fn;
        }

        // PS2X_VU0_HARVEST=<dir>: saves VU0 code images without a translation
        // as <hash>.bin, and their entry points in entries.txt (the VU1
        // harvester lives in the interpreter, ps2_vu1_core.cpp).
        void harvestVu0(const uint8_t *code, uint64_t hash, uint32_t pc)
        {
            static const char *dir = std::getenv("PS2X_VU0_HARVEST");
            if (!dir)
                return;
            static std::unordered_map<uint64_t, std::vector<uint32_t>> seen;
            auto &entries = seen[hash];
            if (std::find(entries.begin(), entries.end(), pc) != entries.end())
                return;
            char name[64];
            std::snprintf(name, sizeof(name), "%016llx", static_cast<unsigned long long>(hash));
            if (entries.empty())
            {
                if (FILE *f = std::fopen((std::string(dir) + "/" + name + ".bin").c_str(), "wb"))
                {
                    std::fwrite(code, 1, 0x1000u, f);
                    std::fclose(f);
                }
            }
            entries.push_back(pc);
            if (FILE *f = std::fopen((std::string(dir) + "/entries.txt").c_str(), "a"))
            {
                std::fprintf(f, "%s %04x\n", name, pc);
                std::fclose(f);
            }
        }

        std::vector<std::vector<uint8_t>> *g_capturedKicks = nullptr;

        void runNative(ProgramFn fn, VU1State &s, uint8_t *data, PS2Memory *memory, bool capture)
        {
            Ctx c;
            c.s = &s;
            c.mem = data;
            c.memory = memory;
            c.cyc = s.cycles;
            c.workingClip = s.clip;
            c.captureKicks = capture;
            // VU float behaviour: round toward zero, denormals read and
            // written as zero.
            const unsigned csr = _mm_getcsr();
            _mm_setcsr((csr & ~0x6000u) | 0x6000u | 0x8040u);
            fn(c);
            _mm_setcsr(csr);
            if (c.trapped)
            {
                static uint32_t reported = 0;
                if (reported++ < 20u)
                    std::fprintf(stderr, "[vu1rec] translated program hit an untranslatable instruction at %04x\n", c.trapPc);
            }
        }

        // PS2X_VU1_VERIFY: the native result, compared after the interpreter ran.
        struct Verify
        {
            bool pending = false;
            uint32_t entryPc = 0;
            VU1State native{};
            std::array<uint8_t, 0x4000> nativeMemory{};
            std::vector<std::vector<uint8_t>> kicks;
            uint64_t programs = 0;
            uint64_t mismatches = 0;
        } g_verify;

        bool closeFloat(uint32_t a, uint32_t b)
        {
            if (a == b)
                return true;
            float fa, fb;
            std::memcpy(&fa, &a, 4);
            std::memcpy(&fb, &b, 4);
            if (!std::isfinite(fa) || !std::isfinite(fb))
                return false;
            const float scale = std::max(std::fabs(fa), std::fabs(fb));
            return std::fabs(fa - fb) <= scale * 1e-5f + 1e-30f;
        }

        void compare(const VU1State &interp, const uint8_t *memory, uint32_t memorySize)
        {
            const VU1State &native = g_verify.native;
            char report[1024];
            int length = 0;
            auto note = [&](const char *format, auto... args)
            {
                if (length < 900)
                    length += std::snprintf(report + length, sizeof(report) - length, format, args...);
            };
            for (int r = 1; r < 32; ++r)
                for (int l = 0; l < 4; ++l)
                {
                    uint32_t a, b;
                    std::memcpy(&a, &interp.vf[r][l], 4);
                    std::memcpy(&b, &native.vf[r][l], 4);
                    if (!closeFloat(a, b))
                        note(" vf%d.%c=%08x/%08x", r, "xyzw"[l], a, b);
                }
            for (int r = 1; r < 16; ++r)
                if ((interp.vi[r] & 0xFFFF) != (native.vi[r] & 0xFFFF))
                    note(" vi%d=%x/%x", r, interp.vi[r] & 0xFFFF, native.vi[r] & 0xFFFF);
            for (int l = 0; l < 4; ++l)
            {
                uint32_t a, b;
                std::memcpy(&a, &interp.acc[l], 4);
                std::memcpy(&b, &native.acc[l], 4);
                if (!closeFloat(a, b))
                    note(" acc.%c=%08x/%08x", "xyzw"[l], a, b);
            }
            auto scalar = [&](const char *name, float x, float y)
            {
                uint32_t a, b;
                std::memcpy(&a, &x, 4);
                std::memcpy(&b, &y, 4);
                if (!closeFloat(a, b))
                    note(" %s=%08x/%08x", name, a, b);
            };
            scalar("q", interp.q, native.q);
            scalar("p", interp.p, native.p);
            scalar("i", interp.i, native.i);
            if ((interp.clip & 0xFFFFFFu) != (native.clip & 0xFFFFFFu))
                note(" clip=%x/%x", interp.clip, native.clip);
            if (interp.pc != native.pc)
                note(" pc=%x/%x", interp.pc, native.pc);
            uint32_t differing = 0, first = 0xFFFFFFFFu;
            for (uint32_t offset = 0; offset < memorySize; offset += 4u)
            {
                uint32_t a, b;
                std::memcpy(&a, memory + offset, 4);
                std::memcpy(&b, g_verify.nativeMemory.data() + offset, 4);
                if (!closeFloat(a, b))
                {
                    if (first == 0xFFFFFFFFu)
                        first = offset;
                    ++differing;
                }
            }
            if (differing)
            {
                uint32_t a, b;
                std::memcpy(&a, memory + first, 4);
                std::memcpy(&b, g_verify.nativeMemory.data() + first, 4);
                note(" mem: %u words differ, first at %04x (%08x/%08x)", differing, first, a, b);
            }
            ++g_verify.programs;
            if (length > 0)
            {
                ++g_verify.mismatches;
                if (g_verify.mismatches <= 40u)
                    std::fprintf(stderr, "[vu1rec] verify: program at %04x differs (interpreter/native):%s\n",
                                 g_verify.entryPc, report);
            }
            if ((g_verify.programs % 20000u) == 0u)
                std::fprintf(stderr, "[vu1rec] verify: %llu programs compared, %llu differed\n",
                             static_cast<unsigned long long>(g_verify.programs),
                             static_cast<unsigned long long>(g_verify.mismatches));
        }

        bool nativeHook(VU1Interpreter &vu, uint8_t *code, uint8_t *data, PS2Memory *memory, bool after)
        {
            static const bool enabled = []()
            {
                const char *env = std::getenv("PS2X_VU1_NATIVE");
                return !env || std::atoi(env) != 0;
            }();
            static const bool verify = std::getenv("PS2X_VU1_VERIFY") != nullptr;
            static const bool stats = std::getenv("PS2X_VU_STATS") != nullptr;
            if (!enabled)
                return false;
            // VU0 micro mode (VCALLMS, VIF0 MSCAL) runs through the same
            // interpreter class; its micro memory tells the units apart.
            const bool vu0 = memory && code == memory->getVU0Code();
            const uint32_t dataSize = vu0 ? 0x1000u : 0x4000u;
            if (after)
            {
                if (g_verify.pending)
                {
                    g_verify.pending = false;
                    compare(vu.state(), data, dataSize);
                }
                return false;
            }
            VU1State &s = vu.state();
            uint64_t hash = 0;
            const ProgramFn fn = lookup(code, memory, vu0, hash);
            if (!fn)
            {
                if (vu0)
                    harvestVu0(code, hash, s.pc);
                return false;
            }
            if (vu0)
            {
                // No GIF path and no dumps: run it (or check it against the
                // interpreter) and return.
                if (verify)
                {
                    const VU1State before = s;
                    static std::array<uint8_t, 0x1000> beforeMemory;
                    std::memcpy(beforeMemory.data(), data, dataSize);
                    runNative(fn, s, data, memory, false);
                    g_verify.native = s;
                    std::memcpy(g_verify.nativeMemory.data(), data, dataSize);
                    g_verify.entryPc = before.pc;
                    g_verify.pending = true;
                    s = before;
                    std::memcpy(data, beforeMemory.data(), dataSize);
                    return false;
                }
                runNative(fn, s, data, memory, false);
                return true;
            }
            t_programPc = s.pc;
            t_programItop = s.itop;
            // PS2X_VU1_DUMP_AT=<emulated seconds>: saves VU1 data memory and
            // registers at the first program started after then.
            static const char *dumpAt = std::getenv("PS2X_VU1_DUMP_AT");
            static const char *dumpPc = std::getenv("PS2X_VU1_DUMP_PC");     // only programs entered here
            static const char *dumpItop = std::getenv("PS2X_VU1_DUMP_ITOP"); // ... with this ITOP
            static bool dumped = false;
            if (dumpAt && !dumped && g_ps2xVBlankCount.load() / 59.94 >= std::atof(dumpAt) &&
                (!dumpPc || s.pc == std::strtoul(dumpPc, nullptr, 16)) &&
                (!dumpItop || s.itop == std::strtoul(dumpItop, nullptr, 16)))
            {
                dumped = true;
                if (FILE *f = std::fopen("vu1_input.bin", "wb"))
                {
                    std::fwrite(data, 1, 0x4000u, f);
                    std::fclose(f);
                }
                std::fprintf(stderr, "[vu1rec] dumped vu1_input.bin at program %04x top=%x itop=%x vi:", s.pc, s.top, s.itop);
                for (int i = 0; i < 16; ++i)
                    std::fprintf(stderr, " %x", s.vi[i] & 0xFFFF);
                std::fprintf(stderr, "\n");
            }
            if (verify)
            {
                const VU1State before = s;
                static std::array<uint8_t, 0x4000> beforeMemory;
                std::memcpy(beforeMemory.data(), data, 0x4000u);
                g_verify.kicks.clear();
                g_capturedKicks = &g_verify.kicks;
                runNative(fn, s, data, memory, true);
                g_capturedKicks = nullptr;
                g_verify.native = s;
                std::memcpy(g_verify.nativeMemory.data(), data, 0x4000u);
                g_verify.entryPc = before.pc;
                g_verify.pending = true;
                s = before;
                std::memcpy(data, beforeMemory.data(), 0x4000u);
                return false; // the interpreter runs it for real
            }
            if (!stats)
            {
                runNative(fn, s, data, memory, false);
                return true;
            }
            static uint64_t programs = 0, cycles = 0;
            static double seconds = 0.0;
            static auto last = std::chrono::steady_clock::now();
            const uint64_t startCycles = s.cycles;
            const auto start = std::chrono::steady_clock::now();
            runNative(fn, s, data, memory, false);
            const auto now = std::chrono::steady_clock::now();
            ++programs;
            cycles += s.cycles - startCycles;
            seconds += std::chrono::duration<double>(now - start).count();
            if (now - last >= std::chrono::seconds(5))
            {
                std::fprintf(stderr, "[vu-stats] native VU1: %llu programs, %.1f M cycles in 5 s, %.2f s host time (%.1f ns/cycle)\n",
                             static_cast<unsigned long long>(programs), cycles / 1e6, seconds,
                             cycles ? seconds * 1e9 / static_cast<double>(cycles) : 0.0);
                programs = cycles = 0;
                seconds = 0.0;
                last = now;
            }
            return true;
        }

        // EFU approximations, as in the interpreter (ps2_vu1_lower.cpp).
        float eatan(float value)
        {
            constexpr float k[] = {0.999999344348907f, -0.333298563957214f, 0.199465364217758f,
                                   -0.13085337519646f, 0.096420042216778f, -0.055909886956215f,
                                   0.021861229091883f, -0.004054057877511f};
            const float squared = value * value;
            float polynomial = k[7];
            for (int i = 6; i >= 0; --i)
                polynomial = k[i] + squared * polynomial;
            return 0.785398185253143f + value * polynomial;
        }

        float esin(float value)
        {
            constexpr float k[] = {1.0f, -0.166666567325592f, 0.008333025500178f, -0.000198074136279f,
                                   0.000002601886990f};
            const float squared = value * value;
            float polynomial = k[4];
            for (int i = 3; i >= 0; --i)
                polynomial = k[i] + squared * polynomial;
            return value * polynomial;
        }

        float eexp(float value)
        {
            constexpr float k[] = {0.249998688697815f, 0.031257584691048f, 0.002591371303424f,
                                   0.000171562001924f, 0.000005430199963f, 0.000000690600018f};
            float polynomial = k[5];
            for (int i = 4; i >= 0; --i)
                polynomial = k[i] + value * polynomial;
            polynomial = 1.0f + value * polynomial;
            polynomial *= polynomial;
            polynomial *= polynomial;
            return polynomial != 0.0f ? 1.0f / polynomial : std::numeric_limits<float>::max();
        }

        // Earliest Q or P result still in flight (flags are applied lazily).
        void refreshNextDue(Ctx &c)
        {
            uint64_t next = UINT64_MAX;
            if (c.qPending)
                next = std::min(next, c.qReady);
            for (int i = 0; i < 2; ++i)
                if (c.pPending[i])
                    next = std::min(next, c.pReady[i]);
            c.nextDue = next;
        }
    }

    void commit(Ctx &c, uint64_t cyc)
    {
        VU1State &s = *c.s;
        if (c.qPending && c.qReady <= cyc)
        {
            s.q = c.qValue;
            c.qPending = false;
        }
        for (int i = 0; i < 2; ++i)
        {
            if (c.pPending[i] && c.pReady[i] <= cyc)
            {
                s.p = c.pValue[i];
                c.pPending[i] = false;
            }
        }
        refreshNextDue(c);
    }

    void finish(Ctx &c, uint64_t cyc)
    {
        // Like the interpreter's flushPipelines(): run on until every result
        // is in place.
        while (c.nextDue != UINT64_MAX)
        {
            wait(cyc, c.nextDue);
            commit(c, cyc);
        }
        if (c.flagHead != c.flagTail)
        {
            wait(cyc, c.flags[(c.flagTail - 1u) & 15u].ready);
            syncFlags(c, cyc);
        }
        wait(cyc, c.kickFree);
        c.cyc = cyc;
        c.s->cycles = cyc;
    }

    void trap(Ctx &c, uint64_t cyc, uint32_t pc)
    {
        c.trapped = true;
        c.trapPc = pc;
        c.s->pc = pc;
        finish(c, cyc);
    }

    // PS2X_VU1_KICK_TRACE=<emulated s>[:count]: decodes the GIF packets kicked
    // by programs entered at PS2X_VU1_KICK_PC (hex, optional) with ITOP
    // PS2X_VU1_KICK_ITOP (hex, optional), from then on.
    void traceKick(const uint8_t *packet, size_t size)
    {
        static const char *env = std::getenv("PS2X_VU1_KICK_TRACE");
        if (!env)
            return;
        static const double from = std::atof(env);
        static uint32_t remaining = std::strchr(env, ':') ? std::strtoul(std::strchr(env, ':') + 1, nullptr, 10) : 4u;
        static const char *pcEnv = std::getenv("PS2X_VU1_KICK_PC");
        static const char *itopEnv = std::getenv("PS2X_VU1_KICK_ITOP");
        if (remaining == 0u || g_ps2xVBlankCount.load() / 59.94 < from ||
            (pcEnv && t_programPc != std::strtoul(pcEnv, nullptr, 16)) ||
            (itopEnv && t_programItop != std::strtoul(itopEnv, nullptr, 16)))
            return;
        // PS2X_VU1_KICK_REGION=x0,y0,x1,y1 (GS primitive coordinates, pixels):
        // print only the XYZF2/XYZ2 vertices inside it, one line each.
        static const char *regionEnv = std::getenv("PS2X_VU1_KICK_REGION");
        if (regionEnv)
        {
            static float rx0 = 0, ry0 = 0, rx1 = 0, ry1 = 0;
            static const int parsed = std::sscanf(regionEnv, "%f,%f,%f,%f", &rx0, &ry0, &rx1, &ry1);
            (void)parsed;
            --remaining;
            size_t offset = 0;
            while (offset + 16u <= size)
            {
                uint64_t tag[2];
                std::memcpy(tag, packet + offset, sizeof(tag));
                offset += 16u;
                const uint32_t nloop = static_cast<uint32_t>(tag[0] & 0x7FFFu);
                const uint32_t format = static_cast<uint32_t>((tag[0] >> 58) & 3u);
                uint32_t nreg = static_cast<uint32_t>((tag[0] >> 60) & 0xFu);
                if (nreg == 0u)
                    nreg = 16u;
                const uint32_t qwords = format == 0u ? nloop * nreg : format == 1u ? (nloop * nreg + 1u) / 2u : nloop;
                for (uint32_t q = 0; q < qwords && offset + 16u <= size; ++q, offset += 16u)
                {
                    const uint32_t reg = static_cast<uint32_t>((tag[1] >> ((q % nreg) * 4u)) & 0xFu);
                    if (format != 0u || (reg != 4u && reg != 5u))
                        continue;
                    uint32_t w[4];
                    std::memcpy(w, packet + offset, sizeof(w));
                    const float x = (w[0] & 0xFFFFu) / 16.0f, y = (w[1] & 0xFFFFu) / 16.0f;
                    if (x < rx0 || x > rx1 || y < ry0 || y > ry1)
                        continue;
                    const uint32_t z = reg == 4u ? (w[2] >> 4) & 0xFFFFFFu : w[2];
                    std::fprintf(stderr, "[kickv] %04x/%x %.1f %.1f %u\n", t_programPc, t_programItop, x, y, z);
                }
                if ((tag[0] >> 15) & 1u)
                    break;
            }
            return;
        }
        --remaining;
        std::fprintf(stderr, "[kick] program %04x itop=%x, %zu bytes\n", t_programPc, t_programItop, size);
        size_t offset = 0;
        while (offset + 16u <= size)
        {
            uint64_t tag[2];
            std::memcpy(tag, packet + offset, sizeof(tag));
            offset += 16u;
            const uint32_t nloop = static_cast<uint32_t>(tag[0] & 0x7FFFu);
            const uint32_t format = static_cast<uint32_t>((tag[0] >> 58) & 3u);
            uint32_t nreg = static_cast<uint32_t>((tag[0] >> 60) & 0xFu);
            if (nreg == 0u)
                nreg = 16u;
            std::fprintf(stderr, "  tag nloop=%u eop=%u pre=%u prim=%03x flg=%u nreg=%u regs=%016llx\n", nloop,
                         static_cast<uint32_t>((tag[0] >> 15) & 1u), static_cast<uint32_t>((tag[0] >> 46) & 1u),
                         static_cast<uint32_t>((tag[0] >> 47) & 0x7FFu), format, nreg,
                         static_cast<unsigned long long>(tag[1]));
            const uint32_t qwords = format == 0u ? nloop * nreg : format == 1u ? (nloop * nreg + 1u) / 2u : nloop;
            for (uint32_t q = 0; q < qwords && offset + 16u <= size; ++q, offset += 16u)
            {
                if (format != 0u)
                    continue;
                const uint32_t reg = static_cast<uint32_t>((tag[1] >> ((q % nreg) * 4u)) & 0xFu);
                uint32_t w[4];
                float f[4];
                std::memcpy(w, packet + offset, sizeof(w));
                std::memcpy(f, packet + offset, sizeof(f));
                std::fprintf(stderr, "    r%x %08x %08x %08x %08x  (%g %g %g %g)\n", reg, w[0], w[1], w[2], w[3], f[0],
                             f[1], f[2], f[3]);
            }
            if ((tag[0] >> 15) & 1u)
                break;
        }
    }

    void xgkick(Ctx &c, uint64_t cyc, uint32_t qwordAddress)
    {
        // The GIF reads the packet from VU memory up to the end of the tag
        // with EOP set (the interpreter's progressXgkick, done at once).
        thread_local std::vector<uint8_t> packet(0x10000u + 0x4000u);
        size_t size = 0;
        uint32_t source = (qwordAddress * 16u) & 0x3FFFu;
        auto copyQwords = [&](uint32_t count)
        {
            // Contiguous runs up to the end of VU memory, where reads wrap.
            uint32_t bytes = std::min<uint32_t>(count, 0x10000u / 16u) * 16u;
            while (bytes > 0u && size + bytes <= packet.size())
            {
                const uint32_t run = std::min<uint32_t>(bytes, 0x4000u - source);
                std::memcpy(packet.data() + size, c.mem + source, run);
                size += run;
                bytes -= run;
                source = (source + run) & 0x3FFFu;
            }
        };
        for (;;)
        {
            if (size > 0x10000u)
            {
                static uint32_t reported = 0;
                if (reported++ < 10u)
                    std::fprintf(stderr, "[vu1rec] XGKICK at %x runs past 64 KB without EOP\n", qwordAddress);
                return;
            }
            uint64_t tag = 0;
            std::memcpy(&tag, c.mem + source, sizeof(tag));
            copyQwords(1);
            const uint32_t nloop = static_cast<uint32_t>(tag & 0x7FFFu);
            const uint32_t format = static_cast<uint32_t>((tag >> 58) & 3u);
            uint32_t nreg = static_cast<uint32_t>((tag >> 60) & 0xFu);
            if (nreg == 0u)
                nreg = 16u;
            uint32_t qwords = 0;
            if (format == 0u)
                qwords = nloop * nreg;
            else if (format == 1u)
                qwords = (nloop * nreg + 1u) / 2u;
            else
                qwords = nloop;
            copyQwords(qwords);
            if ((tag >> 15) & 1u)
                break;
        }
        const uint32_t qwordCount = static_cast<uint32_t>(size / 16u);
        traceKick(packet.data(), size);
        wait(cyc, c.kickFree);
        c.kickFree = cyc + 2u * qwordCount;
        ++c.kicks;
        if (c.captureKicks)
        {
            if (g_capturedKicks)
                g_capturedKicks->emplace_back(packet.begin(), packet.begin() + static_cast<std::ptrdiff_t>(size));
            return;
        }
        if (c.memory)
            c.memory->submitGifPacket(GifPathId::Path1, packet.data(), static_cast<uint32_t>(size));
    }

    void efu(Ctx &c, uint64_t cyc, uint32_t op, const float *vs, uint32_t component, uint32_t latency)
    {
        const float x = vs[0], y = vs[1], z = vs[2];
        float value = 0.0f;
        switch (op)
        {
        case 0x70:
            value = x * x + y * y + z * z;
            break;
        case 0x71:
        {
            const float sum = x * x + y * y + z * z;
            value = sum != 0.0f ? 1.0f / sum : sum;
            break;
        }
        case 0x72:
            value = std::sqrt(x * x + y * y + z * z);
            break;
        case 0x73:
        {
            const float length = std::sqrt(x * x + y * y + z * z);
            value = length != 0.0f ? 1.0f / length : length;
            break;
        }
        case 0x74:
            value = x != 0.0f ? eatan(y / x) : 0.0f;
            break;
        case 0x75:
            value = x != 0.0f ? eatan(z / x) : 0.0f;
            break;
        case 0x76:
            value = vs[0] + vs[1] + vs[2] + vs[3];
            break;
        case 0x77:
        {
            value = vs[component];
            if (value >= 0.0f)
            {
                value = std::sqrt(value);
                if (value != 0.0f)
                    value = 1.0f / value;
            }
            break;
        }
        case 0x78:
            value = vs[component] >= 0.0f ? std::sqrt(vs[component]) : vs[component];
            break;
        case 0x79:
            value = esin(vs[component]);
            break;
        case 0x7A:
            value = vs[component] != 0.0f ? 1.0f / vs[component] : vs[component];
            break;
        case 0x7C:
            value = eatan(vs[component]);
            break;
        case 0x7D:
            value = eexp(vs[component]);
            break;
        default:
            break;
        }
        int slot = !c.pPending[0] ? 0 : (!c.pPending[1] ? 1 : 0);
        c.pPending[slot] = true;
        c.pValue[slot] = value;
        c.pReady[slot] = cyc + latency;
        c.efuFree = cyc + (latency > 0u ? latency - 1u : 0u);
        if (c.pReady[slot] < c.nextDue)
            c.nextDue = c.pReady[slot];
    }

    void rop(VU1State &s, uint32_t op, uint32_t vft, uint32_t vfs, uint32_t dest, uint32_t component)
    {
        auto write = [&](float value)
        {
            if (vft == 0u)
                return;
            for (int l = 0; l < 4; ++l)
                if (dest & (8u >> l))
                    s.vf[vft][l] = value;
        };
        switch (op)
        {
        case 0x40: // RNEXT
        {
            const uint32_t x = (s.r >> 4) & 1u;
            const uint32_t y = (s.r >> 22) & 1u;
            s.r = (((s.r << 1) ^ x ^ y) & 0x007FFFFFu) | 0x3F800000u;
            write(bitsf(s.r));
            break;
        }
        case 0x41: // RGET
            write(bitsf(s.r));
            break;
        case 0x42: // RINIT
            s.r = 0x3F800000u | (fbits(s.vf[vfs][component]) & 0x007FFFFFu);
            break;
        case 0x43: // RXOR
            s.r = 0x3F800000u | ((s.r ^ fbits(s.vf[vfs][component])) & 0x007FFFFFu);
            break;
        default:
            break;
        }
    }

    void clip(Ctx &c, uint64_t cyc, const float *vs, float w)
    {
        const uint32_t wBits = fbits(w);
        const int32_t limit = (wBits & 0x7F800000u) != 0u ? static_cast<int32_t>(wBits & 0x7FFFFFFFu) : 0x007FFFFF;
        auto exceeds = [limit](float value, uint32_t signMask)
        {
            const int32_t ordered = static_cast<int32_t>(fbits(value) ^ signMask);
            return ordered > limit;
        };
        uint32_t flags = 0;
        if (exceeds(vs[0], 0u))
            flags |= 0x01u;
        if (exceeds(vs[0], 0x80000000u))
            flags |= 0x02u;
        if (exceeds(vs[1], 0u))
            flags |= 0x04u;
        if (exceeds(vs[1], 0x80000000u))
            flags |= 0x08u;
        if (exceeds(vs[2], 0u))
            flags |= 0x10u;
        if (exceeds(vs[2], 0x80000000u))
            flags |= 0x20u;
        c.workingClip = ((c.workingClip << 6) | flags) & 0xFFFFFFu;
        pushFlag(c, cyc, c.workingClip, true);
    }

    void fcset(Ctx &c, uint64_t cyc, uint32_t value)
    {
        c.workingClip = value & 0xFFFFFFu;
        pushFlag(c, cyc, c.workingClip, true);
    }

    void install()
    {
        VU1Interpreter::setNativeProgramHook(&nativeHook);
    }
}
