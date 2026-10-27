#pragma once
// Runtime support for VU1 microprograms translated to C++ by
// tools/vu1_translate.py. Generated code runs one program on a Ctx with the
// VU1 interpreter's timing model: each instruction pair issues one cycle after
// the previous one, later if an operand is still in flight (FMAC results take
// 4 cycles, loads 4, integer ops 1), and Q, P, MAC and clip results become
// visible only after their latency. Float maths runs with SSE in
// round-toward-zero with denormals flushed (see vu1rec.cpp), which gives the
// VU's saturation at +-FLT_MAX for free.
#include "runtime/ps2_vu1.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

class PS2Memory;

namespace vu1rec
{
    struct Ctx
    {
        VU1State *s = nullptr;
        uint8_t *mem = nullptr; // VU1 data memory (16 KB)
        PS2Memory *memory = nullptr;
        uint64_t cyc = 0; // issue cycle of the current pair
        uint64_t vfReady[32][4] = {};
        uint64_t viReady[16] = {};

        // FDIV (Q register)
        bool qPending = false;
        float qValue = 0.0f;
        uint64_t qReady = 0;

        // EFU (P register): the interpreter keeps two results in flight.
        bool pPending[2] = {};
        float pValue[2] = {};
        uint64_t pReady[2] = {};
        uint64_t efuFree = 0;

        // MAC and clip flag updates in issue order (constant latency).
        struct Flag
        {
            uint64_t ready;
            uint32_t value;
            bool clip;
        };
        Flag flags[16] = {};
        uint32_t flagHead = 0;
        uint32_t flagTail = 0;
        uint32_t workingClip = 0;

        uint64_t nextDue = UINT64_MAX; // earliest pending result
        uint64_t kickFree = 0;         // PATH1 busy until

        bool captureKicks = false; // verification: keep GIF packets instead of sending them
        uint32_t kicks = 0;
        bool trapped = false;
        uint32_t trapPc = 0;
    };

    using ProgramFn = void (*)(Ctx &);
    struct ProgramEntry
    {
        uint64_t hash;
        ProgramFn fn;
    };
    extern const ProgramEntry kPrograms[];    // generated, ends with {0, nullptr}
    extern const ProgramEntry kVu0Programs[]; // VU0 microcode (tools/vu1_translate.py --vu0)

    // Out of line (vu1rec.cpp). Generated code keeps the issue cycle in a
    // local and passes it to the helpers that need it.
    void commit(Ctx &c, uint64_t cyc);
    void finish(Ctx &c, uint64_t cyc);
    void trap(Ctx &c, uint64_t cyc, uint32_t pc);
    void xgkick(Ctx &c, uint64_t cyc, uint32_t qwordAddress);
    void efu(Ctx &c, uint64_t cyc, uint32_t op, const float *vs, uint32_t component, uint32_t latency);
    void rop(VU1State &s, uint32_t op, uint32_t vft, uint32_t vfs, uint32_t dest, uint32_t component);
    void clip(Ctx &c, uint64_t cyc, const float *vs, float w);
    void fcset(Ctx &c, uint64_t cyc, uint32_t value);

    inline void wait(uint64_t &cyc, uint64_t ready)
    {
        if (ready > cyc)
            cyc = ready;
    }

    inline void waitP(Ctx &c, uint64_t &cyc)
    {
        for (int i = 0; i < 2; ++i)
            if (c.pPending[i])
                wait(cyc, c.pReady[i]);
    }

    // MAC and clip updates are applied when something reads them (or at the
    // end of the program): reads see every update issued 4+ cycles before.
    inline void syncFlags(Ctx &c, uint64_t cyc)
    {
        while (c.flagHead != c.flagTail && c.flags[c.flagHead & 15u].ready <= cyc)
        {
            const Ctx::Flag &entry = c.flags[c.flagHead++ & 15u];
            if (entry.clip)
                c.s->clip = entry.value;
            else
                c.s->mac = entry.value;
        }
    }

    inline uint32_t qaddr(int32_t qword)
    {
        return (static_cast<uint32_t>(qword) * 16u) & 0x3FFFu;
    }

    // VU0 data memory is 4 KB.
    inline uint32_t qaddr0(int32_t qword)
    {
        return (static_cast<uint32_t>(qword) * 16u) & 0x0FFFu;
    }

    inline float ldf(const uint8_t *mem, uint32_t address)
    {
        float value;
        std::memcpy(&value, mem + address, sizeof(value));
        return value;
    }

    inline void stf(uint8_t *mem, uint32_t address, float value)
    {
        std::memcpy(mem + address, &value, sizeof(value));
    }

    inline void stu(uint8_t *mem, uint32_t address, uint32_t value)
    {
        std::memcpy(mem + address, &value, sizeof(value));
    }

    inline int32_t ldi(const uint8_t *mem, uint32_t address)
    {
        uint32_t value;
        std::memcpy(&value, mem + address, sizeof(value));
        return static_cast<int16_t>(value & 0xFFFFu);
    }

    inline float bitsf(uint32_t bits)
    {
        float value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    inline uint32_t fbits(float value)
    {
        uint32_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }

    inline int32_t lo16(float value)
    {
        return static_cast<int16_t>(fbits(value) & 0xFFFFu);
    }

    inline float i2f(int32_t value)
    {
        float result;
        std::memcpy(&result, &value, sizeof(result));
        return result;
    }

    inline float vmax(float a, float b) { return a > b ? a : b; }
    inline float vmin(float a, float b) { return a < b ? a : b; }

    inline float itof(float raw, float scale)
    {
        int32_t value;
        std::memcpy(&value, &raw, sizeof(value));
        return static_cast<float>(value) / scale;
    }

    inline float ftoi(float value, float scale)
    {
        const double scaled = static_cast<double>(value) * static_cast<double>(scale);
        int32_t result;
        if (scaled >= static_cast<double>(std::numeric_limits<int32_t>::max()))
            result = std::numeric_limits<int32_t>::max();
        else if (scaled <= static_cast<double>(std::numeric_limits<int32_t>::min()))
            result = std::numeric_limits<int32_t>::min();
        else
            result = static_cast<int32_t>(scaled);
        return i2f(result);
    }

    // MAC flags of one result lane (Z, S and O; lane is the DEST bit).
    inline uint32_t macBits(float value, uint32_t lane)
    {
        const uint32_t bits = fbits(value);
        const uint32_t magnitude = bits & 0x7FFFFFFFu;
        uint32_t flags = (bits >> 31) ? (lane << 4) : 0u;
        if (magnitude == 0u)
            flags |= lane;
        else if (magnitude == 0x7F7FFFFFu)
            flags |= lane << 12;
        return flags;
    }

    inline void pushFlag(Ctx &c, uint64_t cyc, uint32_t value, bool clipFlag)
    {
        // A full queue retires its oldest entry: issued 16+ cycles ago, it is
        // long past its 4-cycle latency.
        if (c.flagTail - c.flagHead == 16u)
        {
            const Ctx::Flag &oldest = c.flags[c.flagHead++ & 15u];
            if (oldest.clip)
                c.s->clip = oldest.value;
            else
                c.s->mac = oldest.value;
        }
        Ctx::Flag &entry = c.flags[c.flagTail++ & 15u];
        entry.ready = cyc + 4u;
        entry.value = value;
        entry.clip = clipFlag;
    }

    inline void pushMac(Ctx &c, uint64_t cyc, uint32_t mac)
    {
        pushFlag(c, cyc, mac, false);
    }

    inline void fdiv(Ctx &c, uint64_t cyc, float value, uint32_t latency)
    {
        c.qPending = true;
        c.qValue = value;
        c.qReady = cyc + latency;
        if (c.qReady < c.nextDue)
            c.nextDue = c.qReady;
    }

    inline float vdiv(float num, float den)
    {
        if (den == 0.0f)
            return std::signbit(num) != std::signbit(den) ? -std::numeric_limits<float>::max()
                                                          : std::numeric_limits<float>::max();
        return num / den;
    }

    inline float vsqrt(float value)
    {
        return std::sqrt(std::fabs(value));
    }

    inline float vrsqrt(float num, float radicand)
    {
        const float den = std::sqrt(std::fabs(radicand));
        if (den != 0.0f)
            return num / den;
        return std::signbit(num) ? -std::numeric_limits<float>::max() : std::numeric_limits<float>::max();
    }
}
