// Based on Blackline Interactive implementation
#include "runtime/ps2_memory.h"
#include "runtime/ps2_perf.h"
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <unordered_set>

extern std::atomic<uint64_t> g_ps2xVBlankCount;

enum VIFCmd : uint8_t
{
    VIF_NOP = 0x00,
    VIF_STCYCL = 0x01,
    VIF_OFFSET = 0x02,
    VIF_BASE = 0x03,
    VIF_ITOP = 0x04,
    VIF_STMOD = 0x05,
    VIF_MSKPATH3 = 0x06,
    VIF_MARK = 0x07,
    VIF_FLUSHE = 0x10,
    VIF_FLUSH = 0x11,
    VIF_FLUSHA = 0x13,
    VIF_MSCAL = 0x14,
    VIF_MSCALF = 0x15,
    VIF_MSCNT = 0x17,
    VIF_STMASK = 0x20,
    VIF_STROW = 0x30,
    VIF_STCOL = 0x31,
    VIF_MPG = 0x4A,
    VIF_DIRECT = 0x50,
    VIF_DIRECTHL = 0x51,
};

namespace
{
    constexpr uint8_t kGifFmtImage = 2u;

    // Walks the GIF tags of a DIRECT payload and returns how many IMAGE
    // quadwords the last tag still expects beyond the payload (they arrive in
    // the following DIRECT, e.g. a tag ending a TRXPOS/TRXDIR packet whose
    // pixels are sent by the next DMA REF).
    uint32_t pendingGifImageQwc(const uint8_t *data, uint32_t sizeBytes)
    {
        if (!data)
            return 0u;
        uint32_t pos = 0u;
        while (pos + 16u <= sizeBytes)
        {
            uint64_t tagLo = 0u;
            std::memcpy(&tagLo, data + pos, sizeof(tagLo));
            pos += 16u;
            const uint64_t nloop = tagLo & 0x7FFFu;
            const uint8_t flg = static_cast<uint8_t>((tagLo >> 58) & 0x3u);
            uint64_t nreg = (tagLo >> 60) & 0xFu;
            if (nreg == 0u)
                nreg = 16u;
            uint64_t qwords = 0u;
            if (flg == 0u) // PACKED
                qwords = nloop * nreg;
            else if (flg == 1u) // REGLIST
                qwords = (nloop * nreg + 1u) / 2u;
            else // IMAGE
                qwords = nloop;
            const uint64_t available = (sizeBytes - pos) / 16u;
            if (qwords > available)
                return flg >= kGifFmtImage ? static_cast<uint32_t>(qwords - available) : 0u;
            pos += static_cast<uint32_t>(qwords * 16u);
        }
        return 0u;
    }

    // Splits a DIRECT payload before its last GIF tag when that tag's PACKED or
    // REGLIST data runs past the payload (the GIF waits for the rest from the
    // next DIRECT, e.g. sdfGrSetRefGifPacket sends the tag with one DIRECT and
    // the A+D qword with the next). Returns the bytes of complete tags, plus a
    // trailing IMAGE tag whose missing quadwords go to imageOwedQwc; the cut
    // tag's missing quadwords go to partialOwedQwc.
    uint32_t splitGifPayload(const uint8_t *data, uint32_t sizeBytes, uint32_t &imageOwedQwc, uint32_t &partialOwedQwc)
    {
        imageOwedQwc = 0u;
        partialOwedQwc = 0u;
        uint32_t pos = 0u;
        while (pos + 16u <= sizeBytes)
        {
            const uint32_t tagStart = pos;
            uint64_t tagLo = 0u;
            std::memcpy(&tagLo, data + pos, sizeof(tagLo));
            pos += 16u;
            const uint64_t nloop = tagLo & 0x7FFFu;
            const uint8_t flg = static_cast<uint8_t>((tagLo >> 58) & 0x3u);
            uint64_t nreg = (tagLo >> 60) & 0xFu;
            if (nreg == 0u)
                nreg = 16u;
            uint64_t qwords = 0u;
            if (flg == 0u) // PACKED
                qwords = nloop * nreg;
            else if (flg == 1u) // REGLIST
                qwords = (nloop * nreg + 1u) / 2u;
            else // IMAGE
                qwords = nloop;
            const uint64_t available = (sizeBytes - pos) / 16u;
            if (qwords > available)
            {
                if (flg >= kGifFmtImage)
                {
                    imageOwedQwc = static_cast<uint32_t>(qwords - available);
                    return sizeBytes;
                }
                partialOwedQwc = static_cast<uint32_t>(qwords - available);
                return tagStart;
            }
            pos += static_cast<uint32_t>(qwords * 16u);
        }
        return sizeBytes;
    }

    // A PATH2 PACKED/REGLIST GIF packet cut at the end of a DIRECT, and the
    // quadwords it still needs from the following DIRECT payloads.
    std::vector<uint8_t> s_path2Partial;
    uint32_t s_path2PartialOwedQwc = 0u;

    template <uint32_t VL>
    inline uint32_t unpackElement(const uint8_t *p, bool zeroExtend)
    {
        if constexpr (VL == 0u)
        {
            uint32_t raw;
            std::memcpy(&raw, p, sizeof(raw));
            return raw;
        }
        else if constexpr (VL == 1u)
        {
            uint16_t raw;
            std::memcpy(&raw, p, sizeof(raw));
            return zeroExtend ? raw : static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(raw)));
        }
        else
        {
            return zeroExtend ? p[0] : static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(p[0])));
        }
    }

    // The unmasked UNPACK of one S/V2/V3/V4 format with 32/16/8-bit elements
    // and CL >= WL (vifUnpack's fast path), lanes filled as in decodeVector.
    template <uint32_t VN, uint32_t VL>
    void unpackFormat(uint8_t *vuData, uint32_t addressMask, uint32_t vuAddr, uint32_t cl, uint32_t wl,
                      const uint8_t *src, const uint8_t *srcEnd, uint32_t count, bool zeroExtend)
    {
        constexpr uint32_t elementBytes = VL == 0u ? 4u : VL == 1u ? 2u : 1u;
        constexpr uint32_t vectorBytes = (VN + 1u) * elementBytes;
        if constexpr (VN == 3u && VL == 0u)
        {
            if (cl == wl)
            {
                // Consecutive quadwords: copy runs up to the end of VU memory.
                uint32_t dest = vuAddr;
                while (count > 0u)
                {
                    const uint32_t run = std::min(count, addressMask + 1u - dest);
                    std::memcpy(vuData + dest * 16u, src, run * 16u);
                    src += run * 16u;
                    count -= run;
                    dest = (dest + run) & addressMask;
                }
                return;
            }
        }
        uint32_t block = 0u, inBlock = 0u; // w / wl and w % wl
        for (uint32_t w = 0; w < count; ++w, src += vectorBytes)
        {
            const uint32_t destVec = (vuAddr + block * cl + inBlock) & addressMask;
            if (++inBlock == wl)
            {
                inBlock = 0u;
                ++block;
            }
            uint32_t words[4];
            uint32_t lanes = 4u;
            if constexpr (VN == 0u)
            {
                words[0] = words[1] = words[2] = words[3] = unpackElement<VL>(src, zeroExtend);
            }
            else if constexpr (VN == 1u)
            {
                words[0] = words[2] = unpackElement<VL>(src, zeroExtend);
                words[1] = words[3] = unpackElement<VL>(src + elementBytes, zeroExtend);
            }
            else
            {
                words[0] = unpackElement<VL>(src, zeroExtend);
                words[1] = unpackElement<VL>(src + elementBytes, zeroExtend);
                words[2] = unpackElement<VL>(src + 2u * elementBytes, zeroExtend);
                // V3 takes W from the next element of the stream, when there is one.
                if (VN == 3u || srcEnd - src >= static_cast<std::ptrdiff_t>(4u * elementBytes))
                    words[3] = unpackElement<VL>(src + 3u * elementBytes, zeroExtend);
                else
                    lanes = 3u;
            }
            std::memcpy(vuData + destVec * 16u, words, lanes * 4u);
        }
    }

    // Executes the UNPACK VIFcode `cmd` whose data starts at data + pos, for
    // VIF1 (VU1 memory, addresses relative to TOPS when FLG is set) or VIF0
    // (VU0 memory, no double buffering). Advances pos past the data; returns
    // false when the data runs past the end of the packet.
    bool vifUnpack(VIFRegisters &regs, uint8_t *vuData, uint32_t vuSize, bool vif1, uint32_t cmd,
                   const uint8_t *data, uint32_t &pos, uint32_t sizeBytes)
    {
        const uint8_t opcode = static_cast<uint8_t>((cmd >> 24) & 0x7Fu);
        const uint16_t imm = static_cast<uint16_t>(cmd & 0xFFFFu);
        const uint8_t num = static_cast<uint8_t>((cmd >> 16) & 0xFFu);
        const uint8_t vn = (opcode >> 2) & 0x3;
        const uint8_t vl = opcode & 0x3;
        const bool maskEnable = (opcode & 0x10u) != 0u;
        const int components = vn + 1;
        int bitsPerComponent = 32;
        switch (vl)
        {
        case 0:
            bitsPerComponent = 32;
            break;
        case 1:
            bitsPerComponent = 16;
            break;
        case 2:
            bitsPerComponent = 8;
            break;
        default:
            bitsPerComponent = (vn == 3) ? 4 : 16;
            break;
        }
        const int bitsPerVector = (vl == 3 && vn == 3) ? 16 : (components * bitsPerComponent);
        const uint32_t bytesPerVector = (bitsPerVector + 7) / 8;
        // UNPACK semantics: NUM is 8-bit and NUM==0 means 256 vectors (writes).
        const uint32_t writeVectorCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
        const uint32_t addressMask = vuSize / 16u - 1u;

        // STCYCL controls write cycles for UNPACK.
        uint32_t cl = regs.cycle & 0xFFu;
        uint32_t wl = (regs.cycle >> 8) & 0xFFu;
        if (cl == 0u)
            cl = 1u;
        if (wl == 0u)
            wl = 1u;

        uint32_t sourceVectorCount = writeVectorCount;
        if (cl < wl)
        {
            const uint32_t fullBlocks = writeVectorCount / wl;
            uint32_t remainder = writeVectorCount % wl;
            if (remainder > cl)
                remainder = cl;
            sourceVectorCount = fullBlocks * cl + remainder;
        }

        uint32_t totalBytes = sourceVectorCount * bytesPerVector;
        totalBytes = (totalBytes + 3) & ~3u;

        uint32_t vuAddr = (uint32_t)imm & 0x3FFu;
        if (vif1 && (imm & 0x8000u) != 0u)
            vuAddr = vuAddr + (regs.tops & 0x3FFu);
        vuAddr &= addressMask;

        const bool zeroExtend = (imm & 0x4000u) != 0u;

        // PS2X_TRACE_UNPACK=<emulated s>: each distinct UNPACK shape from then on.
        static const char *traceUnpack = std::getenv("PS2X_TRACE_UNPACK");
        if (traceUnpack && g_ps2xVBlankCount.load() / 59.94 >= std::atof(traceUnpack))
        {
            static std::unordered_set<uint64_t> seen;
            const uint64_t key = (static_cast<uint64_t>(cmd & 0x7F00FFFFu) << 32) ^
                                 (static_cast<uint64_t>(regs.cycle & 0xFFFFu) << 8) ^
                                 (static_cast<uint64_t>(regs.mode & 3u) << 4) ^ (maskEnable ? regs.mask : 0u) ^ (vif1 ? 0u : 1u);
            if (seen.insert(key).second && seen.size() < 400u)
                std::fprintf(stderr, "[unpack] vif%d cmd=%08x vn=%u vl=%u num=%u addr=%03x%s%s cl=%u wl=%u mode=%u mask=%s%08x tops=%x\n",
                             vif1 ? 1 : 0, cmd, vn, vl, writeVectorCount, imm & 0x3FFu, (imm & 0x8000u) ? " +tops" : "",
                             zeroExtend ? " usn" : "", cl, wl, regs.mode & 3u, maskEnable ? "" : "off/",
                             regs.mask, regs.tops);
        }

        // Lanes a format does not supply are written as the VIF does (as
        // measured by PCSX2): S broadcasts X, V2 writes X Y X Y, and V3
        // takes W from the next element of the stream (the following
        // vector's X). Returns how many lanes of `words` are valid.
        const uint32_t elementBytes = static_cast<uint32_t>(bitsPerComponent) / 8u;
        auto decodeVector = [&](const uint8_t *src, uint32_t available, uint32_t words[4]) -> uint32_t
        {
            auto element = [&](uint32_t index) -> uint32_t
            {
                const uint8_t *p = src + index * elementBytes;
                if (vl == 0u)
                {
                    uint32_t raw;
                    std::memcpy(&raw, p, sizeof(raw));
                    return raw;
                }
                if (vl == 1u)
                {
                    uint16_t raw;
                    std::memcpy(&raw, p, sizeof(raw));
                    return zeroExtend ? raw : static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(raw)));
                }
                return zeroExtend ? p[0] : static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(p[0])));
            };
            if (vl == 3u)
            {
                uint16_t packed;
                std::memcpy(&packed, src, sizeof(packed));
                words[0] = (packed & 0x1Fu) << 3;
                words[1] = ((packed >> 5) & 0x1Fu) << 3;
                words[2] = ((packed >> 10) & 0x1Fu) << 3;
                words[3] = ((packed >> 15) & 0x01u) << 7;
                return 4u;
            }
            switch (components)
            {
            case 1:
                words[0] = words[1] = words[2] = words[3] = element(0);
                return 4u;
            case 2:
                words[0] = words[2] = element(0);
                words[1] = words[3] = element(1);
                return 4u;
            case 3:
                words[0] = element(0);
                words[1] = element(1);
                words[2] = element(2);
                if (available < 4u * elementBytes)
                    return 3u; // no following element in this packet
                words[3] = element(3);
                return 4u;
            default:
                for (uint32_t c = 0; c < 4u; ++c)
                    words[c] = element(c);
                return 4u;
            }
        };

        // Fast path for the usual case: no write mask, no ROW add, every
        // write cycle has source data (CL >= WL).
        static const bool watchingQword = std::getenv("PS2X_WATCH_VU1_QWORD") != nullptr;
        const bool fastFormat = vl != 3u || vn == 3u;
        if (vuData && totalBytes > 0 && pos + totalBytes <= sizeBytes && !maskEnable &&
            (regs.mode & 3u) == 0u && cl >= wl && fastFormat && !(vif1 && watchingQword))
        {
            const uint8_t *src = data + pos;
            const uint8_t *srcEnd = data + sizeBytes;
            const uint32_t n = writeVectorCount;
            switch (vl * 4u + vn)
            {
#define PS2X_UNPACK_CASE(VL_, VN_) \
    case VL_ * 4u + VN_:           \
        unpackFormat<VN_, VL_>(vuData, addressMask, vuAddr, cl, wl, src, srcEnd, n, zeroExtend); \
        break;
                PS2X_UNPACK_CASE(0u, 0u)
                PS2X_UNPACK_CASE(0u, 1u)
                PS2X_UNPACK_CASE(0u, 2u)
                PS2X_UNPACK_CASE(0u, 3u)
                PS2X_UNPACK_CASE(1u, 0u)
                PS2X_UNPACK_CASE(1u, 1u)
                PS2X_UNPACK_CASE(1u, 2u)
                PS2X_UNPACK_CASE(1u, 3u)
                PS2X_UNPACK_CASE(2u, 0u)
                PS2X_UNPACK_CASE(2u, 1u)
                PS2X_UNPACK_CASE(2u, 2u)
                PS2X_UNPACK_CASE(2u, 3u)
#undef PS2X_UNPACK_CASE
            default: // V4-5
                for (uint32_t w = 0; w < writeVectorCount; ++w, src += bytesPerVector)
                {
                    const uint32_t destVec = (vuAddr + (w / wl) * cl + (w % wl)) & addressMask;
                    uint32_t words[4];
                    const uint32_t count = decodeVector(src, static_cast<uint32_t>(data + sizeBytes - src), words);
                    std::memcpy(vuData + destVec * 16u, words, count * 4u);
                }
                break;
            }
            pos += totalBytes;
            return pos <= sizeBytes;
        }

        if (vuData && totalBytes > 0 && pos + totalBytes <= sizeBytes)
        {
            const uint8_t *srcBase = data + pos;
            uint32_t srcIndex = 0u;
            for (uint32_t writeIndex = 0; writeIndex < writeVectorCount; ++writeIndex)
            {
                const uint32_t cyclePos = writeIndex % wl;
                const bool sourceAvailable = (cl >= wl) || (cyclePos < cl);
                const uint32_t destVec = (cl >= wl) ? ((vuAddr + (writeIndex / wl) * cl + cyclePos) & addressMask)
                                                    : ((vuAddr + writeIndex) & addressMask);
                const uint32_t destOff = destVec * 16u;

                uint32_t lanes[4] = {0u, 0u, 0u, 0u};
                std::memcpy(lanes, vuData + destOff, sizeof(lanes));
                uint32_t decompressed[4] = {lanes[0], lanes[1], lanes[2], lanes[3]};
                bool decoded = false;

                const uint8_t *srcVec = nullptr;
                if (sourceAvailable && srcIndex < sourceVectorCount)
                {
                    srcVec = srcBase + srcIndex * bytesPerVector;
                    ++srcIndex;
                    decoded = true;
                }

                const bool handledFormat = decoded && (vl != 3u || vn == 3u);
                if (handledFormat)
                    decodeVector(srcVec, static_cast<uint32_t>(data + sizeBytes - srcVec), decompressed);

                // Unknown compressed format fallback: preserve legacy raw-copy behavior.
                if (!handledFormat && decoded && !maskEnable && (regs.mode == 0u || regs.mode == 3u))
                {
                    const uint32_t copyBytes = (bytesPerVector < 16u) ? bytesPerVector : 16u;
                    std::memcpy(vuData + destOff, srcVec, copyBytes);
                    continue;
                }

                const bool canAdd = (vl != 3u || vn != 3u);
                const uint32_t mode = regs.mode & 3u;
                const uint32_t colIdx = (cyclePos > 3u) ? 3u : cyclePos;
                const uint32_t maskCycle = (cyclePos > 3u) ? 3u : cyclePos;

                for (uint32_t field = 0u; field < 4u; ++field)
                {
                    uint32_t maskSpec = 0u;
                    if (maskEnable)
                    {
                        const uint32_t shift = ((maskCycle * 4u) + field) * 2u;
                        maskSpec = (regs.mask >> shift) & 0x3u;
                    }

                    // In fill-write cycles with suspended source reads, treat raw-data selections as row-fill.
                    if (!decoded && maskSpec == 0u)
                        maskSpec = 1u;

                    uint32_t writeVal = lanes[field];
                    if (maskSpec == 0u)
                    {
                        if (handledFormat)
                        {
                            writeVal = decompressed[field];
                            if (canAdd && (mode == 1u || mode == 2u))
                            {
                                writeVal = writeVal + regs.row[field];
                                if (mode == 2u)
                                    regs.row[field] = writeVal;
                            }
                        }
                    }
                    else if (maskSpec == 1u)
                    {
                        writeVal = regs.row[field];
                    }
                    else if (maskSpec == 2u)
                    {
                        writeVal = regs.col[colIdx];
                    }
                    else
                    {
                        continue; // write-protect
                    }

                    lanes[field] = writeVal;
                }

                // PS2X_WATCH_VU1_QWORD=<hex>: logs UNPACK writes to that VU1 data qword.
                static const char *watchEnv = std::getenv("PS2X_WATCH_VU1_QWORD");
                if (vif1 && watchEnv && destVec == (std::strtoul(watchEnv, nullptr, 16) & 0x3FFu))
                {
                    static uint32_t logged = 0;
                    if (logged++ < 60u)
                        std::fprintf(stderr, "[vif1] UNPACK imm=%04x num=%u vn=%u vl=%u cl=%u wl=%u tops=%x write#%u mask=%08x mode=%u %s -> %08x %08x %08x %08x\n",
                                     static_cast<uint32_t>(imm), writeVectorCount, vn, vl, cl, wl, regs.tops,
                                     writeIndex, regs.mask, regs.mode, decoded ? "data" : "fill",
                                     lanes[0], lanes[1], lanes[2], lanes[3]);
                }

                std::memcpy(vuData + destOff, lanes, sizeof(lanes));
            }
        }
        pos += totalBytes;
        return pos <= sizeBytes;
    }
}

// VIF0 MSCAL/MSCALF (start address in bytes) and MSCNT (UINT32_MAX) start a
// VU0 microprogram with the given ITOP; installed by the runtime, which runs
// it on the shared VU0 registers.
std::function<void(uint32_t startPc, uint32_t itop)> g_ps2xVif0MicroprogramHandler;

// Shared with the PATH3 (GIF DMA) path in ps2_memory.cpp.
uint32_t ps2xPendingGifImageQwc(const uint8_t *data, uint32_t sizeBytes)
{
    return pendingGifImageQwc(data, sizeBytes);
}

// Drops a PATH2 GIF packet waiting for the rest of its data (VIF1 reset).
void ps2xResetPath2Partial()
{
    s_path2Partial.clear();
    s_path2PartialOwedQwc = 0u;
}

void PS2Memory::processVIF0Data(uint32_t srcPhys, uint32_t sizeBytes)
{
    if (sizeBytes == 0u || srcPhys >= PS2_RAM_SIZE)
        return;

    const uint64_t requestedEnd = static_cast<uint64_t>(srcPhys) + static_cast<uint64_t>(sizeBytes);
    if (requestedEnd > static_cast<uint64_t>(PS2_RAM_SIZE))
        sizeBytes = PS2_RAM_SIZE - srcPhys;

    processVIF0Data(m_rdram + srcPhys, sizeBytes);
}

void PS2Memory::processVIF0Data(const uint8_t *data, uint32_t sizeBytes)
{
    if (sizeBytes == 0u)
        return;

    uint32_t pos = 0;
    while (pos + 4 <= sizeBytes)
    {
        uint32_t cmd = 0u;
        std::memcpy(&cmd, data + pos, sizeof(cmd));
        pos += 4u;

        const uint8_t opcode = static_cast<uint8_t>((cmd >> 24) & 0x7Fu);
        const uint16_t imm = static_cast<uint16_t>(cmd & 0xFFFFu);
        const uint8_t num = static_cast<uint8_t>((cmd >> 16) & 0xFFu);
        const bool irq = (cmd & 0x80000000u) != 0u;

        vif0_regs.code = cmd;
        vif0_regs.num = num;
        if (irq)
            vif0_regs.stat |= (1u << 11);

        if (opcode == VIF_NOP)
        {
            continue;
        }
        else if (opcode == VIF_STCYCL)
        {
            vif0_regs.cycle = imm;
            continue;
        }
        else if (opcode == VIF_ITOP)
        {
            vif0_regs.itops = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_STMOD)
        {
            vif0_regs.mode = imm & 3u;
            continue;
        }
        else if (opcode == VIF_MARK)
        {
            vif0_regs.mark = imm;
            vif0_regs.stat |= (1u << 6);
            continue;
        }
        else if (opcode == VIF_FLUSHE || opcode == VIF_FLUSH || opcode == VIF_FLUSHA)
        {
            continue;
        }
        else if (opcode == VIF_STMASK)
        {
            if (pos + 4u > sizeBytes)
                break;
            std::memcpy(&vif0_regs.mask, data + pos, sizeof(vif0_regs.mask));
            pos += 4u;
            continue;
        }
        else if (opcode == VIF_STROW)
        {
            if (pos + 16u > sizeBytes)
                break;
            std::memcpy(vif0_regs.row, data + pos, 16u);
            pos += 16u;
            continue;
        }
        else if (opcode == VIF_STCOL)
        {
            if (pos + 16u > sizeBytes)
                break;
            std::memcpy(vif0_regs.col, data + pos, 16u);
            pos += 16u;
            continue;
        }
        else if (opcode == VIF_MPG)
        {
            const uint32_t destAddr = static_cast<uint32_t>(imm & 0x1FFu) * 8u;
            const uint32_t instructionCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
            const uint32_t mpgBytes = instructionCount * 8u;
            uint32_t copyBytes = 0u;
            if (m_vu0Code && destAddr < PS2_VU0_CODE_SIZE && mpgBytes > 0u)
            {
                copyBytes = mpgBytes;
                if (destAddr + copyBytes > PS2_VU0_CODE_SIZE)
                    copyBytes = PS2_VU0_CODE_SIZE - destAddr;
                if (pos + copyBytes <= sizeBytes)
                {
                    std::memcpy(m_vu0Code + destAddr, data + pos, copyBytes);
                    markVU0CodeModified();
                }
            }

            pos += mpgBytes;
            if (pos > sizeBytes)
                break;
            continue;
        }
        else if (opcode == VIF_MSCAL || opcode == VIF_MSCALF || opcode == VIF_MSCNT)
        {
            // The VU0 microprogram runs to completion here, before the rest
            // of the packet (which can only refill memory it has finished
            // with: the VIF waits for the program before the next MSCAL).
            vif0_regs.itop = vif0_regs.itops & 0x3FFu;
            if (g_ps2xVif0MicroprogramHandler)
                g_ps2xVif0MicroprogramHandler(opcode == VIF_MSCNT ? UINT32_MAX : static_cast<uint32_t>(imm) * 8u,
                                              vif0_regs.itop);
            continue;
        }
        else if ((opcode & 0x60u) == 0x60u)
        {
            if (!vifUnpack(vif0_regs, m_vu0Data, PS2_VU0_DATA_SIZE, false, cmd, data, pos, sizeBytes))
                break;
            continue;
        }
        else
        {
            break;
        }
    }
}

void PS2Memory::processVIF1Data(uint32_t srcPhys, uint32_t sizeBytes)
{
    if (sizeBytes == 0u || srcPhys >= PS2_RAM_SIZE)
        return;

    const uint64_t requestedEnd = static_cast<uint64_t>(srcPhys) + static_cast<uint64_t>(sizeBytes);
    if (requestedEnd > static_cast<uint64_t>(PS2_RAM_SIZE))
        sizeBytes = PS2_RAM_SIZE - srcPhys;

    processVIF1Data(m_rdram + srcPhys, sizeBytes);
}

void PS2Memory::processVIF1Data(const uint8_t *data, uint32_t sizeBytes)
{
    if (sizeBytes == 0u)
        return;
    const Ps2xPerfScope perf(Ps2xPerfSlot::Vif1);

    uint32_t pos = 0;

    // Sends qwords that continue an IMAGE GIF tag from an earlier packet,
    // wrapped in a tag of their own since the GS consumes whole packets.
    auto submitImageContinuation = [&](const uint8_t *src, uint32_t qwords, bool last, bool directHl)
    {
        std::vector<uint8_t> imagePacket(16u + static_cast<size_t>(qwords) * 16u, 0u);
        const uint64_t imageTag =
            static_cast<uint64_t>(qwords & 0x7FFFu) |
            (last ? (1ull << 15) : 0ull) |
            (static_cast<uint64_t>(kGifFmtImage) << 58);
        std::memcpy(imagePacket.data(), &imageTag, sizeof(imageTag));
        std::memcpy(imagePacket.data() + 16u, src, static_cast<size_t>(qwords) * 16u);
        submitGifPacket(GifPathId::Path2,
                        imagePacket.data(),
                        static_cast<uint32_t>(imagePacket.size()),
                        true,
                        directHl);
    };

    // Feeds DIRECT payload quadwords to PATH2. The GIF keeps its place across
    // DIRECTs, so a payload first completes an IMAGE or PACKED/REGLIST tag
    // left open by the previous one, and an open tag at its end waits for
    // the next.
    auto feedPath2 = [&](const uint8_t *src, uint32_t qwCount, bool directHl)
    {
        uint32_t usedQw = 0u;
        if (m_vif1Path2GifImageOwedQwc != 0u)
        {
            usedQw = std::min<uint32_t>(m_vif1Path2GifImageOwedQwc, qwCount);
            m_vif1Path2GifImageOwedQwc -= usedQw;
            submitImageContinuation(src, usedQw, m_vif1Path2GifImageOwedQwc == 0u, directHl);
        }
        if (usedQw < qwCount && s_path2PartialOwedQwc != 0u)
        {
            const uint32_t takeQw = std::min<uint32_t>(s_path2PartialOwedQwc, qwCount - usedQw);
            s_path2Partial.insert(s_path2Partial.end(), src + usedQw * 16u, src + (usedQw + takeQw) * 16u);
            usedQw += takeQw;
            s_path2PartialOwedQwc -= takeQw;
            if (s_path2PartialOwedQwc == 0u)
            {
                submitGifPacket(GifPathId::Path2, s_path2Partial.data(), static_cast<uint32_t>(s_path2Partial.size()), true,
                                directHl);
                s_path2Partial.clear();
            }
        }
        if (usedQw < qwCount)
        {
            const uint8_t *packet = src + usedQw * 16u;
            const uint32_t packetBytes = (qwCount - usedQw) * 16u;
            uint32_t imageOwedQwc = 0u;
            uint32_t partialOwedQwc = 0u;
            const uint32_t completeBytes = splitGifPayload(packet, packetBytes, imageOwedQwc, partialOwedQwc);
            if (completeBytes != 0u)
                submitGifPacket(GifPathId::Path2, packet, completeBytes, true, directHl);
            m_vif1Path2GifImageOwedQwc = imageOwedQwc;
            if (partialOwedQwc != 0u)
            {
                s_path2Partial.assign(packet + completeBytes, packet + packetBytes);
                s_path2PartialOwedQwc = partialOwedQwc;
            }
        }
    };

    while (pos + 4 <= sizeBytes)
    {
        if (m_vif1PendingPath2ImageQwc != 0u)
        {
            const uint32_t availableQw = (sizeBytes - pos) / 16u;
            if (availableQw == 0u)
            {
                break;
            }

            const uint32_t chunkQw = std::min<uint32_t>(m_vif1PendingPath2ImageQwc, availableQw);
            feedPath2(data + pos, chunkQw, m_vif1PendingPath2DirectHl);

            pos += chunkQw * 16u;
            m_vif1PendingPath2ImageQwc -= chunkQw;
            if (m_vif1PendingPath2ImageQwc == 0u)
            {
                m_vif1PendingPath2DirectHl = false;
            }
            continue;
        }

        uint32_t cmd;
        memcpy(&cmd, data + pos, 4);
        pos += 4;

        uint8_t opcode = (cmd >> 24) & 0x7F;
        uint16_t imm = cmd & 0xFFFF;
        uint8_t num = (cmd >> 16) & 0xFF;
        const bool irq = (cmd & 0x80000000u) != 0u;

        // Track most-recent command for VIFn_CODE emulation.
        vif1_regs.code = cmd;
        vif1_regs.num = num;
        if (irq)
            vif1_regs.stat |= (1u << 11); // INT

        if (opcode == VIF_NOP)
        {
            continue;
        }
        else if (opcode == VIF_STCYCL)
        {
            vif1_regs.cycle = imm;
            continue;
        }
        else if (opcode == VIF_OFFSET)
        {
            // VIF double-buffer setup. OFFSET clears DBF and resets TOPS to BASE.
            // Do not rewrite BASE from the previous TOPS value.
            vif1_regs.ofst = imm & 0x3FFu;
            vif1_regs.tops = vif1_regs.base & 0x3FFu;
            vif1_regs.stat &= ~(1u << 7); // clear DBF
            continue;
        }
        else if (opcode == VIF_BASE)
        {
            // BASE only updates the base register. TOPS changes on OFFSET/MSCAL.
            vif1_regs.base = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_ITOP)
        {
            // ITOP VIFcode writes pending ITOPS; VU XITOP observes it after MSCAL/MSCNT.
            vif1_regs.itops = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_STMOD)
        {
            vif1_regs.mode = imm & 3u;
            continue;
        }
        else if (opcode == VIF_MSKPATH3)
        {
            // VIF command docs: MSKPATH3 uses IMMEDIATE bit 15.
            const bool wasMasked = m_path3Masked;
            m_path3Masked = (imm & 0x8000u) != 0u;
            if (wasMasked && !m_path3Masked)
                flushMaskedPath3Packets();
            continue;
        }
        else if (opcode == VIF_MARK)
        {
            vif1_regs.mark = imm;
            vif1_regs.stat |= (1u << 6); // MRK
            continue;
        }
        else if (opcode == VIF_FLUSHE || opcode == VIF_FLUSH || opcode == VIF_FLUSHA)
        {
            continue;
        }
        else if (opcode == VIF_MSCAL || opcode == VIF_MSCALF)
        {
            uint32_t startPC = (uint32_t)imm * 8u;

            // Values visible to the VU program for this MSCAL.
            // DobieStation semantics: ITOP = ITOPS; TOP = current TOPS;
            // then TOPS/DBF are prepared for the next buffer.
            const uint32_t runTop = vif1_regs.tops & 0x3FFu;
            const uint32_t runItop = vif1_regs.itops & 0x3FFu;
            vif1_regs.top = runTop;
            vif1_regs.itop = runItop;

            const bool dbf = (vif1_regs.stat & (1u << 7)) != 0u;
            if (dbf)
                vif1_regs.tops = vif1_regs.base & 0x3FFu;
            else
                vif1_regs.tops = (vif1_regs.base + vif1_regs.ofst) & 0x3FFu;
            vif1_regs.stat ^= (1u << 7); // toggle DBF

            static const char *traceCalls = std::getenv("PS2X_TRACE_UNPACK");
            if (traceCalls && g_ps2xVBlankCount.load() / 59.94 >= std::atof(traceCalls))
            {
                static std::unordered_set<uint32_t> seen;
                if (seen.insert((startPC << 12) | runItop).second)
                    std::fprintf(stderr, "[mscal] pc=%04x top=%x itop=%x\n", startPC, runTop, runItop);
            }
            if (m_vu1MscalCallback)
                m_vu1MscalCallback(startPC, runTop, runItop);
            continue;
        }
        else if (opcode == VIF_MSCNT)
        {
            const uint32_t runTop = vif1_regs.tops & 0x3FFu;
            const uint32_t runItop = vif1_regs.itops & 0x3FFu;
            vif1_regs.top = runTop;
            vif1_regs.itop = runItop;

            const bool dbf = (vif1_regs.stat & (1u << 7)) != 0u;
            if (dbf)
                vif1_regs.tops = vif1_regs.base & 0x3FFu;
            else
                vif1_regs.tops = (vif1_regs.base + vif1_regs.ofst) & 0x3FFu;
            vif1_regs.stat ^= (1u << 7); // toggle DBF

            static const char *traceCalls = std::getenv("PS2X_TRACE_UNPACK");
            if (traceCalls && g_ps2xVBlankCount.load() / 59.94 >= std::atof(traceCalls))
            {
                static std::unordered_set<uint32_t> seen;
                if (seen.insert(runItop).second)
                    std::fprintf(stderr, "[mscnt] top=%x itop=%x\n", runTop, runItop);
            }
            if (m_vu1MscntCallback)
                m_vu1MscntCallback(runTop, runItop);
            continue;
        }
        else if (opcode == VIF_STMASK)
        {
            if (pos + 4 > sizeBytes)
                break;
            uint32_t maskValue = 0;
            std::memcpy(&maskValue, data + pos, sizeof(maskValue));
            vif1_regs.mask = maskValue;
            pos += 4;
            continue;
        }
        else if (opcode == VIF_STROW)
        {
            if (pos + 16 > sizeBytes)
                break;
            std::memcpy(vif1_regs.row, data + pos, 16);
            pos += 16;
            continue;
        }
        else if (opcode == VIF_STCOL)
        {
            if (pos + 16 > sizeBytes)
                break;
            std::memcpy(vif1_regs.col, data + pos, 16);
            pos += 16;
            continue;
        }
        else if (opcode == VIF_MPG)
        {
            uint32_t destAddr = (uint32_t)imm * 8u;
            // VIF MPG semantics: NUM==0 means 256 instructions (2048 bytes).
            // MPG payload is instruction-packed and should not be QW-aligned.
            const uint32_t instructionCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
            const uint32_t mpgBytes = instructionCount * 8u;
            if (m_vu1Code && destAddr < PS2_VU1_CODE_SIZE && mpgBytes > 0)
            {
                uint32_t copyBytes = mpgBytes;
                if (destAddr + copyBytes > PS2_VU1_CODE_SIZE)
                    copyBytes = PS2_VU1_CODE_SIZE - destAddr;
                if (pos + copyBytes <= sizeBytes)
                {
                    std::memcpy(m_vu1Code + destAddr, data + pos, copyBytes);
                    markVU1CodeModified();
                }
            }
            pos += mpgBytes;
            if (pos > sizeBytes)
                break;
            continue;
        }
        else if (opcode == VIF_DIRECT || opcode == VIF_DIRECTHL)
        {
            uint32_t qwCount = imm;
            if (qwCount == 0)
                qwCount = 65536;
            const uint32_t availableQw = (sizeBytes - pos) / 16u;
            const bool truncated = qwCount > availableQw;
            const uint32_t splitQw = truncated ? qwCount - availableQw : 0u;
            if (qwCount > availableQw)
                qwCount = availableQw;

            if (qwCount > 0)
            {
                const bool directHl = (opcode == VIF_DIRECTHL);
                feedPath2(data + pos, qwCount, directHl);
                // The rest of a DIRECT cut off by the end of this transfer
                // arrives as raw data at the start of the next one.
                if (splitQw != 0u && (m_vif1Path2GifImageOwedQwc != 0u || s_path2PartialOwedQwc != 0u))
                {
                    m_vif1PendingPath2ImageQwc = std::min(m_vif1Path2GifImageOwedQwc + s_path2PartialOwedQwc, splitQw);
                    m_vif1PendingPath2DirectHl = directHl;
                }
            }

            pos += qwCount * 16;
            if (truncated)
            {
                pos = sizeBytes;
                break;
            }
            continue;
        }
        else if ((opcode & 0x60) == 0x60)
        {
            if (!vifUnpack(vif1_regs, m_vu1Data, PS2_VU1_DATA_SIZE, true, cmd, data, pos, sizeBytes))
                break;
            continue;
        }
        else
        {
            continue;
        }
    }
}
