#include "runtime/gs/gs_cpu_backend.h"
#include "runtime/gs/ps2_gs_common.h"
#include "runtime/gs/ps2_gs_psmct16.h"
#include "runtime/gs/ps2_gs_psmct32.h"
#include "runtime/gs/ps2_gs_psmt4.h"
#include "runtime/gs/ps2_gs_psmt8.h"
#include "runtime/gs/ps2_gs_memory.h"
#include "ps2_log.h"
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

using namespace GSInternal;

// VBlanks delivered so far, defined in EeScheduler.cpp.
extern std::atomic<uint64_t> g_ps2xVBlankCount;

namespace
{
    // Row band rasterized by the calling thread (see DrawBanded): rows are
    // dealt out in 8-row stripes; a zero mask means every row.
    thread_local uint32_t t_bandIndex = 0u;
    thread_local uint32_t t_bandMask = 0u;
    // The decoded texture DrawBanded was handed for the current batch.
    thread_local const void *t_runTexture = nullptr;
    thread_local bool t_useRunTexture = false;

    inline bool rowInBand(int y)
    {
        return ((static_cast<uint32_t>(y) >> 3) & t_bandMask) == t_bandIndex;
    }

    // Decoded CLUT entries, kept per raster thread like the GS's own CLUT
    // buffer. Entries are filled on first use and stay valid while the CLUT
    // state (address, formats, CSA/CSM, TEXCLUT, TEXA) is unchanged and no
    // transfer has written the VRAM pages the CLUT can occupy (per-page
    // epochs; g_vramEpoch covers whole-VRAM changes). A draw rendering into
    // CLUT memory does not invalidate them.
    std::atomic<uint32_t> g_vramEpoch{1};
    constexpr uint32_t kVramPageBytes = 8192u;
    constexpr uint32_t kVramPages = (4u * 1024u * 1024u) / kVramPageBytes;
    std::atomic<uint32_t> g_pageEpochs[kVramPages];

    // Conservative byte range of an image of `rows` rows in a buffer at
    // block `bp` that is `bw` x 64 pixels wide. Swizzled memory is laid out
    // in whole pages (32 rows for 32-bit formats, 64 for 16- and 8-bit, 128
    // for 4-bit), so rows are rounded up to a page row.
    inline void bufferRange(uint32_t bp, uint32_t bw, uint32_t rows, uint32_t bitsPerPixel, uint32_t &begin, uint32_t &end)
    {
        const uint32_t pageRows = bitsPerPixel >= 32u ? 32u : (bitsPerPixel >= 8u ? 64u : 128u);
        rows = (rows + pageRows - 1u) / pageRows * pageRows;
        const uint64_t base = static_cast<uint64_t>(bp) * 256u;
        const uint64_t bytes = static_cast<uint64_t>(std::max<uint32_t>(bw, 1u)) * 64u * rows * bitsPerPixel / 8u;
        begin = static_cast<uint32_t>(std::min<uint64_t>(base, 4u * 1024u * 1024u));
        end = static_cast<uint32_t>(std::min<uint64_t>(base + bytes + kVramPageBytes, 4u * 1024u * 1024u));
    }

    // Last writer of each page, for PS2X_GS_STATS: 1 draw, 2 banded run,
    // 3 upload, 4 local copy.
    std::atomic<uint8_t> g_pageTouchSource[kVramPages];
    std::map<std::string, uint32_t> g_redecodes; // GS command thread only

    // PS2X_TRACE_GS=<emulated seconds>: traces sprites and transfers from then on.
    bool traceGsNow()
    {
        static const char *env = std::getenv("PS2X_TRACE_GS");
        if (!env)
            return false;
        static const double start = std::atof(env);
        return static_cast<double>(g_ps2xVBlankCount.load(std::memory_order_relaxed)) / 59.94 >= start;
    }

    inline void touchVramRange(uint32_t begin, uint32_t end, uint8_t source = 0u)
    {
        for (uint32_t page = begin / kVramPageBytes; page < kVramPages && page * kVramPageBytes < end; ++page)
        {
            g_pageEpochs[page].fetch_add(1u, std::memory_order_acq_rel);
            g_pageTouchSource[page].store(source, std::memory_order_relaxed);
        }
    }

    inline uint32_t vramRangeEpoch(uint32_t begin, uint32_t end)
    {
        uint32_t sum = 0;
        for (uint32_t page = begin / kVramPageBytes; page < kVramPages && page * kVramPageBytes < end; ++page)
            sum += g_pageEpochs[page].load(std::memory_order_acquire);
        return sum;
    }

    inline uint32_t psmBits(uint8_t psm)
    {
        switch (psm)
        {
        case GS_PSM_CT16:
        case GS_PSM_CT16S:
        case GS_PSM_Z16:
        case GS_PSM_Z16S:
            return 16u;
        case GS_PSM_T8:
            return 8u;
        case GS_PSM_T4:
            return 4u;
        default:
            return 32u;
        }
    }

    // Byte span of GS memory a width x height texture at block tbp0 can
    // occupy: whole pages (page = (y / pageH) * pagesPerRow + x / pageW), or
    // for a texture inside one page, the blocks up to the highest one it uses.
    void textureSpan(uint32_t tbp0, uint32_t tbw, uint8_t psm, uint32_t width, uint32_t height,
                     uint32_t &begin, uint32_t &end)
    {
        // Block numbers within a page by block row (8 blocks across for the
        // 32-bit and 8-bit layouts, 4 across for 16-bit and 4-bit).
        static constexpr uint8_t kWide[4][8] = {{0, 1, 4, 5, 16, 17, 20, 21},
                                                {2, 3, 6, 7, 18, 19, 22, 23},
                                                {8, 9, 12, 13, 24, 25, 28, 29},
                                                {10, 11, 14, 15, 26, 27, 30, 31}};
        static constexpr uint8_t kTall[8][4] = {{0, 2, 8, 10}, {1, 3, 9, 11}, {4, 6, 12, 14}, {5, 7, 13, 15},
                                                {16, 18, 24, 26}, {17, 19, 25, 27}, {20, 22, 28, 30}, {21, 23, 29, 31}};
        uint32_t pageW = 64, pageH = 32, blockW = 8, blockH = 8;
        bool wide = true, knownLayout = true;
        switch (psm)
        {
        case GS_PSM_CT32:
        case GS_PSM_CT24:
        case GS_PSM_T8H:
        case GS_PSM_T4HL:
        case GS_PSM_T4HH:
            break;
        case GS_PSM_CT16:
            pageW = 64, pageH = 64, blockW = 16, blockH = 8, wide = false;
            break;
        case GS_PSM_T8:
            pageW = 128, pageH = 64, blockW = 16, blockH = 16;
            break;
        case GS_PSM_T4:
            pageW = 128, pageH = 128, blockW = 32, blockH = 16, wide = false;
            break;
        default: // CT16S and the Z formats: 64 x 64 pages, other block orders
            pageW = 64, pageH = psm == GS_PSM_Z32 || psm == GS_PSM_Z24 ? 32 : 64, knownLayout = false;
            break;
        }
        const uint32_t pagesPerRow = std::max<uint32_t>((std::max<uint32_t>(tbw, 1u) * 64u + pageW - 1u) / pageW, 1u);
        const uint32_t lastPage = (height - 1u) / pageH * pagesPerRow + (width - 1u) / pageW;
        uint64_t bytes = (static_cast<uint64_t>(lastPage) + 1u) * kVramPageBytes;
        if (knownLayout && width <= pageW && height <= pageH)
        {
            uint32_t lastBlock = 0;
            for (uint32_t by = 0; by * blockH < height; ++by)
                for (uint32_t bx = 0; bx * blockW < width; ++bx)
                    lastBlock = std::max<uint32_t>(lastBlock, wide ? kWide[by][bx] : kTall[by][bx]);
            bytes = (lastBlock + 1u) * 256u;
        }
        const uint64_t base = static_cast<uint64_t>(tbp0) * 256u;
        begin = static_cast<uint32_t>(std::min<uint64_t>(base, 4u * 1024u * 1024u));
        end = static_cast<uint32_t>(std::min<uint64_t>(base + bytes, 4u * 1024u * 1024u));
    }

    uint64_t hashBytes(const uint8_t *data, uint32_t size, uint64_t seed)
    {
        constexpr uint64_t kMul = 0x9E3779B97F4A7C15ull;
        auto mix = [](uint64_t h, uint64_t w)
        {
            h ^= w;
            h = (h << 29) | (h >> 35);
            return h * kMul;
        };
        uint64_t lanes[4] = {seed, seed ^ 1u, seed ^ 2u, seed ^ 3u};
        uint32_t offset = 0;
        for (; offset + 32u <= size; offset += 32u)
        {
            uint64_t words[4];
            std::memcpy(words, data + offset, sizeof(words));
            for (int i = 0; i < 4; ++i)
                lanes[i] = mix(lanes[i], words[i]);
        }
        uint64_t h = mix(mix(lanes[0], lanes[1]), mix(lanes[2], lanes[3]));
        for (; offset < size; ++offset)
            h = mix(h, data[offset]);
        return mix(h, size);
    }

    struct ClutCacheEntry
    {
        uint32_t generation = 0;
        uint32_t value = 0;
    };

    struct ClutCacheKey
    {
        uint32_t epoch = 0;
        uint32_t cbp = 0;
        uint32_t clut = 0; // cpsm | csm | csa | cbw | cou
        uint32_t cov = 0;
        uint32_t texa = 0; // ta0 | ta1 | aem

        bool operator==(const ClutCacheKey &other) const
        {
            return epoch == other.epoch && cbp == other.cbp && clut == other.clut && cov == other.cov && texa == other.texa;
        }
    };

    thread_local ClutCacheEntry t_clutCache[512];
    thread_local uint32_t t_clutGeneration = 1;
    thread_local ClutCacheKey t_clutKey{};

    inline void beginPrimitiveClutCache(const GSDrawState &state)
    {
        if (!state.prim.tme)
            return;
        const GSTex0Reg &tex = state.context.tex0;
        uint32_t clutBegin = 0, clutEnd = 0;
        bufferRange(tex.cbp, state.texclut.cbw, static_cast<uint32_t>(state.texclut.cov) + 32u, 32u, clutBegin, clutEnd);
        ClutCacheKey key;
        key.epoch = g_vramEpoch.load(std::memory_order_acquire) + vramRangeEpoch(clutBegin, clutEnd);
        key.cbp = tex.cbp;
        key.clut = static_cast<uint32_t>(tex.cpsm) | (static_cast<uint32_t>(tex.csm) << 6) |
                   (static_cast<uint32_t>(tex.csa) << 8) | (static_cast<uint32_t>(state.texclut.cbw) << 16) |
                   (static_cast<uint32_t>(state.texclut.cou) << 24);
        key.cov = state.texclut.cov;
        key.texa = static_cast<uint32_t>(state.texa.ta0) | (static_cast<uint32_t>(state.texa.ta1) << 8) |
                   (state.texa.aem ? 0x10000u : 0u);
        if (key == t_clutKey)
            return;
        t_clutKey = key;
        if (++t_clutGeneration == 0u)
        {
            for (ClutCacheEntry &entry : t_clutCache)
                entry.generation = 0;
            t_clutGeneration = 1;
        }
    }

    inline void invalidateClutCaches()
    {
        g_vramEpoch.fetch_add(1u, std::memory_order_acq_rel);
    }

    float fabsQ(float q)
    {
        return (std::fabs(q) > 1.0e-8f) ? q : 1.0f;
    }

    u16 Rgba8888ToRgba5551(u32 c)
    {
        uint32_t r = ((c >> 0) & 0xFF) >> 3;
        uint32_t g = ((c >> 8) & 0xFF) >> 3;
        uint32_t b = ((c >> 16) & 0xFF) >> 3;
        uint32_t a = ((c >> 24) & 0xFF) >> 7;

        return (r | (g << 5) | (b << 10) | (a << 15));
    }

    u32 Rgba5551ToRgba8888(u16 c)
    {
        u32 r = ((c >> 0) & 0x1F) << 3;
        u32 g = ((c >> 5) & 0x1F) << 3;
        u32 b = ((c >> 10) & 0x1F) << 3;
        u32 a = ((c >> 15) & 0x01) << 7;

        return (r | (g << 8) | (b << 16) | (a << 24));
    }

    u32 pack32(u8 r, u8 g, u8 b, u8 a)
    {
        return static_cast<u32>(r) | (g << 8) | (b << 16) | (a << 24);
    }

    uint32_t applyTexa(const GSTexaReg &texa, uint8_t psm, uint32_t texel)
    {
        if (psm == GS_PSM_CT32)
            return texel;

        const uint8_t r = static_cast<uint8_t>(texel & 0xFFu);
        const uint8_t g = static_cast<uint8_t>((texel >> 8) & 0xFFu);
        const uint8_t b = static_cast<uint8_t>((texel >> 16) & 0xFFu);
        const bool rgbZero = r == 0u && g == 0u && b == 0u;
        uint8_t a = static_cast<uint8_t>((texel >> 24) & 0xFFu);

        switch (psm)
        {
        case GS_PSM_CT24:
            a = (texa.aem && rgbZero) ? 0u : texa.ta0;
            break;
        case GS_PSM_CT16:
        case GS_PSM_CT16S:
            if ((a & 0x80u) != 0u)
                a = texa.ta1;
            else
                a = (texa.aem && rgbZero) ? 0u : texa.ta0;
            break;
        default:
            break;
        }

        return (texel & 0x00FFFFFFu) | (static_cast<uint32_t>(a) << 24);
    }

    uint32_t addrPSMCT16Family(uint32_t basePtr, uint32_t width, uint8_t psm, uint32_t x, uint32_t y)
    {
        switch (psm)
        {
        case GS_PSM_CT16:
            return GSPSMCT16::addrPSMCT16(basePtr, width, x, y);
        case GS_PSM_CT16S:
            return GSPSMCT16::addrPSMCT16S(basePtr, width, x, y);
        case GS_PSM_Z16:
            return GSPSMCT16::addrPSMZ16(basePtr, width, x, y);
        case GS_PSM_Z16S:
            return GSPSMCT16::addrPSMZ16S(basePtr, width, x, y);
        default:
            return 0u;
        }
    }

    std::atomic<uint32_t> s_debugPrimitiveCount{0};
    std::atomic<uint32_t> s_debugPixelCount{0};
    std::atomic<uint32_t> s_debugContext1PrimitiveCount{0};
    std::atomic<uint32_t> s_debugFbp150PixelCount{0};

    int wrapTextureCoordinate(int coordinate,
                              int textureSize,
                              uint8_t mode,
                              uint16_t regionMin,
                              uint16_t regionMax)
    {
        switch (mode & 0x3u)
        {
        case 0: // REPEAT
            return static_cast<int>(static_cast<uint32_t>(coordinate) & static_cast<uint32_t>(textureSize - 1));
        case 1: // CLAMP
            return clampInt(coordinate, 0, textureSize - 1);
        case 2: // REGION_CLAMP
            return std::min(std::max(coordinate, static_cast<int>(regionMin)), static_cast<int>(regionMax));
        case 3: // REGION_REPEAT
            return static_cast<int>((static_cast<uint32_t>(coordinate) & static_cast<uint32_t>(regionMin)) | static_cast<uint32_t>(regionMax));
        default:
            return coordinate;
        }
    }

    bool passesAlphaTest(uint64_t testReg, uint8_t alpha)
    {
        if ((testReg & 0x1u) == 0u)
            return true;

        const uint8_t atst = static_cast<uint8_t>((testReg >> 1) & 0x7u);
        const uint8_t aref = static_cast<uint8_t>((testReg >> 4) & 0xFFu);

        switch (atst)
        {
        case 0:
            return false;
        case 1:
            return true;
        case 2:
            return alpha < aref;
        case 3:
            return alpha <= aref;
        case 4:
            return alpha == aref;
        case 5:
            return alpha >= aref;
        case 6:
            return alpha > aref;
        case 7:
            return alpha != aref;
        default:
            return true;
        }
    }

    struct PixelWriteMask
    {
        bool writeRgb = true;
        bool writeAlpha = true;
        bool writeDepth = true;

        bool writesFramebuffer() const
        {
            return writeRgb || writeAlpha;
        }

        bool writesAnything() const
        {
            return writesFramebuffer() || writeDepth;
        }
    };

    PixelWriteMask classifyAlphaTest(uint64_t testReg, uint8_t alpha, uint8_t framePsm)
    {
        const bool pass = passesAlphaTest(testReg, alpha);
        if (pass)
            return {};

        // TEST.AFAIL controls what happens when the alpha comparison fails.
        switch (static_cast<uint8_t>((testReg >> 12) & 0x3u))
        {
        case 1: // FB_ONLY
            return {true, true, false};
        case 2: // ZB_ONLY
            return {false, false, true};
        case 3: // RGB_ONLY
            // RGB_ONLY is only distinct for RGBA32. The GS treats it as
            // FB_ONLY for RGB24 and RGBA16 framebuffers.
            if (framePsm == GS_PSM_CT32)
                return {true, false, false};
            return {true, true, false};
        case 0: // KEEP
        default:
            return {false, false, false};
        }
    }

    bool passesDestinationAlphaTest(uint64_t testReg, uint8_t framePsm, uint32_t rawFramebufferPixel)
    {
        const bool date = ((testReg >> 14) & 0x1u) != 0u;
        if (!date)
            return true;

        const bool datm = ((testReg >> 15) & 0x1u) != 0u;
        switch (framePsm)
        {
        case GS_PSM_CT32:
            return (((rawFramebufferPixel >> 31) & 0x1u) != 0u) == datm;
        case GS_PSM_CT16:
        case GS_PSM_CT16S:
            return (((rawFramebufferPixel >> 15) & 0x1u) != 0u) == datm;
        case GS_PSM_CT24:
            // RGB24 has no destination alpha, so DATE always passes.
            return true;
        default:
            return true;
        }
    }

    struct TextureCombineResult
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    };

    TextureCombineResult combineTexture(const GSTex0Reg &tex,
                                        uint8_t vr,
                                        uint8_t vg,
                                        uint8_t vb,
                                        uint8_t va,
                                        uint8_t tr,
                                        uint8_t tg,
                                        uint8_t tb,
                                        uint8_t ta)
    {
        const bool textureHasAlpha = tex.tcc != 0u;
        TextureCombineResult out{tr, tg, tb, textureHasAlpha ? ta : va};

        switch (tex.tfx)
        {
        case 0: // MODULATE
            out.r = clampU8((tr * vr) >> 7);
            out.g = clampU8((tg * vg) >> 7);
            out.b = clampU8((tb * vb) >> 7);
            out.a = textureHasAlpha ? clampU8((ta * va) >> 7) : va;
            break;
        case 1: // DECAL
            out.r = tr;
            out.g = tg;
            out.b = tb;
            out.a = textureHasAlpha ? ta : va;
            break;
        case 2: // HIGHLIGHT
            out.r = clampU8(((tr * vr) >> 7) + va);
            out.g = clampU8(((tg * vg) >> 7) + va);
            out.b = clampU8(((tb * vb) >> 7) + va);
            out.a = textureHasAlpha ? clampU8(ta + va) : va;
            break;
        case 3: // HIGHLIGHT2
            out.r = clampU8(((tr * vr) >> 7) + va);
            out.g = clampU8(((tg * vg) >> 7) + va);
            out.b = clampU8(((tb * vb) >> 7) + va);
            out.a = textureHasAlpha ? ta : va;
            break;
        default:
            out.r = tr;
            out.g = tg;
            out.b = tb;
            out.a = textureHasAlpha ? ta : va;
            break;
        }

        return out;
    }

    uint32_t swizzleClutIndexCSM1(uint32_t index)
    {
        // CSM1 swaps address bits 3 and 4. Preserve the remaining bits:
        // 16-bit CLUTs expose a ninth address bit through CSA[4].
        return (index & ~0x18u) | ((index & 0x08u) << 1u) | ((index & 0x10u) >> 1u);
    }

    // TODO: clut cache
    uint32_t resolveClutIndex(uint8_t index, uint8_t cpsm, uint8_t csm, uint8_t csa, uint8_t sourcePsm)
    {
        uint32_t clutIndex = static_cast<uint32_t>(index);

        // CSM2 addresses the source directly through TEXCLUT. CSA is required
        // to be zero there, so it must not offset the source coordinates.
        if (csm != 0u)
            return (sourcePsm == GS_PSM_T4 ||
                    sourcePsm == GS_PSM_T4HH ||
                    sourcePsm == GS_PSM_T4HL)
                       ? (clutIndex & 0x0Fu)
                       : clutIndex;

        const bool is16BitClut = cpsm == GS_PSM_CT16 || cpsm == GS_PSM_CT16S;
        const uint32_t csaMask = is16BitClut ? 0x1Fu : 0x0Fu;
        const uint32_t clutIndexMask = is16BitClut ? 0x1FFu : 0x0FFu;
        const uint32_t clutBase = (static_cast<uint32_t>(csa) & csaMask) << 4u;

        switch (sourcePsm)
        {
        case GS_PSM_T4:
        case GS_PSM_T4HH:
        case GS_PSM_T4HL:
            clutIndex = clutBase + (clutIndex & 0x0Fu);
            break;
        case GS_PSM_T8:
        case GS_PSM_T8H:
            clutIndex = clutBase + clutIndex;
            break;
        default:
            return clutIndex;
        }

        return swizzleClutIndexCSM1(clutIndex & clutIndexMask);
    }

    uint8_t lerpChannel(uint8_t c00, uint8_t c10, uint8_t c01, uint8_t c11, float fx, float fy)
    {
        const float top = static_cast<float>(c00) + (static_cast<float>(c10) - static_cast<float>(c00)) * fx;
        const float bottom = static_cast<float>(c01) + (static_cast<float>(c11) - static_cast<float>(c01)) * fx;
        // The blend of 0..255 values is never negative: +0.5 and truncation
        // rounds like std::lround without the libm call.
        return clampU8(static_cast<int>(top + (bottom - top) * fy + 0.5f));
    }

    // ---- Fast triangle path -------------------------------------------------
    // Pixel offsets inside one page for the usual frame and Z buffer formats
    // (frame and Z buffers always start on a page). They are measured from
    // the GSMem writers, so they follow the reference swizzle exactly.
    struct PageSwizzle
    {
        uint16_t c32[32][64]; // PSMCT32/24
        uint16_t z32[32][64]; // PSMZ32/24
        uint16_t z16[64][64];
        uint16_t z16s[64][64];
    };

    const PageSwizzle &pageSwizzle()
    {
        static const PageSwizzle tables = []()
        {
            PageSwizzle t{};
            std::vector<uint8_t> scratch(4u * 1024u * 1024u, 0u);
            auto probe32 = [&](void (*write)(u8 *, u32, u32, u32, u32, u32), uint16_t (&out)[32][64])
            {
                std::fill(scratch.begin(), scratch.begin() + 8192, 0u);
                for (uint32_t y = 0; y < 32u; ++y)
                    for (uint32_t x = 0; x < 64u; ++x)
                        write(scratch.data(), 0u, 1u, x, y, y * 64u + x + 1u);
                for (uint32_t word = 0; word < 2048u; ++word)
                {
                    uint32_t tag;
                    std::memcpy(&tag, scratch.data() + word * 4u, sizeof(tag));
                    if (tag != 0u && tag <= 2048u)
                        out[(tag - 1u) >> 6][(tag - 1u) & 63u] = static_cast<uint16_t>(word);
                }
            };
            auto probe16 = [&](void (*write)(u8 *, u32, u32, u32, u32, u32), uint16_t (&out)[64][64])
            {
                std::fill(scratch.begin(), scratch.begin() + 8192, 0u);
                for (uint32_t y = 0; y < 64u; ++y)
                    for (uint32_t x = 0; x < 64u; ++x)
                        write(scratch.data(), 0u, 1u, x, y, y * 64u + x + 1u);
                for (uint32_t half = 0; half < 4096u; ++half)
                {
                    uint16_t tag;
                    std::memcpy(&tag, scratch.data() + half * 2u, sizeof(tag));
                    if (tag != 0u && tag <= 4096u)
                        out[(tag - 1u) >> 6][(tag - 1u) & 63u] = static_cast<uint16_t>(half);
                }
            };
            probe32(GSMem::WriteCT32, t.c32);
            probe32(GSMem::WriteZ32, t.z32);
            probe16(GSMem::WriteZ16, t.z16);
            probe16(GSMem::WriteZ16S, t.z16s);
            return t;
        }();
        return tables;
    }

    inline uint32_t byteAddress32(const uint16_t (&page)[32][64], uint32_t basePage, uint32_t bw, uint32_t x, uint32_t y)
    {
        return (((basePage + (y >> 5) * bw + (x >> 6)) * 2048u + page[y & 31u][x & 63u]) * 4u) & 0x3FFFFCu;
    }

    inline uint32_t byteAddress16(const uint16_t (&page)[64][64], uint32_t basePage, uint32_t bw, uint32_t x, uint32_t y)
    {
        return (((basePage + (y >> 6) * bw + (x >> 6)) * 4096u + page[y & 63u][x & 63u]) * 2u) & 0x3FFFFEu;
    }

    // Draws a triangle whose frame is PSMCT32/24 and Z buffer PSMZ32/24/16,
    // with any texture already decoded; returns false (drawing nothing) for
    // anything else. Per pixel it does what DrawTriangle + SampleTexture +
    // WritePixel do, with the draw state decoded once and the buffers
    // addressed through page tables instead of per-pixel swizzle calls.
    bool drawTriangleFast(uint8_t *vram, const GSPrimitiveBatch &batch, const GSCpuBackend::DecodedTexture *decoded)
    {
        const GSDrawState &state = batch.state;
        const auto &ctx = state.context;
        const uint32_t fpsm = ctx.frame.psm;
        const uint32_t zpsm = ctx.zbuf.psm;
        if (!vram || (fpsm != GS_PSM_CT32 && fpsm != GS_PSM_CT24))
            return false;
        if (zpsm != GS_PSM_Z32 && zpsm != GS_PSM_Z24 && zpsm != GS_PSM_Z16 && zpsm != GS_PSM_Z16S)
            return false;

        const bool tme = state.prim.tme;
        const int texW = state.textureWidth;
        const int texH = state.textureHeight;
        const uint64_t clamp = ctx.clamp;
        const uint8_t wrapU = static_cast<uint8_t>(clamp & 0x3u);
        const uint8_t wrapV = static_cast<uint8_t>((clamp >> 2) & 0x3u);
        const uint16_t minU = static_cast<uint16_t>((clamp >> 4) & 0x3FFu);
        const uint16_t maxU = static_cast<uint16_t>((clamp >> 14) & 0x3FFu);
        const uint16_t minV = static_cast<uint16_t>((clamp >> 24) & 0x3FFu);
        const uint16_t maxV = static_cast<uint16_t>((clamp >> 34) & 0x3FFu);
        if (tme)
        {
            // Every texel the wrap modes can produce must be in the decoded copy.
            auto reach = [](uint8_t mode, int size, uint16_t lo, uint16_t hi) -> int
            {
                switch (mode)
                {
                case 2:
                    return std::max<int>(hi, lo) + 1;
                case 3:
                    return static_cast<int>(lo | hi) + 1;
                default:
                    return size;
                }
            };
            if (!decoded || static_cast<int>(decoded->width) < reach(wrapU, texW, minU, maxU) ||
                static_cast<int>(decoded->height) < reach(wrapV, texH, minV, maxV))
                return false;
        }

        const GSVertex &v0 = batch.vertices[0];
        const GSVertex &v1 = batch.vertices[1];
        const GSVertex &v2 = batch.vertices[2];
        const int ofx = ctx.xyoffset.ofx >> 4;
        const int ofy = ctx.xyoffset.ofy >> 4;
        const float fx0 = v0.x - static_cast<float>(ofx);
        const float fy0 = v0.y - static_cast<float>(ofy);
        const float fx1 = v1.x - static_cast<float>(ofx);
        const float fy1 = v1.y - static_cast<float>(ofy);
        const float fx2 = v2.x - static_cast<float>(ofx);
        const float fy2 = v2.y - static_cast<float>(ofy);

        int minX = static_cast<int>(std::floor(std::min({fx0, fx1, fx2})));
        int maxX = static_cast<int>(std::ceil(std::max({fx0, fx1, fx2})));
        int minY = static_cast<int>(std::floor(std::min({fy0, fy1, fy2})));
        int maxY = static_cast<int>(std::ceil(std::max({fy0, fy1, fy2})));
        minX = clampInt(minX, ctx.scissor.x0, ctx.scissor.x1);
        maxX = clampInt(maxX, ctx.scissor.x0, ctx.scissor.x1);
        minY = clampInt(minY, ctx.scissor.y0, ctx.scissor.y1);
        maxY = clampInt(maxY, ctx.scissor.y0, ctx.scissor.y1);

        const float denom = (fy1 - fy2) * (fx0 - fx2) + (fx2 - fx1) * (fy0 - fy2);
        if (std::fabs(denom) < 0.001f)
            return true;
        const float winding = (denom < 0.0f) ? -1.0f : 1.0f;
        const float invAbsDenom = 1.0f / std::fabs(denom);
        constexpr float kEdgeEpsilon = 1.0e-4f;
        const float scale = winding * invAbsDenom;
        const float a0 = (fy1 - fy2) * scale;
        const float a1 = (fy2 - fy0) * scale;
        const float a2 = -(a0 + a1);

        // Draw state, decoded once.
        const uint64_t test = ctx.test;
        const uint32_t ztst = static_cast<uint32_t>((test >> 17) & 3u);
        if (ztst == 0u)
            return true; // NEVER: nothing is written
        const bool ate = (test & 1u) != 0u;
        const uint32_t afail = static_cast<uint32_t>((test >> 12) & 3u);
        const bool date = ((test >> 14) & 1u) != 0u;
        const bool datm = ((test >> 15) & 1u) != 0u;
        const bool abe = state.prim.abe;
        const uint64_t alphaReg = ctx.alpha;
        const uint8_t asel = alphaReg & 3, bsel = (alphaReg >> 2) & 3, csel = (alphaReg >> 4) & 3, dsel = (alphaReg >> 6) & 3;
        const uint8_t fix = static_cast<uint8_t>((alphaReg >> 32) & 0xFF);
        const uint32_t fbmsk = ctx.frame.fbmsk;
        const bool fba = (ctx.fba & 1u) != 0u && fpsm != GS_PSM_CT24;
        const bool zmask = ctx.zbuf.zmask;
        const uint32_t fbw = std::max<uint32_t>(ctx.frame.fbw, 1u);
        const uint32_t fbp = ctx.frame.fbp;
        const uint32_t zbp = ctx.zbuf.zbp;
        const uint32_t zMax = zpsm == GS_PSM_Z32 ? 0xFFFFFFFFu : zpsm == GS_PSM_Z24 ? 0xFFFFFFu : 0xFFFFu;
        const bool z16 = zpsm == GS_PSM_Z16 || zpsm == GS_PSM_Z16S;
        const PageSwizzle &swizzle = pageSwizzle();
        const uint16_t(&zPage32)[32][64] = swizzle.z32;
        const uint16_t(&zPage16)[64][64] = zpsm == GS_PSM_Z16S ? swizzle.z16s : swizzle.z16;
        const bool fge = state.prim.fge;
        const bool iip = state.prim.iip;
        const bool fst = state.prim.fst;
        const bool linear = state.linearFilter;
        const auto &tex = ctx.tex0;
        const uint32_t *texels = tme ? decoded->texels.data() : nullptr;
        const uint32_t texStride = tme ? decoded->width : 0u;

        // Attributes as v2 + (v0 - v2) * w0 + (v1 - v2) * w1.
        const double z2 = v2.z, dz0 = v0.z - v2.z, dz1 = v1.z - v2.z;
        const float c2r = v2.r, c2g = v2.g, c2b = v2.b, c2a = v2.a;
        const float dr0 = float(v0.r) - v2.r, dr1 = float(v1.r) - v2.r;
        const float dg0 = float(v0.g) - v2.g, dg1 = float(v1.g) - v2.g;
        const float db0 = float(v0.b) - v2.b, db1 = float(v1.b) - v2.b;
        const float da0 = float(v0.a) - v2.a, da1 = float(v1.a) - v2.a;
        const float f2 = v2.fog, df0 = float(v0.fog) - v2.fog, df1 = float(v1.fog) - v2.fog;
        const float s2 = v2.s, ds0 = v0.s - v2.s, ds1 = v1.s - v2.s;
        const float t2 = v2.t, dt0 = v0.t - v2.t, dt1 = v1.t - v2.t;
        const float q2 = v2.q, dq0 = v0.q - v2.q, dq1 = v1.q - v2.q;
        const float u2 = v2.u, du0 = float(v0.u) - v2.u, du1 = float(v1.u) - v2.u;
        const float vv2 = v2.v, dv0 = float(v0.v) - v2.v, dv1 = float(v1.v) - v2.v;

        auto texelAt = [&](int u, int v) -> uint32_t
        {
            u = wrapTextureCoordinate(u, texW, wrapU, minU, maxU);
            v = wrapTextureCoordinate(v, texH, wrapV, minV, maxV);
            return texels[static_cast<size_t>(v) * texStride + static_cast<uint32_t>(u)];
        };

        for (int y = minY; y <= maxY; ++y)
        {
            if (!rowInBand(y))
                continue;
            const float py = static_cast<float>(y) + 0.5f;
            const float c0 = ((fx2 - fx1) * (py - fy2)) * scale - a0 * fx2;
            const float c1 = ((fx0 - fx2) * (py - fy2)) * scale - a1 * fx2;
            const float c2 = 1.0f - c0 - c1;
            float spanMin = static_cast<float>(minX) + 0.5f;
            float spanMax = static_cast<float>(maxX) + 0.5f;
            bool emptyRow = false;
            auto clipSpan = [&](float a, float c)
            {
                if (std::fabs(a) < 1.0e-12f)
                {
                    emptyRow |= c < -kEdgeEpsilon;
                    return;
                }
                const float bound = (-kEdgeEpsilon - c) / a;
                if (a > 0.0f)
                    spanMin = std::max(spanMin, bound);
                else
                    spanMax = std::min(spanMax, bound);
            };
            clipSpan(a0, c0);
            clipSpan(a1, c1);
            clipSpan(a2, c2);
            if (emptyRow || !(spanMin <= spanMax + 2.0f))
                continue;
            const int rowMinX = std::max(minX, static_cast<int>(std::floor(spanMin - 0.5f)) - 1);
            const int rowMaxX = std::min(maxX, static_cast<int>(std::ceil(spanMax - 0.5f)) + 1);
            const uint32_t uy = static_cast<uint32_t>(y);

            for (int x = rowMinX; x <= rowMaxX; ++x)
            {
                const float px = static_cast<float>(x) + 0.5f;
                const float w0 = a0 * px + c0;
                const float w1 = a1 * px + c1;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < -kEdgeEpsilon || w1 < -kEdgeEpsilon || w2 < -kEdgeEpsilon)
                    continue;
                const uint32_t ux = static_cast<uint32_t>(x);

                // Depth test first: most rejected pixels stop here.
                uint32_t z = static_cast<uint32_t>(z2 + dz0 * w0 + dz1 * w1 + 0.5);
                if (z > zMax)
                    z = zMax;
                uint8_t *zPtr = z16 ? vram + byteAddress16(zPage16, zbp, fbw, ux, uy)
                                    : vram + byteAddress32(zPage32, zbp, fbw, ux, uy);
                if (ztst >= 2u)
                {
                    uint32_t storedZ;
                    if (z16)
                    {
                        uint16_t raw;
                        std::memcpy(&raw, zPtr, sizeof(raw));
                        storedZ = raw;
                    }
                    else
                    {
                        std::memcpy(&storedZ, zPtr, sizeof(storedZ));
                        storedZ &= zMax;
                    }
                    if (ztst == 2u ? z < storedZ : z <= storedZ)
                        continue;
                }

                int r, g, b, a;
                if (iip)
                {
                    r = clampU8(static_cast<int>(c2r + dr0 * w0 + dr1 * w1));
                    g = clampU8(static_cast<int>(c2g + dg0 * w0 + dg1 * w1));
                    b = clampU8(static_cast<int>(c2b + db0 * w0 + db1 * w1));
                    a = clampU8(static_cast<int>(c2a + da0 * w0 + da1 * w1));
                }
                else
                {
                    r = v2.r;
                    g = v2.g;
                    b = v2.b;
                    a = v2.a;
                }

                if (tme)
                {
                    float texUf, texVf;
                    if (fst)
                    {
                        texUf = static_cast<float>(static_cast<uint16_t>(u2 + du0 * w0 + du1 * w1)) / 16.0f;
                        texVf = static_cast<float>(static_cast<uint16_t>(vv2 + dv0 * w0 + dv1 * w1)) / 16.0f;
                    }
                    else
                    {
                        const float invQ = 1.0f / fabsQ(q2 + dq0 * w0 + dq1 * w1);
                        texUf = (s2 + ds0 * w0 + ds1 * w1) * invQ * static_cast<float>(texW);
                        texVf = (t2 + dt0 * w0 + dt1 * w1) * invQ * static_cast<float>(texH);
                    }
                    uint32_t texel;
                    if (!linear)
                    {
                        texel = texelAt(static_cast<int>(texUf), static_cast<int>(texVf));
                    }
                    else
                    {
                        const float su = texUf - 0.5f;
                        const float sv = texVf - 0.5f;
                        const int iu0 = static_cast<int>(std::floor(su));
                        const int iv0 = static_cast<int>(std::floor(sv));
                        const float fx = su - static_cast<float>(iu0);
                        const float fy = sv - static_cast<float>(iv0);
                        const uint32_t c00 = texelAt(iu0, iv0), c10 = texelAt(iu0 + 1, iv0);
                        const uint32_t c01 = texelAt(iu0, iv0 + 1), c11 = texelAt(iu0 + 1, iv0 + 1);
                        texel = 0u;
                        for (uint32_t shift = 0; shift < 32u; shift += 8u)
                            texel |= static_cast<uint32_t>(lerpChannel(static_cast<uint8_t>(c00 >> shift), static_cast<uint8_t>(c10 >> shift),
                                                                       static_cast<uint8_t>(c01 >> shift), static_cast<uint8_t>(c11 >> shift), fx, fy))
                                     << shift;
                    }
                    const TextureCombineResult color =
                        combineTexture(tex, static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), static_cast<uint8_t>(a),
                                       static_cast<uint8_t>(texel), static_cast<uint8_t>(texel >> 8), static_cast<uint8_t>(texel >> 16),
                                       static_cast<uint8_t>(texel >> 24));
                    r = color.r;
                    g = color.g;
                    b = color.b;
                    a = color.a;
                }

                if (fge)
                {
                    const uint32_t fog = clampU8(static_cast<int>(f2 + df0 * w0 + df1 * w1));
                    const uint32_t inverseFog = 255u - fog;
                    r = static_cast<uint8_t>(((fog * r) >> 8) + ((inverseFog * state.fogR) >> 8));
                    g = static_cast<uint8_t>(((fog * g) >> 8) + ((inverseFog * state.fogG) >> 8));
                    b = static_cast<uint8_t>(((fog * b) >> 8) + ((inverseFog * state.fogB) >> 8));
                }

                // Alpha test (TEST.AFAIL decides what a failing pixel still writes).
                bool writeRgb = true, writeAlpha = true, writeDepth = true;
                if (ate && !passesAlphaTest(test, static_cast<uint8_t>(a)))
                {
                    switch (afail)
                    {
                    case 1:
                        writeDepth = false;
                        break;
                    case 2:
                        writeRgb = writeAlpha = false;
                        break;
                    case 3:
                        writeDepth = false;
                        writeAlpha = fpsm != GS_PSM_CT32;
                        break;
                    default:
                        continue;
                    }
                }

                uint8_t *fbPtr = vram + byteAddress32(swizzle.c32, fbp, fbw, ux, uy);
                const bool preserveDestinationAlpha = writeRgb && !writeAlpha && fpsm == GS_PSM_CT32;
                uint32_t fbrgba = 0u;
                if (date || (writeRgb && (fbmsk != 0u || abe || preserveDestinationAlpha)))
                {
                    std::memcpy(&fbrgba, fbPtr, sizeof(fbrgba));
                    if (date && fpsm == GS_PSM_CT32 && ((fbrgba >> 31) != 0u) != datm)
                        continue;
                    if (fpsm == GS_PSM_CT24)
                        fbrgba = (fbrgba & 0x00FFFFFFu) | 0x80000000u;
                }

                if (writeRgb)
                {
                    if (abe && !(state.pabe && (a & 0x80) == 0))
                    {
                        const int dr = fbrgba & 0xFF, dg = (fbrgba >> 8) & 0xFF, db = (fbrgba >> 16) & 0xFF, da = (fbrgba >> 24) & 0xFF;
                        const int cAlpha = csel == 0 ? a : csel == 1 ? da : fix;
                        auto pick = [](uint8_t sel, int cs, int cd) { return sel == 0 ? cs : sel == 1 ? cd : 0; };
                        r = clampU8(((pick(asel, r, dr) - pick(bsel, r, dr)) * cAlpha >> 7) + pick(dsel, r, dr));
                        g = clampU8(((pick(asel, g, dg) - pick(bsel, g, dg)) * cAlpha >> 7) + pick(dsel, g, dg));
                        b = clampU8(((pick(asel, b, db) - pick(bsel, b, db)) * cAlpha >> 7) + pick(dsel, b, db));
                    }
                    if (writeAlpha && fba)
                        a |= 0x80;
                    uint32_t pixel = pack32(static_cast<u8>(r), static_cast<u8>(g), static_cast<u8>(b), static_cast<u8>(a));
                    if (fbmsk != 0u)
                        pixel = (pixel & ~fbmsk) | (fbrgba & fbmsk);
                    if (preserveDestinationAlpha)
                        pixel = (pixel & 0x00FFFFFFu) | (fbrgba & 0xFF000000u);
                    if (fpsm == GS_PSM_CT24)
                    {
                        uint32_t old;
                        std::memcpy(&old, fbPtr, sizeof(old));
                        pixel = (old & 0xFF000000u) | (pixel & 0x00FFFFFFu);
                    }
                    std::memcpy(fbPtr, &pixel, sizeof(pixel));
                }

                if (writeDepth && !zmask)
                {
                    if (z16)
                    {
                        const uint16_t value = static_cast<uint16_t>(z);
                        std::memcpy(zPtr, &value, sizeof(value));
                    }
                    else if (zpsm == GS_PSM_Z24)
                    {
                        uint32_t old;
                        std::memcpy(&old, zPtr, sizeof(old));
                        const uint32_t value = (old & 0xFF000000u) | (z & 0x00FFFFFFu);
                        std::memcpy(zPtr, &value, sizeof(value));
                    }
                    else
                    {
                        std::memcpy(zPtr, &z, sizeof(z));
                    }
                }
            }
        }
        return true;
    }
}

namespace
{
    static constexpr uint32_t kDefaultDisplayWidth = 640u;
    static constexpr uint32_t kDefaultDisplayHeight = 448u;
    static constexpr uint32_t kHostFrameWidth = 640u;
    static constexpr uint32_t kHostFrameHeight = 512u;

    uint16_t encodeFramePixelPSMCT16(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
    {
        return static_cast<uint16_t>(((r >> 3) & 0x1Fu) |
                                     (((g >> 3) & 0x1Fu) << 5) |
                                     (((b >> 3) & 0x1Fu) << 10) |
                                     ((a >= 0x40u) ? 0x8000u : 0u));
    }

    // Display geometry in framebuffer pixels. DH counts output raster lines; with
    // SMODE2.INT and SMODE2.FFMD set the GS scans each framebuffer line once per
    // field, so the framebuffer holds half as many lines as DH describes (e.g. a
    // 512x224 buffer shown as 448 interlaced lines). readHeight is what is read
    // from VRAM, outHeight what is presented (lines are doubled when needed).
    void decodeDisplaySize(uint64_t display64, uint64_t smode2, uint32_t &outWidth, uint32_t &readHeight,
                           uint32_t &outHeight)
    {
        const uint32_t dw = static_cast<uint32_t>((display64 >> 32) & 0x0FFFu);
        const uint32_t dh = static_cast<uint32_t>((display64 >> 44) & 0x07FFu);
        const uint32_t magh = static_cast<uint32_t>((display64 >> 23) & 0x0Fu);
        const uint32_t magv = static_cast<uint32_t>((display64 >> 27) & 0x03u);
        const bool fieldRendered = (smode2 & 0x3ull) == 0x3ull; // INT && FFMD

        outWidth = (dw + 1u) / (magh + 1u);
        outHeight = (dh + 1u) / (magv + 1u);
        if (outWidth < 64u || outHeight < 64u)
        {
            outWidth = kDefaultDisplayWidth;
            outHeight = kDefaultDisplayHeight;
        }
        outWidth = std::min<uint32_t>(outWidth, kHostFrameWidth);
        outHeight = std::min<uint32_t>(outHeight, kHostFrameHeight);
        readHeight = fieldRendered ? std::max<uint32_t>(1u, outHeight / 2u) : outHeight;
    }

    // Expands rows [0, readHeight) of a host frame to outHeight rows in place.
    void stretchRows(std::vector<uint8_t> &pixels, uint32_t width, uint32_t readHeight, uint32_t outHeight)
    {
        if (pixels.empty() || readHeight == 0u || outHeight <= readHeight)
            return;
        for (uint32_t y = outHeight; y-- > 0u;)
        {
            const uint32_t sourceY = static_cast<uint32_t>((static_cast<uint64_t>(y) * readHeight) / outHeight);
            if (sourceY != y)
                std::memmove(pixels.data() + y * kHostFrameWidth * 4u,
                             pixels.data() + sourceY * kHostFrameWidth * 4u,
                             width * 4u);
        }
    }

    GSFrameReg decodeDisplayFrame(uint64_t dispfb64)
    {
        GSFrameReg frame{};
        frame.fbp = static_cast<uint32_t>(dispfb64 & 0x1FFu);
        frame.fbw = static_cast<uint32_t>((dispfb64 >> 9) & 0x3Fu);
        frame.psm = static_cast<uint8_t>((dispfb64 >> 15) & 0x1Fu);
        return frame;
    }

    struct GSDisplayReadOrigin
    {
        uint32_t x = 0u;
        uint32_t y = 0u;
    };

    GSDisplayReadOrigin decodeDisplayReadOrigin(uint64_t dispfb64)
    {
        return {
            static_cast<uint32_t>((dispfb64 >> 32) & 0x7FFu),
            static_cast<uint32_t>((dispfb64 >> 43) & 0x7FFu)};
    }

    bool hasDisplaySetup(uint64_t display64, const GSFrameReg &frame)
    {
        const uint32_t dw = static_cast<uint32_t>((display64 >> 32) & 0x0FFFu);
        const uint32_t dh = static_cast<uint32_t>((display64 >> 44) & 0x07FFu);
        const uint32_t magh = static_cast<uint32_t>((display64 >> 23) & 0x0Fu);
        return frame.fbw != 0u || dw != 0u || dh != 0u || magh != 0u;
    }

    struct GSPmodeState
    {
        bool enableCrt1 = false;
        bool enableCrt2 = false;
        bool mmod = false;
        bool amod = false;
        bool slbg = false;
        uint8_t alp = 0u;
    };

    GSPmodeState decodePmode(uint64_t pmode64)
    {
        return {
            (pmode64 & 0x1ull) != 0ull,
            (pmode64 & 0x2ull) != 0ull,
            ((pmode64 >> 5) & 0x1ull) != 0ull,
            ((pmode64 >> 6) & 0x1ull) != 0ull,
            ((pmode64 >> 7) & 0x1ull) != 0ull,
            static_cast<uint8_t>((pmode64 >> 8) & 0xFFu)};
    }

    void normalizePresentationAlpha(std::vector<uint8_t> &pixels, uint32_t width, uint32_t height)
    {
        for (uint32_t y = 0; y < height; ++y)
        {
            uint8_t *row = pixels.data() + y * kHostFrameWidth * 4u;
            for (uint32_t x = 0; x < width; ++x)
                row[x * 4u + 3u] = 255u;
        }
    }

    uint8_t blendPresentationChannel(uint8_t src, uint8_t dst, uint32_t factor)
    {
        const int delta = static_cast<int>(src) - static_cast<int>(dst);
        return GSInternal::clampU8(static_cast<int>(dst) + ((delta * static_cast<int>(factor)) / 255));
    }

    uint32_t countNonBlackPixels(const std::vector<uint8_t> &pixels, uint32_t width, uint32_t height)
    {
        uint32_t count = 0u;
        for (uint32_t y = 0; y < height; ++y)
        {
            const uint8_t *row = pixels.data() + y * kHostFrameWidth * 4u;
            for (uint32_t x = 0; x < width; ++x)
            {
                if (row[x * 4u] != 0u || row[x * 4u + 1u] != 0u || row[x * 4u + 2u] != 0u)
                    ++count;
            }
        }
        return count;
    }
}

GSCpuBackend::GSCpuBackend()
{
    using namespace GSMem;
    static std::once_flag lookupTablesOnce;
    std::call_once(lookupTablesOnce, []()
                   { InitLookupTables(); });
    for (size_t i = 0; i < kPsmHandlerCount; ++i)
    {
        switch (i)
        {
        case GS_PSM_CT32:
            m_readVramFuncs[i] = ReadCT32;
            m_writeVramFuncs[i] = WriteCT32;
            break;
        case GS_PSM_CT24:
            m_readVramFuncs[i] = ReadCT24;
            m_writeVramFuncs[i] = WriteCT24;
            break;
        case GS_PSM_CT16:
            m_readVramFuncs[i] = ReadCT16;
            m_writeVramFuncs[i] = WriteCT16;
            break;
        case GS_PSM_CT16S:
            m_readVramFuncs[i] = ReadCT16S;
            m_writeVramFuncs[i] = WriteCT16S;
            break;
        case GS_PSM_T8:
            m_readVramFuncs[i] = ReadP8;
            m_writeVramFuncs[i] = WriteP8;
            break;
        case GS_PSM_T8H:
            m_readVramFuncs[i] = ReadP8H;
            m_writeVramFuncs[i] = WriteP8H;
            break;
        case GS_PSM_T4:
            m_readVramFuncs[i] = ReadP4;
            m_writeVramFuncs[i] = WriteP4;
            break;
        case GS_PSM_T4HH:
            m_readVramFuncs[i] = ReadP4HH;
            m_writeVramFuncs[i] = WriteP4HH;
            break;
        case GS_PSM_T4HL:
            m_readVramFuncs[i] = ReadP4HL;
            m_writeVramFuncs[i] = WriteP4HL;
            break;
        case GS_PSM_Z32:
            m_readVramFuncs[i] = ReadZ32;
            m_writeVramFuncs[i] = WriteZ32;
            break;
        case GS_PSM_Z24:
            m_readVramFuncs[i] = ReadZ24;
            m_writeVramFuncs[i] = WriteZ24;
            break;
        case GS_PSM_Z16:
            m_readVramFuncs[i] = ReadZ16;
            m_writeVramFuncs[i] = WriteZ16;
            break;
        case GS_PSM_Z16S:
            m_readVramFuncs[i] = ReadZ16S;
            m_writeVramFuncs[i] = WriteZ16S;
            break;
        default:
            m_readVramFuncs[i] = ReadNull;
            m_writeVramFuncs[i] = WriteNull;
            break;
        }
    }
    Reset();
}

void GSCpuBackend::Initialize(uint8_t *vram, uint32_t vramSize)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    invalidateClutCaches();
    m_textureCache.clear();
    m_retiredTextures.clear();
    m_textureCacheBytes = 0;
    m_vram = vram;
    m_vramSize = vramSize;
    ResetUnlocked();
}

void GSCpuBackend::Reset()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    invalidateClutCaches();
    m_textureCache.clear();
    m_retiredTextures.clear();
    m_textureCacheBytes = 0;
    ResetUnlocked();
}

void GSCpuBackend::ResetUnlocked()
{
    m_transfer = {};
    m_transfer.direction = 3u;
    m_transferState = {};
    m_transferState.direction = 3u;
    m_localToHostBuffer.clear();
    m_localToHostReadPos = 0u;
}

void GSCpuBackend::Submit(const GSPrimitiveBatch &batch)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_vram || batch.vertexCount == 0u)
        return;
    DrawPrimitive(batch);
    m_retiredTextures.clear();

    // Textures decoded from the pages just drawn to are now stale.
    const GSContext &ctx = batch.state.context;
    const uint32_t rows = static_cast<uint32_t>(ctx.scissor.y1) + 1u;
    uint32_t begin = 0, end = 0;
    bufferRange(ctx.frame.fbp * 32u, ctx.frame.fbw, rows, psmBits(ctx.frame.psm), begin, end);
    touchVramRange(begin, end, 1u);
    if (!ctx.zbuf.zmask)
    {
        bufferRange(ctx.zbuf.zbp * 32u, ctx.frame.fbw, rows, psmBits(ctx.zbuf.psm), begin, end);
        touchVramRange(begin, end, 1u);
    }
}

const GSCpuBackend::DecodedTexture *GSCpuBackend::FindTexture(const GSDrawState &state, bool allowDecode)
{
    constexpr size_t kMaxCacheBytes = 128u * 1024u * 1024u;
    constexpr uint32_t kMaxTextureSize = 1024u;

    const GSTex0Reg &tex = state.context.tex0;
    const uint32_t width = state.textureWidth;
    const uint32_t height = state.textureHeight;
    // PS2X_GS_NO_TEXCACHE=1 samples GS memory directly (for comparisons).
    static const bool disabled = std::getenv("PS2X_GS_NO_TEXCACHE") != nullptr;
    if (disabled || !m_vram || width == 0u || height == 0u || width > kMaxTextureSize || height > kMaxTextureSize)
        return nullptr;

    const bool paletted = tex.psm == GS_PSM_T8 || tex.psm == GS_PSM_T8H || tex.psm == GS_PSM_T4 ||
                          tex.psm == GS_PSM_T4HL || tex.psm == GS_PSM_T4HH;
    uint32_t texBegin = 0, texEnd = 0;
    bufferRange(tex.tbp0, std::max<uint32_t>(tex.tbw, (width + 63u) / 64u), height, psmBits(tex.psm), texBegin, texEnd);
    uint32_t epoch = g_vramEpoch.load(std::memory_order_acquire) + vramRangeEpoch(texBegin, texEnd);

    std::array<uint32_t, 12> key{};
    key[0] = tex.tbp0;
    key[1] = tex.tbw;
    key[2] = tex.psm;
    key[3] = width;
    key[4] = height;
    key[5] = static_cast<uint32_t>(state.texa.ta0) | (static_cast<uint32_t>(state.texa.ta1) << 8) |
             (state.texa.aem ? 0x10000u : 0u);
    if (paletted)
    {
        uint32_t clutBegin = 0, clutEnd = 0;
        bufferRange(tex.cbp, state.texclut.cbw, static_cast<uint32_t>(state.texclut.cov) + 32u, 32u, clutBegin, clutEnd);
        epoch += vramRangeEpoch(clutBegin, clutEnd) * 31u;
        key[6] = tex.cbp;
        key[7] = static_cast<uint32_t>(tex.cpsm) | (static_cast<uint32_t>(tex.csm) << 6) |
                 (static_cast<uint32_t>(tex.csa) << 8) | (static_cast<uint32_t>(state.texclut.cbw) << 16) |
                 (static_cast<uint32_t>(state.texclut.cou) << 24);
        key[8] = state.texclut.cov;
    }
    key[9] = epoch;

    // key[0..8] name the texture, key[9] is the memory epoch it was last
    // seen at and key[10..11] the hash of its bytes (and CLUT).
    auto sameTexture = [](const std::array<uint32_t, 12> &a, const std::array<uint32_t, 12> &b)
    {
        return std::equal(a.begin(), a.begin() + 9, b.begin());
    };
    for (const std::unique_ptr<DecodedTexture> &entry : m_textureCache)
    {
        if (entry->key[9] == epoch && sameTexture(entry->key, key))
        {
            if (allowDecode)
                entry->lastUse = ++m_textureUseCounter;
            return entry.get();
        }
    }
    if (!allowDecode)
        return nullptr;

    // Its memory changed. Games stream textures through the same memory
    // every frame, so the bytes usually match an earlier decode.
    uint32_t spanBegin = 0, spanEnd = 0;
    textureSpan(tex.tbp0, tex.tbw, tex.psm, width, height, spanBegin, spanEnd);
    uint64_t hash = hashBytes(m_vram + spanBegin, spanEnd - spanBegin, 0x6A09E667F3BCC909ull);
    if (paletted)
    {
        uint32_t clutBegin = 0, clutEnd = 0;
        if (tex.csm == 0u)
        {
            // CSM1: 256 entries fill blocks 0-3 (32-bit) or 0-5 (16-bit).
            clutBegin = std::min<uint32_t>(tex.cbp * 256u, 4u * 1024u * 1024u);
            clutEnd = std::min<uint32_t>(clutBegin + (psmBits(tex.cpsm) == 32u ? 1024u : 1536u), 4u * 1024u * 1024u);
        }
        else
        {
            bufferRange(tex.cbp, state.texclut.cbw, static_cast<uint32_t>(state.texclut.cov) + 32u, 32u, clutBegin, clutEnd);
        }
        hash = hashBytes(m_vram + clutBegin, clutEnd - clutBegin, hash);
    }
    key[10] = static_cast<uint32_t>(hash);
    key[11] = static_cast<uint32_t>(hash >> 32);

    auto oldestVariant = m_textureCache.end();
    uint32_t variants = 0;
    for (auto it = m_textureCache.begin(); it != m_textureCache.end(); ++it)
    {
        DecodedTexture &entry = **it;
        if (!sameTexture(entry.key, key))
            continue;
        if (entry.key[10] == key[10] && entry.key[11] == key[11])
        {
            entry.key[9] = epoch;
            entry.lastUse = ++m_textureUseCounter;
            return &entry;
        }
        ++variants;
        if (oldestVariant == m_textureCache.end() || entry.lastUse < (*oldestVariant)->lastUse)
            oldestVariant = it;
    }
    // Keep a few contents per texture (queued draws may still sample a
    // dropped one until the run ends).
    if (variants >= 4u)
        RetireTexture(oldestVariant);
    static const bool redecodeStats = std::getenv("PS2X_GS_STATS") != nullptr;
    if (redecodeStats && variants > 0u)
    {
        uint32_t sources = 0;
        for (uint32_t page = spanBegin / kVramPageBytes; page < kVramPages && page * kVramPageBytes < spanEnd; ++page)
            sources |= 1u << g_pageTouchSource[page].load(std::memory_order_relaxed);
        char name[112];
        std::snprintf(name, sizeof(name), "tbp0=%x tbw=%u psm=%x %ux%u cbp=%x span=%x-%x src=%x",
                      tex.tbp0, tex.tbw, tex.psm, width, height, paletted ? tex.cbp : 0u,
                      spanBegin, spanEnd, sources);
        ++g_redecodes[name];
    }

    // Decode exactly as SampleTexture's point sampler reads texels.
    auto decoded = std::make_unique<DecodedTexture>();
    decoded->key = key;
    decoded->width = width;
    decoded->height = height;
    decoded->lastUse = ++m_textureUseCounter;
    decoded->texels.resize(static_cast<size_t>(width) * height);
    std::array<uint32_t, 256> palette{};
    if (paletted)
    {
        const bool fourBit = tex.psm == GS_PSM_T4 || tex.psm == GS_PSM_T4HL || tex.psm == GS_PSM_T4HH;
        for (uint32_t i = 0; i < (fourBit ? 16u : 256u); ++i)
            palette[i] = LookupCLUT(state, static_cast<u8>(i), tex.cbp, tex.cpsm, tex.csm, tex.csa, tex.psm);
    }
    const ReadVramFunc read = m_readVramFuncs[tex.psm & (kPsmHandlerCount - 1u)];
    for (uint32_t v = 0; v < height; ++v)
    {
        uint32_t *row = decoded->texels.data() + static_cast<size_t>(v) * width;
        switch (tex.psm)
        {
        case GS_PSM_CT32:
        case GS_PSM_Z32:
        case GS_PSM_CT24:
        case GS_PSM_Z24:
            for (uint32_t u = 0; u < width; ++u)
                row[u] = applyTexa(state.texa, tex.psm, read(m_vram, tex.tbp0, tex.tbw, u, v));
            break;
        case GS_PSM_CT16:
        case GS_PSM_CT16S:
        case GS_PSM_Z16:
        case GS_PSM_Z16S:
            for (uint32_t u = 0; u < width; ++u)
                row[u] = applyTexa(state.texa, tex.psm, Rgba5551ToRgba8888(read(m_vram, tex.tbp0, tex.tbw, u, v)));
            break;
        case GS_PSM_T8:
        case GS_PSM_T8H:
        case GS_PSM_T4:
        case GS_PSM_T4HL:
        case GS_PSM_T4HH:
            for (uint32_t u = 0; u < width; ++u)
                row[u] = palette[read(m_vram, tex.tbp0, tex.tbw, u, v) & 0xFFu];
            break;
        default:
            std::fill(row, row + width, 0xFFFF00FFu);
            break;
        }
    }
    // PS2X_DUMP_TEXTURE=<tbp0 hex>: writes the first decodes of that texture
    // as tex_<tbp0>_<n>.ppm (colour) and .pgm (alpha).
    static const char *dumpEnv = std::getenv("PS2X_DUMP_TEXTURE");
    if (dumpEnv && std::strtoul(dumpEnv, nullptr, 16) == tex.tbp0)
    {
        static uint32_t dumps = 0;
        if (dumps < 4u)
        {
            char name[64];
            std::snprintf(name, sizeof(name), "tex_%x_%u.ppm", tex.tbp0, dumps);
            if (FILE *f = std::fopen(name, "wb"))
            {
                std::fprintf(f, "P6\n%u %u\n255\n", width, height);
                for (uint32_t texel : decoded->texels)
                {
                    const uint8_t rgb[3] = {static_cast<uint8_t>(texel), static_cast<uint8_t>(texel >> 8),
                                            static_cast<uint8_t>(texel >> 16)};
                    std::fwrite(rgb, 1, 3, f);
                }
                std::fclose(f);
            }
            std::snprintf(name, sizeof(name), "tex_%x_%u.pgm", tex.tbp0, dumps);
            if (FILE *f = std::fopen(name, "wb"))
            {
                std::fprintf(f, "P5\n%u %u\n255\n", width, height);
                for (uint32_t texel : decoded->texels)
                {
                    const uint8_t alpha = static_cast<uint8_t>(texel >> 24);
                    std::fwrite(&alpha, 1, 1, f);
                }
                std::fclose(f);
            }
            std::fprintf(stderr, "[gs] dumped texture %x (%ux%u psm=%x cbp=%x cpsm=%x csm=%u csa=%u) as dump %u\n", tex.tbp0,
                         width, height, tex.psm, tex.cbp, tex.cpsm, tex.csm, tex.csa, dumps);
            if (paletted)
            {
                uint32_t histogram[256] = {};
                for (uint32_t v = 0; v < height; ++v)
                    for (uint32_t u = 0; u < width; ++u)
                        ++histogram[read(m_vram, tex.tbp0, tex.tbw, u, v) & 0xFFu];
                std::fprintf(stderr, "[gs]   palette:");
                for (uint32_t i = 0; i < 16u; ++i)
                    std::fprintf(stderr, " %08x", palette[i]);
                std::fprintf(stderr, "\n[gs]   index histogram:");
                for (uint32_t i = 0; i < 16u; ++i)
                    std::fprintf(stderr, " %u:%u", i, histogram[i]);
                std::fprintf(stderr, "\n[gs]   raw CLUT words:");
                for (uint32_t i = 0; i < 16u; ++i)
                {
                    uint32_t word = 0;
                    std::memcpy(&word, m_vram + tex.cbp * 256u + i * 4u, 4);
                    std::fprintf(stderr, " %08x", word);
                }
                std::fprintf(stderr, "\n");
            }
            ++dumps;
        }
    }
    static const bool stats = std::getenv("PS2X_GS_STATS") != nullptr;
    if (stats)
    {
        static uint64_t decodes = 0, texels = 0;
        static auto last = std::chrono::steady_clock::now();
        ++decodes;
        texels += static_cast<uint64_t>(width) * height;
        const auto now = std::chrono::steady_clock::now();
        if (now - last >= std::chrono::seconds(5))
        {
            std::fprintf(stderr, "[gs-stats] texture decodes: %llu (%llu texels) in 5 s, cache %zu entries\n",
                         static_cast<unsigned long long>(decodes), static_cast<unsigned long long>(texels),
                         m_textureCache.size());
            std::vector<std::pair<uint32_t, std::string>> top;
            for (const auto &[name, count] : g_redecodes)
                top.emplace_back(count, name);
            std::sort(top.rbegin(), top.rend());
            for (size_t i = 0; i < top.size() && i < 10u; ++i)
                std::fprintf(stderr, "[gs-stats]   redecoded %u x %s\n", top[i].first, top[i].second.c_str());
            g_redecodes.clear();
            decodes = texels = 0;
            last = now;
        }
    }

    m_textureCacheBytes += decoded->texels.size() * sizeof(uint32_t);
    while (m_textureCacheBytes > kMaxCacheBytes && !m_textureCache.empty())
    {
        auto oldest = std::min_element(m_textureCache.begin(), m_textureCache.end(),
                                       [](const std::unique_ptr<DecodedTexture> &a, const std::unique_ptr<DecodedTexture> &b)
                                       { return a->lastUse < b->lastUse; });
        RetireTexture(oldest);
    }
    m_textureCache.push_back(std::move(decoded));
    return m_textureCache.back().get();
}

void GSCpuBackend::RetireTexture(std::vector<std::unique_ptr<DecodedTexture>>::iterator entry)
{
    m_textureCacheBytes -= (*entry)->texels.size() * sizeof(uint32_t);
    m_retiredTextures.push_back(std::move(*entry));
    m_textureCache.erase(entry);
}

void GSCpuBackend::ReleaseRetiredTextures()
{
    m_retiredTextures.clear();
}

const GSCpuBackend::DecodedTexture *GSCpuBackend::PrepareTexture(const GSDrawState &state)
{
    if (!state.prim.tme || !m_vram)
        return nullptr;
    beginPrimitiveClutCache(state);
    return FindTexture(state, true);
}

void GSCpuBackend::NoteVramWrites(uint32_t begin, uint32_t end)
{
    // PS2X_WATCH_VRAM=<byte address hex>: logs draws whose target covers it.
    static const char *watchEnv = std::getenv("PS2X_WATCH_VRAM");
    if (watchEnv)
    {
        static const uint32_t watch = static_cast<uint32_t>(std::strtoul(watchEnv, nullptr, 16));
        static uint32_t logged = 0;
        if (watch >= begin && watch < end && logged++ < 200u)
            std::fprintf(stderr, "[gs] @%.2fs draw run wrote %x-%x (covers watched %x)\n",
                         static_cast<double>(g_ps2xVBlankCount.load(std::memory_order_relaxed)) / 59.94, begin, end, watch);
    }
    touchVramRange(begin, end, 2u);
}

void GSCpuBackend::DrawBanded(const GSPrimitiveBatch &batch, uint32_t bandIndex, uint32_t bandCount,
                              const DecodedTexture *texture)
{
    if (!m_vram || batch.vertexCount == 0u)
        return;
    t_bandIndex = bandIndex;
    t_bandMask = bandCount - 1u;
    t_runTexture = texture;
    t_useRunTexture = true;
    DrawPrimitive(batch);
    t_useRunTexture = false;
    t_bandIndex = 0u;
    t_bandMask = 0u;
}

void GSCpuBackend::Flush()
{
    // CPU backend is immediate. GPU backends may submit command buffers here.
}

void GSCpuBackend::TextureFlush()
{
    // CPU texture reads are coherent with local memory. Future cached/GPU
    // backends use this boundary to invalidate texture views.
}

void GSCpuBackend::Sync(GSSyncReason)
{
    // CPU backend is immediate. GPU backends may wait on fences/readbacks here.
}

uint32_t GSCpuBackend::ReadVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return ReadVramUnlocked(psm, base, bw, x, y);
}

uint32_t GSCpuBackend::ReadVramUnlocked(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y) const
{
    if (!m_vram)
        return 0u;
    return m_readVramFuncs[psm & 0x3Fu](m_vram, base, bw, x, y);
}

void GSCpuBackend::WriteVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y, uint32_t value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    invalidateClutCaches();
    WriteVramUnlocked(psm, base, bw, x, y, value);
}

void GSCpuBackend::WriteVramUnlocked(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y, uint32_t value)
{
    if (!m_vram)
        return;
    m_writeVramFuncs[psm & 0x3Fu](m_vram, base, bw, x, y, value);
}

void GSCpuBackend::SnapshotVram(std::vector<uint8_t> &out) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_vram || m_vramSize == 0u)
    {
        out.clear();
        return;
    }
    out.resize(m_vramSize);
    std::memcpy(out.data(), m_vram, m_vramSize);
}

GSTransferSnapshot GSCpuBackend::GetTransferSnapshot() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    GSTransferSnapshot result = m_transferState;
    result.localToHostPendingBytes = m_localToHostReadPos < m_localToHostBuffer.size()
                                         ? m_localToHostBuffer.size() - m_localToHostReadPos
                                         : 0u;
    return result;
}

void GSCpuBackend::DrawPrimitive(const GSPrimitiveBatch &batch)
{
    const GSDrawState &state = batch.state;
    const auto &ctx = state.context;
    PS2_IF_AGRESSIVE_LOGS({
        const uint32_t primitiveIndex = s_debugPrimitiveCount.fetch_add(1u, std::memory_order_relaxed);
        if (primitiveIndex < 64u)
        {
            std::cout << "[gs:prim] idx=" << primitiveIndex
                      << " type=" << static_cast<uint32_t>(state.prim.type)
                      << " tme=" << static_cast<uint32_t>(state.prim.tme)
                      << " abe=" << static_cast<uint32_t>(state.prim.abe)
                      << " fst=" << static_cast<uint32_t>(state.prim.fst)
                      << " ctxt=" << static_cast<uint32_t>(state.prim.ctxt)
                      << " fbp=" << ctx.frame.fbp
                      << " fbw=" << ctx.frame.fbw
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.frame.psm) << std::dec
                      << " tex0=("
                      << "tbp0=" << ctx.tex0.tbp0
                      << " tbw=" << static_cast<uint32_t>(ctx.tex0.tbw)
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.tex0.psm) << std::dec
                      << " tw=" << static_cast<uint32_t>(ctx.tex0.tw)
                      << " th=" << static_cast<uint32_t>(ctx.tex0.th)
                      << " tcc=" << static_cast<uint32_t>(ctx.tex0.tcc)
                      << " tfx=" << static_cast<uint32_t>(ctx.tex0.tfx)
                      << " cbp=" << ctx.tex0.cbp
                      << " cpsm=0x" << std::hex << static_cast<uint32_t>(ctx.tex0.cpsm) << std::dec
                      << " csm=" << static_cast<uint32_t>(ctx.tex0.csm)
                      << " csa=" << static_cast<uint32_t>(ctx.tex0.csa)
                      << ")"
                      << " texclut=("
                      << "cbw=" << static_cast<uint32_t>(state.texclut.cbw)
                      << " cou=" << static_cast<uint32_t>(state.texclut.cou)
                      << " cov=" << state.texclut.cov
                      << ")"
                      << " ofx=" << (ctx.xyoffset.ofx >> 4)
                      << " ofy=" << (ctx.xyoffset.ofy >> 4)
                      << " scissor=(" << ctx.scissor.x0
                      << "," << ctx.scissor.y0
                      << ")-(" << ctx.scissor.x1
                      << "," << ctx.scissor.y1 << ")"
                      << " test=0x" << std::hex << ctx.test
                      << " alpha=0x" << ctx.alpha
                      << std::dec
                      << " v0=(" << batch.vertices[0].x << "," << batch.vertices[0].y << ")"
                      << " uv0=(" << (batch.vertices[0].u >> 4) << "," << (batch.vertices[0].v >> 4) << ")"
                      << " stq0=(" << batch.vertices[0].s << "," << batch.vertices[0].t << "," << batch.vertices[0].q << ")"
                      << " v1=(" << batch.vertices[1].x << "," << batch.vertices[1].y << ")"
                      << " uv1=(" << (batch.vertices[1].u >> 4) << "," << (batch.vertices[1].v >> 4) << ")"
                      << " stq1=(" << batch.vertices[1].s << "," << batch.vertices[1].t << "," << batch.vertices[1].q << ")"
                      << " v2=(" << batch.vertices[2].x << "," << batch.vertices[2].y << ")"
                      << " uv2=(" << (batch.vertices[2].u >> 4) << "," << (batch.vertices[2].v >> 4) << ")"
                      << " stq2=(" << batch.vertices[2].s << "," << batch.vertices[2].t << "," << batch.vertices[2].q << ")"
                      << " rgba0=(" << static_cast<uint32_t>(batch.vertices[0].r) << ","
                      << static_cast<uint32_t>(batch.vertices[0].g) << ","
                      << static_cast<uint32_t>(batch.vertices[0].b) << ","
                      << static_cast<uint32_t>(batch.vertices[0].a) << ")"
                      << " rgba1=(" << static_cast<uint32_t>(batch.vertices[1].r) << ","
                      << static_cast<uint32_t>(batch.vertices[1].g) << ","
                      << static_cast<uint32_t>(batch.vertices[1].b) << ","
                      << static_cast<uint32_t>(batch.vertices[1].a) << ")"
                      << " rgba2=(" << static_cast<uint32_t>(batch.vertices[2].r) << ","
                      << static_cast<uint32_t>(batch.vertices[2].g) << ","
                      << static_cast<uint32_t>(batch.vertices[2].b) << ","
                      << static_cast<uint32_t>(batch.vertices[2].a) << ")"
                      << std::endl;
        }
    });

    PS2_IF_AGRESSIVE_LOGS({
        if ((state.prim.ctxt != 0u || ctx.frame.fbp == 150u) &&
            s_debugContext1PrimitiveCount.fetch_add(1u, std::memory_order_relaxed) < 32u)
        {
            std::cout << "[gs:copy-prim]"
                      << " type=" << static_cast<uint32_t>(state.prim.type)
                      << " tme=" << static_cast<uint32_t>(state.prim.tme)
                      << " abe=" << static_cast<uint32_t>(state.prim.abe)
                      << " fst=" << static_cast<uint32_t>(state.prim.fst)
                      << " ctxt=" << static_cast<uint32_t>(state.prim.ctxt)
                      << " fbp=" << ctx.frame.fbp
                      << " fbw=" << ctx.frame.fbw
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.frame.psm) << std::dec
                      << " tex0=("
                      << "tbp0=" << ctx.tex0.tbp0
                      << " tbw=" << static_cast<uint32_t>(ctx.tex0.tbw)
                      << " psm=0x" << std::hex << static_cast<uint32_t>(ctx.tex0.psm) << std::dec
                      << " tcc=" << static_cast<uint32_t>(ctx.tex0.tcc)
                      << " tfx=" << static_cast<uint32_t>(ctx.tex0.tfx)
                      << " cbp=" << ctx.tex0.cbp
                      << " cpsm=0x" << std::hex << static_cast<uint32_t>(ctx.tex0.cpsm) << std::dec
                      << " csm=" << static_cast<uint32_t>(ctx.tex0.csm)
                      << " csa=" << static_cast<uint32_t>(ctx.tex0.csa)
                      << ")"
                      << " texclut=("
                      << "cbw=" << static_cast<uint32_t>(state.texclut.cbw)
                      << " cou=" << static_cast<uint32_t>(state.texclut.cou)
                      << " cov=" << state.texclut.cov
                      << ")"
                      << " ofx=" << (ctx.xyoffset.ofx >> 4)
                      << " ofy=" << (ctx.xyoffset.ofy >> 4)
                      << " scissor=(" << ctx.scissor.x0
                      << "," << ctx.scissor.y0
                      << ")-(" << ctx.scissor.x1
                      << "," << ctx.scissor.y1 << ")"
                      << " test=0x" << std::hex << ctx.test
                      << " alpha=0x" << ctx.alpha
                      << std::dec << std::endl;
        }
    });

    beginPrimitiveClutCache(state);
    switch (state.prim.type)
    {
    case GS_PRIM_SPRITE:
        DrawSprite(batch);
        break;
    case GS_PRIM_TRIANGLE:
    case GS_PRIM_TRISTRIP:
    case GS_PRIM_TRIFAN:
        DrawTriangle(batch);
        break;
    case GS_PRIM_LINE:
    case GS_PRIM_LINESTRIP:
        DrawLine(batch);
        break;
    case GS_PRIM_POINT:
    {
        const GSVertex &v = batch.vertices[0];
        const auto &ctx = state.context;
        int px = static_cast<int>(v.x) - (ctx.xyoffset.ofx >> 4);
        int py = static_cast<int>(v.y) - (ctx.xyoffset.ofy >> 4);
        WritePixel(state, px, py, static_cast<u32>(v.z), v.r, v.g, v.b, v.a, v.fog);
        break;
    }
    default:
        break;
    }
}

void GSCpuBackend::WritePixel(const GSDrawState &state, int x, int y, int z, uint8_t r, uint8_t g, uint8_t b, uint8_t a, uint8_t fog)
{
    const auto &ctx = state.context;
    if (x < ctx.scissor.x0 || x > ctx.scissor.x1 || y < ctx.scissor.y0 || y > ctx.scissor.y1)
        return;
    // Lines and points reach here without a per-row band check.
    if (!rowInBand(y))
        return;

    if (state.prim.fge)
    {
        const uint32_t inverseFog = 255u - fog;
        auto applyFog = [&](uint8_t input, uint8_t fogColor) -> uint8_t
        {
            return static_cast<uint8_t>(((static_cast<uint32_t>(fog) * input) >> 8) + ((inverseFog * fogColor) >> 8));
        };

        r = applyFog(r, state.fogR);
        g = applyFog(g, state.fogG);
        b = applyFog(b, state.fogB);
    }

    const u32 fbp = GSInternal::framePageBaseToBlock(ctx.frame.fbp);
    const u32 fbw = std::max<u32>(ctx.frame.fbw, 1u);
    const u32 fpsm = ctx.frame.psm;
    const u32 zbp = GSInternal::framePageBaseToBlock(ctx.zbuf.zbp);
    const u32 zpsm = ctx.zbuf.psm;

    const PixelWriteMask writeMask = classifyAlphaTest(ctx.test, a, static_cast<uint8_t>(fpsm));
    if (!writeMask.writesAnything())
    {
        return;
    }

    const uint32_t ztestMethod = static_cast<uint32_t>((ctx.test >> 17) & 3u);
    // Depth is clamped to the largest value the Z buffer format holds.
    if (zpsm == GS_PSM_Z24)
        z = static_cast<int>(std::min<uint32_t>(static_cast<uint32_t>(z), 0xFFFFFFu));
    else if (zpsm == GS_PSM_Z16 || zpsm == GS_PSM_Z16S)
        z = static_cast<int>(std::min<uint32_t>(static_cast<uint32_t>(z), 0xFFFFu));
    const bool alphaBlendEnabled = state.prim.abe;
    const bool preserveDestinationAlpha = writeMask.writeRgb && !writeMask.writeAlpha && fpsm == GS_PSM_CT32;
    const bool destinationAlphaTestNeedsRead = ((ctx.test >> 14) & 0x1u) != 0u && (fpsm == GS_PSM_CT32 || fpsm == GS_PSM_CT16 || fpsm == GS_PSM_CT16S);

    // small optimization, avoid reading the framebuffer for simple draws
    // TODO: only one address lookup for rmw
    const bool frmw = destinationAlphaTestNeedsRead || (writeMask.writesFramebuffer() && ((ctx.frame.fbmsk != 0) || alphaBlendEnabled || preserveDestinationAlpha));

    u32 rawFramebufferPixel = 0;
    u32 fbrgba = 0;
    if (frmw)
    {
        rawFramebufferPixel = ReadVramUnlocked(fpsm, fbp, fbw, x, y);
        fbrgba = rawFramebufferPixel;

        if (bitsPerPixel(fpsm) == 16)
        {
            fbrgba = Rgba5551ToRgba8888(fbrgba);
        }
        else if (fpsm == GS_PSM_CT24)
        {
            // The GS supplies 0x80 as destination alpha for RGB24 blending.
            fbrgba |= 0x80000000u;
        }
    }

    if (!passesDestinationAlphaTest(ctx.test, static_cast<uint8_t>(fpsm), rawFramebufferPixel))
    {
        return;
    }

    bool zpass = false;
    uint32_t storedZ = 0u;
    switch (ztestMethod)
    {
    case 0:
        zpass = false;
        break;
    case 1:
        zpass = true;
        break;
    case 2:
        storedZ = ReadVramUnlocked(zpsm, zbp, fbw, x, y);
        zpass = static_cast<uint32_t>(z) >= storedZ;
        break;
    case 3:
        storedZ = ReadVramUnlocked(zpsm, zbp, fbw, x, y);
        zpass = static_cast<uint32_t>(z) > storedZ;
        break;
    }

    if (!zpass)
    {
        return;
    }

    if (writeMask.writesFramebuffer())
    {
        const u8 srcR = r;
        const u8 srcG = g;
        const u8 srcB = b;

        if (state.prim.abe)
        {
            uint8_t dr = fbrgba & 0xFF;
            uint8_t dg = (fbrgba >> 8) & 0xFF;
            uint8_t db = (fbrgba >> 16) & 0xFF;
            uint8_t da = (fbrgba >> 24) & 0xFF;

            // PABE disables alpha blending when the source alpha MSB is clear.
            if (!(state.pabe && (a & 0x80u) == 0u))
            {
                uint64_t alphaReg = ctx.alpha;
                uint8_t asel = alphaReg & 3;
                uint8_t bsel = (alphaReg >> 2) & 3;
                uint8_t csel = (alphaReg >> 4) & 3;
                uint8_t dsel = (alphaReg >> 6) & 3;
                uint8_t fix = static_cast<uint8_t>((alphaReg >> 32) & 0xFF);

                auto pickRGB = [&](uint8_t sel, int cs, int cd) -> int
                {
                    if (sel == 0)
                        return cs;
                    if (sel == 1)
                        return cd;
                    return 0;
                };
                int cAlpha = (csel == 0) ? a : (csel == 1) ? da
                                                           : fix;

                r = clampU8(((pickRGB(asel, r, dr) - pickRGB(bsel, r, dr)) * cAlpha >> 7) + pickRGB(dsel, r, dr));
                g = clampU8(((pickRGB(asel, g, dg) - pickRGB(bsel, g, dg)) * cAlpha >> 7) + pickRGB(dsel, g, dg));
                b = clampU8(((pickRGB(asel, b, db) - pickRGB(bsel, b, db)) * cAlpha >> 7) + pickRGB(dsel, b, db));
            }
            else
            {
                r = srcR;
                g = srcG;
                b = srcB;
            }
        }

        if (writeMask.writeAlpha && (ctx.fba & 0x1ull) != 0ull && ctx.frame.psm != GS_PSM_CT24)
        {
            a = static_cast<uint8_t>(a | 0x80u);
        }

        u32 pixel = pack32(r, g, b, a);

        if (ctx.frame.fbmsk != 0)
        {
            pixel = (pixel & ~ctx.frame.fbmsk) | (fbrgba & ctx.frame.fbmsk);
        }

        if (preserveDestinationAlpha)
        {
            pixel = (pixel & 0x00FFFFFFu) | (fbrgba & 0xFF000000u);
        }

        // format conversion
        if (bitsPerPixel(fpsm) == 16)
        {
            pixel = Rgba8888ToRgba5551(pixel);
        }

        WriteVramUnlocked(fpsm, fbp, fbw, x, y, pixel);
    }

    if (writeMask.writeDepth && !ctx.zbuf.zmask)
    {
        WriteVramUnlocked(zpsm, zbp, fbw, x, y, z);
    }
}

uint32_t GSCpuBackend::LookupCLUT(const GSDrawState &state,
                                  uint8_t index,
                                  uint32_t cbp,
                                  uint8_t cpsm,
                                  uint8_t csm,
                                  uint8_t csa,
                                  uint8_t sourcePsm)
{
    const uint32_t clutIndex = resolveClutIndex(index, cpsm, csm, csa, sourcePsm);
    ClutCacheEntry &cached = t_clutCache[clutIndex & 0x1FFu];
    static const bool stats = std::getenv("PS2X_GS_STATS") != nullptr;
    if (stats)
    {
        static std::atomic<uint64_t> lookups{0}, misses{0}, keyChanges{0};
        static thread_local uint32_t lastGeneration = 0;
        if (lastGeneration != t_clutGeneration)
        {
            lastGeneration = t_clutGeneration;
            keyChanges.fetch_add(1u, std::memory_order_relaxed);
        }
        if (cached.generation != t_clutGeneration)
            misses.fetch_add(1u, std::memory_order_relaxed);
        if ((lookups.fetch_add(1u, std::memory_order_relaxed) & 0xFFFFFFu) == 0u)
            std::fprintf(stderr, "[gs-stats] CLUT lookups %llu, misses %llu, cache resets %llu\n",
                         static_cast<unsigned long long>(lookups.load()), static_cast<unsigned long long>(misses.load()),
                         static_cast<unsigned long long>(keyChanges.load()));
    }
    if (cached.generation == t_clutGeneration)
        return cached.value;

    const uint32_t clutWidth = (state.texclut.cbw != 0u) ? static_cast<uint32_t>(state.texclut.cbw) : 1u;
    const uint32_t clutX = static_cast<uint32_t>(state.texclut.cou) + (clutIndex & 0x0Fu);
    const uint32_t clutY = static_cast<uint32_t>(state.texclut.cov) + (clutIndex >> 4);

    uint32_t value = 0xFFFF00FFu;
    switch (cpsm)
    {
    case GS_PSM_CT32:
        value = applyTexa(state.texa, cpsm, GSMem::ReadCT32(m_vram, cbp, clutWidth, clutX, clutY));
        break;
    case GS_PSM_CT24:
        value = applyTexa(state.texa, cpsm, GSMem::ReadCT24(m_vram, cbp, clutWidth, clutX, clutY));
        break;
    case GS_PSM_CT16:
        value = applyTexa(state.texa, cpsm, Rgba5551ToRgba8888(GSMem::ReadCT16(m_vram, cbp, clutWidth, clutX, clutY)));
        break;
    case GS_PSM_CT16S:
        value = applyTexa(state.texa, cpsm, Rgba5551ToRgba8888(GSMem::ReadCT16S(m_vram, cbp, clutWidth, clutX, clutY)));
        break;
    default:
        break;
    }

    cached.generation = t_clutGeneration;
    cached.value = value;
    return value;
}

uint32_t GSCpuBackend::SampleTexture(const GSDrawState &state, const DecodedTexture *decoded, float s, float t, float q, uint16_t u, uint16_t v)
{
    const auto &ctx = state.context;
    const auto &tex = ctx.tex0;

    const int texW = state.textureWidth;
    const int texH = state.textureHeight;
    const uint64_t clamp = ctx.clamp;
    const uint8_t wrapU = static_cast<uint8_t>(clamp & 0x3u);
    const uint8_t wrapV = static_cast<uint8_t>((clamp >> 2) & 0x3u);
    const uint16_t minU = static_cast<uint16_t>((clamp >> 4) & 0x3FFu);
    const uint16_t maxU = static_cast<uint16_t>((clamp >> 14) & 0x3FFu);
    const uint16_t minV = static_cast<uint16_t>((clamp >> 24) & 0x3FFu);
    const uint16_t maxV = static_cast<uint16_t>((clamp >> 34) & 0x3FFu);

    float texUf, texVf;
    if (state.prim.fst)
    {
        texUf = static_cast<float>(u) / 16.0f;
        texVf = static_cast<float>(v) / 16.0f;
    }
    else
    {
        const float invQ = 1.0f / fabsQ(q);
        texUf = s * invQ * static_cast<float>(texW);
        texVf = t * invQ * static_cast<float>(texH);
    }

    auto samplePoint = [&](int sampleU, int sampleV) -> uint32_t
    {
        sampleU = wrapTextureCoordinate(sampleU, texW, wrapU, minU, maxU);
        sampleV = wrapTextureCoordinate(sampleV, texH, wrapV, minV, maxV);
        if (decoded && static_cast<uint32_t>(sampleU) < decoded->width && static_cast<uint32_t>(sampleV) < decoded->height)
            return decoded->texels[static_cast<size_t>(sampleV) * decoded->width + static_cast<uint32_t>(sampleU)];

        u32 out = ReadVramUnlocked(tex.psm, tex.tbp0, tex.tbw, sampleU, sampleV);

        switch (tex.psm)
        {
        case GS_PSM_CT32:
        case GS_PSM_Z32:
        case GS_PSM_CT24:
        case GS_PSM_Z24:
            return applyTexa(state.texa, tex.psm, out);
        case GS_PSM_CT16:
        case GS_PSM_CT16S:
        case GS_PSM_Z16:
        case GS_PSM_Z16S:
            return applyTexa(state.texa, tex.psm, Rgba5551ToRgba8888(out));
        case GS_PSM_T8:
        case GS_PSM_T8H:
        case GS_PSM_T4:
        case GS_PSM_T4HL:
        case GS_PSM_T4HH:
            return LookupCLUT(state, static_cast<u8>(out), tex.cbp, tex.cpsm, tex.csm, tex.csa, tex.psm);
        }

        return 0xFFFF00FFu;
    };

    if (!state.linearFilter)
    {
        return samplePoint(static_cast<int>(texUf), static_cast<int>(texVf));
    }

    const float sampleU = texUf - 0.5f;
    const float sampleV = texVf - 0.5f;
    const int u0 = static_cast<int>(std::floor(sampleU));
    const int v0 = static_cast<int>(std::floor(sampleV));
    const int u1 = u0 + 1;
    const int v1 = v0 + 1;
    const float fx = sampleU - static_cast<float>(u0);
    const float fy = sampleV - static_cast<float>(v0);

    const uint32_t c00 = samplePoint(u0, v0);
    const uint32_t c10 = samplePoint(u1, v0);
    const uint32_t c01 = samplePoint(u0, v1);
    const uint32_t c11 = samplePoint(u1, v1);

    const uint8_t r = lerpChannel(static_cast<uint8_t>(c00 & 0xFFu),
                                  static_cast<uint8_t>(c10 & 0xFFu),
                                  static_cast<uint8_t>(c01 & 0xFFu),
                                  static_cast<uint8_t>(c11 & 0xFFu),
                                  fx, fy);
    const uint8_t g = lerpChannel(static_cast<uint8_t>((c00 >> 8) & 0xFFu),
                                  static_cast<uint8_t>((c10 >> 8) & 0xFFu),
                                  static_cast<uint8_t>((c01 >> 8) & 0xFFu),
                                  static_cast<uint8_t>((c11 >> 8) & 0xFFu),
                                  fx, fy);
    const uint8_t b = lerpChannel(static_cast<uint8_t>((c00 >> 16) & 0xFFu),
                                  static_cast<uint8_t>((c10 >> 16) & 0xFFu),
                                  static_cast<uint8_t>((c01 >> 16) & 0xFFu),
                                  static_cast<uint8_t>((c11 >> 16) & 0xFFu),
                                  fx, fy);
    const uint8_t a = lerpChannel(static_cast<uint8_t>((c00 >> 24) & 0xFFu),
                                  static_cast<uint8_t>((c10 >> 24) & 0xFFu),
                                  static_cast<uint8_t>((c01 >> 24) & 0xFFu),
                                  static_cast<uint8_t>((c11 >> 24) & 0xFFu),
                                  fx, fy);

    return static_cast<uint32_t>(r) |
           (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) |
           (static_cast<uint32_t>(a) << 24);
}

void GSCpuBackend::DrawSprite(const GSPrimitiveBatch &batch)
{
    const GSDrawState &state = batch.state;
    const GSVertex &v0 = batch.vertices[0];
    const GSVertex &v1 = batch.vertices[1];
    const auto &ctx = state.context;

    int ofx = ctx.xyoffset.ofx >> 4;
    int ofy = ctx.xyoffset.ofy >> 4;

    int x0 = static_cast<int>(v0.x) - ofx;
    int y0 = static_cast<int>(v0.y) - ofy;
    int x1 = static_cast<int>(v1.x) - ofx;
    int y1 = static_cast<int>(v1.y) - ofy;
    u32 z1 = static_cast<u32>(v1.z);

    if (traceGsNow())
    {
        static uint32_t traced = 0;
        if (traced++ < 4000u)
            std::fprintf(stderr, "[gs] sprite xy=(%.2f,%.2f)-(%.2f,%.2f) of=(%d,%d) uv=(%u,%u)-(%u,%u) st=(%f,%f)-(%f,%f) rgba=%02x%02x%02x%02x tme=%u fst=%u abe=%u tbp=%x tbw=%u psm=%x tw=%u th=%u tcc=%u tfx=%u cbp=%x cpsm=%x csm=%u csa=%u texa=%02x/%02x/%u tex1=%llx alpha=%llx test=%llx fbp=%x fpsm=%x fbmsk=%x\n",
                         v0.x, v0.y, v1.x, v1.y, ofx, ofy, v0.u, v0.v, v1.u, v1.v, v0.s, v0.t, v1.s, v1.t,
                         v1.r, v1.g, v1.b, v1.a,
                         state.prim.tme, state.prim.fst, state.prim.abe,
                         ctx.tex0.tbp0, ctx.tex0.tbw, ctx.tex0.psm, ctx.tex0.tw, ctx.tex0.th, ctx.tex0.tcc, ctx.tex0.tfx,
                         ctx.tex0.cbp, ctx.tex0.cpsm, ctx.tex0.csm, ctx.tex0.csa,
                         state.texa.ta0, state.texa.ta1, state.texa.aem ? 1u : 0u,
                         static_cast<unsigned long long>(ctx.tex1), static_cast<unsigned long long>(ctx.alpha),
                         static_cast<unsigned long long>(ctx.test), ctx.frame.fbp, ctx.frame.psm, ctx.frame.fbmsk);
    }

    // A sprite may be given from either corner; the texture coordinates
    // follow their vertices (text glyphs here come bottom-right first).
    const bool flipX = x0 > x1;
    const bool flipY = y0 > y1;
    if (flipX)
        std::swap(x0, x1);
    if (flipY)
        std::swap(y0, y1);

    const int unclippedX0 = x0;
    const int unclippedY0 = y0;
    const int spanX = std::max(1, x1 - x0);
    const int spanY = std::max(1, y1 - y0);
    const int unclippedX1 = unclippedX0 + spanX - 1;
    const int unclippedY1 = unclippedY0 + spanY - 1;

    // If the sprite rectangle is fully outside scissor, nothing should render.
    if (unclippedX1 < ctx.scissor.x0 || unclippedX0 > ctx.scissor.x1 ||
        unclippedY1 < ctx.scissor.y0 || unclippedY0 > ctx.scissor.y1)
        return;

    const int drawX0 = clampInt(unclippedX0, ctx.scissor.x0, ctx.scissor.x1);
    const int drawY0 = clampInt(unclippedY0, ctx.scissor.y0, ctx.scissor.y1);
    const int drawX1 = clampInt(unclippedX1, ctx.scissor.x0, ctx.scissor.x1);
    const int drawY1 = clampInt(unclippedY1, ctx.scissor.y0, ctx.scissor.y1);

    const uint64_t alphaReg = ctx.alpha;
    const uint8_t alphaMode = static_cast<uint8_t>(alphaReg & 0xFFu);
    const uint8_t alphaFix = static_cast<uint8_t>((alphaReg >> 32) & 0xFFu);

    uint8_t r = v1.r, g = v1.g, b = v1.b, a = v1.a;

    if (state.prim.tme)
    {
        const auto &tex = ctx.tex0;
        const int texW = state.textureWidth;
        const int texH = state.textureHeight;
        const DecodedTexture *decoded = t_useRunTexture ? static_cast<const DecodedTexture *>(t_runTexture)
                                                        : FindTexture(state, true);

        float u0f, v0f, u1f, v1f;
        if (state.prim.fst)
        {
            u0f = static_cast<float>(v0.u >> 4);
            v0f = static_cast<float>(v0.v >> 4);
            u1f = static_cast<float>(v1.u >> 4);
            v1f = static_cast<float>(v1.v >> 4);
        }
        else
        {
            const float q0 = fabsQ(v0.q);
            const float q1 = fabsQ(v1.q);
            u0f = (v0.s / q0) * static_cast<float>(texW);
            v0f = (v0.t / q0) * static_cast<float>(texH);
            u1f = (v1.s / q1) * static_cast<float>(texW);
            v1f = (v1.t / q1) * static_cast<float>(texH);
        }
        if (flipX)
            std::swap(u0f, u1f);
        if (flipY)
            std::swap(v0f, v1f);

        float spriteW = static_cast<float>(spanX);
        float spriteH = static_cast<float>(spanY);
        if (spriteW < 1.0f)
            spriteW = 1.0f;
        if (spriteH < 1.0f)
            spriteH = 1.0f;

        for (int y = drawY0; y <= drawY1; ++y)
        {
            if (!rowInBand(y))
                continue;
            float ty = (static_cast<float>(y - unclippedY0) + 0.5f) / spriteH;
            float texVf = v0f + (v1f - v0f) * ty;

            for (int x = drawX0; x <= drawX1; ++x)
            {
                float tx = (static_cast<float>(x - unclippedX0) + 0.5f) / spriteW;
                float texUf = u0f + (u1f - u0f) * tx;
                uint32_t texel = 0xFFFF00FFu;
                if (state.prim.fst)
                {
                    const int fixedU = static_cast<int>((texUf * 16.0f) + 0.5f);
                    const int fixedV = static_cast<int>((texVf * 16.0f) + 0.5f);
                    const uint16_t sampleU = static_cast<uint16_t>(clampInt(fixedU, 0, 0xFFFF));
                    const uint16_t sampleV = static_cast<uint16_t>(clampInt(fixedV, 0, 0xFFFF));
                    texel = SampleTexture(state, decoded, 0.0f, 0.0f, 1.0f, sampleU, sampleV);
                }
                else
                {
                    texel = SampleTexture(state, decoded, texUf / static_cast<float>(texW), texVf / static_cast<float>(texH), 1.0f, 0u, 0u);
                }

                uint8_t tr = static_cast<uint8_t>(texel & 0xFF);
                uint8_t tg = static_cast<uint8_t>((texel >> 8) & 0xFF);
                uint8_t tb = static_cast<uint8_t>((texel >> 16) & 0xFF);
                uint8_t ta = static_cast<uint8_t>((texel >> 24) & 0xFF);

                const TextureCombineResult color = combineTexture(tex, r, g, b, a, tr, tg, tb, ta);
                WritePixel(state, x, y, z1, color.r, color.g, color.b, color.a, v1.fog);
            }
        }
    }
    else
    {
        for (int y = drawY0; y <= drawY1; ++y)
        {
            if (!rowInBand(y))
                continue;
            for (int x = drawX0; x <= drawX1; ++x)
                WritePixel(state, x, y, z1, r, g, b, a, v1.fog);
        }
    }
}

void GSCpuBackend::DrawTriangle(const GSPrimitiveBatch &batch)
{
    const GSDrawState &state = batch.state;
    const GSVertex &v0 = batch.vertices[0];
    const GSVertex &v1 = batch.vertices[1];
    const GSVertex &v2 = batch.vertices[2];
    const auto &ctx = state.context;

    int ofx = ctx.xyoffset.ofx >> 4;
    int ofy = ctx.xyoffset.ofy >> 4;

    float fx0 = v0.x - static_cast<float>(ofx);
    float fy0 = v0.y - static_cast<float>(ofy);
    float fx1 = v1.x - static_cast<float>(ofx);
    float fy1 = v1.y - static_cast<float>(ofy);
    float fx2 = v2.x - static_cast<float>(ofx);
    float fy2 = v2.y - static_cast<float>(ofy);

    // Traced once, by the band that owns the triangle's top row;
    // PS2X_TRACE_GS_REGION=x0,y0,x1,y1 (frame pixels) keeps those touching it.
    const int traceTop = std::max(static_cast<int>(std::floor(std::min({fy0, fy1, fy2}))), static_cast<int>(ctx.scissor.y0));
    static const char *traceRegion = std::getenv("PS2X_TRACE_GS_REGION");
    static float tr[4] = {-1e9f, -1e9f, 1e9f, 1e9f};
    static const int traceRegionParsed = traceRegion ? std::sscanf(traceRegion, "%f,%f,%f,%f", &tr[0], &tr[1], &tr[2], &tr[3]) : 0;
    (void)traceRegionParsed;
    // PS2X_TRACE_GS_CTXT=<0|1> keeps only primitives drawn with that GS context.
    static const int traceContext = std::getenv("PS2X_TRACE_GS_CTXT") ? std::atoi(std::getenv("PS2X_TRACE_GS_CTXT")) : -1;
    const bool inTraceRegion = std::max({fx0, fx1, fx2}) >= tr[0] && std::min({fx0, fx1, fx2}) <= tr[2] &&
                               std::max({fy0, fy1, fy2}) >= tr[1] && std::min({fy0, fy1, fy2}) <= tr[3] &&
                               (traceContext < 0 || traceContext == (state.prim.ctxt ? 1 : 0));
    if (inTraceRegion && rowInBand(traceTop) && traceGsNow())
    {
        static std::atomic<uint32_t> traced{0};
        // Stored depth at the centroid (racy with other bands; diagnostics only).
        const int cx = static_cast<int>((fx0 + fx1 + fx2) / 3.0f);
        const int cy = static_cast<int>((fy0 + fy1 + fy2) / 3.0f);
        const uint32_t centroidZ = (cx >= 0 && cy >= 0)
                                       ? ReadVramUnlocked(ctx.zbuf.psm, GSInternal::framePageBaseToBlock(ctx.zbuf.zbp),
                                                          std::max<uint32_t>(ctx.frame.fbw, 1u), static_cast<uint32_t>(cx), static_cast<uint32_t>(cy))
                                       : 0u;
        if (traced++ < 20000u)
            std::fprintf(stderr, "[gs] tri @%.2fs zc=%u@(%d,%d) ctxt=%u tme=%u z=%.0f,%.0f,%.0f xy=(%.1f,%.1f)(%.1f,%.1f)(%.1f,%.1f) uv=(%u,%u)(%u,%u)(%u,%u) stq0=(%.3f,%.3f,%.3f) rgba=%02x%02x%02x%02x fst=%u abe=%u tbp=%x tbw=%u psm=%x tw=%u th=%u tcc=%u tfx=%u cbp=%x clamp=%llx tex1=%llx alpha=%llx test=%llx fbp=%x fbw=%u fpsm=%x fbmsk=%x zbp=%x zmsk=%u zpsm=%x fge=%u\n",
                         static_cast<double>(g_ps2xVBlankCount.load(std::memory_order_relaxed)) / 59.94,
                         centroidZ, cx, cy, state.prim.ctxt ? 1u : 0u, state.prim.tme, v0.z, v1.z, v2.z,
                         fx0, fy0, fx1, fy1, fx2, fy2, v0.u, v0.v, v1.u, v1.v, v2.u, v2.v, v0.s, v0.t, v0.q,
                         v0.r, v0.g, v0.b, v0.a, state.prim.fst, state.prim.abe,
                         ctx.tex0.tbp0, ctx.tex0.tbw, ctx.tex0.psm, ctx.tex0.tw, ctx.tex0.th, ctx.tex0.tcc, ctx.tex0.tfx,
                         ctx.tex0.cbp, static_cast<unsigned long long>(ctx.clamp), static_cast<unsigned long long>(ctx.tex1),
                         static_cast<unsigned long long>(ctx.alpha), static_cast<unsigned long long>(ctx.test),
                         ctx.frame.fbp, ctx.frame.fbw, ctx.frame.psm, ctx.frame.fbmsk, ctx.zbuf.zbp, ctx.zbuf.zmask ? 1u : 0u,
                         static_cast<unsigned>(ctx.zbuf.psm), state.prim.fge ? 1u : 0u);
    }

    const DecodedTexture *decoded = !state.prim.tme ? nullptr
                                    : t_useRunTexture ? static_cast<const DecodedTexture *>(t_runTexture)
                                                      : FindTexture(state, true);
    // PS2X_GS_SLOW_TRIANGLES: always use the general per-pixel path below.
    static const bool slowTriangles = std::getenv("PS2X_GS_SLOW_TRIANGLES") != nullptr;
    if (!slowTriangles && drawTriangleFast(m_vram, batch, decoded))
        return;

    int minX = static_cast<int>(std::floor(std::min({fx0, fx1, fx2})));
    int maxX = static_cast<int>(std::ceil(std::max({fx0, fx1, fx2})));
    int minY = static_cast<int>(std::floor(std::min({fy0, fy1, fy2})));
    int maxY = static_cast<int>(std::ceil(std::max({fy0, fy1, fy2})));

    minX = clampInt(minX, ctx.scissor.x0, ctx.scissor.x1);
    maxX = clampInt(maxX, ctx.scissor.x0, ctx.scissor.x1);
    minY = clampInt(minY, ctx.scissor.y0, ctx.scissor.y1);
    maxY = clampInt(maxY, ctx.scissor.y0, ctx.scissor.y1);

    float denom = (fy1 - fy2) * (fx0 - fx2) + (fx2 - fx1) * (fy0 - fy2);
    if (std::fabs(denom) < 0.001f)
        return;

    const float winding = (denom < 0.0f) ? -1.0f : 1.0f;
    const float invAbsDenom = 1.0f / std::fabs(denom);
    constexpr float kEdgeEpsilon = 1.0e-4f;

    // The barycentric weights are linear in x along a row, so each row only
    // needs the span where all three can be inside; the per-pixel test below
    // stays authoritative (the span is widened by a pixel for rounding).
    const float scale = winding * invAbsDenom;
    const float a0 = (fy1 - fy2) * scale;
    const float a1 = (fy2 - fy0) * scale;
    const float a2 = -(a0 + a1);

    for (int y = minY; y <= maxY; ++y)
    {
        if (!rowInBand(y))
            continue;
        float py = static_cast<float>(y) + 0.5f;

        const float c0 = ((fx2 - fx1) * (py - fy2)) * scale - a0 * fx2;
        const float c1 = ((fx0 - fx2) * (py - fy2)) * scale - a1 * fx2;
        const float c2 = 1.0f - c0 - c1;
        float spanMin = static_cast<float>(minX) + 0.5f;
        float spanMax = static_cast<float>(maxX) + 0.5f;
        bool emptyRow = false;
        auto clipSpan = [&](float a, float c)
        {
            if (std::fabs(a) < 1.0e-12f)
            {
                emptyRow |= c < -kEdgeEpsilon;
                return;
            }
            const float bound = (-kEdgeEpsilon - c) / a;
            if (a > 0.0f)
                spanMin = std::max(spanMin, bound);
            else
                spanMax = std::min(spanMax, bound);
        };
        clipSpan(a0, c0);
        clipSpan(a1, c1);
        clipSpan(a2, c2);
        if (emptyRow || !(spanMin <= spanMax + 2.0f))
            continue;
        const int rowMinX = std::max(minX, static_cast<int>(std::floor(spanMin - 0.5f)) - 1);
        const int rowMaxX = std::min(maxX, static_cast<int>(std::ceil(spanMax - 0.5f)) + 1);

        for (int x = rowMinX; x <= rowMaxX; ++x)
        {
            float px = static_cast<float>(x) + 0.5f;

            float w0 = (((fy1 - fy2) * (px - fx2) + (fx2 - fx1) * (py - fy2)) * winding) * invAbsDenom;
            float w1 = (((fy2 - fy0) * (px - fx2) + (fx0 - fx2) * (py - fy2)) * winding) * invAbsDenom;
            float w2 = 1.0f - w0 - w1;

            if (w0 < -kEdgeEpsilon || w1 < -kEdgeEpsilon || w2 < -kEdgeEpsilon)
                continue;

            double z = v0.z * w0 + v1.z * w1 + v2.z * w2;

            uint8_t r, g, b, a;
            if (state.prim.iip)
            {
                r = clampU8(static_cast<int>(v0.r * w0 + v1.r * w1 + v2.r * w2));
                g = clampU8(static_cast<int>(v0.g * w0 + v1.g * w1 + v2.g * w2));
                b = clampU8(static_cast<int>(v0.b * w0 + v1.b * w1 + v2.b * w2));
                a = clampU8(static_cast<int>(v0.a * w0 + v1.a * w1 + v2.a * w2));
            }
            else
            {
                r = v2.r;
                g = v2.g;
                b = v2.b;
                a = v2.a;
            }

            if (state.prim.tme)
            {
                float is, it, iq;
                uint16_t iu, iv;
                if (state.prim.fst)
                {
                    iu = static_cast<uint16_t>(v0.u * w0 + v1.u * w1 + v2.u * w2);
                    iv = static_cast<uint16_t>(v0.v * w0 + v1.v * w1 + v2.v * w2);
                    is = 0.0f;
                    it = 0.0f;
                    iq = 1.0f;
                }
                else
                {
                    // The GS DDA interpolates the homogeneous S, T and Q
                    // values. Texel coordinates are calculated from S/Q and
                    // T/Q only after interpolation.
                    is = v0.s * w0 + v1.s * w1 + v2.s * w2;
                    it = v0.t * w0 + v1.t * w1 + v2.t * w2;
                    iq = v0.q * w0 + v1.q * w1 + v2.q * w2;
                    iu = 0;
                    iv = 0;
                }

                uint32_t texel = SampleTexture(state, decoded, is, it, iq, iu, iv);

                uint8_t tr = static_cast<uint8_t>(texel & 0xFF);
                uint8_t tg = static_cast<uint8_t>((texel >> 8) & 0xFF);
                uint8_t tb = static_cast<uint8_t>((texel >> 16) & 0xFF);
                uint8_t ta = static_cast<uint8_t>((texel >> 24) & 0xFF);

                const auto &tex = ctx.tex0;
                const uint8_t shadeR = r;
                const uint8_t shadeG = g;
                const uint8_t shadeB = b;
                const uint8_t shadeA = a;
                const TextureCombineResult color = combineTexture(tex, shadeR, shadeG, shadeB, shadeA, tr, tg, tb, ta);

                r = color.r;
                g = color.g;
                b = color.b;
                a = color.a;
            }

            const uint8_t fog = clampU8(static_cast<int>(v0.fog * w0 + v1.fog * w1 + v2.fog * w2));
            WritePixel(state, x, y, static_cast<u32>(z + 0.5), r, g, b, a, fog);
        }
    }
}

void GSCpuBackend::DrawLine(const GSPrimitiveBatch &batch)
{
    const GSDrawState &state = batch.state;
    const GSVertex &v0 = batch.vertices[0];
    const GSVertex &v1 = batch.vertices[1];
    const auto &ctx = state.context;

    int ofx = ctx.xyoffset.ofx >> 4;
    int ofy = ctx.xyoffset.ofy >> 4;

    int x0 = static_cast<int>(v0.x) - ofx;
    int y0 = static_cast<int>(v0.y) - ofy;
    int x1 = static_cast<int>(v1.x) - ofx;
    int y1 = static_cast<int>(v1.y) - ofy;

    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    int totalSteps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    if (totalSteps == 0)
        totalSteps = 1;
    int step = 0;

    for (;;)
    {
        float t = static_cast<float>(step) / static_cast<float>(totalSteps);
        uint8_t r, g, b, a;
        if (state.prim.iip)
        {
            r = clampU8(static_cast<int>(v0.r + (v1.r - v0.r) * t));
            g = clampU8(static_cast<int>(v0.g + (v1.g - v0.g) * t));
            b = clampU8(static_cast<int>(v0.b + (v1.b - v0.b) * t));
            a = clampU8(static_cast<int>(v0.a + (v1.a - v0.a) * t));
        }
        else
        {
            r = v1.r;
            g = v1.g;
            b = v1.b;
            a = v1.a;
        }

        double z = (v0.z + (v1.z - v0.z) * t);
        const uint8_t fog = clampU8(static_cast<int>(v0.fog + (v1.fog - v0.fog) * t));
        WritePixel(state, x0, y0, static_cast<u32>(z), r, g, b, a, fog);

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
        ++step;
    }
}

void GSCpuBackend::BeginTransfer(const GSTransferCommand &command)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    // The destination pages are marked modified by the copy itself, and only
    // when it changes them.
    m_transfer = command;
    m_transferState.x = command.trxpos.dsax;
    m_transferState.y = command.trxpos.dsay;
    m_transferState.totalPixels = static_cast<uint32_t>(command.trxreg.rrw) * static_cast<uint32_t>(command.trxreg.rrh);
    m_transferState.copiedPixels = 0u;
    m_transferState.direction = command.direction;
    m_transferState.localToHostPendingBytes = 0u;

    if (traceGsNow())
    {
        static uint32_t traced = 0;
        if (traced++ < 40000u)
            std::fprintf(stderr, "[gs] transfer @%.2fs dir=%u dbp=%x dbw=%u dpsm=%x pos=(%u,%u) size=%ux%u sbp=%x spsm=%x\n",
                         static_cast<double>(g_ps2xVBlankCount.load(std::memory_order_relaxed)) / 59.94,
                         command.direction, command.bitbltbuf.dbp, command.bitbltbuf.dbw, command.bitbltbuf.dpsm,
                         command.trxpos.dsax, command.trxpos.dsay, command.trxreg.rrw, command.trxreg.rrh,
                         command.bitbltbuf.sbp, command.bitbltbuf.spsm);
    }

    if (command.direction == 2u)
        PerformLocalToLocalTransfer();
    else if (command.direction == 1u)
        PerformLocalToHostTransfer();
}

void GSCpuBackend::UploadImage(const uint8_t *data, uint32_t sizeBytes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    static const char *watchEntryEnv = std::getenv("PS2X_WATCH_VRAM");
    if (watchEntryEnv)
    {
        static const uint32_t watch = static_cast<uint32_t>(std::strtoul(watchEntryEnv, nullptr, 16));
        static uint32_t logged = 0;
        if (m_transfer.bitbltbuf.dbp * 256u == watch && logged++ < 200u)
            std::fprintf(stderr, "[gs] @%.2fs image data for watched %x: %u bytes, state direction=%u pixels %u/%u\n",
                         static_cast<double>(g_ps2xVBlankCount.load(std::memory_order_relaxed)) / 59.94, watch, sizeBytes,
                         m_transferState.direction, m_transferState.copiedPixels, m_transferState.totalPixels);
    }
    if (!data || sizeBytes == 0u || !m_vram || m_transferState.direction != 0u)
        return;
    if (m_transfer.trxreg.rrw == 0u || m_transfer.trxreg.rrh == 0u || m_transferState.totalPixels == 0u)
        return;

    const uint32_t dbp = m_transfer.bitbltbuf.dbp;
    const uint32_t dbw = std::max<uint32_t>(m_transfer.bitbltbuf.dbw, 1u);
    const uint8_t dpsm = m_transfer.bitbltbuf.dpsm;
    const uint32_t rrw = m_transfer.trxreg.rrw;
    const uint32_t dsax = m_transfer.trxpos.dsax;
    uint32_t offset = 0u;

    // Games re-upload most textures every frame, usually unchanged; only real
    // changes invalidate decoded textures and CLUTs.
    struct TouchIfChanged
    {
        const GSTransferCommand &transfer;
        bool changed = false;
        ~TouchIfChanged()
        {
            if (!changed)
                return;
            uint32_t begin = 0, end = 0;
            bufferRange(transfer.bitbltbuf.dbp, transfer.bitbltbuf.dbw,
                        static_cast<uint32_t>(transfer.trxpos.dsay) + transfer.trxreg.rrh,
                        psmBits(transfer.bitbltbuf.dpsm), begin, end);
            touchVramRange(begin, end, 3u);
        }
    } touch{m_transfer};
    static const char *watchEnv = std::getenv("PS2X_WATCH_VRAM");
    if (watchEnv)
    {
        static const uint32_t watch = static_cast<uint32_t>(std::strtoul(watchEnv, nullptr, 16));
        static uint32_t logged = 0;
        if (dbp * 256u == watch && logged++ < 200u)
        {
            std::fprintf(stderr, "[gs] @%.2fs upload to watched %x: %u bytes, pixel %u/%u, first words",
                         static_cast<double>(g_ps2xVBlankCount.load(std::memory_order_relaxed)) / 59.94, watch,
                         sizeBytes, m_transferState.copiedPixels, m_transferState.totalPixels);
            for (uint32_t i = 0; i + 4u <= sizeBytes && i < 32u; i += 4u)
            {
                uint32_t word = 0;
                std::memcpy(&word, data + i, 4);
                std::fprintf(stderr, " %08x", word);
            }
            std::fprintf(stderr, "\n");
        }
    }
    const ReadVramFunc readDest = m_readVramFuncs[dpsm & (kPsmHandlerCount - 1u)];
    auto WriteVramUnlocked = [&](uint8_t psm, uint32_t base, uint32_t width, uint32_t x, uint32_t y, uint32_t value)
    {
        if (!touch.changed && readDest(m_vram, base, width, x, y) == value)
            return;
        touch.changed = true;
        this->WriteVramUnlocked(psm, base, width, x, y, value);
    };

    auto advancePixel = [&](uint32_t count)
    {
        const uint32_t totalPixels = m_transferState.totalPixels;
        m_transferState.copiedPixels =
            std::min<uint32_t>(totalPixels, m_transferState.copiedPixels + count);

        if (m_transferState.copiedPixels >= totalPixels)
        {
            m_transferState.direction = 3u;
            m_transferState.totalPixels = 0u;
            return;
        }

        m_transferState.x = dsax + (m_transferState.copiedPixels % rrw);
        m_transferState.y = m_transfer.trxpos.dsay + (m_transferState.copiedPixels / rrw);
    };

    while (offset < sizeBytes && m_transferState.direction == 0u)
    {
        switch (dpsm)
        {
        case GS_PSM_CT32:
        case GS_PSM_Z32:
        {
            if (sizeBytes - offset < 4u)
                return;
            uint32_t value = 0u;
            std::memcpy(&value, data + offset, sizeof(value));
            WriteVramUnlocked(dpsm, dbp, dbw, m_transferState.x, m_transferState.y, value);
            offset += 4u;
            advancePixel(1u);
            break;
        }
        case GS_PSM_CT24:
        case GS_PSM_Z24:
        {
            if (sizeBytes - offset < 3u)
                return;
            const uint32_t value = static_cast<uint32_t>(data[offset]) |
                                   (static_cast<uint32_t>(data[offset + 1u]) << 8u) |
                                   (static_cast<uint32_t>(data[offset + 2u]) << 16u);
            WriteVramUnlocked(dpsm, dbp, dbw, m_transferState.x, m_transferState.y, value);
            offset += 3u;
            advancePixel(1u);
            break;
        }
        case GS_PSM_CT16:
        case GS_PSM_CT16S:
        case GS_PSM_Z16:
        case GS_PSM_Z16S:
        {
            if (sizeBytes - offset < 2u)
                return;
            uint16_t value = 0u;
            std::memcpy(&value, data + offset, sizeof(value));
            WriteVramUnlocked(dpsm, dbp, dbw, m_transferState.x, m_transferState.y, value);
            offset += 2u;
            advancePixel(1u);
            break;
        }
        case GS_PSM_T8:
        case GS_PSM_T8H:
            WriteVramUnlocked(dpsm, dbp, dbw, m_transferState.x, m_transferState.y, data[offset++]);
            advancePixel(1u);
            break;
        case GS_PSM_T4:
        case GS_PSM_T4HL:
        case GS_PSM_T4HH:
        {
            const uint8_t packed = data[offset++];
            const uint32_t firstPixel = m_transferState.copiedPixels;
            WriteVramUnlocked(dpsm, dbp, dbw,
                              dsax + (firstPixel % rrw),
                              m_transfer.trxpos.dsay + (firstPixel / rrw),
                              packed & 0x0Fu);
            if (firstPixel + 1u < m_transferState.totalPixels)
            {
                const uint32_t secondPixel = firstPixel + 1u;
                WriteVramUnlocked(dpsm, dbp, dbw,
                                  dsax + (secondPixel % rrw),
                                  m_transfer.trxpos.dsay + (secondPixel / rrw),
                                  (packed >> 4u) & 0x0Fu);
            }
            advancePixel(std::min<uint32_t>(2u, m_transferState.totalPixels - firstPixel));
            break;
        }
        default:
            return;
        }
    }
}

void GSCpuBackend::PerformLocalToLocalTransfer()
{
    if (!m_vram)
        return;

    const uint32_t rrw = m_transfer.trxreg.rrw;
    const uint32_t rrh = m_transfer.trxreg.rrh;
    const uint32_t total = rrw * rrh;
    if (total == 0u)
    {
        m_transferState.direction = 3u;
        return;
    }

    bool changed = false;
    for (uint32_t pixel = 0; pixel < total; ++pixel)
    {
        uint32_t x = pixel % rrw;
        uint32_t y = pixel / rrw;
        if ((m_transfer.trxpos.dir & 0x2u) != 0u)
            x = rrw - x - 1u;
        if ((m_transfer.trxpos.dir & 0x1u) != 0u)
            y = rrh - y - 1u;

        const uint32_t value = ReadVramUnlocked(m_transfer.bitbltbuf.spsm,
                                                m_transfer.bitbltbuf.sbp,
                                                std::max<uint32_t>(m_transfer.bitbltbuf.sbw, 1u),
                                                x + m_transfer.trxpos.ssax,
                                                y + m_transfer.trxpos.ssay);
        const uint32_t dbw = std::max<uint32_t>(m_transfer.bitbltbuf.dbw, 1u);
        if (!changed && ReadVramUnlocked(m_transfer.bitbltbuf.dpsm, m_transfer.bitbltbuf.dbp, dbw,
                                         x + m_transfer.trxpos.dsax, y + m_transfer.trxpos.dsay) == value)
            continue;
        changed = true;
        WriteVramUnlocked(m_transfer.bitbltbuf.dpsm,
                          m_transfer.bitbltbuf.dbp,
                          dbw,
                          x + m_transfer.trxpos.dsax,
                          y + m_transfer.trxpos.dsay,
                          value);
    }
    if (changed)
    {
        uint32_t begin = 0, end = 0;
        bufferRange(m_transfer.bitbltbuf.dbp, m_transfer.bitbltbuf.dbw,
                    static_cast<uint32_t>(m_transfer.trxpos.dsay) + rrh,
                    psmBits(m_transfer.bitbltbuf.dpsm), begin, end);
        touchVramRange(begin, end, 4u);
    }

    m_transferState.copiedPixels = total;
    m_transferState.direction = 3u;
}

void GSCpuBackend::PerformLocalToHostTransfer()
{
    m_localToHostBuffer.clear();
    m_localToHostReadPos = 0u;
    if (!m_vram)
        return;

    const uint32_t rrw = m_transfer.trxreg.rrw;
    const uint32_t rrh = m_transfer.trxreg.rrh;
    const uint32_t sbw = std::max<uint32_t>(m_transfer.bitbltbuf.sbw, 1u);
    const uint8_t spsm = m_transfer.bitbltbuf.spsm;
    const uint32_t bpp = static_cast<uint32_t>(GSMem::BitsPerPixel(static_cast<GSMem::PixelStorageMode>(spsm)));
    const uint32_t total = rrw * rrh;

    // 32-bit colour or depth from a page boundary (e.g. a whole Z buffer read
    // back every frame): address through the page tables.
    const uint32_t sbp = m_transfer.bitbltbuf.sbp;
    if ((spsm == GS_PSM_CT32 || spsm == GS_PSM_Z32) && (sbp & 31u) == 0u)
    {
        const PageSwizzle &swizzle = pageSwizzle();
        const uint16_t(&page)[32][64] = spsm == GS_PSM_Z32 ? swizzle.z32 : swizzle.c32;
        m_localToHostBuffer.resize(static_cast<size_t>(total) * 4u);
        uint8_t *out = m_localToHostBuffer.data();
        for (uint32_t y = 0; y < rrh; ++y)
            for (uint32_t x = 0; x < rrw; ++x, out += 4)
                std::memcpy(out, m_vram + byteAddress32(page, sbp / 32u, sbw, x + m_transfer.trxpos.ssax, y + m_transfer.trxpos.ssay), 4u);
        m_transferState.copiedPixels = total;
        m_transferState.localToHostPendingBytes = m_localToHostBuffer.size();
        return;
    }

    m_localToHostBuffer.reserve((static_cast<size_t>(total) * bpp + 7u) / 8u);

    for (uint32_t pixel = 0u; pixel < total; ++pixel)
    {
        const uint32_t x = pixel % rrw;
        const uint32_t y = pixel / rrw;
        const uint32_t value = ReadVramUnlocked(spsm,
                                                m_transfer.bitbltbuf.sbp,
                                                sbw,
                                                x + m_transfer.trxpos.ssax,
                                                y + m_transfer.trxpos.ssay);
        switch (bpp)
        {
        case 32:
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value));
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value >> 8u));
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value >> 16u));
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value >> 24u));
            break;
        case 24:
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value));
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value >> 8u));
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value >> 16u));
            break;
        case 16:
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value));
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value >> 8u));
            break;
        case 8:
            m_localToHostBuffer.push_back(static_cast<uint8_t>(value));
            break;
        case 4:
        {
            if ((pixel & 1u) != 0u)
                break;
            uint32_t next = 0u;
            if (pixel + 1u < total)
            {
                const uint32_t nextPixel = pixel + 1u;
                const uint32_t nextX = nextPixel % rrw;
                const uint32_t nextY = nextPixel / rrw;
                next = ReadVramUnlocked(spsm, m_transfer.bitbltbuf.sbp, sbw,
                                        nextX + m_transfer.trxpos.ssax,
                                        nextY + m_transfer.trxpos.ssay);
            }
            m_localToHostBuffer.push_back(static_cast<uint8_t>((value & 0x0Fu) | ((next & 0x0Fu) << 4u)));
            break;
        }
        default:
            break;
        }
    }

    m_transferState.copiedPixels = total;
    m_transferState.localToHostPendingBytes = m_localToHostBuffer.size();
}

uint32_t GSCpuBackend::ConsumeLocalToHostBytes(uint8_t *dst, uint32_t maxBytes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!dst || maxBytes == 0u || m_localToHostReadPos >= m_localToHostBuffer.size())
        return 0u;
    const size_t count = std::min<size_t>(maxBytes, m_localToHostBuffer.size() - m_localToHostReadPos);
    std::memcpy(dst, m_localToHostBuffer.data() + m_localToHostReadPos, count);
    m_localToHostReadPos += count;
    m_transferState.localToHostPendingBytes = m_localToHostBuffer.size() - m_localToHostReadPos;
    return static_cast<uint32_t>(count);
}

bool GSCpuBackend::ClearFramebuffer(const GSContext &context, uint32_t rgba)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    invalidateClutCaches();
    if (!m_vram || context.frame.fbw == 0u)
        return false;

    const uint32_t x0 = context.scissor.x0;
    const uint32_t x1 = std::max<uint32_t>(x0, context.scissor.x1);
    const uint32_t y0 = context.scissor.y0;
    const uint32_t y1 = std::max<uint32_t>(y0, context.scissor.y1);
    uint8_t r = static_cast<uint8_t>(rgba);
    uint8_t g = static_cast<uint8_t>(rgba >> 8u);
    uint8_t b = static_cast<uint8_t>(rgba >> 16u);
    uint8_t a = static_cast<uint8_t>(rgba >> 24u);
    if ((context.fba & 1ull) != 0ull && context.frame.psm != GS_PSM_CT24)
        a |= 0x80u;

    const uint32_t fbp = GSInternal::framePageBaseToBlock(context.frame.fbp);
    const uint32_t fbw = std::max<uint32_t>(context.frame.fbw, 1u);
    if (context.frame.psm == GS_PSM_CT32 || context.frame.psm == GS_PSM_CT24)
    {
        const uint32_t source = static_cast<uint32_t>(r) |
                                (static_cast<uint32_t>(g) << 8u) |
                                (static_cast<uint32_t>(b) << 16u) |
                                (static_cast<uint32_t>(a) << 24u);
        for (uint32_t y = y0; y <= y1; ++y)
            for (uint32_t x = x0; x <= x1; ++x)
            {
                uint32_t pixel = source;
                if (context.frame.fbmsk != 0u)
                {
                    const uint32_t old = ReadVramUnlocked(context.frame.psm, fbp, fbw, x, y);
                    pixel = (pixel & ~context.frame.fbmsk) | (old & context.frame.fbmsk);
                }
                WriteVramUnlocked(context.frame.psm, fbp, fbw, x, y, pixel);
            }
        return true;
    }

    if (context.frame.psm == GS_PSM_CT16 || context.frame.psm == GS_PSM_CT16S)
    {
        const uint16_t source = encodeFramePixelPSMCT16(r, g, b, a);
        const uint16_t mask = static_cast<uint16_t>(context.frame.fbmsk);
        for (uint32_t y = y0; y <= y1; ++y)
            for (uint32_t x = x0; x <= x1; ++x)
            {
                uint16_t pixel = source;
                if (mask != 0u)
                {
                    const uint16_t old = static_cast<uint16_t>(ReadVramUnlocked(context.frame.psm, fbp, fbw, x, y));
                    pixel = static_cast<uint16_t>((pixel & ~mask) | (old & mask));
                }
                WriteVramUnlocked(context.frame.psm, fbp, fbw, x, y, pixel);
            }
        return true;
    }
    return false;
}

bool GSCpuBackend::CopyFrameToHostRgba(const GSFrameReg &frame,
                                       uint32_t width,
                                       uint32_t height,
                                       std::vector<uint8_t> &outPixels,
                                       bool preserveAlpha,
                                       bool useLocalMemoryLayout,
                                       bool frameBaseIsPages,
                                       uint32_t sourceOriginX,
                                       uint32_t sourceOriginY) const
{
    if (!m_vram || m_vramSize == 0u)
        return false;

    outPixels.assign(kHostFrameWidth * kHostFrameHeight * 4u, 0u);
    const uint32_t baseBytes = frameBaseIsPages ? frame.fbp * 8192u : frame.fbp * 256u;
    const uint32_t basePtr = frameBaseIsPages ? GSInternal::framePageBaseToBlock(frame.fbp) : frame.fbp;
    const uint32_t fbw = frame.fbw ? frame.fbw : kHostFrameWidth / 64u;
    const uint32_t bytesPerPixel = (frame.psm == GS_PSM_CT16 || frame.psm == GS_PSM_CT16S) ? 2u : 4u;
    const uint32_t stride = fbw * 64u * bytesPerPixel;

    for (uint32_t y = 0; y < height; ++y)
    {
        uint8_t *dst = outPixels.data() + y * kHostFrameWidth * 4u;
        for (uint32_t x = 0; x < width; ++x)
        {
            const uint32_t sx = sourceOriginX + x;
            const uint32_t sy = sourceOriginY + y;
            if (frame.psm == GS_PSM_CT32 || frame.psm == GS_PSM_CT24)
            {
                uint32_t color = 0u;
                if (useLocalMemoryLayout)
                    color = ReadVramUnlocked(frame.psm, basePtr, fbw, sx, sy);
                else
                {
                    const uint32_t pixelBytes = frame.psm == GS_PSM_CT24 ? 3u : 4u;
                    const uint64_t offset = static_cast<uint64_t>(baseBytes) + static_cast<uint64_t>(sy) * stride + static_cast<uint64_t>(sx) * pixelBytes;
                    if (offset + pixelBytes > m_vramSize)
                        return false;
                    color = m_vram[offset] | (static_cast<uint32_t>(m_vram[offset + 1u]) << 8u) |
                            (static_cast<uint32_t>(m_vram[offset + 2u]) << 16u);
                    if (pixelBytes == 4u)
                        color |= static_cast<uint32_t>(m_vram[offset + 3u]) << 24u;
                }
                dst[x * 4u] = static_cast<uint8_t>(color);
                dst[x * 4u + 1u] = static_cast<uint8_t>(color >> 8u);
                dst[x * 4u + 2u] = static_cast<uint8_t>(color >> 16u);
                dst[x * 4u + 3u] = preserveAlpha && frame.psm != GS_PSM_CT24 ? static_cast<uint8_t>(color >> 24u) : 255u;
            }
            else if (frame.psm == GS_PSM_CT16 || frame.psm == GS_PSM_CT16S)
            {
                uint16_t color = 0u;
                if (useLocalMemoryLayout)
                    color = static_cast<uint16_t>(ReadVramUnlocked(frame.psm, basePtr, fbw, sx, sy));
                else
                {
                    const uint64_t offset = static_cast<uint64_t>(baseBytes) + static_cast<uint64_t>(sy) * stride + static_cast<uint64_t>(sx) * 2u;
                    if (offset + 2u > m_vramSize)
                        return false;
                    std::memcpy(&color, m_vram + offset, sizeof(color));
                }
                const uint32_t r = color & 31u;
                const uint32_t g = (color >> 5u) & 31u;
                const uint32_t b = (color >> 10u) & 31u;
                dst[x * 4u] = static_cast<uint8_t>((r << 3u) | (r >> 2u));
                dst[x * 4u + 1u] = static_cast<uint8_t>((g << 3u) | (g >> 2u));
                dst[x * 4u + 2u] = static_cast<uint8_t>((b << 3u) | (b >> 2u));
                dst[x * 4u + 3u] = preserveAlpha ? ((color & 0x8000u) ? 0x80u : 0u) : 255u;
            }
            else
            {
                outPixels.clear();
                return false;
            }
        }
    }
    return true;
}

PresentationFrame GSCpuBackend::Present(const GSPresentationRequest &request)
{
    // Snapshot local memory under the backend lock, then perform the expensive
    // display conversion without holding the producer-side raster lock.
    thread_local std::vector<uint8_t> snapshot;
    SnapshotVram(snapshot);
    return PresentSnapshot(snapshot, request);
}

PresentationFrame GSCpuBackend::PresentSnapshot(std::vector<uint8_t> &vram, const GSPresentationRequest &request)
{
    if (vram.empty())
        return {};
    thread_local GSCpuBackend snapshotBackend;
    snapshotBackend.Initialize(vram.data(), static_cast<uint32_t>(vram.size()));
    return snapshotBackend.PresentFromLocalMemory(request);
}

PresentationFrame GSCpuBackend::PresentFromLocalMemory(const GSPresentationRequest &request)
{
    PresentationFrame result{};
    const GSPmodeState pmode = decodePmode(request.pmode);
    const GSFrameReg displayFrame1 = decodeDisplayFrame(request.dispfb1);
    const GSFrameReg displayFrame2 = decodeDisplayFrame(request.dispfb2);
    const GSDisplayReadOrigin origin1 = decodeDisplayReadOrigin(request.dispfb1);
    const GSDisplayReadOrigin origin2 = decodeDisplayReadOrigin(request.dispfb2);
    uint32_t width1 = 0u, read1 = 0u, height1 = 0u, width2 = 0u, read2 = 0u, height2 = 0u;
    decodeDisplaySize(request.display1, request.smode2, width1, read1, height1);
    decodeDisplaySize(request.display2, request.smode2, width2, read2, height2);
    const bool valid1 = pmode.enableCrt1 && hasDisplaySetup(request.display1, displayFrame1);
    const bool valid2 = pmode.enableCrt2 && hasDisplaySetup(request.display2, displayFrame2);
    if (!valid1 && !valid2)
        return result;

    auto copySourceRows = [&](const GSFrameReg &displayFrame,
                              const GSDisplayReadOrigin &origin,
                              uint32_t width,
                              uint32_t height,
                              bool allowPreferred,
                              bool preserveAlpha,
                              GSFrameReg &selected,
                              std::vector<uint8_t> &pixels,
                              bool &usedPreferred) -> bool
    {
        selected = displayFrame;
        pixels.clear();
        usedPreferred = false;
        if (allowPreferred && request.hasPreferredSource && request.preferredDestFbp == displayFrame.fbp &&
            (request.preferredSource.fbw != 0u || request.preferredSource.fbp != displayFrame.fbp) &&
            CopyFrameToHostRgba(request.preferredSource, width, height, pixels, preserveAlpha, true, false, 0u, 0u))
        {
            selected = request.preferredSource;
            usedPreferred = true;
        }
        if (pixels.empty() && !CopyFrameToHostRgba(displayFrame, width, height, pixels, preserveAlpha, true, true, origin.x, origin.y))
            return false;

        if (!usedPreferred && displayFrame.fbp == 0u && countNonBlackPixels(pixels, width, height) == 0u)
        {
            for (const GSFrameReg &candidate : request.contextFrames)
            {
                if (candidate.fbp == selected.fbp && candidate.fbw == selected.fbw && candidate.psm == selected.psm)
                    continue;
                std::vector<uint8_t> candidatePixels;
                if (!CopyFrameToHostRgba(candidate, width, height, candidatePixels, preserveAlpha, true, true, 0u, 0u))
                    continue;
                if (countNonBlackPixels(candidatePixels, width, height) == 0u)
                    continue;
                selected = candidate;
                pixels.swap(candidatePixels);
                break;
            }
        }
        return true;
    };

    // Reads readHeight framebuffer rows and stretches them to the output height.
    auto copySource = [&](const GSFrameReg &displayFrame,
                          const GSDisplayReadOrigin &origin,
                          uint32_t width,
                          uint32_t readHeight,
                          uint32_t outHeight,
                          bool allowPreferred,
                          bool preserveAlpha,
                          GSFrameReg &selected,
                          std::vector<uint8_t> &pixels,
                          bool &usedPreferred) -> bool
    {
        if (!copySourceRows(displayFrame, origin, width, readHeight, allowPreferred, preserveAlpha,
                            selected, pixels, usedPreferred))
            return false;
        stretchRows(pixels, width, readHeight, outHeight);
        return true;
    };

    if (valid1 && valid2)
    {
        GSFrameReg selected1{}, selected2{};
        std::vector<uint8_t> crt1, crt2;
        bool preferred1 = false, preferred2 = false;
        if (copySource(displayFrame1, origin1, width1, read1, height1, false, true, selected1, crt1, preferred1) &&
            copySource(displayFrame2, origin2, width2, read2, height2, false, true, selected2, crt2, preferred2))
        {
            result.width = std::max(width1, width2);
            result.height = std::max(height1, height2);
            result.pixels.assign(kHostFrameWidth * kHostFrameHeight * 4u, 0u);
            const uint8_t bgR = static_cast<uint8_t>(request.bgcolor);
            const uint8_t bgG = static_cast<uint8_t>(request.bgcolor >> 8u);
            const uint8_t bgB = static_cast<uint8_t>(request.bgcolor >> 16u);
            for (uint32_t y = 0; y < result.height; ++y)
                for (uint32_t x = 0; x < result.width; ++x)
                {
                    uint8_t *dst = result.pixels.data() + (y * kHostFrameWidth + x) * 4u;
                    dst[0] = bgR;
                    dst[1] = bgG;
                    dst[2] = bgB;
                    dst[3] = pmode.alp;
                }
            if (!pmode.slbg)
                for (uint32_t y = 0; y < height2; ++y)
                    std::memcpy(result.pixels.data() + y * kHostFrameWidth * 4u, crt2.data() + y * kHostFrameWidth * 4u, width2 * 4u);
            for (uint32_t y = 0; y < height1; ++y)
                for (uint32_t x = 0; x < width1; ++x)
                {
                    const uint8_t *src = crt1.data() + (y * kHostFrameWidth + x) * 4u;
                    uint8_t *dst = result.pixels.data() + (y * kHostFrameWidth + x) * 4u;
                    const uint32_t factor = pmode.mmod ? pmode.alp : std::min<uint32_t>(255u, static_cast<uint32_t>(src[3]) * 2u);
                    dst[0] = blendPresentationChannel(src[0], dst[0], factor);
                    dst[1] = blendPresentationChannel(src[1], dst[1], factor);
                    dst[2] = blendPresentationChannel(src[2], dst[2], factor);
                    dst[3] = pmode.amod ? dst[3] : src[3];
                }
            normalizePresentationAlpha(result.pixels, result.width, result.height);
            result.displayFbp = displayFrame1.fbp;
            result.sourceFbp = selected1.fbp;
            return result;
        }
    }

    const GSFrameReg &displayFrame = valid1 ? displayFrame1 : displayFrame2;
    const GSDisplayReadOrigin &origin = valid1 ? origin1 : origin2;
    result.width = valid1 ? width1 : width2;
    result.height = valid1 ? height1 : height2;
    const uint32_t readHeight = valid1 ? read1 : read2;
    GSFrameReg selected = displayFrame;
    if (!copySource(displayFrame, origin, result.width, readHeight, result.height, true, false, selected,
                    result.pixels, result.usedPreferred))
        return {};
    normalizePresentationAlpha(result.pixels, result.width, result.height);
    result.displayFbp = displayFrame.fbp;
    result.sourceFbp = selected.fbp;
    return result;
}
