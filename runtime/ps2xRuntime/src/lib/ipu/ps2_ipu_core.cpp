#include "ps2_ipu_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace ps2_ipu
{
    namespace
    {
        // ------------------------------------------------------------------
        // Variable length code tables (ISO/IEC 13818-2 annex B).
        // ------------------------------------------------------------------

        struct DctCode
        {
            const char *bits;
            int8_t run;    // -1 = end of block, -2 = escape
            int8_t level;
        };

        constexpr int kEob = -1;
        constexpr int kEscape = -2;

        // Shared tail of tables B.14 and B.15 (codes of 14 to 16 bits).
        constexpr DctCode kDctLongCodes[] = {
            {"00000000011111", 0, 16}, {"00000000011110", 0, 17}, {"00000000011101", 0, 18},
            {"00000000011100", 0, 19}, {"00000000011011", 0, 20}, {"00000000011010", 0, 21},
            {"00000000011001", 0, 22}, {"00000000011000", 0, 23}, {"00000000010111", 0, 24},
            {"00000000010110", 0, 25}, {"00000000010101", 0, 26}, {"00000000010100", 0, 27},
            {"00000000010011", 0, 28}, {"00000000010010", 0, 29}, {"00000000010001", 0, 30},
            {"00000000010000", 0, 31},
            {"000000000011000", 0, 32}, {"000000000010111", 0, 33}, {"000000000010110", 0, 34},
            {"000000000010101", 0, 35}, {"000000000010100", 0, 36}, {"000000000010011", 0, 37},
            {"000000000010010", 0, 38}, {"000000000010001", 0, 39}, {"000000000010000", 0, 40},
            {"000000000011111", 1, 8}, {"000000000011110", 1, 9}, {"000000000011101", 1, 10},
            {"000000000011100", 1, 11}, {"000000000011011", 1, 12}, {"000000000011010", 1, 13},
            {"000000000011001", 1, 14},
            {"0000000000010011", 1, 15}, {"0000000000010010", 1, 16}, {"0000000000010001", 1, 17},
            {"0000000000010000", 1, 18}, {"0000000000010100", 6, 3}, {"0000000000011010", 11, 2},
            {"0000000000011001", 12, 2}, {"0000000000011000", 13, 2}, {"0000000000010111", 14, 2},
            {"0000000000010110", 15, 2}, {"0000000000010101", 16, 2}, {"0000000000011111", 27, 1},
            {"0000000000011110", 28, 1}, {"0000000000011101", 29, 1}, {"0000000000011100", 30, 1},
            {"0000000000011011", 31, 1},
        };

        // Table B.14 for coefficients after the intra DC term.
        constexpr DctCode kDctTableZero[] = {
            {"10", kEob, 0}, {"11", 0, 1}, {"011", 1, 1}, {"0100", 0, 2}, {"0101", 2, 1},
            {"00101", 0, 3}, {"00111", 3, 1}, {"00110", 4, 1}, {"000110", 1, 2}, {"000111", 5, 1},
            {"000101", 6, 1}, {"000100", 7, 1}, {"000001", kEscape, 0}, {"0000110", 0, 4},
            {"0000100", 2, 2}, {"0000111", 8, 1}, {"0000101", 9, 1}, {"00100110", 0, 5},
            {"00100001", 0, 6}, {"00100101", 1, 3}, {"00100100", 3, 2}, {"00100111", 10, 1},
            {"00100011", 11, 1}, {"00100010", 12, 1}, {"00100000", 13, 1}, {"0000001010", 0, 7},
            {"0000001100", 1, 4}, {"0000001011", 2, 3}, {"0000001111", 4, 2}, {"0000001001", 5, 2},
            {"0000001110", 14, 1}, {"0000001101", 15, 1}, {"0000001000", 16, 1},
            {"000000011101", 0, 8}, {"000000011000", 0, 9}, {"000000010011", 0, 10},
            {"000000010000", 0, 11}, {"000000011011", 1, 5}, {"000000010100", 2, 4},
            {"000000011100", 3, 3}, {"000000010010", 4, 3}, {"000000011110", 6, 2},
            {"000000010101", 7, 2}, {"000000010001", 8, 2}, {"000000011111", 17, 1},
            {"000000011010", 18, 1}, {"000000011001", 19, 1}, {"000000010111", 20, 1},
            {"000000010110", 21, 1}, {"0000000011010", 0, 12}, {"0000000011001", 0, 13},
            {"0000000011000", 0, 14}, {"0000000010111", 0, 15}, {"0000000010110", 1, 6},
            {"0000000010101", 1, 7}, {"0000000010100", 2, 5}, {"0000000010011", 3, 4},
            {"0000000010010", 5, 3}, {"0000000010001", 9, 2}, {"0000000010000", 10, 2},
            {"0000000011111", 22, 1}, {"0000000011110", 23, 1}, {"0000000011101", 24, 1},
            {"0000000011100", 25, 1}, {"0000000011011", 26, 1},
        };

        // Table B.15 (intra_vlc_format = 1).
        constexpr DctCode kDctTableOne[] = {
            {"0110", kEob, 0}, {"10", 0, 1}, {"010", 1, 1}, {"110", 0, 2}, {"00101", 2, 1},
            {"0111", 0, 3}, {"00111", 3, 1}, {"000110", 4, 1}, {"00110", 1, 2}, {"000111", 5, 1},
            {"0000110", 6, 1}, {"0000100", 7, 1}, {"11100", 0, 4}, {"0000111", 2, 2},
            {"0000101", 8, 1}, {"1111000", 9, 1}, {"000001", kEscape, 0}, {"11101", 0, 5},
            {"000101", 0, 6}, {"1111001", 1, 3}, {"00100110", 3, 2}, {"1111010", 10, 1},
            {"00100001", 11, 1}, {"00100101", 12, 1}, {"00100100", 13, 1}, {"000100", 0, 7},
            {"00100111", 1, 4}, {"11111100", 2, 3}, {"11111101", 4, 2}, {"000000100", 5, 2},
            {"000000101", 14, 1}, {"000000111", 15, 1}, {"0000001101", 16, 1}, {"1111011", 0, 8},
            {"1111100", 0, 9}, {"00100011", 0, 10}, {"00100010", 0, 11}, {"00100000", 1, 5},
            {"0000001100", 2, 4}, {"000000011100", 3, 3}, {"000000010010", 4, 3},
            {"000000011110", 6, 2}, {"000000010101", 7, 2}, {"000000010001", 8, 2},
            {"000000011111", 17, 1}, {"000000011010", 18, 1}, {"000000011001", 19, 1},
            {"000000010111", 20, 1}, {"000000010110", 21, 1}, {"11111010", 0, 12},
            {"11111011", 0, 13}, {"11111110", 0, 14}, {"11111111", 0, 15},
            {"0000000010110", 1, 6}, {"0000000010101", 1, 7}, {"0000000010100", 2, 5},
            {"0000000010011", 3, 4}, {"0000000010010", 5, 3}, {"0000000010001", 9, 2},
            {"0000000010000", 10, 2}, {"0000000011111", 22, 1}, {"0000000011110", 23, 1},
            {"0000000011101", 24, 1}, {"0000000011100", 25, 1}, {"0000000011011", 26, 1},
        };

        struct SmallCode
        {
            const char *bits;
            int value;
        };

        // Table B.1 macroblock_address_increment; 0 = escape (+33), -1 = stuffing.
        constexpr SmallCode kAddressIncrement[] = {
            {"1", 1}, {"011", 2}, {"010", 3}, {"0011", 4}, {"0010", 5}, {"00011", 6},
            {"00010", 7}, {"0000111", 8}, {"0000110", 9}, {"00001011", 10}, {"00001010", 11},
            {"00001001", 12}, {"00001000", 13}, {"00000111", 14}, {"00000110", 15},
            {"0000010111", 16}, {"0000010110", 17}, {"0000010101", 18}, {"0000010100", 19},
            {"0000010011", 20}, {"0000010010", 21}, {"00000100011", 22}, {"00000100010", 23},
            {"00000100001", 24}, {"00000100000", 25}, {"00000011111", 26}, {"00000011110", 27},
            {"00000011101", 28}, {"00000011100", 29}, {"00000011011", 30}, {"00000011010", 31},
            {"00000011001", 32}, {"00000011000", 33}, {"00000001000", 0}, {"00000001111", -1},
        };

        // Tables B.12 / B.13 dct_dc_size.
        constexpr SmallCode kDcSizeLuma[] = {
            {"100", 0}, {"00", 1}, {"01", 2}, {"101", 3}, {"110", 4}, {"1110", 5}, {"11110", 6},
            {"111110", 7}, {"1111110", 8}, {"11111110", 9}, {"111111110", 10}, {"111111111", 11},
        };
        constexpr SmallCode kDcSizeChroma[] = {
            {"00", 0}, {"01", 1}, {"10", 2}, {"110", 3}, {"1110", 4}, {"11110", 5}, {"111110", 6},
            {"1111110", 7}, {"11111110", 8}, {"111111110", 9}, {"1111111110", 10}, {"1111111111", 11},
        };

        constexpr uint8_t kZigzag[64] = {
            0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
            12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
            35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
            58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};

        constexpr uint8_t kAlternateScan[64] = {
            0, 8, 16, 24, 1, 9, 2, 10, 17, 25, 32, 40, 48, 56, 57, 49,
            41, 33, 26, 18, 3, 11, 4, 12, 19, 27, 34, 42, 50, 58, 35, 43,
            51, 59, 20, 28, 5, 13, 6, 14, 21, 29, 36, 44, 52, 60, 37, 45,
            53, 61, 22, 30, 7, 15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63};

        constexpr uint8_t kNonLinearQuantizerScale[32] = {
            0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 20, 22,
            24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 96, 104, 112};

        constexpr int kDither[4][4] = {{-4, 0, -3, 1}, {2, -2, 3, -1}, {-3, 1, -4, 0}, {3, -1, 2, -2}};

        // Lookup tables indexed by the next N bits.
        struct LookupEntry
        {
            int16_t value = 0;   // run / value
            int8_t level = 0;
            uint8_t length = 0;  // 0 = invalid
        };

        template <size_t Bits>
        struct Lookup
        {
            std::vector<LookupEntry> entries = std::vector<LookupEntry>(size_t{1} << Bits);

            void add(const char *code, int value, int level)
            {
                const size_t length = std::strlen(code);
                uint32_t prefix = 0;
                for (size_t i = 0; i < length; ++i)
                    prefix = (prefix << 1) | (code[i] == '1' ? 1u : 0u);
                const uint32_t shift = static_cast<uint32_t>(Bits - length);
                for (uint32_t tail = 0; tail < (1u << shift); ++tail)
                {
                    LookupEntry &e = entries[(prefix << shift) | tail];
                    e.value = static_cast<int16_t>(value);
                    e.level = static_cast<int8_t>(level);
                    e.length = static_cast<uint8_t>(length);
                }
            }
        };

        struct Tables
        {
            Lookup<16> dctZero;
            Lookup<16> dctOne;
            Lookup<11> addressIncrement;
            Lookup<10> dcLuma;
            Lookup<10> dcChroma;
            float cosine[8][8];

            Tables()
            {
                for (const DctCode &c : kDctTableZero)
                    dctZero.add(c.bits, c.run, c.level);
                for (const DctCode &c : kDctTableOne)
                    dctOne.add(c.bits, c.run, c.level);
                for (const DctCode &c : kDctLongCodes)
                {
                    dctZero.add(c.bits, c.run, c.level);
                    dctOne.add(c.bits, c.run, c.level);
                }
                for (const SmallCode &c : kAddressIncrement)
                    addressIncrement.add(c.bits, c.value, 0);
                for (const SmallCode &c : kDcSizeLuma)
                    dcLuma.add(c.bits, c.value, 0);
                for (const SmallCode &c : kDcSizeChroma)
                    dcChroma.add(c.bits, c.value, 0);
                const double pi = std::acos(-1.0);
                for (int x = 0; x < 8; ++x)
                    for (int u = 0; u < 8; ++u)
                    {
                        const double cu = (u == 0) ? std::sqrt(0.5) : 1.0;
                        cosine[x][u] = static_cast<float>(cu * std::cos((2 * x + 1) * u * pi / 16.0) / 2.0);
                    }
            }
        };

        const Tables &tables()
        {
            static const Tables instance;
            return instance;
        }

        inline uint8_t clampByte(int v)
        {
            return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
        }
    }

    IpuCore::IpuCore()
    {
        (void)tables();
        reset();
    }

    void IpuCore::reset()
    {
        m_buffer.clear();
        m_bitPos = 0;
        m_output.clear();
        m_ecd = m_scd = m_busy = false;
        m_cmdData = 0;
        m_op = Op::None;
    }

    void IpuCore::setControl(uint32_t value)
    {
        m_idp = (value >> 16) & 3u;
        m_alternateScan = ((value >> 20) & 1u) != 0;
        m_intraVlcFormat = ((value >> 21) & 1u) != 0;
        m_nonLinearQ = ((value >> 22) & 1u) != 0;
        m_mpeg1 = ((value >> 23) & 1u) != 0;
        m_pictureType = (value >> 24) & 7u;
    }

    uint32_t IpuCore::control() const
    {
        return (m_ecd ? (1u << 14) : 0u) | (m_scd ? (1u << 15) : 0u) | (m_idp << 16) |
               (m_alternateScan ? (1u << 20) : 0u) | (m_intraVlcFormat ? (1u << 21) : 0u) |
               (m_nonLinearQ ? (1u << 22) : 0u) | (m_mpeg1 ? (1u << 23) : 0u) |
               (m_pictureType << 24) | (m_busy ? (1u << 31) : 0u);
    }

    uint32_t IpuCore::internalQwords() const
    {
        return static_cast<uint32_t>(std::min<size_t>(m_buffer.size(), 2u));
    }

    uint32_t IpuCore::queuedQwords() const
    {
        return static_cast<uint32_t>(m_buffer.size() > 2u ? m_buffer.size() - 2u : 0u);
    }

    void IpuCore::pushInput(const Qword &qw)
    {
        m_buffer.push_back(qw);
        if (m_busy)
            resume();
    }

    // ---------------------------------------------------------------- bits

    bool IpuCore::ensureBits(uint32_t bits)
    {
        while (m_buffer.size() * 128u < m_bitPos + bits)
        {
            Qword qw;
            if (!m_source || !m_source->pull(qw))
                return false;
            m_buffer.push_back(qw);
        }
        return true;
    }

    uint32_t IpuCore::peekBits(uint32_t bits)
    {
        if (bits == 0)
            return 0;
        if (!ensureBits(bits))
            throw NeedInput{};
        uint64_t window = 0;
        uint32_t pos = m_bitPos;
        uint32_t gathered = 0;
        // Gather whole bytes covering [pos, pos + bits).
        const uint32_t firstByte = pos >> 3;
        const uint32_t lastByte = (pos + bits - 1) >> 3;
        for (uint32_t b = firstByte; b <= lastByte; ++b)
        {
            window = (window << 8) | m_buffer[b >> 4][b & 15u];
            gathered += 8;
        }
        const uint32_t trailing = gathered - (pos & 7u) - bits;
        return static_cast<uint32_t>((window >> trailing) & ((uint64_t{1} << bits) - 1u));
    }

    uint32_t IpuCore::getBits(uint32_t bits)
    {
        const uint32_t v = peekBits(bits);
        m_bitPos += bits;
        return v;
    }

    int32_t IpuCore::getSignedBits(uint32_t bits)
    {
        const uint32_t v = getBits(bits);
        const uint32_t sign = 1u << (bits - 1);
        return static_cast<int32_t>((v ^ sign)) - static_cast<int32_t>(sign);
    }

    void IpuCore::skipBits(uint32_t bits)
    {
        if (!ensureBits(bits))
            throw NeedInput{};
        m_bitPos += bits;
    }

    void IpuCore::commit()
    {
        while (m_bitPos >= 128u && !m_buffer.empty())
        {
            m_buffer.pop_front();
            m_bitPos -= 128u;
        }
        refill();
    }

    void IpuCore::refill()
    {
        // The DMA keeps the 2-quadword bit buffer and the 8-quadword FIFO full.
        while (m_buffer.size() < 10u)
        {
            Qword qw;
            if (!m_source || !m_source->pull(qw))
                break;
            m_buffer.push_back(qw);
        }
    }

    uint32_t IpuCore::top()
    {
        try
        {
            return peekBits(32);
        }
        catch (const NeedInput &)
        {
            return 0;
        }
    }

    // ------------------------------------------------------------ commands

    void IpuCore::command(uint32_t value)
    {
        const uint32_t code = value >> 28;
        m_args = value;
        m_ecd = false;
        switch (code)
        {
        case 0: // BCLR
            m_buffer.clear();
            m_bitPos = value & 0x7Fu;
            m_op = Op::None;
            m_busy = false;
            refill();
            return;
        case 1: // IDEC
            m_op = Op::Idec;
            m_scd = false;
            m_stage = IdecStage::Macroblock;
            m_quantizerScale = static_cast<int>((value >> 16) & 0x1Fu);
            m_dctTypeDecode = ((value >> 24) & 1u) != 0;
            m_ditherEnable = ((value >> 26) & 1u) != 0;
            m_rgb16 = ((value >> 27) & 1u) != 0;
            m_dcPredictor[0] = m_dcPredictor[1] = m_dcPredictor[2] = 128 << m_idp;
            break;
        case 4: // FDEC
            m_op = Op::Fdec;
            break;
        case 5: // SETIQ
            m_op = Op::SetIq;
            break;
        case 6: // SETVQ
            m_op = Op::SetVq;
            break;
        case 7: // CSC
            m_op = Op::Csc;
            m_remaining = value & 0x7FFu;
            m_ditherEnable = ((value >> 26) & 1u) != 0;
            m_rgb16 = ((value >> 27) & 1u) != 0;
            break;
        case 9: // SETTH
            m_threshold0 = value & 0x1FFu;
            m_threshold1 = (value >> 16) & 0x1FFu;
            m_op = Op::None;
            m_busy = false;
            return;
        default:
        {
            static uint32_t warned = 0;
            if (warned < 8u)
            {
                ++warned;
                std::fprintf(stderr, "[ipu] unimplemented command %u (0x%08x)\n", code, value);
            }
            m_op = Op::None;
            m_busy = false;
            return;
        }
        }

        // Commands with a forward-bits field skip it before executing.
        m_remainingSkip = (m_op == Op::Csc) ? 0u : (value & 0x3Fu);
        m_busy = true;
        run();
    }

    void IpuCore::resume()
    {
        if (m_busy)
            run();
    }

    void IpuCore::finishCommand()
    {
        m_op = Op::None;
        m_busy = false;
        commit();
    }

    void IpuCore::run()
    {
        try
        {
            if (m_remainingSkip != 0u)
            {
                skipBits(m_remainingSkip);
                m_remainingSkip = 0;
                commit();
            }

            switch (m_op)
            {
            case Op::Fdec:
                m_cmdData = peekBits(32);
                finishCommand();
                return;

            case Op::SetIq:
            {
                std::array<uint8_t, 64> &matrix = ((m_args >> 27) & 1u) ? m_nonIntraMatrix : m_intraMatrix;
                if (!ensureBits(64u * 8u))
                    throw NeedInput{};
                for (uint8_t &v : matrix)
                    v = static_cast<uint8_t>(getBits(8));
                finishCommand();
                return;
            }

            case Op::SetVq:
                if (!ensureBits(32u * 8u))
                    throw NeedInput{};
                for (uint16_t &entry : m_vqClut)
                {
                    const uint32_t lo = getBits(8);
                    const uint32_t hi = getBits(8);
                    entry = static_cast<uint16_t>(lo | (hi << 8));
                }
                finishCommand();
                return;

            case Op::Csc:
                while (m_remaining != 0u)
                {
                    if (!ensureBits(384u * 8u))
                        throw NeedInput{};
                    for (uint8_t &v : m_y)
                        v = static_cast<uint8_t>(getBits(8));
                    for (uint8_t &v : m_cb)
                        v = static_cast<uint8_t>(getBits(8));
                    for (uint8_t &v : m_cr)
                        v = static_cast<uint8_t>(getBits(8));
                    colourConvert();
                    emitMacroblock();
                    commit();
                    --m_remaining;
                }
                finishCommand();
                return;

            case Op::Idec:
                for (;;)
                {
                    const uint32_t checkpointBits = m_bitPos;
                    const int savedDc[3] = {m_dcPredictor[0], m_dcPredictor[1], m_dcPredictor[2]};
                    const int savedQuant = m_quantizerScale;
                    try
                    {
                        if (m_stage == IdecStage::Macroblock)
                        {
                            decodeMacroblock();
                            emitMacroblock();
                            m_stage = IdecStage::AddressIncrement;
                            commit();
                        }
                        else if (m_stage == IdecStage::AddressIncrement)
                        {
                            const int increment = readAddressIncrement();
                            if (increment < 0)
                            {
                                m_stage = IdecStage::Finish;
                            }
                            else
                            {
                                if (increment > 1)
                                    m_dcPredictor[0] = m_dcPredictor[1] = m_dcPredictor[2] = 128 << m_idp;
                                m_stage = IdecStage::Macroblock;
                                commit();
                            }
                        }
                        else
                        {
                            // A zero byte means a start code follows: align to it.
                            if (peekBits(8) == 0u)
                            {
                                m_bitPos = (m_bitPos + 7u) & ~7u;
                                m_scd = true;
                            }
                            (void)peekBits(32);
                            finishCommand();
                            return;
                        }
                    }
                    catch (const NeedInput &)
                    {
                        m_bitPos = checkpointBits;
                        std::copy(savedDc, savedDc + 3, m_dcPredictor);
                        m_quantizerScale = savedQuant;
                        throw;
                    }
                }

            default:
                finishCommand();
                return;
            }
        }
        catch (const NeedInput &)
        {
            // Stay busy; resume() continues once more input arrives.
        }
        catch (const DecodeError &)
        {
            static uint32_t warned = 0;
            if (warned < 8u)
            {
                ++warned;
                std::fprintf(stderr, "[ipu] bitstream error in command 0x%08x\n", m_args);
            }
            m_ecd = true;
            finishCommand();
        }
    }

    // ---------------------------------------------------------- macroblocks

    int IpuCore::readAddressIncrement()
    {
        const Tables &t = tables();
        int increment = 0;
        for (;;)
        {
            const uint32_t code = peekBits(11);
            const LookupEntry &e = t.addressIncrement.entries[code];
            if (e.length == 0)
                return -1; // start code or end of slice
            skipBits(e.length);
            if (e.value == 0) // macroblock_escape
            {
                increment += 33;
                continue;
            }
            if (e.value < 0) // stuffing
                continue;
            return increment + e.value;
        }
    }

    void IpuCore::decodeMacroblock()
    {
        // macroblock_type for an intra picture (table B.2).
        bool quant = false;
        if (peekBits(1))
        {
            skipBits(1);
        }
        else if (peekBits(2) == 1u)
        {
            skipBits(2);
            quant = true;
        }
        else
        {
            throw DecodeError{};
        }

        const bool fieldDct = m_dctTypeDecode ? (getBits(1) != 0u) : false;
        if (quant)
        {
            const uint32_t code = getBits(5);
            m_quantizerScale = m_nonLinearQ ? kNonLinearQuantizerScale[code] : static_cast<int>(code << 1);
        }

        int16_t block[64];
        for (int b = 0; b < 4; ++b)
        {
            decodeIntraBlock(0, block);
            const int x = (b & 1) * 8;
            if (fieldDct)
                inverseDct(block, m_y + (b >> 1) * 16 + x, 32);
            else
                inverseDct(block, m_y + (b >> 1) * 8 * 16 + x, 16);
        }
        decodeIntraBlock(1, block);
        inverseDct(block, m_cb, 8);
        decodeIntraBlock(2, block);
        inverseDct(block, m_cr, 8);
        colourConvert();
    }

    int IpuCore::readDcDifferential(int component)
    {
        const Tables &t = tables();
        const uint32_t code = peekBits(10);
        const LookupEntry &e = component == 0 ? t.dcLuma.entries[code] : t.dcChroma.entries[code];
        if (e.length == 0)
            throw DecodeError{};
        skipBits(e.length);
        const int size = e.value;
        if (size == 0)
            return 0;
        const int diff = static_cast<int>(getBits(static_cast<uint32_t>(size)));
        return (diff < (1 << (size - 1))) ? diff - (1 << size) + 1 : diff;
    }

    void IpuCore::decodeIntraBlock(int component, int16_t coefficients[64])
    {
        std::memset(coefficients, 0, sizeof(int16_t) * 64);
        m_dcPredictor[component] += readDcDifferential(component);
        int sum = coefficients[0] = static_cast<int16_t>(m_dcPredictor[component] << (3 - m_idp));

        const Tables &t = tables();
        const Lookup<16> &table = (m_intraVlcFormat && !m_mpeg1) ? t.dctOne : t.dctZero;
        const uint8_t *scan = m_alternateScan ? kAlternateScan : kZigzag;
        int i = 0;
        for (;;)
        {
            const LookupEntry &e = table.entries[peekBits(16)];
            if (e.length == 0)
                throw DecodeError{};
            skipBits(e.length);
            if (e.value == kEob)
                break;

            int run;
            int level;
            if (e.value == kEscape)
            {
                run = static_cast<int>(getBits(6));
                if (m_mpeg1)
                {
                    level = getSignedBits(8);
                    if (level == 0)
                        level = static_cast<int>(getBits(8));
                    else if (level == -128)
                        level = static_cast<int>(getBits(8)) - 256;
                }
                else
                {
                    level = getSignedBits(12);
                }
            }
            else
            {
                run = e.value;
                level = getBits(1) ? -e.level : e.level;
            }

            i += run + 1;
            if (i > 63)
                throw DecodeError{};

            const int magnitude = (std::abs(level) * m_quantizerScale * m_intraMatrix[i]) >> 4;
            int value = level < 0 ? -magnitude : magnitude;
            if (m_mpeg1 && value != 0)
                value = (value > 0) ? ((value - 1) | 1) : -(((-value) - 1) | 1);
            value = std::clamp(value, -2048, 2047);
            coefficients[scan[i]] = static_cast<int16_t>(value);
            sum += value;
        }

        if (!m_mpeg1 && (sum & 1) == 0)
            coefficients[63] = static_cast<int16_t>(coefficients[63] ^ 1);
    }

    void IpuCore::inverseDct(const int16_t in[64], uint8_t *dest, int stride)
    {
        const Tables &t = tables();
        float tmp[64];
        // Rows: tmp[v][x] = sum_u C(u) F(v,u) cos(...)
        for (int v = 0; v < 8; ++v)
            for (int x = 0; x < 8; ++x)
            {
                float s = 0.0f;
                for (int u = 0; u < 8; ++u)
                    s += t.cosine[x][u] * static_cast<float>(in[v * 8 + u]);
                tmp[v * 8 + x] = s;
            }
        for (int x = 0; x < 8; ++x)
            for (int y = 0; y < 8; ++y)
            {
                float s = 0.0f;
                for (int v = 0; v < 8; ++v)
                    s += t.cosine[y][v] * tmp[v * 8 + x];
                // Round half away from zero (std::lround) without the libm call.
                dest[y * stride + x] = clampByte(static_cast<int>(s + (s < 0.0f ? -0.5f : 0.5f)));
            }
    }

    void IpuCore::colourConvert()
    {
        // IPU YCbCr -> RGB coefficients (in 1/128 units): Y 1.164, Cr->R 1.594,
        // Cr->G -0.8125, Cb->G -0.391, Cb->B 2.016.
        for (int y = 0; y < 16; ++y)
            for (int x = 0; x < 16; ++x)
            {
                const int c = (y >> 1) * 8 + (x >> 1);
                const int lum = (0x95 * std::max(0, static_cast<int>(m_y[y * 16 + x]) - 16)) >> 6;
                const int cr = static_cast<int>(m_cr[c]) - 128;
                const int cb = static_cast<int>(m_cb[c]) - 128;
                const int rcr = (0xCC * cr) >> 6;
                const int gcr = (-0x68 * cr) >> 6;
                const int gcb = (-0x32 * cb) >> 6;
                const int bcb = (0x102 * cb) >> 6;
                uint8_t *p = m_rgba + (y * 16 + x) * 4;
                p[0] = clampByte((lum + rcr + 1) >> 1);
                p[1] = clampByte((lum + gcr + gcb + 1) >> 1);
                p[2] = clampByte((lum + bcb + 1) >> 1);
                p[3] = 0x80;
                if (m_threshold0 > 0u && p[0] < m_threshold0 && p[1] < m_threshold0 && p[2] < m_threshold0)
                    p[0] = p[1] = p[2] = p[3] = 0;
                else if (m_threshold1 > 0u && p[0] < m_threshold1 && p[1] < m_threshold1 && p[2] < m_threshold1)
                    p[3] = 0x40;
            }
    }

    void IpuCore::emitMacroblock()
    {
        Qword qw{};
        if (!m_rgb16)
        {
            for (int i = 0; i < 64; ++i)
            {
                std::memcpy(qw.data(), m_rgba + i * 16, 16);
                m_output.push_back(qw);
            }
            return;
        }
        uint8_t packed[16 * 16 * 2];
        for (int y = 0; y < 16; ++y)
            for (int x = 0; x < 16; ++x)
            {
                const uint8_t *p = m_rgba + (y * 16 + x) * 4;
                const int d = m_ditherEnable ? kDither[y & 3][x & 3] : 0;
                const uint32_t r = clampByte(p[0] + d) >> 3;
                const uint32_t g = clampByte(p[1] + d) >> 3;
                const uint32_t b = clampByte(p[2] + d) >> 3;
                const uint32_t a = p[3] == 0x40 ? 1u : 0u;
                const uint16_t v = static_cast<uint16_t>(r | (g << 5) | (b << 10) | (a << 15));
                packed[(y * 16 + x) * 2] = static_cast<uint8_t>(v);
                packed[(y * 16 + x) * 2 + 1] = static_cast<uint8_t>(v >> 8);
            }
        for (int i = 0; i < 32; ++i)
        {
            std::memcpy(qw.data(), packed + i * 16, 16);
            m_output.push_back(qw);
        }
    }
}
