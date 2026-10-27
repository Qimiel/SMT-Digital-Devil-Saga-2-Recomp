#pragma once

// PS2 Image Processing Unit (IPU) decoder core.
//
// Implements the IPU command set used for MPEG-style intra video: BCLR, IDEC,
// FDEC, SETIQ, SETVQ, SETTH and CSC. Decoding follows the MPEG-1/MPEG-2 intra
// macroblock syntax (ISO/IEC 13818-2 tables B.1, B.2, B.12-B.15) with the IPU's
// colour conversion. The core does not touch guest memory: input arrives
// through IpuInputSource and decoded quadwords are appended to an output queue.
//
// Commands are resumable at macroblock granularity: when the input runs dry a
// command rolls back to the last completed macroblock and stays busy until
// resume() is called with more data available.

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

namespace ps2_ipu
{
    using Qword = std::array<uint8_t, 16>;

    // Supplies bitstream quadwords in order. pull() returns false when no more
    // data is currently available (a DMA transfer has not been started yet).
    class IpuInputSource
    {
    public:
        virtual ~IpuInputSource() = default;
        virtual bool pull(Qword &out) = 0;
    };

    class IpuCore
    {
    public:
        IpuCore();

        void reset();
        void setInput(IpuInputSource *source) { m_source = source; }

        // IPU_CTRL fields written by the guest (bits 16..26).
        void setControl(uint32_t value);
        uint32_t control() const;   // IDP/AS/IVF/QST/MP1/PCT/ECD/SCD/BUSY; FIFO counts are added by the caller
        bool busy() const { return m_busy; }

        // Starts a command written to IPU_CMD. Runs until completion or until
        // the input runs out.
        void command(uint32_t value);
        // Continues a busy command after more input became available.
        void resume();

        uint32_t commandData() const { return m_cmdData; }
        uint32_t top();                       // IPU_TOP: next 32 bits (0 if unavailable)
        uint32_t bitPointer() const { return m_bitPos & 0x7Fu; }
        uint32_t internalQwords() const;      // FP: quadwords held in the bit buffer
        uint32_t queuedQwords() const;        // IFC: quadwords waiting in the input FIFO

        // Input FIFO written directly through IPU_in_FIFO.
        void pushInput(const Qword &qw);

        std::deque<Qword> &output() { return m_output; }

    private:
        struct NeedInput
        {
        };
        struct DecodeError
        {
        };

        enum class Op : uint8_t
        {
            None,
            Idec,
            Fdec,
            SetIq,
            SetVq,
            Csc,
        };

        enum class IdecStage : uint8_t
        {
            Macroblock,
            AddressIncrement,
            Finish,
        };

        // Bit reader over the internal buffer + FIFO (lazily refilled).
        bool ensureBits(uint32_t bits);
        uint32_t peekBits(uint32_t bits);
        uint32_t getBits(uint32_t bits);
        int32_t getSignedBits(uint32_t bits);
        void skipBits(uint32_t bits);
        void commit();
        void refill();

        void run();
        void finishCommand();

        void decodeMacroblock();
        int readAddressIncrement(); // -1 at a start code / end of slice
        void decodeIntraBlock(int component, int16_t coefficients[64]);
        int readDcDifferential(int component);
        void inverseDct(const int16_t in[64], uint8_t *dest, int stride);
        void colourConvert();
        void emitMacroblock();

        IpuInputSource *m_source = nullptr;

        // Bit buffer: m_buffer holds quadwords not yet fully consumed; m_bitPos is
        // the absolute bit position inside m_buffer.front(). Quadwords beyond the
        // first two model the input FIFO.
        std::deque<Qword> m_buffer;
        uint32_t m_bitPos = 0;

        // Registers.
        uint32_t m_idp = 0;   // intra DC precision
        bool m_alternateScan = false;
        bool m_intraVlcFormat = false;
        bool m_nonLinearQ = false;
        bool m_mpeg1 = false;
        uint32_t m_pictureType = 1;
        bool m_ecd = false;
        bool m_scd = false;
        bool m_busy = false;
        uint32_t m_cmdData = 0;

        // Tables loaded by SETIQ / SETVQ / SETTH.
        std::array<uint8_t, 64> m_intraMatrix{};
        std::array<uint8_t, 64> m_nonIntraMatrix{};
        std::array<uint16_t, 16> m_vqClut{};
        uint32_t m_threshold0 = 0;
        uint32_t m_threshold1 = 0;

        // Current command.
        Op m_op = Op::None;
        uint32_t m_args = 0;
        uint32_t m_remaining = 0;     // CSC macroblocks left
        uint32_t m_remainingSkip = 0; // forward bits (FB) not yet skipped

        // IDEC state.
        IdecStage m_stage = IdecStage::Macroblock;
        int m_quantizerScale = 0;
        int m_dcPredictor[3] = {};
        bool m_dctTypeDecode = false;
        bool m_ditherEnable = false;
        bool m_rgb16 = false;

        // Macroblock work buffers.
        uint8_t m_y[16 * 16] = {};
        uint8_t m_cb[8 * 8] = {};
        uint8_t m_cr[8 * 8] = {};
        uint8_t m_rgba[16 * 16 * 4] = {};

        std::deque<Qword> m_output;
    };
}
