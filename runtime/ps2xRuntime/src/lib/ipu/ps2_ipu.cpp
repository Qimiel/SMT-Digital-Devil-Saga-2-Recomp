#include "runtime/ps2_ipu.h"

#include "ps2_ipu_core.h"
#include "runtime/ps2_memory.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

namespace ps2_ipu
{
    namespace
    {
        constexpr uint32_t kIpuCmd = 0x10002000u;
        constexpr uint32_t kIpuCtrl = 0x10002010u;
        constexpr uint32_t kIpuBp = 0x10002020u;
        constexpr uint32_t kIpuTop = 0x10002030u;
        constexpr uint32_t kIpuOutFifo = 0x10007000u;
        constexpr uint32_t kIpuInFifo = 0x10007010u;
        constexpr uint32_t kDStat = 0x1000E010u;

        bool traceEnabled()
        {
            static const bool enabled = std::getenv("PS2X_TRACE_IPU") != nullptr;
            return enabled;
        }

#define IPU_TRACE(...)                           \
    do                                           \
    {                                            \
        if (traceEnabled())                      \
            std::fprintf(stderr, __VA_ARGS__);   \
    } while (0)

        constexpr uint32_t kChcr = 0x00u;
        constexpr uint32_t kMadr = 0x10u;
        constexpr uint32_t kQwc = 0x20u;
        constexpr uint32_t kTadr = 0x30u;
        constexpr uint32_t kChcrStart = 0x100u;

        // Reads one quadword of guest memory (RDRAM or scratchpad).
        bool readGuestQword(PS2Memory &memory, uint32_t address, Qword &out)
        {
            try
            {
                const bool scratch = memory.isScratchpad(address);
                const uint32_t phys = memory.translateAddress(address);
                const uint8_t *base = scratch ? memory.getScratchpad() : memory.getRDRAM();
                const uint32_t limit = scratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
                if (!base || phys + 16u > limit)
                    return false;
                std::memcpy(out.data(), base + phys, 16);
                return true;
            }
            catch (...)
            {
                return false;
            }
        }

        bool writeGuestQword(PS2Memory &memory, uint32_t address, const Qword &qw)
        {
            try
            {
                const bool scratch = memory.isScratchpad(address);
                const uint32_t phys = memory.translateAddress(address);
                uint8_t *base = scratch ? memory.getScratchpad() : memory.getRDRAM();
                const uint32_t limit = scratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
                if (!base || phys + 16u > limit)
                    return false;
                std::memcpy(base + phys, qw.data(), 16);
                return true;
            }
            catch (...)
            {
                return false;
            }
        }

        void raiseDmaCompletion(PS2Memory &memory, uint32_t channelBase, uint32_t channelIndex)
        {
            IPU_TRACE("[ipu] dma ch%u complete madr=%08x\n", channelIndex, memory.m_ioRegisters[channelBase + kMadr]);
            memory.m_ioRegisters[channelBase + kChcr] &= ~kChcrStart;
            uint32_t dstat = memory.m_ioRegisters[kDStat];
            dstat |= 1u << channelIndex;
            const uint32_t status = dstat & 0x3FFu;
            const uint32_t mask = (dstat >> 16) & 0x3FFu;
            dstat = (status & mask) ? (dstat | (1u << 31)) : (dstat & ~(1u << 31));
            memory.m_ioRegisters[kDStat] = dstat;
            memory.queueCompletedDmacCause(channelIndex);
        }

        // DMA channel 4 (memory -> IPU) as a lazily pulled input source.
        class ToIpuDma final : public IpuInputSource
        {
        public:
            PS2Memory *memory = nullptr;
            bool active = false;

            void start()
            {
                const uint32_t chcr = reg(kChcr);
                m_chain = ((chcr >> 2) & 3u) == 1u;
                m_tie = (chcr & 0x80u) != 0u;
                m_lastTag = false;
                active = true;
                if (!m_chain && reg(kQwc) == 0u)
                    complete();
            }

            bool pull(Qword &out) override
            {
                if (!active || !memory)
                    return false;
                while (reg(kQwc) == 0u)
                {
                    if (!m_chain || m_lastTag || !readTag())
                    {
                        complete();
                        return false;
                    }
                }
                const uint32_t madr = reg(kMadr);
                if (!readGuestQword(*memory, madr, out))
                {
                    complete();
                    return false;
                }
                reg(kMadr) = madr + 16u;
                reg(kQwc) = reg(kQwc) - 1u;
                if (reg(kQwc) == 0u && (!m_chain || m_lastTag))
                    complete();
                return true;
            }

        private:
            uint32_t &reg(uint32_t offset) { return memory->m_ioRegisters[kToIpuChannel + offset]; }

            bool readTag()
            {
                const uint32_t tadr = reg(kTadr);
                Qword tagQw;
                if (!readGuestQword(*memory, tadr, tagQw))
                    return false;
                uint64_t tag = 0;
                std::memcpy(&tag, tagQw.data(), 8);
                const uint32_t qwc = static_cast<uint32_t>(tag & 0xFFFFu);
                const uint32_t id = static_cast<uint32_t>((tag >> 28) & 7u);
                const uint32_t addr = static_cast<uint32_t>((tag >> 32) & 0x7FFFFFF0u);
                const bool irq = ((tag >> 31) & 1u) != 0u;
                uint32_t &chcr = reg(kChcr);
                chcr = (chcr & 0xFFFFu) | (static_cast<uint32_t>(tag >> 16) & 0xFFFF0000u);
                switch (id)
                {
                case 0: // refe
                    reg(kMadr) = addr;
                    reg(kTadr) = tadr + 16u;
                    m_lastTag = true;
                    break;
                case 1: // cnt
                    reg(kMadr) = tadr + 16u;
                    reg(kTadr) = tadr + 16u + qwc * 16u;
                    break;
                case 2: // next
                    reg(kMadr) = tadr + 16u;
                    reg(kTadr) = addr;
                    break;
                case 3: // ref
                case 4: // refs
                    reg(kMadr) = addr;
                    reg(kTadr) = tadr + 16u;
                    break;
                case 7: // end
                    reg(kMadr) = tadr + 16u;
                    reg(kTadr) = tadr + 16u + qwc * 16u;
                    m_lastTag = true;
                    break;
                default:
                {
                    static uint32_t warned = 0;
                    if (warned < 4u)
                    {
                        ++warned;
                        std::fprintf(stderr, "[ipu] unsupported toIPU DMA tag id %u at 0x%08x\n", id, tadr);
                    }
                    return false;
                }
                }
                if (irq && m_tie)
                    m_lastTag = true;
                reg(kQwc) = qwc;
                return true;
            }

            void complete()
            {
                if (!active)
                    return;
                active = false;
                raiseDmaCompletion(*memory, kToIpuChannel, 4u);
            }

            bool m_chain = false;
            bool m_tie = false;
            bool m_lastTag = false;
        };

        struct IpuState
        {
            std::recursive_mutex mutex;
            IpuCore core;
            ToIpuDma toIpu;
            bool fromIpuActive = false;
            uint32_t fromIpuStart = 0;    // diagnostics: start of the current ch3 transfer
            uint32_t fromIpuQwords = 0;
            uint32_t inFifoWords[4] = {};
            Qword outLatch{};
            bool outLatchValid = false;

            IpuState() { core.setInput(&toIpu); }
        };

        IpuState &state()
        {
            static IpuState instance;
            return instance;
        }

        // Moves decoded quadwords into an active channel 3 transfer.
        void drainOutput(PS2Memory &memory, IpuState &s)
        {
            if (!s.fromIpuActive)
                return;
            uint32_t &madr = memory.m_ioRegisters[kFromIpuChannel + kMadr];
            uint32_t &qwc = memory.m_ioRegisters[kFromIpuChannel + kQwc];
            auto &out = s.core.output();
            while (qwc != 0u && !out.empty())
            {
                writeGuestQword(memory, madr, out.front());
                out.pop_front();
                madr += 16u;
                --qwc;
            }
            if (qwc == 0u)
            {
                s.fromIpuActive = false;
                if (traceEnabled() && memory.getRDRAM())
                {
                    uint64_t sum = 0;
                    const uint8_t *base = memory.getRDRAM() + (s.fromIpuStart & (PS2_RAM_SIZE - 1u));
                    const uint64_t bytes = std::min<uint64_t>(uint64_t{s.fromIpuQwords} * 16u, PS2_RAM_SIZE - (s.fromIpuStart & (PS2_RAM_SIZE - 1u)));
                    for (uint64_t i = 0; i < bytes; i += 4)
                        sum += base[i] + base[i + 1] + base[i + 2];
                    IPU_TRACE("[ipu] picture at %08x: mean rgb %.1f\n", s.fromIpuStart,
                              bytes ? static_cast<double>(sum) / (bytes / 4 * 3) : 0.0);
                    // PS2X_IPU_DUMP=<n>: write the n-th decoded picture (raw IDEC
                    // output, macroblock order) to ipu_picture.bin.
                    static int pictureIndex = 0;
                    static const int dumpIndex = std::getenv("PS2X_IPU_DUMP") ? std::atoi(std::getenv("PS2X_IPU_DUMP")) : -1;
                    if (pictureIndex++ == dumpIndex)
                    {
                        if (FILE *f = std::fopen("ipu_picture.bin", "wb"))
                        {
                            std::fwrite(base, 1, static_cast<size_t>(bytes), f);
                            std::fclose(f);
                        }
                    }
                }
                raiseDmaCompletion(memory, kFromIpuChannel, 3u);
            }
        }

        void bindMemory(PS2Memory &memory, IpuState &s)
        {
            s.toIpu.memory = &memory;
        }
    }

    bool isIpuRegister(uint32_t physAddr)
    {
        return (physAddr >= kIpuCmd && physAddr < kIpuCmd + 0x40u) ||
               (physAddr >= kIpuOutFifo && physAddr < kIpuOutFifo + 0x20u);
    }

    uint32_t readRegister(PS2Memory &memory, uint32_t physAddr)
    {
        IpuState &s = state();
        std::lock_guard<std::recursive_mutex> lock(s.mutex);
        bindMemory(memory, s);
        const uint32_t ifc = std::min<uint32_t>(s.core.queuedQwords(), 15u);
        switch (physAddr & ~3u)
        {
        case kIpuCmd:
            return s.core.commandData();
        case kIpuCmd + 4u:
            return s.core.busy() ? 0x80000000u : 0u;
        case kIpuCtrl:
        {
            const uint32_t ofc = static_cast<uint32_t>(std::min<size_t>(s.core.output().size(), 8u));
            return s.core.control() | ifc | (ofc << 4);
        }
        case kIpuBp:
            return s.core.bitPointer() | (ifc << 8) | (s.core.internalQwords() << 16);
        case kIpuTop:
            return s.core.top();
        case kIpuTop + 4u:
            return s.core.busy() ? 0x80000000u : 0u;
        default:
            break;
        }
        if (physAddr >= kIpuOutFifo && physAddr < kIpuOutFifo + 16u)
        {
            const uint32_t word = (physAddr - kIpuOutFifo) >> 2;
            if (word == 0u)
            {
                s.outLatchValid = !s.core.output().empty();
                if (s.outLatchValid)
                {
                    s.outLatch = s.core.output().front();
                    s.core.output().pop_front();
                }
                else
                {
                    s.outLatch.fill(0);
                }
            }
            uint32_t value = 0;
            std::memcpy(&value, s.outLatch.data() + word * 4u, 4);
            return value;
        }
        return 0u;
    }

    void writeRegister(PS2Memory &memory, uint32_t physAddr, uint32_t value)
    {
        IpuState &s = state();
        std::lock_guard<std::recursive_mutex> lock(s.mutex);
        bindMemory(memory, s);
        switch (physAddr & ~3u)
        {
        case kIpuCmd:
            s.core.command(value);
            drainOutput(memory, s);
            IPU_TRACE("[ipu] cmd %08x -> busy=%d data=%08x bp=%u fp=%u ifc=%u out=%zu\n", value, s.core.busy() ? 1 : 0,
                      s.core.commandData(), s.core.bitPointer(), s.core.internalQwords(), s.core.queuedQwords(),
                      s.core.output().size());
            return;
        case kIpuCtrl:
            IPU_TRACE("[ipu] ctrl <- %08x\n", value);
            if (value & (1u << 30))
            {
                s.core.reset();
                s.outLatchValid = false;
            }
            s.core.setControl(value);
            return;
        default:
            break;
        }
        if (physAddr >= kIpuInFifo && physAddr < kIpuInFifo + 16u)
        {
            const uint32_t word = (physAddr - kIpuInFifo) >> 2;
            s.inFifoWords[word] = value;
            if (word == 3u)
            {
                Qword qw;
                std::memcpy(qw.data(), s.inFifoWords, 16);
                s.core.pushInput(qw);
                drainOutput(memory, s);
            }
        }
    }

    bool isIpuDmaChannel(uint32_t channelBase)
    {
        return channelBase == kFromIpuChannel || channelBase == kToIpuChannel;
    }

    void writeDmaChcr(PS2Memory &memory, uint32_t channelBase, uint32_t value)
    {
        IpuState &s = state();
        std::lock_guard<std::recursive_mutex> lock(s.mutex);
        bindMemory(memory, s);
        const bool start = (value & kChcrStart) != 0u;
        IPU_TRACE("[ipu] ch%u chcr <- %08x madr=%08x qwc=%x tadr=%08x\n", channelBase == kToIpuChannel ? 4u : 3u, value,
                  memory.m_ioRegisters[channelBase + kMadr], memory.m_ioRegisters[channelBase + kQwc],
                  memory.m_ioRegisters[channelBase + kTadr]);
        if (channelBase == kToIpuChannel)
        {
            if (!start)
            {
                s.toIpu.active = false;
                return;
            }
            s.toIpu.start();
            s.core.resume();
        }
        else
        {
            s.fromIpuActive = start;
            if (!start)
                return;
            s.fromIpuStart = memory.m_ioRegisters[kFromIpuChannel + kMadr];
            s.fromIpuQwords = memory.m_ioRegisters[kFromIpuChannel + kQwc];
        }
        drainOutput(memory, s);
    }

    uint32_t readDmaChcr(PS2Memory &memory, uint32_t channelBase)
    {
        IpuState &s = state();
        std::lock_guard<std::recursive_mutex> lock(s.mutex);
        const bool running = channelBase == kToIpuChannel ? s.toIpu.active : s.fromIpuActive;
        uint32_t chcr = memory.m_ioRegisters[channelBase + kChcr];
        chcr = running ? (chcr | kChcrStart) : (chcr & ~kChcrStart);
        return chcr;
    }

    void reset()
    {
        IpuState &s = state();
        std::lock_guard<std::recursive_mutex> lock(s.mutex);
        s.core.reset();
        s.toIpu.active = false;
        s.fromIpuActive = false;
        s.outLatchValid = false;
    }
}
