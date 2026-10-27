#pragma once

// Glue between the IPU decoder core and the EE memory map / DMA controller.
// Kept out of ps2_memory.h so guest translation units do not depend on it.

#include <cstdint>

class PS2Memory;

namespace ps2_ipu
{
    constexpr uint32_t kFromIpuChannel = 0x1000B000u; // DMA channel 3
    constexpr uint32_t kToIpuChannel = 0x1000B400u;   // DMA channel 4

    bool isIpuRegister(uint32_t physAddr);
    uint32_t readRegister(PS2Memory &memory, uint32_t physAddr);
    void writeRegister(PS2Memory &memory, uint32_t physAddr, uint32_t value);

    bool isIpuDmaChannel(uint32_t channelBase);
    // Called for every CHCR write on channel 3 or 4 (after the register store):
    // STR set starts the transfer, STR clear stops it.
    void writeDmaChcr(PS2Memory &memory, uint32_t channelBase, uint32_t value);
    // CHCR read for channel 3 or 4 (reflects whether the transfer is still running).
    uint32_t readDmaChcr(PS2Memory &memory, uint32_t channelBase);

    void reset();
}
