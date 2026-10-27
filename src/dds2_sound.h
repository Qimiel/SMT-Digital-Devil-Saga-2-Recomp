#pragma once

// High-level replacement for DDS2's IOP sound driver (SDFSDMAN.IRX, Atlus
// "sdf_sound_driver") and the IOP-to-EE RPC traffic it generates.

#include <cstdint>

class PS2Runtime;

namespace dds2_sound
{
    // Binds the RPC server hooks and the per-VBlank service tick, and opens the
    // host audio stream. Called once from the SLUS_211.52 game override.
    void install(PS2Runtime &runtime);

    // A sound command packet posted by the EE on SIF command 0 (sender 0x3414E8).
    // Commands run in order; bank loads finish asynchronously.
    void onSoundCommand(PS2Runtime &runtime, uint8_t *rdram, uint32_t packetAddress);

    // Sequence number of the last command the driver finished (SIF register 30).
    uint32_t completedSequence();
}
