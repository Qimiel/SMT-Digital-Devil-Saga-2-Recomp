// DDS2 (SLUS_211.52) game-specific runtime bindings. Addresses are for this exact
// US build only.
//
// The ELF is stripped, so the Sony SDK library routines linked into it are bound
// here to the runtime's HLE implementations. Addresses come from
// recovered/code/library_name_candidates.csv (analyzer signature matches) and were
// cross-checked against library link order and the Ghidra pseudocode, e.g.
// dds2_init_archive calls 0x34C6B8 (sceCdSearchFile) then 0x34D400 (sceCdStInit),
// and 0x329C20 loads MSIFRPC/SDFIOP through 0x36D860 (sceSifLoadModule).
#include "dds2_debug.h"
#include "dds2_sound.h"
#include "game_overrides.h"
#include "ps2_runtime.h"
#include "ps2_stubs.h"
#include "ps2_syscalls.h"
#include "runtime/ee_scheduler.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>

namespace
{
    struct Binding
    {
        uint32_t address;
        const char *handler;
    };

    constexpr Binding kBindings[] = {
        // libsif: RPC and SIF command layer (0x367C38-0x369264)
        {0x00367C38u, "sceSifInitCmd"},
        {0x00367EB8u, "sceSifExitCmd"},
        {0x00367F20u, "sceSifAddCmdHandler"},
        {0x00367F50u, "sceSifRemoveCmdHandler"},
        {0x00368258u, "sceSifWriteBackDCache"},
        {0x00368308u, "sceSifInitRpc"},
        {0x00368720u, "sceSifGetOtherData"},
        {0x00368978u, "sceSifBindRpc"},
        {0x00368B48u, "sceSifCallRpc"},
        {0x00368D38u, "sceSifCheckStatRpc"},
        {0x00368D78u, "sceSifSetRpcQueue"},
        {0x00368EE0u, "sceSifRemoveRpc"},
        {0x00368F78u, "sceSifRemoveRpcQueue"},
        {0x00369008u, "sceSifGetNextRequest"},
        {0x00369060u, "sceSifExecRequest"},
        // sceSifRegisterRpc (0x368E10) and sceSifRpcLoop (0x369228) are hooked by
        // dds2_sound so the IOP HLE can call EE RPC servers.

        // libsif: IOP heap, loadfile and IOP reset (0x36C910-0x36DE6C)
        {0x0036C910u, "sceSifInitIopHeap"},
        {0x0036C998u, "sceSifAllocIopHeap"},
        {0x0036CA08u, "sceSifAllocSysMemory"},
        {0x0036CA88u, "sceSifFreeSysMemory"},
        {0x0036CB00u, "sceSifFreeIopHeap"},
        {0x0036CB20u, "sceSifLoadIopHeap"},
        {0x0036CFF0u, "sceSifLoadFileReset"},
        {0x0036D028u, "sceSifLoadModuleBuffer"}, // internal _sceSifLoadModuleBuffer
        {0x0036D5F8u, "sceSifLoadModuleBuffer"}, // thin wrappers around 0x36D028
        {0x0036D618u, "sceSifLoadModuleBuffer"},
        {0x0036D230u, "SifStopModule"},
        {0x0036D638u, "sceSifLoadModule"}, // internal _sceSifLoadModule
        {0x0036D860u, "sceSifLoadModule"}, // wrappers around 0x36D638
        {0x0036D880u, "sceSifLoadModule"},
        {0x0036D8A0u, "sceSifLoadElfPart"},
        {0x0036D9A8u, "sceSifLoadElfPart"},
        {0x0036D9C8u, "sceSifLoadElf"},
        {0x0036D9F0u, "sceSifGetIopAddr"},
        {0x0036DAE0u, "sceSifSetIopAddr"},
        {0x0036DBC0u, "sceSifResetIop"},
        {0x0036DD00u, "sceSifIsAliveIop"},
        {0x0036DD28u, "sceSifSyncIop"},
        {0x0036DD60u, "sceSifRebootIop"},

        // libmrpc (sceSifM*, talks to MSIFRPC.IRX). The client structure keeps
        // the sceSifClientData layout: the game polls +0x24 (server) after
        // binding, see 0x329C20 and 0x33D990. Route to the ordinary RPC HLE.
        {0x0034D8E0u, "sceSifInitRpc"},  // sceSifMInitRpc
        {0x0034DB88u, "SifBindRpc"},     // sceSifMBindRpc(client, sid, mode)
        {0x0034DBB0u, "SifBindRpc"},
        {0x0034DD48u, "ret0"},           // sceSifMUnBindRpc
        {0x0034DE68u, "SifCallRpc"},     // sceSifMCallRpc, same register ABI

        // libscf/fileio over SIF (0x3698D8-0x36AA2C)
        {0x003698D8u, "sceFsInit"},
        {0x00369B38u, "sceFsReset"},
        // sceOpen (0x369B70) is dds2SceOpen below.
        {0x00369DF8u, "sceClose"},
        {0x00369F78u, "sceLseek"},
        // sceRead (0x36A1B0) is dds2SceRead below.
        {0x0036A420u, "sceWrite"},
        {0x0036A6E0u, "sceIoctl"},

        // libcdvd (0x34BDA0-0x34D8DC)
        {0x0034BDA0u, "sceCdDelayThread"},
        {0x0034BE08u, "sceCdCallback"},
        {0x0034BFC8u, "sceCdInitEeCB"},
        {0x0034C3A0u, "sceCdSearchFile"}, // sceCdLayerSearchFile(fp, name, layer)
        {0x0034C6B8u, "sceCdSearchFile"},
        {0x0034C848u, "sceCdNcmdDiskReady"},
        {0x0034C8E0u, "sceCdSync"},
        {0x0034C980u, "sceCdSyncS"},
        {0x0034CB60u, "sceCdInit"},
        {0x0034D038u, "sceCdMmode"},
        {0x0034D100u, "sceCdGetDiskType"},
        {0x0034D198u, "sceCdStatus"},
        {0x0034D250u, "ret1"},            // sceCdPowerOff
        {0x0034D308u, "sceCdReadClock"},
        {0x0034D400u, "sceCdStInit"},
        {0x0034D430u, "sceCdStStart"},
        {0x0034D468u, "sceCdStSeekF"},
        {0x0034D498u, "sceCdStSeek"},
        {0x0034D4C8u, "sceCdStStop"},
        {0x0034D500u, "sceCdStRead"},
        {0x0034D688u, "sceCdStPause"},
        {0x0034D6D8u, "sceCdStResume"},
        {0x0034D730u, "sceCdStStat"},
        {0x0034D778u, "sceCdStream"},

        // libpad (0x34AAF8-0x34BD74)
        {0x0034AAF8u, "scePadInit"},
        {0x0034AC38u, "scePadInit2"},
        {0x0034ACD8u, "scePadEnd"},
        {0x0034AD58u, "scePadPortOpen"},
        {0x0034AF40u, "scePadPortClose"},
        {0x0034AFF8u, "scePadGetDmaStr"},
        {0x0034B058u, "scePadGetFrameCount"},
        {0x0034B0A8u, "scePadRead"},
        {0x0034B128u, "scePadGetState"},
        {0x0034B1A0u, "scePadStateIntToStr"},
        {0x0034B1D8u, "scePadSetReqState"},
        {0x0034B240u, "scePadGetReqState"},
        {0x0034B2C8u, "scePadInfoAct"},
        {0x0034B3E8u, "scePadInfoComb"},
        {0x0034B508u, "scePadInfoMode"},
        {0x0034B640u, "scePadSetMainMode"},
        {0x0034B6F8u, "scePadSetActDirect"},
        {0x0034B7B8u, "scePadSetActAlign"},
        {0x0034B890u, "scePadGetButtonMask"},
        {0x0034B948u, "scePadSetButtonInfo"},
        {0x0034B9F8u, "scePadInfoPressMode"},
        {0x0034BA58u, "scePadEnterPressMode"},
        {0x0034BAB0u, "scePadExitPressMode"},
        {0x0034BB08u, "scePadSetVrefParam"},
        {0x0034BBD8u, "scePadGetPortMax"},
        {0x0034BC40u, "scePadGetSlotMax"},
        {0x0034BCA8u, "scePadGetModVersion"},
        {0x0034BD10u, "scePadSetWarningLevel"},

        // libmc (0x34EC40-0x3504A4)
        {0x0034EC40u, "sceMcInit"},
        {0x0034EDF8u, "sceMcEnd"},
        {0x0034EE68u, "sceMcChangeThreadPriority"},
        {0x0034EF20u, "sceMcGetSlotMax"},
        {0x0034EFE0u, "sceMcOpen"},
        {0x0034F118u, "sceMcMkdir"},
        {0x0034F150u, "sceMcClose"},
        {0x0034F208u, "sceMcSeek"},
        {0x0034F370u, "sceMcRead"},
        {0x0034F490u, "sceMcWrite"},
        {0x0034F680u, "sceMcSync"},
        {0x0034F7B8u, "sceMcGetInfo"},
        {0x0034F9B0u, "sceMcGetDir"},
        {0x0034FB90u, "sceMcChdir"},
        {0x0034FCE0u, "sceMcFormat"},
        {0x0034FDA8u, "sceMcDelete"},
        {0x0034FED0u, "sceMcFlush"},
        {0x0034FF88u, "sceMcSetFileInfo"},
        {0x00350160u, "sceMcRename"},
        {0x003502C8u, "sceMcUnformat"},
        {0x00350390u, "sceMcGetEntSpace"},

        // libsdr remote (0x34E6E0-0x34EA1C)
        {0x0034E6E0u, "sceSdRemoteInit"},
        {0x0034E788u, "sceSdTransToIOP"},
        {0x0034E820u, "sceSdRemote"},

        // libipu (0x34E1A0-0x34E6DC) is deliberately not bound: the runtime
        // emulates the IPU hardware, and the generic sceIpuInit stub hardcodes
        // another game's addresses.

        // Debug output, routed to the host console.
        {0x0035B6E0u, "printf"},
        {0x00367B60u, "scePrintf"},
    };

    void returnToCaller(R5900Context *ctx)
    {
        ctx->pc = getRegU32(ctx, 31);
    }

    // SDFSDMAN.IRX (Atlus sound driver) protocol, from its IOP command loop at
    // module offset 0x4300: the EE posts 0x80-byte packets on SIF command 0
    // (sender 0x3414E8). Packet +0x0C holds the command word (bits 20..27
    // command, 16..19 sub-argument, 0..15 parameter), +0x10 a sequence number
    // and +0x14 the payload. After each command the driver writes the sequence
    // number to SIF software register 30; at startup it writes 1 to register 31.
    constexpr uint32_t kSregSoundCommandDone = 0x1Eu;
    constexpr uint32_t kSregSoundDriverReady = 0x1Fu;

    // 0x367BF0 sceSifGetSreg(reg); the EE sound client (0x3415A8) spins on these.
    void dds2SifGetSreg(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t reg = getRegU32(ctx, 4);
        if (reg == kSregSoundDriverReady || reg == kSregSoundCommandDone)
        {
            setReturnU32(ctx, reg == kSregSoundDriverReady ? 1u : dds2_sound::completedSequence());
            returnToCaller(ctx);
            return;
        }
        ctx->pc = getRegU32(ctx, 31);
        ps2_stubs::sceSifGetSreg(rdram, ctx, runtime);
    }

    // 0x3680B0 sceSifSendCmd(cid, packet, psize, src, dst, size).
    void dds2SifSendCmd(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        static uint32_t logged = 0;
        const uint32_t cid = getRegU32(ctx, 4);
        const uint32_t packet = getRegU32(ctx, 5);
        const uint32_t psize = getRegU32(ctx, 6);
        if (cid == 0u)
        {
            if (logged < 256u && std::getenv("DDS2_LOG_SOUND"))
            {
                ++logged;
                std::fprintf(stderr, "[snd-cmd] packet=%08x size=%u:", packet, psize);
                for (uint32_t i = 0; i < psize && i < 0x40u; i += 4)
                    std::fprintf(stderr, " %08x", runtime->Load32(rdram, ctx, packet + i));
                std::fprintf(stderr, "\n");
            }
            dds2_sound::onSoundCommand(*runtime, rdram, packet);
        }
        ctx->pc = getRegU32(ctx, 31);
        ps2_stubs::sceSifSendCmd(rdram, ctx, runtime);
    }

    // Disc file operations: on the console the calling thread sleeps while
    // the IOP seeks and reads, so lower-priority threads run before the
    // result arrives, and the game depends on it. The resource loader
    // (0x2C8AC0) queues a request on the higher-priority I/O thread
    // (0x33EF58) and only then stores the request in its loader object; if
    // the whole open/read/close finished before that store, the store put
    // back a request the loader's callback had already released, and the
    // second release corrupted the small-object heap. The operation itself
    // is done at once; the thread then waits for the next VBlank to get the
    // result, which is still far quicker than a DVD seek.
    [[noreturn]] void completeAfterVBlank(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const int result = static_cast<int>(getRegU32(ctx, 2));
        EeScheduler &scheduler = runtime->eeScheduler();
        scheduler.bindMainContextForSyscall(*ctx, rdram);
        scheduler.waitVSync(scheduler.currentVSyncTick(), result);
    }

    // 0x369B70 sceOpen(name, flags, mode).
    void dds2SceOpen(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ctx->pc = getRegU32(ctx, 31);
        ps2_syscalls::fioOpen(rdram, ctx, runtime);
        completeAfterVBlank(rdram, ctx, runtime);
    }

    // 0x36A1B0 sceRead(fd, buf, size).
    void dds2SceRead(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ctx->pc = getRegU32(ctx, 31);
        ps2_syscalls::fioRead(rdram, ctx, runtime);
        completeAfterVBlank(rdram, ctx, runtime);
    }

    void applyDds2Overrides(PS2Runtime &runtime)
    {
        runtime.replaceFunction(0x00369B70u, &dds2SceOpen);
        runtime.replaceFunction(0x0036A1B0u, &dds2SceRead);
        runtime.replaceFunction(0x003680B0u, &dds2SifSendCmd);
        runtime.replaceFunction(0x003680F0u, &dds2SifSendCmd); // isceSifSendCmd
        runtime.replaceFunction(0x00367BF0u, &dds2SifGetSreg);

        size_t bound = 0;
        for (const Binding &binding : kBindings)
        {
            if (ps2_game_overrides::bindAddressHandler(runtime, binding.address, binding.handler))
            {
                ++bound;
            }
            else
            {
                std::cerr << "[dds2] failed to bind 0x" << std::hex << binding.address << std::dec
                          << " -> " << binding.handler << std::endl;
            }
        }
        std::cout << "[dds2] bound " << bound << " SDK routines to runtime HLE" << std::endl;
        dds2_sound::install(runtime);
        dds2_debug::installHeapChecks(runtime);
        dds2_debug::installFunctionTrace(runtime);
    }
}

PS2_REGISTER_GAME_OVERRIDE("Digital Devil Saga 2 (US)", "SLUS_211.52", 0u, 0u, &applyDds2Overrides)
