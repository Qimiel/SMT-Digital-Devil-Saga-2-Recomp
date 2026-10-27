// DDS2 (SLUS_211.52) IOP sound driver HLE.
//
// SDFSDMAN.IRX receives 0x80-byte command packets on SIF command 0 and, for
// streamed audio, calls back into the EE through the "SNRT" RPC server that the
// game registers at 0x342A90 (server function 0x342848). The real driver runs
// on the IOP; here it runs natively on the EE scheduler:
//
//   * IOP -> EE RPC: sceSifRpcLoop (0x369228) is replaced by a loop that blocks
//     the game's server thread until this module posts a request, then runs the
//     registered server function as a guest invocation and hands the reply back.
//   * ADPCM streaming (commands 0x12 play-from-EE, 0x10 stop, 0x11 volume),
//     following the driver's thread at module offset 0x4FF8: RPC 6 starts the
//     stream, two 16-byte RPC 7 reads return the "APCM" headers, then 0x200-byte
//     RPC 7 reads return PS2 ADPCM data (left 0x100 + right 0x100 when stereo)
//     until the byte count from the second header is exhausted; RPC 8 stops.
//     The EE side copies each chunk with SIF DMA to the IOP address in the
//     request; kIopBuffer stands in for the driver's IOP buffers.
//   * Output: ADPCM is decoded and resampled into a 48 kHz raylib audio stream.
//   * Sound banks (command 0x0A, "/sound/MIDIxxxx.SMG"): the driver reads them
//     through SDFIOPFL, i.e. RPCs to the EE file server "IOPf" (0x348408: 0x20
//     open, 0x22 read to an IOP address, 0x23 close), and records each loaded
//     bank in the driver work area the EE keeps at 0x47AA40, which it DMAs
//     back to the EE. The game polls that table (0x342168) before using a bank.
//   * Commands complete in order, as in the driver's command loop; SIF
//     register 30 reports the sequence number of the last completed one.
//
// Music synthesis and sound effects from the banks are not handled yet.
#include "dds2_sound.h"

#include "ps2_host_backend.h"
#include "ps2_runtime.h"
#include "ps2_runtime_macros.h"
#include "ps2_stubs.h"
#include "runtime/ee_scheduler.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dds2_sound
{
    namespace
    {
        constexpr uint32_t kSifRegisterRpc = 0x00368E10u;
        constexpr uint32_t kSifRpcLoop = 0x00369228u;
        constexpr uint32_t kVBlankHandler = 0x00328AE8u; // INTC VBlank handler installed by 0x328B20
        constexpr uint32_t kSoundServerSid = 0x54524E53u; // "SNRT"
        constexpr uint32_t kFileServerSid = 0x66504F49u;  // "IOPf"
        constexpr uint32_t kIopBuffer = 0x000F8000u;      // above the runtime's invocation stacks
        constexpr uint32_t kBankHeaderBuffer = 0x000F9000u;
        constexpr uint32_t kRpcWaitType = 0x44445352u;

        // Driver work area (EE copy) and its loaded-bank table: 13 slots of
        // {u32 bank id or -1, u8, u8} from +0x190 (see SDFSDMAN 0x2C0C).
        constexpr uint32_t kWorkArea = 0x0047AA40u;
        constexpr uint32_t kBankTable = kWorkArea + 0x190u;
        constexpr uint32_t kBankSlots = 13u;

        enum FileRpc : uint32_t
        {
            kFileOpen = 0x20,
            kFileRead = 0x22,
            kFileClose = 0x23,
        };

        enum StreamRpc : uint32_t
        {
            kStreamStart = 6,
            kStreamRead = 7,
            kStreamStop = 8,
        };

        // Sample-rate pitch table at SDFSDMAN.IRX offset 0x6570 (0x1000 = 48 kHz).
        constexpr uint32_t kPitchTable[] = {0x1000, 0xEB4, 0xAAA, 0x800, 0x75A, 0x555};

        bool logEnabled()
        {
            static const bool enabled = std::getenv("DDS2_LOG_SOUND") != nullptr;
            return enabled;
        }

#define SOUND_LOG(...)                         \
    do                                         \
    {                                          \
        if (logEnabled())                      \
            std::fprintf(stderr, __VA_ARGS__); \
    } while (0)

        uint32_t readGuest32(const uint8_t *rdram, uint32_t address)
        {
            uint32_t value = 0;
            std::memcpy(&value, rdram + (address & PS2_RAM_MASK), sizeof(value));
            return value;
        }

        // ------------------------------------------------------------------
        // Host output
        // ------------------------------------------------------------------

        class Mixer
        {
        public:
            static Mixer &instance()
            {
                static Mixer mixer;
                return mixer;
            }

            void open()
            {
                if (m_open || !IsAudioDeviceReady())
                    return;
                m_stream = LoadAudioStream(kOutputRate, 16, 2);
                SetAudioStreamCallback(m_stream, &Mixer::callback);
                PlayAudioStream(m_stream);
                m_open = true;
            }

            void reset(uint32_t sourceRate, float volume)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_frames.clear();
                m_position = 0.0;
                m_step = static_cast<double>(sourceRate) / kOutputRate;
                m_volume = volume;
            }

            void setVolume(float volume)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_volume = volume;
            }

            void clear()
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_frames.clear();
                m_position = 0.0;
            }

            void push(const int16_t *left, const int16_t *right, size_t count)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                for (size_t i = 0; i < count; ++i)
                    m_frames.push_back({left[i], right[i]});
            }

            size_t buffered()
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                return m_frames.size();
            }

        private:
            static constexpr unsigned kOutputRate = 48000;

            static void callback(void *buffer, unsigned int frames)
            {
                instance().mix(static_cast<int16_t *>(buffer), frames);
            }

            void mix(int16_t *out, unsigned frames)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                for (unsigned f = 0; f < frames; ++f)
                {
                    if (m_frames.size() < 2)
                    {
                        out[f * 2] = out[f * 2 + 1] = 0;
                        continue;
                    }
                    const double t = m_position;
                    const auto &a = m_frames[0];
                    const auto &b = m_frames[1];
                    for (int c = 0; c < 2; ++c)
                    {
                        const double v = (a[c] + (b[c] - a[c]) * t) * m_volume;
                        out[f * 2 + c] = static_cast<int16_t>(std::clamp(v, -32768.0, 32767.0));
                    }
                    m_position += m_step;
                    while (m_position >= 1.0 && m_frames.size() >= 2)
                    {
                        m_frames.pop_front();
                        m_position -= 1.0;
                    }
                }
            }

            std::mutex m_mutex;
            std::deque<std::array<int16_t, 2>> m_frames;
            double m_position = 0.0;
            double m_step = 44100.0 / kOutputRate;
            float m_volume = 1.0f;
            AudioStream m_stream{};
            bool m_open = false;
        };

        // PS2 SPU2 ADPCM: 16-byte blocks of 28 samples.
        void decodeAdpcm(const uint8_t *src, size_t blocks, int32_t history[2], int16_t *out)
        {
            static constexpr int kFilters[5][2] = {{0, 0}, {60, 0}, {115, -52}, {98, -55}, {122, -60}};
            for (size_t b = 0; b < blocks; ++b, src += 16)
            {
                const int shift = std::min(src[0] & 0x0F, 12);
                int filter = (src[0] >> 4) & 0x07;
                if (filter > 4)
                    filter = 0;
                for (int i = 0; i < 28; ++i)
                {
                    const int nibble = (src[2 + i / 2] >> ((i & 1) * 4)) & 0x0F;
                    int sample = static_cast<int16_t>(nibble << 12) >> shift;
                    sample += (history[0] * kFilters[filter][0] + history[1] * kFilters[filter][1] + 32) >> 6;
                    sample = std::clamp(sample, -32768, 32767);
                    history[1] = history[0];
                    history[0] = sample;
                    *out++ = static_cast<int16_t>(sample);
                }
            }
        }

        // ------------------------------------------------------------------
        // IOP -> EE RPC
        // ------------------------------------------------------------------

        struct Server
        {
            uint32_t func = 0;
            uint32_t buf = 0;
            uint32_t qd = 0;
        };

        using Done = std::function<void(PS2Runtime &, uint8_t *, int32_t result)>;

        struct Request
        {
            uint32_t sid = 0;
            uint32_t rpcNum = 0;
            std::vector<uint8_t> data; // copied to the server's receive buffer
            Done done;
        };

        std::unordered_map<uint32_t, Server> g_servers;              // by sid
        std::unordered_map<uint32_t, std::deque<Request>> g_queues; // by queue address

        bool postRpcData(PS2Runtime &runtime, uint32_t sid, uint32_t rpcNum, std::vector<uint8_t> data, Done done)
        {
            const auto it = g_servers.find(sid);
            if (it == g_servers.end())
            {
                static std::unordered_set<uint32_t> warned;
                if (warned.insert(sid).second)
                    std::fprintf(stderr, "[dds2-sound] RPC server %08x not registered yet\n", sid);
                return false;
            }
            Request request;
            request.sid = sid;
            request.rpcNum = rpcNum;
            request.data = std::move(data);
            request.done = std::move(done);
            g_queues[it->second.qd].push_back(std::move(request));
            runtime.eeScheduler().completeExternalWait(kRpcWaitType, it->second.qd, 0);
            return true;
        }

        // Requests made of four words; the drivers put their arguments in the
        // ones after the first.
        bool postRpc(PS2Runtime &runtime, uint32_t sid, uint32_t rpcNum, std::array<uint32_t, 4> words, Done done)
        {
            std::vector<uint8_t> data(sizeof(words));
            std::memcpy(data.data(), words.data(), sizeof(words));
            return postRpcData(runtime, sid, rpcNum, std::move(data), std::move(done));
        }

        // SNRT stream requests: {0, 0, IOP destination, size}.
        bool postRpc(PS2Runtime &runtime, uint32_t sid, uint32_t rpcNum, uint32_t iopAddress, uint32_t size, Done done)
        {
            return postRpc(runtime, sid, rpcNum, {0u, 0u, iopAddress, size}, std::move(done));
        }

        // 0x368E10 sceSifRegisterRpc(sd, sid, func, buf, cfunc, cbuf, qd). The EE
        // ABI passes arguments five to eight in t0-t3, so qd is in t2.
        void registerRpcHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            const uint32_t sid = getRegU32(ctx, 5);
            Server server;
            server.func = getRegU32(ctx, 6);
            server.buf = getRegU32(ctx, 7);
            server.qd = getRegU32(ctx, 10);
            g_servers[sid] = server;
            ctx->pc = getRegU32(ctx, 31);
            ps2_stubs::sceSifRegisterRpc(rdram, ctx, runtime);
        }

        // 0x369228 sceSifRpcLoop(qd): serves requests posted by postRpc().
        void rpcLoopHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            const uint32_t qd = getRegU32(ctx, 4);
            EeScheduler &scheduler = runtime->eeScheduler();
            scheduler.bindMainContextForSyscall(*ctx, rdram);
            ctx->pc = kSifRpcLoop; // the thread stays in the loop

            auto queue = g_queues.find(qd);
            if (queue == g_queues.end() || queue->second.empty())
                scheduler.waitExternal(EeWaitReason::External, kRpcWaitType, qd);

            Request request = std::move(queue->second.front());
            queue->second.pop_front();
            const Server &server = g_servers[request.sid];
            std::memcpy(rdram + (server.buf & PS2_RAM_MASK), request.data.data(), request.data.size());

            GuestInvocation invocation{};
            invocation.kind = GuestInvocationKind::RpcCallback;
            invocation.context = *ctx;
            invocation.context.pc = server.func;
            SET_GPR_U32(&invocation.context, 4, request.rpcNum);
            SET_GPR_U32(&invocation.context, 5, server.buf);
            SET_GPR_U32(&invocation.context, 6, static_cast<uint32_t>(request.data.size()));
            SET_GPR_U32(&invocation.context, 29, 0u);
            SET_GPR_U32(&invocation.context, 31, 0u);
            invocation.onComplete = [runtime, rdram, done = std::move(request.done)](const R5900Context &completed, R5900Context &)
            {
                const uint32_t reply = getRegU32(&completed, 2);
                const int32_t result = reply ? static_cast<int32_t>(readGuest32(rdram, reply)) : -1;
                if (done)
                    done(*runtime, rdram, result);
            };
            scheduler.invokeCurrent(std::move(invocation));
        }

        // ------------------------------------------------------------------
        // ADPCM stream (sdfAdpcm)
        // ------------------------------------------------------------------

        struct Stream
        {
            enum class State
            {
                Idle,
                Header,
                Streaming,
            };
            State state = State::Idle;
            uint32_t generation = 0;
            int headerIndex = 0;
            bool headerPending = false;
            bool stereo = false;
            uint32_t sampleRate = 44100;
            uint32_t toRequest = 0; // bytes not yet requested
            uint32_t toReceive = 0; // bytes not yet received
            int inFlight = 0;
            uint32_t slot = 0;
            float volume = 1.0f;
            int32_t history[2][2] = {};
        } g_stream;

        constexpr uint32_t kChunkBytes = 0x200;
        constexpr int kMaxInFlight = 4;

        float volumeFromNibble(uint32_t nibble)
        {
            uint32_t v = nibble << 3;
            if (v & 8u)
                v |= 7u;
            return static_cast<float>(v << 7) / 16383.0f;
        }

        void stopStream(PS2Runtime &runtime, bool flush)
        {
            if (g_stream.state == Stream::State::Idle)
                return;
            SOUND_LOG("[dds2-sound] ADPCM stream stop (%s)\n", flush ? "command" : "end of data");
            ++g_stream.generation;
            g_stream.state = Stream::State::Idle;
            g_stream.inFlight = 0;
            if (flush)
                Mixer::instance().clear();
            postRpc(runtime, kSoundServerSid, kStreamStop, 0, 0, {});
        }

        void requestHeader(PS2Runtime &runtime)
        {
            const uint32_t generation = g_stream.generation;
            g_stream.headerPending = true;
            postRpc(runtime, kSoundServerSid, kStreamRead, kIopBuffer, 0x10,
                    [generation](PS2Runtime &rt, uint8_t *rdram, int32_t result)
                    {
                        if (generation != g_stream.generation)
                            return;
                        g_stream.headerPending = false;
                        if (result < 0)
                            return; // no data yet: retried from the VBlank tick
                        const uint8_t *header = rdram + kIopBuffer;
                        if (g_stream.headerIndex == 0)
                        {
                            if (header[0] != 0x0B || readGuest32(rdram, kIopBuffer + 8) != 0x4D435041u) // "APCM"
                            {
                                std::fprintf(stderr, "[dds2-sound] bad ADPCM stream header\n");
                                stopStream(rt, true);
                                return;
                            }
                            g_stream.headerIndex = 1;
                            requestHeader(rt);
                            return;
                        }
                        const uint32_t totalBytes = readGuest32(rdram, kIopBuffer) << 1;
                        g_stream.stereo = header[0x0C] != 0;
                        const uint32_t rateIndex = std::min<uint32_t>(header[0x0D], std::size(kPitchTable) - 1);
                        g_stream.sampleRate = kPitchTable[rateIndex] * 48000u / 4096u;
                        g_stream.toRequest = g_stream.toReceive = totalBytes;
                        g_stream.state = Stream::State::Streaming;
                        Mixer::instance().reset(g_stream.sampleRate, g_stream.volume);
                        SOUND_LOG("[dds2-sound] ADPCM stream: %u bytes, %s, %u Hz\n", totalBytes,
                                  g_stream.stereo ? "stereo" : "mono", g_stream.sampleRate);
                    });
        }

        void startStream(PS2Runtime &runtime, uint32_t volumeNibble)
        {
            if (g_stream.state != Stream::State::Idle)
            {
                SOUND_LOG("[dds2-sound] ADPCM already playing\n");
                return;
            }
            const uint32_t generation = ++g_stream.generation;
            g_stream.state = Stream::State::Header;
            g_stream.headerIndex = 0;
            g_stream.headerPending = true;
            g_stream.inFlight = 0;
            g_stream.volume = volumeFromNibble(volumeNibble);
            std::memset(g_stream.history, 0, sizeof(g_stream.history));
            SOUND_LOG("[dds2-sound] ADPCM stream start (volume %x)\n", volumeNibble);
            postRpc(runtime, kSoundServerSid, kStreamStart, 0, 0,
                    [generation](PS2Runtime &rt, uint8_t *, int32_t)
                    {
                        if (generation == g_stream.generation)
                            requestHeader(rt);
                    });
        }

        void receiveChunk(PS2Runtime &runtime, uint8_t *rdram, uint32_t address)
        {
            const uint8_t *data = rdram + address;
            int16_t left[896];
            int16_t right[896];
            size_t frames = 0;
            if (g_stream.stereo)
            {
                decodeAdpcm(data, 16, g_stream.history[0], left);
                decodeAdpcm(data + 0x100, 16, g_stream.history[1], right);
                frames = 448;
            }
            else
            {
                decodeAdpcm(data, 32, g_stream.history[0], left);
                std::memcpy(right, left, sizeof(int16_t) * 896);
                frames = 896;
            }
            Mixer::instance().push(left, right, frames);
            g_stream.toReceive = g_stream.toReceive > kChunkBytes ? g_stream.toReceive - kChunkBytes : 0u;
            if (g_stream.toReceive == 0u)
                stopStream(runtime, false);
        }

        void requestChunk(PS2Runtime &runtime)
        {
            const uint32_t generation = g_stream.generation;
            const uint32_t address = kIopBuffer + 0x100u + (g_stream.slot++ % kMaxInFlight) * kChunkBytes;
            g_stream.toRequest -= std::min(g_stream.toRequest, kChunkBytes);
            ++g_stream.inFlight;
            postRpc(runtime, kSoundServerSid, kStreamRead, address, kChunkBytes,
                    [generation, address](PS2Runtime &rt, uint8_t *rdram, int32_t result)
                    {
                        if (generation != g_stream.generation)
                            return;
                        --g_stream.inFlight;
                        if (result < 0)
                        {
                            g_stream.toRequest += kChunkBytes; // underrun: ask again later
                            static uint32_t underruns = 0;
                            if ((++underruns & 0xFFu) == 1u)
                                SOUND_LOG("[dds2-sound] ADPCM source underrun (%u so far, %zu frames buffered)\n",
                                          underruns, Mixer::instance().buffered());
                            return;
                        }
                        receiveChunk(rt, rdram, address);
                    });
        }

        // Runs once per VBlank on the EE scheduler.
        void tick(PS2Runtime &runtime)
        {
            if (g_stream.state == Stream::State::Header && !g_stream.headerPending)
            {
                requestHeader(runtime);
                return;
            }
            if (g_stream.state != Stream::State::Streaming)
                return;
            static uint32_t ticks = 0;
            if ((++ticks % 300u) == 0u)
                SOUND_LOG("[dds2-sound] ADPCM stream: %u bytes to request, %u to receive, %d in flight, %zu frames buffered\n",
                          g_stream.toRequest, g_stream.toReceive, g_stream.inFlight, Mixer::instance().buffered());
            const size_t framesPerChunk = g_stream.stereo ? 448u : 896u;
            const size_t target = g_stream.sampleRate / 4u; // keep ~250 ms buffered
            while (g_stream.inFlight < kMaxInFlight && g_stream.toRequest > 0u &&
                   Mixer::instance().buffered() + static_cast<size_t>(g_stream.inFlight) * framesPerChunk < target)
            {
                requestChunk(runtime);
            }
        }

        PS2Runtime::RecompiledFunction g_vblankHandler = nullptr;

        void vblankHook(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
        {
            tick(*runtime);
            g_vblankHandler(rdram, ctx, runtime);
        }

        // ------------------------------------------------------------------
        // Sound banks (command 0x0A)
        // ------------------------------------------------------------------

        uint8_t *g_rdram = nullptr;
        uint32_t g_mainBankSlot = 0;   // SDFSDMAN DAT_0000648c: slot of the last main bank
        uint32_t g_effectRotation = 0; // DAT_0000648d: next of three slots for types 6-8

        uint32_t bankId(uint32_t slot)
        {
            return readGuest32(g_rdram, kBankTable + slot * 8u);
        }

        void clearBankSlot(uint32_t slot)
        {
            const uint32_t empty = 0xFFFFFFFFu;
            std::memcpy(g_rdram + kBankTable + slot * 8u, &empty, sizeof(empty));
        }

        // Slot choice of SDFSDMAN 0x2C0C by bank type. Banks are double
        // buffered between two sets (odd and even slots); main banks (type 1)
        // alternate between slots 1 and 2 and drop every other bank, and the
        // other types load into the set that is not current, dropping that
        // set's main bank.
        int32_t placeBank(uint32_t type)
        {
            static constexpr uint32_t kEffectSlots[2][3] = {{7, 9, 11}, {8, 10, 12}};
            const bool otherSet = g_mainBankSlot == 1u;
            int32_t slot = -1;
            int32_t dropped = -1;
            switch (type)
            {
            case 0:
                slot = 0;
                break;
            case 1:
                slot = otherSet ? 2 : 1;
                g_mainBankSlot = static_cast<uint32_t>(slot);
                for (uint32_t other = 3; other < kBankSlots; ++other)
                    clearBankSlot(other);
                break;
            case 4:
                slot = otherSet ? 4 : 3;
                dropped = otherSet ? 2 : 1;
                break;
            case 5:
                slot = otherSet ? 6 : 5;
                dropped = otherSet ? 2 : 1;
                break;
            case 6:
            case 7:
            case 8:
                slot = static_cast<int32_t>(kEffectSlots[otherSet ? 1 : 0][g_effectRotation]);
                dropped = otherSet ? 2 : 1;
                g_effectRotation = (g_effectRotation + 1u) % 3u;
                break;
            default:
                return -1;
            }
            if (dropped >= 0)
                clearBankSlot(static_cast<uint32_t>(dropped));
            clearBankSlot(static_cast<uint32_t>(slot));
            return slot;
        }

        // Records the bank whose 0xB0-byte SMG header was read: id in the high
        // half of +0x28, type in byte +0x2F, two attribute bytes at +0x2C.
        void registerBank(const std::string &path)
        {
            const uint32_t idWord = readGuest32(g_rdram, kBankHeaderBuffer + 0x28u);
            const uint32_t id = idWord >> 16;
            uint32_t type = g_rdram[kBankHeaderBuffer + 0x2Fu];
            if (idWord == 0x10000u)
                type = 4;
            for (uint32_t slot = 0; slot < kBankSlots; ++slot)
            {
                if (bankId(slot) == id)
                {
                    SOUND_LOG("[dds2-sound] bank %s (id %u) already loaded in slot %u\n", path.c_str(), id, slot);
                    return;
                }
            }
            const int32_t slot = placeBank(type);
            if (slot < 0)
            {
                SOUND_LOG("[dds2-sound] bank %s has unsupported type %u\n", path.c_str(), type);
                return;
            }
            const uint32_t entry = kBankTable + static_cast<uint32_t>(slot) * 8u;
            std::memcpy(g_rdram + entry, &id, sizeof(id));
            g_rdram[entry + 4u] = g_rdram[kBankHeaderBuffer + 0x2Cu];
            g_rdram[entry + 5u] = g_rdram[kBankHeaderBuffer + 0x2Du];
            SOUND_LOG("[dds2-sound] bank %s (id %u, type %u) loaded in slot %d\n", path.c_str(), id, type, slot);
        }

        // Reads the bank header through the EE file server, as SDFIOPFL does.
        void loadBank(PS2Runtime &runtime, const std::string &path, std::function<void()> finished)
        {
            std::vector<uint8_t> name(path.begin(), path.end());
            name.resize((name.size() + 4u) & ~3u, 0u);
            auto readHeader = [path, finished](PS2Runtime &rt, uint8_t *, int32_t handle)
            {
                if (handle == 0 || handle == -1)
                {
                    std::fprintf(stderr, "[dds2-sound] cannot open bank %s\n", path.c_str());
                    finished();
                    return;
                }
                const uint32_t file = static_cast<uint32_t>(handle);
                auto close = [path, finished, file](PS2Runtime &rt2, uint8_t *, int32_t bytes)
                {
                    if (bytes >= 0xB0)
                        registerBank(path);
                    else
                        std::fprintf(stderr, "[dds2-sound] short read of bank %s header\n", path.c_str());
                    if (!postRpc(rt2, kFileServerSid, kFileClose, {file, 0u, 0u, 0u},
                                 [finished](PS2Runtime &, uint8_t *, int32_t) { finished(); }))
                        finished();
                };
                if (!postRpc(rt, kFileServerSid, kFileRead, {file, kBankHeaderBuffer, 0xB0u, 0u}, close))
                    finished();
            };
            if (!postRpcData(runtime, kFileServerSid, kFileOpen, std::move(name), readHeader))
                finished();
        }

        // ------------------------------------------------------------------
        // Command loop
        // ------------------------------------------------------------------

        struct Command
        {
            uint32_t word = 0;
            uint32_t sequence = 0;
            std::string text; // payload as a string (file names)
        };

        std::deque<Command> g_commands;
        bool g_commandRunning = false;
        uint32_t g_completedSequence = 0;

        void runCommands(PS2Runtime &runtime);

        void completeCommand(PS2Runtime &runtime, uint32_t sequence)
        {
            g_completedSequence = sequence;
            g_commandRunning = false;
            runCommands(runtime);
        }

        // Returns false for commands that complete later via completeCommand().
        bool executeCommand(PS2Runtime &runtime, const Command &command)
        {
            const uint32_t code = (command.word >> 20) & 0xFFu;
            const uint32_t sub = (command.word >> 16) & 0x0Fu;
            switch (code)
            {
            case 0x0A: // load a sound bank
            {
                const uint32_t sequence = command.sequence;
                loadBank(runtime, command.text, [&runtime, sequence]()
                         { completeCommand(runtime, sequence); });
                return false;
            }
            case 0x12: // ADPCM stream from the EE (movies)
                startStream(runtime, sub);
                return true;
            case 0x10: // ADPCM stop
                stopStream(runtime, true);
                return true;
            case 0x11: // ADPCM volume
                g_stream.volume = volumeFromNibble(sub);
                Mixer::instance().setVolume(g_stream.volume);
                return true;
            default:
                SOUND_LOG("[dds2-sound] command %02x sub %x param %04x not implemented\n", code, sub, command.word & 0xFFFFu);
                return true;
            }
        }

        void runCommands(PS2Runtime &runtime)
        {
            while (!g_commandRunning && !g_commands.empty())
            {
                const Command command = std::move(g_commands.front());
                g_commands.pop_front();
                g_commandRunning = true;
                if (executeCommand(runtime, command))
                {
                    g_completedSequence = command.sequence;
                    g_commandRunning = false;
                }
            }
        }
    }

    void install(PS2Runtime &runtime)
    {
        g_rdram = runtime.memory().getRDRAM();
        runtime.replaceFunction(kSifRegisterRpc, &registerRpcHook);
        runtime.replaceFunction(kSifRpcLoop, &rpcLoopHook);
        g_vblankHandler = runtime.lookupFunction(kVBlankHandler);
        runtime.replaceFunction(kVBlankHandler, &vblankHook);
        Mixer::instance().open();
    }

    void onSoundCommand(PS2Runtime &runtime, uint8_t *rdram, uint32_t packetAddress)
    {
        Command command;
        command.word = readGuest32(rdram, packetAddress + 0x0Cu);
        command.sequence = readGuest32(rdram, packetAddress + 0x10u);
        for (uint32_t i = 0x14u; i < 0x80u; ++i)
        {
            const char c = static_cast<char>(rdram[(packetAddress + i) & PS2_RAM_MASK]);
            if (c == '\0')
                break;
            command.text.push_back(c);
        }
        g_commands.push_back(std::move(command));
        runCommands(runtime);
    }

    uint32_t completedSequence()
    {
        return g_completedSequence;
    }
}
