#pragma once

#include "runtime/gs/gs_backend.h"
#include "runtime/gs/gs_cpu_backend.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// Runs another raster backend on a worker thread, as the real GS runs
// alongside the EE. Draws, image uploads and VRAM-to-VRAM copies are queued in
// order; every call that returns data from VRAM (readback, snapshots,
// presentation) first waits for the work queued before it. The EE may run at
// most two fields ahead: the first draw after each VBlank waits for the work
// queued before the VBlank two fields back (a FINISH sync keeps one frame),
// so a scene too heavy for the host slows down evenly instead of building up
// seconds of queued work.
//
// With the CPU rasterizer, runs of consecutive draws are further split by
// framebuffer rows over several band threads. A run ends before a draw or
// transfer that would read memory an earlier draw of the run writes, or write
// memory the run reads, so render-to-texture stays ordered. Draws sampling a
// decoded texture don't read GS memory, so textures streamed through the
// same memory between draws don't end the run.
class GSThreadedBackend final : public GSRasterBackend
{
public:
    explicit GSThreadedBackend(std::unique_ptr<GSRasterBackend> inner);
    ~GSThreadedBackend() override;

    // The VBlank counter used to pace submissions (GSRegisters::vsyncTick).
    void setFieldCounter(const std::atomic<uint64_t> *counter);
    // Builds the display state at the start of each field (called on the
    // producer thread, which holds the frontend state). The worker snapshots
    // GS memory once the work queued before that field is done, and
    // presentation shows the newest snapshot without waiting for the GS.
    void setPresentationSource(std::function<GSPresentationRequest()> source);

    void Initialize(uint8_t *vram, uint32_t vramSize) override;
    void Reset() override;

    void Submit(const GSPrimitiveBatch &batch) override;

    void BeginTransfer(const GSTransferCommand &command) override;
    void UploadImage(const uint8_t *data, uint32_t sizeBytes) override;

    void Flush() override;
    void TextureFlush() override;
    void Sync(GSSyncReason reason) override;
    PresentationFrame Present(const GSPresentationRequest &request) override;

    bool ClearFramebuffer(const GSContext &context, uint32_t rgba) override;
    uint32_t ConsumeLocalToHostBytes(uint8_t *dst, uint32_t maxBytes) override;

    uint32_t ReadVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y) const override;
    void WriteVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y, uint32_t value) override;
    void SnapshotVram(std::vector<uint8_t> &out) const override;
    GSTransferSnapshot GetTransferSnapshot() const override;

private:
    enum class CommandKind : uint8_t
    {
        Submit,
        BeginTransfer,
        UploadImage,
        Flush,
        TextureFlush,
        Sync,
        Capture, // snapshot GS memory for presentation
    };

    struct Command
    {
        CommandKind kind = CommandKind::Flush;
        GSSyncReason reason = GSSyncReason::Finish;
        GSPrimitiveBatch batch{};
        GSTransferCommand transfer{};
        std::vector<uint8_t> data;
        GSPresentationRequest presentation{};
    };

    struct ByteRange
    {
        uint32_t begin = 0;
        uint32_t end = 0;
    };

    void push(Command &&command);
    // Called by the producer before a draw: one field of lag at most.
    void paceToField();
    // Worker: snapshots GS memory with the display state to present.
    void captureDisplay(const GSPresentationRequest &request);
    // Blocks until every command submitted before the call has executed.
    void drain() const;
    void waitForCompleted(uint64_t sequence) const;
    void workerLoop();
    void execute(Command &command);

    void addToRun(const GSPrimitiveBatch &batch);
    // Ends the run first if a transfer touching these ranges would race it.
    void orderTransfer(const ByteRange &destination, const ByteRange *source);
    void flushRun();
    void bandLoop(uint32_t bandIndex);
    void reportFrameStats();

    std::unique_ptr<GSRasterBackend> m_inner;
    GSCpuBackend *m_cpu = nullptr;

    // Worker-thread state: the current run of draws and the memory it touches.
    std::vector<const GSPrimitiveBatch *> m_run;
    std::vector<const GSCpuBackend::DecodedTexture *> m_runTextures;
    std::vector<ByteRange> m_runWrites;
    std::vector<ByteRange> m_runReads;
    uint64_t m_runPixels = 0;
    ByteRange m_uploadDestination{}; // of the transfer in progress

    // Worker-thread frame statistics (PS2X_GS_STATS).
    std::chrono::steady_clock::time_point m_frameStart{};
    uint64_t m_framePrimitives = 0;
    uint64_t m_framePixels = 0;

    uint32_t m_bandCount = 1;
    std::vector<std::thread> m_bandThreads;
    std::mutex m_bandMutex;
    std::condition_variable m_bandStart;
    std::condition_variable m_bandDone;
    uint64_t m_bandGeneration = 0;
    uint32_t m_bandsRemaining = 0;
    bool m_bandStop = false;
    mutable std::mutex m_mutex;
    mutable std::condition_variable m_workAvailable;
    mutable std::condition_variable m_workDone;
    std::vector<Command> m_pending;
    bool m_workerWaiting = false; // guarded by m_mutex
    uint64_t m_submitted = 0;
    uint64_t m_completed = 0;
    uint64_t m_lastFinishFence = 0;
    uint64_t m_fieldFences[2] = {};                         // guarded by m_mutex
    const std::atomic<uint64_t> *m_fieldCounter = nullptr; // producer thread only
    uint64_t m_lastField = 0;                               // producer thread only

    // The newest display snapshot (GS memory and display state, guarded by
    // m_captureMutex), the worker's spare buffer, and the presenting
    // thread's copy and last frame.
    std::function<GSPresentationRequest()> m_presentationSource;
    std::mutex m_captureMutex;
    std::vector<uint8_t> m_capturedVram;
    GSPresentationRequest m_capturedRequest{};
    uint64_t m_captureSerial = 0;
    std::vector<uint8_t> m_captureScratch;
    std::vector<uint8_t> m_presentVram;
    uint64_t m_presentedSerial = 0;
    PresentationFrame m_lastFrame;
    bool m_stop = false;
    std::thread m_worker;
};
