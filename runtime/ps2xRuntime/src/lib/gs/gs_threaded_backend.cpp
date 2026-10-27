#include "runtime/gs/gs_threaded_backend.h"

#include "runtime/gs/gs_cpu_backend.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <utility>

// Host time (ns) spent per subsystem on the EE thread, reported with
// PS2X_TRACE_GS_PERF: 0 VIF1 data, 1 VU1 programs, 2 GS packets, 3 VU0 programs.
std::atomic<uint64_t> g_ps2xPerfNs[4];

namespace
{
    // Safety bound on queued commands; VBlank pacing normally keeps the queue
    // to about one field of work.
    constexpr uint64_t kMaxQueuedCommands = 1u << 16;
    // Queued draws that wake a sleeping worker.
    constexpr size_t kWakeBatch = 64u;
    // Runs smaller than this (estimated pixels) are not worth a band barrier.
    constexpr uint64_t kMinBandedPixels = 16384u;
    constexpr uint32_t kMaxRunLength = 8192u;
    constexpr uint32_t kGsMemorySize = 4u * 1024u * 1024u;
    constexpr uint32_t kPageBytes = 8192u;

    uint32_t bitsPerPixel(uint8_t psm)
    {
        switch (psm)
        {
        case 0x02: // CT16
        case 0x0A: // CT16S
        case 0x32: // Z16
        case 0x3A: // Z16S
            return 16u;
        case 0x13: // T8
            return 8u;
        case 0x14: // T4
            return 4u;
        default: // CT32/CT24/Z32/Z24 and the T8H/T4HL/T4HH formats stored in them
            return 32u;
        }
    }

    bool isPaletted(uint8_t psm)
    {
        return psm == 0x13 || psm == 0x14 || psm == 0x1B || psm == 0x24 || psm == 0x2C;
    }

    // Conservative byte range of a width x height image at baseBytes. The
    // swizzled layout uses whole pages (32 rows for 32-bit formats, 64 for
    // 16- and 8-bit, 128 for 4-bit), so height is rounded up to a page row,
    // plus one page of slack.
    std::pair<uint32_t, uint32_t> imageRange(uint32_t baseBytes, uint32_t width, uint32_t height, uint32_t bpp)
    {
        const uint32_t pageRows = bpp >= 32u ? 32u : (bpp >= 8u ? 64u : 128u);
        height = (height + pageRows - 1u) / pageRows * pageRows;
        const uint64_t bytes = static_cast<uint64_t>(width) * height * bpp / 8u;
        const uint64_t pages = (bytes + kPageBytes - 1u) / kPageBytes + 1u;
        const uint64_t begin = std::min<uint64_t>(baseBytes, kGsMemorySize);
        const uint64_t end = std::min<uint64_t>(begin + pages * kPageBytes, kGsMemorySize);
        return {static_cast<uint32_t>(begin), static_cast<uint32_t>(end)};
    }

    uint32_t defaultBandCount()
    {
        if (const char *env = std::getenv("PS2X_GS_THREADS"))
        {
            const int requested = std::atoi(env);
            uint32_t count = 1u;
            while (count * 2u <= static_cast<uint32_t>(std::max(requested, 1)) && count < 8u)
                count *= 2u;
            return count;
        }
        // Leave cores for the EE, presentation and audio threads.
        const unsigned hardware = std::thread::hardware_concurrency();
        if (hardware >= 12u)
            return 8u;
        return hardware >= 8u ? 4u : (hardware >= 4u ? 2u : 1u);
    }
}

GSThreadedBackend::GSThreadedBackend(std::unique_ptr<GSRasterBackend> inner)
    : m_inner(std::move(inner))
{
    m_cpu = dynamic_cast<GSCpuBackend *>(m_inner.get());
    if (m_cpu)
    {
        m_bandCount = defaultBandCount();
        for (uint32_t band = 1; band < m_bandCount; ++band)
            m_bandThreads.emplace_back([this, band]()
                                       { bandLoop(band); });
    }
    m_worker = std::thread([this]()
                           { workerLoop(); });
}

GSThreadedBackend::~GSThreadedBackend()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop = true;
    }
    m_workAvailable.notify_all();
    if (m_worker.joinable())
        m_worker.join();
    {
        std::lock_guard<std::mutex> lock(m_bandMutex);
        m_bandStop = true;
    }
    m_bandStart.notify_all();
    for (std::thread &thread : m_bandThreads)
        thread.join();
}

void GSThreadedBackend::bandLoop(uint32_t bandIndex)
{
    uint64_t seen = 0;
    for (;;)
    {
        {
            std::unique_lock<std::mutex> lock(m_bandMutex);
            m_bandStart.wait(lock, [this, seen]()
                             { return m_bandStop || m_bandGeneration != seen; });
            if (m_bandStop)
                return;
            seen = m_bandGeneration;
        }
        for (size_t i = 0; i < m_run.size(); ++i)
            m_cpu->DrawBanded(*m_run[i], bandIndex, m_bandCount, m_runTextures[i]);
        {
            std::lock_guard<std::mutex> lock(m_bandMutex);
            if (--m_bandsRemaining == 0u)
                m_bandDone.notify_one();
        }
    }
}

void GSThreadedBackend::addToRun(const GSPrimitiveBatch &batch)
{
    const GSDrawState &state = batch.state;
    const GSContext &ctx = state.context;

    const uint32_t frameWidth = std::max<uint32_t>(ctx.frame.fbw, 1u) * 64u;
    const uint32_t frameRows = static_cast<uint32_t>(ctx.scissor.y1) + 1u;
    ByteRange writes[2];
    uint32_t writeCount = 0;
    {
        const auto range = imageRange(ctx.frame.fbp * kPageBytes, frameWidth, frameRows, bitsPerPixel(ctx.frame.psm));
        writes[writeCount++] = {range.first, range.second};
    }
    if (!ctx.zbuf.zmask)
    {
        const auto range = imageRange(ctx.zbuf.zbp * kPageBytes, frameWidth, frameRows, bitsPerPixel(ctx.zbuf.psm));
        writes[writeCount++] = {range.first, range.second};
    }

    ByteRange reads[2];
    uint32_t readCount = 0;
    if (state.prim.tme)
    {
        const GSTex0Reg &tex = ctx.tex0;
        const uint32_t width = std::max<uint32_t>(std::max<uint32_t>(tex.tbw, 1u) * 64u, 1u << tex.tw);
        const auto range = imageRange(tex.tbp0 * 256u, width, 1u << tex.th, bitsPerPixel(tex.psm));
        reads[readCount++] = {range.first, range.second};
        if (isPaletted(tex.psm))
        {
            const uint32_t clut = std::min<uint32_t>(tex.cbp * 256u, kGsMemorySize);
            reads[readCount++] = {clut, std::min<uint32_t>(clut + 1024u, kGsMemorySize)};
        }
    }

    auto overlapsAny = [](const ByteRange &range, const std::vector<ByteRange> &set)
    {
        for (const ByteRange &other : set)
            if (range.begin < other.end && other.begin < range.end)
                return true;
        return false;
    };
    bool hazard = false;
    for (uint32_t i = 0; i < readCount && !hazard; ++i)
        hazard = overlapsAny(reads[i], m_runWrites);
    for (uint32_t i = 0; i < writeCount && !hazard; ++i)
        hazard = overlapsAny(writes[i], m_runReads);
    if (hazard || m_run.size() >= kMaxRunLength)
        flushRun();

    // Raster threads only read the texture cache: decode here, between runs.
    // A draw that samples its decoded copy no longer reads GS memory.
    const GSCpuBackend::DecodedTexture *texture = state.prim.tme ? m_cpu->PrepareTexture(state) : nullptr;
    if (texture)
        readCount = 0;

    auto addRange = [](const ByteRange &range, std::vector<ByteRange> &set)
    {
        for (const ByteRange &other : set)
            if (other.begin == range.begin && other.end == range.end)
                return;
        set.push_back(range);
    };
    for (uint32_t i = 0; i < writeCount; ++i)
        addRange(writes[i], m_runWrites);
    for (uint32_t i = 0; i < readCount; ++i)
        addRange(reads[i], m_runReads);

    // Pixel estimate from the vertex bounds, clipped to the scissor.
    const float ofx = static_cast<float>(ctx.xyoffset.ofx >> 4);
    const float ofy = static_cast<float>(ctx.xyoffset.ofy >> 4);
    float minX = batch.vertices[0].x - ofx, maxX = minX;
    float minY = batch.vertices[0].y - ofy, maxY = minY;
    for (uint32_t i = 1; i < batch.vertexCount && i < batch.vertices.size(); ++i)
    {
        minX = std::min(minX, batch.vertices[i].x - ofx);
        maxX = std::max(maxX, batch.vertices[i].x - ofx);
        minY = std::min(minY, batch.vertices[i].y - ofy);
        maxY = std::max(maxY, batch.vertices[i].y - ofy);
    }
    minX = std::max(minX, static_cast<float>(ctx.scissor.x0));
    maxX = std::min(maxX, static_cast<float>(ctx.scissor.x1) + 1.0f);
    minY = std::max(minY, static_cast<float>(ctx.scissor.y0));
    maxY = std::min(maxY, static_cast<float>(ctx.scissor.y1) + 1.0f);
    if (maxX > minX && maxY > minY)
        m_runPixels += static_cast<uint64_t>((maxX - minX) * (maxY - minY));

    m_run.push_back(&batch);
    m_runTextures.push_back(texture);
}

void GSThreadedBackend::orderTransfer(const ByteRange &destination, const ByteRange *source)
{
    auto overlapsAny = [](const ByteRange &range, const std::vector<ByteRange> &set)
    {
        for (const ByteRange &other : set)
            if (range.begin < other.end && other.begin < range.end)
                return true;
        return false;
    };
    if (overlapsAny(destination, m_runWrites) || overlapsAny(destination, m_runReads) ||
        (source && overlapsAny(*source, m_runWrites)))
    {
        flushRun();
    }
}

void GSThreadedBackend::flushRun()
{
    if (m_run.empty())
        return;
    m_framePrimitives += m_run.size();
    m_framePixels += m_runPixels;
    if (m_bandCount <= 1u || m_runPixels < kMinBandedPixels)
    {
        for (size_t i = 0; i < m_run.size(); ++i)
            m_cpu->DrawBanded(*m_run[i], 0u, 1u, m_runTextures[i]);
    }
    else
    {
        {
            std::lock_guard<std::mutex> lock(m_bandMutex);
            m_bandsRemaining = m_bandCount - 1u;
            ++m_bandGeneration;
        }
        m_bandStart.notify_all();
        for (size_t i = 0; i < m_run.size(); ++i)
            m_cpu->DrawBanded(*m_run[i], 0u, m_bandCount, m_runTextures[i]);
        std::unique_lock<std::mutex> lock(m_bandMutex);
        m_bandDone.wait(lock, [this]()
                        { return m_bandsRemaining == 0u; });
    }
    // Textures and CLUTs decoded from the pages the run drew to are stale.
    for (const ByteRange &range : m_runWrites)
        m_cpu->NoteVramWrites(range.begin, range.end);
    m_cpu->ReleaseRetiredTextures();
    m_run.clear();
    m_runTextures.clear();
    m_runWrites.clear();
    m_runReads.clear();
    m_runPixels = 0;
}

void GSThreadedBackend::push(Command &&command)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_submitted - m_completed >= kMaxQueuedCommands)
    {
        static const bool stats = std::getenv("PS2X_GS_STATS") != nullptr;
        const auto waitStart = std::chrono::steady_clock::now();
        const uint64_t target = m_submitted - kMaxQueuedCommands / 2u;
        m_workDone.wait(lock, [this, target]()
                        { return m_completed >= target; });
        if (stats)
            std::fprintf(stderr, "[gs-stats] queue full: waited %.0f ms\n",
                         std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - waitStart).count());
    }
    // Signalling costs a system call (and the worker a wake-up): wake a
    // sleeping worker once a batch has built up, or for anything beyond
    // draws and image data.
    const bool urgent = command.kind != CommandKind::Submit && command.kind != CommandKind::UploadImage;
    m_pending.push_back(std::move(command));
    ++m_submitted;
    const bool wake = m_workerWaiting && (urgent || m_pending.size() >= kWakeBatch);
    lock.unlock();
    if (wake)
        m_workAvailable.notify_one();
}

void GSThreadedBackend::waitForCompleted(uint64_t sequence) const
{
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_completed >= sequence)
        return;
    if (m_workerWaiting)
        m_workAvailable.notify_one();
    m_workDone.wait(lock, [this, sequence]()
                    { return m_completed >= sequence; });
}

void GSThreadedBackend::drain() const
{
    uint64_t target = 0;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        target = m_submitted;
    }
    waitForCompleted(target);
}

void GSThreadedBackend::workerLoop()
{
    std::vector<Command> batch;
    for (;;)
    {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            // Producers wake the worker in batches; never leave queued work
            // waiting longer than a millisecond.
            m_workerWaiting = true;
            while (!m_stop && m_pending.empty())
                m_workAvailable.wait_for(lock, std::chrono::milliseconds(1));
            m_workerWaiting = false;
            if (m_pending.empty() && m_stop)
                return;
            batch.swap(m_pending);
        }
        for (Command &command : batch)
            execute(command);
        // The run points into this batch's commands.
        flushRun();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_completed += batch.size();
        }
        m_workDone.notify_all();
        batch.clear();
    }
}

void GSThreadedBackend::execute(Command &command)
{
    if (command.kind == CommandKind::Submit && m_cpu)
    {
        addToRun(command.batch);
        return;
    }
    if (m_cpu)
    {
        // Games stream textures through GS memory between draws; a transfer
        // only has to wait for the queued draws it would race with.
        auto transferRange = [](uint32_t bp, uint32_t bw, uint32_t rows, uint8_t psm)
        {
            const auto range = imageRange(bp * 256u, std::max<uint32_t>(bw, 1u) * 64u, rows, bitsPerPixel(psm));
            return ByteRange{range.first, range.second};
        };
        const GSTransferCommand &transfer = command.transfer;
        switch (command.kind)
        {
        case CommandKind::BeginTransfer:
            if (transfer.direction == 0u || transfer.direction == 2u)
            {
                m_uploadDestination = transferRange(transfer.bitbltbuf.dbp, transfer.bitbltbuf.dbw,
                                                    static_cast<uint32_t>(transfer.trxpos.dsay) + transfer.trxreg.rrh,
                                                    transfer.bitbltbuf.dpsm);
                const ByteRange source = transferRange(transfer.bitbltbuf.sbp, transfer.bitbltbuf.sbw,
                                                       static_cast<uint32_t>(transfer.trxpos.ssay) + transfer.trxreg.rrh,
                                                       transfer.bitbltbuf.spsm);
                orderTransfer(m_uploadDestination, transfer.direction == 2u ? &source : nullptr);
            }
            else if (transfer.direction == 1u)
            {
                flushRun();
            }
            break;
        case CommandKind::UploadImage:
            orderTransfer(m_uploadDestination, nullptr);
            break;
        case CommandKind::Flush:
        case CommandKind::TextureFlush:
            break;
        default:
            flushRun();
            break;
        }
    }
    switch (command.kind)
    {
    case CommandKind::Submit:
        m_inner->Submit(command.batch);
        break;
    case CommandKind::BeginTransfer:
        m_inner->BeginTransfer(command.transfer);
        break;
    case CommandKind::UploadImage:
        m_inner->UploadImage(command.data.data(), static_cast<uint32_t>(command.data.size()));
        break;
    case CommandKind::Flush:
        m_inner->Flush();
        break;
    case CommandKind::TextureFlush:
        m_inner->TextureFlush();
        break;
    case CommandKind::Sync:
        m_inner->Sync(command.reason);
        if (command.reason == GSSyncReason::Finish)
            reportFrameStats();
        break;
    case CommandKind::Capture:
        captureDisplay(command.presentation);
        break;
    }
}

// PS2X_GS_STATS: logs frames (FINISH to FINISH) that took the worker over
// 50 ms, with their primitive count and estimated pixel area.
void GSThreadedBackend::reportFrameStats()
{
    static const bool enabled = std::getenv("PS2X_GS_STATS") != nullptr;
    const auto now = std::chrono::steady_clock::now();
    if (enabled && m_frameStart != std::chrono::steady_clock::time_point{})
    {
        const double ms = std::chrono::duration<double, std::milli>(now - m_frameStart).count();
        if (ms > 50.0)
            std::fprintf(stderr, "[gs-stats] frame took %.0f ms: %llu primitives, ~%llu pixels\n", ms,
                         static_cast<unsigned long long>(m_framePrimitives),
                         static_cast<unsigned long long>(m_framePixels));
        static uint64_t finishes = 0;
        if ((++finishes % 600u) == 0u)
            std::fprintf(stderr, "[gs-stats] %llu FINISH frames\n", static_cast<unsigned long long>(finishes));
    }
    m_frameStart = now;
    m_framePrimitives = 0;
    m_framePixels = 0;
}

void GSThreadedBackend::Initialize(uint8_t *vram, uint32_t vramSize)
{
    drain();
    m_inner->Initialize(vram, vramSize);
}

void GSThreadedBackend::Reset()
{
    drain();
    m_inner->Reset();
}

void GSThreadedBackend::setFieldCounter(const std::atomic<uint64_t> *counter)
{
    m_fieldCounter = counter;
}

void GSThreadedBackend::setPresentationSource(std::function<GSPresentationRequest()> source)
{
    m_presentationSource = std::move(source);
}

void GSThreadedBackend::paceToField()
{
    if (!m_fieldCounter)
        return;
    const uint64_t field = m_fieldCounter->load(std::memory_order_acquire);
    if (field == m_lastField)
        return;
    m_lastField = field;
    // Games drawing at 30 fps spread a frame over two fields; letting the GS
    // lag two fields keeps the EE and the raster threads overlapping.
    uint64_t oldestFence = 0;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldestFence = m_fieldFences[0];
        m_fieldFences[0] = m_fieldFences[1];
        m_fieldFences[1] = m_submitted;
    }
    // What the game displays this field was drawn by the work queued so far:
    // the worker snapshots it once that work is done, before drawing more.
    if (m_cpu && m_presentationSource)
    {
        Command capture;
        capture.kind = CommandKind::Capture;
        capture.presentation = m_presentationSource();
        push(std::move(capture));
    }
    waitForCompleted(oldestFence);
}

void GSThreadedBackend::captureDisplay(const GSPresentationRequest &request)
{
    m_inner->SnapshotVram(m_captureScratch);
    std::lock_guard<std::mutex> lock(m_captureMutex);
    m_capturedVram.swap(m_captureScratch);
    m_capturedRequest = request;
    ++m_captureSerial;
}

void GSThreadedBackend::Submit(const GSPrimitiveBatch &batch)
{
    paceToField();
    Command command;
    command.kind = CommandKind::Submit;
    command.batch = batch;
    push(std::move(command));
}

void GSThreadedBackend::BeginTransfer(const GSTransferCommand &command)
{
    // Local->host transfers are read back by the EE right away.
    if (command.direction == 1u)
    {
        // PS2X_TRACE_READBACK: average drain/readback cost every 300 transfers.
        static const bool traceReadback = std::getenv("PS2X_TRACE_READBACK") != nullptr;
        const auto start = std::chrono::steady_clock::now();
        drain();
        const auto drained = std::chrono::steady_clock::now();
        m_inner->BeginTransfer(command);
        if (traceReadback)
        {
            static double drainMs = 0.0, readMs = 0.0;
            static int count = 0;
            drainMs += std::chrono::duration<double, std::milli>(drained - start).count();
            readMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - drained).count();
            if (++count == 300)
            {
                std::fprintf(stderr, "[readback] avg drain %.2f ms, local->host %.2f ms\n", drainMs / count, readMs / count);
                drainMs = readMs = 0.0;
                count = 0;
            }
        }
        return;
    }
    Command queued;
    queued.kind = CommandKind::BeginTransfer;
    queued.transfer = command;
    push(std::move(queued));
}

void GSThreadedBackend::UploadImage(const uint8_t *data, uint32_t sizeBytes)
{
    if (!data || sizeBytes == 0u)
        return;
    Command command;
    command.kind = CommandKind::UploadImage;
    command.data.assign(data, data + sizeBytes);
    push(std::move(command));
}

void GSThreadedBackend::Flush()
{
    Command command;
    command.kind = CommandKind::Flush;
    push(std::move(command));
}

void GSThreadedBackend::TextureFlush()
{
    Command command;
    command.kind = CommandKind::TextureFlush;
    push(std::move(command));
}

void GSThreadedBackend::Sync(GSSyncReason reason)
{
    if (reason == GSSyncReason::Finish)
    {
        // The previous frame must be rendered before this one is queued
        // behind it: the EE stays at most one frame ahead of the GS.
        uint64_t previousFence = 0;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            previousFence = m_lastFinishFence;
            m_lastFinishFence = m_submitted + 1u;
        }
        Command command;
        command.kind = CommandKind::Sync;
        command.reason = reason;
        push(std::move(command));
        // PS2X_TRACE_GS_PERF: every 300 FINISHes, the average time between
        // them and how much of it the EE spent waiting for the GS here.
        static const bool tracePerf = std::getenv("PS2X_TRACE_GS_PERF") != nullptr;
        const auto waitStart = std::chrono::steady_clock::now();
        waitForCompleted(previousFence);
        if (tracePerf)
        {
            static auto windowStart = waitStart;
            static double waitMs = 0.0;
            static int frames = 0;
            const auto now = std::chrono::steady_clock::now();
            waitMs += std::chrono::duration<double, std::milli>(now - waitStart).count();
            if (++frames == 300)
            {
                const double frameMs = std::chrono::duration<double, std::milli>(now - windowStart).count() / frames;
                double subsystemMs[4];
                for (int i = 0; i < 4; ++i)
                    subsystemMs[i] = static_cast<double>(g_ps2xPerfNs[i].exchange(0)) / 1e6 / frames;
                std::fprintf(stderr,
                             "[gs-perf] %.2f ms per FINISH, EE waiting for GS %.2f ms; VIF1 %.2f (VU1 %.2f, GS packets %.2f), VU0 %.2f ms\n",
                             frameMs, waitMs / frames, subsystemMs[0], subsystemMs[1], subsystemMs[2], subsystemMs[3]);
                windowStart = now;
                waitMs = 0.0;
                frames = 0;
            }
        }
        return;
    }
    // Present() picks a completed field itself.
    if (reason == GSSyncReason::Presentation)
        return;
    drain();
    m_inner->Sync(reason);
}

PresentationFrame GSThreadedBackend::Present(const GSPresentationRequest &request)
{
    if (!m_cpu)
    {
        drain();
        return m_inner->Present(request);
    }
    // Never wait for the raster threads: show the newest snapshot the worker
    // took between fields. A screen the game no longer draws to gets no field
    // snapshots, so while the GS is idle ask for one of the live display
    // state (shown from the next present on).
    bool idle = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        idle = m_completed == m_submitted;
    }
    if (idle)
    {
        Command capture;
        capture.kind = CommandKind::Capture;
        capture.presentation = request;
        push(std::move(capture));
    }

    GSPresentationRequest captured{};
    {
        std::lock_guard<std::mutex> lock(m_captureMutex);
        if (m_captureSerial == m_presentedSerial || m_capturedVram.empty())
            return m_lastFrame;
        m_presentVram = m_capturedVram;
        captured = m_capturedRequest;
        m_presentedSerial = m_captureSerial;
    }
    PresentationFrame frame = GSCpuBackend::PresentSnapshot(m_presentVram, captured);
    if (frame)
        m_lastFrame = frame;
    return m_lastFrame;
}

bool GSThreadedBackend::ClearFramebuffer(const GSContext &context, uint32_t rgba)
{
    drain();
    return m_inner->ClearFramebuffer(context, rgba);
}

uint32_t GSThreadedBackend::ConsumeLocalToHostBytes(uint8_t *dst, uint32_t maxBytes)
{
    drain();
    return m_inner->ConsumeLocalToHostBytes(dst, maxBytes);
}

uint32_t GSThreadedBackend::ReadVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y) const
{
    drain();
    return m_inner->ReadVram(psm, base, bw, x, y);
}

void GSThreadedBackend::WriteVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y, uint32_t value)
{
    drain();
    m_inner->WriteVram(psm, base, bw, x, y, value);
}

void GSThreadedBackend::SnapshotVram(std::vector<uint8_t> &out) const
{
    drain();
    m_inner->SnapshotVram(out);
}

GSTransferSnapshot GSThreadedBackend::GetTransferSnapshot() const
{
    drain();
    return m_inner->GetTransferSnapshot();
}
