#pragma once

#include "runtime/gs/gs_backend.h"

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

class GSCpuBackend final : public GSRasterBackend
{
public:
    // A texture decoded to RGBA (CLUT and TEXA applied), row-major.
    struct DecodedTexture
    {
        std::array<uint32_t, 12> key{};
        uint32_t width = 0;
        uint32_t height = 0;
        uint64_t lastUse = 0;
        std::vector<uint32_t> texels;
    };

    GSCpuBackend();

    void Initialize(uint8_t *vram, uint32_t vramSize) override;
    void Reset() override;

    void Submit(const GSPrimitiveBatch &batch) override;
    // Draws only the rows of band bandIndex (8-row stripes, interleaved over a
    // power-of-two bandCount) without taking the backend lock, so several
    // threads can rasterize one primitive stream concurrently. A textured
    // batch samples `texture` (from PrepareTexture), or GS memory when null.
    void DrawBanded(const GSPrimitiveBatch &batch, uint32_t bandIndex, uint32_t bandCount,
                    const DecodedTexture *texture);
    // Decodes the texture a draw samples into the texture cache and returns
    // it (null when the draw must sample GS memory directly). Banded
    // rendering calls this from its dispatcher between runs, since raster
    // threads only read the cache. The result stays valid until
    // ReleaseRetiredTextures(), even if GS memory changes meanwhile.
    const DecodedTexture *PrepareTexture(const GSDrawState &state);
    void ReleaseRetiredTextures();
    // Records that draws wrote VRAM bytes [begin, end), which invalidates
    // decoded textures and CLUTs read from there.
    void NoteVramWrites(uint32_t begin, uint32_t end);
    void BeginTransfer(const GSTransferCommand &command) override;
    void UploadImage(const uint8_t *data, uint32_t sizeBytes) override;

    void Flush() override;
    void TextureFlush() override;
    void Sync(GSSyncReason reason) override;
    PresentationFrame Present(const GSPresentationRequest &request) override;
    // Builds the displayed image from a copy of GS memory (SnapshotVram).
    static PresentationFrame PresentSnapshot(std::vector<uint8_t> &vram, const GSPresentationRequest &request);

    bool ClearFramebuffer(const GSContext &context, uint32_t rgba) override;
    uint32_t ConsumeLocalToHostBytes(uint8_t *dst, uint32_t maxBytes) override;

    uint32_t ReadVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y) const override;
    void WriteVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y, uint32_t value) override;
    void SnapshotVram(std::vector<uint8_t> &out) const override;
    GSTransferSnapshot GetTransferSnapshot() const override;

private:
    const DecodedTexture *FindTexture(const GSDrawState &state, bool allowDecode);
    void RetireTexture(std::vector<std::unique_ptr<DecodedTexture>>::iterator entry);

    void ResetUnlocked();
    uint32_t ReadVramUnlocked(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y) const;
    void WriteVramUnlocked(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y, uint32_t value);

    void DrawPrimitive(const GSPrimitiveBatch &batch);
    void DrawSprite(const GSPrimitiveBatch &batch);
    void DrawTriangle(const GSPrimitiveBatch &batch);
    void DrawLine(const GSPrimitiveBatch &batch);
    void WritePixel(const GSDrawState &state, int x, int y, int z, uint8_t r, uint8_t g, uint8_t b, uint8_t a, uint8_t fog);
    uint32_t SampleTexture(const GSDrawState &state, const DecodedTexture *decoded, float s, float t, float q, uint16_t u, uint16_t v);
    uint32_t LookupCLUT(const GSDrawState &state, uint8_t index, uint32_t cbp, uint8_t cpsm, uint8_t csm, uint8_t csa, uint8_t sourcePsm);

    void PerformLocalToLocalTransfer();
    void PerformLocalToHostTransfer();
    PresentationFrame PresentFromLocalMemory(const GSPresentationRequest &request);
    bool CopyFrameToHostRgba(const GSFrameReg &frame,
                             uint32_t width,
                             uint32_t height,
                             std::vector<uint8_t> &outPixels,
                             bool preserveAlpha,
                             bool useLocalMemoryLayout,
                             bool frameBaseIsPages,
                             uint32_t sourceOriginX,
                             uint32_t sourceOriginY) const;

    // Plain function pointers: these run for every pixel the rasterizer touches.
    using WriteVramFunc = void (*)(uint8_t *, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
    using ReadVramFunc = uint32_t (*)(uint8_t *, uint32_t, uint32_t, uint32_t, uint32_t);

    static constexpr size_t kPsmHandlerCount = 1u << 6u;
    mutable std::mutex m_mutex;
    uint8_t *m_vram = nullptr;
    uint32_t m_vramSize = 0;
    std::array<ReadVramFunc, kPsmHandlerCount> m_readVramFuncs{};
    std::array<WriteVramFunc, kPsmHandlerCount> m_writeVramFuncs{};

    GSTransferCommand m_transfer{};
    GSTransferSnapshot m_transferState{};
    std::vector<uint8_t> m_localToHostBuffer;
    size_t m_localToHostReadPos = 0;

    // Decoded textures, least recently used evicted first. Only modified by
    // the drawing thread when no raster bands are running.
    std::vector<std::unique_ptr<DecodedTexture>> m_textureCache;
    // Replaced or evicted decodes that queued draws may still sample.
    std::vector<std::unique_ptr<DecodedTexture>> m_retiredTextures;
    size_t m_textureCacheBytes = 0;
    uint64_t m_textureUseCounter = 0;
};
