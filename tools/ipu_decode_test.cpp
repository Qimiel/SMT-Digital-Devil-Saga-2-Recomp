// Standalone check for the IPU decoder core: decodes frames of a Sony "ipum"
// stream (e.g. MOVIE3/DS2_LOGO.IPU) with the same command sequence DDS2's movie
// player issues (0x00344F08), and writes them as PNG files.
//
//   ipu_decode_test <file.ipu> <out_prefix> [first_frame] [frame_count]
#include "../runtime/ps2xRuntime/src/lib/ipu/ps2_ipu_core.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace
{
    class FileSource : public ps2_ipu::IpuInputSource
    {
    public:
        FileSource(const std::vector<uint8_t> &data, size_t offset) : m_data(data), m_pos(offset) {}
        bool pull(ps2_ipu::Qword &out) override
        {
            if (m_pos >= m_data.size())
                return false;
            out.fill(0);
            const size_t n = std::min<size_t>(16, m_data.size() - m_pos);
            std::memcpy(out.data(), m_data.data() + m_pos, n);
            m_pos += 16;
            return true;
        }

    private:
        const std::vector<uint8_t> &m_data;
        size_t m_pos;
    };

    uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc = 0)
    {
        crc = ~crc;
        for (size_t i = 0; i < len; ++i)
        {
            crc ^= data[i];
            for (int k = 0; k < 8; ++k)
                crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
        return ~crc;
    }

    void put32(std::vector<uint8_t> &v, uint32_t x)
    {
        v.push_back(static_cast<uint8_t>(x >> 24));
        v.push_back(static_cast<uint8_t>(x >> 16));
        v.push_back(static_cast<uint8_t>(x >> 8));
        v.push_back(static_cast<uint8_t>(x));
    }

    void chunk(std::vector<uint8_t> &png, const char *type, const std::vector<uint8_t> &data)
    {
        put32(png, static_cast<uint32_t>(data.size()));
        std::vector<uint8_t> body(type, type + 4);
        body.insert(body.end(), data.begin(), data.end());
        png.insert(png.end(), body.begin(), body.end());
        put32(png, crc32(body.data(), body.size()));
    }

    // Minimal PNG writer (RGBA, uncompressed deflate blocks).
    void writePng(const std::string &path, int w, int h, const std::vector<uint8_t> &rgba)
    {
        std::vector<uint8_t> raw;
        for (int y = 0; y < h; ++y)
        {
            raw.push_back(0);
            raw.insert(raw.end(), rgba.begin() + y * w * 4, rgba.begin() + (y + 1) * w * 4);
        }
        std::vector<uint8_t> z = {0x78, 0x01};
        uint32_t a = 1, b = 0;
        for (uint8_t c : raw)
        {
            a = (a + c) % 65521;
            b = (b + a) % 65521;
        }
        for (size_t pos = 0; pos < raw.size();)
        {
            const size_t n = std::min<size_t>(65535, raw.size() - pos);
            z.push_back(pos + n == raw.size() ? 1 : 0);
            z.push_back(static_cast<uint8_t>(n));
            z.push_back(static_cast<uint8_t>(n >> 8));
            z.push_back(static_cast<uint8_t>(~n));
            z.push_back(static_cast<uint8_t>(~n >> 8));
            z.insert(z.end(), raw.begin() + pos, raw.begin() + pos + n);
            pos += n;
        }
        put32(z, (b << 16) | a);
        std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
        std::vector<uint8_t> ihdr;
        put32(ihdr, static_cast<uint32_t>(w));
        put32(ihdr, static_cast<uint32_t>(h));
        ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0});
        chunk(png, "IHDR", ihdr);
        chunk(png, "IDAT", z);
        chunk(png, "IEND", {});
        std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char *>(png.data()), static_cast<std::streamsize>(png.size()));
    }
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::fprintf(stderr, "usage: %s file.ipu out_prefix [first] [count]\n", argv[0]);
        return 1;
    }
    std::ifstream in(argv[1], std::ios::binary);
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(in)), {});
    if (data.size() < 16 || std::memcmp(data.data(), "ipum", 4) != 0)
    {
        std::fprintf(stderr, "not an ipum stream\n");
        return 1;
    }
    const int width = data[8] | (data[9] << 8);
    const int height = data[10] | (data[11] << 8);
    const uint32_t frames = data[12] | (data[13] << 8) | (data[14] << 16) | (static_cast<uint32_t>(data[15]) << 24);
    const uint32_t first = argc > 3 ? static_cast<uint32_t>(std::stoul(argv[3])) : 0u;
    const uint32_t count = argc > 4 ? static_cast<uint32_t>(std::stoul(argv[4])) : 3u;
    std::printf("%dx%d, %u frames\n", width, height, frames);

    FileSource source(data, 16);
    ps2_ipu::IpuCore ipu;
    ipu.setInput(&source);
    // sceIpuInit: default intra matrix etc. are loaded from the bitstream by the
    // game; feed MPEG-2 defaults directly here.
    static const uint8_t defaultIntra[64] = {
        8, 16, 16, 19, 16, 19, 22, 22, 22, 22, 22, 22, 26, 24, 26, 27, 27, 27, 26, 26, 26, 26, 27, 27,
        27, 29, 29, 29, 34, 34, 34, 29, 29, 29, 27, 27, 29, 29, 32, 32, 34, 34, 37, 38, 37, 35, 35, 34,
        35, 38, 38, 40, 40, 40, 48, 48, 46, 46, 56, 56, 58, 69, 69, 83};
    {
        ps2_ipu::IpuCore init;
        struct Mem : ps2_ipu::IpuInputSource
        {
            const uint8_t *p;
            int n;
            bool pull(ps2_ipu::Qword &q) override
            {
                if (n <= 0)
                    return false;
                std::memcpy(q.data(), p, 16);
                p += 16;
                n -= 16;
                return true;
            }
        } mem;
        mem.p = defaultIntra;
        mem.n = 64;
        ipu.setInput(&mem);
        ipu.command(0x00000000u); // BCLR
        ipu.command(0x50000000u); // SETIQ intra
        ipu.setInput(&source);
        ipu.command(0x00000000u); // BCLR
    }

    const int mbw = (width + 15) / 16;
    const int mbh = (height + 15) / 16;
    std::vector<uint8_t> image(static_cast<size_t>(mbw) * 16 * mbh * 16 * 4);
    for (uint32_t f = 0; f < first + count && f < frames; ++f)
    {
        if (f != 0)
            ipu.command(0x40000020u); // skip the start code ending the previous picture
        ipu.command(0x40000000u);     // FDEC: peek header
        const uint32_t header = ipu.commandData();
        ipu.command(0x40000008u);     // consume the flag byte
        ipu.setControl((header >> 8) & 0xFB0000u);
        const uint32_t dtd = (header & 0x4000000u) ? 0x1010000u : 0x10000u;
        ipu.output().clear();
        ipu.command(0x10000000u | dtd); // IDEC, RGB32, QSC=1
        if (ipu.busy())
        {
            std::fprintf(stderr, "frame %u: decoder still busy (input exhausted)\n", f);
            return 1;
        }
        const size_t mbs = ipu.output().size() / 64;
        if ((ipu.control() >> 14) & 1u)
            std::fprintf(stderr, "frame %u: ECD set after %zu macroblocks\n", f, mbs);
        if (f < first)
            continue;
        std::fill(image.begin(), image.end(), 0);
        size_t mb = 0;
        while (ipu.output().size() >= 64 && mb < static_cast<size_t>(mbw * mbh))
        {
            const int mx = static_cast<int>(mb % mbw), my = static_cast<int>(mb / mbw);
            for (int q = 0; q < 64; ++q)
            {
                const ps2_ipu::Qword qw = ipu.output().front();
                ipu.output().pop_front();
                const int row = q / 4, col = (q % 4) * 4;
                std::memcpy(&image[((my * 16 + row) * mbw * 16 + mx * 16 + col) * 4], qw.data(), 16);
            }
            ++mb;
        }
        for (size_t i = 3; i < image.size(); i += 4)
            image[i] = 255;
        std::printf("frame %u: header=%08x macroblocks=%zu scd=%u\n", f, header, mbs, (ipu.control() >> 15) & 1u);
        writePng(std::string(argv[2]) + "_" + std::to_string(f) + ".png", mbw * 16, mbh * 16, image);
    }
    return 0;
}
