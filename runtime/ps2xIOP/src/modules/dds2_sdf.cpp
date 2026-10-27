// HLE for the Atlus "SDF" IOP modules used by Digital Devil Saga 2 (SLUS_211.52).
//
// SDFIOP.IRX ("sdf_iop_manager") is an IOP-side module loader reached through
// MSIFRPC (sceSifM*). EE request layout, from the client at 0x00329B18:
//   u32 pathLength, u32 argLength, char path[pathLength + 1], char args[argLength]
// reply: s32 result (module id or <0), s32 module start result.
//
// A separate IOP debug-print service (EE sender 0x0033DA30) receives a
// NUL-terminated string through function 0x7D1.
#include "module_factories.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace ps2x::iop::detail
{
    namespace
    {
        constexpr uint32_t kSdIoSid = 0x6F496453u;    // "SdIo"
        constexpr uint32_t kPrintSid = 0x646E7270u;   // "prnd"
        constexpr uint32_t kSdIoLoadStartModule = 1u;
        constexpr uint32_t kPrintFunction = 0x7D1u;

        std::string readGuestString(IopHost &host, uint32_t address, uint32_t maxBytes)
        {
            std::string out;
            for (uint32_t i = 0; i < maxBytes; ++i)
            {
                char c = 0;
                if (!host.readGuest(address + i, &c, 1) || c == '\0')
                    break;
                out.push_back(c);
            }
            return out;
        }

        class Dds2SdfService final : public IopService
        {
        public:
            explicit Dds2SdfService(IopHost &host) : m_host(host) {}

            [[nodiscard]] std::string_view name() const override { return "dds2-sdf"; }
            [[nodiscard]] std::span<const uint32_t> sids() const override { return kSids; }

            void reset() override
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_nextModuleId = 0x100;
                m_loaded.clear();
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                if (request.sid == kSdIoSid)
                {
                    result.handled = true;
                    result.resultAddress = request.receive.address;
                    if (request.function == kSdIoLoadStartModule)
                        loadStartModule(request);
                    else
                        logUnknown(request);
                    return result;
                }
                if (request.sid == kPrintSid)
                {
                    result.handled = true;
                    result.resultAddress = request.receive.address;
                    if (request.function == kPrintFunction && request.send.address != 0u)
                    {
                        std::string text = readGuestString(m_host, request.send.address,
                                                           request.send.size ? request.send.size : 256u);
                        while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
                            text.pop_back();
                        m_host.log(LogLevel::Info, "[IOP] " + text);
                    }
                    return result;
                }
                return result;
            }

            void appendDebugMetrics(std::vector<DebugMetric> &metrics) const override
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                metrics.push_back({"sdf_modules_loaded", m_loaded.size(), false});
            }

        private:
            void loadStartModule(const RpcRequest &request)
            {
                uint32_t pathLength = 0;
                (void)m_host.readGuest(request.send.address, &pathLength, sizeof(pathLength));
                const std::string path =
                    readGuestString(m_host, request.send.address + 8u, std::min<uint32_t>(pathLength + 1u, 256u));

                int32_t id = 0;
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    id = m_nextModuleId++;
                    m_loaded.push_back(path);
                }
                m_host.log(LogLevel::Info, "[SDFIOP] sdfLoadStartModule " + path + " -> HLE id " + std::to_string(id));

                if (request.receive.address != 0u && request.receive.size >= 8u)
                {
                    const int32_t reply[2] = {id, 0};
                    (void)m_host.writeGuest(request.receive.address, reply, sizeof(reply));
                }
            }

            void logUnknown(const RpcRequest &request)
            {
                std::ostringstream message;
                message << "[SDFIOP] unhandled rpc 0x" << std::hex << request.function
                        << " send=0x" << request.send.address << "/0x" << request.send.size
                        << " recv=0x" << request.receive.address << "/0x" << request.receive.size;
                m_host.log(LogLevel::Warning, message.str());
                if (request.receive.address != 0u && request.receive.size != 0u)
                    (void)m_host.zeroGuest(request.receive.address, request.receive.size);
            }

            inline static constexpr std::array<uint32_t, 2> kSids{kSdIoSid, kPrintSid};

            IopHost &m_host;
            mutable std::mutex m_mutex;
            int32_t m_nextModuleId = 0x100;
            std::vector<std::string> m_loaded;
        };
    }

    std::unique_ptr<IopService> createDds2SdfService(IopHost &host)
    {
        return std::make_unique<Dds2SdfService>(host);
    }
}
