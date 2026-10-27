// DDS2_PROFILE=<seconds>: from that many seconds after start, samples the
// instruction pointer of every thread in the process about once a millisecond
// and rewrites profile.txt every few seconds (the process is usually killed
// rather than closed). tools/profile_report.py turns it into a per-thread
// symbol histogram.
#include "dds2_profiler.h"

#include <windows.h>
#include <dbghelp.h>
#include <psapi.h>
#include <tlhelp32.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace dds2_profiler
{
    namespace
    {
        struct SampledThread
        {
            DWORD id = 0;
            HANDLE handle = nullptr;
            std::unordered_map<uint64_t, uint32_t> hits;
            std::unordered_map<uint64_t, uint32_t> callers;
        };

        void refreshThreads(std::vector<SampledThread> &threads, DWORD self)
        {
            HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
            if (snapshot == INVALID_HANDLE_VALUE)
                return;
            THREADENTRY32 entry{};
            entry.dwSize = sizeof(entry);
            const DWORD pid = GetCurrentProcessId();
            for (BOOL ok = Thread32First(snapshot, &entry); ok; ok = Thread32Next(snapshot, &entry))
            {
                if (entry.th32OwnerProcessID != pid || entry.th32ThreadID == self)
                    continue;
                bool known = false;
                for (const SampledThread &thread : threads)
                    known |= thread.id == entry.th32ThreadID;
                if (known)
                    continue;
                HANDLE handle = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION,
                                           FALSE, entry.th32ThreadID);
                if (handle)
                    threads.push_back({entry.th32ThreadID, handle, {}});
            }
            CloseHandle(snapshot);
        }

        // Names addresses outside the executable (system DLL exports) so that
        // waits, locks and CRT copies show up separately in the report.
        std::string externalSymbol(uint64_t address)
        {
            static std::unordered_map<uint64_t, std::string> cache;
            const auto cached = cache.find(address);
            if (cached != cache.end())
                return cached->second;

            std::string name;
            HMODULE module = nullptr;
            if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                   reinterpret_cast<LPCWSTR>(address), &module))
            {
                char path[MAX_PATH] = {};
                GetModuleFileNameA(module, path, MAX_PATH);
                const char *slash = std::strrchr(path, '\\');
                name = slash ? slash + 1 : path;
            }
            alignas(SYMBOL_INFO) char buffer[sizeof(SYMBOL_INFO) + 256] = {};
            auto *symbol = reinterpret_cast<SYMBOL_INFO *>(buffer);
            symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
            symbol->MaxNameLen = 255;
            DWORD64 displacement = 0;
            if (SymFromAddr(GetCurrentProcess(), address, &displacement, symbol))
                name += std::string("!") + symbol->Name;
            for (char &c : name)
                if (c == ' ')
                    c = '_';
            cache.emplace(address, name);
            return name;
        }

        void writeProfile(const std::vector<SampledThread> &threads)
        {
            FILE *f = std::fopen("profile.txt", "w");
            if (!f)
                return;
            const HMODULE exe = GetModuleHandleW(nullptr);
            MODULEINFO exeInfo{};
            GetModuleInformation(GetCurrentProcess(), exe, &exeInfo, sizeof(exeInfo));
            const uint64_t exeBegin = reinterpret_cast<uintptr_t>(exe);
            const uint64_t exeEnd = exeBegin + exeInfo.SizeOfImage;
            std::fprintf(f, "base %llx\n", static_cast<unsigned long long>(exeBegin));
            for (const SampledThread &thread : threads)
                for (const auto &[rip, count] : thread.hits)
                {
                    if (rip >= exeBegin && rip < exeEnd)
                        std::fprintf(f, "%lu %llx %u\n", thread.id, static_cast<unsigned long long>(rip), count);
                    else
                        std::fprintf(f, "%lu %llx %u %s\n", thread.id, static_cast<unsigned long long>(rip), count,
                                     externalSymbol(rip).c_str());
                }
            // Callers of the sampled routines outside the executable.
            for (const SampledThread &thread : threads)
                for (const auto &[caller, count] : thread.callers)
                    std::fprintf(f, "caller %lu %llx %u\n", thread.id, static_cast<unsigned long long>(caller), count);
            std::fclose(f);
        }
    }

    void start(std::atomic<bool> &stop)
    {
        const char *env = std::getenv("DDS2_PROFILE");
        if (!env)
            return;
        const double delaySeconds = std::atof(env);
        std::thread([&stop, delaySeconds]()
                    {
            const auto begin = std::chrono::steady_clock::now() + std::chrono::duration<double>(delaySeconds);
            while (!stop.load() && std::chrono::steady_clock::now() < begin)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const DWORD self = GetCurrentThreadId();
            SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
            SymInitialize(GetCurrentProcess(), nullptr, TRUE);
            const HMODULE exeModule = GetModuleHandleW(nullptr);
            MODULEINFO exeModuleInfo{};
            GetModuleInformation(GetCurrentProcess(), exeModule, &exeModuleInfo, sizeof(exeModuleInfo));
            const uint64_t exeBegin = reinterpret_cast<uintptr_t>(exeModule);
            const uint64_t exeEnd = exeBegin + exeModuleInfo.SizeOfImage;
            std::vector<SampledThread> threads;
            auto lastRefresh = std::chrono::steady_clock::now() - std::chrono::seconds(10);
            auto lastWrite = std::chrono::steady_clock::now();
            timeBeginPeriod(1);
            while (!stop.load())
            {
                const auto now = std::chrono::steady_clock::now();
                if (now - lastRefresh > std::chrono::seconds(1))
                {
                    refreshThreads(threads, self);
                    lastRefresh = now;
                }
                for (SampledThread &thread : threads)
                {
                    if (SuspendThread(thread.handle) == static_cast<DWORD>(-1))
                        continue;
                    // Nothing that can allocate or lock may run while the
                    // thread is suspended: it might own the heap lock.
                    CONTEXT context{};
                    context.ContextFlags = CONTEXT_CONTROL;
                    const bool sampled = GetThreadContext(thread.handle, &context) != 0;
                    // In a leaf routine outside the executable (memset and
                    // the like) the word at the stack pointer is the return
                    // address: it names the caller.
                    uint64_t caller = 0;
                    if (sampled && (context.Rip < exeBegin || context.Rip >= exeEnd))
                        std::memcpy(&caller, reinterpret_cast<const void *>(context.Rsp), sizeof(caller));
                    ResumeThread(thread.handle);
                    if (sampled)
                    {
                        ++thread.hits[context.Rip];
                        if (caller >= exeBegin && caller < exeEnd)
                            ++thread.callers[caller];
                    }
                }
                if (now - lastWrite > std::chrono::seconds(5))
                {
                    writeProfile(threads);
                    lastWrite = now;
                }
                Sleep(1);
            }
            writeProfile(threads); })
            .detach();
    }
}
