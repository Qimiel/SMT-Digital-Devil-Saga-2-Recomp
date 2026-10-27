#pragma once

#include "ps2_stubs.h"
#include "ps2_log.h"
#include "ps2_runtime.h"
#include "ps2_runtime_macros.h"
#include "ps2_syscalls.h"
#include "Unimplemented.h"

#include <iostream>
#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <filesystem>
#include <mutex>
#include <atomic>
#include <limits>

#include "ps2_host_backend.h"
#include "Helpers/Support.h"

namespace ps2_stubs
{
    // PS2X_GUEST_PRINTF=1 echoes guest printf/scePrintf output to the host
    // stdout in normal builds (otherwise only aggressive-log builds show it).
    inline void echoGuestPrintf(const std::string &rendered)
    {
        static const bool enabled = std::getenv("PS2X_GUEST_PRINTF") != nullptr;
        if (!enabled)
            return;
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        std::fwrite(rendered.data(), 1, rendered.size(), stdout);
        std::fflush(stdout);
    }
}
