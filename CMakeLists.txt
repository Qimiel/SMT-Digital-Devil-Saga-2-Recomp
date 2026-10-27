cmake_minimum_required(VERSION 3.21)
project(DDS2Recomp LANGUAGES C CXX)

# Native Windows build of the statically recompiled DDS2 (SLUS_211.52) executable.
# The guest code is the PS2Recomp translation in recovered/code/recompiled, patched
# by tools/patch_guest.py into ./guest; the runtime is a vendored, DDS2-patched copy
# of ps2xRuntime/ps2xIOP (upstream commit 14b1e5cb39b4af7e6fc12f9a29fdc751efde49d7,
# GPL-3.0, see runtime/LICENSE).

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(DDS2_GUEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/guest" CACHE PATH "Patched PS2Recomp output for SLUS_211.52 (tools/patch_guest.py)")
set(DDS2_GUEST_SOURCE_DIR "${DDS2_GUEST_DIR}")
set(DDS2_GUEST_OPT "-O1" CACHE STRING "Optimization flag for normal-size guest translation units")
set(DDS2_GUEST_LARGE_OPT "-O1" CACHE STRING "Optimization flag for guest units above DDS2_GUEST_LARGE_BYTES")
set(DDS2_GUEST_LARGE_BYTES 300000 CACHE STRING "Guest units larger than this are compiled alone")
set(DDS2_GUEST_UNITY_BATCH 48 CACHE STRING "Guest unity batch size")
option(DDS2_RUNTIME_LOGS "Enable PS2 runtime logs" ON)

set(RT "${CMAKE_CURRENT_SOURCE_DIR}/runtime/ps2xRuntime")
set(IOP "${CMAKE_CURRENT_SOURCE_DIR}/runtime/ps2xIOP")

if(MSVC)
    message(FATAL_ERROR "This project is set up for the MinGW-w64 GCC toolchain (build.ps1)")
endif()

# ---------------------------------------------------------------- host deps
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(CUSTOMIZE_BUILD OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib
    URL "https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz"
    URL_HASH SHA256=aea98ecf5bc5c5e0b789a76de0083a21a70457050ea4cc2aec7566935f5e258e
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(raylib)

# ---------------------------------------------------------------- IOP HLE
add_library(ps2_iop STATIC
    ${IOP}/src/iop_subsystem.cpp
    ${IOP}/src/builtin_profiles.cpp
    ${IOP}/src/plugin_loader.cpp
    ${IOP}/src/modules/dbcman.cpp
    ${IOP}/src/modules/libsd.cpp
    ${IOP}/src/modules/mcserv.cpp
    ${IOP}/src/modules/tsnddrv.cpp
    ${IOP}/src/modules/cri_dtx.cpp
    ${IOP}/src/modules/clfile.cpp
    ${IOP}/src/modules/sound_update_stub.cpp
    ${IOP}/src/modules/sdrdrv.cpp
    ${IOP}/src/modules/dds2_sdf.cpp)
target_include_directories(ps2_iop PUBLIC ${IOP}/include PRIVATE ${IOP}/src)
target_compile_definitions(ps2_iop PUBLIC PS2X_IOP_ENABLE_PLUGINS=0)
target_compile_options(ps2_iop PRIVATE -O2)

# ---------------------------------------------------------------- runtime
file(GLOB_RECURSE KERNEL_SRC CONFIGURE_DEPENDS "${RT}/src/lib/Kernel/*.cpp")
add_library(ps2_runtime STATIC
    ${RT}/src/lib/game_overrides.cpp
    ${RT}/src/lib/gs/ps2_gif_arbiter.cpp
    ${RT}/src/lib/ps2_audio.cpp
    ${RT}/src/lib/ps2_audio_vag.cpp
    ${RT}/src/lib/gs/ps2_gs_memory.cpp
    ${RT}/src/lib/gs/gs_frontend.cpp
    ${RT}/src/lib/gs/gs_cpu_backend.cpp
    ${RT}/src/lib/gs/gs_threaded_backend.cpp
    ${RT}/src/lib/ps2_iop_host.cpp
    ${RT}/src/lib/ps2_memory.cpp
    ${RT}/src/lib/ps2_pad.cpp
    ${RT}/src/lib/ps2_runtime.cpp
    ${RT}/src/lib/ps2_vif1_interpreter.cpp
    ${RT}/src/lib/ipu/ps2_ipu_core.cpp
    ${RT}/src/lib/ipu/ps2_ipu.cpp
    ${RT}/src/lib/vu/ps2_vu1_core.cpp
    ${RT}/src/lib/vu/ps2_vu1_upper.cpp
    ${RT}/src/lib/vu/ps2_vu1_lower.cpp
    ${RT}/src/lib/games_database.cpp
    ${KERNEL_SRC})
target_include_directories(ps2_runtime PUBLIC
    ${RT}/include
    ${RT}/src/lib/Kernel
    ${DDS2_GUEST_SOURCE_DIR})
target_compile_definitions(ps2_runtime PRIVATE PS2X_HAS_FFMPEG=0)
target_compile_definitions(ps2_runtime PUBLIC PS2X_ENABLE_IOP_RPC_TRACE=1)
if(DDS2_RUNTIME_LOGS)
    target_compile_definitions(ps2_runtime PUBLIC PS2_RUNTIME_LOGS=1)
endif()
target_compile_options(ps2_runtime PUBLIC -msse4.1 PRIVATE -O2 -g1)
target_link_libraries(ps2_runtime PUBLIC raylib ps2_iop winmm)

# ---------------------------------------------------------------- guest code
file(GLOB GUEST_SRC CONFIGURE_DEPENDS "${DDS2_GUEST_SOURCE_DIR}/*.cpp")
list(FILTER GUEST_SRC EXCLUDE REGEX "/register_functions\\.cpp$")
if(NOT GUEST_SRC)
    message(FATAL_ERROR "No generated guest sources found in ${DDS2_GUEST_SOURCE_DIR}")
endif()

set(GUEST_SMALL)
set(GUEST_LARGE)
foreach(src IN LISTS GUEST_SRC)
    file(SIZE "${src}" sz)
    if(sz GREATER DDS2_GUEST_LARGE_BYTES)
        list(APPEND GUEST_LARGE "${src}")
    else()
        list(APPEND GUEST_SMALL "${src}")
    endif()
endforeach()
list(LENGTH GUEST_SMALL n_small)
list(LENGTH GUEST_LARGE n_large)
message(STATUS "DDS2 guest units: ${n_small} batched, ${n_large} standalone")

# Common precompiled header used by every guest unit.
set(GUEST_PCH_HEADERS
    "${RT}/include/ps2_runtime_macros.h"
    "${RT}/include/ps2_runtime.h"
    "${DDS2_GUEST_SOURCE_DIR}/ps2_recompiled_functions.h"
    "${DDS2_GUEST_SOURCE_DIR}/ps2_recompiled_stubs.h"
    "${RT}/include/ps2_syscalls.h"
    "${RT}/include/ps2_stubs.h")

add_library(dds2_guest_small OBJECT ${GUEST_SMALL})
set_target_properties(dds2_guest_small PROPERTIES
    UNITY_BUILD ON
    UNITY_BUILD_BATCH_SIZE ${DDS2_GUEST_UNITY_BATCH})
target_precompile_headers(dds2_guest_small PRIVATE ${GUEST_PCH_HEADERS})
target_compile_options(dds2_guest_small PRIVATE ${DDS2_GUEST_OPT} -g0 -fno-strict-aliasing -w)
target_link_libraries(dds2_guest_small PRIVATE ps2_runtime)

add_library(dds2_guest_large OBJECT ${GUEST_LARGE})
target_precompile_headers(dds2_guest_large REUSE_FROM dds2_guest_small)
target_compile_options(dds2_guest_large PRIVATE ${DDS2_GUEST_LARGE_OPT} -g0 -fno-strict-aliasing -w)
target_link_libraries(dds2_guest_large PRIVATE ps2_runtime)

# The 12 MB dispatch-table initializer only needs to be compiled once, cheaply.
add_library(dds2_guest_table OBJECT "${DDS2_GUEST_SOURCE_DIR}/register_functions.cpp")
target_compile_options(dds2_guest_table PRIVATE -O0 -g0 -w)
target_link_libraries(dds2_guest_table PRIVATE ps2_runtime)

# ---------------------------------------------------------------- VU1 microcode
# VU1 microprograms translated to C++ by tools/vu1_translate.py from images
# harvested with PS2X_VU1_HARVEST (see src/vu1rec).
file(GLOB VU1GEN_SRC CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/vu1gen/*.cpp")
add_library(dds2_vu1gen OBJECT ${VU1GEN_SRC})
target_include_directories(dds2_vu1gen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
# No FMA contraction: VU maths rounds the product before the sum. The code
# only type-puns through memcpy, so strict aliasing lets the compiler keep
# cycle counters and registers in host registers across float stores.
target_compile_options(dds2_vu1gen PRIVATE -O2 -g1 -ffp-contract=off -w)
target_link_libraries(dds2_vu1gen PRIVATE ps2_runtime)

# ---------------------------------------------------------------- executable
add_executable(dds2
    src/main.cpp
    src/dds2_overrides.cpp
    src/dds2_sound.cpp
    src/dds2_profiler.cpp
    src/dds2_debug.cpp
    src/dds2_platform.cpp
    src/vu1rec/vu1rec.cpp
    $<TARGET_OBJECTS:dds2_vu1gen>
    $<TARGET_OBJECTS:dds2_guest_small>
    $<TARGET_OBJECTS:dds2_guest_large>
    $<TARGET_OBJECTS:dds2_guest_table>)
target_include_directories(dds2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(dds2 PRIVATE -O2 -g1 -Wall)
target_link_libraries(dds2 PRIVATE ps2_runtime dbghelp psapi)
target_link_options(dds2 PRIVATE -static -Wl,--stack,16777216)
