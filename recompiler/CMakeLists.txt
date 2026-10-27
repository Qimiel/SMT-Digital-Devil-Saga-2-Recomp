cmake_minimum_required(VERSION 3.21)
project(DDS2Recompiler LANGUAGES C CXX)

# Vendored PS2Recomp recompiler (upstream 14b1e5c, GPL-3.0) with DDS2 additions:
#   * general.extra_entry_points_file: extra guest entry addresses (one hex
#     address per line) for code reached only through pointers.
# Dependencies are taken from the sources already fetched for the original
# generation pass so no network access is needed.

set(PS2X_RUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../runtime/ps2xRuntime")
set(DDS2_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/../../../analysis/decompilation/ps2recomp-build/_deps" CACHE PATH "Previously fetched dependency sources")

include(FetchContent)
foreach(dep elfio toml11 fmt libdwarf rabbitizer)
    string(TOUPPER ${dep} DEP)
    if(EXISTS "${DDS2_DEPS}/${dep}-src")
        set(FETCHCONTENT_SOURCE_DIR_${DEP} "${DDS2_DEPS}/${dep}-src" CACHE PATH "" FORCE)
    endif()
endforeach()

add_subdirectory(ps2xRecomp)
