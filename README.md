# SMT DDS2 recompilation

Turn the Ps2 iso into a native windows executable.
Started with 2 since I was more interested in its assets, once the project concludes DDS1 should be trivial.
Currently the game extends into wandering around occupied sector, the scripted forced battle crashes the game.


## Build from a disc image

On Windows 10/11 with an internet connection:

```bat
build_from_iso.cmd "Shin Megami Tensei - Digital Devil Saga 2.iso"
```

(or `.\build_from_iso.ps1 -Iso <image>` from PowerShell; `-OutDir`, `-Jobs`,
`-Toolchain <w64devkit dir>`, `-Python <python.exe>` and `-PrepareOnly` are
optional). The script

1. downloads w64devkit 2.9.1 (GCC 16.2, CMake, Ninja, Ccache) into `.tools/`,
   and the Python 3.12 embeddable package unless Python 3.8+ is installed,
   checking their SHA-256;
2. reads the image's ISO 9660 file system (`tools/iso9660.py`, no mounting)
   and extracts it to `dist/game/`, after checking that `SLUS_211.52` is the
   USA release;
3. builds the recompiler (`recompiler/`; its dependencies are pinned release
   archives) and translates the ELF to C++ in `generated/` using
   `research/recomp_units.csv` (function bounds) and
   `research/extra_entry_points.txt` (pointer-only code), then applies
   `tools/patch_guest.py` (`guest/`);
4. rebuilds the VU0/VU1 microcode images from the ELF's `.vutext` MPG uploads
   (`tools/extract_vu_microcode.py`, checked against the image hashes in
   `vu1_harvest/entries.txt` and `vu0_harvest/entries.txt`) and translates
   them (`tools/vu1_translate.py` -> `vu1gen/`);
5. compiles `dds2.exe` and copies it to `dist/`.

Then run `dist\dds2.exe`; it finds `game\` next to it.

## My Development environment

Extract the directory from the iso then throw this repo in there and run:
```powershell
& pc_port/recomp/build.ps1 -Jobs 3
& pc_port/recomp/run.ps1 -Seconds 60
```

`build.ps1` uses the toolchain under `analysis/references/decomp-tools` when
present, else the one in `.tools/`. Incremental runtime changes relink in
about a minute; editing a header in the guest precompiled header
(`ps2_runtime.h`, `ps2_memory.h`, ...) recompiles everything.

`dds2.exe [disc directory]`: with no argument it uses `DDS2_GAME_DIR`, or
searches the executable's directory and its parents (and a `game`
subdirectory of each) for `SLUS_211.52`. `DDS2_HEARTBEAT=<seconds>` prints
the guest thread table.

Keyboard: arrows/WASD = d-pad, X/Space = Cross, C/Esc = Circle, Z = Square,
V = Triangle, Q/E = L1/R1, Shift = L2/R2, Enter = Start, Tab = Select. An
XInput/SDL-compatible gamepad works at the same time as the keyboard.

## Layout

| Path | Purpose |
| --- | --- |
| `build_from_iso.ps1`, `build_from_iso.cmd` | self-contained build from a disc image (see above) |
| `tools/iso9660.py`, `tools/extract_vu_microcode.py` | disc image reader/extractor; VU microcode images rebuilt from the ELF |
| `research/recomp_units.csv`, `research/extra_entry_points.txt` | recompiler inputs: function bounds (from Ghidra) and pointer-only entry points (`tools/find_code_pointers.py`) |
| `CMakeLists.txt` | runtime, IOP HLE, guest code and executable targets |
| `src/main.cpp` | launcher: locates the disc files, boots the runtime |
| `src/dds2_overrides.cpp` | binds DDS2's stripped Sony SDK routines (SIF, CDVD, file I/O, pad, memory card, libsdr) to runtime HLE; disc-latency file reads |
| `src/dds2_sound.cpp` | HLE of the Atlus sound driver SDFSDMAN.IRX: IOP-to-EE RPC, movie ADPCM streaming to the host audio device, sound bank loading |
| `src/dds2_debug.cpp`, `src/dds2_profiler.cpp` | opt-in diagnostics (write watchpoint, heap checks, sampling profiler) |
| `tools/profile_report.py`, `tools/ipu_dump_to_png.py` | profiler report; decoded IPU picture dump viewer |
| `runtime/ps2xRuntime`, `runtime/ps2xIOP` | vendored PS2Recomp runtime (upstream `14b1e5c`, GPL-3.0) with DDS2 fixes |
| `runtime/ps2xIOP/src/modules/dds2_sdf.cpp` | HLE for Atlus SDF IOP modules (SDFIOP module loader, IOP print service) |
| `tools/patch_guest.py` | reproducible source fixes applied to the generated code |
| `guest/` | generated, patched guest sources (do not edit by hand) |
| `tools/vu1_translate.py`, `vu1gen/` | static recompiler for VU1 microcode: harvested 16 KB code images become C++ (`vu1gen/vu1_<hash>.cpp`, do not edit) |
| `vu1_harvest/`, `vu0_harvest/` | microcode entry points captured with `PS2X_VU1_HARVEST`/`PS2X_VU0_HARVEST` (`entries.txt`) and the images themselves (translator input, rebuilt from the ELF) |
| `src/vu1rec/` | runtime support and hook that runs translated VU1 programs in place of the interpreter, with a verification mode |
| `src/dds2_platform.cpp` | host setup kept apart from raylib (1 ms timer resolution) |


## Status

The game boots natively, there is some choppiness in menu UI and cutscenes, battles crash the game.

Work in progress, in rough order of importance:

- Music and sound effects: banks are registered but not synthesized (SMG
  HD/BD/SQ sequencing through MODHSYN/MODMIDI is not emulated), so only movie
  audio plays.
- Speed headroom: in the field about 20% of the EE thread is idle. Translated
  VU1 code (~4 ns per VU cycle, ~3,400 programs a frame) is the largest cost;
  battles and later areas have not been measured.
- Memory card saves use the runtime's memory card HLE, create proper save files.
- Fix battle crashes.
- Add graphical improvements.
- Add save file detection for DDS1.
- Only the opening hours have been played through.

Diagnostics: `run.ps1 -Snapshot <s>` saves window snapshots. Environment
switches: `PS2X_GUEST_PRINTF`, `PS2X_TRACE_DMA`, `PS2X_TRACE_IPU`,
`PS2X_TRACE_GS=<emulated s>` (sprites, textured triangles, transfers from
then on), `PS2X_TRACE_GIF=<emulated s>` (GIF tags until then), `PS2X_GS_STATS`
(frame times, texture decodes and their causes), `PS2X_TRACE_VSYNC` (VBlank
spacing), `PS2X_TRACE_PCS=addr,...`, `PS2X_IPU_DUMP=<picture>`,
`PS2X_DUMP_TEXTURE=<tbp0>`, `PS2X_WATCH_VRAM=<byte address>`,
`PS2X_WATCH_VU1_QWORD=<qword>`, `PS2X_GS_NO_TEXCACHE`, `PS2X_GS_THREAD=0`
(synchronous GS), `PS2X_GS_THREADS=<n>` (raster bands), `PS2X_PACING=realtime`,
`PS2X_PAD_SCRIPT=start@20,cross@27+0.12` (scripted input in emulated seconds),
`PS2X_VU1_HARVEST=<dir>` (save microcode images for the translator),
`PS2X_VU1_NATIVE=0` (interpret VU1), `PS2X_VU1_VERIFY` (run every VU1 program
both ways and report differences), `PS2X_VU_STATS`, `PS2X_VU1_DUMP_AT=<s>`
(with `PS2X_VU1_DUMP_PC`/`PS2X_VU1_DUMP_ITOP`: VU1 memory at a program start),
`DDS2_WATCH=addr:len,...`, `DDS2_WALK=<list head>`, `DDS2_WATCH_WRITE=<addr>`,
`DDS2_CHECK_HEAP`, `DDS2_CHECK_SMALL`, `DDS2_LOG_SOUND`,
`DDS2_PROFILE=<start seconds>` (then `tools/profile_report.py build/dds2.exe
build/profile.txt`, which also attributes CRT/OS routines to their callers),
`PS2X_TRACE_GS_PERF` (ms per frame on the EE thread for VIF1, VU1, GS packets
and VU0), `PS2X_TRACE_READBACK` (GS-to-EE readbacks and their cost),
`DDS2_TRACE_FN=addr[:count],...` (with `DDS2_TRACE_FN_AT=<s>`,
`DDS2_TRACE_FN_OFFSET`, `DDS2_TRACE_FN_QWORDS`: arguments and memory at `$a0`).

GS diagnostics take effect after `PS2X_GS_DEBUG_FROM=<emulated s>`:
`PS2X_TRACE_GS_SEQ=<lines>` (FRAME/ZBUF/TEST/ALPHA writes with their GIF path
and the vertices between them), `PS2X_GS_DUMP_TEST=<hex>,...` (dumps frame,
alpha and Z buffer as `gsdump_*.ppm/pgm` at those TEST values and at FRAME
writes, up to `PS2X_GS_DUMP_COUNT`), `PS2X_TRACE_TRX_BELOW=<hex block>`
(transfers into low GS memory). With `PS2X_TRACE_GS`, `PS2X_TRACE_GS_REGION=x0,y0,x1,y1`
and `PS2X_TRACE_GS_CTXT=<0|1>` narrow the triangle trace, which also prints the
stored depth at each triangle's centroid; `PS2X_TRACE_TEST_VALUE=<hex>[,<hex>]`
logs the register writes around a TEST value.

To translate new VU1 microcode: run with `PS2X_VU1_HARVEST=vu1_harvest`
through the scenes that use it, then
`python tools/vu1_translate.py vu1gen vu1_harvest/*.bin` and rebuild. Images
without a translation fall back to the interpreter (logged once). VU0
microcode works the same way with `PS2X_VU0_HARVEST=vu0_harvest` and
`python tools/vu1_translate.py --vu0 vu1gen vu0_harvest/*.bin`.



## Credits
Thanks to:
[TGE](https://github.com/tge-was-taken): for providing tools to study the DDS3 engine.
[PS2Recomp](https://github.com/ran-j/PS2Recomp): Recomp tool for ps2.
[dds-decomp](https://github.com/Megami-Decomps/dds-decomp): for providing decompiled functions and tables for DDS1&2.
