param(
    [int]$Jobs = 4,
    [string]$BuildDir = '',
    [string]$Config = 'RelWithDebInfo',
    [string]$Target = 'dds2'
)
# Native tools write progress to stderr; failures are detected by exit code.
$ErrorActionPreference = 'Continue'
$root = $PSScriptRoot
$workspace = Split-Path -Parent (Split-Path -Parent $root)
$tools = Join-Path $workspace 'analysis/references/decomp-tools'
$bin = Join-Path $tools 'w64devkit/bin'
$cmake = Join-Path $tools 'cmake-4.4.3-windows-x86_64/bin/cmake.exe'
# Without the development tree's tools, use the toolchain build_from_iso.ps1
# downloaded (w64devkit includes CMake, Ninja and Ccache).
if (-not (Test-Path (Join-Path $bin 'gcc.exe'))) {
    $bin = Join-Path $root '.tools/w64devkit/bin'
    $cmake = Join-Path $bin 'cmake.exe'
}
if (-not $BuildDir) { $BuildDir = Join-Path $root 'build' }
$previousPath = $env:PATH
try {
    $env:PATH = "$bin;$previousPath"
    $env:CCACHE_DIR = Join-Path $root '.ccache'
    & python (Join-Path $root 'tools/patch_guest.py')
    if ($LASTEXITCODE -ne 0) { throw 'Guest source patching failed' }
    if (-not (Test-Path (Join-Path $BuildDir 'build.ninja'))) {
        & $cmake -S $root -B $BuildDir -G Ninja `
            "-DCMAKE_MAKE_PROGRAM=$bin/ninja.exe" `
            "-DCMAKE_C_COMPILER=$bin/gcc.exe" `
            "-DCMAKE_CXX_COMPILER=$bin/g++.exe" `
            "-DCMAKE_C_COMPILER_LAUNCHER=$bin/ccache.exe" `
            "-DCMAKE_CXX_COMPILER_LAUNCHER=$bin/ccache.exe" `
            "-DCMAKE_BUILD_TYPE=$Config"
        if ($LASTEXITCODE -ne 0) { throw 'Configure failed' }
    }
    & $cmake --build $BuildDir --target $Target --parallel $Jobs
    if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
} finally { $env:PATH = $previousPath }
