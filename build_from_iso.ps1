<#
.SYNOPSIS
Builds the native Windows port of Shin Megami Tensei: Digital Devil Saga 2
(USA, SLUS-21152) from a disc image, with nothing installed beforehand.

.DESCRIPTION
Everything is derived from the disc image you supply; no game code or data is
part of this tree. The script:
  1. downloads the pinned toolchain into .tools (w64devkit: GCC, CMake,
     Ninja, Ccache; Python if none is installed), checking hashes;
  2. extracts the ISO 9660 file system into <OutDir>\game and checks that
     SLUS_211.52 is the US release this port is made for;
  3. builds the static recompiler (recompiler\) and translates the game's
     MIPS code into C++ (generated\, then tools\patch_guest.py -> guest\);
  4. rebuilds the VU0/VU1 microcode images from the ELF and translates them
     to C++ (vu1gen\);
  5. compiles the runtime and the translated code into dds2.exe and copies
     it to <OutDir>.
Every step skips work that is already done, so rerunning it is cheap. The
first full build compiles about 10,000 translated units: expect an hour or
more, and about 2 GB of free memory per compile job.

.PARAMETER Iso
The disc image (.iso) of the USA release.

.PARAMETER OutDir
Where dds2.exe and the extracted game files (game\) go. Default: .\dist

.PARAMETER Jobs
Parallel compile jobs. Default: picked from the free memory and CPU count.

.PARAMETER Toolchain
An existing w64devkit directory to use instead of downloading one.

.PARAMETER Python
A python.exe (3.8 or newer) to use instead of the one found on PATH.

.PARAMETER PrepareOnly
Stop once the sources are generated (game files, guest\, vu1gen\), before
compiling dds2.exe.

.EXAMPLE
.\build_from_iso.ps1 -Iso "D:\isos\Shin Megami Tensei - Digital Devil Saga 2.iso"
#>
param(
    [Parameter(Mandatory = $true)][string]$Iso,
    [string]$OutDir = '',
    [int]$Jobs = 0,
    [string]$Toolchain = '',
    [string]$Python = '',
    [switch]$PrepareOnly
)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue' # Invoke-WebRequest is slow when drawing progress
$Root = $PSScriptRoot
if (-not $OutDir) { $OutDir = Join-Path $Root 'dist' }
$OutDir = [IO.Path]::GetFullPath($OutDir)
$ToolsDir = Join-Path $Root '.tools'
$GameDir = Join-Path $OutDir 'game'

# Pinned downloads.
$W64DevKit = @{
    Url    = 'https://github.com/skeeto/w64devkit/releases/download/v2.9.1/w64devkit-x64-2.9.1.7z.exe'
    Sha256 = '9208c19755cd4964b7915b9afcf02c66d493a4c870c4b3e83f6c538d9c1237a5'
}
$PythonEmbed = @{
    Url    = 'https://www.python.org/ftp/python/3.12.10/python-3.12.10-embed-amd64.zip'
    Sha256 = '4acbed6dd1c744b0376e3b1cf57ce906f9dc9e95e68824584c8099a63025a3c3'
}
# SLUS_211.52 of Shin Megami Tensei: Digital Devil Saga 2 (USA).
$ElfSha256 = '0404710d711c148f0ab94910f9a783a5c06782fff9dcc85e5f862ca70c87ac55'

function Step([string]$text) { Write-Host ''; Write-Host "==> $text" -ForegroundColor Cyan }
function Info([string]$text) { Write-Host "    $text" }
function Fail([string]$text) {
    Write-Host ''
    Write-Host "error: $text" -ForegroundColor Red
    exit 1
}

# Runs a native program; native tools report progress on stderr, so success
# is judged by the exit code alone.
function Invoke-Native([string]$exe, [string[]]$arguments) {
    & $exe @arguments
    if ($LASTEXITCODE -ne 0) { Fail "$([IO.Path]::GetFileName($exe)) failed (exit code $LASTEXITCODE)" }
}

function Get-Sha256([string]$path) { (Get-FileHash -Algorithm SHA256 -LiteralPath $path).Hash.ToLowerInvariant() }

function Get-Download([hashtable]$item) {
    $downloads = Join-Path $ToolsDir 'downloads'
    New-Item -ItemType Directory -Force $downloads | Out-Null
    $path = Join-Path $downloads ([IO.Path]::GetFileName($item.Url))
    if ((Test-Path -LiteralPath $path) -and (Get-Sha256 $path) -eq $item.Sha256) { return $path }
    Info "downloading $($item.Url)"
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    Invoke-WebRequest -UseBasicParsing -Uri $item.Url -OutFile "$path.part"
    if ((Get-Sha256 "$path.part") -ne $item.Sha256) {
        Remove-Item -LiteralPath "$path.part"
        Fail "checksum mismatch for $($item.Url)"
    }
    Move-Item -Force -LiteralPath "$path.part" $path
    return $path
}

function Test-Python([string]$exe, [string[]]$prefix) {
    try {
        $ok = & $exe @prefix -c 'import sys; print(sys.version_info >= (3, 8))' 2>$null
        return ($LASTEXITCODE -eq 0 -and "$ok".Trim() -eq 'True')
    } catch { return $false }
}

# ---------------------------------------------------------------- 1. tools
Step 'Toolchain'
if ($Toolchain) {
    $DevKit = [IO.Path]::GetFullPath($Toolchain)
} else {
    $DevKit = Join-Path $ToolsDir 'w64devkit'
    if (-not (Test-Path (Join-Path $DevKit 'bin\gcc.exe'))) {
        $sfx = Get-Download $W64DevKit
        Info 'unpacking w64devkit'
        # 7-Zip self-extractor: -o<dir> unpacks w64devkit\ there.
        $p = Start-Process -FilePath $sfx -ArgumentList '-y', "-o$ToolsDir" -Wait -PassThru -WindowStyle Hidden
        if ($p.ExitCode -ne 0) { Fail "w64devkit self-extractor failed (exit code $($p.ExitCode))" }
    }
}
$Bin = Join-Path $DevKit 'bin'
foreach ($tool in 'gcc.exe', 'g++.exe', 'cmake.exe', 'ninja.exe') {
    if (-not (Test-Path (Join-Path $Bin $tool))) { Fail "$tool not found in $Bin" }
}
$env:PATH = "$Bin;$env:PATH"
$Cmake = Join-Path $Bin 'cmake.exe'
$Ccache = Join-Path $Bin 'ccache.exe'
Info "w64devkit: $DevKit"

$PythonArgs = @()
if ($Python) {
    if (-not (Test-Python $Python @())) { Fail "$Python is not Python 3.8 or newer" }
} elseif ((Get-Command py -ErrorAction SilentlyContinue) -and (Test-Python 'py' @('-3'))) {
    $Python = 'py'; $PythonArgs = @('-3')
} elseif ((Get-Command python -ErrorAction SilentlyContinue) -and (Test-Python 'python' @())) {
    $Python = (Get-Command python).Source
} else {
    $Python = Join-Path $ToolsDir 'python\python.exe'
    if (-not (Test-Path $Python)) {
        $zip = Get-Download $PythonEmbed
        Info 'unpacking Python'
        Expand-Archive -Force -LiteralPath $zip -DestinationPath (Join-Path $ToolsDir 'python')
    }
}
Info "python: $Python $PythonArgs"
function Invoke-Python([string[]]$arguments) { Invoke-Native $Python ($PythonArgs + $arguments) }

if ($Jobs -le 0) {
    $freeGb = (Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory / 1MB
    $cpus = [Environment]::ProcessorCount
    $Jobs = [int][Math]::Max(1, [Math]::Min($cpus, [Math]::Floor($freeGb / 2.0)))
    Info ("{0} compile job(s) ({1:N1} GB free, {2} logical CPUs)" -f $Jobs, $freeGb, $cpus)
}

# ---------------------------------------------------------------- 2. game files
Step "Extracting $Iso"
if (-not (Test-Path -LiteralPath $Iso)) { Fail "no such file: $Iso" }
New-Item -ItemType Directory -Force $GameDir | Out-Null
$isoTool = Join-Path $Root 'tools\iso9660.py'
# The executable first, to reject another game or release before copying 4 GB.
Invoke-Python @($isoTool, 'extract', $Iso, $GameDir, 'SLUS_211.52')
$Elf = Join-Path $GameDir 'SLUS_211.52'
if (-not (Test-Path $Elf)) { Fail "the image has no SLUS_211.52: this port needs Digital Devil Saga 2 (USA, SLUS-21152)" }
if ((Get-Sha256 $Elf) -ne $ElfSha256) {
    Remove-Item -LiteralPath $Elf
    Fail "SLUS_211.52 differs from the USA release this port is made for (SHA-256 $ElfSha256)"
}
Info 'SLUS_211.52 verified (USA release)'
Invoke-Python @($isoTool, 'extract', $Iso, $GameDir)

# ---------------------------------------------------------------- 3. EE code
Step 'Building the recompiler'
$RecompBuild = Join-Path $Root 'build\recompiler'
if (-not (Test-Path (Join-Path $RecompBuild 'build.ninja'))) {
    Invoke-Native $Cmake @('-S', (Join-Path $Root 'recompiler'), '-B', $RecompBuild, '-G', 'Ninja',
        "-DCMAKE_MAKE_PROGRAM=$(Join-Path $Bin 'ninja.exe')",
        "-DCMAKE_C_COMPILER=$(Join-Path $Bin 'gcc.exe')", "-DCMAKE_CXX_COMPILER=$(Join-Path $Bin 'g++.exe')",
        '-DCMAKE_BUILD_TYPE=Release')
}
Invoke-Native $Cmake @('--build', $RecompBuild, '--target', 'ps2_recomp', '--parallel', "$Jobs")
$Recompiler = Join-Path $RecompBuild 'ps2xRecomp\ps2_recomp.exe'

Step 'Translating the game code (MIPS -> C++)'
$Generated = Join-Path $Root 'generated'
$Units = Join-Path $Root 'research\recomp_units.csv'
$EntryPoints = Join-Path $Root 'research\extra_entry_points.txt'
# Regenerate when the ELF, the recompiler or its inputs change.
$stamp = @($ElfSha256, (Get-Sha256 $Recompiler), (Get-Sha256 $Units), (Get-Sha256 $EntryPoints)) -join "`n"
$stampFile = Join-Path $Generated '.inputs'
if ((Test-Path $stampFile) -and ((Get-Content -Raw $stampFile).Trim() -eq $stamp)) {
    Info 'generated\ is up to date'
} else {
    if (Test-Path $Generated) { Remove-Item -Recurse -Force $Generated }
    New-Item -ItemType Directory -Force $Generated | Out-Null
    $config = Join-Path $RecompBuild 'dds2_recomp.toml'
    $toml = @(
        '[general]',
        "input = '$($Elf -replace '\\', '/')'",
        "ghidra_output = '$($Units -replace '\\', '/')'",
        "output = '$($Generated -replace '\\', '/')'",
        "extra_entry_points_file = '$($EntryPoints -replace '\\', '/')'",
        'single_file_output = false',
        'low_memory_mode = true',
        'output_worker_threads = 4',
        'patch_syscalls = false',
        'patch_cop0 = false',
        'patch_cache = false',
        'stubs = []',
        'skip = []',
        '',
        '[patches]',
        'instructions = []')
    Set-Content -Encoding ascii -Path $config -Value $toml
    Invoke-Native $Recompiler @($config)
    Set-Content -Encoding ascii -Path $stampFile -Value $stamp
}
Invoke-Python @((Join-Path $Root 'tools\patch_guest.py'), '--src', $Generated, '--dst', (Join-Path $Root 'guest'))

# ---------------------------------------------------------------- 4. VU code
Step 'Translating the VU microcode'
$Vu1Images = Join-Path $Root 'vu1_harvest'
$Vu0Images = Join-Path $Root 'vu0_harvest'
$VuGen = Join-Path $Root 'vu1gen'
Invoke-Python @((Join-Path $Root 'tools\extract_vu_microcode.py'), $Elf, $Vu1Images, $Vu0Images)
New-Item -ItemType Directory -Force $VuGen | Out-Null
$translator = Join-Path $Root 'tools\vu1_translate.py'
Invoke-Python (@($translator, $VuGen) + @(Get-ChildItem (Join-Path $Vu1Images '*.bin') | ForEach-Object FullName))
Invoke-Python (@($translator, '--vu0', $VuGen) + @(Get-ChildItem (Join-Path $Vu0Images '*.bin') | ForEach-Object FullName))

if ($PrepareOnly) {
    Step 'Sources ready (-PrepareOnly): rerun without it to compile dds2.exe'
    return
}

# ---------------------------------------------------------------- 5. dds2.exe
Step "Compiling dds2.exe ($Jobs job(s); the first build takes an hour or more)"
$Build = Join-Path $Root 'build'
$env:CCACHE_DIR = Join-Path $Root '.ccache'
if (-not (Test-Path (Join-Path $Build 'build.ninja'))) {
    $configure = @('-S', $Root, '-B', $Build, '-G', 'Ninja',
        "-DCMAKE_MAKE_PROGRAM=$(Join-Path $Bin 'ninja.exe')",
        "-DCMAKE_C_COMPILER=$(Join-Path $Bin 'gcc.exe')", "-DCMAKE_CXX_COMPILER=$(Join-Path $Bin 'g++.exe')",
        '-DCMAKE_BUILD_TYPE=RelWithDebInfo')
    if (Test-Path $Ccache) {
        $configure += @("-DCMAKE_C_COMPILER_LAUNCHER=$Ccache", "-DCMAKE_CXX_COMPILER_LAUNCHER=$Ccache")
    }
    Invoke-Native $Cmake $configure
}
Invoke-Native $Cmake @('--build', $Build, '--target', 'dds2', '--parallel', "$Jobs")

Copy-Item -Force (Join-Path $Build 'dds2.exe') (Join-Path $OutDir 'dds2.exe')
Step 'Done'
Info "Play: $(Join-Path $OutDir 'dds2.exe')"
Info 'Keyboard: arrows/WASD = d-pad, X/Space = Cross, C/Esc = Circle, Z = Square, V = Triangle,'
Info '          Q/E = L1/R1, Shift = L2/R2, Enter = Start, Tab = Select (gamepads work too).'
