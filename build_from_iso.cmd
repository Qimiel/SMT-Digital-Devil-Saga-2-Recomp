@echo off
rem Builds dds2.exe from a Digital Devil Saga 2 (USA) disc image:
rem   build_from_iso.cmd "C:\path\to\Shin Megami Tensei - Digital Devil Saga 2.iso" [other options]
rem See build_from_iso.ps1 for the options (-OutDir, -Jobs, -Toolchain, -Python, -PrepareOnly).
if "%~1"=="" (
    echo usage: %~nx0 ^<disc image .iso^> [options]
    exit /b 2
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_from_iso.ps1" -Iso %*
exit /b %ERRORLEVEL%
