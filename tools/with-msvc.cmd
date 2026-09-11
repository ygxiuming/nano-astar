@echo off
rem Activate the MSVC x64 toolchain, then run the given command.
rem Usage: tools\with-msvc.cmd <command> [args...]
call "E:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >/dev/null
set "PATH=E:\KimiData\daimon-bundle\runtime\uv;%PATH%"
%*
