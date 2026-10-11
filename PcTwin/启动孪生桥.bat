@echo off
rem Twin bridge launcher: build twin_harness.exe if missing, then run ws_bridge.py
rem NOTE: keep this file ASCII-only with CRLF line endings.
rem       cmd.exe parses .bat in the ANSI codepage (GBK on zh-CN systems);
rem       UTF-8 Chinese comments turn into mojibake and desync the parser.
setlocal
cd /d "%~dp0"

set "PY=%USERPROFILE%\.workbuddy\binaries\python\envs\default\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

rem Always incremental-build before run: robot_config.h constants are compiled
rem into twin_harness, so an edit to the header needs a rebuild to take effect.
rem Incremental build is a no-op (a few seconds) when nothing changed.
echo [twin] incremental build (skips fast if unchanged) ...
cmake -S . -B build -G Ninja
if errorlevel 1 cmake -S . -B build
if errorlevel 1 goto err
cmake --build build
if errorlevel 1 goto err

:run
"%PY%" ws_bridge.py %*
goto eof

:err
echo [twin] build failed - make sure cmake and ninja are in PATH
echo [twin] or build manually:  cmake -S . -B build -G Ninja  ^&^&  cmake --build build
exit /b 1

:eof
