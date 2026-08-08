@echo off
rem ---------------------------------------------------------------------------------------
rem build.bat - Windows wrapper that runs build.sh inside MSYS2 UCRT64.
rem
rem Same contract as build.sh: rebuilds the game module every time, rebuilds drifty.exe only
rem when it is not already running, always terminates immediately, and returns the
rem compiler's exit status. It never launches drifty.exe except for the explicit
rem --smoke-test mode, which is bounded and exits on its own.
rem
rem   build.bat                 build the module (and the exe if it is not already running)
rem   build.bat --release       single executable, static raylib, no game module
rem   build.bat --tests         headless test executable
rem   build.bat --hotreload-harness
rem   build.bat --smoke-test    build then run a bounded visual smoke test
rem   build.bat --clean         remove generated artifacts
rem ---------------------------------------------------------------------------------------
setlocal enableextensions

cd /d "%~dp0"

if not defined MSYS2_ROOT set "MSYS2_ROOT=C:\msys64"

if not exist "%MSYS2_ROOT%\ucrt64\bin\gcc.exe" (
    echo build.bat: MSYS2 UCRT64 not found at "%MSYS2_ROOT%". 1>&2
    echo Install with: winget install -e --id MSYS2.MSYS2 1>&2
    echo Then run:     powershell -ExecutionPolicy Bypass -File tools\setup\setup_windows.ps1 1>&2
    exit /b 127
)

if not exist "%MSYS2_ROOT%\usr\bin\bash.exe" (
    echo build.bat: bash.exe missing under "%MSYS2_ROOT%". 1>&2
    exit /b 127
)

rem Build provenance. git is on PATH here but usually not inside the MSYS2 minimal PATH, so
rem the values are resolved in cmd.exe and inherited by the child shell. build.sh falls back
rem to "unknown" when they are absent, so a git-less checkout still builds.
set "DRIFTY_GIT_COMMIT="
set "DRIFTY_GIT_BRANCH="
set "DRIFTY_GIT_DIRTY="
where git >nul 2>&1
if not errorlevel 1 (
    for /f "delims=" %%i in ('git rev-parse --short^=12 HEAD 2^>nul') do set "DRIFTY_GIT_COMMIT=%%i"
    for /f "delims=" %%i in ('git rev-parse --abbrev-ref HEAD 2^>nul') do set "DRIFTY_GIT_BRANCH=%%i"
    git diff --quiet HEAD >nul 2>&1
    if errorlevel 1 (set "DRIFTY_GIT_DIRTY=dirty") else (set "DRIFTY_GIT_DIRTY=clean")
)

rem Convert the repository path to an MSYS path via a temp file (avoids brittle for /f quoting).
set "MSYS_REPO_FILE=%TEMP%\drifty_msys_repo_%RANDOM%.txt"
"%MSYS2_ROOT%\usr\bin\cygpath.exe" -u "%CD%" > "%MSYS_REPO_FILE%"
if errorlevel 1 (
    echo build.bat: cygpath failed for "%CD%". 1>&2
    del /q "%MSYS_REPO_FILE%" 2>nul
    exit /b 127
)
set /p MSYS_REPO=<"%MSYS_REPO_FILE%"
del /q "%MSYS_REPO_FILE%" 2>nul
if not defined MSYS_REPO (
    echo build.bat: failed to convert "%CD%" to an MSYS path. 1>&2
    exit /b 127
)

rem %* is expanded by cmd.exe (build flags have no spaces). The repo path is single-quoted
rem for bash so spaces in the directory name remain safe.
"%MSYS2_ROOT%\usr\bin\env.exe" MSYSTEM=UCRT64 CHERE_INVOKING=1 MSYS2_PATH_TYPE=minimal ^
    "%MSYS2_ROOT%\usr\bin\bash.exe" --login -c "cd '%MSYS_REPO%' && ./build.sh %*"
set "STATUS=%ERRORLEVEL%"
exit /b %STATUS%
