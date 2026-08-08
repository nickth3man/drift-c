@echo off
rem ---------------------------------------------------------------------------------------
rem mk.bat - run a Makefile target inside MSYS2 UCRT64 from cmd.exe or PowerShell.
rem
rem   mk                    same as `make dev`
rem   mk test               fast unit and infrastructure scenarios
rem   mk report NAME=skidpad
rem   mk ci                 the local equivalent of the required CI checks
rem
rem Same contract as build.bat: it enters the UCRT64 environment, forwards the arguments,
rem and returns make's exit status. `mk run` is the one target that does not terminate on
rem its own, because it launches the game; that is the developer's to run, never an agent's.
rem ---------------------------------------------------------------------------------------
setlocal enableextensions

cd /d "%~dp0"

if not defined MSYS2_ROOT set "MSYS2_ROOT=C:\msys64"

if not exist "%MSYS2_ROOT%\ucrt64\bin\gcc.exe" (
    echo mk.bat: MSYS2 UCRT64 not found at "%MSYS2_ROOT%". 1>&2
    echo Install with: winget install -e --id MSYS2.MSYS2 1>&2
    echo Then run:     powershell -ExecutionPolicy Bypass -File tools\setup\setup_windows.ps1 1>&2
    exit /b 127
)

rem Build provenance, resolved here where git is on PATH (see build.bat).
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

set "MSYS_REPO_FILE=%TEMP%\drifty_msys_repo_%RANDOM%.txt"
"%MSYS2_ROOT%\usr\bin\cygpath.exe" -u "%CD%" > "%MSYS_REPO_FILE%"
if errorlevel 1 (
    echo mk.bat: cygpath failed for "%CD%". 1>&2
    del /q "%MSYS_REPO_FILE%" 2>nul
    exit /b 127
)
set /p MSYS_REPO=<"%MSYS_REPO_FILE%"
del /q "%MSYS_REPO_FILE%" 2>nul

rem MSYS2_PATH_TYPE=inherit so the Windows python, git, and ImageMagick the tools use stay
rem reachable; the Makefile still refuses any compiler outside /ucrt64/bin.
"%MSYS2_ROOT%\usr\bin\env.exe" MSYSTEM=UCRT64 CHERE_INVOKING=1 MSYS2_PATH_TYPE=inherit ^
    "%MSYS2_ROOT%\usr\bin\bash.exe" --login -c "cd '%MSYS_REPO%' && make %*"
set "STATUS=%ERRORLEVEL%"
exit /b %STATUS%
