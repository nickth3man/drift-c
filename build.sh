#!/bin/sh
#
# build.sh — canonical Drifty build, executed inside MSYS2 UCRT64.
#
# Rebuilds the game module every time, and rebuilds build/dev/drifty.exe only when it is not
# already running. It always terminates in well under a second and returns the compiler's exit
# status. It never launches or supervises drifty.exe: the developer starts that once and
# leaves it open. Use --smoke-test for a bounded visual run that exits on its own.
#
# A failed compile leaves the previous, working module untouched — the link output goes to a
# temporary name and is only renamed into place on success — so a compile error can never
# close the running game.
#
# Every artifact goes under build/, one directory per configuration:
#   build/dev/      drifty.exe, game.dll, the hot-reload harness, and their runtime DLLs
#   build/tests/    drifty_tests.exe (headless)
#   build/release/  drifty_release.exe and glfw3.dll
#
#   ./build.sh              build the module (and the exe if it is not running)
#   ./build.sh --release    single executable; raylib linked statically (no libraylib.dll)
#   ./build.sh --tests      headless test executable
#   ./build.sh --hotreload-harness  windowless hot-reload validation executable
#   ./build.sh --smoke-test build (if needed) and run build/dev/drifty.exe --smoke-test
#   ./build.sh --clean      remove generated artifacts
#
# From cmd.exe / PowerShell prefer build.bat, which enters the UCRT64 environment for you.
#
set -u

cd "$(dirname "$0")" || exit 1
ROOT="$(pwd)"

# ---------------------------------------------------------------------------- environment --

if [ "${MSYSTEM:-}" != "UCRT64" ] || [ "${MINGW_PREFIX:-}" != "/ucrt64" ]; then
    echo "build.sh: must run under MSYS2 UCRT64 (MSYSTEM=UCRT64, MINGW_PREFIX=/ucrt64)." >&2
    echo "  From Windows:  build.bat" >&2
    echo "  Or open the   MSYS2 UCRT64 shell and re-run ./build.sh" >&2
    exit 127
fi

CC="${CC:-gcc}"
if ! command -v "$CC" >/dev/null 2>&1; then
    echo "build.sh: '$CC' not found on PATH. Run tools/setup/setup_windows.ps1." >&2
    exit 127
fi

CC_PATH="$(command -v "$CC")"
case "$CC_PATH" in
    */ucrt64/bin/gcc|*/ucrt64/bin/gcc.exe) ;;
    *)
        echo "build.sh: refusing non-UCRT64 compiler '$CC_PATH'." >&2
        echo "  Use build.bat or the MSYS2 UCRT64 shell. Chocolatey/MinGW PATH entries are unsupported." >&2
        exit 127
        ;;
esac

if ! command -v pkg-config >/dev/null 2>&1; then
    echo "build.sh: pkg-config not found. Run tools/setup/setup_windows.ps1." >&2
    exit 127
fi

PKG_PATH="$(command -v pkg-config)"
case "$PKG_PATH" in
    */ucrt64/bin/pkg-config|*/ucrt64/bin/pkg-config.exe|*/ucrt64/bin/pkgconf|*/ucrt64/bin/pkgconf.exe) ;;
    *)
        echo "build.sh: refusing non-UCRT64 pkg-config '$PKG_PATH'." >&2
        exit 127
        ;;
esac

if ! pkg-config --exists raylib; then
    echo "build.sh: pkg-config cannot find raylib. Run tools/setup/setup_windows.ps1." >&2
    exit 127
fi

# ------------------------------------------------------------------------------- raylib --
#
# Development / hot reload: shared libraylib.dll (MSYS2 name). Both drifty.exe and
# build/game.dll must import the same DLL so raylib global state is not duplicated.
#
# Release: static libraylib.a. The MSYS2 static archive still references shared GLFW
# (__imp_glfw*), so glfw3.dll remains a runtime dependency of the release executable.
# That is a package limitation, not a project fallback to vendored raylib.

RAYLIB_CFLAGS="$(pkg-config --cflags raylib)"

# Prefer the import library explicitly so -lraylib cannot quietly pick libraylib.a.
RAYLIB_SHARED_LIBS="${MINGW_PREFIX}/lib/libraylib.dll.a -lopengl32 -lgdi32 -lwinmm"
RAYLIB_STATIC_LIBS="-l:libraylib.a -lglfw3 -lopengl32 -lgdi32 -lwinmm"

RAYLIB_SHARED_DLL="${MINGW_PREFIX}/bin/libraylib.dll"
GLFW_SHARED_DLL="${MINGW_PREFIX}/bin/glfw3.dll"

# -------------------------------------------------------------------- build provenance --
#
# Recorded in the binary so a failure bundle can name the commit it came from. build.bat
# passes these in from cmd.exe, where git is on PATH; when it is not set we ask git here and
# fall back to "unknown". Values must not contain spaces: they are expanded unquoted into the
# compiler command line below.

if [ -z "${DRIFTY_GIT_COMMIT:-}" ] && command -v git >/dev/null 2>&1; then
    DRIFTY_GIT_COMMIT="$(git rev-parse --short=12 HEAD 2>/dev/null || echo unknown)"
    DRIFTY_GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
    if git diff --quiet HEAD 2>/dev/null; then
        DRIFTY_GIT_DIRTY="clean"
    else
        DRIFTY_GIT_DIRTY="dirty"
    fi
fi
BUILD_COMMIT="${DRIFTY_GIT_COMMIT:-unknown}"
BUILD_BRANCH="${DRIFTY_GIT_BRANCH:-unknown}"
BUILD_DIRTY="${DRIFTY_GIT_DIRTY:-unknown}"

build_info_defines() {
    # $1 = build mode, $2 = comma-separated optimisation flags (no spaces, see above)
    echo "-DDRIFTY_BUILD_COMMIT=\"$BUILD_COMMIT\" -DDRIFTY_BUILD_BRANCH=\"$BUILD_BRANCH\"" \
         "-DDRIFTY_BUILD_DIRTY=\"$BUILD_DIRTY\" -DDRIFTY_BUILD_MODE=\"$1\"" \
         "-DDRIFTY_BUILD_FLAGS=\"$2\""
}

# -------------------------------------------------------------------------------- flags --

CSTD="-std=c11"
INCLUDES="-Isrc -Itests -Ithird_party -Ithird_party/raygui"
WARNINGS="-Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith"
DEBUG_FLAGS="-O0 -g"
RELEASE_FLAGS="-O2 -DNDEBUG"

# Development tooling (Physics Lab, replay inspector, failure bundles). The registry, the
# scenario table, the scope history, and the bundle writer are raylib-free and are compiled
# into every configuration; only the raygui interface is conditional.
DEV_TOOL_DEFINES="-DDRIFTY_DEV_TOOLS"

# Extra defines for opt-in builds: `DRIFTY_EXTRA_DEFINES=-DDRIFTY_PROFILE ./build.sh`.
# `make profile` uses it. Values must not contain spaces (see the note above).
EXTRA_DEFINES="${DRIFTY_EXTRA_DEFINES:-}"

# ------------------------------------------------------------------------------ sources --
#
# The source manifest lives in the Makefile and nowhere else. It used to be duplicated here,
# which meant two lists that had to be edited together and a comment asking the next person to
# remember. Adopt it instead: `print-source-groups` prints shell assignments.

# GNU Make specifically: `print-source-groups` is built on $(foreach) and the export relies on
# --no-print-directory, neither of which a BSD or NMAKE-style make provides.
if ! command -v make >/dev/null 2>&1; then
    echo "build.sh: make not found. Run tools/setup/setup_windows.ps1." >&2
    exit 1
fi
if ! make --version 2>/dev/null | grep -qi "GNU Make"; then
    echo "build.sh: '$(command -v make)' is not GNU Make. Run tools/setup/setup_windows.ps1." >&2
    exit 1
fi

# MAKEFLAGS/MAKELEVEL are cleared because this runs as a nested make when the caller was
# `make dev` or `make tests`: inheriting a jobserver flag here buys nothing and warns.
if ! SOURCE_GROUPS="$(MAKEFLAGS= MAKELEVEL= make --no-print-directory -s print-source-groups)"; then
    echo "build.sh: could not read the source manifest from the Makefile." >&2
    exit 1
fi
eval "$SOURCE_GROUPS"
unset SOURCE_GROUPS

# A typo in the manifest must not link a smaller binary in silence. Every exported group is
# checked, not just the ones this script links, so the manifest is validated as a whole.
for group in SHARED_SRCS DEV_SRCS DEV_UI_SRCS GAME_SRCS PLATFORM_SRCS HOTRELOAD_SRC \
             TEST_RUNNER_SRCS TEST_SRCS HOTRELOAD_HARNESS_SRCS FUZZ_SUPPORT_SRCS; do
    eval "value=\${$group:-}"
    if [ -z "$value" ]; then
        echo "build.sh: empty source group $group" >&2
        exit 2
    fi
done
unset group value

# ------------------------------------------------------------------------------ outputs --
#
# Generated binaries live under build/, one directory per configuration, so the repository
# root stays readable and `rm -rf build` is a complete clean. The four entry points
# (build.bat, build.sh, mk.bat, Makefile) stay at the root, where a developer expects them.
#
# Each configuration's runtime DLLs sit BESIDE its executable: Windows resolves an import from
# the directory of the loading module first, so this is what makes build/dev/drifty.exe find
# libraylib.dll without a PATH entry.

BUILD_DEV="build/dev"
BUILD_TESTS="build/tests"
BUILD_RELEASE="build/release"

MODULE="$BUILD_DEV/game.dll"
MODULE_TMP="$BUILD_DEV/game_tmp.tmp"
EXE="$BUILD_DEV/drifty.exe"
EXE_RELEASE="$BUILD_RELEASE/drifty_release.exe"
EXE_TESTS="$BUILD_TESTS/drifty_tests.exe"
EXE_HOTRELOAD="$BUILD_DEV/drifty_hotreload_harness.exe"

# Only the build output roots. Run-evidence directories under artifacts/ are created by the
# writer that needs them (telemetry_ensure_dir / ensure_parent_directory), so a clean checkout
# never depends on this line having listed them.
mkdir -p "$BUILD_DEV" "$BUILD_TESTS" "$BUILD_RELEASE"

copy_dev_runtime() {
    # Development binaries import libraylib.dll, which itself imports glfw3.dll.
    if [ -f "$RAYLIB_SHARED_DLL" ]; then
        cp -f "$RAYLIB_SHARED_DLL" "$ROOT/$BUILD_DEV/libraylib.dll"
    else
        echo "build.sh: missing $RAYLIB_SHARED_DLL" >&2
        return 1
    fi
    if [ -f "$GLFW_SHARED_DLL" ]; then
        cp -f "$GLFW_SHARED_DLL" "$ROOT/$BUILD_DEV/glfw3.dll"
    else
        echo "build.sh: missing $GLFW_SHARED_DLL" >&2
        return 1
    fi
}

copy_release_runtime() {
    # Release embeds raylib statically but still needs glfw3.dll with the MSYS2 package.
    if [ -f "$GLFW_SHARED_DLL" ]; then
        cp -f "$GLFW_SHARED_DLL" "$ROOT/$BUILD_RELEASE/glfw3.dll"
    else
        echo "build.sh: missing $GLFW_SHARED_DLL" >&2
        return 1
    fi
}

# ------------------------------------------------------------------------------ actions --

MODE="dev"
if [ $# -gt 0 ]; then
    case "$1" in
        --release) MODE="release" ;;
        --tests)   MODE="tests" ;;
        --hotreload-harness) MODE="hotreload-harness" ;;
        --smoke-test) MODE="smoke-test" ;;
        --clean)   MODE="clean" ;;
        --help|-h)
            sed -n '2,27p' "$0"
            exit 0
            ;;
        *)
            echo "build.sh: unrecognised argument '$1' (try --help)" >&2
            exit 2
            ;;
    esac
fi

if [ "$MODE" = "clean" ]; then
    rm -rf build coverage dist corpus
    rm -rf artifacts/telemetry artifacts/replays artifacts/fuzz
    rm -f artifacts/screenshots/phase1_smoke.png artifacts/screenshots/phase2_smoke.png
    rm -f artifacts/screenshots/phase3_smoke.png
    # Legacy root-level outputs from before the build/ and artifacts/ consolidation, so a tree
    # built by an older checkout does not keep a stale runnable or generated copy beside the
    # new one.
    rm -rf telemetry replays
    rm -f drifty.exe drifty_release.exe drifty_tests.exe drifty_hotreload_harness.exe
    rm -f drifty_tests_asan.exe drifty_tests_cov.exe
    rm -f libraylib.dll raylib.dll glfw3.dll
    rm -f ./*.o src/*.o tests/*.o ./*.d ./*.pdb ./*.ilk ./*.exp ./*.map
    rm -f ./*.gcda ./*.gcno ./*.gcov
    rm -f mk_verify*.log
    echo "cleaned."
    exit 0
fi

if [ "$MODE" = "tests" ]; then
    # shellcheck disable=SC2086,SC2046
    $CC $CSTD $INCLUDES $WARNINGS $RELEASE_FLAGS -DDRIFTY_HEADLESS \
        $(build_info_defines tests "-O2,-DNDEBUG,-DDRIFTY_HEADLESS") \
        $TEST_SRCS -o "$EXE_TESTS" $RAYLIB_CFLAGS -lm
    status=$?
    [ $status -eq 0 ] && echo "Built $EXE_TESTS."
    exit $status
fi

if [ "$MODE" = "hotreload-harness" ]; then
    copy_dev_runtime || exit 1

    # The harness loads $MODULE; build it the same safe way as the dev path.
    # shellcheck disable=SC2086,SC2046
    $CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -shared \
        -DDRIFTY_HOT_RELOAD -DDRIFTY_GAME_MODULE $DEV_TOOL_DEFINES \
        $(build_info_defines module "-O0,-g") $EXTRA_DEFINES \
        $GAME_SRCS $DEV_UI_SRCS $SHARED_SRCS -o "$MODULE_TMP" \
        $RAYLIB_CFLAGS $RAYLIB_SHARED_LIBS
    status=$?
    if [ $status -ne 0 ]; then
        rm -f "$MODULE_TMP"
        echo "build.sh: game module failed to compile; harness not built." >&2
        exit $status
    fi
    mv -f "$MODULE_TMP" "$MODULE" || exit 1

    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -DDRIFTY_HOT_RELOAD \
        $HOTRELOAD_HARNESS_SRCS -o "$EXE_HOTRELOAD" $RAYLIB_CFLAGS $RAYLIB_SHARED_LIBS
    status=$?
    [ $status -eq 0 ] && echo "Built $MODULE and $EXE_HOTRELOAD."
    exit $status
fi

if [ "$MODE" = "release" ]; then
    copy_release_runtime || exit 1
    # shellcheck disable=SC2086,SC2046
    $CC $CSTD $INCLUDES $WARNINGS $RELEASE_FLAGS \
        $(build_info_defines release "-O2,-DNDEBUG") \
        $PLATFORM_SRCS $GAME_SRCS $SHARED_SRCS -o "$EXE_RELEASE" \
        $RAYLIB_CFLAGS $RAYLIB_STATIC_LIBS
    status=$?
    [ $status -eq 0 ] && echo "Built $EXE_RELEASE (no hot reload, no game module; raylib static)."
    exit $status
fi

build_dev() {
    copy_dev_runtime || return 1

    # Always rebuild the game module. Link to a temporary name first: the linker leaves a
    # zero-length file in place while it works, and a running game polling for changes would
    # load that. The rename is atomic, so the running game either sees the old module or a
    # complete new one. If the compile fails, the rename never happens and the previous module
    # survives.
    # shellcheck disable=SC2086,SC2046
    $CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -shared \
        -DDRIFTY_HOT_RELOAD -DDRIFTY_GAME_MODULE $DEV_TOOL_DEFINES \
        $(build_info_defines module "-O0,-g") $EXTRA_DEFINES \
        $GAME_SRCS $DEV_UI_SRCS $SHARED_SRCS -o "$MODULE_TMP" \
        $RAYLIB_CFLAGS $RAYLIB_SHARED_LIBS
    status=$?
    if [ $status -ne 0 ]; then
        rm -f "$MODULE_TMP"
        echo "build.sh: game module failed to compile; $MODULE left untouched." >&2
        return $status
    fi
    mv -f "$MODULE_TMP" "$MODULE" || return 1

    # If the game is already running, that is all there is to do: it will pick the module up.
    if command -v tasklist >/dev/null 2>&1; then
        if tasklist 2>/dev/null | grep -qi "drifty\.exe"; then
            echo "Hot reloading $MODULE..."
            return 0
        fi
    fi

    # shellcheck disable=SC2086,SC2046
    $CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -DDRIFTY_HOT_RELOAD \
        $(build_info_defines platform "-O0,-g") $EXTRA_DEFINES \
        $PLATFORM_SRCS $SHARED_SRCS $HOTRELOAD_SRC -o "$EXE" \
        $RAYLIB_CFLAGS $RAYLIB_SHARED_LIBS
    status=$?
    if [ $status -ne 0 ]; then
        echo "build.sh: platform layer failed to compile." >&2
        return $status
    fi

    echo "Built $MODULE and $EXE - run $EXE and leave it running."
    return 0
}

if [ "$MODE" = "smoke-test" ]; then
    build_dev || exit $?
    echo "Running bounded visual smoke test..."
    ./"$EXE" --smoke-test
    exit $?
fi

build_dev
exit $?
