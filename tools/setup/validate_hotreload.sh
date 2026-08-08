#!/bin/sh
# tools/setup/validate_hotreload.sh — bounded hot-reload + failed-build preservation checks.
# Must run under MSYS2 UCRT64 (via build.bat --hotreload-harness path or directly).
set -eu

cd "$(dirname "$0")/.."

echo "== building module + harness =="
./build.sh --hotreload-harness

echo "== running windowless harness =="
./build/dev/drifty_hotreload_harness.exe

echo "== failed compile must preserve build/dev/game.dll =="
SUM_BEFORE="$(sha256sum build/dev/game.dll | awk '{print $1}')"
SIZE_BEFORE="$(wc -c < build/dev/game.dll)"

# Deliberately break a game translation unit, attempt a module build, restore it.
cp -f src/game/game.c src/game/game.c.harness_bak
printf '%s\n' '#error deliberate harness compile failure' >> src/game/game.c
set +e
./build.sh
STATUS=$?
set -e
mv -f src/game/game.c.harness_bak src/game/game.c

if [ "$STATUS" -eq 0 ]; then
    echo "validate_hotreload: expected build failure, got success" >&2
    exit 1
fi

SUM_AFTER="$(sha256sum build/dev/game.dll | awk '{print $1}')"
SIZE_AFTER="$(wc -c < build/dev/game.dll)"

if [ "$SUM_BEFORE" != "$SUM_AFTER" ] || [ "$SIZE_BEFORE" != "$SIZE_AFTER" ]; then
    echo "validate_hotreload: build/dev/game.dll changed after failed compile" >&2
    exit 1
fi

if ls build/dev/game_tmp.tmp >/dev/null 2>&1; then
    echo "validate_hotreload: temporary module artifact left behind" >&2
    exit 1
fi

if ls build/dev/game_load_*.dll >/dev/null 2>&1; then
    echo "validate_hotreload: stale load copies remain after harness" >&2
    exit 1
fi

echo "validate_hotreload: ok (module preserved sha256=$SUM_AFTER)"
