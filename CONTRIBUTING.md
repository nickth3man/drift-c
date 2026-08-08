# Contributing to Drifty

Behavioural rules for AI agents are in [AGENTS.md](AGENTS.md). This file is about the
mechanics: what to install, what to run, and what has to be green before you push.

## Setting up

**Windows only.** The supported environment is MSYS2 UCRT64; see the prerequisites section
in the [README](README.md#prerequisites-windows--msys2-ucrt64) and run:

```bash
tools/setup/setup_windows.ps1
```

Expected by `make verify`: `clang`, `clang-tools-extra` (clang-format), and `cppcheck`.
`gcovr` is only needed by `make coverage`, which `make ci` runs and `verify` does not. All
four come from `pacman`, and `make info` prints which of them it found.

**Python is a prerequisite, not an optional extra.** `make regression` compares telemetry
through `tools/telemetry/compare_telemetry.py`, and `make compile-commands` is a Python
script too, so `verify` cannot complete without it. `setup_windows.ps1` does not install it:

```bash
pacman -S mingw-w64-ucrt-x86_64-python mingw-w64-ucrt-x86_64-python-pip
```

Then the Python tooling — ruff and pre-commit:

```bash
pip install -r requirements-dev.txt
```

Everything else under `tools/` is deliberately pure-stdlib, so the telemetry and appearance
checks run on a bare Python. Prefer keeping it that way over adding a dependency.

Install the commit hooks once per clone:

```bash
pre-commit install
```

## Before you push

```bash
make verify
```

That is format-check, ruff, cppcheck, the clang analyzer, every physics scenario, and the
baseline regression. `make verify-fast` is the quicker format-and-tests subset while
iterating.

### A SKIP is not a pass

Targets whose tool is not installed print `SKIP` and exit 0, so a local run can be green
having checked very little. To find out what actually ran:

```bash
make DRIFTY_STRICT=1 verify
```

Under `DRIFTY_STRICT`, a missing tool is a hard failure. This is what CI passes, so if it
fails for you it will fail there too.

The one exception is the sanitizer runtime check. MSYS2 ships ASan/UBSan in CLANG64, not in
the supported UCRT64 environment, so `make sanitize` skips unconditionally on Windows. The
Linux CI job is where the sanitizers actually run.

## CI

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs two jobs on every push and pull
request:

| Job | What it covers |
| --- | --- |
| `windows-ucrt64` | The canonical toolchain: format-check, cppcheck, clang analyzer, physics scenarios, baseline regression |
| `linux-headless` | Python lint, the headless scenarios and regression, and ASan/UBSan |

Python linting runs on Linux only. It is host-independent, and MSYS2's ruff tracks a
different version from the pinned one, so running it on both would mean two versions
disagreeing about formatting.

## Formatting

C is `clang-format` (`.clang-format`), Python is `ruff` (`pyproject.toml`). `pre-commit`
applies both on commit; `make format` and `make format-py` apply them by hand.

Line endings are LF, enforced by `.gitattributes` rather than by each developer's
`core.autocrlf`. This matters more than it looks: clang-format counts a `\r` toward
`ColumnLimit`, so C formatted on a CRLF worktree wraps its backslash continuations one column
differently and fails `make format-check` on any LF checkout, CI included.

## Baselines

`tests/baselines/` holds reviewed, committed, deterministic telemetry. `make regression`
compares a fresh run against it with tolerances.

Re-recording them with `make baselines` overwrites the thing that would have caught a
physics regression. Do it only when you meant to change physics, and **say so in the commit
message, with what changed and why the new numbers are right**. A baseline update that says
"update baselines" is indistinguishable from a bug being blessed.

**The baselines are platform-specific.** They are recorded on Windows/UCRT64, and the
tolerances are tight enough that a different libm does not reproduce them — the first CI run
breached 13 of 46 comparisons on Linux, all longitudinal, with identical aggregate metrics
but large single-row deltas. That is a discrete event landing one tick apart, not drifting
physics: the scenario assertions pass identically on both hosts. So `make regression` is
gated on the Windows job only, and re-record baselines on Windows.

## Hot reload

The game is a thin platform layer (`build/dev/drifty.exe`) plus a reloadable module
(`build/dev/game.dll`). Run the executable once, leave it open, and rebuild with `build.bat`
or `./build.sh`; the running game swaps the new module in. A compile error cannot close it —
the link output only moves into place on success.

**Restart is required after** a change to the layout of `Game` or anything reachable from it
(most often: adding a field to `VehicleSpec`), to `src/platform/main.c`, `timestep.c`, or
`hotreload_windows.c`, or to `GAME_ENTRY_POINTS`.

**Reload-safety rules for module code:**

- No pointer in `Game`, or reachable from it, may point into the module's code or static data.
- No function pointers in persistent state.
- `Game` is allocated and owned by the platform layer. Never declare `static Game game;`
  inside the module.
- Anything raylib tracks is released in `game_pre_reload` and re-acquired in
  `game_post_reload`.

Validate it without a window:

```bash
tools/setup/validate_hotreload.sh
```

## Commits and pull requests

Small, reviewable commits, each one doing a single thing. Explain *why* in the body — the
diff already says what. The [PR template](.github/pull_request_template.md) lists what a
review expects.
