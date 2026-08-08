# Drifty

A top-down 2D drift driving simulator in C11 with raylib 6.0. The design goal is a
physically coherent vehicle simulation underneath an arcade presentation layer: the car
initiates, holds, transitions, and recovers a drift because of tire, drivetrain, and
load-transfer behaviour, not because a state machine reaches in and changes forces.

**Windows only.** The supported development environment is **MSYS2 UCRT64**.

- Setup, checks, and the hot-reload rules: [CONTRIBUTING.md](CONTRIBUTING.md)
- Design contracts and the current milestone plan: [docs/](docs/README.md)
- Agent-facing workflow rules: [AGENTS.md](AGENTS.md)
- Notable changes: [CHANGELOG.md](CHANGELOG.md)

## Current phase

Two workstreams number their phases independently, so "phase 4" needs qualifying.

**Physics phases 0–3 are complete** — load transfer and handling validation. Physics phase 4
(four-wheel fidelity) is an optional, deliberate upgrade.

**Vehicle-appearance phases 0–6 are complete.** A car's appearance is a pure, total function
of its physics parameters — there is no hand-authored art for any vehicle. That workstream
built the parameter registry expansion, the appearance grammar, a 100-vehicle demonstration
corpus, the production texture path, and the in-game gallery. The grammar lives in
`src/render/car_visual.h/.c`; `build/tests/drifty_tests.exe --dump-corpus-index` prints the
fleet.

The running game uses a deterministic planar rigid-body vehicle in SI units:

| System | State |
|--------|-------|
| SI units, coordinate and sign convention | `src/core/units.h`, `src/core/config.h` |
| Math helpers (`clampf`, `lerpf`, `smooth_to`, `wrap_angle`, `smoothstep`, `lerp_angle`) | `src/core/math_utils.h/.c` |
| Fixed 120 Hz timestep with substep cap and backlog-drop counter | `src/platform/timestep.h/.c`, driven by `src/platform/main.c` |
| Held controls vs one-shot commands, consumed exactly once | `src/game/input.h/.c` |
| Deterministic fixed-tick input recording and playback | `src/game/replay.h/.c` |
| CSV telemetry writer | `src/game/telemetry.h/.c` |
| Platform-owned `Game` block, hot-reloadable game module | `src/platform/main.c`, `src/platform/hotreload_windows.c`, `src/game/game.h/.c` |
| Headless test executable | `tests/test_main.c` + `tests/scenarios/`, `tests/support/` |
| Windowless hot-reload harness | `tests/hotreload/hotreload_harness.c` |
| Bounded visual smoke test | `build/dev/drifty.exe --smoke-test` |
| Canonical vehicle specification/state/diagnostics | `src/physics/vehicle.h/.c` |
| Steering, contact kinematics, tire forces, body integration | `src/physics/physics.h/.c` |
| Normalized nonlinear lateral/longitudinal tire curves and friction ellipse | `src/physics/tire.h/.c` |
| Engine curve, signed gearing, RWD torque, brakes, handbrake, wheel integration | `src/physics/drivetrain.h/.c` |
| Interpolated body, four wheels, HUD, debug vectors | `src/render/render.h/.c` |

Front and rear lateral force use `-mu * Fz * sin(C * atan(B * alpha))`; longitudinal force
uses the same normalized form with wheel slip ratio. Engine torque is interpolated from a
seven-point curve, multiplied through forward/reverse gearing and final drive, and split
only across the locked rear axle. Service brake torque follows the configured front bias;
the handbrake is rear torque only. Each wheel integrates angular speed from drive, brake,
and tire reaction torque, and longitudinal/lateral forces share one per-wheel ellipse.

The 1.5–3.0 m/s kinematic/dynamic derivative blend remains in place. Phase 3 filters the
previous step's solved body-longitudinal acceleration, transfers axle load from the physical
CG geometry, propagates the dynamic loads into tire capacity, and applies separated
quadratic aerodynamic drag and per-wheel rolling resistance.

Neither the scenario count nor the check count is quoted here — both move with every scenario
and parameter added, and a number written down once is wrong shortly afterwards.
`./build/tests/drifty_tests.exe --list` names every scenario; running it without arguments
prints the current totals. Eight reviewed
Phase 3 CSV baselines cover acceleration/braking load transfer, coast-down, skidpad, step
steer, lift-off, transition, and a catchable drift. They live in `tests/baselines/`;
`mk regression` compares a fresh run against them, and `mk baselines` re-records them only
when the model changed on purpose and the accepted deltas have been written down.

## Prerequisites (Windows / MSYS2 UCRT64)

1. Install MSYS2 (default root `C:\msys64`):

```bat
winget install -e --id MSYS2.MSYS2 --accept-package-agreements --accept-source-agreements
```

2. Install the project toolchain and raylib 6.0 from the MSYS2 package manager:

```powershell
powershell -ExecutionPolicy Bypass -File tools\setup\setup_windows.ps1
```

That script is idempotent. It installs (when missing):

- `mingw-w64-ucrt-x86_64-gcc`
- `mingw-w64-ucrt-x86_64-raylib`
- `mingw-w64-ucrt-x86_64-pkgconf`
- `mingw-w64-ucrt-x86_64-binutils`
- `make`

There is **no** Chocolatey GCC path, **no** vendored `vendor/raylib` build, and **no**
manual raylib source compile. raylib comes only from the MSYS2 package.

## Building

`build.bat` is the Windows entry point: it enters MSYS2 UCRT64 and runs `build.sh`.
`build.sh` is the canonical implementation and refuses to run outside UCRT64.
The `Makefile` exposes the same configurations and the same flags.

```bat
build.bat                 rem hot-reload development build
build.bat --release       rem single executable, static raylib, no game.dll
build.bat --tests         rem headless test executable
build.bat --hotreload-harness
build.bat --smoke-test    rem build, then bounded visual smoke test (exits alone)
build.bat --clean
```

Equivalent inside an MSYS2 UCRT64 shell:

```bash
./build.sh
./build.sh --release
./build.sh --tests
./build.sh --hotreload-harness
./build.sh --smoke-test
./build.sh --clean
```

```bash
make debug
make release
make tests
make hotreload-harness
make run-tests
make smoke-test
make clean
make info
```

Every normal build command terminates immediately. Nothing starts a watcher or leaves a
persistent game process running. `--smoke-test` launches the real window, runs a fixed
frame budget, writes a screenshot, and exits.

### Running

```bat
build/dev/drifty.exe                 rem development build; start once and leave it running
build/dev/drifty.exe --smoke-test    rem bounded visual verification; exits on its own
build/release/drifty_release.exe         rem release build
build/tests/drifty_tests.exe           rem headless tests; run from the repository root
build/dev/drifty_hotreload_harness.exe
```

`build/tests/drifty_tests.exe` writes CSV telemetry to `artifacts/telemetry/` relative to the working directory,
so run it from the repository root. It accepts `--list`, `--scenario NAME`, and `-v`.

## Linkage

### Development (hot reload)

`build/dev/drifty.exe` and `build/dev/game.dll` both link the MSYS2 **shared** raylib import library and
therefore both import `libraylib.dll` (MSYS2's DLL name). The build copies:

- `libraylib.dll`
- `glfw3.dll` (required by `libraylib.dll`)

next to the executables so launching from Explorer or a normal terminal does not depend on
a hand-edited `PATH`. Those DLLs are generated and gitignored.

Never compile raylib sources into `game.dll`.

### Release

`build/release/drifty_release.exe` compiles platform + game into one executable with `DRIFTY_HOT_RELOAD`
undefined. It links `libraylib.a` statically and does **not** import `libraylib.dll` or
`game.dll`. With the current MSYS2 raylib package, the static archive still references
shared GLFW, so `glfw3.dll` is copied next to the release executable. That is a package
limitation, not a project DLL.

### Verify imports

```bash
objdump -p build/dev/drifty.exe | grep -i "DLL Name"
objdump -p build/dev/game.dll | grep -i "DLL Name"
objdump -p build/tests/drifty_tests.exe | grep -i "DLL Name"
objdump -p build/release/drifty_release.exe | grep -i "DLL Name"
```

Expected: development artifacts import `libraylib.dll`; tests and release do not.

## Hot-reload workflow

The game is a thin platform layer (`build/dev/drifty.exe`) plus a hot-reloadable game module
(`build/dev/game.dll`). The platform layer owns the window, the raylib context, the `Game`
allocation, and the fixed-timestep loop. Everything else lives in the module.

1. Run `build/dev/drifty.exe` once and leave it open.
2. Edit game code.
3. Run `build.bat` (or `./build.sh` in UCRT64). It rebuilds the module always, rebuilds the
   executable only when it is not already running, and returns in well under a second.
4. The running game notices the new module and swaps it in, keeping its position, counters,
   and checksum.

The loader never unloads a working module until a replacement has been proven good. A
compile error cannot close the running game.

Automated validation without leaving `build/dev/drifty.exe` running:

```bat
build.bat --hotreload-harness
build/dev/drifty_hotreload_harness.exe
```

Or the fuller script (harness + failed-compile preservation):

```bash
# inside MSYS2 UCRT64, from the repo root
./tools/setup/validate_hotreload.sh
```

### When a restart is required

Hot reload cannot handle these. Restart `build/dev/drifty.exe` after:

- A change to the layout of `Game` or anything it contains.
- **Adding a field to `VehicleSpec`** (`src/physics/vehicle.h`) — it lives inside `Game`, so this is
  the same layout change. Worth its own line because every new tunable parameter is a field
  on it, so this is the restart trigger you will hit most often.
- A change to `src/platform/main.c`, `src/platform/timestep.c`, or `src/platform/hotreload_windows.c`.
- A change to `GAME_ENTRY_POINTS`.

### Reload-safety rules for game code

- No pointer stored in `Game`, or reachable from it, may point into the module's code or
  static data.
- No function pointers in persistent state.
- The `Game` block is allocated and owned by the platform layer. Never declare
  `static Game game;` inside the module.
- Anything raylib tracks is released in `game_pre_reload` and re-acquired in
  `game_post_reload`.

## Development tooling

The game itself stays plain C and raylib. Around it sits a development shell — an in-game
Physics Lab, a replay inspector, telemetry reports, failure bundles, and one command per
operation. `make help` lists every target.

```bash
mk test                 # fast scenarios          mk verify        analysis + tests + regression
mk scenario NAME=skidpad
mk report NAME=skidpad  # self-contained HTML report with plots and a baseline comparison
mk ci                   # format, lint, analyze, every scenario, regression, sanitizers, coverage
mk cards                # per-car sprites, feature-label maps and cards.json (headless)
mk visual-diagnose      # appearance measurements and evidence into artifacts/visual/
mk gallery              # the fleet through the production texture path, for human review
```

`mk.bat` enters MSYS2 UCRT64 for you and works from cmd.exe or PowerShell; from a UCRT64
shell use `make <target>`. Every target terminates on its own except **`mk run`** (launches
the game) and **`mk inspect`** (serves the browser inspector) — use `mk visual-diagnose`
instead of the latter, since it starts and stops its own server.

Press **F2** in the running game for the Physics Lab: scenario selector, pause and single
step, live sliders for every tunable in the registry with its default and unit — currently
123, listed by `drifty_tests --dump-params` — tuning profiles, overlay toggles,
an eight-channel scope with a baseline ghost, and an invariant panel. **F3** opens the replay
inspector.

![The Physics Lab](tests/visual/baseline/physics_lab.png)

Every tunable is defined once, in `src/dev/dev_params.c`, and that one definition generates the
sliders, the profile format, the telemetry metadata, and the `--dump-params` table.

## Known limitations

- **Changing the `Game` struct layout requires a restart.** Inherent to the technique.
- **Restart `build/dev/drifty.exe` once for the development-tool layout.** `Game` now carries the
  Physics Lab's state (`DevState`); restart the executable once after updating, and ordinary
  module-only hot reload preserves state as usual after that.
- **Restart `build/dev/drifty.exe` once for the Phase 2 layout.** Canonical vehicle diagnostics and
  reverse gearing changed persistent structure layout; normal code-only hot reload works
  after that restart.
- **Every new tunable parameter costs one restart.** A parameter is a field on `VehicleSpec`,
  which lives inside `Game`, so adding one is a layout change like any other. Expect this
  while the parameter set is still growing.
- **Linux gameplay remains unsupported.** Linux builds cover the headless targets only and
  are not a substitute for the MSYS2 UCRT64 gameplay build.
- **Release still needs `glfw3.dll`.** The MSYS2 `libraylib.a` was built against shared
  GLFW (`__imp_glfw*`), so a fully static single-file release without any third-party DLL
  is blocked by that package layout. `libraylib.dll` and `game.dll` are not required.

## Layout

Grouped by directory, so this block can be checked against `ls` rather than read as prose.

```
Makefile, build.sh, build.bat             build entry points; all terminate immediately
mk.bat                                    run a Makefile target inside MSYS2 UCRT64 from cmd.exe

src/core/config.h                         default constants, every physical value unit-bearing
src/core/units.h                          world<->render conversion and the coordinate convention
src/core/math_utils.h/.c                  scalar helpers raymath.h does not provide

src/platform/main.c                       platform layer: window, Game allocation, fixed-timestep loop
src/platform/timestep.h/.c                the accumulator, isolated so the harness can assert it
src/platform/hotreload.h                  GAME_ENTRY_POINTS, the one authoritative entry-point list
src/platform/hotreload_windows.c          LoadLibrary / GetProcAddress loader
src/platform/build_info.h                 commit, branch, dirty flag, compiler, flags, platform

src/game/game.h/.c                        the Game block and the reloadable entry points
src/game/input.h/.c                       held controls and one-shot commands
src/game/ai_driver.h/.c                   baseline lap driver; writes only the Input a player writes
src/game/car_roster.h/.c                  the six validated cars, two per drivetrain layout
src/game/replay.h/.c                      deterministic fixed-tick input timeline
src/game/telemetry.h/.c                   CSV row writer, no raylib dependency
src/game/validation_metrics.h/.c          pure reduction of a lap run to summary metrics
src/game/run_report.h/.c                  writes the run.json one validation lap is judged against
src/game/particle.h/.c                    skidmarks and tire smoke
src/game/audio.h/.c                       engine, screech, and impact playback
src/game/profile.h/.c                     zone instrumentation: off, built-in timers, or Tracy

src/physics/vehicle.h/.c                  canonical vehicle data and initialization
src/physics/tire.h/.c                     pure nonlinear curves, slip ratio, combined-friction limit
src/physics/drivetrain.h/.c               pure engine/gearing/torque/wheel dynamics
src/physics/auto_transmission.h/.c        automatic shift logic over the drivetrain
src/physics/physics.h/.c                  pure Phase 3 integration and fixed-update owner
src/physics/surface.h/.c                  surface types by SurfaceId, resolved at point of use

src/world/track.h/.c                      track layout, surface regions, learned racing line
src/world/collision.h/.c                  swept body collision against track geometry

src/render/car_visual.h/.c                the appearance grammar: VehicleSpec -> CarVisual, raylib-free
src/render/car_visual_raster.h/.c         CPU rasterizer, feature-label maps, nose-up rotation
src/render/car_appearance.h               presentation-only paint identity, kept out of VehicleSpec
src/render/vehicle_effects.h/.c           transient braking/saturation/load feedback, render-only
src/render/render.h/.c                    interpolation, pixel-art target, GPU lifecycle, frame orchestration
src/render/render_internal.h              the implementation-only seams between the renderer's units
src/render/render_world.c                 the track, the tire smoke, and the debug vector overlay
src/render/render_vehicle.c               baked sprite upload, the cached car, the corpus gallery
src/render/render_hud.c                   diagnostics readout, arcade HUD, full-screen state overlays
src/render/render_validation_overlay.c    identity/progress overlay burned into captured video

src/dev/dev_params.h/.c                   the one tunable registry: sliders, profiles, docs, metadata
src/dev/dev_presets.h/.c                  hand-designed vehicle presets
src/dev/dev_scenario.h/.c                 scripted maneuvers, shared by the lab and the headless runner
src/dev/dev_state.h/.c                    lab state inside Game: scope, trajectory, invariant monitor
src/dev/dev_lab.h/.c                      the raygui Physics Lab (development builds only)
src/dev/dev_replay.h/.c                   durable replay timelines and the inspector's event markers
src/dev/failure_bundle.h/.c               reproducible failure directories
src/dev/car_corpus.h/.c                   the 100 demonstration vehicles as pure functions of an index
src/dev/car_corpus_archetypes.c           the 17 hand-designed archetype forms and their table
src/dev/car_corpus_internal.h             the seam between that table and the corpus machinery

tests/test_main.c                         headless scenario runner
tests/test_scenarios.h                    the registry contract between runner and scenario groups
tests/test_commands.h/.c                  the non-scenario modes: generators, benchmarks, verifiers
tests/scenarios/                          the five scenario groups the runner registers
tests/support/                            check framework, Game->row projection, appearance metrics
tests/hotreload/                          windowless hot-reload validation
tests/baselines/                          reviewed deterministic scenario CSV baselines
tests/visual/                             deterministic scene baselines and the RMSE gate

tools/setup/                              MSYS2 UCRT64 bootstrap, hot-reload validation
tools/build/                              compile_commands.json generation
tools/telemetry/                          telemetry comparison, plots, summaries, HTML reports
tools/validation/                         suite runner, run diffing, racing-line optimiser
tools/appearance/                         RGBA regression and sprite rotation measurement
tools/recording/                          gameplay capture
tools/visual/                             browser appearance inspector and its Playwright measurements

data/input/                               the SDL game controller database
data/vehicles/                            reviewed vehicle profiles; the Physics Lab also saves here
data/vehicles/corpus/                     the corpus exported as tuning profiles; checked in, and the
                                          `corpus` scenario asserts it round-trips

docs/README.md                            the index for everything below it
docs/design/                              current contracts, present tense
docs/PLAN.md                              the active milestone plan, future tense

fuzz/fuzz_*.c                             libFuzzer targets for the parsers and the tire functions
third_party/raygui/                       vendored raygui, development builds only
third_party/stb/                          vendored stb_image_write, headless PNG output
resources/audio/                          runtime audio assets
artifacts/                                telemetry CSV, reports, screenshots, replays,
                                          failure bundles, cards, gallery (all gitignored)
```

## Licence

Drifty is MIT licensed. See [LICENSE](LICENSE).

Vendored third-party sources under `third_party/` keep their own licences — raygui is
zlib/libpng and stb_image_write is dual public domain / MIT. Neither is linked into the
release build. [third_party/README.md](third_party/README.md) records the provenance,
the exact version, and the licence of each one.
