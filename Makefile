# Drifty — one command per operation.
#
# These local targets are the project's checks. There is no hosted CI: the compiler/OS
# matrix, hot-reload harness, linkage inspection, and security analysis are run by hand.
#
#   make dev              hot-reload development build: build/dev/game.dll + build/dev/drifty.exe
#   make run              build, then LAUNCH the game (interactive; blocks until the window closes)
#   make test             fast unit and infrastructure scenarios
#   make test-physics     every physics and maneuver scenario, with telemetry
#   make scenario NAME=skidpad     one scenario
#   make report NAME=skidpad       one scenario, then a self-contained HTML report
#   make regression       compare artifacts/telemetry against tests/baselines/ with tolerances
#   make baselines        re-record tests/baselines/ from the current build (explain it!)
#   make verify-fast      format check + tests
#   make verify           static analysis + tests + physics + regression
#   make sanitize         ASan + UBSan build and tests (clang)
#   make coverage         gcov/gcovr text, HTML, and Cobertura output
#   make screenshots      capture the deterministic visual scenes
#   make visual-test      compare captured scenes against tests/visual/baseline
#   make gallery          capture every page of the in-game vehicle corpus gallery
#   make profile          build with the Tracy hooks enabled (DRIFTY_TRACY)
#   make benchmark        fixed-update throughput
#   make release          release build
#   make ci               core local checks; inspect every SKIP line
#   make compile-commands write compile_commands.json for clangd
#   make format           apply .clang-format        make format-check  check only
#   make lint             cppcheck                   make analyze       clang --analyze
#   make fuzz             build and briefly run the libFuzzer targets (clang)
#   make clean            remove every generated artifact
#   make info             print the resolved toolchain and linkage
#   make help             this list
#
# Every target terminates on its own except the interactive `run` and `inspect` targets.
#
# On Windows the canonical build lives in build.sh; the targets below call it rather than
# duplicating the hot-reload-safe link sequence. The headless targets (tests, sanitizers,
# coverage, fuzzing) also build on Linux; the interactive game remains Windows-only.

# ------------------------------------------------------------------------------- host --

ifeq ($(MSYSTEM),UCRT64)
    DRIFTY_HOST := ucrt64
else
    UNAME_S := $(shell uname -s 2>/dev/null)
ifneq (,$(filter Linux Darwin,$(UNAME_S)))
        DRIFTY_HOST := posix
else
        DRIFTY_HOST := unsupported
endif
endif

ifeq ($(DRIFTY_HOST),unsupported)
$(error Run make from an MSYS2 UCRT64 shell (or use build.bat / mk.bat), or from Linux for the headless targets.)
endif

# --------------------------------------------------------------------------- toolchain --

ifeq ($(DRIFTY_HOST),ucrt64)

CC := gcc
CC_PATH := $(shell command -v $(CC) 2>/dev/null)
ifeq ($(findstring /ucrt64/bin/,$(CC_PATH)),)
$(error refusing non-UCRT64 compiler '$(CC_PATH)'. Use build.bat or the UCRT64 shell.)
endif

PKGCONFIG := $(shell command -v pkg-config 2>/dev/null)
ifeq ($(PKGCONFIG),)
$(error pkg-config not found. Run scripts/setup_windows.ps1.)
endif
ifeq ($(shell pkg-config --exists raylib 2>/dev/null && echo yes),)
$(error pkg-config cannot find raylib. Run scripts/setup_windows.ps1.)
endif

RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib)
EXE_SUFFIX    := .exe
# On Windows `python3` is usually the Microsoft Store stub, which does nothing useful.
PYTHON ?= $(shell command -v python 2>/dev/null || command -v python3 2>/dev/null)

else   # posix: headless only

CC ?= cc
# raylib is needed for its HEADER only — the headless build calls no raylib function and
# links no raylib library. Point RAYLIB_INCLUDE_DIR at any raylib source or install tree.
RAYLIB_INCLUDE_DIR ?= $(shell pkg-config --variable=includedir raylib 2>/dev/null)
ifeq ($(strip $(RAYLIB_INCLUDE_DIR)),)
    RAYLIB_INCLUDE_DIR := third_party/raylib-src/src
endif
RAYLIB_CFLAGS := -I$(RAYLIB_INCLUDE_DIR)
EXE_SUFFIX    :=
PYTHON ?= $(shell command -v python3 2>/dev/null || command -v python 2>/dev/null)

endif

# Optional tools. Missing ones make their target explain how to install rather than fail
# with a shell error, so a SKIP line is the only signal that a check did not actually run.
CLANG        := $(shell command -v clang 2>/dev/null)
CLANG_FORMAT := $(shell command -v clang-format 2>/dev/null)
CPPCHECK     := $(shell command -v cppcheck 2>/dev/null)
GCOVR        := $(shell command -v gcovr 2>/dev/null)
MAGICK       := $(shell command -v magick 2>/dev/null)

# ------------------------------------------------------------------------------- flags --

CSTD     := -std=c11
INCLUDES := -Isrc -Itests -Ithird_party -Ithird_party/raygui
WARNINGS := -Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith
DEBUG_FLAGS   := -O0 -g
RELEASE_FLAGS := -O2 -DNDEBUG

BUILD_COMMIT := $(shell git rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
BUILD_BRANCH := $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)
BUILD_DIRTY  := $(shell git diff --quiet HEAD 2>/dev/null && echo clean || echo dirty)
BUILD_DEFINES = -DDRIFTY_BUILD_COMMIT=\"$(BUILD_COMMIT)\" \
                -DDRIFTY_BUILD_BRANCH=\"$(BUILD_BRANCH)\" \
                -DDRIFTY_BUILD_DIRTY=\"$(BUILD_DIRTY)\"

# ----------------------------------------------------------------------------- sources --
#
# THE source manifest. build.sh and tools/build/gen_compile_commands.py both read these
# groups out of this file through `print-source-groups` / `print-source-group`,
# so link membership is declared exactly once. It used to be declared four times, and the
# fourth copy had already drifted.
#
# Membership is explicit on purpose: no globs for link membership and no recursive make,
# because a glob silently adopts a new file into every binary that happens to match it.
#
# Paths must stay repository-relative and free of whitespace and apostrophes: one manifest is
# consumed by both Make and POSIX shell.

SHARED_SRCS := src/game/input.c src/core/math_utils.c src/dev/dev_scenario.c src/game/profile.c src/render/car_visual.c src/render/car_visual_raster.c src/render/vehicle_effects.c
DEV_SRCS    := src/dev/dev_params.c src/dev/dev_presets.c src/dev/dev_replay.c src/dev/dev_state.c \
               src/dev/failure_bundle.c src/dev/car_corpus.c src/dev/car_corpus_archetypes.c
DEV_UI_SRCS := src/dev/dev_lab.c
GAME_SRCS   := src/game/game.c src/game/ai_driver.c src/game/audio.c src/game/car_roster.c src/physics/auto_transmission.c src/game/particle.c src/physics/vehicle.c \
               src/physics/physics.c src/physics/tire.c src/physics/drivetrain.c src/physics/surface.c src/world/track.c \
               src/world/collision.c src/render/render.c src/render/render_world.c \
               src/render/render_vehicle.c src/render/render_hud.c src/game/replay.c src/game/telemetry.c \
               $(DEV_SRCS)
PLATFORM_SRCS := src/platform/main.c src/platform/timestep.c
HOTRELOAD_SRC := src/platform/hotreload_windows.c

# Test-owned translation units only. TEST_SRCS below is the headless LINK CLOSURE: the
# runner plus everything it pulls in.
TEST_RUNNER_SRCS := tests/test_main.c tests/test_commands.c \
                    tests/support/test_harness.c tests/support/simulation_fixture.c \
                    tests/support/appearance_metrics.c tests/support/car_sheet.c \
                    tests/scenarios/core_tests.c tests/scenarios/appearance_tests.c \
                    tests/scenarios/physics_tests.c tests/scenarios/handling_tests.c \
                    tests/scenarios/gameplay_tests.c
TEST_SRCS   := $(TEST_RUNNER_SRCS) src/platform/timestep.c $(GAME_SRCS) $(SHARED_SRCS)
HOTRELOAD_HARNESS_SRCS := tests/hotreload/hotreload_harness.c src/platform/timestep.c $(HOTRELOAD_SRC) $(SHARED_SRCS)

# The support set every libFuzzer target links against.
FUZZ_SUPPORT_SRCS := src/dev/dev_params.c src/dev/dev_replay.c src/physics/vehicle.c src/physics/tire.c \
                     src/game/replay.c src/core/math_utils.c src/game/input.c

# The groups `print-source-groups` exports, in the order it prints them.
SOURCE_GROUP_NAMES := SHARED_SRCS DEV_SRCS DEV_UI_SRCS GAME_SRCS PLATFORM_SRCS \
                      HOTRELOAD_SRC TEST_RUNNER_SRCS TEST_SRCS HOTRELOAD_HARNESS_SRCS \
                      FUZZ_SUPPORT_SRCS

# Formatter input is derived from the manifest rather than globbed, so a file that is not in
# any build cannot quietly become the only thing the formatter checks.
ALL_C_SRCS  := $(sort $(PLATFORM_SRCS) $(HOTRELOAD_SRC) $(GAME_SRCS) $(DEV_UI_SRCS) \
                      $(TEST_RUNNER_SRCS) $(TEST_SRCS) $(HOTRELOAD_HARNESS_SRCS) \
                      $(wildcard fuzz/*.c))
ALL_H_SRCS  := $(sort $(wildcard src/*.h src/*/*.h tests/*.h tests/*/*.h))

# Static analysis covers the headless-safe set. The exclusions are named as translation units,
# NOT as source groups: subtracting whole groups looks tidier but HOTRELOAD_HARNESS_SRCS and
# PLATFORM_SRCS both contain shared code, so `filter-out $(HOTRELOAD_HARNESS_SRCS)` would take
# car_visual.c, car_visual_raster.c, input.c, math_utils.c, dev_scenario.c, profile.c and
# timestep.c out of analysis as collateral — the appearance core included.
#
# What genuinely cannot be analysed: the Windows-only platform entry point, the hot-reload
# loader, dev_lab.c (mostly a 6000-line vendored header, not our code to fix), the harness
# entry point, and the libFuzzer drivers, which need -fsanitize=fuzzer to parse.
NO_ANALYZE_SRCS := src/platform/main.c $(HOTRELOAD_SRC) $(DEV_UI_SRCS) tests/hotreload/hotreload_harness.c \
                   $(wildcard fuzz/*.c)
ANALYZE_SRCS := $(filter-out $(NO_ANALYZE_SRCS),$(ALL_C_SRCS))

# ----------------------------------------------------------------------------- outputs --
#
# One directory per configuration under build/. The GNU Make manual recommends keeping
# binaries out of the source tree; no VPATH or recursive make is needed for it because every
# recipe here already names its sources explicitly.
BUILD_DIR      := build
BUILD_DEV      := $(BUILD_DIR)/dev
BUILD_TESTS    := $(BUILD_DIR)/tests
BUILD_RELEASE  := $(BUILD_DIR)/release
BUILD_SANITIZE := $(BUILD_DIR)/sanitize
BUILD_COVERAGE := $(BUILD_DIR)/coverage
BUILD_FUZZ     := $(BUILD_DIR)/fuzz
BUILD_PACKAGES := $(BUILD_DIR)/packages

EXE_TESTS   := $(BUILD_TESTS)/drifty_tests$(EXE_SUFFIX)
EXE_DEBUG   := $(BUILD_DEV)/drifty$(EXE_SUFFIX)
EXE_RELEASE := $(BUILD_RELEASE)/drifty_release$(EXE_SUFFIX)

ARTIFACTS := artifacts
# Ephemeral run evidence, all of it under the already-ignored artifacts/ root.
TELEMETRY := $(ARTIFACTS)/telemetry
BASELINES := tests/baselines
SCENES    := debug_overlay tire_curves drift_hud physics_lab \
             accel_load brake_load skidpad_p3 lift_off transition_p3 catchable

# The scenarios that write telemetry and are compared against a baseline.
REGRESSION_SCENARIOS := skidpad step-steer transition lift-off \
                        accel-load brake-load coast-down catchable-drift

.PHONY: all help info dev run release tests test test-physics scenario report regression \
        baselines verify-fast verify sanitize coverage screenshots visual-test gallery profile \
        benchmark ci compile-commands format format-check lint analyze fuzz \
        clean clean-telemetry dirs windows-only cards inspect visual-diagnose \
        print-source-groups print-source-group

all: dev

help:
	@sed -n '6,39p' Makefile

info:
	@echo "host        : $(DRIFTY_HOST)"
	@echo "compiler    : $(CC)"
	@echo "raylib cflags: $(RAYLIB_CFLAGS)"
	@echo "python      : $(PYTHON)"
	@echo "commit      : $(BUILD_COMMIT) ($(BUILD_BRANCH), $(BUILD_DIRTY))"
	@echo "clang       : $(if $(CLANG),$(CLANG),not installed)"
	@echo "clang-format: $(if $(CLANG_FORMAT),$(CLANG_FORMAT),not installed)"
	@echo "cppcheck    : $(if $(CPPCHECK),$(CPPCHECK),not installed)"
	@echo "gcovr       : $(if $(GCOVR),$(GCOVR),not installed)"
	@echo "magick      : $(if $(MAGICK),$(MAGICK),not installed)"

# ------------------------------------------------------------- the source manifest, out --
#
# `print-source-groups` emits one single-quoted shell assignment per group, in
# SOURCE_GROUP_NAMES order, so a POSIX shell can adopt the whole manifest at once:
#
#     eval "$(make --no-print-directory -s print-source-groups)"
#
# `print-source-group GROUP=NAME` prints one group's unquoted, space-separated value, which
# is what a script wants to interpolate straight into a compiler command line. An unknown
# or empty group exits 2 rather than expanding to nothing and silently linking less.

print-source-groups:
	@$(foreach g,$(SOURCE_GROUP_NAMES),printf "%s='%s'\n" '$(g)' '$(strip $($(g)))';)

print-source-group:
	@name='$(strip $(GROUP))'; \
	known=' $(SOURCE_GROUP_NAMES) '; \
	if [ -z "$$name" ]; then \
	    echo "make: print-source-group needs GROUP=NAME (one of:$$known)" >&2; exit 2; \
	fi; \
	case "$$known" in \
	    *" $$name "*) ;; \
	    *) echo "make: unknown source group '$$name' (one of:$$known)" >&2; exit 2 ;; \
	esac; \
	value='$(strip $($(strip $(GROUP))))'; \
	if [ -z "$$value" ]; then \
	    echo "make: source group '$$name' is empty" >&2; exit 2; \
	fi; \
	printf '%s\n' "$$value"

dirs:
	@mkdir -p $(BUILD_DEV) $(BUILD_TESTS) $(BUILD_RELEASE) $(TELEMETRY) $(ARTIFACTS)

windows-only:
ifneq ($(DRIFTY_HOST),ucrt64)
	@echo "This target builds the game itself and needs MSYS2 UCRT64 on Windows." >&2
	@exit 1
endif

# ------------------------------------------------------------------ builds (delegated) --
#
# build.sh owns the link sequence that keeps hot reload safe (temporary output name plus an
# atomic rename). Duplicating it here would eventually mean two subtly different builds.

dev: windows-only
	./build.sh

run: dev
	@echo "Launching $(EXE_DEBUG). This returns only after you close the window."
	./$(EXE_DEBUG)

release: windows-only
	./build.sh --release

ifeq ($(DRIFTY_HOST),ucrt64)
tests:
	./build.sh --tests
else
tests: dirs
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(RELEASE_FLAGS) -DDRIFTY_HEADLESS \
	    $(BUILD_DEFINES) -DDRIFTY_BUILD_MODE=\"tests\" -DDRIFTY_BUILD_FLAGS=\"-O2,-DNDEBUG\" \
	    $(TEST_SRCS) -o $(EXE_TESTS) $(RAYLIB_CFLAGS) -lm
	@echo "Built $(EXE_TESTS)."
endif

# ------------------------------------------------------------------------------- tests --

# Fast feedback: infrastructure and pure-function scenarios, no long maneuvers.
test: tests
	./$(EXE_TESTS) --scenario math
	./$(EXE_TESTS) --scenario units
	./$(EXE_TESTS) --scenario timestep
	./$(EXE_TESTS) --scenario oneshot
	./$(EXE_TESTS) --scenario replay
	./$(EXE_TESTS) --scenario devreplay
	./$(EXE_TESTS) --scenario params
	./$(EXE_TESTS) --scenario telemetry
	./$(EXE_TESTS) --scenario renderscale

test-physics: tests
	./$(EXE_TESTS)

scenario: tests
	@test -n "$(NAME)" || (echo "usage: make scenario NAME=skidpad" >&2; exit 2)
	./$(EXE_TESTS) --scenario $(NAME)

benchmark: tests
	./$(EXE_TESTS) --benchmark 240000

# ------------------------------------------------------------------- telemetry tooling --

report: tests
	@test -n "$(NAME)" || (echo "usage: make report NAME=skidpad" >&2; exit 2)
	./$(EXE_TESTS) --scenario $(NAME)
	$(PYTHON) tools/telemetry/make_report.py $(TELEMETRY)/scenario_$(NAME).csv \
	    $(if $(wildcard $(BASELINES)/scenario_$(NAME).csv),--baseline $(BASELINES)/scenario_$(NAME).csv,) \
	    --title "Drifty — $(NAME)" --out $(ARTIFACTS)/report_$(NAME).html
	@echo "open $(ARTIFACTS)/report_$(NAME).html"

regression: test-physics
	$(PYTHON) tools/telemetry/compare_telemetry.py --baseline-dir $(BASELINES) --current-dir $(TELEMETRY) \
	    --markdown $(ARTIFACTS)/regression.md

# Re-recording a baseline is never a way to make a failing test green. Say why in the PR.
baselines: test-physics
	@echo "Re-recording baselines from the CURRENT build."
	@echo "A PR that changes these must explain, in words, why the new numbers are correct."
	cp -f $(TELEMETRY)/scenario_*.csv $(BASELINES)/
	cp -f $(TELEMETRY)/phase2_*.csv $(BASELINES)/ 2>/dev/null || true
	@ls -la $(BASELINES)

# ------------------------------------------------------------------- quality and gates --

format:
ifeq ($(CLANG_FORMAT),)
	@echo "clang-format not installed. pacman -S mingw-w64-ucrt-x86_64-clang-tools-extra" >&2
	@exit 1
else
	$(CLANG_FORMAT) -i $(ALL_C_SRCS) $(ALL_H_SRCS)
	@echo "formatted $(words $(ALL_C_SRCS) $(ALL_H_SRCS)) files"
endif

format-check:
ifeq ($(CLANG_FORMAT),)
	@echo "SKIP format-check: clang-format not installed."
else
	$(CLANG_FORMAT) --dry-run --Werror $(ALL_C_SRCS) $(ALL_H_SRCS)
	@echo "format ok"
endif

lint:
ifeq ($(CPPCHECK),)
	@echo "SKIP lint: cppcheck not installed (pacman -S mingw-w64-ucrt-x86_64-cppcheck)."
else
	# unusedFunction is left to the nightly `--enable=all` pass: a library of registry and
	# inspector helpers legitimately has entry points that any single analysed set does not
	# call, and a gate that cries wolf gets ignored. Anything specific can be silenced with an
	# inline suppression comment.
	$(CPPCHECK) --quiet --error-exitcode=1 --std=c11 --language=c \
	    --enable=warning,style,performance,portability \
	    --inline-suppr --suppress=missingIncludeSystem \
	    --suppress='*:third_party/*' \
	    -I src -I third_party/raygui $(RAYLIB_CFLAGS) \
	    src tests
endif

analyze:
ifeq ($(CLANG),)
	@echo "SKIP analyze: clang not installed (pacman -S mingw-w64-ucrt-x86_64-clang)."
else
	@for f in $(ANALYZE_SRCS); do \
	    echo "  analyze $$f"; \
	    $(CLANG) --analyze -Xanalyzer -analyzer-output=text $(CSTD) $(INCLUDES) \
	        $(RAYLIB_CFLAGS) -DDRIFTY_HEADLESS $$f -o /dev/null || exit 1; \
	done
	@echo "clang --analyze clean"
endif

verify-fast: format-check test
	@echo "verify-fast: ok"

verify: format-check lint analyze test-physics regression
	@echo "verify: ok"

# ------------------------------------------------------------------------- sanitizers --

sanitize:
ifeq ($(CLANG),)
ifeq ($(DRIFTY_HOST),ucrt64)
	@echo "SKIP sanitize: clang not installed (pacman -S mingw-w64-ucrt-x86_64-clang)." >&2
else
	@echo "SKIP sanitize: clang not installed. Install it with the platform package manager." >&2
endif
else
	@mkdir -p $(BUILD_SANITIZE)
ifeq ($(DRIFTY_HOST),ucrt64)
	@printf 'int main(void) { return 0; }\n' > $(BUILD_SANITIZE)/runtime_probe.c
	@if ! $(CLANG) -fsanitize=address,undefined \
	        $(BUILD_SANITIZE)/runtime_probe.c \
	        -o $(BUILD_SANITIZE)/runtime_probe$(EXE_SUFFIX) >/dev/null 2>&1; then \
	    rm -f $(BUILD_SANITIZE)/runtime_probe.c $(BUILD_SANITIZE)/runtime_probe$(EXE_SUFFIX); \
	    echo "SKIP sanitize: this clang installation has no linkable ASan/UBSan runtime." >&2; \
	    echo "MSYS2 provides those runtimes in CLANG64, not the supported UCRT64 environment." >&2; \
	else \
	    rm -f $(BUILD_SANITIZE)/runtime_probe.c $(BUILD_SANITIZE)/runtime_probe$(EXE_SUFFIX); \
	    $(CLANG) $(CSTD) $(INCLUDES) -O1 -g -fsanitize=address,undefined \
	        -fno-omit-frame-pointer -fno-sanitize-recover=all -DDRIFTY_HEADLESS \
	        $(BUILD_DEFINES) -DDRIFTY_BUILD_MODE=\"sanitize\" \
	        -DDRIFTY_BUILD_FLAGS=\"-O1,-g,-fsanitize=address+undefined\" \
	        $(TEST_SRCS) -o $(BUILD_SANITIZE)/drifty_tests_asan$(EXE_SUFFIX) $(RAYLIB_CFLAGS) -lm && \
	    ./$(BUILD_SANITIZE)/drifty_tests_asan$(EXE_SUFFIX); \
	fi
else
	$(CLANG) $(CSTD) $(INCLUDES) -O1 -g -fsanitize=address,undefined \
	    -fno-omit-frame-pointer -fno-sanitize-recover=all -DDRIFTY_HEADLESS \
	    $(BUILD_DEFINES) -DDRIFTY_BUILD_MODE=\"sanitize\" \
	    -DDRIFTY_BUILD_FLAGS=\"-O1,-g,-fsanitize=address+undefined\" \
	    $(TEST_SRCS) -o $(BUILD_SANITIZE)/drifty_tests_asan$(EXE_SUFFIX) $(RAYLIB_CFLAGS) -lm
	./$(BUILD_SANITIZE)/drifty_tests_asan$(EXE_SUFFIX)
endif
endif

# -------------------------------------------------------------------------- coverage --
#
# gcov writes the .gcno note file beside the OBJECT it compiled, and records that same
# directory inside the object so the .gcda lands there at run time. A single compile-and-link
# command therefore sprays both across the repository root — 43 files, measured.
#
# -fprofile-dir does NOT fix it: it only redirects the runtime .gcda, and it mangles the
# absolute path into a `build/coverage/C~...` directory name. So compile to real objects inside
# build/coverage instead, which puts notes and data where they belong by construction.

coverage:
	@mkdir -p $(BUILD_COVERAGE)
	@set -e; \
	objects=""; \
	for src in $(TEST_SRCS); do \
	    object="$(BUILD_COVERAGE)/$$(echo $$src | tr '/' '_' | sed 's/\.c$$/.o/')"; \
	    $(CC) $(CSTD) $(INCLUDES) -O0 -g --coverage -DDRIFTY_HEADLESS \
	        $(BUILD_DEFINES) -DDRIFTY_BUILD_MODE=\"coverage\" \
	        -DDRIFTY_BUILD_FLAGS=\"-O0,--coverage\" \
	        $(RAYLIB_CFLAGS) -c $$src -o $$object; \
	    objects="$$objects $$object"; \
	done; \
	$(CC) --coverage $$objects -o $(BUILD_COVERAGE)/drifty_tests_cov$(EXE_SUFFIX) -lm
	./$(BUILD_COVERAGE)/drifty_tests_cov$(EXE_SUFFIX)
ifeq ($(GCOVR),)
	@echo "SKIP gcovr report: gcovr not installed (pacman -S mingw-w64-ucrt-x86_64-gcovr). Raw .gcda files kept."
else
	$(GCOVR) --root . --filter 'src/.*' --exclude 'src/dev/dev_lab.c' \
	    --txt --html-details $(BUILD_COVERAGE)/index.html \
	    --cobertura $(BUILD_COVERAGE)/cobertura.xml \
	    --print-summary
	@echo "$(BUILD_COVERAGE)/index.html written"
endif

# ------------------------------------------------------------------- visual regression --

screenshots: windows-only dev
	@mkdir -p $(ARTIFACTS)/screenshots
	@for scene in $(SCENES); do \
	    ./$(EXE_DEBUG) --capture-scene $$scene --width 1280 --height 720 --seed 12345 \
	        --output $(ARTIFACTS)/screenshots/$$scene.png || exit 1; \
	done

# The in-game vehicle gallery: every corpus page through the production texture path.
#
# Bounded and self-exiting like every other capture here — `drifty.exe --gallery-page N`
# draws one page and quits, so this target never launches an interactive session.
#
# The output is a HUMAN-REVIEW artifact and deliberately not a GPU regression baseline: a
# hundred cars behind an RMSE gate, on hardware that rasterizes differently per vendor, is a
# maintenance sinkhole with no regression value. The checks that matter are headless — the
# `corpus` scenario and `drifty_tests --dump-corpus-sheet`.
GALLERY_PAGES ?= 7

gallery: windows-only dev
	@mkdir -p $(ARTIFACTS)/gallery-ingame
	@page=1; \
	while [ $$page -le $(GALLERY_PAGES) ]; do \
	    ./$(EXE_DEBUG) --gallery-page $$page --width 1280 --height 720 \
	        --output $(ARTIFACTS)/gallery-ingame/page_$$page.png || exit 1; \
	    page=$$((page + 1)); \
	done
	@echo "gallery: $(GALLERY_PAGES) page(s) in $(ARTIFACTS)/gallery-ingame/"

# ------------------------------------------------------- appearance debugging tools --
#
# `cards` is headless and cheap: per-car PNGs, feature-label maps and cards.json. Everything
# below reads it, and nothing below needs a window or a GPU.
cards: tests
	@./$(EXE_TESTS) --dump-corpus-cards $(ARTIFACTS)/corpus-cards

# The browser inspector serves tools/visual over artifacts/corpus-cards and blocks until the
# operator stops it. `visual-diagnose` is the bounded automation path.
inspect: cards
	@echo "Serving the vehicle inspector. This returns only after you stop it."
	@cd tools/visual && node serve.js

# Bounded automation: runs the Playwright suite, starts and stops its own server, and writes
# per-car cards, label maps, sweep strips, and diagnostics.txt under artifacts/visual/.
# It remains diagnostic rather than a pass/fail gate; `|| true` preserves all evidence when
# a measurement fails so the report can identify every defect in one run.
visual-diagnose: cards
	@cd tools/visual && npm install --silent && npx playwright test --reporter=list || true
	@echo "visual-diagnose: evidence in $(ARTIFACTS)/visual/ (start with diagnostics.txt)"

visual-test: screenshots
ifeq ($(MAGICK),)
	@echo "SKIP visual-test: ImageMagick not installed (winget install ImageMagick.ImageMagick)."
else
	@mkdir -p $(ARTIFACTS)/visual-diff
	@status=0; \
	for scene in $(SCENES); do \
	    base=tests/visual/baseline/$$scene.png; \
	    current=$(ARTIFACTS)/screenshots/$$scene.png; \
	    if [ ! -f $$base ]; then \
	        echo "  no baseline for $$scene (cp $$current $$base to accept it)"; \
	        continue; \
	    fi; \
	    rmse=$$("$(MAGICK)" compare -metric RMSE $$base $$current \
	        $(ARTIFACTS)/visual-diff/$$scene.png 2>&1 | sed 's/.*(\(.*\))/\1/'); \
	    echo "  $$scene RMSE $$rmse"; \
	    awk -v v="$$rmse" 'BEGIN { exit (v+0 > 0.02) ? 1 : 0 }' || \
	        { echo "  ! $$scene differs beyond tolerance"; status=1; }; \
	done; \
	exit $$status
endif

# ----------------------------------------------------------------------------- fuzzing --

fuzz:
ifeq ($(CLANG),)
	@echo "SKIP fuzz: clang with libFuzzer not installed." >&2
else
	@mkdir -p $(BUILD_FUZZ) $(ARTIFACTS)/fuzz/crashes
	@for target in fuzz/fuzz_*.c; do \
	    name=$$(basename $$target .c); \
	    echo "  building $$name"; \
	    mkdir -p $(ARTIFACTS)/fuzz/corpus/$$name; \
	    $(CLANG) $(CSTD) $(INCLUDES) -O1 -g -DDRIFTY_HEADLESS \
	        -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all \
	        $$target $(FUZZ_SUPPORT_SRCS) \
	        -o $(BUILD_FUZZ)/$$name $(RAYLIB_CFLAGS) -lm || exit 1; \
	    $(BUILD_FUZZ)/$$name $(ARTIFACTS)/fuzz/corpus/$$name \
	        -max_total_time=$(FUZZ_SECONDS) \
	        -artifact_prefix=$(ARTIFACTS)/fuzz/crashes/ || exit 1; \
	done
endif
FUZZ_SECONDS ?= 20

# ---------------------------------------------------------------------------- profiling --

# Tracy is opt-in and vendored by the developer: drop the distribution in third_party/tracy
# and this target compiles its client in. Without it the built-in zone timers are used, which
# need no dependency at all and print a summary at exit.
profile: windows-only
	@if [ -f third_party/tracy/public/TracyClient.cpp ]; then \
	    echo "Building with Tracy (third_party/tracy found)."; \
	    DRIFTY_EXTRA_DEFINES=-DDRIFTY_TRACY ./build.sh; \
	else \
	    echo "third_party/tracy not present — building with the built-in zone timers."; \
	    DRIFTY_EXTRA_DEFINES=-DDRIFTY_PROFILE ./build.sh; \
	fi

# ------------------------------------------------------------------- aggregate check --

ci: format-check lint analyze test-physics regression sanitize coverage
	@echo ""
	@echo "==============================================="
	@echo "ci: core local checks passed; inspect any SKIP lines."

# ---------------------------------------------------------------------- editor support --

compile-commands:
	$(PYTHON) tools/build/gen_compile_commands.py --output compile_commands.json \
	    --raylib-cflags "$(RAYLIB_CFLAGS)"

# -------------------------------------------------------------------------- housekeeping --

# `rm -rf build` covers every current output. Everything after it removes the LEGACY
# root-level layout, so a tree that was last built before the build/ consolidation cannot keep
# a stale runnable executable or a stale coverage file beside the new ones.
clean:
	rm -rf $(BUILD_DIR) coverage dist replays corpus
	rm -rf $(ARTIFACTS)/fuzz $(ARTIFACTS)/plots $(ARTIFACTS)/screenshots
	rm -f drifty$(EXE_SUFFIX) drifty_release$(EXE_SUFFIX) drifty_tests$(EXE_SUFFIX)
	rm -f drifty_hotreload_harness$(EXE_SUFFIX)
	rm -f drifty_tests_asan$(EXE_SUFFIX) drifty_tests_cov$(EXE_SUFFIX)
	rm -f libraylib.dll raylib.dll glfw3.dll
	rm -f *.gcda *.gcno *.gcov
	rm -f mk_verify*.log
	rm -f $(ARTIFACTS)/screenshots/phase1_smoke.png $(ARTIFACTS)/screenshots/phase2_smoke.png
	rm -f $(ARTIFACTS)/screenshots/phase3_smoke.png
	rm -rf $(ARTIFACTS)/telemetry $(ARTIFACTS)/replays
	# Legacy root telemetry output from before the artifacts/ consolidation.
	rm -rf telemetry
	rm -f *.o src/*.o tests/*.o *.pdb *.d *.ilk *.exp *.map

clean-telemetry:
	rm -f $(TELEMETRY)/*.csv $(TELEMETRY)/*.png
