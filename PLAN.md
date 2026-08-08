# Closing the Drifty project-setup gaps

## Context

An assessment of `drift-c` against comparable C/raylib projects (`raysan5/raylib-game-template`, the
hot-reload templates) and serious C codebases (neovim, DevilutionX, aseprite, curl) found that the
**code and build system are above peer median** — the `SOURCE_GROUP_NAMES` manifest in
[Makefile](Makefile), the telemetry-baseline regression harness, the fuzz targets, and the `src/`
subsystem layout are all better engineered than the raylib ecosystem norm. Dead code in `src/` is
essentially nil (5 of 251 header-declared functions unreferenced, 4 of them legitimate Tracy stubs).

The gaps are all at the **perimeter**: nothing is enforced, nothing is licensed, and 40% of the tool
surface (Python, Node) has no linting, formatting, or declared dependencies. Ten gaps were found:

1. No CI — `.github/` has templates, zero workflows; `make ci` prints `SKIP` and exits 0
2. No LICENSE — all-rights-reserved despite vendoring zlib/MIT third-party code
3. Root clutter — 4 build entry points, byte-identical `CLAUDE.md`/`AGENTS.md`, editor debris
4. ~1.35 GB of unreaped run evidence in the working tree
5. `scripts/` vs `tools/` incoherent — 5 of 9 entries orphaned
6. Python: 3,671 lines, zero config, undeclared `numpy`/`PIL` deps
7. No clang-tidy
8. No `.editorconfig`, no pre-commit, no CONTRIBUTING/CHANGELOG
9. `docs/` — 9 files, all orphaned from the README, live specs mixed with dead proposals
10. Visual testing split across `tests/visual/` and `tools/visual/` with no cross-reference

**Outcome:** every check that exists today runs automatically and fails loudly; every tool is
invocable from the build; the working tree is navigable; the repo is legally forkable.

**Decisions made:** MIT license · CI on Windows UCRT64 + Linux · delete dead code and stale docs ·
deliver as three tiered PRs.

## Guardrails — explicitly not doing

- **No CMake migration.** The hot-reload link sequence, 7 output configurations, and Tracy build are
  things CMake makes harder, and the source manifest already solves the duplication CMake would be
  brought in for.
- **No test-framework replacement.** 12.8k lines of scenario tests stay on the bespoke runner.
- **No `src/` reorganization.** The layout is already correct.
- **No `tools/telemetry/` packaging refactor.** The `sys.path` + `# noqa: E402` idiom is correct for
  scripts invoked by path; converting to a package would churn the Makefile for no behavior change.

---

## PR 1 — Enforcement and hygiene

Closes gaps 1, 2, 6, 8. Nothing here changes program behavior.

### 1.1 `LICENSE` (MIT)

Standard MIT text, `Copyright (c) 2026 Nicolas Alexander`. Add a **Third-party** section to
[README.md](README.md) pointing at [third_party/README.md](third_party/README.md), which already
carries exemplary provenance records (upstream URL, sha256, license) for raygui (zlib) and
stb_image_write (public domain / MIT). Do not restate the licenses — link them.

### 1.2 Make `SKIP` fail in CI — the prerequisite for every other check

`format-check`, `lint`, `analyze`, `sanitize`, `coverage`, `fuzz`, and the ImageMagick-gated
`screenshots`/`visual-test` targets all print `SKIP` and **exit 0** when a tool is missing.
[Makefile:595](Makefile) `ci:` therefore can pass having run almost nothing.

Add one helper near the optional-tool block (`Makefile` ~line 95, beside `CLANG`/`CPPCHECK`/`GCOVR`/`MAGICK`):

```make
# A missing tool is advisory locally and fatal in CI, so `make ci` cannot pass by skipping.
ifdef DRIFTY_STRICT
skip = @echo "MISSING $(1)" >&2; exit 1
else
skip = @echo "SKIP $(1)" >&2
endif
```

Replace each `@echo "SKIP ..."` line with `$(call skip,...)`. Keep the existing install-hint text —
it is good. The UCRT64 ASan/UBSan-runtime probe inside `sanitize:` ([Makefile:414](Makefile)) keeps
its own SKIP unconditionally: that is a documented platform limitation, not a missing tool, and CI
covers sanitizers on the Linux job instead.

### 1.3 `.github/workflows/ci.yml`

Two jobs, on push and pull_request. Pin actions by SHA (as curl and crystal-lang do).

**`windows-ucrt64`** — the canonical environment, via `msys2/setup-msys2@v2` with `msystem: UCRT64`,
`pacboy: raylib:p clang:p clang-tools-extra:p cppcheck:p gcovr:p python:p`, `defaults.run.shell: msys2 {0}`.
Runs `make DRIFTY_STRICT=1 verify` (format-check + lint + analyze + test-physics + regression).

**`linux-headless`** — `ubuntu-latest`. The POSIX branch at [Makefile:78](Makefile) needs raylib
**headers only** and already falls back to `third_party/raylib-src/src`, so:

```bash
git clone --depth 1 --branch 6.0 https://github.com/raysan5/raylib third_party/raylib-src
```

No raylib build. Then `apt-get install clang clang-format cppcheck gcovr` and
`make DRIFTY_STRICT=1 test-physics regression sanitize`. This job is where ASan/UBSan actually run.

### 1.4 `.editorconfig`

Mirror [.clang-format](.clang-format) so editors agree before the formatter runs: `indent_style = space`,
`indent_size = 4`, `max_line_length = 96`, `end_of_line = lf`, `insert_final_newline = true`,
`trim_trailing_whitespace = true`. Override `indent_size = 2` for `*.{yml,yaml,json,js}` and
`*.bat` → `end_of_line = crlf`.

### 1.5 `pyproject.toml` + ruff

One file at root covering all 16 Python files:

- `[project]` — `requires-python = ">=3.9"`, dependencies `numpy`, `pillow` (both currently imported
  and undeclared — `tools/validation/learn_racing_line.py` and `scripts/run_gameplay_recording.py`
  cannot run on a fresh clone), plus `matplotlib` if `tools/telemetry/plot_telemetry.py` needs it
  (verify at implementation time; it may be pure-stdlib like the PNG decoder in `compare_car_rgba.py`).
- `[tool.ruff]` — `line-length = 96` matching the C style; `select = ["E", "F", "W", "I", "B"]`.
  **Deliberately excluding `UP`** for now: the codebase uses `from __future__ import annotations` with
  `typing.Dict`/`List`/`Optional` consistently, and modernizing that is a separate, reviewable change.
- `per-file-ignores` for `E402` in `tools/telemetry/*` (the intentional `sys.path` prelude).

Add `make format-py` / `make lint-py`, fold `lint-py` into `verify` and both CI jobs.

### 1.6 `.pre-commit-config.yaml`

`pre-commit/mirrors-clang-format` (pinned rev, `files: \.(c|h)$`) + `astral-sh/ruff-pre-commit`
(lint + format) + `pre-commit-hooks` basics (trailing-whitespace, end-of-file-fixer,
check-merge-conflict). This is the pattern godot, aseprite, FreeCAD, pandas, and Arrow all use, and
it fixes format failures at commit time rather than in review.

**Sequencing risk:** [.clang-format](.clang-format) carries an ADOPTION NOTE warning that
`make format` over the existing tree still produces a diff from hand-aligned tables. `clang-format`
is not on PATH outside MSYS2 so I could not verify the current state. **First implementation step is
`make format-check` inside UCRT64.** If it fails, land the normalization as its own reviewable commit
*before* the pre-commit hook and before CI turns format-check blocking.

### 1.7 `CONTRIBUTING.md`, `CHANGELOG.md`, root de-duplication

- `CONTRIBUTING.md` — extract the workflow content already buried in [README.md](README.md)
  (prerequisites, `make verify` before pushing, the reload-safety rules, when baselines may be
  re-recorded and why that needs explaining).
- `CHANGELOG.md` — Keep a Changelog format, seeded from the existing milestone commits
  (`b7d1636` racing line, `3ee8a53` scoring removal, `f2d0162` drivetrain, `35b8d3b` chicane track).
- `CLAUDE.md` and `AGENTS.md` are **byte-identical** (verified). Keep `AGENTS.md` as the real file;
  replace `CLAUDE.md` with a one-line pointer.
- Move `/PowerShellEditorServices.json` out of `.git/info/exclude` (a local-only ignore that does not
  travel to a fresh clone) into [.gitignore](.gitignore). Add `.cache/`. Remove the duplicated
  `.slim/deepwork/` line.

---

## PR 2 — Structure and cleanup

Closes gaps 3, 4, 5, 9, 10.

### 2.1 Fold `scripts/` into `tools/`

`tools/` is organized by domain and Makefile-wired; `scripts/` is a junk drawer where 5 of 9 entries
are referenced by nothing.

| Current | Action |
|---|---|
| `scripts/setup_windows.ps1` | → `tools/setup/setup_windows.ps1`; update the 5 referrers (Makefile, build.sh, build.bat, mk.bat, README) |
| `scripts/validate_hotreload.sh` | → `tools/setup/validate_hotreload.sh`; add `make validate-hotreload` |
| `scripts/compare_car_rgba.py` | → `tools/appearance/`; add `make compare-rgba`. **Keep** — documented by `CAR_VISUAL_RGBA_REGRESSION.md` |
| `scripts/measure_sprite_rotation.py` | → `tools/appearance/`; add `make measure-rotation`. **Keep** — documented by `CAR_SPRITE_ROTATION_STABILITY.md`; imports the PNG decoder from `compare_car_rgba.py`, so **these two must move together** |
| `scripts/test_boundary_collision.py` | **Delete** — near-identical Win32 `keybd_event` one-off, named like a test but in no suite |
| `scripts/test_sustained_right_drive.py` | **Delete** — same |
| `scripts/run_gameplay_recording.py` | **Delete** — writes to legacy `recording_output/` |
| `scripts/query_gameplay_review.py` | **Delete** — reads legacy `recording_output/` |
| `debug/record/record_gameplay.py` | → `tools/recording/record_gameplay.py`; add `make record`. **Keep** — the current recorder |

Rule after this: **every surviving tool has a Makefile target.** A tool with no entry point is a tool
nobody runs.

### 2.2 Artifact retention

`artifacts/` is 959 MB across 142 entries (116 are `failure-*-<timestamp>/` bundles); `debug/record/`
is 212 MB; `recording_output/` is 7.5 MB of superseded output. All correctly ignored — nothing is
ever reaped.

- Add `make clean-artifacts` keeping the N newest `failure-*` bundles (default 10), and
  `make clean-artifacts KEEP=0` for a full purge. Model it on the existing `clean-telemetry:`
  target ([Makefile:627](Makefile)).
- Delete outright — these are not run evidence: `artifacts/physics_tests.c.preexisting` (333 KB),
  `artifacts/reorganization-{list,suite}.{before,after}.txt`, `artifacts/_final_linkage.sh`,
  `artifacts/_organization_assert.ps1`, `artifacts/report_skidpad.html` (742 KB),
  `artifacts/junit.xml`.
- Delete `recording_output/` and drop its now-dead `.gitignore` entry.

### 2.3 `docs/` — index and triage

Every doc is unreachable from the README. Add `docs/README.md` as the index and link it from
[README.md](README.md). Split into `docs/design/` (describes current behavior) and `docs/notes/`
(historical, dated, marked resolved).

**Live → `docs/design/`:** `VALIDATION_SCHEMA.md` (the `run.json`/`suite.json` contract that
`tools/validation/run_suite.py` produces) · `RACING_LINE_OPTIMIZATION.md` (landed in `b7d1636`) ·
`CAR_SPRITE_ROTATION_STABILITY.md` and `CAR_VISUAL_RGBA_REGRESSION.md` (tool documentation for two
scripts being kept in 2.1).

**Delete:** `GAME_INTERACTION_RECORDING_AND_REVIEW.md` — a research report proposing five options,
none chosen, referencing a `scripts/analyze_trace.py` that has never existed and a
`recording_output/` being deleted. `ADVERSARIAL_CRITIQUE.md` — a point-in-time Phase 2 evaluation
whose entire evidence base is `recording_output/`.

**Triage individually — do not blanket-delete:** the three `Status: draft implementation contract`
docs (`CAR_APPEARANCE_IDENTITY_PROPOSAL.md`, `CAR_VISUAL_PRIMITIVES_PROPOSAL.md`,
`DYNAMIC_CAR_VISUAL_EFFECTS_PROPOSAL.md`). `src/render/car_visual.c`, `car_visual_raster.c`, and
`vehicle_effects.c` all exist, so these were probably implemented. For each: if implemented, update
the status line and move to `docs/design/`; if superseded, move to `docs/notes/` with a resolution
note. Deleting a spec that still describes shipped behavior is the one irreversible mistake here.

Trim [README.md](README.md) (17 KB) to overview + quickstart + pointers, moving the workflow prose to
`CONTRIBUTING.md` and the rest behind the docs index.

### 2.4 Visual testing cross-reference

`tests/visual/baseline/` (10 PNGs) and `tools/visual/` (Playwright, `serve.js`, specs) are a single
system in two trees. The split is defensible — baselines with tests, runner with tools — but is
documented in neither. Add a cross-reference paragraph to `tests/visual/README.md` and a
`tools/visual/README.md`, and note the `npm` prerequisite in `CONTRIBUTING.md`.

---

## PR 3 — Depth

Closes gap 7 and hardens 1.

### 3.1 `.clang-tidy`

The highest-value C linter and the one gap where every serious peer beats this repo (neovim runs
`WarningsAsErrors: '*'`). `compile_commands.json` is already generated by
[tools/build/gen_compile_commands.py](tools/build/gen_compile_commands.py) via `make compile-commands`,
so the input exists.

Start from the SerenityOS/aseprite exclusion sets: `bugprone-*, clang-analyzer-*, cert-*,
performance-*, readability-*` minus `-bugprone-easily-swappable-parameters`,
`-readability-magic-numbers`, `-readability-function-cognitive-complexity`,
`-readability-identifier-length`. `HeaderFilterRegex: '^src/|^tests/'`.

Add `make tidy` reusing `$(ANALYZE_SRCS)` — the existing set at [Makefile:186](Makefile) that already
excludes `main.c`, the hot-reload loader, `dev_lab.c` (mostly vendored raygui), the harness entry
point, and the fuzz drivers. **Gate on changed-files-only in CI** (`git diff --name-only origin/main`)
so 19.5k lines of pre-existing findings do not block every PR; a full-tree `make tidy` runs nightly.

### 3.2 `.github/workflows/nightly.yml`

Scheduled, non-blocking. Runs the things too slow or too noisy for PR CI, several of which the
Makefile already anticipates:

- `cppcheck --enable=all` including `unusedFunction` — the [Makefile:378](Makefile) `lint:` comment
  explicitly defers this to "the nightly `--enable=all` pass". This is where the deferral becomes real.
- Full-tree `make tidy`
- `make coverage`, uploading the Cobertura output
- `make fuzz` with a longer budget than the current brief run
- `make sanitize` on Linux

---

## Verification

Run in order; each PR is verifiable on its own.

**PR 1**
1. `make format-check` inside MSYS2 UCRT64 — must pass before pre-commit or CI become blocking. If it
   fails, land the normalization commit first (see 1.6).
2. `make DRIFTY_STRICT=1 verify` locally → passes with tools installed.
3. Temporarily rename `cppcheck` on PATH, re-run → must **fail** with `MISSING lint: ...`, not pass
   with `SKIP`. This is the whole point of 1.2; verify it directly.
4. `ruff check .` and `ruff format --check .` → clean.
5. `pre-commit run --all-files` → clean.
6. Push the branch; both CI jobs green. Confirm the Linux job's raylib clone succeeded and that
   `sanitize` actually ran there (not skipped).
7. `python -c "import numpy, PIL"` after `pip install -e .` → the fresh-clone path for
   `tools/validation/learn_racing_line.py` works.

**PR 2**
8. `make verify` still passes after the `scripts/` → `tools/` move — catches any missed referrer in
   Makefile, build.sh, build.bat, mk.bat, or `.vscode/tasks.json`.
9. `grep -rn "scripts/" Makefile build.sh build.bat mk.bat README.md docs/ .vscode/` → no hits for
   moved or deleted paths.
10. Each new Makefile target runs: `make validate-hotreload`, `make compare-rgba`,
    `make measure-rotation`, `make record`.
11. `make clean-artifacts KEEP=2` → exactly 2 `failure-*` bundles survive; `make test-physics
    regression` still passes afterward (confirms nothing under `artifacts/` was load-bearing).
12. `tests/baselines/` untouched — it is committed and must not be swept.
13. Every link in `docs/README.md` resolves.

**PR 3**
14. `make compile-commands && make tidy` completes; record the baseline finding count.
15. Open a trivial PR touching one `src/` file → the changed-files clang-tidy gate runs and passes.
16. Trigger `nightly.yml` via `workflow_dispatch` → completes, coverage artifact uploaded.

**Regression guard, all three PRs:** `make test-physics regression` must pass unchanged. No change in
this plan touches `src/` behavior, so any telemetry drift against `tests/baselines/` means something
was moved that shouldn't have been.
