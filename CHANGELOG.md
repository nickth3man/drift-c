# Changelog

Notable changes to Drifty. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

The project is pre-release and has no version tags yet, so entries are grouped under
`Unreleased` until the first one is cut.

## [Unreleased]

### Added

- MIT `LICENSE`. The repository previously carried none, which left it all-rights-reserved
  while vendoring zlib-licensed raygui and public-domain/MIT stb_image_write.
- Continuous integration (`.github/workflows/ci.yml`): a Windows UCRT64 job for the canonical
  toolchain and a Linux job for the headless targets and the sanitizers.
- `DRIFTY_STRICT=1`, which turns a missing tool from a `SKIP` into a build failure. Without
  it `make ci` could exit 0 having run almost nothing.
- Python linting and formatting with ruff (`pyproject.toml`, `requirements-dev.txt`), plus
  `make format-py` and `make lint-py`.
- `pre-commit` hooks for clang-format and ruff, pinned to the versions the project builds
  with.
- `.editorconfig` and `.gitattributes`. Line endings are now a repository property rather
  than a per-developer `core.autocrlf` setting.
- `CONTRIBUTING.md`, covering setup, what must be green before pushing, the hot-reload rules,
  and when re-recording baselines is legitimate.

### Changed

- `make analyze` no longer reports "clang --analyze clean" while emitting warnings. clang
  exits 0 on analyzer findings, so the old message reported a pass over 12 real ones. It now
  prints the count and writes `build/analyze.log`.
- `src/platform/hotreload.h` and `src/render/car_visual.h` reformatted. Their formatting was
  only valid on a CRLF worktree, because clang-format counts the `\r` toward `ColumnLimit`.
- `CLAUDE.md` is now a pointer to `AGENTS.md` instead of a byte-identical copy of it.
- The Milestone 1 plan moved from `PLAN.md` at the root into `docs/PLAN.md`, and is indexed by
  `docs/README.md`. At the root it was linked from nothing, so the one document describing where
  the project is going was reachable only by noticing it in a directory listing.
- `docs/README.md` no longer describes a `design/` + `notes/` split. `notes/` never existed, so
  the index referred to a missing directory three times and told contributors to file documents
  into it. The split it was reaching for — current contracts versus historical notes — is now
  stated as a tense distinction between `design/` and `PLAN.md`.
- The `## Layout` map in `README.md` was rebuilt from the tree and grouped by directory. It had
  drifted badly: 18 source files were missing (including `ai_driver`, `car_roster`,
  `run_report`, `validation_metrics`, `vehicle_effects`, and the whole `render.c` split), and it
  still pointed at `tools/*.py`, which had moved into six subdirectories. It also quoted a
  scenario count that was four short; the count is no longer quoted, on the same reasoning the
  check count already wasn't.

### Removed

- Four unreferenced scripts: `test_boundary_collision.py` and `test_sustained_right_drive.py`
  (Win32 key-injection one-offs named like tests but in no suite), and
  `run_gameplay_recording.py` / `query_gameplay_review.py` (both bound to the superseded
  `recording_output/`). Dropping the first of these removed the project's only non-stdlib
  Python dependency.

## Earlier work

Before this changelog existed, milestones were tracked in commits and pull requests:

- Milestone 1 Phase 3 — drift scoring removed ([#35](https://github.com/nickth3man/drift-c/pull/35), 2026-08-07)
- Milestone 1 Phase 2 — physical drivetrain layout and the six-car roster ([#34](https://github.com/nickth3man/drift-c/pull/34), 2026-08-07)
- Milestone 1 Phase 1 — chicane track, AI driver prototype, checkpoint fix ([#33](https://github.com/nickth3man/drift-c/pull/33), 2026-08-07)
- Review findings from the merged scaffolds closed ([#32](https://github.com/nickth3man/drift-c/pull/32), 2026-08-06)
