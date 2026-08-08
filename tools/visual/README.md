# Vehicle appearance inspector

A browser view over the vehicle corpus, plus a Playwright suite that measures it. This is a
**diagnostic**, not a gate: it produces evidence for a human to read, and does not fail a build.

## Not to be confused with `tests/visual/`

Two directories with "visual" in the name, doing different jobs:

| | What it looks at | How it decides | Gate? |
| --- | --- | --- | --- |
| `tests/visual/` | Rendered game scenes from `drifty.exe --capture-scene` | ImageMagick RMSE against committed PNG baselines | Yes — `make visual-test` passes or fails |
| `tools/visual/` (here) | The appearance grammar: per-car sprites, feature-label maps, parameter sweeps | Human reading, plus Playwright measurements | No — evidence only |

The overlap is that both end up looking at pictures of cars. The difference is that
`tests/visual/` asks "did the rendered frame change?" and this asks "does the grammar produce
distinct, correct-looking vehicles across the corpus?".

See [tests/visual/README.md](../../tests/visual/README.md) for the regression gate.

## Running it

Everything starts from `make cards`, which is headless and cheap — it writes per-car PNGs,
feature-label maps, and `cards.json` to `artifacts/corpus-cards/` with no window and no GPU.

```bash
make inspect
```

Serves `inspector.html` over the cards and blocks until you stop it. This is the interactive
path: browse the corpus, compare vehicles, look at label maps.

```bash
make visual-diagnose
```

The bounded path. Runs the Playwright suite, starts and stops its own server, and writes
cards, label maps, sweep strips, and `diagnostics.txt` under `artifacts/visual/`. It ends with
`|| true` on purpose: a failed measurement should still leave every other piece of evidence in
place, so one run can identify every defect rather than stopping at the first.

Start with `artifacts/visual/diagnostics.txt`.

## Layout

```
inspector.html        the browser view
serve.js              static server for it; no dependencies beyond node
playwright.config.js  suite configuration
tests/capture.spec.js  per-car captures and sweep strips
tests/grammar.spec.js  measurements against the appearance grammar
```

## Prerequisites

Node and npm, which are **not** installed by `tools/setup/setup_windows.ps1` — this is the
only part of the toolchain that needs them. `make visual-diagnose` runs `npm install` itself;
`make inspect` assumes it has already been run once.

`node_modules/`, `playwright-report/`, and `test-results/` are gitignored. Everything else
here is source and is tracked.
