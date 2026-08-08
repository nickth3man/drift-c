# Visual regression

> Not the same thing as [`tools/visual/`](../../tools/visual/README.md). This directory is the
> **gate**: does a rendered game scene still match its committed baseline? That one is the
> **inspector**: does the appearance grammar produce distinct, correct-looking vehicles across
> the corpus? Both look at pictures of cars; only this one passes or fails.

`make screenshots` captures the deterministic scenes listed in `src/platform/main.c` and writes them
to `artifacts/screenshots/`. `make visual-test` compares those against the images in
`baseline/` with ImageMagick and a small RMSE tolerance, writing difference images to
`artifacts/visual-diff/`.

```bash
mk visual-test
```

## Why the captures are reproducible

`drifty.exe --capture-scene NAME` does not run the normal frame loop. It steps the simulation
with an exact fixed dt, draws one frame with interpolation alpha 0, writes a PNG, and exits.
The image therefore depends on the scene, the tick count, and the build — not on how fast the
machine happened to be that day.

## Why this gate is local, not CI

The comparison is only meaningful on one pinned environment. raylib renders through OpenGL,
and rasterisation differs between GPU vendors and drivers enough to move an RMSE well past
any tolerance worth setting. So this runs on the developer's machine, alongside the checks
that *are* portable — the headless physics, the telemetry comparison, the sanitizers.

## Accepting a new baseline

There is no automatic update. Look at the difference image first, then:

```bash
cp artifacts/screenshots/physics_lab.png tests/visual/baseline/physics_lab.png
```

A pull request that changes a baseline says, in words, what visual change it is accepting.
