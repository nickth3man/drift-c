# Measuring pixel-art rotation stability

The production renderer rotates low-resolution, point-filtered body and wheel textures to arbitrary headings. That can cause apparent area changes, one-pixel edge shimmer, or pivot drift even when the source sprite itself is deterministic.

`tools/appearance/measure_sprite_rotation.py` provides a dependency-free first-pass diagnostic. It imports the PNG decoder from `tools/appearance/compare_car_rgba.py`, rotates the source alpha mask through a full turn with inverse-mapped point sampling, and reports:

- minimum and maximum occupied pixel counts;
- occupied-area span relative to the source area;
- maximum alpha-centroid drift from the destination pivot;
- mean and maximum alpha-mask XOR between adjacent headings.

## Usage

Generate one corpus card and run:

```text
build/tests/drifty_tests.exe --dump-corpus-cards artifacts/cars
python tools/appearance/measure_sprite_rotation.py \
  artifacts/cars/archetype_00_stock_baseline.png \
  --steps 128 \
  --json
```

By default, the pivot is the alpha-mask centroid. Supply the documented sprite pivot when testing the production body texture:

```text
python tools/appearance/measure_sprite_rotation.py CAR.png \
  --pivot-x 41.5 --pivot-y 16.0 \
  --steps 128
```

Optional thresholds turn the measurement into a gate:

```text
python tools/appearance/measure_sprite_rotation.py CAR.png \
  --steps 128 \
  --max-occupied-span-ratio 0.08 \
  --max-centroid-drift-px 0.75 \
  --max-adjacent-xor-ratio 0.35
```

Always pass `--steps` explicitly when gating: the XOR threshold is only meaningful against a
fixed step count. See the reference measurements below.

The command returns `0` when thresholds pass, `1` when a threshold fails, and `2` for input errors.

## Reference measurements

Recorded at commit `8bd8805`, from cards written by `--dump-corpus-cards` at the default bake
scale, with `--steps 128` and the default centroid pivot. `--dump-corpus-cards` is deterministic,
and so is this tool, so these reproduce exactly:

| Corpus entry | Size | Occupied area | Span | Max drift | XOR mean | XOR max |
|---|---|---|---|---|---|---|
| `archetype_00_stock_baseline` | 35x63 | 1217..1234 | 1.39% | 0.537 px | 7.21% | 7.99% |
| `archetype_01_kei_car` | 31x51 | 821..834 | 1.57% | 0.576 px | 7.28% | 8.36% |
| `archetype_10_open_wheel` | 55x67 | 1245..1267 | 1.75% | 0.358 px | 10.46% | 10.96% |
| `archetype_14_bus` | 45x133 | 3833..3874 | 1.06% | 0.277 px | 10.37% | 11.28% |
| `archetype_16_box_truck` | 42x107 | 2931..2968 | 1.25% | 0.140 px | 9.24% | 9.82% |

Two properties of the metrics matter more than any single number here.

**Area span and centroid drift converge quickly across sample counts for these sprites.** For the
recorded corpus entries, the extrema happen to match across 32, 64, 128, and 256 steps. However,
because `measure_sprite_rotation.py` evaluates metrics over the discrete headings visited by
`range(steps)`, finer sampling can in general uncover new extrema. Always use a consistent
`--steps` parameter for all metric gates.

**Adjacent-heading XOR scales strongly with `--steps`.** It measures the change between
neighbouring headings, so it falls roughly by half each time the step count doubles — for
`archetype_01_kei_car`: 21.84% at 32 steps, 12.91% at 64, 7.28% at 128, 3.97% at 256. An XOR figure
is therefore meaningless without the step count beside it, and `--max-adjacent-xor-ratio` may
only be compared against runs that fix `--steps` to the same value. Always state the step count
whenever quoting any stability metric.

## How to use the result

1. Record measurements for representative small, long, wide, open-wheel, and commercial corpus entries at the current bake scale.
2. Capture the same headings through raylib and compare the CPU diagnostic with actual GPU output.
3. Establish thresholds only after the two agree closely enough for the intended decision.
4. Measure candidate strategies independently:
   - current single texture with point-filtered rotation;
   - 2x bake resolution;
   - a fixed heading atlas, such as 32 or 64 body orientations;
   - controlled downsampling after rotation.
5. Prefer the smallest strategy that materially reduces area variation and pivot drift without softening the approved pixel style.

## Limitations

This script models point sampling; it is not a replacement for a production capture. Exact edge choices can differ with raylib/OpenGL texture-coordinate conventions and target scaling. Its immediate value is reproducible relative measurement: the same source and algorithm can compare candidate bake resolutions and atlas sizes before changing runtime architecture.

The tool measures the alpha mask rather than color flicker. Use `compare_car_rgba.py` on captured frames when palette-edge changes also matter.
