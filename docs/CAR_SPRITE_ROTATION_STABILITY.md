# Measuring pixel-art rotation stability

The production renderer rotates low-resolution, point-filtered body and wheel textures to arbitrary headings. That can cause apparent area changes, one-pixel edge shimmer, or pivot drift even when the source sprite itself is deterministic.

`scripts/measure_sprite_rotation.py` provides a dependency-free first-pass diagnostic. It imports the PNG decoder from `scripts/compare_car_rgba.py`, rotates the source alpha mask through a full turn with inverse-mapped point sampling, and reports:

- minimum and maximum occupied pixel counts;
- occupied-area span relative to the source area;
- maximum alpha-centroid drift from the destination pivot;
- mean and maximum alpha-mask XOR between adjacent headings.

## Usage

Generate one corpus card and run:

```text
build/tests/drifty_tests.exe --dump-corpus-cards artifacts/cars
python scripts/measure_sprite_rotation.py \
  artifacts/cars/archetype_00_stock_baseline.png \
  --steps 128 \
  --json
```

By default, the pivot is the alpha-mask centroid. Supply the documented sprite pivot when testing the production body texture:

```text
python scripts/measure_sprite_rotation.py CAR.png \
  --pivot-x 41.5 --pivot-y 16.0 \
  --steps 128
```

Optional thresholds turn the measurement into a gate:

```text
python scripts/measure_sprite_rotation.py CAR.png \
  --max-occupied-span-ratio 0.08 \
  --max-centroid-drift-px 0.75 \
  --max-adjacent-xor-ratio 0.35
```

The command returns `0` when thresholds pass, `1` when a threshold fails, and `2` for input errors.

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
