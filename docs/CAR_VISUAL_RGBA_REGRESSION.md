# RGBA regression checks for procedural cars

The existing appearance scenarios correctly test semantic label differences and geometry signatures. They do not answer a separate review question: did the actual displayed colors, alpha occupancy, or silhouette pixels change?

`scripts/compare_car_rgba.py` compares two 8-bit RGB or RGBA PNGs using only the Python standard library. It validates PNG CRCs, decodes all five PNG row filters, and reports:

- ratio of pixels whose RGBA value changed;
- ratio of occupied pixels that entered or left the alpha mask;
- RGB and RGBA root-mean-square error;
- maximum single-channel delta.

## Generate comparable artifacts

From the baseline revision:

```text
build/tests/drifty_tests.exe --dump-corpus-cards artifacts/baseline-cars
```

From the candidate revision:

```text
build/tests/drifty_tests.exe --dump-corpus-cards artifacts/candidate-cars
```

Each directory contains one rendered PNG and one label-map PNG per corpus vehicle. Compare a focused vehicle exactly:

```text
python scripts/compare_car_rgba.py \
  artifacts/baseline-cars/archetype_00_stock_baseline.png \
  artifacts/candidate-cars/archetype_00_stock_baseline.png
```

The default thresholds are all zero, so the command is an exact regression gate.

For an intentionally palette-only change, allow color error while keeping silhouette occupancy exact:

```text
python scripts/compare_car_rgba.py BASELINE.png CANDIDATE.png \
  --max-alpha-xor-ratio 0 \
  --max-differing-ratio 1 \
  --max-rgb-rmse 80 \
  --max-rgba-rmse 80 \
  --max-channel-delta 255
```

Machine-readable output is available with `--json`.

Every reported metric has a matching threshold flag, and all default to zero:

| Metric | Flag | Measures |
|---|---|---|
| `differing_ratio` | `--max-differing-ratio` | fraction of pixels whose RGBA differs at all |
| `alpha_xor_ratio` | `--max-alpha-xor-ratio` | silhouette occupancy change, over the alpha union |
| `rgb_rmse` | `--max-rgb-rmse` | colour error ignoring alpha |
| `rgba_rmse` | `--max-rgba-rmse` | colour error including alpha |
| `max_channel_delta` | `--max-channel-delta` | worst single-channel difference |

Prefer `--max-rgb-rmse` for palette work, where alpha is expected to be exact and folding it
into the average would dilute the signal. `--max-rgba-rmse` is the stricter one to reach for
when partial transparency is itself under review.

## Intended CI integration

Do not immediately freeze the entire corpus behind one undocumented tolerance. Add baselines and thresholds per change class:

1. **Pure refactor:** exact RGBA equality and exact alpha equality.
2. **Palette change:** exact alpha equality; reviewed RGB thresholds.
3. **Geometry fix:** reviewed alpha-XOR threshold plus the existing semantic and signature gates.
4. **New feature:** compare only the affected corpus entries until the visual contract is approved.

A checked-in baseline should record the generating commit and `pxPerM`. Regenerate it only in the same PR that intentionally changes the approved output.

The comparator returns:

- `0` when every threshold passes;
- `1` when the images decode but a threshold fails;
- `2` for input, format, or dimension errors.

## Supported PNG format

The tool supports non-interlaced, 8-bit RGB and RGBA PNGs. That matches the headless corpus artifacts. Other bit depths and indexed/interlaced PNGs fail explicitly rather than being compared incorrectly.
