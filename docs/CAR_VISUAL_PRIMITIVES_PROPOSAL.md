# Derived vehicle primitives

Status: draft implementation contract

## Problem

`car_visual.h` states that `car_visual.c` is the only place where a vehicle styling decision may live. The CPU rasterizer should convert already-derived geometry from metres to pixels and fill it.

The current rasterizer still invents several visible dimensions and placements, including:

- the rear shading band;
- lamp dimensions and placement;
- canard dimensions, placement, and angle;
- mirror dimensions;
- exhaust presentation bore and spacing;
- pickup-bed width and rail placement;
- hood-bulge dimensions;
- hood-pin placement;
- stripe dimensions and placement;
- fixed glass-width multipliers.

Some of those formulas also include `onePx`, which makes the effective geometry depend on raster scale. That weakens the documented separation between the metre-based grammar and the pixel rasterizer, and it leaves some visible decisions outside the diagnostic signature.

## Goal

Make every visible primitive a deterministic field of `CarVisual`. After this migration, `car_visual_raster.c` should make only these decisions:

1. metres-to-pixels conversion;
2. clipping;
3. scan conversion and alpha compositing;
4. fixed layer ordering;
5. pixel-quantization floors that cannot be represented in metres without changing output semantics.

The rasterizer must not derive a feature's position, dimensions, angle, count, or color from high-level weights.

## Proposed data model

Add a reusable oriented primitive:

```c
typedef struct {
    Vector2 centreM;
    float lengthM;
    float widthM;
    float angleRad;
} CarVisualRect;
```

Add already-derived fields to `CarVisual`:

```c
CarVisualRect rearShade;
CarVisualRect roofOpening;
CarVisualRect windscreen;
CarVisualRect backlight;
CarVisualRect sunroofRect;
CarVisualRect headlights[2];
CarVisualRect taillights[2];
CarVisualRect canards[2];
CarVisualRect mirrors[2];
CarVisualRect pickupBedFloor;
CarVisualRect pickupBedRails[2];
CarVisualRect hoodBulge;
CarVisualRect stripes[2];
Vector2 hoodPinCentreM[2];
float exhaustDrawBoreM;
float exhaustSpacingM;
```

Zero dimensions mean that a primitive is absent. Presence flags should remain only where they carry independent semantic meaning; a zero-sized primitive should otherwise be sufficient.

## Formula migration

The first implementation should preserve the current image wherever the old geometry is scale independent:

| Primitive | Current raster formula to move into `car_visual_derive()` |
|---|---|
| Rear shade | `length = 0.42 * lengthM`; center from `tailX`; width from hull station 1 |
| Lamps | `0.10 m x 0.26 m`; positions at the nose/tail endpoint and 55% of endpoint half-width |
| Mirrors | `0.14 m x 0.10 m`; X at windscreen station, Y at `mirrorOffsetM` |
| Exhaust | area-continuous bore scale and `1.8 * bore` spacing |
| Pickup bed | width `0.78 * widthM * pickupBedWeight`; center from tail and declared bed length |
| Hood bulge | center between nose and windscreen; current length and width rules |
| Hood pins | X `noseX - 0.25 m`; Y `0.16 * widthM` |
| Stripes | current length, width, and offset formulas |
| Backlight | current `1.88 * glassHalfWidthM` width |
| Sunroof | current `0.56 * roofWidthM` width |

The canard width currently adds `onePx`, which is raster-scale-dependent. Replace that term with a documented physical presentation floor. The initial floor should be `0.08 m`, approximately one world pixel at the canonical 13.2 px/m scale. The corpus and sensitivity tests must determine whether that floor needs adjustment.

## Bake key and signature

Every new field read by the rasterizer must be added to `car_visual_bake_key()` by name. The old upstream weights may remain in the key only if they still directly affect pixels.

The diagnostic signature should include independent visible geometry rather than both a driver and its resulting primitive. For example:

- keep canard length or width, not both `canardStrength` and an identical restatement;
- keep exhaust bore and count, not `exhaustTransition` if its only visible consequence is already represented;
- keep stripe width/offset if stripe geometry is intended to contribute to distinctness.

Append signature components; do not reorder existing slots.

## Tests

The implementation is complete only when all of these pass:

1. **Raster equivalence for migrated scale-independent primitives.** Before/after label and RGBA rasters are bit-identical for the full corpus, except for the canard physical-floor change.
2. **Scale independence.** Deriving `CarVisual` at different raster scales remains impossible by construction; no derived field reads `pxPerM` or `onePx`.
3. **Bake-key coverage.** Mutating any new primitive changes the bake key.
4. **No hidden geometry.** A source-level check rejects new geometry constants and high-level feature formulas in `car_visual_raster.c` outside an explicit quantization allowlist.
5. **Corpus sensitivity.** Every designated visual driver still moves both a named feature and visible pixels.
6. **Body/wheel composability.** Existing body-versus-whole-car equivalence remains intact.

## Suggested implementation sequence

1. Add `CarVisualRect` and the new fields without changing the rasterizer.
2. Populate them in `car_visual_derive()` and extend sanity tests and the bake key.
3. Switch one raster layer at a time to the derived fields, checking corpus equivalence after each layer.
4. Migrate canards last because replacing `onePx` intentionally changes output.
5. Retire redundant weights or fields only after all consumers and signature tests have moved.

## Non-goals

- no body-type enum;
- no hand-authored per-car drawing branches;
- no GPU-specific geometry path;
- no new cache or allocation strategy;
- no change to the physics values themselves.

This preserves the existing procedural design while enforcing the architecture the header already promises.
