# Stable procedural car identity

Status: implemented. `CarAppearanceSpec` is defined in `src/render/car_appearance.h`;
this document is the contract it satisfies, not a proposal.

## Problem

The current palette seed hashes a list of physics values. A small tuning edit can cross a quantization boundary and avalanche into a completely unrelated hue. That couples two concerns:

- physics parameters determine geometry and handling;
- an arbitrary hash of those parameters determines paint and decorative color.

As a result, changing mass, track, grip, gearing, braking, or engine values can make the same edited vehicle appear to be a different car. It also makes geometry comparisons in the Physics Lab harder because color changes at the same time.

## Proposed contract

Introduce a presentation-only identity:

```c
typedef struct {
    bool hasSeed;
    uint32_t seed;
} CarAppearanceSpec;
```

The draft type is defined in `src/render/car_appearance.h` and intentionally does not live in `VehicleSpec`.

Add an explicit derivation entry point:

```c
void car_visual_derive_with_appearance(const VehicleSpec *spec,
                                       const CarAppearanceSpec *appearance,
                                       CarVisual *out);
```

Behavior:

1. Geometry continues to be derived exclusively from `VehicleSpec`.
2. When `appearance != NULL && appearance->hasSeed`, palette and decorative color use `appearance->seed`.
3. Otherwise, the existing `car_visual_colour_seed(spec)` behavior remains as a deterministic compatibility fallback.
4. `car_visual_derive(spec, out)` remains and delegates to the compatibility path.
5. The appearance seed may affect color only. It must never affect dimensions, feature presence, the geometry signature, physics, or collision.

A zero seed is valid; `hasSeed` is the presence indicator.

## Ownership and persistence

The running game should store one `CarAppearanceSpec` beside the active `VehicleSpec` in presentation state. This is a deliberate `Game` layout change and therefore requires the same platform-restart handling documented for other persistent-layout changes.

The field must be:

- excluded from `game_state_checksum()`;
- excluded from all physics validation and refresh functions;
- excluded from replay determinism unless a replay captures screenshots as part of its contract;
- preserved across game-module hot reloads because it is plain value data;
- reset or retained explicitly when `game_apply_spec()` changes vehicles.

Recommended `game_apply_spec()` behavior: retain the existing appearance for live tuning of the same active car. A separate vehicle-load path may replace both spec and appearance.

## Profile format

Vehicle profile files may add one optional presentation key:

```text
appearance.seed = 123456789
```

Loading an older profile without the key uses the compatibility fallback. Saving a profile with an assigned identity writes the key. The parser must not route the value through the physics parameter registry.

Corpus entries should use stable seeds derived from their corpus IDs, not from the mutable parameter values. This keeps generated contact sheets visually stable when an archetype is retuned.

## Palette behavior

The first implementation should preserve the current HSV mapping and change only the seed source. That keeps the visual change reviewable.

Later extensions may add explicit paint colors or livery identifiers, but they are out of scope until there is a concrete product requirement. The seed is enough to decouple identity from handling without creating a second styling framework.

## Bake key

Palette colors are baked pixels and therefore remain part of `car_visual_bake_key()`. Changing the appearance seed should change the derived colors and the bake key naturally.

The raw appearance seed does not need to be hashed separately if all of its visible consequences are already hashed as colors.

## Tests

The implementation is complete when these assertions pass:

1. Two identical physics specs with the same explicit appearance seed produce bit-identical `CarVisual` and RGBA rasters.
2. Two identical physics specs with different explicit seeds have identical geometry signatures and label maps, but different palette pixels and bake keys.
3. Changing a physics parameter while keeping an explicit seed does not change any palette field unless that parameter intentionally changes a non-color visual rule.
4. The compatibility entry point remains bit-identical to the pre-change output.
5. State checksums are unchanged when only appearance changes.
6. Existing profiles without `appearance.seed` still load.
7. Corpus contact sheets keep stable colors when only physics values inside an entry are adjusted.

## Migration sequence

1. Land `CarAppearanceSpec` and the new derive entry point while retaining compatibility output.
2. Add focused color/geometry independence tests.
3. Add the presentation field to `Game` and thread it through the renderer.
4. Add optional profile serialization.
5. Assign stable seeds to corpus entries.
6. Only then consider exposing paint selection in the UI.

## Non-goals

- no color-driven geometry;
- no body-type enum;
- no solver access to appearance;
- no mandatory migration of existing profiles;
- no arbitrary per-feature art overrides in this change.
