# Dynamic vehicle visual feedback

Status: implemented. `VehicleVisualEffects` and `vehicle_visual_effects_derive()` are in
`src/render/vehicle_effects.h`; this document is the contract they satisfy.

## Goal

Add runtime feedback that communicates braking, tire saturation, and lateral load without rebaking the procedural car textures.

The base `CarVisual` remains a deterministic, cached function of the vehicle specification. Dynamic effects are derived every rendered frame from transient simulation state and drawn as overlays or particles.

## Draft API

`src/render/vehicle_effects.h` defines explicit input and output structures:

```c
typedef struct {
    float brakeInput01;
    float filteredLateralAccelerationMps2;
    float frictionUsage01[WHEEL_COUNT];
    float slipAngleRad[WHEEL_COUNT];
    float slipRatio[WHEEL_COUNT];
    SurfaceId surface[WHEEL_COUNT];
} VehicleEffectInputs;

typedef struct {
    float brakeLamp01;
    float tireSmoke01[WHEEL_COUNT];
    float bodyRollRad;
    Vector2 shadowOffsetBodyM;
} VehicleVisualEffects;
```

`vehicle_visual_effects_derive()` must be pure, allocation-free, raylib-free, and linked into the headless test binary.

## Initial mappings

The first implementation should stay deliberately small.

### Brake lamps

- Input: service-brake command only.
- Mapping: smooth transition from off below 0.05 to full brightness by 0.35.
- The handbrake does not automatically illuminate road-car brake lamps.
- Draw two additive or alpha-blended overlays at the derived taillight primitives.
- Do not alter or rebake the static body texture.

The exact lamp centers and dimensions should come from `CarVisual`, not be recalculated in `render_vehicle.c`. This work therefore depends on the derived-primitive migration in PR #26.

### Tire smoke and loose-surface trails

For each wheel, derive a normalized intensity from the largest of:

- friction usage above the saturation threshold;
- normalized absolute slip ratio;
- normalized absolute slip angle.

Use smoothstep bands so a small numerical change cannot pop a full particle emitter into existence. The particle material depends on `SurfaceId`:

- asphalt: pale tire smoke;
- gravel: brown dust;
- grass: muted green/brown debris;
- snow: light powder.

The effect intensity is presentation-only. It must not feed tire temperature, grip, scoring, or any physics quantity.

### Body-load cue

A top-down sprite cannot show physical roll directly without introducing a second body transform. The first cue should therefore be restrained:

- derive a bounded visual roll signal from filtered lateral acceleration;
- use it to move the ground-contact shadow laterally in body space;
- optionally shift a roof highlight by a sub-pixel-to-one-pixel amount at the canonical scale;
- keep the center-of-mass pivot and collision representation unchanged.

The maximum apparent roll should be documented and clamped. Start with a presentation cap equivalent to 3 degrees and validate it in motion before increasing it.

## Render order

Recommended order:

1. persistent skid marks and loose-surface trails;
2. smoke/dust particles behind the car;
3. cached body texture;
4. steerable wheel textures;
5. dynamic brake-lamp overlays;
6. smoke particles that should pass in front of the rear body;
7. debug overlays.

Dynamic overlays must use the same body-to-render transform as wheel placement so heading, screen-Y inversion, and pixels-per-metre stay consistent.

## State and determinism

The effect derivation reads simulation state but writes presentation state only.

- Exclude all effect values from `game_state_checksum()`.
- Do not store effect state in replays unless replay video output becomes part of the contract.
- Given the same interpolated inputs, effect intensities must be deterministic.
- Particle emission may use the existing deterministic particle seed path; do not call an unseeded random generator.
- Hot reload may discard transient emitters, but must not leak GPU or heap resources.

## Tests

The implementation is complete when these checks pass:

1. Null input produces a fully zeroed `VehicleVisualEffects`.
2. Every intensity is finite and in `[0, 1]`; roll and shadow offsets are bounded.
3. Brake intensity is monotonic and does not respond to handbrake-only input.
4. Each wheel's trail intensity responds only to its own slip/friction inputs.
5. Surface selection changes material, not intensity.
6. Deriving effects does not change the game checksum.
7. The base sprite bake key is unchanged by every dynamic input.
8. A headless scripted drift produces nonzero rear tire effects and a scripted straight stop produces brake lamps without tire smoke.
9. Rendering at two scales changes only pixel dimensions, not derived effect intensities.

## Performance boundary

No texture upload, allocation, or corpus derivation may occur per frame. The pure effect state is a handful of arithmetic operations; particle emission uses the existing fixed-capacity pool.

## Non-goals

- no damage deformation;
- no suspension animation in this first change;
- no per-car particle systems;
- no effect-driven physics;
- no new general-purpose rendering framework.

The purpose is readable driving feedback, not a second simulation.
