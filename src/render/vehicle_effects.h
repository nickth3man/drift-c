/*
 * vehicle_effects.h — draft render-only state for dynamic vehicle feedback.
 *
 * The base CarVisual remains a cached function of the vehicle specification. These inputs and
 * outputs are transient presentation values: they must not invalidate the baked sprite, enter
 * the simulation checksum, or feed any solver.
 */
#ifndef DRIFTY_VEHICLE_EFFECTS_H
#define DRIFTY_VEHICLE_EFFECTS_H

#include "physics/vehicle.h"

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

/* Pure render-only derivation. A NULL input returns a zeroed effect state.
 *
 * NOT YET DERIVED: vehicle_effects.c currently returns the neutral state for every input,
 * because the effect thresholds are still unagreed and the taillight overlay depends on the
 * derived lamp geometry. Callers link and draw today's picture; they do not get feedback
 * overlays until the real derivation lands. */
VehicleVisualEffects vehicle_visual_effects_derive(const VehicleEffectInputs *inputs);

#endif /* DRIFTY_VEHICLE_EFFECTS_H */
