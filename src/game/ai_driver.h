/*
 * ai_driver.h — a baseline lap driver that can only do what a player can do.
 *
 * THE AUDIT BOUNDARY IS THE SIGNATURE. Every argument to ai_driver_update() is `const`
 * except the `Input *out` it fills and the driver's own scratch state. It therefore cannot
 * set a position, a velocity, a wheel state, a gear, or a force: the compiler forbids it,
 * not a convention. Whatever this file does, the car experiences it as steering, throttle
 * and brake arriving on the same struct the keyboard writes to.
 *
 * It does not write `handbrake`, and it never writes a one-shot: no pause, no reset, no
 * manual shift. Gear selection belongs to the automatic transmission, which is what a player
 * driving with it enabled would also get.
 *
 * The control law is textbook pure pursuit plus a curvature speed target, and that choice is
 * deliberate: a baseline driver exists to expose how a car behaves, so it must be simple
 * enough that a surprising lap is evidence about the car rather than about the driver. One
 * AiDriverConfig is shared by every car. A car this driver cannot get round is a finding.
 *
 * Raylib-free apart from Vector2, and free of I/O and global state, so the headless harness
 * links it unchanged.
 */
#ifndef DRIFTY_AI_DRIVER_H
#define DRIFTY_AI_DRIVER_H

#include <stdbool.h>

#include "raylib.h" /* Vector2 */

#include "game/input.h"
#include "physics/vehicle.h"
#include "world/track.h"

/*
 * Tuning shared by every car. Values are physical wherever they can be, so that the same
 * numbers mean the same thing for a 900 kg hatchback and a 1400 kg GT car.
 */
typedef struct {
    float lookaheadBaseM;  /* m; lookahead distance at a standstill */
    float lookaheadSpeedS; /* s; extra lookahead per m/s of speed */

    /* Fractions of the tyre's own published grip the driver is willing to use. Below 1.0
     * because a controller that aims for the exact limit spends its life just past it. */
    float corneringGripFraction;
    float brakeGripFraction;

    float steerGainP; /* scales the pure-pursuit steer angle */
    float steerGainD; /* per (m/s) of cross-track error rate; damps the weave */

    float speedGainP;       /* 1/(m/s); pedal travel per m/s of speed error */
    float speedDeadbandMps; /* m/s; no pedal at all inside this band */

    float maxSpeedMps; /* m/s; hard ceiling on the speed target */
} AiDriverConfig;

/* The driver's memory between ticks. Plain value data: it lives inside Game. */
typedef struct {
    float prevCrossTrackErrorM;
    bool hasPrevError;
    int nearestSegment;      /* diagnostic: the segment matched last tick */
    float crossTrackErrorM;  /* diagnostic: signed, positive when left of the centreline */
    float targetSpeedMps;    /* diagnostic: what the speed controller was aiming for */
    float lookaheadAngleRad; /* diagnostic: bearing to the lookahead point, body frame */
} AiDriverState;

/* Sensible starting values for every field. Never per car. */
void ai_driver_config_default(AiDriverConfig *cfg);

/*
 * Produce one tick of driving.
 *
 * Writes `steer`, `throttle` and `brake` on `out` and zeroes `handbrake`; leaves every
 * one-shot exactly as it found it, so a caller may still latch its own pause or reset.
 * Does nothing when the track has fewer than three centreline nodes.
 */
void ai_driver_update(const AiDriverConfig *cfg, AiDriverState *state, const Track *track,
                      const VehicleState *vehicle, const VehicleDerived *derived,
                      const VehicleSpec *spec, Input *out, float dt);

#endif /* DRIFTY_AI_DRIVER_H */
