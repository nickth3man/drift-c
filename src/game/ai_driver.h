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
 * THE DRIVER FINDS ITS OWN PATH. It is given the road — the authored centreline and the
 * half-width either side of it — and searches for the line it wants through that corridor,
 * from where the car currently is. Nothing hands it a precomputed racing line. This matters
 * because one line cannot serve a roster: corner speed goes as sqrt(mu/kappa), so a line
 * tightened for a grippy car is slower for a low-grip one, and the only fix for a stored line
 * is to blunt it until it suits nobody. A driver that plans from its own car's grip does not
 * have that problem, and a car this driver cannot get round is still a finding.
 *
 * One AiDriverConfig is shared by every car. The differences between laps come from the specs.
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
 * Search lattice size. These are array bounds, not tuning: they fix how much stack the search
 * needs, so they are compile-time constants rather than config fields.
 *
 * AI_PLAN_LAYERS centreline nodes ahead is 250 m on the chicane's 4 m node spacing and 150 m on
 * the technical circuit's 2.4 m — comfortably past the 114 m it takes to shed 40 m/s, which is
 * the distance the speed controller has to be able to see over to brake in time.
 *
 * AI_PLAN_OFFSETS candidates span whatever width is left after planEdgeMarginM: 1.2 m apart on
 * the chicane's 8 m half-width, 0.33 m on the technical circuit's narrowest 4.5 m. The cost of
 * raising it is quadratic — the search is over PAIRS of adjacent offsets — so this is the
 * resolution that pays for itself. Sweeps at 11 and 13 offsets bought nothing measurable.
 */
#define AI_PLAN_LAYERS 64
#define AI_PLAN_OFFSETS 9

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

    /* Pedal travel limits, in fraction of full travel per second. The car is driven with
     * analogue triggers, and a trigger has a finite travel speed: without these the
     * controller emits whatever its algebra produces on the current tick, which is a
     * square wave when the term it reads is a noisy per-tick physics signal. Pressing is
     * slower than releasing because releasing is spring-assisted. */
    float pedalPressRatePerS;
    float pedalReleaseRatePerS;

    /* Stick travel limits, same units and the same reason. A thumbstick cannot teleport
     * either, and steering was the one input still allowed to step discontinuously.
     *
     * These are deliberately FASTER than the road wheel's own STEER_RATE_RAD_S, so the stick
     * bounds the jerk in the signal without becoming the thing that limits how quickly the car
     * can be turned. A limit tighter than the actuator would cost lap time and prove nothing. */
    float steerPressRatePerS;
    float steerReleaseRatePerS;

    /* Fraction of travel the demand must move before the pedal or stick follows it at all. A
     * control held at a steady pressure does not dither; without this the rate limit still
     * tracks every small wobble in the demand, at full rate. */
    float pedalDeadband;

    /* Traction management. Friction usage is a saturating instantaneous tyre state: it sits
     * at or near 1.0 for most of a racing lap and recovers within a tick or two. Reading it
     * proportionally makes throttle a high-gain function of a signal that is mostly noise,
     * which is a square wave no filter can clean up without also blinding the driver to the
     * genuine low-grip windows. So it is read as an event instead — over the threshold, ease
     * off at a bounded rate; under it, feed the power back in at a bounded rate. The output
     * is smooth because the rates bound it, not because the input was smoothed. */
    float gripCutThreshold;    /* friction usage above which the driver eases off */
    float gripCutRatePerS;     /* fraction of throttle surrendered per second while over */
    float gripRecoverRatePerS; /* fraction fed back per second while under */
    float gripCutMax;          /* most throttle the driver will ever give up */

    /* --- Path search --- */

    /* m; clearance held between the planned line and the edge of the racing surface. Half a
     * car plus the tracking error a pure-pursuit driver shows on a curved line. This is what
     * keeps the found path on asphalt by construction rather than by a checked constraint. */
    float planEdgeMarginM;

    /* Cost per metre of path, per (1/m)^2 of curvature. Curvature rather than time because at
     * the friction limit a_lat = kappa * v^2, so curvature is what sets the speed a path can be
     * driven at — and unlike time it does not depend on the entry speed, which is what keeps the
     * cost separable and the search a plain shortest path instead of a speed-coupled one. */
    float planCurvatureWeight;

    /* Cost per metre of departure from the previous plan. Two lines of near-equal cost would
     * otherwise swap between searches and twitch the steering; this is the planner's equivalent
     * of the deadband that stops the pedals dithering. */
    float planHysteresisWeight;

    /* Fixed ticks between searches. A driver commits to a line through a corner rather than
     * reconsidering it 120 times a second, and the plan is stable over far longer than this. */
    int planReplanTicks;
} AiDriverConfig;

/* The driver's memory between ticks. Plain value data: it lives inside Game. */
typedef struct {
    float prevCrossTrackErrorM;
    bool hasPrevError;
    int nearestSegment;      /* diagnostic: the segment matched last tick */
    float crossTrackErrorM;  /* diagnostic: signed, positive when left of the planned line */
    float targetSpeedMps;    /* diagnostic: what the speed controller was aiming for */
    float lookaheadAngleRad; /* diagnostic: bearing to the lookahead point, body frame */
    /* Which point on the plan set the speed target, and how far ahead it was. Diagnostics, like
     * the four above, and the two that matter most: a target set by curvature a metre in front
     * of the car is the signature of the driver braking for its own tracking error rather than
     * for the road, and without these that failure looks like nothing but a slow lap. */
    float bindingCurvature1pm;
    float bindingDistanceM;

    /* Longitudinal demand as one signed axis: +1 is full throttle, -1 is full brake. Held
     * across ticks because it is what the pedal rate limit acts on. Keeping it as a single
     * axis is also what guarantees the driver never presses both pedals — throttle and
     * brake are two halves of this one number, so they cannot both be nonzero. */
    float pedalAxis;

    /* The steering axis actually emitted, held across ticks for the same reason. */
    float steerAxis;

    /* Throttle currently surrendered to traction management, 0 (none) .. gripCutMax. Stored
     * as the amount given up rather than the amount available so that a zeroed state means a
     * driver willing to use all of the engine, which is what every caller memsets it to. */
    float gripCut;

    /* The plan: a lateral offset from the centreline for each of the next planLayerCount
     * nodes, starting at node planBaseNode. Positive is left of travel. Only the RESULT of the
     * search is kept — the lattice itself is recomputed from the track each time, because its
     * vertices are a function of geometry the track already publishes.
     *
     * planLayerCount == 0 means no plan yet, which is what a zeroed state means and what every
     * caller memsets it to. */
    float planOffsetM[AI_PLAN_LAYERS];
    int planLayerCount;
    int planBaseNode;
    int ticksSinceReplan;
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

/*
 * World position of the planned line at centreline node `nodeIndex`, for a caller that wants to
 * inspect the path the driver chose — a test asserting it stays on the racing surface, or a
 * debug overlay drawing it. Nodes outside the current plan window fall back to the centreline.
 * Returns the centreline point when there is no plan.
 */
Vector2 ai_driver_plan_point(const AiDriverState *state, const Track *track, int nodeIndex);

#endif /* DRIFTY_AI_DRIVER_H */
