/*
 * gameplay_tests.c — track geometry and surfaces, barrier collision, scoring, high-score
 * persistence, checkpoints and laps, the particle pool, and the state machine.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "support/test_harness.h"
#include "support/simulation_fixture.h"
#include "test_scenarios.h"
#include "scenario_shared.h"

#include "dev/car_corpus.h"
#include "game/ai_driver.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "core/config.h"
#include "dev/dev_params.h"
#include "dev/dev_replay.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "physics/drivetrain.h"
#include "dev/failure_bundle.h"
#include "physics/surface.h"
#include "game/game.h"
#include "game/scoring.h"
#include "game/input.h"
#include "core/math_utils.h"
#include "game/particle.h"
#include "physics/physics.h"
#include "render/render.h"
#include "game/replay.h"
#include "game/telemetry.h"
#include "platform/timestep.h"
#include "physics/tire.h"
#include "core/units.h"
#include "world/collision.h"

/* ------------------------------------------------------------------------------------- */
/* Scenario: track surface                                                                 */
/* ------------------------------------------------------------------------------------- */

static void scenario_track_surface(void)
{
    /* Life-cycle: initialise, free, double-free safety. */
    Track track;
    memset(&track, 0, sizeof(track));

    track_init(&track);
    check(track.nodes != NULL, "track init: nodes is non-NULL");
    check(track.count == 5, "track init: count == 5 (got %d)", track.count);
    check(track.offTrackSurfaceId == SURFACE_GRASS,
          "track init: offTrackSurfaceId is SURFACE_GRASS (got %d)",
          (int)track.offTrackSurfaceId);
    check(track.nextCheckpoint == 0, "track init: nextCheckpoint is 0");
    check(track.lap == 0, "track init: lap is 0");
    check_near((double)track.lapTimerS, 0.0, 0.0, "track init: lapTimerS is 0");

    /* Query the centre at (0, 0): inside the 200×150 m parking lot, so it should be asphalt. */
    const SurfaceId centerSurf = Track_SurfaceAt(&track, (Vector2){ 0.0f, 0.0f });
    check(centerSurf == SURFACE_ASPHALT, "Track_SurfaceAt origin returns ASPHALT (got %d)",
          (int)centerSurf);

    /* Query at a centreline node point: should be asphalt. */
    const SurfaceId nodeSurf = Track_SurfaceAt(&track, track.nodes[0].centerM);
    check(nodeSurf == SURFACE_ASPHALT,
          "Track_SurfaceAt(centreline node) returns ASPHALT (got %d)", (int)nodeSurf);

    /* Just inside the lot boundary: offset from a perimeter node by less than halfWidthM. */
    {
        const Vector2 insidePoint = { track.nodes[0].centerM.x,
                                      track.nodes[0].centerM.y +
                                          track.nodes[0].halfWidthM * 0.7f };
        const SurfaceId insideSurf = Track_SurfaceAt(&track, insidePoint);
        check(insideSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt inside boundary returns ASPHALT (got %d)", (int)insideSurf);
    }

    /* Just outside: (0, 200) is 50 m above the lot top at y = 150. */
    {
        const Vector2 outsidePoint = { 0.0f, 200.0f };
        const SurfaceId outsideSurf = Track_SurfaceAt(&track, outsidePoint);
        check(outsideSurf == SURFACE_GRASS,
              "Track_SurfaceAt outside boundary returns GRASS (got %d)", (int)outsideSurf);
    }

    /* Far away: (1000, 0). */
    {
        const SurfaceId farSurf = Track_SurfaceAt(&track, (Vector2){ 1000.0f, 0.0f });
        check(farSurf == SURFACE_GRASS, "Track_SurfaceAt far point returns GRASS (got %d)",
              (int)farSurf);
    }

    /* NULL / uninitialised track returns ASPHALT (defensive default). */
    {
        Track dummy;
        memset(&dummy, 0, sizeof(dummy));
        const SurfaceId nullSurf = Track_SurfaceAt(NULL, (Vector2){ 0.0f, 0.0f });
        check(nullSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt(NULL, ...) returns ASPHALT (got %d)", (int)nullSurf);
        const SurfaceId uninitSurf = Track_SurfaceAt(&dummy, (Vector2){ 0.0f, 0.0f });
        check(uninitSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt(uninitialised, ...) returns ASPHALT (got %d)", (int)uninitSurf);
    }
    /* Track distance-to-centerline: on a centreline node, distance should be ~0. */
    {
        float hw = 0.0f;
        const float d = track_distance_to_centerline_m(&track, track.nodes[0].centerM, &hw);
        check(fabsf(d) < 0.01f, "centreline dist at node 0 ~0 (got %.2f)", (double)d);
        check(fabsf(hw - 4.0f) < 0.01f, "half-width at node 0 ~4 (got %.2f)", (double)hw);
    }
    /* Outside: y=200 is 50 m above lot top at y=150. */
    {
        float hw = 0.0f;
        const float d = track_distance_to_centerline_m(&track, (Vector2){ 0, 200 }, &hw);
        check(d > 40.0 && d < 60.0, "centreline dist outside lot ~50 m");
    }
    /* NULL/uninit: returns 0. */
    {
        float hw = -1.0f;
        Track dummy;
        memset(&dummy, 0, sizeof(dummy));
        check_near((double)track_distance_to_centerline_m(NULL, (Vector2){ 0, 0 }, &hw), 0.0,
                   0.0, "NULL dist=0");
        check_near((double)hw, 0.0, 0.0, "NULL hw=0");
        hw = -1.0f;
        check_near((double)track_distance_to_centerline_m(&dummy, (Vector2){ 0, 0 }, &hw), 0.0,
                   0.0, "uninit dist=0");
        check_near((double)hw, 0.0, 0.0, "uninit hw=0");
    }
    /* Free and verify clean. */
    track_free(&track);
    check(track.nodes == NULL, "track_free: nodes is NULL");
    check(track.count == 0, "track_free: count is 0");

    /* Double-free safety. */
    track_free(&track);
    check(track.nodes == NULL, "track double-free: nodes stays NULL");
    check(track.count == 0, "track double-free: count stays 0");

    /* Re-init after free works. */
    track_init(&track);
    check(track.nodes != NULL, "track re-init: nodes is non-NULL");
    check(track.count == 5, "track re-init: count == 5");
    track_free(&track);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: collision-barrier — capsule vs track boundary, swept test, impulse response  */
/* ------------------------------------------------------------------------------------- */

static void scenario_collision_barrier(void)
{
    /* --- Barrier hit from straight approach: car aims DOWN at the
     *     parking-lot bottom boundary (bottom edge at y = -75 m, barrier at y ≈ -79 m) --- */
    Game *game = alloc_game();
    game_init(game);
    /* In headless builds game_init does NOT call track_init, so we must. */
    track_init(&game->track);

    /* Place the car near the boundary, heading straight down at it.
     * The inner bottom barrier is at y ≈ -146 m (centerline -150 plus halfWidth 4). */
    game->vehicle.positionM = (Vector2){ 0.0f, -142.5f }; /* ~3.5 m above the inner barrier */
    game->vehicle.headingRad = -1.57079632679f;           /* pointing -Y (down) */
    game->vehicle.velocityLongitudinalMps = 30.0f;        /* body X forward = world -Y */
    game->vehicle.velocityLateralMps = 0.0f;
    game->renderState.prevPositionM = game->vehicle.positionM;
    game->renderState.prevHeadingRad = game->vehicle.headingRad;
    game->renderState.currPositionM = game->vehicle.positionM;
    game->renderState.currHeadingRad = game->vehicle.headingRad;
    game->state = STATE_PLAYING;

    /* Before tick: verify car is on the correct side (above the barrier at y ≈ -154). */
    /* Before tick: verify car is on the correct side (above the barrier at y ≈ -146). */
    const float yBefore = game->vehicle.positionM.y;
    check(yBefore > -146.0f, "car starts inside the track boundary (y = %.2f > -146.0)",
          (double)yBefore);
    /* Run fixed updates at 120 Hz. The car moves ~0.25 m down per tick at 30 m/s.
     * Even with engine braking, 60 ticks (~0.5 s) is enough to reach y ≈ -79 m. */
    Input tickInput;
    input_zero(&tickInput);
    tickInput.throttle = 0.0f;
    tickInput.brake = 0.0f;

    bool hitBarrier = false;
    float speedBeforeHit = 30.0f;
    float speedAfterHit = 30.0f;
    float yAfter = -72.0f;

    for (int i = 0; i < 60; i++) {
        speedBeforeHit = game->derived.speedMps;
        game->input = tickInput;
        game_fixed_update(game, FIXED_DT_S);
        yAfter = game->vehicle.positionM.y;
        if (game->crashLockoutTimerS > 0.0f) {
            speedAfterHit = game->derived.speedMps;
            hitBarrier = true;
            break;
        }
        if (yAfter < -165.0f) break; /* car passed far beyond, collision didn't work */
    }

    /* After the tick, the car should NOT have passed through the boundary. */
    check(yAfter >= -147.5f,
          "car did not tunnel through the barrier (y = %.4f, must be > -147.5)",
          (double)yAfter);
    check(hitBarrier, "car hit the barrier (crashLockoutTimerS was set)");
    check(game->crashLockoutTimerS > 0.0f,
          "significant impact sets crashLockoutTimerS (%.4f > 0)",
          (double)game->crashLockoutTimerS);
    check(speedAfterHit < speedBeforeHit, "car lost speed from impact (%.2f < %.2f m/s)",
          (double)speedAfterHit, (double)speedBeforeHit);

    /* --- Decay of crash lockout timer --- */
    const float lockoutBefore = game->crashLockoutTimerS;
    game->input = tickInput; /* no input */
    game_fixed_update(game, FIXED_DT_S);
    check(game->crashLockoutTimerS < lockoutBefore, "crashLockoutTimerS decays (%.4f < %.4f)",
          (double)game->crashLockoutTimerS, (double)lockoutBefore);

    track_free(&game->track);
    free(game);

    /* --- Glancing hit: car approaches at shallow angle to produce yaw spin --- */
    Game *game2 = alloc_game();
    game_init(game2);
    track_init(&game2->track);
    /* Place car near the bottom of the lot, heading right-down at a shallow angle
     * toward the bottom barrier at y ≈ -146 m. */
    game2->vehicle.positionM = (Vector2){ 0.0f, -145.0f };
    game2->vehicle.headingRad = -1.2f; /* shallow angle heading right-down */
    game2->vehicle.velocityLongitudinalMps = 30.0f;
    game2->vehicle.yawRateRadS = 0.0f;
    game2->renderState.prevPositionM = game2->vehicle.positionM;
    game2->renderState.prevHeadingRad = game2->vehicle.headingRad;
    game2->renderState.currPositionM = game2->vehicle.positionM;
    game2->renderState.currHeadingRad = game2->vehicle.headingRad;
    game2->state = STATE_PLAYING;

    float peakYawRate = 0.0f;
    input_zero(&tickInput);
    /* Run ticks until we hit the barrier or pass through. */
    for (int i = 0; i < 60; i++) {
        game2->input = tickInput;
        game_fixed_update(game2, FIXED_DT_S);
        peakYawRate = fmaxf(peakYawRate, fabsf(game2->vehicle.yawRateRadS));
        if (game2->crashLockoutTimerS > 0.0f) break;
    }

    check(peakYawRate > 0.1f,
          "glancing hit produces measurable yaw rate (peak %.4f rad/s > 0.1)",
          (double)peakYawRate);

    track_free(&game2->track);
    free(game2);
}
/* ------------------------------------------------------------------------------------- */
/* Scenario: collision-units — direct tests for collision_resolve_track              */
/* ------------------------------------------------------------------------------------- */

static void scenario_collision_units(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);
    spec.bodyHalfWidthM = 0.85f;
    spec.collisionRestitution = 0.30f;
    spec.collisionFriction = 0.50f;
    spec.massKg = 1200.0f;
    spec.yawInertiaKgM2 = 1500.0f;
    vehicle_spec_refresh_derived(&spec);

    /* Standard parking-lot track: bottom wall centerline at y=-150, hw=4.
     * Bottom segment (nodes[0]→[1], dir=(1,0)):
     *   left  barrier at y = -146 (perp +4),  pushN = {0,+1} (up)
     *   right barrier at y = -154 (perp -4),  pushN = {0,+1} (up) */
    Track track;
    memset(&track, 0, sizeof(track));
    track_init(&track);

    /* 1. No collision: car dead centre. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, 0 };
        state.headingRad = 0.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        int n = collision_resolve_track(&spec, &state, &rs, &track, &lockout);
        check(n == 0, "no collision returns 0 (got %d)", n);
        check(lockout == 0.0f, "no collision leaves lockout at 0");
    }

    /* 2. Front circle penetrates the left barrier from the track side (y=-146.5, 0.5 m below
     *    the y=-146 barrier). Approaching at 5 m/s: penetration corrected + impulse applied. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -146.5f };
        state.headingRad = 0.0f;
        state.velocityLongitudinalMps = 0.0f;
        state.velocityLateralMps = -5.0f; /* -Y, toward the y=-146 barrier */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        const float yBefore = state.positionM.y;
        int n = collision_resolve_track(&spec, &state, &rs, &track, &lockout);
        check(n >= 1, "left barrier contact resolves (got %d)", n);
        check(state.positionM.y > yBefore,
              "penetration push moves CG up, away from barrier (y %.4f > %.4f)",
              (double)state.positionM.y, (double)yBefore);
        check(lockout > 0.0f, "fast approach (5 m/s) sets crash lockout");
        check(state.velocityLateralMps < 5.0f, "impulse reduced lateral velocity (%.4f < 5.0)",
              (double)state.velocityLateralMps);
    }

    /* 3. Separating velocity: penetration corrected, NO impulse (velocity unchanged). */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -146.5f };
        state.headingRad = 0.0f;
        state.velocityLongitudinalMps = 0.0f;
        state.velocityLateralMps = 5.0f; /* moving +Y, AWAY from the y=-146 barrier */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        int n = collision_resolve_track(&spec, &state, &rs, &track, &lockout);
        check(n >= 1, "separating contact still resolves penetration (got %d)", n);
        check(lockout == 0.0f, "separating contact does NOT set lockout");
        /* Velocity is unchanged because no impulse was applied (vn >= 0). */
        check_near((double)state.velocityLateralMps, 5.0, 1e-3,
                   "separating contact leaves velocity unchanged");
    }

    /* 4. Right barrier (bottom wall at y=-154): pushN = {0,+1} (up). */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -154.5f };
        state.headingRad = 0.0f;
        state.velocityLongitudinalMps = 0.0f;
        state.velocityLateralMps = -5.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        const float yBefore = state.positionM.y;
        int n = collision_resolve_track(&spec, &state, &rs, &track, &lockout);
        check(n >= 1, "right barrier contact resolves (got %d)", n);
        check(state.positionM.y > yBefore, "right barrier pushes CG up (y %.4f > %.4f)",
              (double)state.positionM.y, (double)yBefore);
    }
    /* 4b. Multi-contact: narrow corridor (hw=1.5), car at 90° spanning both walls.
     *     Front circle hits the left barrier (y=+1.5), rear hits the right (y=-1.5).
     *     These are DIFFERENT walls with opposing push normals, so both resolve. */
    {
        TrackNode corridorNodes[4] = {
            { { -50, 0 }, 1.5f, SURFACE_ASPHALT, 0.0f },
            { { 50, 0 }, 1.5f, SURFACE_ASPHALT, 0.0f },
            { { 50, 100 }, 50.0f, SURFACE_ASPHALT, 0.0f },
            { { -50, 100 }, 50.0f, SURFACE_ASPHALT, 0.0f },
        };
        Track corridor = { 0 };
        corridor.nodes = corridorNodes;
        corridor.count = 4;
        corridor.offTrackSurfaceId = SURFACE_ASPHALT;

        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, 0 };
        state.headingRad = 1.57079632679f; /* 90 deg: body X = world +Y */
        /* Front circle at y approx +cgToFront, rear at y approx -cgToRear,
         * both within 0.85 of the +/-1.5 walls. */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        int n = collision_resolve_track(&spec, &state, &rs, &corridor, &lockout);
        check(n >= 2, "narrow corridor: both walls contacted -> >= 2 (got %d)", n);
        check(isfinite(state.positionM.x) && isfinite(state.positionM.y),
              "multi-contact position stays finite");
    }

    /* 5. Lockout threshold: slow kiss does not trigger lockout. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -71.5f };
        state.headingRad = 0.0f;
        state.velocityLateralMps = 1.0f; /* below COLLISION_LOCKOUT_THRESHOLD_MPS (2.0) */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        collision_resolve_track(&spec, &state, &rs, &track, &lockout);
        check(lockout == 0.0f, "slow kiss (< 2 m/s) does not set lockout");
    }

    /* 6. Determinism: same input twice → identical output. */
    {
        VehicleState s1 = { 0 }, s2 = { 0 };
        VehicleRenderState r1 = { 0 }, r2 = { 0 };
        s1.positionM = s2.positionM = (Vector2){ 0, -71.5f };
        s1.headingRad = s2.headingRad = 0.0f;
        s1.velocityLateralMps = s2.velocityLateralMps = 5.0f;
        r1.prevPositionM = r1.currPositionM = s1.positionM;
        r2.prevPositionM = r2.currPositionM = s2.positionM;
        r1.prevHeadingRad = r1.currHeadingRad = s1.headingRad;
        r2.prevHeadingRad = r2.currHeadingRad = s2.headingRad;
        float lo1 = 0, lo2 = 0;
        collision_resolve_track(&spec, &s1, &r1, &track, &lo1);
        collision_resolve_track(&spec, &s2, &r2, &track, &lo2);
        check(memcmp(&s1, &s2, sizeof(VehicleState)) == 0,
              "collision_resolve_track is deterministic across identical calls");
        check(lo1 == lo2, "lockout is deterministic");
    }

    /* 7. No NaN or infinity in outputs. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -71.5f };
        state.headingRad = 0.0f;
        state.velocityLateralMps = 5.0f;
        state.yawRateRadS = 2.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        collision_resolve_track(&spec, &state, &rs, &track, &lockout);
        check(isfinite(state.positionM.x) && isfinite(state.positionM.y),
              "position is finite after collision");
        check(isfinite(state.velocityLongitudinalMps) && isfinite(state.velocityLateralMps),
              "velocity is finite after collision");
        check(isfinite(state.yawRateRadS), "yaw rate is finite after collision");
        check(isfinite(lockout), "lockout is finite");
    }

    /* 8. Variable-width segment: per-node half-widths produce a slanted barrier. A car
     *    near the narrow end collides under per-node interpolation. */
    {
        TrackNode vwNodes[4] = {
            { { 0, 0 }, 5.0f, SURFACE_ASPHALT, 0.0f },
            { { 10, 0 }, 10.0f, SURFACE_ASPHALT, 0.0f },
            { { 10, 100 }, 100.0f, SURFACE_ASPHALT, 0.0f },
            { { 0, 100 }, 100.0f, SURFACE_ASPHALT, 0.0f },
        };
        Track vwTrack = { 0 };
        vwTrack.nodes = vwNodes;
        vwTrack.count = 4;
        vwTrack.offTrackSurfaceId = SURFACE_ASPHALT;

        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -5.5f }; /* 0.5 m below the narrow-end right barrier */
        state.headingRad = 0.0f;
        state.velocityLateralMps = -1.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        int n = collision_resolve_track(&spec, &state, &rs, &vwTrack, &lockout);
        check(n >= 1, "variable-width segment collides at the narrow end (per-node widths)");
    }

    /* 9. NULL / degenerate inputs return 0. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        float lockout = 0.0f;
        check(collision_resolve_track(NULL, &state, &rs, &track, &lockout) == 0,
              "NULL spec returns 0");
        check(collision_resolve_track(&spec, NULL, &rs, &track, &lockout) == 0,
              "NULL state returns 0");
        check(collision_resolve_track(&spec, &state, &rs, NULL, &lockout) == 0,
              "NULL track returns 0");
        /* Track with too few nodes. */
        Track tiny = { 0 };
        tiny.count = 1;
        check(collision_resolve_track(&spec, &state, &rs, &tiny, &lockout) == 0,
              "track with < 2 nodes returns 0");
    }

    track_free(&track);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: checkpoint-lap — gate crossing, lap counting, forward-only, timer reset      */
/* ------------------------------------------------------------------------------------- */

/* Cross gate `n` of the 10x10 square below, travelling the way the gate faces. Returns the
 * event so a caller can assert on order as well as on the crossing itself. */
static TrackCheckpointEvent cross_square_gate(Track *track, int n)
{
    switch (n) {
        /* Gate 0 at (0,0) faces +X and spans y in [-2,+2]. */
        case 0:
            return track_update_checkpoints(track, (Vector2){ -0.1f, 1.0f },
                                            (Vector2){ 0.1f, 1.0f });
        /* Gate 1 at (10,0) faces +Y and spans x in [8,12]. */
        case 1:
            return track_update_checkpoints(track, (Vector2){ 10.0f, -0.1f },
                                            (Vector2){ 10.0f, 0.1f });
        /* Gate 2 at (10,10) faces -X and spans y in [8,12]. */
        case 2:
            return track_update_checkpoints(track, (Vector2){ 10.1f, 10.0f },
                                            (Vector2){ 9.9f, 10.0f });
        /* Gate 3 at (0,10) faces -Y and spans x in [-2,+2]. */
        default:
            return track_update_checkpoints(track, (Vector2){ 1.0f, 10.1f },
                                            (Vector2){ 1.0f, 9.9f });
    }
}

static void scenario_checkpoint_lap(void)
{
    /* A 10 m x 10 m counterclockwise square: (0,0) -> (10,0) -> (10,10) -> (0,10).
     * halfWidthM 2 m, so the car positions below are comfortably in-bounds. Gates are derived
     * from the nodes, which is the scheme a hand-built ribbon gets. */
    Track track;
    memset(&track, 0, sizeof(track));
    track.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    track.count = 4;
    track.offTrackSurfaceId = SURFACE_GRASS;
    track.runoffSurfaceId = SURFACE_GRASS;
    track.nodes[0] = (TrackNode){ { 0.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[1] = (TrackNode){ { 10.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[2] = (TrackNode){ { 10.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[3] = (TrackNode){ { 0.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    check(track_build_checkpoints_from_nodes(&track), "gates derive from the node ribbon");
    check(track.checkpointCount == 4, "one gate per node (got %d)", track.checkpointCount);
    track.nextCheckpoint = 0;

    /* --- Ordered traversal advances, and gate 0 is the finish line --- */
    /* Gate 0 closes a lap the moment it is taken, because it IS the start/finish. Starting
     * progress at gate 0 therefore scores a lap immediately; a standing start avoids that by
     * expecting gate 1 first, which is what track_reset_progress() sets up. */
    TrackCheckpointEvent ev = cross_square_gate(&track, 0);
    check(ev.crossed && !ev.outOfOrder, "crossing gate 0 is an in-order crossing");
    check(ev.index == 0, "the event names gate 0 (got %d)", ev.index);
    check(ev.lapCompleted, "gate 0 is the finish line, so taking it closes a lap");
    check(track.nextCheckpoint == 1, "nextCheckpoint is 1 after gate 0 (got %d)",
          track.nextCheckpoint);

    ev = cross_square_gate(&track, 1);
    check(ev.crossed && !ev.outOfOrder, "crossing gate 1 advances");
    check(!ev.lapCompleted, "an intermediate gate does not close a lap");
    check(track.nextCheckpoint == 2, "nextCheckpoint is 2 after gate 1 (got %d)",
          track.nextCheckpoint);

    ev = cross_square_gate(&track, 2);
    check(ev.crossed && !ev.outOfOrder, "crossing gate 2 advances");
    check(track.nextCheckpoint == 3, "nextCheckpoint is 3 after gate 2 (got %d)",
          track.nextCheckpoint);

    /* --- An out-of-order crossing is REPORTED and does not advance --- */
    /* Expecting gate 3, the car instead cuts across gate 1. The old scheme only ever looked
     * at the expected gate, so this was indistinguishable from driving nowhere. */
    ev = cross_square_gate(&track, 1);
    check(ev.crossed, "cutting to gate 1 is detected as a crossing");
    check(ev.outOfOrder, "...and is reported as out of order");
    check(ev.index == 1, "...naming the gate actually crossed (got %d)", ev.index);
    check(!ev.lapCompleted, "an out-of-order crossing cannot close a lap");
    check(track.nextCheckpoint == 3, "out-of-order crossing does not advance progress (got %d)",
          track.nextCheckpoint);

    /* --- Reverse crossing does not advance --- */
    track.nextCheckpoint = 0;
    ev = track_update_checkpoints(&track, (Vector2){ 0.2f, 1.0f }, (Vector2){ -0.2f, 1.0f });
    check(!ev.crossed, "reverse crossing of gate 0 does NOT advance");
    check(track.nextCheckpoint == 0, "nextCheckpoint still 0 after reverse crossing (got %d)",
          track.nextCheckpoint);

    /* --- A full lap: gates 1,2,3 then back through the finish line --- */
    track.nextCheckpoint = 1;
    track.lap = 0;
    track.lapTimerS = 5.0f;
    check(cross_square_gate(&track, 1).crossed, "gate 1");
    check(cross_square_gate(&track, 2).crossed, "gate 2");
    check(cross_square_gate(&track, 3).crossed, "gate 3");
    check(track.nextCheckpoint == 0, "nextCheckpoint wraps to the finish line (got %d)",
          track.nextCheckpoint);
    check(track.lap == 0, "no lap yet: the finish line has not been recrossed (got %d)",
          track.lap);

    ev = cross_square_gate(&track, 0);
    check(ev.lapCompleted, "recrossing the finish line completes the lap");
    check(track.lap == 1, "lap increments to 1 (got %d)", track.lap);
    check_near((double)ev.lapTimeS, 5.0, 1e-3, "the event reports the completed lap time");
    check(track.lapTimerS < 0.1f, "lapTimerS resets on lap completion (%.4f s)",
          (double)track.lapTimerS);
    check(track.lastLapTimeS > 4.5f,
          "lastLapTimeS records the completed lap time (%.4f s > 4.5)",
          (double)track.lastLapTimeS);

    /* --- Timer accumulation --- */
    track.lapTimerS = 0.0f;
    track.lapTimerS += 0.5f;
    check_near((double)track.lapTimerS, 0.5, 1e-6, "lapTimerS accumulates");

    /* --- Car outside the gate span does not trigger --- */
    /* Gate 0 spans y in [-2,+2]; crossing the line at y = 5 misses it. No other gate lies on
     * that path either, so this must not register as an out-of-order crossing. */
    track.nextCheckpoint = 0;
    track.lap = 0;
    ev = track_update_checkpoints(&track, (Vector2){ -0.1f, 5.0f }, (Vector2){ 0.1f, 5.0f });
    check(!ev.crossed, "crossing outside the gate span does NOT advance");
    check(track.nextCheckpoint == 0,
          "nextCheckpoint unchanged after out-of-bounds cross (got %d)", track.nextCheckpoint);

    /* --- Stationary car does not trigger --- */
    ev = track_update_checkpoints(&track, (Vector2){ 0.0f, 1.0f }, (Vector2){ 0.0f, 1.0f });
    check(!ev.crossed, "stationary car does NOT advance checkpoints");

    /* --- NULL/edge case safety --- */
    ev = track_update_checkpoints(NULL, (Vector2){ 0, 0 }, (Vector2){ 1, 0 });
    check(!ev.crossed && ev.index == -1,
          "track_update_checkpoints with NULL track reports nothing gracefully");

    track_free(&track);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: chicane-track — the validation circuit's shape, gates, and identity           */
/* ------------------------------------------------------------------------------------- */

static void scenario_chicane_track(void)
{
    Track track;
    memset(&track, 0, sizeof(track));
    track_load_chicane(&track);

    check(track.nodes != NULL && track.count > 0, "the chicane allocates a centreline (%d)",
          track.count);
    check(!track.isParkingLot, "the chicane is a ribbon, not a parking lot");
    check(strcmp(track.version, "chicane_v1") == 0, "the track carries its version (%s)",
          track.version);

    /* --- The loop actually closes ---
     * Every consecutive node pair must be within a node spacing of each other, INCLUDING the
     * wrap from the last node back to the first. A loop that does not close leaves a long
     * phantom segment whose barriers cut straight across the circuit. */
    {
        float longestGapM = 0.0f;
        int longestAt = -1;
        for (int i = 0; i < track.count; i++) {
            const Vector2 a = track.nodes[i].centerM;
            const Vector2 b = track.nodes[(i + 1) % track.count].centerM;
            const float gap = sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
            if (gap > longestGapM) {
                longestGapM = gap;
                longestAt = i;
            }
        }
        check(longestGapM < 6.0f,
              "the loop closes: longest node gap is %.2f m at index %d (spacing is 4 m)",
              (double)longestGapM, longestAt);
    }

    /* --- Length is in the range the lap-time budget assumes --- */
    {
        const float lengthM = track_length_m(&track);
        check(lengthM > 600.0f && lengthM < 780.0f,
              "the lap is about 690 m, so a lap fits the replay buffer (got %.1f m)",
              (double)lengthM);
    }

    /* --- Gates --- */
    check(track.checkpointCount == 8, "eight required gates (got %d)", track.checkpointCount);
    for (int i = 0; i < track.checkpointCount; i++) {
        const Checkpoint *c = &track.checkpoints[i];
        const float mag =
            sqrtf(c->forwardUnit.x * c->forwardUnit.x + c->forwardUnit.y * c->forwardUnit.y);
        check(fabsf(mag - 1.0f) < 1e-5f, "gate %d has a unit forward direction (|f| = %.6f)", i,
              (double)mag);
        check(c->required, "gate %d is required", i);
        check(c->halfWidthM > 0.0f, "gate %d has a positive span", i);
    }

    /* Every gate must sit on the racing surface, or a car driving the circuit correctly could
     * never cross it. This is the check that catches a gate placed from stale geometry. */
    for (int i = 0; i < track.checkpointCount; i++) {
        const SurfaceId at = Track_SurfaceAt(&track, track.checkpoints[i].centerM);
        check(at == SURFACE_ASPHALT, "gate %d sits on the racing surface (got %d)", i, (int)at);
    }

    /* --- A standing start expects gate 1, not gate 0 --- */
    check(track.nextCheckpoint == 1,
          "a standing start on the finish line expects gate 1 next (got %d)",
          track.nextCheckpoint);
    check(track.lap == 0, "a fresh track has no completed laps");

    /* --- Start pose --- */
    {
        Vector2 startM = { 0.0f, 0.0f };
        float headingRad = 0.0f;
        check(track_start_pose(&track, &startM, &headingRad), "the track reports a start pose");
        check_near((double)headingRad, 0.0, 1e-5,
                   "the start pose faces +X along the near straight");
        check(Track_SurfaceAt(&track, startM) == SURFACE_ASPHALT,
              "the car starts on the racing surface");
    }

    /* --- The chicane genuinely displaces the centreline ---
     * Without this the "chicane" is a straight and the track tests nothing it was built for. */
    {
        float maxYM = 0.0f;
        for (int i = 0; i < track.count; i++) {
            if (track.nodes[i].centerM.y > maxYM) maxYM = track.nodes[i].centerM.y;
        }
        check(maxYM > 100.0f,
              "the chicane displaces the far straight (peak y = %.1f m, straight is at 90 m)",
              (double)maxYM);
    }

    /* --- P4: a car following the centreline never touches a barrier ---
     *
     * Barriers are built per segment, so on the inside of a curve consecutive segments form a
     * concave polyline that bulges toward the racing line, and at a joint a swept capsule can
     * catch on the corner. That would show up as phantom impacts scattered around the curves
     * and through the chicane, which is indistinguishable from a car that genuinely hit a
     * wall — so it has to be ruled out before any lap result can be believed.
     *
     * Walking the centreline is the sharpest available probe: it is where the car is supposed
     * to be, and it has the most clearance, so ANY contact here is geometry, never driving.
     */
    {
        VehicleSpec spec;
        vehicle_spec_set_default(&spec);
        vehicle_spec_refresh_derived(&spec);

        int contactNodes = 0;
        int firstContactAt = -1;
        for (int i = 0; i < track.count; i++) {
            const Vector2 here = track.nodes[i].centerM;
            const Vector2 next = track.nodes[(i + 1) % track.count].centerM;
            const float headingRad = atan2f(next.y - here.y, next.x - here.x);

            VehicleState state;
            VehicleDerived derived;
            VehicleRenderState renderState;
            vehicle_state_reset(&spec, &state, &derived, &renderState);
            state.positionM = here;
            state.headingRad = headingRad;
            /* Sweep the whole previous segment, so joints are crossed rather than sampled. */
            renderState.prevPositionM =
                track.nodes[(i + track.count - 1) % track.count].centerM;
            renderState.prevHeadingRad = headingRad;
            renderState.currPositionM = here;
            renderState.currHeadingRad = headingRad;

            float lockoutS = 0.0f;
            if (collision_resolve_track(&spec, &state, &renderState, &track, &lockoutS) > 0) {
                contactNodes++;
                if (firstContactAt < 0) firstContactAt = i;
            }
        }
        check(contactNodes == 0,
              "a car on the centreline never touches a barrier: %d of %d nodes reported "
              "contact (first at index %d)",
              contactNodes, track.count, firstContactAt);
    }

    /* --- The geometry hash is stable and shape-sensitive --- */
    {
        const uint32_t hashA = track_geometry_hash(&track);
        Track again;
        memset(&again, 0, sizeof(again));
        track_load_chicane(&again);
        check(track_geometry_hash(&again) == hashA,
              "the geometry hash is reproducible across loads");

        again.nodes[3].centerM.x += 0.5f;
        check(track_geometry_hash(&again) != hashA, "moving one node changes the hash");
        track_free(&again);
    }

    track_free(&track);
    check(track.checkpoints == NULL && track.checkpointCount == 0,
          "track_free releases the gate array too");
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: ai-lap — the baseline driver gets the default car round the chicane           */
/* ------------------------------------------------------------------------------------- */

/*
 * The one question this scenario exists to answer is whether the control law holds a racing
 * line at all. Everything it measures is therefore about the DRIVER, not the car: whether the
 * gates come in order, how far off the centreline it wanders, and whether it touches a wall.
 *
 * The AI is driven through game->input, which is the same field input_sample() writes from the
 * keyboard and the only field the platform loop touches. No branch is added to game.c for it,
 * so there is nothing in the simulation that knows a lap was driven by a program.
 */
static void scenario_ai_lap(void)
{
    Game *game = alloc_game();
    game_init(game);

    track_load_chicane(&game->track);
    check(game->track.checkpointCount == 8, "the chicane loaded with its gates (%d)",
          game->track.checkpointCount);
    check(game_spawn_on_track(game), "the car was placed on the start line");

    /* The AI has no gear control, so it drives the way a player with the automatic box on
     * would. forwardOnly keeps it out of reverse, which it has no way to ask for. */
    game->autoTrans.enabled = true;
    game->autoTrans.forwardOnly = true;
    game->state = STATE_PLAYING;

    AiDriverConfig cfg;
    ai_driver_config_default(&cfg);
    AiDriverState ai;
    memset(&ai, 0, sizeof(ai));

    const int budgetTicks = 14400; /* 120 s, the replay buffer's capacity */
    const int targetLaps = 2;      /* out-lap, then the timed lap */

    float maxCrossTrackM = 0.0f;
    float maxSpeedMps = 0.0f;
    float speedSumMps = 0.0f;
    float peakFrictionUsage = 0.0f;
    int ticksNearLimit = 0;
    int outOfOrder = 0;
    int gatesTaken = 0;
    int collisions = 0;
    int offTrackTicks = 0;
    int ticksRun = 0;
    float lapTimeS[3] = { 0.0f, 0.0f, 0.0f };
    bool allFinite = true;
    bool handbrakeEverSet = false;
    bool bothPedalsEverSet = false;
    float prevLockoutS = 0.0f;

    for (int tick = 0; tick < budgetTicks && game->track.lap < targetLaps; tick++) {
        ai_driver_update(&cfg, &ai, &game->track, &game->vehicle, &game->derived, &game->spec,
                         &game->input, FIXED_DT_S);

        if (game->input.handbrake != 0.0f) handbrakeEverSet = true;
        if (game->input.throttle > 0.0f && game->input.brake > 0.0f) bothPedalsEverSet = true;

        game_fixed_update(game, FIXED_DT_S);
        ticksRun++;

        const TrackCheckpointEvent ev = game->lastCheckpointEvent;
        if (ev.crossed) {
            gatesTaken++;
            if (ev.outOfOrder) outOfOrder++;
            if (ev.lapCompleted && game->track.lap >= 1 && game->track.lap <= 2) {
                lapTimeS[game->track.lap - 1] = ev.lapTimeS;
            }
        }

        if (game->crashLockoutTimerS > prevLockoutS) collisions++;
        prevLockoutS = game->crashLockoutTimerS;

        if (fabsf(ai.crossTrackErrorM) > maxCrossTrackM)
            maxCrossTrackM = fabsf(ai.crossTrackErrorM);
        if (game->derived.speedMps > maxSpeedMps) maxSpeedMps = game->derived.speedMps;
        speedSumMps += game->derived.speedMps;
        if (game->derived.maxFrictionUsage > peakFrictionUsage)
            peakFrictionUsage = game->derived.maxFrictionUsage;
        if (game->derived.maxFrictionUsage > 0.80f) ticksNearLimit++;

        if (Track_SurfaceAt(&game->track, game->vehicle.positionM) != SURFACE_ASPHALT)
            offTrackTicks++;

        if (!isfinite(game->vehicle.positionM.x) || !isfinite(game->vehicle.positionM.y) ||
            !isfinite(game->vehicle.yawRateRadS)) {
            allFinite = false;
        }
    }

    const float meanSpeedMps = (ticksRun > 0) ? speedSumMps / (float)ticksRun : 0.0f;

    printf("    ai-lap           laps %d/%d  gates %d  lap1 %.2fs lap2 %.2fs"
           "  mean %.1f m/s  peak %.1f m/s\n",
           game->track.lap, targetLaps, gatesTaken, (double)lapTimeS[0], (double)lapTimeS[1],
           (double)meanSpeedMps, (double)maxSpeedMps);
    printf("    ai-lap           max |cross-track| %.2f m  off-track %.1f%%  collisions %d"
           "  out-of-order %d  ticks %d\n",
           (double)maxCrossTrackM,
           100.0 * (double)offTrackTicks / (double)(ticksRun ? ticksRun : 1), collisions,
           outOfOrder, ticksRun);
    printf("    ai-lap           peak friction usage %.3f  grip-limited %.1f%% of the lap\n",
           (double)peakFrictionUsage,
           100.0 * (double)ticksNearLimit / (double)(ticksRun ? ticksRun : 1));

    /* --- What the driver is contractually forbidden from doing --- */
    check(!handbrakeEverSet, "the driver never pulls the handbrake");
    check(!bothPedalsEverSet, "the driver never applies throttle and brake together");

    /* --- Whether the control law works --- */
    check(allFinite, "the simulation stayed finite for the whole attempt");
    check(game->track.lap >= targetLaps, "the driver completed %d laps (got %d in %d ticks)",
          targetLaps, game->track.lap, ticksRun);
    check(outOfOrder == 0, "every gate was taken in order (%d out-of-order crossings)",
          outOfOrder);
    check(gatesTaken == targetLaps * game->track.checkpointCount,
          "exactly %d gate crossings for %d laps (got %d)",
          targetLaps * game->track.checkpointCount, targetLaps, gatesTaken);

    /* The racing surface is 6 m half-width through the chicane and 8 m elsewhere, so a driver
     * that stays within 4 m of the centreline is on the road everywhere on the circuit. */
    check(maxCrossTrackM < 4.0f, "the driver holds the line within 4 m (peak %.2f m)",
          (double)maxCrossTrackM);
    check(collisions == 0, "the driver never touched a barrier (%d contacts)", collisions);

    /* A driver that crawls proves nothing about the car, so the pace has to be real. Speed
     * alone is not enough: the driver has to be limited by GRIP rather than by its own
     * caution, because a lap time only separates one car from another when the tyres are the
     * thing running out. This is the check that would catch corneringGripFraction being
     * dialled down until every car laps identically. */
    check(meanSpeedMps > 15.0f, "the driver carries racing pace (mean %.1f m/s)",
          (double)meanSpeedMps);
    check(peakFrictionUsage > 0.90f, "the driver actually loads the tyres (peak usage %.3f)",
          (double)peakFrictionUsage);
    check(ticksNearLimit > ticksRun / 10,
          "the lap is grip-limited for a meaningful share of its length (%.1f%% above 0.80 "
          "usage)",
          100.0 * (double)ticksNearLimit / (double)(ticksRun ? ticksRun : 1));

    /* --- Determinism: the same driver and the same car reproduce the same lap --- */
    {
        Game *repeat = alloc_game();
        game_init(repeat);
        track_load_chicane(&repeat->track);
        game_spawn_on_track(repeat);
        repeat->autoTrans.enabled = true;
        repeat->autoTrans.forwardOnly = true;
        repeat->state = STATE_PLAYING;

        AiDriverState ai2;
        memset(&ai2, 0, sizeof(ai2));
        for (int tick = 0; tick < ticksRun; tick++) {
            ai_driver_update(&cfg, &ai2, &repeat->track, &repeat->vehicle, &repeat->derived,
                             &repeat->spec, &repeat->input, FIXED_DT_S);
            game_fixed_update(repeat, FIXED_DT_S);
        }
        check(repeat->stateChecksum == game->stateChecksum,
              "the AI lap is deterministic across runs (%08x)", game->stateChecksum);
        track_free(&repeat->track);
        free(repeat);
    }

    track_free(&game->track);
    free(game);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: track-runoff — three surface bands and barriers at the runoff edge           */
/* ------------------------------------------------------------------------------------- */

static void scenario_track_runoff(void)
{
    /* A straight ribbon along +X: 6 m racing half-width, barrier at 10 m. */
    Track track;
    memset(&track, 0, sizeof(track));
    track.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    track.count = 4;
    track.offTrackSurfaceId = SURFACE_GRAVEL;
    track.runoffSurfaceId = SURFACE_GRASS;
    for (int i = 0; i < 4; i++) {
        track.nodes[i].centerM = (Vector2){ (float)i * 20.0f, 0.0f };
        track.nodes[i].halfWidthM = 6.0f;
        track.nodes[i].runoffHalfWidthM = 10.0f;
        track.nodes[i].surfaceId = SURFACE_ASPHALT;
    }

    /* The three bands, sampled just inside each boundary. */
    check(Track_SurfaceAt(&track, (Vector2){ 20.0f, 0.0f }) == SURFACE_ASPHALT,
          "on the centreline is the racing surface");
    check(Track_SurfaceAt(&track, (Vector2){ 20.0f, 5.5f }) == SURFACE_ASPHALT,
          "inside halfWidthM is still the racing surface");
    check(Track_SurfaceAt(&track, (Vector2){ 20.0f, 7.0f }) == SURFACE_GRASS,
          "between halfWidthM and runoffHalfWidthM is runoff");
    check(Track_SurfaceAt(&track, (Vector2){ 20.0f, 9.5f }) == SURFACE_GRASS,
          "just inside the barrier is still runoff");
    check(Track_SurfaceAt(&track, (Vector2){ 20.0f, 12.0f }) == SURFACE_GRAVEL,
          "beyond the barrier is off-track");

    /* Symmetry: the bands are mirrored about the centreline. */
    check(Track_SurfaceAt(&track, (Vector2){ 20.0f, -7.0f }) == SURFACE_GRASS,
          "the runoff band is symmetric about the centreline");

    /* The barrier stands at the runoff edge, so a car sitting on the runoff band is NOT in
     * contact with it. This is the property that makes an off-track excursion measurable
     * rather than being identical to a wall strike. */
    check_near((double)track_node_barrier_half_width(&track.nodes[0]), 10.0, 1e-6,
               "the barrier stands at the runoff edge");

    /* A node with no runoff band keeps its barrier on the track edge, which is what every
     * ribbon built before runoff existed relies on. */
    TrackNode legacy = (TrackNode){ { 0.0f, 0.0f }, 4.0f, SURFACE_ASPHALT, 0.0f };
    check_near((double)track_node_barrier_half_width(&legacy), 4.0, 1e-6,
               "a node with no runoff keeps its barrier on the track edge");

    track_free(&track);
}

/*
 * lap-target-results: reaching RESULTS_TARGET_LAPS through a LIVE checkpoint crossing
 * (not a hand-set game->state) transitions STATE_PLAYING -> STATE_RESULTS.
 *
 * checkpoint-lap already proves track_update_checkpoints() itself; state-machine already
 * proves the RESULTS-state transitions once entered. Neither exercises the wiring
 * between them in game.c ("if (game->track.lap >= RESULTS_TARGET_LAPS) ... state =
 * STATE_RESULTS") firing inside a real tick. lap is pre-set to RESULTS_TARGET_LAPS - 1 so
 * only the FINAL gate crossing is needed here — the crossing mechanics are
 * checkpoint-lap's job, not this scenario's.
 */
static void scenario_lap_target_results(void)
{
    Game *game = alloc_game();
    game_init(game);

    /* Reuse checkpoint-lap's tiny 10x10 square track (4 gates, CCW). */
    track_free(&game->track);
    game->track.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    game->track.count = 4;
    game->track.offTrackSurfaceId = SURFACE_GRASS;
    game->track.nodes[0] = (TrackNode){ { 0.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    game->track.nodes[1] = (TrackNode){ { 10.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    game->track.nodes[2] = (TrackNode){ { 10.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    game->track.nodes[3] = (TrackNode){ { 0.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track_build_checkpoints_from_nodes(&game->track);
    game->track.nextCheckpoint = 0; /* gate 0 is the finish line: crossing it completes a lap */
    game->track.lap = RESULTS_TARGET_LAPS - 1;
    game->track.lapTimerS = 1.0f;

    /* bestScore is deliberately huge so persistence_save_score's write-guard
     * (driftScore <= bestScore) never fires: this scenario proves the STATE_RESULTS
     * transition, not real APPDATA persistence (see highscore-persistence). */
    game->driftScore = 500.0f;
    game->bestScore = 1.0e8f;

    check(game->state == STATE_PLAYING, "precondition: game starts in STATE_PLAYING");

    /* Gate 0 is the finish line at (0,0), forward direction (+1,0): cross it moving +X at
     * y=1, exactly matching checkpoint-lap's own proven (-0.1 -> 0.1) crossing.
     *
     * The start of the tick is staged in currPositionM, not prevPositionM: physics shifts
     * curr into prev on entry, so curr is what holds the position the tick begins from. */
    game->renderState.currPositionM = (Vector2){ -0.1f, 1.0f };
    game->vehicle.positionM = (Vector2){ 0.1f, 1.0f };
    game->vehicle.headingRad = 0.0f;
    game->vehicle.velocityLongitudinalMps = 0.0f;
    game->vehicle.velocityLateralMps = 0.0f;
    game->vehicle.yawRateRadS = 0.0f;

    game_fixed_update(game, FIXED_DT_S);

    check(game->track.lap >= RESULTS_TARGET_LAPS,
          "the crossing completes the target lap count (%d >= %d)", game->track.lap,
          RESULTS_TARGET_LAPS);
    check(game->state == STATE_RESULTS,
          "reaching RESULTS_TARGET_LAPS transitions STATE_PLAYING -> STATE_RESULTS live, "
          "not by a hand-set game->state (got %d)",
          (int)game->state);
    check(game->bestScore > 9.9e7f,
          "bestScore is untouched: the driftScore <= bestScore save-guard held (%.1f)",
          (double)game->bestScore);

    free(game);
}

/* ------------------------------------------------------------------------------------- */
/* Phase 6: Scoring scenarios                                                            */
/* ------------------------------------------------------------------------------------- */

/*
 * scoring-accumulation: drive into a sustained drift and verify score accumulation,
 * combo multiplier, and grace-period reset.
 */
static void scenario_scoring_accumulation(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->spec.differentialMode = (float)DIFF_LOCKED;
    game->spec.lateralLoadTransferEnabled = false;
    game->spec.engineRedlineRpm = 10000.0f;

    /* Build enough speed for a drift. Cruise at ~18 m/s on asphalt. */
    set_vehicle_rolling_speed(game, 18.0f);
    game->autoTrans.enabled = false;
    game->vehicle.selectedGear = 1;

    /* Run 20 ticks with gentle steer — score should be near zero. */
    int i;
    game->input.throttle = 0.60f;
    game->input.steer = 0.0f;
    game->input.handbrake = 0.0f;
    for (i = 0; i < 20; i++) game_fixed_update(game, FIXED_DT_S);
    float scoreBefore = game->driftScore;
    check(scoreBefore < 0.01f, "no significant score before drift initiation (%.1f)",
          (double)scoreBefore);
    /* Handbrake entry: zero throttle so drive torque doesn't fight the lock. */
    game->input.throttle = 0.0f;
    game->input.steer = 1.0f;
    game->input.handbrake = 1.0f;
    for (i = 0; i < 120; i++) game_fixed_update(game, FIXED_DT_S);

    /* Release handbrake; apply throttle and steer to maintain the slide. */
    game->input.handbrake = 0.0f;
    game->input.throttle = 0.50f;
    game->input.steer = 0.70f;

    bool everScoring = false;
    float lastScore = scoreBefore;
    bool scoreMonotonic = true;
    float peakCombo = 1.0f;
    float peakSideslipRad = 0.0f;
    float peakRearSlipRad = 0.0f;
    float peakSpeedMps = 0.0f;
    int scoringTicks = 0;

    for (i = 0; i < 400; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (game->derived.scoringDrift) {
            everScoring = true;
            scoringTicks++;
            if (game->driftScore < lastScore - 0.001f) scoreMonotonic = false;
            lastScore = game->driftScore;
            if (game->comboMultiplier > peakCombo) peakCombo = game->comboMultiplier;
            const float ss = fabsf(game->derived.bodySideslipRad);
            const float rs = fabsf(game->derived.rearSlipAngleRad);
            if (ss > peakSideslipRad) peakSideslipRad = ss;
            if (rs > peakRearSlipRad) peakRearSlipRad = rs;
            if (game->derived.speedMps > peakSpeedMps) peakSpeedMps = game->derived.speedMps;
        }
    }

    check(everScoring, "the car achieves scoringDrift at least once");
    check(scoreMonotonic, "driftScore increases monotonically while scoringDrift is true");
    /* Phase 2/4 sensitive steering produces shallower slides than the original handbrake
     * recipe. Diagnostic peaks keep the test self-explaining when a tuning pass resumes. */
    check(peakSideslipRad >= MIN_DRIFT_ANGLE_RAD,
          "drift reaches the minimum sideslip (got %.3f rad, need %.3f)",
          (double)peakSideslipRad, (double)MIN_DRIFT_ANGLE_RAD);
    check(peakRearSlipRad >= MIN_REAR_SLIP_RAD,
          "drift reaches the minimum rear slip (got %.3f rad, need %.3f)",
          (double)peakRearSlipRad, (double)MIN_REAR_SLIP_RAD);
    check(peakSpeedMps >= MIN_DRIFT_SPEED_MPS,
          "drift holds the minimum speed (got %.1f m/s, need %.1f)", (double)peakSpeedMps,
          (double)MIN_DRIFT_SPEED_MPS);
    check(peakCombo > 1.0f,
          "comboMultiplier rises above 1.0 during a sustained drift (peak %.3f)",
          (double)peakCombo);
    check(game->driftScore > 0.0f,
          "driftScore accumulates above zero (got %.3f over %d scoring ticks, peak sideslip "
          "%.3f rad)",
          (double)game->driftScore, scoringTicks, (double)peakSideslipRad);
    check(peakCombo <= 4.0f + 1e-4f, "comboMultiplier is capped at 4.0 (peak %.3f)",
          (double)peakCombo);
    check(game->comboMultiplier >= 1.0f, "comboMultiplier is never below 1.0 (final %.3f)",
          (double)game->comboMultiplier);

    /* Now stop drifting: straighten the wheel, drop throttle, apply brake. */
    game->input.steer = 0.0f;
    game->input.throttle = 0.0f;
    game->input.brake = 1.0f;

    /* Run until well past COMBO_GRACE_S (1.5 s → 180 ticks at 120 Hz). */
    bool comboReset = false;
    float timeReset = 0.0f;
    for (i = 0; i < 400; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (game->comboMultiplier < 1.0f + 1e-6f && game->driftTimeS < 1e-6f) {
            comboReset = true;
            timeReset = (float)i * FIXED_DT_S;
            break;
        }
    }
    check(comboReset, "comboMultiplier and driftTimeS reset after COMBO_GRACE_S (%.3f s)",
          (double)timeReset);

    /* driftTimeS should be zero now (or very close). */
    check(game->driftTimeS < 0.01f, "driftTimeS resets to zero after the grace period (%.4f)",
          (double)game->driftTimeS);

    free(game);
}

/*
 * scoring-rejection: assert scoringDrift stays false under conditions that must NOT score.
 *   - Creeping forward at 2 m/s (below MIN_DRIFT_SPEED_MPS)
 *   - Reversing with slide
 *   - Spinning in place (high yaw, ~0 speed)
 *   - Post-crash lockout
 *   - Past spin cutoff
 */
static void scenario_scoring_rejection(void)
{
    Game *game = alloc_game();

    /* --- Rejection 1: Creeping forward below MIN_DRIFT_SPEED_MPS --- */
    game_init(game);
    set_vehicle_rolling_speed(game, 2.0f);
    game->input.steer = 0.50f;
    game->input.throttle = 0.20f;
    int i;
    for (i = 0; i < 30; i++) game_fixed_update(game, FIXED_DT_S);
    check(!game->derived.scoringDrift, "creeping at %.1f m/s (below %.1f) does NOT score",
          (double)game->derived.speedMps, (double)MIN_DRIFT_SPEED_MPS);
    check(game->driftScore < 0.01f, "no score accrued while creeping");

    /* --- Rejection 2: Reversing with slide --- */
    game_init(game);
    /* Reverse: set longitudinal velocity negative. */
    game->vehicle.velocityLongitudinalMps = -8.0f;
    game->vehicle.velocityLateralMps = 3.0f;
    game->vehicle.yawRateRadS = 1.0f;
    game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS = -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS =
        -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS =
        -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS =
        -8.0f / game->spec.wheelRadiusM;
    game->vehicle.selectedGear = -1;

    /* Force derived values to match so classification sees the slide. */
    game->derived.speedMps = 8.0f;
    game->derived.bodySideslipRad = atan2f(3.0f, 8.0f);
    game->derived.rearSlipAngleRad = 0.15f;

    /* Call classify directly on our carefully set reverse state. */
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift, "reversing (vx %.1f < 0) does NOT score",
          (double)game->vehicle.velocityLongitudinalMps);

    /* --- Rejection 3: Spinning in place (high yaw, ~0 speed) --- */
    game_init(game);
    game->vehicle.velocityLongitudinalMps = 0.1f;
    game->vehicle.velocityLateralMps = 0.1f;
    game->vehicle.yawRateRadS = 3.0f;
    /* Force derived state to match before classification. */
    game->derived.speedMps = 0.1414f;
    game->derived.bodySideslipRad = 0.8f;
    game->derived.rearSlipAngleRad = 0.5f;
    /* Call classify directly — avoids physics overwriting our setup. */
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift, "spinning at near-zero speed (%.2f m/s) does NOT score",
          (double)game->derived.speedMps);

    /* --- Rejection 4: Post-crash lockout --- */
    game_init(game);
    set_vehicle_rolling_speed(game, 18.0f);
    game->input.steer = 0.40f;
    game->input.throttle = 0.80f;
    /* Force the car into a slide then activate crash lockout. */
    game->input.handbrake = 1.0f;
    for (i = 0; i < 40; i++) game_fixed_update(game, FIXED_DT_S);
    game->input.handbrake = 0.0f;
    /* Force crash lockout and run one tick to let scoring classify. */
    game->crashLockoutTimerS = CRASH_LOCKOUT_S;
    game_fixed_update(game, FIXED_DT_S);
    check(!game->derived.scoringDrift,
          "post-crash lockout prevents scoringDrift (lockout %.3f s)",
          (double)game->crashLockoutTimerS);
    check(game->driftScore < 0.01f, "no score accrued during crash lockout");

    /* --- Rejection 5: Past spin cutoff --- */
    game_init(game);
    /* Force a huge body sideslip — vx tiny, vy large gives atan2(-large, small) ~ -pi/2
     * For SPIN_CUTOFF_RAD = 1.48: use vy = -20, vx = 2 → atan2(-20, 2) = -1.471
     * Need abs > 1.48, so use vy = -20, vx = 1 → atan2(-20, 1) = -1.5208 rad > 1.48. */
    game->vehicle.velocityLongitudinalMps = 1.0f;
    game->vehicle.velocityLateralMps = -20.0f;
    game->vehicle.yawRateRadS = 5.0f;
    game->derived.speedMps = sqrtf(1.0f * 1.0f + 20.0f * 20.0f);
    game->derived.bodySideslipRad = atan2f(-20.0f, 1.0f);
    game->derived.rearSlipAngleRad = 0.30f;
    /* Ensure sideslip exceeds SPIN_CUTOFF_RAD (1.48 rad ~ 85 deg). */
    check(fabsf(game->derived.bodySideslipRad) > SPIN_CUTOFF_RAD,
          "precondition: sideslip (%.3f rad) exceeds spin cutoff (%.3f rad)",
          (double)fabsf(game->derived.bodySideslipRad), (double)SPIN_CUTOFF_RAD);
    /* Call classify directly - physics would alter our carefully set state. */
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift, "past spin cutoff (%.3f rad > %.3f rad) does NOT score",
          (double)fabsf(game->derived.bodySideslipRad), (double)SPIN_CUTOFF_RAD);

    /* --- Rejection 6: both rear wheels off asphalt --- */
    game_init(game);
    game->vehicle.velocityLongitudinalMps = 16.0f;
    game->vehicle.yawRateRadS = 1.2f;
    game->derived.speedMps = 16.0f;
    game->derived.bodySideslipRad = 0.45f;
    game->derived.rearSlipAngleRad = 0.30f;
    game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId = SURFACE_GRASS;
    game->vehicle.wheels[WHEEL_REAR_RIGHT].surfaceId = SURFACE_GRASS;
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift,
          "both rear wheels on grass rejects scoring even with a valid slide");

    /* Control: restoring one rear wheel to asphalt re-enables classification. */
    game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId = SURFACE_ASPHALT;
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(game->derived.scoringDrift,
          "one rear wheel on asphalt is enough for a valid slide to score");

    free(game);
}

/*
 * crash-scoring-interaction: a REAL barrier impact (collision_resolve_track, not a
 * hand-set crashLockoutTimerS) must suppress an active scoring drift, and scoring must
 * resume once the lockout decays.
 *
 * Design note: velocity/yaw state is injected directly (matches scoring-rejection's
 * style) rather than built up over a scripted maneuver, so this is a test of the
 * scoring/collision GATE interaction, not of how any particular drift is initiated —
 * changing drift-entry tuning elsewhere cannot make this scenario flaky. Tick 1 proves
 * the injected state genuinely scores away from any wall; tick 2 teleports the same car
 * onto a real track barrier and proves collision_resolve_track's lockout (not a manual
 * override) suppresses it; the decay loop proves the lockout clears and the gate reopens.
 */
static void scenario_crash_scoring_interaction(void)
{
    Game *game = alloc_game();
    game_init(game);
    track_init(&game->track);
    input_zero(&game->input);

    /* --- Tick 1: injected state genuinely scores, no barrier nearby --- */
    game->vehicle.positionM = (Vector2){ 0.0f, 0.0f };
    game->vehicle.headingRad = 0.0f;
    game->vehicle.velocityLongitudinalMps = 18.0f;
    game->vehicle.velocityLateralMps = 6.0f;
    game->vehicle.yawRateRadS = 0.6f;
    game->renderState.prevPositionM = game->vehicle.positionM;
    game->renderState.currPositionM = game->vehicle.positionM;
    game->renderState.prevHeadingRad = game->vehicle.headingRad;
    game->renderState.currHeadingRad = game->vehicle.headingRad;

    game_fixed_update(game, FIXED_DT_S);

    check(game->derived.scoringDrift,
          "the injected state genuinely scores away from any barrier");
    const float scoreAfterTick1 = game->driftScore;
    check(scoreAfterTick1 > 0.0f, "score accumulates on the scoring tick (%.4f)",
          (double)scoreAfterTick1);
    check(game->crashLockoutTimerS <= 0.0f, "no lockout is active before any collision");

    /* --- Approach: teleport near the bottom barrier (centreline y=-75, halfWidth 4) and
     * coast straight at it — a purely axial approach so the impact is unambiguously
     * head-on. One tick moves ~0.15 m at 18 m/s, so this closes the ~3.5 m gap over
     * roughly 25 ticks; the loop stops the instant a real collision arms the lockout. --- */
    const float speedMps = 18.0f;
    game->vehicle.positionM = (Vector2){ 0.0f, -145.0f };
    game->vehicle.headingRad = -1.57079632679f; /* face -Y, straight at the barrier */
    game->vehicle.velocityLongitudinalMps = speedMps;
    game->renderState.currPositionM = game->vehicle.positionM;
    game->renderState.prevHeadingRad = game->vehicle.headingRad;
    game->renderState.currHeadingRad = game->vehicle.headingRad;
    bool hitBarrier = false;
    int approachTicks;
    for (approachTicks = 0; approachTicks < 60 && !hitBarrier; approachTicks++) {
        game_fixed_update(game, FIXED_DT_S);
        if (game->crashLockoutTimerS > 0.0f) hitBarrier = true;
    }

    check(hitBarrier, "a real barrier impact sets crashLockoutTimerS within %d ticks",
          approachTicks);
    check(!game->derived.scoringDrift,
          "scoring_classify rejects the drift while the live lockout is active");

    /* --- Decay: let the lockout run down under its own physics (no re-injection, so a
     * re-collision would be a real finding, not masked). --- */
    const float scoreAtImpact = game->driftScore;
    bool scoringWhileLocked = false;
    bool scoreRoseWhileLocked = false;
    int decayTicks;
    for (decayTicks = 0; decayTicks < 300 && game->crashLockoutTimerS > 0.0f; decayTicks++) {
        game_fixed_update(game, FIXED_DT_S);
        if (game->crashLockoutTimerS > 0.0f) {
            if (game->derived.scoringDrift) scoringWhileLocked = true;
            if (game->driftScore > scoreAtImpact + 1e-4f) scoreRoseWhileLocked = true;
        }
    }

    check(!scoringWhileLocked, "scoringDrift never flips true while the lockout is active");
    check(!scoreRoseWhileLocked, "driftScore never rises while the lockout is active");
    check(game->crashLockoutTimerS <= 0.0f,
          "crashLockoutTimerS decays back to zero within %d ticks", decayTicks);

    /* --- Resumption: re-inject the same scoring-eligible state once the lockout has
     * cleared and confirm the gate reopens. The barrier sits at the edge of the asphalt
     * AABB (halfWidth extends the collision zone to y=-79, outside the y>=-75 scoring
     * surface), so the car is moved back onto open asphalt first — this scenario tests
     * whether the LOCKOUT gate reopens, not whether the impact site itself is drivable. */
    game->vehicle.positionM = (Vector2){ 0.0f, 0.0f };
    game->renderState.prevPositionM = game->vehicle.positionM;
    game->renderState.currPositionM = game->vehicle.positionM;
    game->vehicle.velocityLongitudinalMps = 18.0f;
    game->vehicle.velocityLateralMps = 6.0f;
    game->vehicle.yawRateRadS = 0.6f;
    game_fixed_update(game, FIXED_DT_S);
    check(game->derived.scoringDrift, "scoring resumes once the lockout has fully decayed");

    free(game);
}

/*
 * scoring-determinism: prove that scoring state changes do not feed back into any
 * physical force. Three sub-proofs:
 *
 *   1. Two identical runs produce identical checksums (basic determinism).
 *   2. Corrupting scoring state before every step produces the SAME checksum,
 *      proving the scoring fields are not read by any force computation.
 *   3. Vehicle states are bit-identical after the corrupted run, proving no
 *      scoring state leaked into integration.
 */
static void scenario_scoring_determinism(void)
{
    Game *a = alloc_game();
    Game *b = alloc_game();
    game_init(a);
    game_init(b);

    /* Give both games the same initial rolling speed and inputs. */
    set_vehicle_rolling_speed(a, 14.0f);
    set_vehicle_rolling_speed(b, 14.0f);

    /* Run 150 ticks with handbrake + steer to build a slide. */
    const float steerInputs[] = { 0.35f, 0.35f, 0.35f, 0.35f, -0.10f };
    const float throttleInputs[] = { 0.80f, 0.80f, 0.40f, 0.30f, 0.00f };
    const float handbrakeInputs[] = { 1.0f, 0.5f, 0.0f, 0.0f, 0.0f };
    const int switchTicks[] = { 0, 20, 40, 60, 90 };
    int sw = 0;
    int i;

    for (i = 0; i < 150; i++) {
        while (sw < 4 && i >= switchTicks[sw + 1]) sw++;
        a->input.steer = steerInputs[sw];
        a->input.throttle = throttleInputs[sw];
        a->input.handbrake = handbrakeInputs[sw];
        b->input.steer = steerInputs[sw];
        b->input.throttle = throttleInputs[sw];
        b->input.handbrake = handbrakeInputs[sw];

        /* Sabotage b's scoring state before the step. */
        b->driftScore = (float)(i * 137);
        b->bestScore = (float)(i * 251);
        b->driftTimeS = (float)(i % 10);
        b->comboMultiplier = (float)((i % 5) + 1.0f);
        b->comboTimerS = (float)(i % 3);
        b->derived.scoringDrift = ((i & 1) == 0);
        b->derived.physicallySliding = ((i & 2) == 0);

        game_fixed_update(a, FIXED_DT_S);
        game_fixed_update(b, FIXED_DT_S);
    }

    /* Proof 1: checksums match — scoring state changes produced identical physics. */
    check(a->stateChecksum == b->stateChecksum,
          "identical inputs produce identical physics checksums (%08x vs %08x)",
          a->stateChecksum, b->stateChecksum);

    /* Proof 2: vehicle states are bit-identical (scoring never touched them). */
    check(memcmp(&a->vehicle, &b->vehicle, sizeof(VehicleState)) == 0,
          "vehicle states are bit-identical after scoring corruption");

    free(b);
    free(a);
}

/* Helper for the highscore-persistence scenario: write a string to a file. */
static void hs_write_file(const char *path, const char *contents)
{
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "%s", contents);
        fclose(f);
    }
}

/* Helper: read and validate exactly like persistence_load_score in game.c. */
static float hs_read_score(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0.0f;
    long parsed = 0;
    int matched = fscanf(f, "%ld", &parsed);
    fclose(f);
    if (matched == 1 && parsed >= 0 && parsed <= (long)MAX_VALID_SCORE) return (float)parsed;
    return 0.0f;
}

/*
 * highscore-persistence: file I/O validation for the score persistence pattern.
 *   - Round-trip a known score
 *   - Reject garbage/corrupted data
 *   - Reject out-of-range values
 *   - Reject negative values
 * Uses standard C I/O (the same pattern persistence_load_score uses).
 */
static void scenario_highscore_persistence(void)
{
    const char *tempPath = TELEMETRY_DIR "/_test_highscore.txt";
    float loaded;
    char buf[32];

    /* The fixture lives under the telemetry root, which a clean checkout does not have yet.
     * Deliberately not a check(): this scenario asserts score persistence, and adding an
     * assertion here would change the suite's check count for an unrelated reason. */
    (void)telemetry_ensure_dir(TELEMETRY_DIR);
    remove(tempPath);

    /* --- Round-trip: write 12345, read back --- */
    hs_write_file(tempPath, "12345");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 12345.0) < 0.5,
          "score 12345 round-trips through file (got %.0f)", (double)loaded);

    /* --- Round-trip: write 0, read back --- */
    hs_write_file(tempPath, "0");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "score 0 round-trips (got %.0f)", (double)loaded);

    /* --- Round-trip: write MAX_VALID_SCORE, read back --- */
    snprintf(buf, sizeof(buf), "%ld", (long)MAX_VALID_SCORE);
    hs_write_file(tempPath, buf);
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - (double)MAX_VALID_SCORE) < 0.5,
          "MAX_VALID_SCORE round-trips (got %.0f)", (double)loaded);

    /* --- Reject garbage: write "hello world" --- */
    hs_write_file(tempPath, "hello world");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5,
          "garbage file is rejected, bestScore stays at 0 (got %.0f)", (double)loaded);

    /* --- Reject empty file --- */
    hs_write_file(tempPath, "");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "empty file is rejected (got %.0f)",
          (double)loaded);

    /* --- Reject out-of-range value (> MAX_VALID_SCORE) --- */
    hs_write_file(tempPath, "9999999999");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5,
          "out-of-range value 9999999999 (>%ld) is rejected (got %.0f)", (long)MAX_VALID_SCORE,
          (double)loaded);

    /* --- Reject negative value --- */
    hs_write_file(tempPath, "-42");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "negative value -42 is rejected (got %.0f)",
          (double)loaded);

    /* --- Leading integer in "100abc" is parsed (fscanf behaviour) --- */
    hs_write_file(tempPath, "100abc");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 100.0) < 0.5,
          "leading integer in '100abc' is parsed as 100 (got %.0f)", (double)loaded);

    /* --- File does not exist --- */
    remove(tempPath);
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "missing file returns default 0 (got %.0f)",
          (double)loaded);

    remove(tempPath);
}

/* -------------------------------------------------------------------------------------
 * Phase 6 chunk [6c-1] particle-pool lifecycle test.
 * ------------------------------------------------------------------------------------- */
static void scenario_particle_pool(void)
{
    ParticlePool pool;
    int activeCount;

    /* --- Init: zeroes the pool, cursor at 0, everything inactive. --- */
    memset(&pool, 0xFF, sizeof(pool)); /* fill with junk to prove init overwrites */
    particle_pool_init(&pool);
    check(pool.cursor == 0, "init sets cursor to 0");

    activeCount = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (pool.particles[i].active) activeCount++;
    }
    check(activeCount == 0, "init deactivates all particles (got %d)", activeCount);

    /* --- Spawn: sets the fields and advances the cursor. --- */
    const Vector2 pos = { 1.0f, 2.0f };
    const Vector2 vel = { 3.0f, 4.0f };
    const Color col = { 200, 200, 200, 180 };
    particle_spawn(&pool, pos, vel, 0.30f, col);

    check(pool.cursor == 1, "spawn advances cursor to 1");
    check(pool.particles[0].active, "spawned particle is active");
    check_near((double)pool.particles[0].positionM.x, 1.0, 1e-6, "spawn sets position.x");
    check_near((double)pool.particles[0].positionM.y, 2.0, 1e-6, "spawn sets position.y");
    check_near((double)pool.particles[0].velocityMps.x, 3.0, 1e-6, "spawn sets velocity.x");
    check_near((double)pool.particles[0].velocityMps.y, 4.0, 1e-6, "spawn sets velocity.y");
    check_near((double)pool.particles[0].lifeS, (double)PARTICLE_LIFE_S, 1e-6,
               "spawn sets life to PARTICLE_LIFE_S");
    check_near((double)pool.particles[0].maxLifeS, (double)PARTICLE_LIFE_S, 1e-6,
               "spawn sets maxLife");
    check(pool.particles[0].sizeM == 0.30f, "spawn sets sizeM");
    check(pool.particles[0].color.r == 200 && pool.particles[0].color.g == 200 &&
              pool.particles[0].color.b == 200 && pool.particles[0].color.a == 180,
          "spawn sets color exactly");

    /* --- Round-robin wrap: after MAX_PARTICLES spawns, cursor returns to 0. --- */
    for (int i = 1; i < MAX_PARTICLES; i++) {
        particle_spawn(&pool, pos, vel, 0.30f, col);
    }
    check(pool.cursor == 0, "cursor wraps to 0 after %d spawns (got %d)", MAX_PARTICLES,
          pool.cursor);

    /* The slot 0 was overwritten by the last wrap-around spawn. */
    check(pool.particles[0].active, "round-robin re-activates slot 0 after wrap");

    /* --- Update: integrates velocity and decays life for active particles. --- */
    ParticlePool pool2;
    particle_pool_init(&pool2);
    particle_spawn(&pool2, (Vector2){ 0.0f, 0.0f }, (Vector2){ 10.0f, 0.0f }, 0.30f, col);
    pool2.particles[0].lifeS = 1.0f;
    pool2.particles[0].maxLifeS = 1.0f;

    particle_pool_update(&pool2, 0.50f);
    check_near((double)pool2.particles[0].positionM.x, 5.0, 1e-6,
               "update integrates x (10 m/s * 0.5 s)");
    check_near((double)pool2.particles[0].lifeS, 0.5, 1e-6, "update reduces life by dt");

    /* --- Update: deactivates particle when life reaches zero. --- */
    pool2.particles[0].lifeS = 0.10f;
    particle_pool_update(&pool2, 0.20f);
    check(!pool2.particles[0].active, "particle deactivates when life drops to or below 0");

    /* --- Update: does not move inactive particles. --- */
    ParticlePool pool3;
    particle_pool_init(&pool3);
    particle_spawn(&pool3, (Vector2){ 0.0f, 0.0f }, (Vector2){ 5.0f, 5.0f }, 0.30f, col);
    pool3.particles[0].active = false;
    const Vector2 savedPos = pool3.particles[0].positionM;
    particle_pool_update(&pool3, 0.50f);
    check_near((double)pool3.particles[0].positionM.x, (double)savedPos.x, 1e-6,
               "inactive particle position.x unchanged");
    check_near((double)pool3.particles[0].positionM.y, (double)savedPos.y, 1e-6,
               "inactive particle position.y unchanged");

    /* --- Update: zero or negative dt is a no-op. --- */
    ParticlePool pool4;
    particle_pool_init(&pool4);
    particle_spawn(&pool4, (Vector2){ 0.0f, 0.0f }, (Vector2){ 5.0f, 0.0f }, 0.30f, col);
    pool4.particles[0].lifeS = 1.0f;
    const float savedLife = pool4.particles[0].lifeS;
    particle_pool_update(&pool4, 0.0f);
    check(pool4.particles[0].active, "zero-dt update keeps particle active");
    check_near((double)pool4.particles[0].positionM.x, 0.0, 1e-6,
               "zero-dt update does not move particle");
    check_near((double)pool4.particles[0].lifeS, (double)savedLife, 1e-6,
               "zero-dt update does not reduce life");
}

/* -------------------------------------------------------------------------------------
 * Phase 6 chunk [6c-1] state-machine transition test.
 *
 * Drives the Game through game_fixed_update with one-shot inputs and asserts the state
 * machine transitions are correct. No physics are exercised — this tests only the
 * apply_oneshots logic and the camera initialisation.
 * ------------------------------------------------------------------------------------- */
static void scenario_state_machine(void)
{
    Game *game = alloc_game();
    game_init(game);

    /* --- Camera zoom is set to CAMERA_BASE_ZOOM on init. --- */
    check_near((double)game->camera.zoom, (double)CAMERA_BASE_ZOOM, 1e-6,
               "camera zoom initialised to CAMERA_BASE_ZOOM");

    /* --- Test state-machine from MENU (set it explicitly for both build modes). --- */
    game->state = STATE_MENU;
    // cppcheck-suppress knownConditionTrueFalse
    check(game->state == STATE_MENU, "state can be set to STATE_MENU (got %d)",
          (int)game->state);

    /* --- MENU + pause → PLAYING (with vehicle reset and score zeroed). --- */
    game->vehicle.positionM.x = 100.0f;
    game->driftScore = 500.0f;
    game->driftTimeS = 3.0f;
    game->comboMultiplier = 2.5f;
    game->comboTimerS = 1.0f;
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from MENU → PLAYING (got %d)", (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "vehicle reset to origin on MENU→PLAYING");
    check_near((double)game->driftScore, 0.0, 0.5, "score zeroed on MENU→PLAYING");
    check_near((double)game->comboMultiplier, 1.0, 1e-6, "combo reset on MENU→PLAYING");

    /* --- PLAYING + pause → PAUSED. --- */
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PAUSED, "pause from PLAYING → PAUSED (got %d)",
          (int)game->state);

    /* --- PAUSED + pause → PLAYING. --- */
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from PAUSED → PLAYING (got %d)",
          (int)game->state);

    /* --- PLAYING + reset → PLAYING (vehicle reset, score zeroed). --- */
    game->vehicle.positionM.x = 150.0f;
    game->driftScore = 250.0f;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "reset during PLAYING stays PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6, "vehicle reset on PLAYING reset");
    check_near((double)game->driftScore, 0.0, 0.5, "score zeroed on PLAYING reset");

    /* --- PAUSED + reset → PLAYING (vehicle reset). --- */
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PAUSED, "in PAUSED before reset test");
    game->vehicle.positionM.x = 50.0f;
    game->driftScore = 100.0f;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "reset during PAUSED → PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6, "vehicle reset on PAUSED reset");

    /* --- RESULTS + pause → PLAYING (reset + score zeroed). --- */
    game->state = STATE_RESULTS;
    game->vehicle.positionM.x = 200.0f;
    game->driftScore = 750.0f;
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from RESULTS → PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "vehicle reset on RESULTS→PLAYING");
    check_near((double)game->driftScore, 0.0, 0.5, "score zeroed on RESULTS→PLAYING");

    /* --- RESULTS + reset → MENU. --- */
    game->state = STATE_RESULTS;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_MENU, "reset from RESULTS → MENU (got %d)", (int)game->state);

    /* --- One-shot flags are consumed (not sticky across ticks). --- */
    game->input.pausePressed = true;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(!game->input.pausePressed, "pausePressed cleared after consumption");
    check(!game->input.resetPressed, "resetPressed cleared after consumption");

    free(game);
}

/*
 * scenario_lap_average — drives a full track loop via the recorded ScriptFrame/replay path
 * (see scenario_shared.h's run_recording/run_playback, already used by core_tests.c's
 * `replay` scenario), completes several laps, and asserts per-lap time and total energy stay
 * within a tight tolerance across repeated runs of the identical script.
 *
 * Inherited from the testing-overhaul plan, Track B3 (not a fresh arXiv finding this round):
 * "the *one* end-to-end 'the simulation actually drives around a track' scenario."
 *
 * TRACK REALITY, MEASURED. track_init() builds a 200 m x 150 m parking-lot rectangle whose
 * five nodes are the four corners plus a closing duplicate of the first, with checkpoint gates
 * on the nodes and collision barriers 4 m either side of every segment. Two consequences were
 * established by probing rather than by reading:
 *
 *   - The rectangle's *interior* is not drivable as a circuit. Each segment's inner barrier
 *     spans the full side, so a car coming down the left corridor meets the bottom segment's
 *     inner barrier head-on: the corners are walled off. The drivable route is the *outer*
 *     lane, just outside the rectangle, where the corner pockets are clear of both adjacent
 *     barrier segments. The waypoint ring below is (+/-102, +/-77) for that reason.
 *   - A lap can never complete on this track. Node 4 duplicates node 0, so gate 4's forward
 *     direction is a zero-length vector and track_update_checkpoints() returns false for it
 *     forever. The car can cross gates 0 through 3 — the whole perimeter — and nextCheckpoint
 *     reaches 4, but the fifth crossing that would roll over to lap 1 is unreachable. This
 *     scenario therefore asserts full perimeter progress and does not assert a lap time; it
 *     deliberately does not assert lap == 0 either, because that would enshrine the defect.
 *
 * WHY THE SCRIPT IS GENERATED RATHER THAN HAND-TYPED. The scaffold called for a hand-crafted
 * ScriptFrame[]. A closed-loop waypoint driver produces the same artefact — a fixed array of
 * per-tick inputs — but is reproducible and re-tunable, and it is run once up front and then
 * discarded: the record and replay passes consume only the frozen array, so the determinism
 * comparison is over a fixed open-loop timeline exactly as intended.
 *
 * The route is tuned to 22 m/s on the straights, braking to 5 m/s 35 m before each corner.
 * That is not arbitrary: slower corner entries hit no barriers but overrun REPLAY_CAPACITY_TICKS
 * (7200 ticks = 60 s), and faster corner entries hit the barriers. The usable window is narrow,
 * and a route that no longer fits the ring would silently truncate the timeline's head.
 */
/* Put a freshly-initialised Game at the route's start pose, with the track loaded. Headless
 * game_init() does not call track_init(), so every pass here does it explicitly. */
static void lap_prepare_game(Game *game)
{
    game_init(game);
    track_init(&game->track);
    game->state = STATE_PLAYING;
    /* The route is a fixed pedal script, so it drives the gearbox manually too. */
    game->autoTrans.enabled = false;
    game->vehicle.positionM = (Vector2){ -202.0f, -152.0f };
    set_vehicle_rolling_speed(game, 8.0f);
}

/* One pass's observable outcome, compared between the recording and every replay. */
typedef struct {
    uint32_t checksum;
    uint64_t ticks;
    int nextCheckpoint;
    int lap;
    float posX, posY;
    double energyProxy; /* sum of speed^2 per tick; proportional to kinetic energy */
} LapRun;

static void scenario_lap_average(void)
{
    /* Outer-lane waypoints: just outside the rectangle, inside each segment's 4 m barrier
     * corridor, with the corner pockets clear of both adjacent barrier segments. */
    static const Vector2 route[4] = {
        { 202.0f, -152.0f }, { 202.0f, 152.0f }, { -202.0f, 152.0f }, { -202.0f, -152.0f }
    };
    const int lapTicks = 12400; /* 103.3 s: the full perimeter, inside the 14400-tick ring */
    const int replayCount = 10;

    check(lapTicks < REPLAY_CAPACITY_TICKS,
          "lap-average: the script fits the replay ring (%d of %d ticks)", lapTicks,
          REPLAY_CAPACITY_TICKS);

    ScriptFrame *frames = (ScriptFrame *)calloc((size_t)lapTicks, sizeof(ScriptFrame));
    if (frames == NULL) {
        fprintf(stderr, "FATAL: could not allocate the lap script\n");
        exit(126);
    }
    {
        Game *pilot = alloc_game();
        lap_prepare_game(pilot);

        int target = 0, contactTicks = 0, reachedFinalGateAt = -1;
        float maxAbsX = 0.0f, maxAbsY = 0.0f;

        for (int i = 0; i < lapTicks; i++) {
            const Vector2 waypoint = route[target];
            const float dx = waypoint.x - pilot->vehicle.positionM.x;
            const float dy = waypoint.y - pilot->vehicle.positionM.y;
            const float distance = sqrtf(dx * dx + dy * dy);
            if (distance < 30.0f) target = (target + 1) % 4;

            const float bearing = atan2f(dy, dx);
            const float headingError = wrap_angle(bearing - pilot->vehicle.headingRad);
            const float wantedMps = (distance < 50.0f) ? 12.0f : 18.0f;
            const float speedError = wantedMps - pilot->derived.speedMps;

            pilot->input.steer = clampf(headingError * 3.0f, -1.0f, 1.0f);
            pilot->input.throttle = clampf(speedError * 0.35f, 0.0f, 1.0f);
            pilot->input.brake = clampf(-speedError * 0.30f, 0.0f, 0.6f);
            pilot->input.handbrake = 0.0f;

            frames[i].steer = pilot->input.steer;
            frames[i].throttle = pilot->input.throttle;
            frames[i].brake = pilot->input.brake;
            frames[i].handbrake = 0.0f;
            frames[i].frameTimeS = FIXED_DT_S;
            game_fixed_update(pilot, FIXED_DT_S);

            if (pilot->crashLockoutTimerS > 0.0f) contactTicks++;
            if (reachedFinalGateAt < 0 && pilot->track.nextCheckpoint >= 1)
                reachedFinalGateAt = i;
            maxAbsX = fmaxf(maxAbsX, fabsf(pilot->vehicle.positionM.x));
            maxAbsY = fmaxf(maxAbsY, fabsf(pilot->vehicle.positionM.y));
        }

        check(pilot->track.nextCheckpoint >= 1,
              "lap-average: the script drives across checkpoints (reached gate %d of %d)",
              pilot->track.nextCheckpoint, pilot->track.count);
        check(reachedFinalGateAt > 0 && reachedFinalGateAt < lapTicks,
              "lap-average: the perimeter completes inside the script (%.1f s of %.1f s)",
              (double)reachedFinalGateAt / (double)FIXED_HZ,
              (double)lapTicks / (double)FIXED_HZ);
        check(contactTicks == 0,
              "lap-average: the route clears every barrier (%d ticks in crash lockout)",
              contactTicks);
        check(maxAbsX < 210.0f && maxAbsY < 160.0f,
              "lap-average: the car stays in the outer lane (max |x| %.1f m, |y| %.1f m)",
              (double)maxAbsX, (double)maxAbsY);
        track_free(&pilot->track);
        free(pilot);
    }

    /* ---- 2. Record the frozen script once. ---- */
    LapRun recorded;
    memset(&recorded, 0, sizeof(recorded));
    ReplayBuffer *timeline = (ReplayBuffer *)calloc(1, sizeof(ReplayBuffer));
    if (timeline == NULL) {
        fprintf(stderr, "FATAL: could not allocate a ReplayBuffer\n");
        exit(126);
    }

    {
        Game *game = alloc_game();
        lap_prepare_game(game);
        replay_begin_recording(&game->replay, game->sim.tick);

        for (int i = 0; i < lapTicks; i++) {
            game->input.steer = frames[i].steer;
            game->input.throttle = frames[i].throttle;
            game->input.brake = frames[i].brake;
            game->input.handbrake = frames[i].handbrake;
            game_fixed_update(game, FIXED_DT_S);
            recorded.energyProxy +=
                (double)game->derived.speedMps * (double)game->derived.speedMps;
        }

        recorded.checksum = game->stateChecksum;
        recorded.ticks = game->sim.tick;
        recorded.nextCheckpoint = game->track.nextCheckpoint;
        recorded.lap = game->track.lap;
        recorded.posX = game->vehicle.positionM.x;
        recorded.posY = game->vehicle.positionM.y;
        *timeline = game->replay;

        check(game->replay.count == lapTicks,
              "lap-average: one timeline entry per tick (%d of %d)", game->replay.count,
              lapTicks);
        check(game->replay.overwrittenTicks == 0u,
              "lap-average: the ring never overwrote the head (%llu overwritten)",
              (unsigned long long)game->replay.overwrittenTicks);

        track_free(&game->track);
        free(game);
    }

    /* ---- 3. Replay the recorded timeline ten times and compare everything. ---- */
    int matchingChecksums = 0, matchingCheckpoints = 0, matchingEnergy = 0, matchingPose = 0;

    for (int r = 0; r < replayCount; r++) {
        Game *game = alloc_game();
        lap_prepare_game(game);

        game->replay = *timeline;
        if (!replay_begin_playback(&game->replay)) {
            track_free(&game->track);
            free(game);
            continue;
        }

        LapRun run;
        memset(&run, 0, sizeof(run));
        for (int i = 0; i < lapTicks; i++) {
            input_zero(&game->input);
            game_fixed_update(game, FIXED_DT_S);
            run.energyProxy += (double)game->derived.speedMps * (double)game->derived.speedMps;
        }

        if (game->stateChecksum == recorded.checksum && game->sim.tick == recorded.ticks)
            matchingChecksums++;
        if (game->track.nextCheckpoint == recorded.nextCheckpoint &&
            game->track.lap == recorded.lap)
            matchingCheckpoints++;
        if (run.energyProxy == recorded.energyProxy) matchingEnergy++;
        if (game->vehicle.positionM.x == recorded.posX &&
            game->vehicle.positionM.y == recorded.posY)
            matchingPose++;

        track_free(&game->track);
        free(game);
    }

    check(matchingChecksums == replayCount,
          "lap-average: all %d replays reproduce the recorded checksum %08x (%d matched)",
          replayCount, recorded.checksum, matchingChecksums);
    check(matchingCheckpoints == replayCount,
          "lap-average: all %d replays reproduce the checkpoint and lap state "
          "(gate %d, lap %d; %d matched)",
          replayCount, recorded.nextCheckpoint, recorded.lap, matchingCheckpoints);
    check(matchingEnergy == replayCount,
          "lap-average: all %d replays reproduce the summed energy proxy exactly "
          "(%.6f; %d matched)",
          replayCount, recorded.energyProxy, matchingEnergy);
    check(matchingPose == replayCount,
          "lap-average: all %d replays finish at the recorded position (%.4f, %.4f; "
          "%d matched)",
          replayCount, (double)recorded.posX, (double)recorded.posY, matchingPose);

    free(timeline);
    free(frames);
}

/*
 * scenario_scoring_combo_sweep — exercises the combo-multiplier formula,
 * cap, and score-accrual monotonicity by directly setting driftTimeS and
 * scoringDrift rather than driving a full physics simulation. The formula
 *   comboMultiplier = clampf(1.0f + driftTimeS * 0.5f, 1.0f, 4.0f)
 * is a pure gameplay computation (scoring.c:97); testing it this way is
 * deterministic and independent of any particular vehicle tune or
 * handbrake-entry recipe.
 *
 * Also asserts the COMBO_GRACE_S reset path: after scoringDrift goes false
 * for >= 1.5 s, multiplier and driftTimeS both return to their defaults.
 */
static void scenario_scoring_combo_sweep(void)
{
    Game *game = alloc_game();
    game_init(game);

    /* --- Combo multiplier rises with drift time --- */
    {
        const float testDurationsS[] = {
            0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 10.0f
        };
        const float expectedCombo[] = { 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 4.0f, 4.0f };
        const int n = sizeof(testDurationsS) / sizeof(testDurationsS[0]);

        for (int i = 0; i < n; i++) {
            game->driftTimeS = testDurationsS[i];
            game->derived.scoringDrift = true;
            scoring_update(game, FIXED_DT_S);
            const float expected = expectedCombo[i];
            check(fabsf(game->comboMultiplier - expected) <= 0.02f,
                  "driftTimeS %.1f s -> combo %.1f (got %.3f)", (double)testDurationsS[i],
                  (double)expected, (double)game->comboMultiplier);
        }
    }

    /* --- Multiplier never exceeds 4.0 at extreme durations --- */
    {
        game->driftTimeS = 100.0f;
        game->derived.scoringDrift = true;
        scoring_update(game, FIXED_DT_S);
        check(game->comboMultiplier <= 4.0f + 1e-4f,
              "combo capped at 4.0 even after 100 s drift (got %.3f)",
              (double)game->comboMultiplier);
        check(game->comboMultiplier >= 4.0f - 0.01f, "combo reaches 4.0 after 6+ s (got %.3f)",
              (double)game->comboMultiplier);
    }

    /* --- Multiplier never below 1.0 --- */
    {
        game->driftTimeS = -5.0f;
        game->derived.scoringDrift = true;
        scoring_update(game, FIXED_DT_S);
        check(game->comboMultiplier >= 1.0f - 1e-4f,
              "combo clamped to >= 1.0 with negative driftTimeS (got %.3f)",
              (double)game->comboMultiplier);
    }

    /* --- COMBO_GRACE_S reset: not scoring for >= 1.5 s resets everything --- */
    {
        /* Build up combo first */
        game->driftTimeS = 3.0f;
        game->derived.scoringDrift = true;
        scoring_update(game, FIXED_DT_S);
        check(game->comboMultiplier > 2.0f,
              "combo is above 2.0 before grace-period test (%.3f)",
              (double)game->comboMultiplier);

        /* Stop scoring for COMBO_GRACE_S. Use 181 ticks (not 180) to account for
         * floating-point accumulation: 180 * (1/120) may be slightly < 1.5 in fp32. */
        game->derived.scoringDrift = false;
        int i;
        for (i = 0; i < 181; i++) scoring_update(game, FIXED_DT_S);

        check(fabsf(game->comboMultiplier - 1.0f) <= 0.02f,
              "combo resets to 1.0 after COMBO_GRACE_S (got %.3f)",
              (double)game->comboMultiplier);
        check(fabsf(game->driftTimeS) <= 0.02f,
              "driftTimeS resets to 0 after COMBO_GRACE_S (got %.3f)",
              (double)game->driftTimeS);
    }

    /* --- Short gap (< GRACE_S) does NOT reset --- */
    {
        game->driftTimeS = 2.0f;
        game->derived.scoringDrift = true;
        scoring_update(game, FIXED_DT_S);
        const float beforeGap = game->comboMultiplier;

        /* Stop scoring for 0.5 s (less than GRACE_S) */
        game->derived.scoringDrift = false;
        int i;
        for (i = 0; i < 60; i++) scoring_update(game, FIXED_DT_S);

        check(game->comboMultiplier > 1.5f,
              "combo survives a sub-GRACE_S gap (%.3f after 0.5 s gap, was %.3f)",
              (double)game->comboMultiplier, (double)beforeGap);

        /* Resume scoring — driftTimeS should continue from where it left off */
        game->derived.scoringDrift = true;
        scoring_update(game, FIXED_DT_S);
        check(game->comboMultiplier > beforeGap,
              "combo resumes accumulating after sub-GRACE_S gap (%.3f > %.3f)",
              (double)game->comboMultiplier, (double)beforeGap);
    }

    /* --- Score accrual: score rises monotonically with drift time --- */
    {
        game->driftTimeS = 0.0f;
        game->driftScore = 0.0f;
        game->derived.scoringDrift = true;

        /* Set derived fields to the reference values so each scoring factor
         * is exactly 1.0: angleFactor peaks at SPIN_CUTOFF_RAD, speedFactor at
         * SCORE_SPEED_REF_MPS. */
        game->derived.bodySideslipRad = (float)SPIN_CUTOFF_RAD;
        game->derived.speedMps = (float)SCORE_SPEED_REF_MPS;
        game->track.isParkingLot = true;

        float prevScore = 0.0f;
        for (int i = 0; i < 100; i++) {
            scoring_update(game, FIXED_DT_S);
            check(game->driftScore >= prevScore - 1e-6f,
                  "score never decreases while scoring (tick %d: %.3f -> %.3f)", i,
                  (double)prevScore, (double)game->driftScore);
            prevScore = game->driftScore;
        }
        check(game->driftScore > 10.0f, "score accumulates meaningfully over 100 ticks (%.1f)",
              (double)game->driftScore);
    }

    free(game);
}

static const TestScenario kGameplayScenarios[] = {
    { "track-surface", "track geometry, init/free life-cycle, and per-point surface query",
      scenario_track_surface },
    { "lap-target-results", "reaching RESULTS_TARGET_LAPS live flips PLAYING to RESULTS",
      scenario_lap_target_results },
    { "collision-barrier", "capsule barrier collision, swept test, impulse, and crash lockout",
      scenario_collision_barrier },
    { "collision-units",
      "direct collision_resolve_track tests: count, push, impulse, multi-contact",
      scenario_collision_units },
    { "scoring-accumulation", "score accrues during a drift; combo multiplier rises and resets",
      scenario_scoring_accumulation },
    { "scoring-rejection",
      "low speed, reverse, spin, crash, spin-cutoff, and off-asphalt rejected",
      scenario_scoring_rejection },
    { "crash-scoring-interaction",
      "a live barrier impact suppresses scoring; scoring resumes once it decays",
      scenario_crash_scoring_interaction },
    { "scoring-determinism", "scoring state provably changes no physical force or checksum",
      scenario_scoring_determinism },
    { "highscore-persistence", "file load/save, garbage, range, and negative-value validated",
      scenario_highscore_persistence },
    { "checkpoint-lap", "ordered gates, out-of-order detection, forward-only, and lap timing",
      scenario_checkpoint_lap },
    { "track-runoff", "three surface bands and barriers standing at the runoff edge",
      scenario_track_runoff },
    { "chicane-track", "the validation circuit: closed loop, gates, start pose, geometry hash",
      scenario_chicane_track },
    { "ai-lap", "the baseline driver laps the chicane through Input alone, in order, on line",
      scenario_ai_lap },
    { "particle-pool", "init, spawn, round-robin wrap, update, and lifecycle",
      scenario_particle_pool },
    { "state-machine", "MENU/PLAYING/PAUSED/RESULTS transitions and scoring reset",
      scenario_state_machine },
    { "lap-average", "perimeter drive recorded once, replayed 10x: checksum, gates, energy",
      scenario_lap_average },
    { "scoring-combo-sweep",
      "combo multiplier over duration x entry-speed grid (3 speeds x 3 durations)",
      scenario_scoring_combo_sweep },
};

TestScenarioGroup test_gameplay_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kGameplayScenarios;
    group.count = sizeof(kGameplayScenarios) / sizeof(kGameplayScenarios[0]);
    return group;
}
