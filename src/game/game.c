/*
 * game.c — reloadable game entry points and deterministic Phase 2 dispatch.
 *
 * physics.c is the sole owner of vehicle integration. This file retains Phase 0's input,
 * one-shot, replay, checksum, and module-lifecycle guarantees.
 */
#include "game/game.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev/dev_lab.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "game/audio.h"
#include "physics/auto_transmission.h"
#include "world/collision.h"
#include "physics/physics.h"
#include "game/profile.h"
#include "render/render.h"
#include "world/track.h"

#if !defined(DRIFTY_HEADLESS)
#include "raylib.h"
#endif

#if defined(_WIN32)
#include <direct.h>
#define DRIFTY_MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#define DRIFTY_MKDIR(path) mkdir(path, 0755)
#endif

#define FNV1A_OFFSET_BASIS 2166136261u
#define FNV1A_PRIME 16777619u

static uint32_t hash_bytes(uint32_t h, const void *data, size_t length)
{
    const unsigned char *bytes = (const unsigned char *)data;
    for (size_t i = 0; i < length; i++) {
        h ^= (uint32_t)bytes[i];
        h *= FNV1A_PRIME;
    }
    return h;
}

static uint32_t hash_f32(uint32_t h, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return hash_bytes(h, &bits, sizeof(bits));
}

static uint32_t hash_u32(uint32_t h, uint32_t value)
{
    return hash_bytes(h, &value, sizeof(value));
}

static uint32_t hash_u64(uint32_t h, uint64_t value)
{
    return hash_bytes(h, &value, sizeof(value));
}

GAME_API uint32_t game_state_checksum(const Game *game)
{
    if (game == NULL) return 0u;
    uint32_t h = FNV1A_OFFSET_BASIS;
    h = hash_u32(h, (uint32_t)game->state);
    h = hash_u64(h, game->sim.tick);
    h = hash_u32(h, game->sim.resetCount);
    h = hash_u32(h, game->sim.pauseToggleCount);
    h = hash_u32(h, game->sim.debugToggleCount);
    h = hash_u32(h, game->sim.shiftUpCount);
    h = hash_u32(h, game->sim.shiftDownCount);

    const VehicleState *v = &game->vehicle;
    h = hash_f32(h, v->positionM.x);
    h = hash_f32(h, v->positionM.y);
    h = hash_f32(h, v->headingRad);
    h = hash_f32(h, v->velocityLongitudinalMps);
    h = hash_f32(h, v->velocityLateralMps);
    h = hash_f32(h, v->yawRateRadS);
    h = hash_f32(h, v->frontRoadWheelAngleRad);
    h = hash_f32(h, v->engineRpm);
    h = hash_u32(h, (uint32_t)v->selectedGear);
    h = hash_f32(h, v->filteredLongAccelMps2);
    h = hash_f32(h, v->prevLongAccelMps2);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &v->wheels[i];
        h = hash_f32(h, wheel->localPositionM.x);
        h = hash_f32(h, wheel->localPositionM.y);
        h = hash_f32(h, wheel->steerAngleRad);
        h = hash_f32(h, wheel->angularVelocityRadS);
        h = hash_f32(h, wheel->normalLoadN);
        h = hash_f32(h, wheel->slipAngleRad);
        h = hash_f32(h, wheel->slipRatio);
        h = hash_f32(h, wheel->forceLongitudinalN);
        h = hash_f32(h, wheel->forceLateralN);
        h = hash_f32(h, wheel->frictionUsage);
        h = hash_u32(h, wheel->locked ? 1u : 0u);
        h = hash_u32(h, (uint32_t)wheel->surfaceId);
    }
    return h;
}

GAME_API void game_reset_sim(Game *game)
{
    if (game == NULL) return;
    vehicle_state_reset(&game->spec, &game->vehicle, &game->derived, &game->renderState);
    game->autoTrans.driveState = AUTO_DRIVE;
    game->autoTrans.neutralTimer = 0.0f;
}

GAME_API void game_apply_spec(Game *game, const VehicleSpec *spec)
{
    if (game == NULL || spec == NULL) return;
    game->spec = *spec;
    vehicle_state_reset(&game->spec, &game->vehicle, &game->derived, &game->renderState);
}

GAME_API bool game_configure_run(Game *game, const GameRunConfig *config)
{
    if (game == NULL || config == NULL) return false;

    switch (config->track) {
        case GAME_TRACK_PARKING_LOT: track_init(&game->track); break;
        case GAME_TRACK_CHICANE: track_load_chicane(&game->track); break;
        case GAME_TRACK_KEEP: break;
        default: return false;
    }

    game->dev.cameraZoomOverride = config->cameraZoomOverride;

    if (config->track != GAME_TRACK_KEEP) return game_spawn_on_track(game);
    return true;
}

GAME_API bool game_spawn_on_track(Game *game)
{
    if (game == NULL) return false;

    Vector2 startM = { 0.0f, 0.0f };
    float headingRad = 0.0f;
    if (!track_start_pose(&game->track, &startM, &headingRad)) return false;

    /* Reset first, then place: vehicle_state_reset() puts the car at the world origin, so
     * doing it the other way round would throw the pose away. */
    vehicle_state_reset(&game->spec, &game->vehicle, &game->derived, &game->renderState);
    game->vehicle.positionM = startM;
    game->vehicle.headingRad = headingRad;

    /* Render history must agree with the new pose or the first frame interpolates from the
     * origin, and the first checkpoint test sweeps a segment spanning the whole track. */
    game->renderState.prevPositionM = startM;
    game->renderState.currPositionM = startM;
    game->renderState.prevHeadingRad = headingRad;
    game->renderState.currHeadingRad = headingRad;

    track_reset_progress(&game->track);
    memset(&game->lastCheckpointEvent, 0, sizeof(game->lastCheckpointEvent));
    game->lastCheckpointEvent.index = -1;

    game->autoTrans.driveState = AUTO_DRIVE;
    game->autoTrans.neutralTimer = 0.0f;
    return true;
}

static void apply_oneshots(Game *game, const Input *input)
{
    if (input->pausePressed) {
        switch (game->state) {
            case STATE_MENU:
                game_reset_sim(game);
                game->state = STATE_PLAYING;
                break;
            case STATE_PLAYING: game->state = STATE_PAUSED; break;
            case STATE_PAUSED: game->state = STATE_PLAYING; break;
            case STATE_RESULTS:
                game_reset_sim(game);
                game->state = STATE_PLAYING;
                break;
            default: break;
        }
        game->sim.pauseToggleCount++;
    }
    if (input->resetPressed) {
        switch (game->state) {
            case STATE_PLAYING: game_reset_sim(game); break;
            case STATE_PAUSED:
                game_reset_sim(game);
                game->state = STATE_PLAYING;
                break;
            case STATE_RESULTS: game->state = STATE_MENU; break;
            default: break;
        }
        game->sim.resetCount++;
    }
    if (input->debugPressed) {
        game->debugOverlay = !game->debugOverlay;
        game->sim.debugToggleCount++;
    }
    if (input->toggleAutoPressed) {
        game->autoTrans.enabled = !game->autoTrans.enabled;
        if (game->autoTrans.enabled) {
            game->autoTrans.driveState = AUTO_DRIVE;
            game->autoTrans.neutralTimer = 0.0f;
            game->vehicle.selectedGear = 1;
        }
    }
    if (!game->autoTrans.enabled) {
        if (input->shiftUpPressed) {
            if (game->vehicle.selectedGear < game->spec.gearCount) {
                game->vehicle.selectedGear++;
            }
            game->sim.shiftUpCount++;
        }
        if (input->shiftDownPressed) {
            if (game->vehicle.selectedGear > -1) game->vehicle.selectedGear--;
            game->sim.shiftDownCount++;
        }
    }
}

GAME_API void game_init(Game *game)
{
    if (game == NULL) return;
    input_zero(&game->input);
    memset(&game->sim, 0, sizeof(game->sim));
    vehicle_spec_set_default(&game->spec);
    game_reset_sim(game);
    game->autoTrans.enabled = true;
    game->autoTrans.forwardOnly = false;
    game->autoTrans.driveState = AUTO_DRIVE;
#if defined(DRIFTY_HEADLESS)
    game->state = STATE_PLAYING; /* headless: no menus, simulate immediately */
#else
    game->state = STATE_MENU;
#endif
    game->accumulatorS = 0.0f;
    game->lastSubstepCount = 0;
    game->physicsBacklogDrops = 0;
    game->debugOverlay = false;
    game->reloadCount = 0;
    game->reloadFlashTimerS = 0.0f;
    particle_pool_init(&game->particles);
    game->renderPixelsPerMeter = PIXELS_PER_METER;
    game->camera = (Camera2D){ .offset = { SCREEN_W * 0.5f, SCREEN_H * 0.5f },
                               .target = { 0.0f, 0.0f },
                               .rotation = 0.0f,
                               .zoom = CAMERA_BASE_ZOOM };
    replay_begin_recording(&game->replay, 0);
    dev_state_init(&game->dev);
#if !defined(DRIFTY_HEADLESS)
    /* Load the community gamepad mapping database so non-XInput controllers
     * (DualSense, DualShock 4, Switch Pro, etc.) are recognized by raylib on
     * Windows. The file is optional — if missing, built-in GLFW mappings
     * (which cover XInput/Xbox controllers) still work. */
    char *gamepadMappings = LoadFileText("data/input/gamecontrollerdb.txt");
    if (gamepadMappings != NULL) {
        SetGamepadMappings(gamepadMappings);
        UnloadFileText(gamepadMappings);
    }
    track_init(&game->track);
    audio_init();
#endif
    game->stateChecksum = game_state_checksum(game);
    game->initialized = true;
#if !defined(DRIFTY_HEADLESS)
    TRACELOG(LOG_INFO, "GAME: initialised (particles, camera, state machine)");
#endif
}

GAME_API void game_pre_reload(Game *game)
{
    if (game == NULL) return;
    if (game->replay.mode == REPLAY_MODE_RECORDING) replay_stop(&game->replay);
#if !defined(DRIFTY_HEADLESS)
    audio_pre_reload();
    /* Same contract as the sounds above: the baked vehicle textures are raylib-tracked GPU
     * resources held in the render module's statics, and a handle from the outgoing module
     * is a dangling GPU name in the incoming one. */
    render_pre_reload();
    TRACELOG(LOG_INFO, "GAME: pre-reload (tick %llu)", (unsigned long long)game->sim.tick);
#endif
}

GAME_API void game_post_reload(Game *game)
{
    if (game == NULL) return;
    if (game->replay.mode == REPLAY_MODE_IDLE) game->replay.mode = REPLAY_MODE_RECORDING;

    /* raygui's style lives in the module's static data, which the swap threw away. */
    dev_lab_reload_style();

    game->reloadCount++;
    game->reloadFlashTimerS = RELOAD_FLASH_S;
#if !defined(DRIFTY_HEADLESS)
    audio_post_reload();
    render_post_reload();
    TRACELOG(LOG_INFO, "GAME: post-reload #%d (tick %llu, checksum %08x)", game->reloadCount,
             (unsigned long long)game->sim.tick, game->stateChecksum);
#endif
}

GAME_API void game_fixed_update(Game *game, float dt)
{
    if (game == NULL) return;
    DRIFTY_ZONE_BEGIN(fixedUpdate, "FixedUpdate");
    Input tickInput;
    input_zero(&tickInput);
    bool fromPlayback = false;
    if (game->replay.mode == REPLAY_MODE_PLAYBACK) {
        if (replay_next(&game->replay, &tickInput))
            fromPlayback = true;
        else
            replay_stop(&game->replay);
    }
    if (!fromPlayback) tickInput = game->input;

    /* A running scripted scenario replaces live input, but only when we are not already
     * replaying a timeline: playback must never be second-guessed. The substituted input is
     * recorded like any other, so the scenario itself is replayable. */
    if (!fromPlayback && game->dev.scenarioRunning) {
        dev_scenario_input(game->dev.scenario, game->sim.tick - game->dev.scenarioStartTick,
                           &tickInput);
    }

    input_clear_oneshots(&game->input);
    replay_record(&game->replay, &tickInput);
    apply_oneshots(game, &tickInput);

    /* Particle pool: always updates so existing particles fade over time regardless of the
     * game state. Spawn only happens inside the PLAYING gate below. */
    particle_pool_update(&game->particles, dt);

    if (game->state == STATE_PLAYING) {
        /* Auto transmission: override gear and remap throttle/brake */
        auto_transmission_update(&game->autoTrans, &game->vehicle, &game->spec, &game->derived,
                                 &tickInput, dt);

        DRIFTY_ZONE_BEGIN(physics, "Physics");
        /* Start-of-tick position, for the checkpoint crossing test below.
         *
         * It has to be currPositionM read HERE, before the physics step. physics_fixed_update
         * shifts curr into prev on entry and writes the new position into curr on exit, so
         * reading prevPositionM at this point yields the position two ticks ago — and a
         * two-tick sweep overlaps the next tick's sweep, which makes every gate crossing get
         * detected twice. That was invisible while only the expected gate was tested (the
         * second detection simply failed to advance); testing all gates reports it as an
         * out-of-order crossing, which is exactly the false accusation a lap validator must
         * not make. */
        const Vector2 startPosM = game->renderState.currPositionM;

        /* Per-wheel surface query: each contact point tests the track independently so the
         * car can straddle two surfaces. This lives in game.c, not physics.c, keeping the
         * physics TU free of track knowledge. The query is done every fixed tick before the
         * physics step consumes the surface ids.
         *
         * Guarded: skip when no track is loaded so tests and scenarios that explicitly set
         * per-wheel surfaceId are not overwritten. */
        if (game->track.nodes != NULL && game->track.count > 0) {
            for (int i = 0; i < WHEEL_COUNT; i++) {
                const Vector2 worldContact =
                    physics_wheel_world_position(&game->vehicle, (WheelId)i);
                game->vehicle.wheels[i].surfaceId = Track_SurfaceAt(&game->track, worldContact);
            }
        }
        physics_fixed_update(&game->spec, &game->vehicle, &game->derived, &game->renderState,
                             &tickInput, dt);
        DRIFTY_ZONE_END(physics);

        /* Checkpoint crossing: check whether the car passed a gate this tick. The event is
         * kept for the tick so telemetry and the validation runner can record which gate was
         * taken, and whether it was taken out of order. */
        if (game->track.nodes != NULL && game->track.count > 0) {
            game->lastCheckpointEvent = track_update_checkpoints(
                &game->track, startPosM, game->renderState.currPositionM);
            game->track.lapTimerS += dt;
        } else {
            memset(&game->lastCheckpointEvent, 0, sizeof(game->lastCheckpointEvent));
            game->lastCheckpointEvent.index = -1;
        }

        /* Crash lockout timer: count down toward zero. */
        if (game->crashLockoutTimerS > 0.0f) {
            game->crashLockoutTimerS -= dt;
            if (game->crashLockoutTimerS < 0.0f) game->crashLockoutTimerS = 0.0f;
        }

        /* Collision with track barriers. */
        if (game->track.nodes != NULL && game->track.count > 0) {
            float oldLockout = game->crashLockoutTimerS;
            collision_resolve_track(&game->spec, &game->vehicle, &game->renderState,
                                    &game->track, &game->crashLockoutTimerS);
            if (game->crashLockoutTimerS > oldLockout) {
                audio_play_collision_thud();
            }
        }

        /* Audio: per-tick engine pitch and tyre screech (after physics). */
        audio_update(game->vehicle.engineRpm, game->spec.engineIdleRpm,
                     game->spec.engineRedlineRpm, game->derived.physicallySliding,
                     game->derived.speedMps, dt);

        /* Results trigger: when the target lap count is reached, transition to
         * STATE_RESULTS. The car is no longer simulated after this tick. */
        if (game->track.lap >= RESULTS_TARGET_LAPS) {
            game->state = STATE_RESULTS;
        }

        /* Particle spawn: tire smoke from the rear wheels while physically sliding.
         * Two spawns per rear wheel per tick at 120 Hz ≈ 480 / s while sliding,
         * which the 512-slot pool sustains over the 0.8 s particle life. */
        if (game->derived.physicallySliding && game->derived.speedMps > 5.0f) {
            const float heading = game->vehicle.headingRad;
            const float speedMps = fminf(game->derived.speedMps, 50.0f);

            /* Base velocity: rearward, opposite to the car's heading. */
            const float baseBackX = -cosf(heading) * speedMps * 0.25f;
            const float baseBackY = -sinf(heading) * speedMps * 0.25f;

            for (int w = 0; w < 2; w++) {
                const WheelId wheelId = (w == 0) ? WHEEL_REAR_LEFT : WHEEL_REAR_RIGHT;
                const Vector2 wheelWorldM =
                    physics_wheel_world_position(&game->vehicle, wheelId);

                for (int s = 0; s < 2; s++) {
                    /* Deterministic spread that varies per-tick for visual variety. */
                    const float hashX =
                        (float)((int)(game->sim.tick * 7 + s * 13 + w * 31) & 0xFF);
                    const float hashY =
                        (float)((int)(game->sim.tick * 11 + s * 17 + w * 23) & 0xFF);
                    const float spreadX = (hashX / 255.0f - 0.5f) * 1.2f;
                    const float spreadY = (hashY / 255.0f - 0.5f) * 1.2f + 0.6f;
                    const Vector2 vel = { baseBackX + spreadX, baseBackY + spreadY };

                    particle_spawn(&game->particles, wheelWorldM, vel, 0.30f,
                                   (Color){ 200, 200, 200, 180 });
                }
            }
        }
    }
    game->sim.tick++;
    game->stateChecksum = game_state_checksum(game);

    /* Development history: scope channels, trajectory, and the invariant monitor. Reads the
     * state, writes only to game->dev, and is excluded from the checksum, so it cannot
     * influence the simulation. */
    dev_state_record(game, &tickInput);
    DRIFTY_ZONE_END(fixedUpdate);
}

#if defined(DRIFTY_HEADLESS)
GAME_API void game_draw(Game *game, float interpolationAlpha)
{
    (void)game;
    (void)interpolationAlpha;
}

GAME_API void game_shutdown(Game *game)
{
    if (game != NULL) {
        track_free(&game->track);
    }
    (void)game;
}
#else
GAME_API void game_draw(Game *game, float interpolationAlpha)
{
    render_draw_game(game, interpolationAlpha);
}

GAME_API void game_shutdown(Game *game)
{
    if (game == NULL) return;
    track_free(&game->track);
    audio_shutdown();
    render_shutdown();
    TRACELOG(LOG_INFO, "GAME: shutdown after %llu fixed ticks (checksum %08x)",
             (unsigned long long)game->sim.tick, game->stateChecksum);

    /* Every instrumented zone lives in this module, so this is where the table is. Compiles
     * to nothing unless the build asked for a profiling backend. */
    DRIFTY_PROFILE_REPORT(stdout);
}
#endif
