/*
 * game.h — the platform-owned persistent Game block.
 *
 * OWNERSHIP. A single Game structure, allocated once by the platform layer (main.c) with
 * calloc and passed to the game module by pointer on every entry point. It is deliberately
 * NOT a `static Game game;` inside the module: BSS belongs to whichever module declares it,
 * so a module-owned Game would be destroyed on every hot reload.
 *
 * RELOAD SAFETY. Nothing reachable from Game may point into the game module's code or
 * static data, and no function pointers may be stored here. See the invariant spelled out
 * in hotreload.h. Everything below is plain value data, which is what lets the block
 * survive a module swap unchanged.
 *
 * Phase 2 extends the embedded canonical vehicle structures. This layout change requires
 * one platform restart; subsequent game-module-only edits preserve the block normally.
 */
#ifndef DRIFTY_GAME_H
#define DRIFTY_GAME_H

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h" /* Vector2 only; game.c is the only TU here that calls raylib */

#include "core/config.h"
#include "dev/dev_state.h"
#include "platform/hotreload.h"
#include "game/input.h"
#include "game/particle.h"
#include "game/replay.h"
#include "world/track.h"
#include "physics/vehicle.h"
#include "physics/auto_transmission.h"

typedef enum {
    STATE_MENU = 0,
    STATE_PLAYING,
    STATE_PAUSED,
    STATE_RESULTS,
    STATE_COUNT
} GameStateId;

typedef enum {
    GAME_TRACK_KEEP = 0, /* leave whatever game_init() loaded */
    GAME_TRACK_PARKING_LOT,
    GAME_TRACK_CHICANE,
    GAME_TRACK_COUNT
} GameTrackId;

/*
 * What a bounded run should be set up with. Plain value data passed by pointer from the
 * platform layer, which cannot reach track or vehicle code directly — those live in the
 * reloadable module. Nothing here is retained: game_configure_run() reads it and returns.
 */
struct GameRunConfig {
    GameTrackId track;
    float cameraZoomOverride; /* 0 leaves the follow camera's own choice alone */
};

typedef struct {
    uint64_t tick;
    uint32_t resetCount;
    uint32_t pauseToggleCount;
    uint32_t debugToggleCount;
    uint32_t shiftUpCount;
    uint32_t shiftDownCount;
} SimState;

struct Game {
    GameStateId state;
    Input input;
    SimState sim;
    VehicleSpec spec;
    VehicleState vehicle;
    VehicleDerived derived;
    VehicleRenderState renderState;
    Track track;
    /* What the checkpoint test saw on the most recent fixed tick. Plain value data, rewritten
     * every tick, and excluded from the state checksum because it is a report about the
     * simulation rather than part of it. */
    TrackCheckpointEvent lastCheckpointEvent;
    ParticlePool particles;
    Camera2D camera;

    /* Fixed-timestep bookkeeping, written by the platform loop via timestep_advance(). */
    float accumulatorS;
    int lastSubstepCount;
    int physicsBacklogDrops;

    /* Deterministic input recording and playback. Fixed capacity, no allocation. */
    ReplayBuffer replay;

    /* Rolling checksum of the deterministic simulation state, recomputed every fixed
     * update. Two runs of the same input timeline must agree on this value. */
    uint32_t stateChecksum;

    /* Render-only. Initialised from PIXELS_PER_METER and consumed exclusively by units.h
     * helpers in game_draw(). No simulation quantity may read it; the "renderscale"
     * scenario in tests/scenarios/core_tests.c asserts that by running the same timeline at two
     * different scales and comparing checksums. */
    float renderPixelsPerMeter;

    /* Presentation and diagnostics. */
    float crashLockoutTimerS; /* seconds remaining in the post-impact lockout */
    bool debugOverlay;
    int reloadCount;
    float reloadFlashTimerS;
    bool initialized;

    /* Automatic transmission mode (toggle with T). */
    AutoTransmission autoTrans;

    /* Development tooling: Physics Lab, scope history, trajectory, invariant monitor, and
     * the time controls the platform loop reads. Plain value data like everything else here,
     * and present in every build configuration so that drifty.exe and build/game.dll cannot
     * disagree about the layout of this struct. See src/dev/dev_state.h. */
    DevState dev;
};

/*
 * Helpers exposed to the platform layer and the headless harness. These are ordinary
 * module functions, not reloadable entry points: the entry-point list in hotreload.h stays
 * exactly as the GAME_ENTRY_POINTS X-macro in src/platform/hotreload.h defines it.
 */

/* FNV-1a over the deterministic simulation fields only. Explicitly excludes the
 * accumulator, the substep and backlog counters, the render scale, and every presentation
 * field, so the checksum depends on the input timeline and nothing else. */
GAME_API uint32_t game_state_checksum(const Game *game);

/* Reset the vehicle and resynchronise render history. Counters and tick are preserved. */
GAME_API void game_reset_sim(Game *game);

/* Overwrite the vehicle spec and reset the simulation to match. Safe to call after
 * game_init() for headless scenarios that need to test multiple specs against the
 * same maneuver. Does not re-initialise track, audio, or visual subsystems —
 * callers that need those must handle them separately. */
GAME_API void game_apply_spec(Game *game, const VehicleSpec *spec);

/* Place the car at the loaded track's start/finish line, facing the way the circuit goes, and
 * put lap progress back to the beginning of an out-lap. Returns false when no track with
 * gates is loaded, in which case nothing is modified. */
GAME_API bool game_spawn_on_track(Game *game);

#endif /* DRIFTY_GAME_H */
