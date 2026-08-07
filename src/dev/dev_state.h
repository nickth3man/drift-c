/*
 * dev_state.h — the development-tool state that lives inside the persistent Game block.
 *
 * WHY THIS IS NOT #ifdef'd OUT. drifty.exe and build/game.dll both include game.h and both
 * must agree byte-for-byte on the layout of Game, or the first hot reload hands the module a
 * pointer it will misread. Making DevState conditional would make that agreement depend on
 * two independent compiler invocations defining the same macro. So the *data* is always
 * present (a few tens of kilobytes of plain values) and only the raygui *interface* in
 * src/dev/dev_lab.c is conditional. Release builds carry the struct and never draw it.
 *
 * Everything here is plain value data — no pointers, no function pointers — so it survives a
 * module swap like the rest of Game (see the invariant in hotreload.h).
 *
 * This translation unit is raylib-free apart from the Vector2 type, so it also links into
 * the headless test executable.
 */
#ifndef DRIFTY_DEV_STATE_H
#define DRIFTY_DEV_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h" /* Vector2 only */

#include "game/input.h"

/* Scope history: 480 samples is four seconds at 120 Hz, which is the whole of any
 * transition worth watching without turning Game into a megabyte. */
#define DEV_SCOPE_SAMPLES 480
#define DEV_PATH_POINTS 1024
#define DEV_PATH_DECIMATION 4 /* record one trajectory point every N fixed ticks */
#define DEV_STATUS_CHARS 96
#define DEV_INVARIANT_CHARS 128

typedef enum {
    DEV_SCOPE_SIDESLIP = 0,
    DEV_SCOPE_YAW_RATE,
    DEV_SCOPE_STEER,
    DEV_SCOPE_THROTTLE,
    DEV_SCOPE_FRONT_SLIP,
    DEV_SCOPE_REAR_SLIP,
    DEV_SCOPE_FRICTION,
    DEV_SCOPE_SPEED,
    /* Phase 3 */
    DEV_SCOPE_FILTERED_ACCEL,
    DEV_SCOPE_SOLVED_ACCEL,
    DEV_SCOPE_FRONT_LOAD,
    DEV_SCOPE_REAR_LOAD,
    DEV_SCOPE_LOAD_TRANSFER,
    DEV_SCOPE_AERO_DRAG,
    DEV_SCOPE_ROLLING,
    DEV_SCOPE_FRONT_USAGE,
    DEV_SCOPE_REAR_USAGE,
    DEV_SCOPE_COUNT
} DevScopeChannel;

/*
 * Every channel is recorded; only eight are drawn at once.
 *
 * The limit is a readability budget, not a storage one: seventeen 42-pixel traces stacked
 * down the right-hand side is a wall, not an instrument. Presets let the developer swap
 * which eight are on screen without losing the history of the rest, so switching from
 * "handling" to "load" mid-run shows the load channels for the whole retained window.
 */
#define DEV_SCOPE_VISIBLE 8

typedef enum {
    DEV_SCOPE_PRESET_HANDLING = 0, /* the Phase 1/2 set: what the driver did and felt */
    DEV_SCOPE_PRESET_LOAD,         /* the Phase 3 set: acceleration filter -> loads -> drag */
    DEV_SCOPE_PRESET_GRIP,         /* slip and friction budget, front against rear */
    DEV_SCOPE_PRESET_COUNT
} DevScopePreset;

const char *dev_scope_preset_name(DevScopePreset preset);

/* Which channel occupies visible slot `slot` (0 .. DEV_SCOPE_VISIBLE-1) in this preset. */
DevScopeChannel dev_scope_preset_channel(DevScopePreset preset, int slot);

/*
 * State-derived replay markers.
 *
 * dev_replay_collect_events() derives markers from the INPUT timeline, which is all a replay
 * file holds. These are the ones only the simulation knows about — first saturation, peak
 * load transfer, the invariant violation, the recovery. Recording them here rather than in
 * the replay format keeps the format unchanged, and keeps them plain value data inside Game.
 */
#define DEV_MARKER_CAPACITY 64

typedef enum {
    DEV_MARKER_THROTTLE_ON = 0,
    DEV_MARKER_THROTTLE_LIFT,
    DEV_MARKER_BRAKE_ON,
    DEV_MARKER_HANDBRAKE_ON,
    DEV_MARKER_STEER_REVERSAL,
    DEV_MARKER_SATURATION,    /* first tick at the friction limit */
    DEV_MARKER_PEAK_TRANSFER, /* running maximum |load transfer| */
    DEV_MARKER_INVARIANT,     /* first invariant violation */
    DEV_MARKER_RECOVERY,      /* sideslip came back from a slide to stable travel */
    DEV_MARKER_KIND_COUNT
} DevMarkerKind;

typedef struct {
    uint64_t tick;
    DevMarkerKind kind;
    float value; /* whatever the marker is about, in its own unit */
} DevMarker;

typedef struct {
    /* ------------------------------------------------------------------ lab presentation -- */
    bool labVisible;
    bool inspectorVisible;

    /* Scene capture sets this. raygui highlights whatever the pointer happens to be over, so
     * an un-suppressed capture depends on where the developer left the mouse — which is
     * exactly the kind of difference a screenshot comparison must not see. */
    bool uiDeterministic;
    int activeGroup;    /* index into the dev_params group list */
    float paramScrollY; /* raygui scroll offset for the parameter panel */
    int galleryPage;    /* bounded corpus gallery page (Phase 5 capture) */
    int labTierFilter;  /* DevParamTier; hide deeper tiers in the Lab */
    /* Pin the world camera zoom instead of letting the per-frame follow logic choose it.
     * 0 means "not pinned". A whole-track framing is what makes an automated recording useful
     * for diagnosing where a car went, which a car-following camera cannot show. */
    float cameraZoomOverride;

    /* ------------------------------------------------------ time control (read by main.c) -- */
    bool paused;
    int stepTicks;   /* > 0: run exactly this many fixed ticks, then pause */
    float timeScale; /* 0.05 .. 4.0; 1.0 is real time */

    /* ------------------------------------------------------------------------- scenarios -- */
    int scenario; /* index into the dev_scenario table */
    bool scenarioRunning;
    uint64_t scenarioStartTick;
    uint32_t seed; /* the running scenario's seed, shown in the lab */

    /* -------------------------------------------------------------------------- overlays -- */
    bool showForces;
    bool showVelocity;
    bool showSlip;
    bool showLoads;
    bool showContacts;
    bool showPath;
    bool showGhost;
    bool showScope;
    bool showResistance; /* drag and rolling-resistance vectors from the CG */

    /* ----------------------------------------------------------------------------- scope -- */
    float scope[DEV_SCOPE_COUNT][DEV_SCOPE_SAMPLES];
    int scopeHead; /* ring index of the next write */
    int scopeCount;
    int scopePreset; /* DevScopePreset; which eight channels are drawn */

    /* ------------------------------------------------------------- trajectory and ghost -- */
    Vector2 path[DEV_PATH_POINTS];
    int pathCount;
    int pathHead;
    int pathDecimator;
    Vector2 ghost[DEV_PATH_POINTS];
    int ghostCount;
    bool ghostValid;

    /* The scope history captured alongside the ghost trajectory. Keeping both means the
     * inspector can answer "how does this run differ from the baseline at this tick?"
     * rather than only "where did the baseline go?". */
    float ghostScope[DEV_SCOPE_COUNT][DEV_SCOPE_SAMPLES];
    int ghostScopeCount;

    /* ------------------------------------------------------------------------ invariants -- */
    bool invariantFailed;
    uint64_t invariantTick;
    int invariantCount; /* cumulative violations since the last clear */
    char invariantText[DEV_INVARIANT_CHARS];

    /* -------------------------------------------------------------- state-derived markers -- */
    DevMarker markers[DEV_MARKER_CAPACITY];
    int markerCount;
    /* Edge-detection memory for the marker rules above. Plain values, reset with history. */
    float markerPrevThrottle;
    float markerPrevBrake;
    float markerPrevHandbrake;
    float markerPrevSteer;
    bool markerSaturated;
    bool markerInSlide;
    float markerPeakTransferN;
    int markerPeakTransferSlot; /* index into markers[], or -1 */

    /* The input the fixed update actually applied — the scripted timeline while a scenario
     * runs, not whatever the keyboard said. Telemetry and the lab both read it. */
    Input appliedInput;

    /* -------------------------------------------------------------------------- inspector -- */
    int inspectorCursor; /* frame index within the retained replay window */
    bool inspectorFollowsLive;

    /* --------------------------------------------------------------------- status line -- */
    char status[DEV_STATUS_CHARS];
    float statusTimerS;
    bool statusIsError;
} DevState;

struct Game;

/* Defaults: lab hidden, real time, free-drive scenario, useful overlays on. */
void dev_state_init(DevState *dev);

/* Called at the end of every fixed update with the input that was actually applied — which
 * is the scripted input while a scenario runs, not whatever the keyboard says. Pushes scope
 * samples, extends the trajectory, and evaluates the invariants. Safe in headless builds. */
void dev_state_record(struct Game *game, const Input *appliedInput);

/* Oldest-first accessor over the scope ring. index 0 is the oldest retained sample. */
float dev_state_scope_value(const DevState *dev, DevScopeChannel channel, int index);
const char *dev_state_scope_name(DevScopeChannel channel);
const char *dev_state_scope_unit(DevScopeChannel channel);

/* Oldest-first accessor over the trajectory ring. */
Vector2 dev_state_path_point(const DevState *dev, int index);

/* Freeze the current trajectory and scope history as the ghost/baseline overlay. */
void dev_state_capture_ghost(DevState *dev);

/* Oldest-first accessor over the captured baseline scope. */
float dev_state_ghost_scope_value(const DevState *dev, DevScopeChannel channel, int index);
void dev_state_clear_history(DevState *dev);

/* Scenario control. Starting one resets the simulation, rewinds the replay recording, and
 * clears the scope so the run begins from a known state. */
void dev_state_scenario_start(struct Game *game, int scenarioIndex);
void dev_state_scenario_stop(struct Game *game);

/* Transient message shown in the lab's status line (seconds, then it fades). */
void dev_state_set_status(DevState *dev, bool isError, const char *format, ...);

/* Clear a latched invariant violation. */
void dev_state_clear_invariants(DevState *dev);

#endif /* DRIFTY_DEV_STATE_H */
