/*
 * render.h — Phase 1 interpolation and simple vehicle/debug presentation.
 */
#ifndef DRIFTY_RENDER_H
#define DRIFTY_RENDER_H

#include "physics/vehicle.h"
#include "platform/hotreload.h"

typedef struct {
    Vector2 positionM;
    float headingRad;
    float wheelAngleRad[WHEEL_COUNT];
} VehicleDrawState;
typedef struct ValidationOverlayData {
    const char *carId;
    const char *carDisplayName;
    float elapsedS;
    int checkpointIndex;
    int checkpointTotal;
    float speedMps;
    int lapState;        /* 0 OUT-LAP, 1 TIMED, 2 COMPLETE, 3 FAILED */
    bool isPassing;      /* true if running attempt has no violations */
    float steerInput;    /* -1..+1 */
    float throttleInput; /* 0..1 */
    float brakeInput;    /* 0..1 */
} ValidationOverlayData;

#if !defined(DRIFTY_HOT_RELOAD) || defined(DRIFTY_GAME_MODULE)
GAME_API void render_draw_validation_overlay(const ValidationOverlayData *data);
#endif
struct Game;

VehicleDrawState render_interpolate_vehicle(const VehicleRenderState *state, float alpha);
void render_draw_game(struct Game *game, float interpolationAlpha);

/* ------------------------------------------------------------- GPU resource lifetime ----
 *
 * The baked vehicle sprites are raylib-tracked GPU textures held in this module's statics,
 * so they must be released before the module is swapped and re-acquired afterwards — the
 * same contract audio.c has for its sounds, and the reason AGENTS.md lists textures among
 * the things a reload cannot carry across. A handle from the old module is a dangling GPU
 * name in the new one.
 *
 * No Game layout changes: the cache is module-static, not persistent state. Losing it across
 * a reload costs one rebake on the next frame and nothing else.
 *
 * All three are no-ops under DRIFTY_HEADLESS. */
void render_pre_reload(void);
void render_post_reload(void);
void render_shutdown(void);

/* ------------------------------------------------------------------------- gallery ----
 *
 * One page of the vehicle corpus, drawn through the production texture path — the same bake
 * and the same compositor the running game uses, so the gallery cannot show a car the game
 * would not. No simulation runs; the pose is fixed with the front wheels turned so lock,
 * Ackermann and static toe are all visible at once.
 *
 * One page — sixteen cars — is baked, drawn, and then released once `EndDrawing` has
 * submitted the batch: reviewing a hundred vehicles never holds more than a page of
 * textures on the GPU. Unloading between the draw calls and the flush would leave raylib's
 * deferred batch pointing at freed texture ids, which is why the release is deferred.
 *
 * `page` is 1-based. Out-of-range pages draw the page-count notice and nothing else, so
 * `--gallery-page 99` is a readable no-op rather than an empty screen.
 *
 * A human-review artifact, deliberately not a GPU regression baseline: a hundred cars behind
 * an RMSE gate on hardware that renders differently per vendor is a maintenance sinkhole with
 * no CI value. The headless contact sheet and the `corpus` scenario are the actual gates. */
int render_gallery_page_count(void);
void render_draw_gallery(struct Game *game, int page);

#endif /* DRIFTY_RENDER_H */
