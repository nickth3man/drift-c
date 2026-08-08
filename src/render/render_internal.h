/*
 * render_internal.h — the implementation-only seams between the renderer's translation
 * units. Nothing here is public: render/render.h is the only header callers outside
 * src/render/ may use, and these functions exist only in non-headless builds.
 */
#ifndef DRIFTY_RENDER_INTERNAL_H
#define DRIFTY_RENDER_INTERNAL_H

#include "render/render.h"

#include "game/game.h"

/* The presentation palette, defined once in render_hud.c and shared with the gallery in
 * render_vehicle.c. Documented for the design review where it is defined. */
extern const Color COL_ACCENT;
extern const Color COL_ACCENT_WARM;
extern const Color COL_COOL;
extern const Color COL_TEXT;
extern const Color COL_TEXT_DIM;
extern const Color COL_PANEL;
extern const Color COL_PANEL_EDGE;
extern const Color COL_DIM_SCREEN;

/* The world pass, drawn into the low-resolution target inside BeginMode2D. */
void render_world_draw_track(const Track *track, float ppm);
void render_world_draw_particles(const Game *game);
void render_world_draw_debug(const Game *game, const VehicleDrawState *draw);

/* The baked-sprite vehicle and its GPU resource lifecycle. */
void render_vehicle_draw(const Game *game, const VehicleDrawState *draw);
void render_vehicle_resources_unload(void);
void render_vehicle_resources_reset(void);

/* Screen-space presentation, after the world target has been blitted. */
void render_hud_draw_diagnostics(const Game *game, float alpha);
void render_hud_draw_arcade(const Game *game);
void render_hud_draw_state_overlay(const Game *game);

/* Draws the staged validation overlay, or nothing when none is staged. Called inside
 * game_draw()'s frame so the offline capture readback sees it. */
void render_draw_staged_validation_overlay(void);

#endif /* DRIFTY_RENDER_INTERNAL_H */
