/*
 * render.c — interpolation, the pixel-art world target, the GPU-resource lifecycle, and
 * the top-level frame orchestration. The track, vehicle, and HUD drawing live in
 * render_world.c, render_vehicle.c, and render_hud.c behind render_internal.h.
 */
#include "render/render.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "dev/dev_lab.h"
#include "game/game.h"
#include "core/math_utils.h"
#include "game/profile.h"
#include "core/units.h"

VehicleDrawState render_interpolate_vehicle(const VehicleRenderState *state, float alpha)
{
    VehicleDrawState out;
    memset(&out, 0, sizeof(out));
    if (state == NULL) return out;
    const float t = clampf(alpha, 0.0f, 1.0f);
    out.positionM.x = lerpf(state->prevPositionM.x, state->currPositionM.x, t);
    out.positionM.y = lerpf(state->prevPositionM.y, state->currPositionM.y, t);
    out.headingRad = lerp_angle(state->prevHeadingRad, state->currHeadingRad, t);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        out.wheelAngleRad[i] =
            lerp_angle(state->prevWheelAngleRad[i], state->currWheelAngleRad[i], t);
    }
    return out;
}

#if defined(DRIFTY_HEADLESS)
void render_draw_game(struct Game *game, float interpolationAlpha)
{
    (void)game;
    (void)interpolationAlpha;
}
void render_pre_reload(void) {}
void render_post_reload(void) {}
void render_shutdown(void) {}
int render_gallery_page_count(void)
{
    return 0;
}
void render_draw_gallery(struct Game *game, int page)
{
    (void)game;
    (void)page;
}
#else

#include "raylib.h"

#include "render/render_internal.h"

/* ------------------------------------------------------------- pixel-art world target ----
 *
 * The world is rasterized into a low-resolution RenderTexture2D and blown up once by an exact
 * integer factor with nearest-neighbour filtering; the HUD, raygui and the Physics Lab draw
 * afterwards at native resolution. src/core/config.h carries the whole scale chain and the reason
 * for each number in it.
 */
static RenderTexture2D s_worldTarget;
static bool s_worldTargetReady = false;

static void unload_world_target(void)
{
    if (!s_worldTargetReady) return;
    UnloadRenderTexture(s_worldTarget);
    memset(&s_worldTarget, 0, sizeof(s_worldTarget));
    s_worldTargetReady = false;
}

static void ensure_world_target(void)
{
    if (s_worldTargetReady) return;
    s_worldTarget = LoadRenderTexture(PIXEL_ART_TARGET_W, PIXEL_ART_TARGET_H);
    if (s_worldTarget.id == 0) return;
    SetTextureFilter(s_worldTarget.texture, TEXTURE_FILTER_POINT);
    s_worldTargetReady = true;
}

/* The world camera, adjusted for the low-resolution target.
 *
 * Two changes from game->camera, both load-bearing:
 *   - the offset centres on the TARGET, which is half the window in each axis;
 *   - the camera translation is snapped to whole target pixels. A Camera2D maps world to
 *     screen as (world - target) * zoom + offset, so a fractional (-target * zoom) makes
 *     every world pixel land between two target pixels and the whole grid crawls a pixel at a
 *     time as the car moves. Snapping costs sub-pixel camera smoothness, which nobody can
 *     see, and buys a stable pixel grid, which everybody can.
 */
static Camera2D world_camera_for_target(Camera2D camera)
{
    Camera2D cam = camera;
    cam.offset =
        (Vector2){ (float)PIXEL_ART_TARGET_W * 0.5f, (float)PIXEL_ART_TARGET_H * 0.5f };

    cam.offset.x = units_snap_camera_offset_axis(cam.offset.x, cam.target.x, cam.zoom);
    cam.offset.y = units_snap_camera_offset_axis(cam.offset.y, cam.target.y, cam.zoom);
    return cam;
}

/* Blit the finished world target over the whole window. The source height is negative because
 * a RenderTexture2D is stored bottom-up; the destination is exactly SCREEN_W x SCREEN_H, so
 * the enlargement is PIXEL_ART_UPSCALE and nothing else. */
static void blit_world_target(void)
{
    if (!s_worldTargetReady) return;
    const Rectangle src = { 0.0f, 0.0f, (float)s_worldTarget.texture.width,
                            -(float)s_worldTarget.texture.height };
    const Rectangle dst = { 0.0f, 0.0f, (float)SCREEN_W, (float)SCREEN_H };
    DrawTexturePro(s_worldTarget.texture, src, dst, (Vector2){ 0.0f, 0.0f }, 0.0f, WHITE);
}

void render_pre_reload(void)
{
    /* Release before the module is swapped: the handles belong to module statics and would
     * be dangling GPU names after the reload. Two independent GPU-resource lifecycles are
     * coordinated here — the baked vehicle sprites and the low-resolution world target. */
    render_vehicle_resources_unload();
    unload_world_target();
}

void render_post_reload(void)
{
    render_vehicle_resources_reset();
}

void render_shutdown(void)
{
    render_vehicle_resources_unload();
    unload_world_target();
}

void render_draw_game(struct Game *game, float interpolationAlpha)
{
    if (game == NULL) return;

    /* The gallery is a whole-screen mode, not an overlay: no world, no HUD, no simulation.
     * It is selected through the DevState field Phase 2 reserved for it rather than a new
     * GAME_ENTRY_POINTS function, so the platform layer can ask for a page without changing
     * the module's ABI or the layout of anything persistent. */
    if (game->dev.galleryPage > 0) {
        render_draw_gallery(game, game->dev.galleryPage);
        return;
    }

    /* Development shortcuts and the status-line timer, once per render frame. Compiles to
     * nothing when the dev tools are not built in. */
    dev_lab_update(game);

    DRIFTY_ZONE_BEGIN(render, "Render");
    const float renderDt = GetFrameTime();
    if (game->reloadFlashTimerS > 0.0f) {
        game->reloadFlashTimerS = fmaxf(0.0f, game->reloadFlashTimerS - renderDt);
    }
    const float alpha = clampf(interpolationAlpha, 0.0f, 1.0f);
    const VehicleDrawState draw = render_interpolate_vehicle(&game->renderState, alpha);
    game->camera.target = units_world_to_render_px(draw.positionM, game->renderPixelsPerMeter);

    /* Camera zoom: pinned at the base zoom unless a diagnostic override is active.
     * A pinned zoom gives a stable framing for frame-to-frame comparison. */
    if (game->dev.cameraZoomOverride > 0.0f) {
        game->camera.zoom = game->dev.cameraZoomOverride;
    } else {
        game->camera.zoom = CAMERA_BASE_ZOOM;
    }

    ensure_world_target();

    /* ---- the world, at pixel-art resolution -------------------------------------------
     *
     * Everything inside the camera goes into the low-resolution target and is enlarged once
     * by an exact integer factor. Everything after the blit — HUD, raygui, Physics Lab — is
     * drawn at native resolution, because text through a nearest-neighbour upscale is
     * unreadable. If the target could not be created the world draws straight to the window;
     * a missing render texture should cost the pixel-art look, not the game. */
    const bool pixelArt = s_worldTargetReady;
    const Camera2D worldCam = pixelArt ? world_camera_for_target(game->camera) : game->camera;

    if (pixelArt) {
        BeginTextureMode(s_worldTarget);
        ClearBackground((Color){ 22, 24, 28, 255 });
    } else {
        BeginDrawing();
        ClearBackground((Color){ 22, 24, 28, 255 });
    }

    BeginMode2D(worldCam);
    render_world_draw_track(&game->track, game->renderPixelsPerMeter);

    /* ---- particles (between track and car, per the spec draw order) ---- */
    render_world_draw_particles(game);

    render_vehicle_draw(game, &draw);
    if (game->debugOverlay) render_world_draw_debug(game, &draw);
    dev_lab_draw_world(game, &draw);
    EndMode2D();

    if (pixelArt) {
        EndTextureMode();
        BeginDrawing();
        ClearBackground((Color){ 22, 24, 28, 255 });
        blit_world_target();
    }

    /* ---- raw physics diagnostics (F1) -----------------------------------------
     * The development readout lives in render_hud.c and draws only when the debug
     * overlay is on; the always-on presentation is the arcade HUD below. */
    render_hud_draw_diagnostics(game, alpha);

    /* ---- arcade HUD + state overlays (screen space, after EndMode2D) ----------------- */
    if (game->state == STATE_PLAYING || game->state == STATE_PAUSED) {
        render_hud_draw_arcade(game);
    }
    render_hud_draw_state_overlay(game);

    /* The lab paints over the HUD deliberately: when it is open it is what you are reading. */
    DRIFTY_ZONE_BEGIN(lab, "PhysicsLab");
    dev_lab_draw_ui(game);
    DRIFTY_ZONE_END(lab);

    EndDrawing();
    DRIFTY_ZONE_END(render);
    DRIFTY_FRAME_MARK();
}
#endif
