/*
 * render_vehicle.c — the baked vehicle sprites: upload, compositing, the running game's
 * single cached car, and the corpus gallery. No styling or geometry decision lives here;
 * the pixels come from car_visual.c and car_visual_raster.c, this file only uploads and
 * draws them. Compiled to an empty translation unit under DRIFTY_HEADLESS.
 */
#if !defined(DRIFTY_HEADLESS)

#include "render/render_internal.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev/car_corpus.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "core/math_utils.h"
#include "core/units.h"

/* ------------------------------------------------------------- baked vehicle sprites ----
 *
 * The car is drawn from textures baked by the SAME CPU rasterizer the headless contact sheet
 * uses (src/render/car_visual_raster.c), so there is exactly one geometry grammar in the project and
 * the gallery cannot drift away from the game.
 *
 * BAKE, DO NOT REDRAW. A car's pixels change only when its spec changes, which for a running
 * game is almost never — but the Physics Lab writes game->spec live, so it does happen. Every
 * frame derives the CarVisual (cheap, no allocation) and hashes it with car_visual_bake_key();
 * the textures are rebuilt only when that key or the metres-to-pixels scale differs from what
 * is already on the GPU. A still car costs one derive and one integer compare per frame.
 *
 * SCALE. Baked at the WORLD scale, PIXELS_PER_METER * CAMERA_BASE_ZOOM, so one texel is
 * exactly one pixel of the low-resolution world target at rest and the sprite is never
 * resampled — see the scale chain in src/core/config.h. The drift zoom still moves continuously
 * around that resting point; at rest, which is where a still image is judged, the mapping is
 * exact.
 *
 * COMPOSITION. Body first, then the four wheels on top, matching the L6-after-L1 order the
 * rasterizer draws them in. The front wheels rotate to heading + steer + their derived static
 * toe; the rears keep their derived static alignment. The wheels are NOT baked into the body
 * texture — a wheel that cannot turn would make the steering unreadable at exactly the moment
 * it matters most.
 *
 * HOT RELOAD. Texture2D handles are raylib-tracked GPU resources living in module statics, so
 * render_pre_reload() releases them and render_post_reload() drops the cache key so the next
 * frame rebakes. This is the audio.c contract applied to textures; see render.h.
 */
#define CAR_TEX_PAD_PX 1

/* One car's sprites. The running game keeps exactly one of these in a module static; the
 * gallery builds and discards one per cell, so reviewing a hundred vehicles never holds more
 * than a single car's textures on the GPU.
 *
 * Invariant: every instance is zero-initialised before first use. `s_car` and the gallery cells
 * have static storage, and the gallery explicitly clears each cell before baking. That makes a
 * zero Texture2D id the reliable empty-handle state used by partial-bake cleanup. */
typedef struct {
    bool ready;
    float pxPerM; /* texels per metre these were baked at */
    Texture2D body;
    Vector2 bodyOriginPx;
    Texture2D wheel[WHEEL_COUNT];
    Vector2 wheelOriginPx[WHEEL_COUNT];
} CarSprites;

static void car_sprites_unload(CarSprites *s)
{
    if (s == NULL) return;

    /* A bake may fail after uploading only some parts, before `ready` becomes true. Release
     * each live handle independently so the failure path cannot leak the partial upload. */
    if (s->body.id != 0) UnloadTexture(s->body);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (s->wheel[i].id != 0) UnloadTexture(s->wheel[i]);
    }
    memset(s, 0, sizeof(*s));
}

/* Upload one rasterized part. Returns false on any failure, which the caller treats as a
 * failed bake rather than drawing a garbage handle. */
static bool upload_part(const CarVisual *visual, CarRasterPart part, int wheelIndex,
                        float pxPerM, Texture2D *texOut, Vector2 *originOut)
{
    const CarRasterInfo info =
        car_raster_part_info(visual, part, wheelIndex, pxPerM, CAR_TEX_PAD_PX);
    const size_t bytes = car_raster_bytes(info);
    if (bytes == 0) return false;

    unsigned char *rgba = (unsigned char *)malloc(bytes);
    if (rgba == NULL) return false;
    if (!car_raster_draw_part(visual, part, wheelIndex, info, rgba, bytes)) {
        free(rgba);
        return false;
    }

    /* Wrap the buffer raylib expects without handing ownership over: LoadTextureFromImage
     * copies to the GPU and does not keep the pointer, so this frees its own memory rather
     * than calling UnloadImage on something raylib never allocated. */
    Image image;
    memset(&image, 0, sizeof(image));
    image.data = rgba;
    image.width = info.width;
    image.height = info.height;
    image.mipmaps = 1;
    image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;

    *texOut = LoadTextureFromImage(image);
    free(rgba);

    if (texOut->id == 0) return false;
    /* Pixel art: nearest neighbour, never a smoothed upscale. */
    SetTextureFilter(*texOut, TEXTURE_FILTER_POINT);
    originOut->x = info.originXPx;
    originOut->y = info.originYPx;
    return true;
}

/* Bake one car's five sprites. Leaves `out` not-ready and holding nothing on failure. */
static bool car_sprites_bake(CarSprites *out, const CarVisual *visual, float pxPerM)
{
    if (out == NULL || visual == NULL) return false;
    car_sprites_unload(out);

    bool ok =
        upload_part(visual, CAR_RASTER_PART_BODY, 0, pxPerM, &out->body, &out->bodyOriginPx);
    for (int i = 0; i < WHEEL_COUNT && ok; i++) {
        ok = upload_part(visual, CAR_RASTER_PART_WHEEL, i, pxPerM, &out->wheel[i],
                         &out->wheelOriginPx[i]);
    }
    if (!ok) {
        car_sprites_unload(out);
        return false;
    }
    out->pxPerM = pxPerM;
    out->ready = true;
    return true;
}

/* Draw a baked part about its documented pivot, `scale` destination pixels per texel.
 * `rotationRad` is a heading in world terms; units_heading_to_rotation_deg handles the
 * screen-space Y flip. */
static void draw_car_part(Texture2D tex, Vector2 originPx, Vector2 centrePx, float rotationRad,
                          float scale)
{
    if (tex.id == 0) return;
    const Rectangle src = { 0.0f, 0.0f, (float)tex.width, (float)tex.height };
    const Rectangle dst = { centrePx.x, centrePx.y, (float)tex.width * scale,
                            (float)tex.height * scale };
    /* DrawTexturePro measures `origin` in DESTINATION units, so the pivot scales with the
     * sprite. Getting this wrong shifts the car off its own centre of mass as the scale
     * changes, which is exactly the kind of bug a still screenshot hides. */
    const Vector2 origin = { originPx.x * scale, originPx.y * scale };
    DrawTexturePro(tex, src, dst, origin, units_heading_to_rotation_deg(rotationRad), WHITE);
}

/* The one place a car is composited, used by the running game and by the gallery alike.
 * `steerRad` is per wheel and is added to the derived static alignment; pass NULL for a
 * straight-ahead pose. `scale` is destination pixels per texel. */
static void car_sprites_draw(const CarSprites *s, const CarVisual *visual, Vector2 centrePx,
                             float headingRad, const float *steerRad, float scale)
{
    if (s == NULL || !s->ready || visual == NULL) return;

    draw_car_part(s->body, s->bodyOriginPx, centrePx, headingRad, scale);

    /* Destination pixels per metre follows from what the sprites were baked at and what they
     * are being drawn at, so a caller never has to restate the world scale. */
    const float destPxPerM = s->pxPerM * scale;
    const float c = cosf(headingRad);
    const float sn = sinf(headingRad);

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const Vector2 hubM = visual->wheels[i].centreM;
        /* Body frame -> world offset in metres, then metres -> destination pixels. +Y is
         * left in body and world space and up the screen, hence the negated Y. */
        const float wx = hubM.x * c - hubM.y * sn;
        const float wy = hubM.x * sn + hubM.y * c;
        const Vector2 hubPx = { centrePx.x + wx * destPxPerM, centrePx.y - wy * destPxPerM };

        const float steer = (steerRad != NULL) ? steerRad[i] : 0.0f;
        draw_car_part(s->wheel[i], s->wheelOriginPx[i], hubPx,
                      headingRad + steer + visual->wheels[i].staticAngleRad, scale);
    }
}

/* ---- the running game's single cached car ---- */

static CarSprites s_car;
static uint32_t s_carTexKey = 0u;
static float s_carTexPxPerM = 0.0f;

/* Rebake if and only if the picture or the scale changed. */
static void ensure_car_textures(const CarVisual *visual, float pxPerM)
{
    const uint32_t key = car_visual_bake_key(visual);
    if (s_car.ready && key == s_carTexKey && pxPerM == s_carTexPxPerM) return;

    if (!car_sprites_bake(&s_car, visual, pxPerM)) {
        TRACELOG(LOG_WARNING, "RENDER: vehicle sprite bake failed; retrying next frame");
        s_carTexKey = 0u;
        s_carTexPxPerM = 0.0f;
        return;
    }
    s_carTexKey = key;
    s_carTexPxPerM = pxPerM;
}

/* Destination pixels per texel inside the world camera. Sprites are baked at the world scale
 * (PIXELS_PER_METER * CAMERA_BASE_ZOOM) but drawn in render-pixel space, which the camera
 * then multiplies by its zoom — so a texel becomes exactly one target pixel when the zoom is
 * at rest, and the drift zoom moves smoothly around that. */
#define CAR_WORLD_TEXEL_SCALE (1.0f / CAMERA_BASE_ZOOM)

static float car_bake_px_per_m(float renderPixelsPerMeter)
{
    return renderPixelsPerMeter * CAMERA_BASE_ZOOM;
}

void render_vehicle_draw(const Game *game, const VehicleDrawState *draw)
{
    const float ppm = game->renderPixelsPerMeter;

    CarVisual visual; /* stack-local by design: never stored in Game */
    car_visual_derive(&game->spec, &visual);
    ensure_car_textures(&visual, car_bake_px_per_m(ppm));
    if (!s_car.ready) return;

    /* Front wheels draw at heading + steerAngleRad with a presentation-only gain so the steer
     * is unmistakable at top-down scale; the physics angle itself is untouched. Rear wheels
     * carry no steer, but they do carry whatever static alignment the grammar derived for
     * them, which car_sprites_draw adds for every wheel. */
    const float steerVisualGain = 1.25f; /* render-only amplification, documented above */
    float steer[WHEEL_COUNT];
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool isFront = (i == WHEEL_FRONT_LEFT || i == WHEEL_FRONT_RIGHT);
        steer[i] = isFront ? draw->wheelAngleRad[i] * steerVisualGain : 0.0f;
    }

    car_sprites_draw(&s_car, &visual, units_world_to_render_px(draw->positionM, ppm),
                     draw->headingRad, steer, CAR_WORLD_TEXEL_SCALE);
}

/* Release before the module is swapped: the handles belong to this module's statics and
 * would be dangling GPU names after the reload. */
void render_vehicle_resources_unload(void)
{
    car_sprites_unload(&s_car);
}

/* Nothing to re-acquire eagerly. Clearing the key makes the next draw rebake, which is
 * both simpler and safer than baking here without a spec in hand. */
void render_vehicle_resources_reset(void)
{
    s_carTexKey = 0u;
    s_carTexPxPerM = 0.0f;
}

/* ------------------------------------------------------------------------- gallery ----
 *
 * See render.h for the contract. Layout numbers, and why they are what they are:
 *
 *   GALLERY_COLS/ROWS   4 x 4 cells, so a cell is exactly 320 x 180 native pixels and the
 *                       grid divides SCREEN_W and SCREEN_H without a remainder.
 *   GALLERY_SCALE       2 destination pixels per texel — integer, like the world pass, so
 *                       the cars stay pixel art and are not smeared by a fractional resize.
 *   bake scale          the world scale, PIXELS_PER_METER * CAMERA_BASE_ZOOM, ONE common
 *                       value for every car. Fitting each cell to its own car would erase
 *                       the size axis, which is the single most informative thing the
 *                       gallery shows: a kei car and a bus have to look different sizes.
 *
 * At that scale the longest vehicle in the corpus (the bus, 12 m) is about 316 px, which is
 * what sets the cell width. A car wider than its cell overhangs rather than being rescaled —
 * a visible, honest overflow rather than a quiet lie about scale.
 */
#define GALLERY_COLS 4
#define GALLERY_ROWS 4
#define GALLERY_PER_PAGE (GALLERY_COLS * GALLERY_ROWS)
#define GALLERY_SCALE 2.0f
/* Front wheels turned so lock, Ackermann and static toe are all legible in a still image. */
#define GALLERY_STEER_RAD (8.0f * DRIFTY_DEG2RAD)

int render_gallery_page_count(void)
{
    const int count = car_corpus_count();
    if (count <= 0) return 0;
    return (count + GALLERY_PER_PAGE - 1) / GALLERY_PER_PAGE;
}

void render_draw_gallery(struct Game *game, int page)
{
    const int pageCount = render_gallery_page_count();
    const float ppm = (game != NULL) ? game->renderPixelsPerMeter : (float)PIXELS_PER_METER;
    const float bakePxPerM = car_bake_px_per_m(ppm);

    if (page < 1 || page > pageCount) {
        BeginDrawing();
        ClearBackground((Color){ 21, 24, 29, 255 });
        DrawText(TextFormat("gallery page %d is out of range (1..%d)", page, pageCount), 24, 24,
                 20, COL_TEXT);
        EndDrawing();
        return;
    }

    /* ONE PAGE of sprites is uploaded at a time — sixteen cars, not the whole corpus.
     *
     * Bake-draw-discard PER CELL was the obvious shape and is wrong: raylib batches draw
     * calls and only resolves texture ids when the batch is flushed, so unloading a texture
     * between DrawTexturePro and the end of the frame leaves the batch pointing at a freed
     * id — which renders as whatever happens to be bound, in practice the default font atlas.
     * Every car came out as a block of glyphs. So: bake the page, draw the page, then release
     * once the frame has actually been submitted. */
    static CarSprites cells[GALLERY_PER_PAGE];
    static CarVisual visuals[GALLERY_PER_PAGE];
    bool baked[GALLERY_PER_PAGE];

    const int first = (page - 1) * GALLERY_PER_PAGE;
    const int count = car_corpus_count();

    for (int slot = 0; slot < GALLERY_PER_PAGE; slot++) {
        const int index = first + slot;
        baked[slot] = false;
        memset(&cells[slot], 0, sizeof(cells[slot]));
        if (index >= count) continue;

        VehicleSpec spec;
        if (!car_corpus_spec(index, &spec)) continue;
        car_visual_derive(&spec, &visuals[slot]);
        baked[slot] = car_sprites_bake(&cells[slot], &visuals[slot], bakePxPerM);
    }

    const int cellW = SCREEN_W / GALLERY_COLS;
    const int cellH = SCREEN_H / GALLERY_ROWS;

    float steer[WHEEL_COUNT];
    for (int w = 0; w < WHEEL_COUNT; w++) {
        const bool isFront = (w == WHEEL_FRONT_LEFT || w == WHEEL_FRONT_RIGHT);
        steer[w] = isFront ? GALLERY_STEER_RAD : 0.0f;
    }

    BeginDrawing();
    ClearBackground((Color){ 21, 24, 29, 255 });

    for (int slot = 0; slot < GALLERY_PER_PAGE; slot++) {
        const int index = first + slot;
        if (index >= count) break;

        const int col = slot % GALLERY_COLS;
        const int row = slot / GALLERY_COLS;
        const int cellX = col * cellW;
        const int cellY = row * cellH;

        /* Cell plate, so a car with a dark palette still reads against the background. */
        DrawRectangle(cellX + 2, cellY + 2, cellW - 4, cellH - 4, (Color){ 29, 33, 40, 255 });

        if (baked[slot]) {
            const Vector2 centrePx = { (float)cellX + (float)cellW * 0.5f,
                                       (float)cellY + (float)cellH * 0.5f - 8.0f };
            car_sprites_draw(&cells[slot], &visuals[slot], centrePx, 0.0f, steer,
                             GALLERY_SCALE);
        }

        char id[128], note[192];
        car_corpus_id(index, id, sizeof(id));
        car_corpus_describe(index, note, sizeof(note));
        DrawText(id, cellX + 8, cellY + cellH - 34, 10, COL_COOL);
        DrawText(note, cellX + 8, cellY + cellH - 20, 10, COL_TEXT_DIM);
    }

    DrawText(TextFormat("vehicle corpus - page %d / %d - %d vehicles - %.1f px/m x%d", page,
                        pageCount, count, (double)bakePxPerM, (int)GALLERY_SCALE),
             12, 8, 12, COL_TEXT);

    EndDrawing();

    /* The frame is submitted; the ids are resolved and these are safe to release. */
    for (int slot = 0; slot < GALLERY_PER_PAGE; slot++) {
        if (baked[slot]) car_sprites_unload(&cells[slot]);
    }
}

#endif /* !DRIFTY_HEADLESS */
