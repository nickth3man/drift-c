/*
 * car_visual_raster.c — CPU rasterizer for CarVisual. See the header for the contract.
 *
 * One geometry pass writes both an RGBA colour buffer and a feature-label buffer (either may
 * be omitted), so the label map used by the distinctness test can never describe a different
 * shape from the one actually drawn.
 *
 * Hard-edged fills only: a pixel centre is inside a shape or it is not. That is what the
 * reference sprites in resources/sprite_examples/ do, and it keeps the output exactly
 * reproducible.
 *
 * Raylib-free: linked into drifty_tests.exe.
 *
 * =============================================================== LAYER STACK, fixed order ===
 *
 *   L0  shadow / ground contact       — offset dark translucent ellipse, drawn first
 *   L1  body silhouette + outline     — hull polygon, dark outline plate underneath
 *   L2  body secondary shading        — rear-half darker band, two-tone read
 *   L3  greenhouse roof panel         — cabin roof rectangle
 *   L4  glass: windscreen, backlight,  — glass bands; segmented for van/bus
 *        side windows
 *   L5  lights: head/tail lamps        — front lamps (warm) + rear lamps (red)
 *   L6  wheels, rims, tires, brakes,   — arch brow, then tread / sidewall / rim / disc
 *        arches, sidewall details
 *   L7  appendages: wing, splitter,    — drawn after the body so they read as mounted ON it
 *        canards, mirrors, exhaust,
 *        bed rails, hood bulge, tow
 *        hook, hood pins
 *   L8  livery: stripes, panels        — decorative, colour-seeded; geometry from body extents
 *   L9  heading marker                 — small accent chevron at nose (gameplay affordance)
 *
 * THE IN-FILE DRAW ORDER IS THE STACK, IN ORDER. No caller may reorder features, and no
 * feature may be hoisted out of its layer — in particular the wheels are L6, drawn OVER the
 * body silhouette, not under it. That is both what the layer table says and what the
 * reference sprites in resources/sprite_examples/ show: the tires read as dark blocks at the
 * four corners, overlapping the bodywork. Drawing them underneath hid the tire geometry
 * almost completely, which made tire-derived features invisible in the pixel metric.
 *
 * The shadow layer (L0) is drawn first and its pixels stay CAR_LABEL_EMPTY (its alpha is
 * below the label threshold in put_px), so a shadow can never inflate the distinctness pixel
 * ratio. The heading marker (L9) is drawn last, on top of everything.
 */
#include "render/car_visual_raster.h"

#include <math.h>
#include <string.h>

#include "core/math_utils.h"

#define MAX_POLY_POINTS (2 * CAR_HULL_STATIONS + 8)

/* Where a fill writes. Either channel may be NULL. `clipHull` is optional and is used only
 * for paint and fittings that must stay inside the body silhouette. */
typedef struct {
    unsigned char *rgba;
    unsigned char *labels;
    int width;
    int height;
    float pxPerM;
    float originXPx;
    float originYPx;
    const CarVisual *clipHull;
} RasterTarget;

/* ------------------------------------------------------------------------- primitives -- */

/* The hull polygon connects its nine stations with straight segments. Test the same geometry
 * at the pixel centre so a body-clipped fill cannot create silhouette pixels of its own. */
static bool pixel_inside_clip_hull(const RasterTarget *t, int x, int y)
{
    if (t->clipHull == NULL) return true;

    const CarVisual *v = t->clipHull;
    const int last = CAR_HULL_STATIONS - 1;
    const float xM = (((float)x + 0.5f) - t->originXPx) / t->pxPerM;
    const float yM = (t->originYPx - ((float)y + 0.5f)) / t->pxPerM;

    if (xM < v->hull[0].xM || xM > v->hull[last].xM) return false;

    int station = 0;
    while (station < last - 1 && xM > v->hull[station + 1].xM) station++;

    const float x0 = v->hull[station].xM;
    const float x1 = v->hull[station + 1].xM;
    const float u = (x1 > x0) ? clampf((xM - x0) / (x1 - x0), 0.0f, 1.0f) : 0.0f;
    const float halfWidth = lerpf(v->hull[station].halfWidthM,
                                  v->hull[station + 1].halfWidthM, u);
    return fabsf(yM) <= halfWidth;
}

static void put_px(const RasterTarget *t, int x, int y, Color c, unsigned char label)
{
    if (x < 0 || y < 0 || x >= t->width || y >= t->height) return;
    if (!pixel_inside_clip_hull(t, x, y)) return;
    const size_t index = (size_t)y * (size_t)t->width + (size_t)x;

    if (t->rgba != NULL) {
        unsigned char *p = t->rgba + index * CAR_RASTER_BPP;
        if (c.a >= 255) {
            p[0] = c.r;
            p[1] = c.g;
            p[2] = c.b;
            p[3] = 255;
        } else if (c.a > 0) {
            /* Straight-alpha source-over. */
            const float sa = (float)c.a / 255.0f;
            const float da = (float)p[3] / 255.0f;
            const float oa = sa + da * (1.0f - sa);
            if (oa > 0.0f) {
                for (int k = 0; k < 3; k++) {
                    const float sc = (k == 0) ? (float)c.r : (k == 1) ? (float)c.g : (float)c.b;
                    const float dc = (float)p[k];
                    p[k] = (unsigned char)clampf((sc * sa + dc * da * (1.0f - sa)) / oa, 0.0f,
                                                 255.0f);
                }
                p[3] = (unsigned char)clampf(oa * 255.0f, 0.0f, 255.0f);
            }
        }
    }

    /* A label is identity, not paint: a translucent wash does not change what a pixel IS.
     * Shadow pixels (CAR_LABEL_EMPTY) cannot inflate distinctness. */
    if (t->labels != NULL && c.a >= 128 && label != CAR_LABEL_EMPTY) {
        t->labels[index] = label;
    }
}

static Vector2 to_px(const RasterTarget *t, float xM, float yM)
{
    /* +X forward -> +px; +Y left -> -py, matching src/core/units.h. */
    Vector2 p;
    p.x = t->originXPx + xM * t->pxPerM;
    p.y = t->originYPx - yM * t->pxPerM;
    return p;
}

/* Even-odd scanline fill. Handles convex and mildly concave outlines alike. */
static void fill_polygon_px(const RasterTarget *t, const Vector2 *pts, int count, Color c,
                            unsigned char label)
{
    if (pts == NULL || count < 3) return;

    // cppcheck-suppress duplicateAssignExpression
    float minY = pts[0].y;
    float maxY = pts[0].y;
    for (int i = 1; i < count; i++) {
        if (pts[i].y < minY) minY = pts[i].y;
        if (pts[i].y > maxY) maxY = pts[i].y;
    }

    int y0 = (int)floorf(minY);
    int y1 = (int)ceilf(maxY);
    if (y0 < 0) y0 = 0;
    if (y1 > t->height) y1 = t->height;

    for (int y = y0; y < y1; y++) {
        const float sy = (float)y + 0.5f;
        float xs[MAX_POLY_POINTS * 2];
        int n = 0;

        for (int i = 0, j = count - 1; i < count; j = i++) {
            const float ay = pts[j].y, by = pts[i].y;
            if ((ay <= sy && by > sy) || (by <= sy && ay > sy)) {
                const float tt = (sy - ay) / (by - ay);
                if (n < (int)(sizeof(xs) / sizeof(xs[0]))) {
                    xs[n++] = pts[j].x + tt * (pts[i].x - pts[j].x);
                }
            }
        }
        if (n < 2) continue;

        /* Insertion sort: n is tiny. */
        for (int i = 1; i < n; i++) {
            const float key = xs[i];
            int k = i - 1;
            while (k >= 0 && xs[k] > key) {
                xs[k + 1] = xs[k];
                k--;
            }
            xs[k + 1] = key;
        }

        for (int i = 0; i + 1 < n; i += 2) {
            int x0 = (int)ceilf(xs[i] - 0.5f);
            int x1 = (int)ceilf(xs[i + 1] - 0.5f);
            if (x0 < 0) x0 = 0;
            if (x1 > t->width) x1 = t->width;
            for (int x = x0; x < x1; x++) put_px(t, x, y, c, label);
        }
    }
}

/* An oriented rectangle in body space: centre, length along its own +X, width across. */
static void fill_oriented_rect(const RasterTarget *t, float cxM, float cyM, float lengthM,
                               float widthM, float angleRad, Color c, unsigned char label)
{
    if (!(lengthM > 0.0f) || !(widthM > 0.0f)) return;
    const float hl = 0.5f * lengthM, hw = 0.5f * widthM;
    const float ca = cosf(angleRad), sa = sinf(angleRad);
    const float ox[4] = { +hl, +hl, -hl, -hl };
    const float oy[4] = { +hw, -hw, -hw, +hw };

    Vector2 pts[4];
    for (int i = 0; i < 4; i++) {
        pts[i] = to_px(t, cxM + ox[i] * ca - oy[i] * sa, cyM + ox[i] * sa + oy[i] * ca);
    }
    fill_polygon_px(t, pts, 4, c, label);
}

static void fill_disc(const RasterTarget *t, float cxM, float cyM, float diameterM, Color c,
                      unsigned char label)
{
    if (!(diameterM > 0.0f)) return;
    const Vector2 centre = to_px(t, cxM, cyM);
    const float r = 0.5f * diameterM * t->pxPerM;
    const float r2 = r * r;

    int y0 = (int)floorf(centre.y - r), y1 = (int)ceilf(centre.y + r);
    int x0 = (int)floorf(centre.x - r), x1 = (int)ceilf(centre.x + r);
    if (y0 < 0) y0 = 0;
    if (x0 < 0) x0 = 0;
    if (y1 > t->height) y1 = t->height;
    if (x1 > t->width) x1 = t->width;

    for (int y = y0; y < y1; y++) {
        const float dy = ((float)y + 0.5f) - centre.y;
        for (int x = x0; x < x1; x++) {
            const float dx = ((float)x + 0.5f) - centre.x;
            if (dx * dx + dy * dy <= r2) put_px(t, x, y, c, label);
        }
    }
}

/* ------------------------------------------------------------------------ hull outline -- */

/* Build the closed outline: up the left flank tail-to-nose, back down the right flank.
 * `expandM` grows it outward, which is how the dark outline underneath is produced. */
static int build_hull(const RasterTarget *t, const CarVisual *v, float expandM, Vector2 *pts)
{
    const int last = CAR_HULL_STATIONS - 1;
    int n = 0;
    for (int i = 0; i < CAR_HULL_STATIONS; i++) {
        float x = v->hull[i].xM;
        if (i == 0) x -= expandM;
        if (i == last) x += expandM;
        pts[n++] = to_px(t, x, v->hull[i].halfWidthM + expandM);
    }
    for (int i = last; i >= 0; i--) {
        float x = v->hull[i].xM;
        if (i == 0) x -= expandM;
        if (i == last) x += expandM;
        pts[n++] = to_px(t, x, -(v->hull[i].halfWidthM + expandM));
    }
    return n;
}

/* ----------------------------------------------------------------------------- extents -- */

static void grow(float *minX, float *maxX, float *minY, float *maxY, float x, float y)
{
    if (x < *minX) *minX = x;
    if (x > *maxX) *maxX = x;
    if (y < *minY) *minY = y;
    if (y > *maxY) *maxY = y;
}

CarRasterInfo car_raster_info(const CarVisual *visual, float pxPerM, int padPx)
{
    CarRasterInfo info;
    memset(&info, 0, sizeof(info));
    if (visual == NULL || !(pxPerM > 0.0f)) return info;
    if (padPx < 0) padPx = 0;

    float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;

    for (int i = 0; i < CAR_HULL_STATIONS; i++) {
        grow(&minX, &maxX, &minY, &maxY, visual->hull[i].xM, visual->hull[i].halfWidthM);
        grow(&minX, &maxX, &minY, &maxY, visual->hull[i].xM, -visual->hull[i].halfWidthM);
    }

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const CarWheelVisual *w = &visual->wheels[i];
        const float hl = 0.5f * w->diameterM + w->archFlareM;
        const float hw = 0.5f * w->widthM + w->archFlareM + fmaxf(0.0f, w->pokeM);
        grow(&minX, &maxX, &minY, &maxY, w->centreM.x + hl, w->centreM.y + hw);
        grow(&minX, &maxX, &minY, &maxY, w->centreM.x - hl, w->centreM.y - hw);
    }

    if (visual->wingSpanM > 0.0f) {
        grow(&minX, &maxX, &minY, &maxY, visual->wingXM - 0.5f * visual->wingChordM,
             +0.5f * visual->wingSpanM);
        grow(&minX, &maxX, &minY, &maxY, visual->wingXM + 0.5f * visual->wingChordM,
             -0.5f * visual->wingSpanM);
    }
    if (visual->splitterProtrusionM > 0.0f) {
        const float noseX = visual->hull[CAR_HULL_STATIONS - 1].xM;
        grow(&minX, &maxX, &minY, &maxY, noseX + visual->splitterProtrusionM,
             +0.5f * visual->splitterWidthM);
        grow(&minX, &maxX, &minY, &maxY, noseX, -0.5f * visual->splitterWidthM);
    }
    if (visual->mirrorOffsetM > 0.0f) {
        grow(&minX, &maxX, &minY, &maxY, visual->windscreenXM, +visual->mirrorOffsetM + 0.06f);
        grow(&minX, &maxX, &minY, &maxY, visual->windscreenXM, -visual->mirrorOffsetM - 0.06f);
    }
    /* Canard extent. */
    if (visual->canardStrength > 0.01f) {
        const float noseX = visual->hull[CAR_HULL_STATIONS - 1].xM;
        const float canardLen = 0.10f + 0.15f * visual->canardStrength;
        grow(&minX, &maxX, &minY, &maxY, noseX + canardLen * 0.5f, visual->widthM * 0.48f);
        grow(&minX, &maxX, &minY, &maxY, noseX + canardLen * 0.5f, -visual->widthM * 0.48f);
    }

    /* One extra pixel of body space for the dark outline drawn under the hull.
     * Plus extra for the L0 shadow offset. */
    const float outlineM = 1.0f / pxPerM;
    const float shadowOffsetM = 0.12f;
    minX -= outlineM;
    maxX += outlineM + shadowOffsetM;
    minY -= outlineM + 0.08f;
    maxY += outlineM + 0.08f;

    info.pxPerM = pxPerM;
    info.width = (int)ceilf((maxX - minX) * pxPerM) + 2 * padPx;
    info.height = (int)ceilf((maxY - minY) * pxPerM) + 2 * padPx;
    if (info.width < 1) info.width = 1;
    if (info.height < 1) info.height = 1;
    info.originXPx = (float)padPx - minX * pxPerM;
    info.originYPx = (float)padPx + maxY * pxPerM;
    return info;
}

CarRasterInfo car_raster_part_info(const CarVisual *visual, CarRasterPart part, int wheelIndex,
                                   float pxPerM, int padPx)
{
    if (part != CAR_RASTER_PART_WHEEL) {
        /* BODY shares the car's bounds with ALL: the wheel arches are body geometry and
         * reach wherever the track puts the wheels, so anything tighter would clip them. */
        return car_raster_info(visual, pxPerM, padPx);
    }

    CarRasterInfo info;
    memset(&info, 0, sizeof(info));
    if (visual == NULL || !(pxPerM > 0.0f)) return info;
    if (wheelIndex < 0 || wheelIndex >= WHEEL_COUNT) return info;
    if (padPx < 0) padPx = 0;

    const CarWheelVisual *w = &visual->wheels[wheelIndex];
    /* About the hub, so the caller can rotate the sprite to the steer angle. Half-extents
     * cover the largest ring drawn — the tread — in both directions. */
    const float halfLen = 0.5f * maxf(w->diameterM, w->rimDiameterM);
    const float halfWid = 0.5f * maxf(w->widthM, w->rimWidthM);

    info.pxPerM = pxPerM;
    info.width = (int)ceilf(2.0f * halfLen * pxPerM) + 2 * padPx;
    info.height = (int)ceilf(2.0f * halfWid * pxPerM) + 2 * padPx;
    if (info.width < 1) info.width = 1;
    if (info.height < 1) info.height = 1;
    info.originXPx = 0.5f * (float)info.width;
    info.originYPx = 0.5f * (float)info.height;
    return info;
}

size_t car_raster_bytes(CarRasterInfo info)
{
    if (info.width <= 0 || info.height <= 0) return 0;
    return (size_t)info.width * (size_t)info.height * CAR_RASTER_BPP;
}

/* ------------------------------------------------------------------------------ render -- */

/* The tire stack: tread, sidewall, rim, disc, outside in. Drawn about (cxM, cyM) at
 * `angleRad`, which is what lets the same code serve the in-place wheel of a contact sheet
 * and the standalone, steerable wheel sprite the game composites. */
static void draw_wheel_stack(const RasterTarget *t, const CarVisual *v, const CarWheelVisual *w,
                             float cxM, float cyM, float angleRad)
{
    const float visualWidth = w->widthM * w->camberVisualCos;

    fill_oriented_rect(t, cxM, cyM, w->diameterM, visualWidth, angleRad, v->tire,
                       CAR_LABEL_TIRE);

    /* Sidewall ring — between tread and rim. Slightly different shade from tread.
     * Only drawn when the sidewall is thick enough to read (> 0.5 px). */
    if (w->sidewallHeightM > 0.005f) {
        /* The sidewall takes the inner 70% of the band between the rim and the tread, the
         * tread the outer 30%. Proportional rather than a fixed depth, so a 25-series and
         * an 80-series tire differ in where the boundary sits as well as in how big the
         * tire is — two moving edges instead of one, which is the difference between the
         * profile reading at 13 px/m and not reading at all. */
        const float swDia = w->rimDiameterM + 2.0f * w->sidewallHeightM * 0.70f;
        fill_oriented_rect(t, cxM, cyM, swDia, visualWidth * 0.88f, angleRad, v->tireSidewall,
                           CAR_LABEL_TIRE_SIDEWALL);
    }

    /* Rim barrel. Width reflects the actual rim width from the designation. */
    fill_oriented_rect(t, cxM, cyM, w->rimDiameterM, maxf(w->rimWidthM, visualWidth * 0.55f),
                       angleRad, v->rim, CAR_LABEL_RIM);

    /* Brake disc — centre of the rim. */
    if (w->discDiameterM > 0.0f && w->discDiameterM < w->rimDiameterM) {
        fill_oriented_rect(t, cxM, cyM, w->discDiameterM, w->rimWidthM * 0.50f, angleRad,
                           v->disc, CAR_LABEL_DISC);
    }
}

static void render(const CarVisual *v, const RasterTarget *t, CarRasterPart part,
                   int wheelIndex)
{
    if (part == CAR_RASTER_PART_WHEEL) {
        if (wheelIndex < 0 || wheelIndex >= WHEEL_COUNT) return;
        /* Axis-aligned about its own hub. The static toe angle is the CALLER's to apply, on
         * top of heading and steer — see the pivot contract in car_visual_raster.h. */
        draw_wheel_stack(t, v, &v->wheels[wheelIndex], 0.0f, 0.0f, 0.0f);
        return;
    }
    const bool withTires = (part == CAR_RASTER_PART_ALL);

    Vector2 poly[MAX_POLY_POINTS];
    const float onePx = 1.0f / t->pxPerM;
    const float noseX = v->hull[CAR_HULL_STATIONS - 1].xM;
    const float tailX = v->hull[0].xM;

    /* ============================================================= L0: shadow =====
     *
     * Offset dark translucent shape under the body. Shadow pixels use CAR_LABEL_EMPTY
     * (actually we pass CAR_LABEL_SHADOW but put_px skips labels for alpha < 255 when
     * label is non-EMPTY... wait: the label assignment only happens when alpha >= 128,
     * and shadow alpha is ~60. So shadow pixels get no label, which is correct: they
     * don't inflate the distinctness metric.
     *
     * We draw a simple dark rectangle offset slightly downward-right from the body. */
    {
        const Color shadowColor = (Color){ 10, 12, 16, 55 };
        const float shadowOffX = 0.06f; /* shadow offset rearward */
        const float shadowOffY = 0.04f; /* shadow offset leftward */
        const float shadowLen = v->lengthM + 0.08f;
        const float shadowWid = v->widthM * 0.92f;
        fill_oriented_rect(t, 0.0f - shadowOffX, 0.0f - shadowOffY, shadowLen, shadowWid, 0.0f,
                           shadowColor, CAR_LABEL_SHADOW);
    }

    /* =================================================== L1 + L2: body silhouette =====
     *
     * L1: dark outline plate expanded by one pixel, then the body colour on top.
     * L2: rear half a shade darker so front and rear read apart at a glance. */
    fill_polygon_px(t, poly, build_hull(t, v, onePx, poly), v->outline, CAR_LABEL_OUTLINE);
    fill_polygon_px(t, poly, build_hull(t, v, 0.0f, poly), v->body, CAR_LABEL_BODY);

    /* Everything mounted on or painted onto the body uses the exact hull as a pixel clip.
     * Appendages, arches, wheels, exhausts, and the heading marker continue to use `t` so they
     * may intentionally extend beyond the silhouette. */
    RasterTarget bodyTarget = *t;
    bodyTarget.clipHull = v;
    const RasterTarget *body = &bodyTarget;

    {
        const float rearLen = 0.42f * (noseX - tailX);
        fill_oriented_rect(body, tailX + 0.5f * rearLen, 0.0f, rearLen,
                           2.0f * v->hull[1].halfWidthM, 0.0f,
                           (Color){ v->bodyShade.r, v->bodyShade.g, v->bodyShade.b, 150 },
                           CAR_LABEL_BODY_SHADE);
    }

    /* ========================================================== L3: greenhouse =====
     *
     * A dark opening is laid down first. Fixed-roof panels cover it completely, targa
     * panels leave their derived centre gap, and convertibles leave the opening exposed.
     * All geometry was decided by car_visual_derive(). */
    fill_oriented_rect(body, 0.5f * (v->roofStartXM + v->roofEndXM), 0.0f, v->roofLengthM,
                       v->roofWidthM, 0.0f, v->glass, CAR_LABEL_GLASS);
    for (int i = 0; i < v->roofPanelCount; i++) {
        fill_oriented_rect(body, v->roofPanels[i].centreXM, 0.0f, v->roofPanels[i].lengthM,
                           v->roofWidthM, 0.0f, v->cabin, CAR_LABEL_CABIN);
    }
    /* L3 body-coloured roof highlight; every L4 glass feature is painted afterwards. */
    for (int i = 0; i < v->roofPanelCount; i++) {
        fill_oriented_rect(body, v->roofPanels[i].centreXM, 0.0f,
                           v->roofPanels[i].lengthM * v->roofHighlightLengthScale,
                           v->roofHighlightWidthM, 0.0f, v->body, CAR_LABEL_BODY);
    }

    /* ============================================================= L4: glass ===== */

    const float sideY = 0.5f * (v->roofWidthM - v->sideWindowBandWidthM);
    for (int i = 0; i < v->sideWindowCount; i++) {
        for (int side = -1; side <= 1; side += 2) {
            fill_oriented_rect(body, v->sideWindows[i].centreXM, (float)side * sideY,
                               v->sideWindows[i].lengthM, v->sideWindowBandWidthM, 0.0f,
                               v->glass, CAR_LABEL_GLASS);
        }
    }
    for (int side = -1; side <= 1; side += 2) {
        fill_oriented_rect(body, v->quarterWindow.centreXM, (float)side * sideY,
                           v->quarterWindow.lengthM, v->sideWindowBandWidthM, 0.0f, v->glass,
                           CAR_LABEL_QUARTER_WINDOW);
    }
    fill_oriented_rect(body, v->windscreenXM - 0.5f * v->windscreenLengthM, 0.0f,
                       v->windscreenLengthM, 2.0f * v->glassHalfWidthM, 0.0f, v->glass,
                       CAR_LABEL_GLASS);
    fill_oriented_rect(body, v->backlightXM + 0.5f * v->backlightLengthM, 0.0f,
                       v->backlightLengthM, 1.88f * v->glassHalfWidthM, 0.0f, v->glass,
                       CAR_LABEL_GLASS);
    fill_oriented_rect(body, v->sunroof.centreXM, 0.0f, v->sunroof.lengthM,
                       0.56f * v->roofWidthM, 0.0f, v->glass, CAR_LABEL_SUNROOF);

    /* ============================================================== L4b: cage ===== */
    if (v->hasCage && v->cabinLengthM > 0.0f) {
        fill_oriented_rect(body, v->cabinCentreXM, 0.0f, v->cabinLengthM, 2.0f * onePx, 0.0f,
                           v->outline, CAR_LABEL_CAGE);
        fill_oriented_rect(body, v->cabinCentreXM, 0.0f, 2.0f * onePx,
                           2.0f * v->cabinHalfWidthM, 0.0f, v->outline, CAR_LABEL_CAGE);
    }

    /* ========================================================= L5: lights ===== */
    {
        const float lampLen = 0.10f, lampWid = 0.26f;
        const float noseHalf = v->hull[CAR_HULL_STATIONS - 1].halfWidthM;
        const float tailHalf = v->hull[0].halfWidthM;
        for (int s = -1; s <= 1; s += 2) {
            /* Headlights: warm white at the nose. */
            fill_oriented_rect(body, noseX - 0.5f * lampLen, (float)s * noseHalf * 0.55f,
                               lampLen, lampWid, 0.0f, v->lamp, CAR_LABEL_LAMP);
            /* Tail/brake lights: red at the tail. */
            fill_oriented_rect(body, tailX + 0.5f * lampLen, (float)s * tailHalf * 0.55f,
                               lampLen, lampWid, 0.0f, (Color){ 200, 48, 40, 255 },
                               CAR_LABEL_LAMP);
        }
    }

    /* ======================================================== L6: wheels =====
     *
     * Drawn OVER the body silhouette, which is both what the layer table specifies and what
     * the reference sprites show. Where the track exceeds the hull width the wheels also
     * stand outboard of the body, which is how a wide-track car gets its stance with no
     * special case anywhere.
     *
     * Per wheel, outside in:
     *   arch brow   — fender lip, sized by tire diameter + arch gap + flare (drawn first,
     *                 so the tire sits inside it)
     *   tread       — full tire diameter x visual width (camber-narrowed)
     *   sidewall    — the ring between the tread edge and the rim edge; its thickness is
     *                 the tire's aspect ratio made visible
     *   rim barrel  — rim diameter x rim width from the designation
     *   brake disc  — inside the rim
     */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const CarWheelVisual *w = &v->wheels[i];

        /* A declared bolt-on flare is bodywork that stands PROUD of the hull, outboard of the
         * tire it covers — that is what body.fender_flare_[front|rear] means, and car_raster_info
         * above already reserves `archFlareM` of outboard room for it. Draw it there.
         *
         * Without this the declared flare only lengthened the inboard brow band below, which
         * moved 0.0102 (front) and 0.0136 (rear) of the silhouette against a 0.015 floor: the
         * signature moved the full declared metre while the picture barely changed, which is
         * exactly the "computes correctly, covers no pixels" defect the sensitivity gate exists
         * to catch.
         *
         * Only the DECLARED part stands outboard. The shared derived flare keeps the inboard
         * brow it has always had, so a car that declares no flare rasterizes unchanged. */
        const bool isFront = (i == WHEEL_FRONT_LEFT || i == WHEEL_FRONT_RIGHT);
        const float declaredFlareM = isFront ? v->fenderFlareFrontM : v->fenderFlareRearM;
        const float outboard = (w->centreM.y > 0.0f) ? 1.0f : -1.0f;

        /* Arch brow at the outboard edge; arch gap from rideHeight + suspTravel sets how far
         * the lip stands off the tire. The arch is BODYWORK and stays with the body sprite:
         * a fender does not steer with the wheel it covers. */
        if (w->archFlareM > 0.0f) {
            const float archDia = w->diameterM + 2.0f * w->archGapM + 2.0f * w->archFlareM;
            const float archBandM = 2.0f * onePx;

            if (declaredFlareM > 0.0f) {
                fill_oriented_rect(t, w->centreM.x,
                                   w->centreM.y +
                                       outboard * (0.5f * w->widthM + 0.5f * declaredFlareM),
                                   archDia, declaredFlareM, 0.0f, v->bodyShade, CAR_LABEL_ARCH);
            }

            fill_oriented_rect(t, w->centreM.x,
                               w->centreM.y - outboard * (0.5f * w->widthM + 0.5f * archBandM),
                               archDia, archBandM, 0.0f, v->bodyShade, CAR_LABEL_ARCH);
        }

        if (withTires) {
            draw_wheel_stack(t, v, w, w->centreM.x, w->centreM.y, w->staticAngleRad);
        }
    }

    /* ======================================================== L7: appendages ===== */

    /* L7a: splitter — a dark lip ahead of the bumper, not an accent-coloured appendage. */
    if (v->splitterProtrusionM > 0.0f) {
        fill_oriented_rect(t, noseX + 0.5f * v->splitterProtrusionM - onePx, 0.0f,
                           v->splitterProtrusionM + 2.0f * onePx, v->splitterWidthM, 0.0f,
                           v->outline, CAR_LABEL_SPLITTER);
    }

    /* L7b: wing over the deck. Two-tone: accent colour with a dark centre strip so it
     * reads as a wing profile, not a coloured rectangle. */
    if (v->wingSpanM > 0.0f) {
        fill_oriented_rect(t, v->wingXM, 0.0f, v->wingChordM, v->wingSpanM, 0.0f, v->accent,
                           CAR_LABEL_WING);
        fill_oriented_rect(t, v->wingXM, 0.0f, v->wingChordM * 0.34f, v->wingSpanM, 0.0f,
                           v->outline, CAR_LABEL_WING);
    }

    /* L7c: canards — small fins at front corners. Only drawn when canardStrength > threshold
     * (smoothstep transition band in derive ensures no pop-in). */
    if (v->canardStrength > 0.01f) {
        const float canardLen = 0.10f + 0.15f * v->canardStrength;
        const float canardWid = v->canardStrength * 0.05f + onePx;
        const float canardY = v->widthM * 0.44f;
        for (int s = -1; s <= 1; s += 2) {
            fill_oriented_rect(t, noseX - canardLen * 0.25f, (float)s * canardY, canardLen,
                               canardWid, (float)s * 0.35f, v->accent, CAR_LABEL_CANARD);
        }
    }

    /* L7d: mirrors on their stalks. */
    if (v->hasMirrors && v->mirrorOffsetM > 0.0f) {
        for (int s = -1; s <= 1; s += 2) {
            fill_oriented_rect(t, v->windscreenXM, (float)s * v->mirrorOffsetM, 0.14f, 0.10f,
                               0.0f, v->bodyShade, CAR_LABEL_MIRROR);
        }
    }

    /* L7e: exhaust tips at the tail, spaced across the centreline. exhaustTransition scales
     * the bore for area-continuity: at a count threshold the pipe count doubles while bore
     * drops to sqrt(1/2), so total exhaust area stays continuous instead of popping. */
    if (v->exhaustCount > 0 && v->exhaustBoreM > 0.0f) {
        const float boreScale = 0.70710678f + 0.29289322f * v->exhaustTransition;
        const float bore = v->exhaustBoreM * boreScale;
        const float spacing = bore * 1.8f;
        const float first = -0.5f * spacing * (float)(v->exhaustCount - 1);
        for (int i = 0; i < v->exhaustCount; i++) {
            fill_disc(t, tailX + 0.5f * bore, first + spacing * (float)i, bore,
                      (Color){ 70, 74, 82, 255 }, CAR_LABEL_EXHAUST);
        }
    }

    /* L7f: pickup bed — bed floor and rails, spanning bedLengthM forward from the tail.
     * The length is the declared one (car_visual.c already clamped it to the space behind the
     * greenhouse), NOT the whole stretch behind the backlight: a boot is not a bed. */
    if (v->pickupBedWeight > 0.05f && v->bedLengthM > 0.0f) {
        const float bedEndX = tailX;
        const float bedLen = maxf(v->bedLengthM, 0.05f);
        const float bedWid = v->widthM * 0.78f * v->pickupBedWeight;
        if (bedLen > 0.0f && bedWid > 0.0f) {
            /* Bed floor: darker rectangle covering the rear body. */
            fill_oriented_rect(body, bedEndX + 0.5f * bedLen, 0.0f, bedLen, bedWid, 0.0f,
                               v->cabin, CAR_LABEL_BED);
            /* Bed rails: thin outline strips at the bed edges. */
            for (int s = -1; s <= 1; s += 2) {
                fill_oriented_rect(body, bedEndX + 0.5f * bedLen, (float)s * bedWid * 0.50f,
                                   bedLen, onePx * 1.5f, 0.0f, v->outline, CAR_LABEL_BED);
            }
        }
    }

    /* L7g: hood bulge — from engine displacement on front-engine cars. */
    if (v->hoodBulgeStrength > 0.01f) {
        /* Hood centre between nose and cowl. */
        const float hoodCentreX = (noseX + v->windscreenXM) * 0.5f;
        const float hoodLen = maxf(noseX - v->windscreenXM, 0.4f);
        const float bulgeW = v->cabinHalfWidthM * 0.38f * v->hoodBulgeStrength;
        /* Slightly lighter than cabin to read as a raised surface. */
        const Color bulgeColor = (Color){ (unsigned char)minf((int)v->cabin.r + 25, 255),
                                          (unsigned char)minf((int)v->cabin.g + 25, 255),
                                          (unsigned char)minf((int)v->cabin.b + 25, 255), 255 };
        fill_oriented_rect(body, hoodCentreX, 0.0f, hoodLen * 0.65f, bulgeW * 2.0f, 0.0f,
                           bulgeColor, CAR_LABEL_HOOD_BULGE);
    }

    /* L7h: race-detail markers. Their presentation dimensions are derived by the grammar. */
    if (v->towHookDiameterM > 0.0f) {
        fill_disc(t, v->towHookXM, 0.0f, v->towHookDiameterM, v->accent, CAR_LABEL_TOW_HOOK);
    }
    if (v->hoodPinDiameterM > 0.0f) {
        const float pinX = noseX - 0.25f;
        const float pinY = v->widthM * 0.16f;
        for (int s = -1; s <= 1; s += 2) {
            fill_disc(body, pinX, (float)s * pinY, v->hoodPinDiameterM, v->accent,
                      CAR_LABEL_HOOD_PINS);
        }
    }

    /* ==================================================== L8: stripes / livery =====
     *
     * Racing stripes along the body centreline. Stripe placement is a fixed function of
     * stripeWeight and body extents; stripe COLOUR uses colour-seed bits (the only
     * seed-dependent part of stripes, documented). Stripes ramp in smoothly — a 0.001
     * change in strip01 never snaps full-width stripes into existence. */
    if (v->stripeWeight > 0.05f) {
        const float stripeW = v->widthM * 0.07f * v->stripeWeight;
        const float stripeLen = v->lengthM * (0.70f + 0.25f * v->stripeWeight);
        const float stripeGap = v->widthM * 0.10f * v->stripeWeight;
        /* Two stripes offset from centreline. */
        for (int s = -1; s <= 1; s += 2) {
            fill_oriented_rect(body, 0.0f, (float)s * stripeGap, stripeLen, stripeW, 0.0f,
                               v->stripeColor, CAR_LABEL_STRIPE);
        }
    }

    /* ==================================================== L9: heading marker =====
     *
     * Small chevron/triangle at the nose, in CarVisual.heading — the one colour in the
     * palette that is deliberately the body's complement, because this is a gameplay
     * affordance and must not sink into the paint. This is the affordance previously drawn
     * in src/render/render.c — now part of the shared raster so headless and in-game output are
     * pixel-identical. */
    {
        const float tipLen = 0.75f * v->headingLengthM;
        const float baseSetback = 0.25f * v->headingLengthM;
        const Vector2 tip = to_px(t, noseX + tipLen, 0.0f);
        const Vector2 lb = to_px(t, noseX - baseSetback, +v->headingHalfWidthM);
        const Vector2 rb = to_px(t, noseX - baseSetback, -v->headingHalfWidthM);
        const Vector2 pts[3] = { tip, lb, rb };
        fill_polygon_px(t, pts, 3, v->heading, CAR_LABEL_HEADING);
    }
}

static bool prepare(const CarVisual *visual, CarRasterInfo info, RasterTarget *t)
{
    if (visual == NULL || info.width <= 0 || info.height <= 0 || !(info.pxPerM > 0.0f)) {
        return false;
    }
    t->width = info.width;
    t->height = info.height;
    t->pxPerM = info.pxPerM;
    t->originXPx = info.originXPx;
    t->originYPx = info.originYPx;
    return true;
}

bool car_raster_draw_part(const CarVisual *visual, CarRasterPart part, int wheelIndex,
                          CarRasterInfo info, unsigned char *rgba, size_t bytes)
{
    RasterTarget t;
    memset(&t, 0, sizeof(t));
    if (rgba == NULL || bytes < car_raster_bytes(info)) return false;
    if (!prepare(visual, info, &t)) return false;
    if (part == CAR_RASTER_PART_WHEEL && (wheelIndex < 0 || wheelIndex >= WHEEL_COUNT))
        return false;

    memset(rgba, 0, car_raster_bytes(info));
    t.rgba = rgba;
    t.labels = NULL;
    render(visual, &t, part, wheelIndex);
    return true;
}

bool car_raster_draw(const CarVisual *visual, CarRasterInfo info, unsigned char *rgba,
                     size_t bytes)
{
    return car_raster_draw_part(visual, CAR_RASTER_PART_ALL, 0, info, rgba, bytes);
}

bool car_raster_draw_labels(const CarVisual *visual, CarRasterInfo info, unsigned char *labels,
                            size_t bytes)
{
    RasterTarget t;
    memset(&t, 0, sizeof(t));
    const size_t need = (size_t)info.width * (size_t)info.height;
    if (labels == NULL || info.width <= 0 || info.height <= 0 || bytes < need) return false;
    if (!prepare(visual, info, &t)) return false;

    memset(labels, CAR_LABEL_EMPTY, need);
    t.rgba = NULL;
    t.labels = labels;
    render(visual, &t, CAR_RASTER_PART_ALL, 0);
    return true;
}

bool car_raster_rotate_nose_up(const unsigned char *src, int srcW, int srcH, unsigned char *dst,
                               size_t dstBytes)
{
    if (src == NULL || dst == NULL || srcW <= 0 || srcH <= 0) return false;
    const size_t need = (size_t)srcW * (size_t)srcH * CAR_RASTER_BPP;
    if (dstBytes < need) return false;

    /* Rotate CCW: the nose (+X, at large source x) ends up at small destination y. */
    const int dstW = srcH;
    for (int y = 0; y < srcH; y++) {
        for (int x = 0; x < srcW; x++) {
            const int dx = y;
            const int dy = srcW - 1 - x;
            const size_t s = ((size_t)y * (size_t)srcW + (size_t)x) * CAR_RASTER_BPP;
            const size_t d = ((size_t)dy * (size_t)dstW + (size_t)dx) * CAR_RASTER_BPP;
            dst[d + 0] = src[s + 0];
            dst[d + 1] = src[s + 1];
            dst[d + 2] = src[s + 2];
            dst[d + 3] = src[s + 3];
        }
    }
    return true;
}

float car_raster_difference(const unsigned char *labelsA, const unsigned char *labelsB,
                            int width, int height)
{
    if (labelsA == NULL || labelsB == NULL || width <= 0 || height <= 0) return 0.0f;

    const size_t count = (size_t)width * (size_t)height;
    size_t unionPx = 0, differing = 0;
    for (size_t i = 0; i < count; i++) {
        const unsigned char a = labelsA[i];
        const unsigned char b = labelsB[i];
        if (a == CAR_LABEL_EMPTY && b == CAR_LABEL_EMPTY) continue;
        /* Shadow pixels (CAR_LABEL_EMPTY) are excluded from distinctness by design:
         * the label map write skips them because shadow alpha < 128. */
        unionPx++;
        if (a != b) differing++;
    }
    if (unionPx == 0) return 0.0f;
    return (float)differing / (float)unionPx;
}