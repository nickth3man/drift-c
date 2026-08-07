/*
 * render_world.c — everything the camera sees: the track, the tire smoke, and the debug
 * vector overlay. Compiled to an empty translation unit under DRIFTY_HEADLESS.
 */
#if !defined(DRIFTY_HEADLESS)

#include "render/render_internal.h"

#include <math.h>
#include <string.h>

#include "core/math_utils.h"
#include "core/units.h"

static Vector2 body_point_to_world(Vector2 bodyPointM, Vector2 positionM, float headingRad)
{
    const float c = cosf(headingRad);
    const float s = sinf(headingRad);
    return (Vector2){ positionM.x + bodyPointM.x * c - bodyPointM.y * s,
                      positionM.y + bodyPointM.x * s + bodyPointM.y * c };
}

static Vector2 body_vector_to_world(Vector2 bodyVector, float headingRad)
{
    const float c = cosf(headingRad);
    const float s = sinf(headingRad);
    return (Vector2){ bodyVector.x * c - bodyVector.y * s,
                      bodyVector.x * s + bodyVector.y * c };
}

static void draw_world_vector(Vector2 startM, Vector2 vectorM, float ppm, Color color,
                              const char *label)
{
    if (vectorM.x * vectorM.x + vectorM.y * vectorM.y < 0.0004f) return;
    const Vector2 endM = { startM.x + vectorM.x, startM.y + vectorM.y };
    const Vector2 startPx = units_world_to_render_px(startM, ppm);
    const Vector2 endPx = units_world_to_render_px(endM, ppm);
    DrawLineEx(startPx, endPx, 2.0f, color);
    DrawCircleV(endPx, 3.0f, color);
    if (label != NULL && label[0] != '\0') {
        DrawText(label, (int)endPx.x + 4, (int)endPx.y - 8, 12, color);
    }
}

/* ---- track rendering --------------------------------------------------------------- */

void render_world_draw_track(const Track *track, float ppm)
{
    if (track == NULL || track->nodes == NULL || track->count < 2) return;

    /* Parking lot mode: wide open rectangular area with parking-space line grid. */
    if (track->isParkingLot) {
        const float lotL = track->lotMinXM;
        const float lotR = track->lotMaxXM;
        const float lotB = track->lotMinYM;
        const float lotT = track->lotMaxYM;

        /* ---- Grass surround (behind everything) ---- */
        {
            const float marginM = 60.0f;
            const Vector2 blPx =
                units_world_to_render_px((Vector2){ lotL - marginM, lotB - marginM }, ppm);
            const Vector2 trPx =
                units_world_to_render_px((Vector2){ lotR + marginM, lotT + marginM }, ppm);
            DrawRectangle((int)blPx.x, (int)trPx.y, (int)(trPx.x - blPx.x),
                          (int)(blPx.y - trPx.y), (Color){ 76, 117, 67, 255 });
        }

        /* ---- Asphalt lot ---- */
        {
            const Vector2 blPx = units_world_to_render_px((Vector2){ lotL, lotB }, ppm);
            const Vector2 trPx = units_world_to_render_px((Vector2){ lotR, lotT }, ppm);
            DrawRectangle((int)blPx.x, (int)trPx.y, (int)(trPx.x - blPx.x),
                          (int)(blPx.y - trPx.y), (Color){ 45, 45, 50, 255 });
        }

        /* ---- Parking-space lines (vertical & horizontal grid) ---- */
        {
            const float spacingM = 6.0f;                   /* distance between line centres */
            const float lineLenM = 5.0f;                   /* length of each painted line */
            const float lineW = 3.0f;                      /* pixel thickness */
            const Color colWhite = { 210, 210, 215, 200 }; /* semi-transparent white */

            /* Vertical parking strips: rows of short east-west lines at regular Y intervals */
            for (float yy = lotB + 3.0f; yy <= lotT - 3.0f; yy += spacingM) {
                for (float xx = lotL + 3.0f; xx <= lotR - 3.0f; xx += spacingM) {
                    const Vector2 aPx =
                        units_world_to_render_px((Vector2){ xx - lineLenM * 0.5f, yy }, ppm);
                    const Vector2 bPx =
                        units_world_to_render_px((Vector2){ xx + lineLenM * 0.5f, yy }, ppm);
                    DrawLineEx(aPx, bPx, lineW, colWhite);
                }
            }

            /* Perimeter double-line (curb) */
            const Color colCurb = { 180, 180, 185, 230 };
            const float curbThick = 4.0f;
            {
                const Vector2 bl = units_world_to_render_px((Vector2){ lotL, lotB }, ppm);
                const Vector2 br = units_world_to_render_px((Vector2){ lotR, lotB }, ppm);
                const Vector2 tr = units_world_to_render_px((Vector2){ lotR, lotT }, ppm);
                const Vector2 tl = units_world_to_render_px((Vector2){ lotL, lotT }, ppm);
                DrawLineEx(bl, br, curbThick, colCurb);
                DrawLineEx(br, tr, curbThick, colCurb);
                DrawLineEx(tr, tl, curbThick, colCurb);
                DrawLineEx(tl, bl, curbThick, colCurb);
            }

            /* Inner curb offset line (2m inset) */
            const float inset = 2.0f;
            const Color colInnerCurb = { 140, 140, 145, 120 };
            {
                const Vector2 bl =
                    units_world_to_render_px((Vector2){ lotL + inset, lotB + inset }, ppm);
                const Vector2 br =
                    units_world_to_render_px((Vector2){ lotR - inset, lotB + inset }, ppm);
                const Vector2 tr =
                    units_world_to_render_px((Vector2){ lotR - inset, lotT - inset }, ppm);
                const Vector2 tl =
                    units_world_to_render_px((Vector2){ lotL + inset, lotT - inset }, ppm);
                DrawLineEx(bl, br, 2.0f, colInnerCurb);
                DrawLineEx(br, tr, 2.0f, colInnerCurb);
                DrawLineEx(tr, tl, 2.0f, colInnerCurb);
                DrawLineEx(tl, bl, 2.0f, colInnerCurb);
            }
        }

        return;
    }

    /* ---- Original stadium oval rendering (below) ---- */

    const int n = track->count;

    /* --- grass surround (drawn first, behind everything) --- */
    {
        float minXM = track->nodes[0].centerM.x;
        float maxXM = minXM;
        float minYM = track->nodes[0].centerM.y;
        float maxYM = minYM;
        for (int i = 1; i < n; i++) {
            const Vector2 c = track->nodes[i].centerM;
            if (c.x < minXM) minXM = c.x;
            if (c.x > maxXM) maxXM = c.x;
            if (c.y < minYM) minYM = c.y;
            if (c.y > maxYM) maxYM = c.y;
        }
        const float marginM = 60.0f;
        minXM -= marginM;
        maxXM += marginM;
        minYM -= marginM;
        maxYM += marginM;

        /* Convert corners.  Y is negated by units_world_to_render_px, so the
         * bottom-left world corner produces the largest render Y. */
        const Vector2 blPx = units_world_to_render_px((Vector2){ minXM, minYM }, ppm);
        const Vector2 trPx = units_world_to_render_px((Vector2){ maxXM, maxYM }, ppm);
        DrawRectangle((int)blPx.x, (int)trPx.y, (int)(trPx.x - blPx.x), (int)(blPx.y - trPx.y),
                      (Color){ 76, 117, 67, 255 });
    }

    /* --- asphalt ribbon (thick filled centreline) ---
     * Width is per segment: the chicane is deliberately narrower than the straights, and a
     * single sampled half-width would draw it at the wrong size. */
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        const float segHalfWidthM =
            0.5f * (track->nodes[i].halfWidthM + track->nodes[j].halfWidthM);
        const Vector2 aPx = units_world_to_render_px(track->nodes[i].centerM, ppm);
        const Vector2 bPx = units_world_to_render_px(track->nodes[j].centerM, ppm);
        DrawLineEx(aPx, bPx, 2.0f * segHalfWidthM * ppm, (Color){ 40, 40, 45, 255 });
    }

    /* --- track limits and barriers (offset polylines) ---
     * Two distinct edges now exist and the difference matters to a driver: the white line is
     * where the racing surface ends and grip falls away, the red line is where the wall is.
     * A node with no runoff band draws them on top of each other, which is the truth. */
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        const Vector2 a = track->nodes[i].centerM;
        const Vector2 b = track->nodes[j].centerM;

        const float dx = b.x - a.x;
        const float dy = b.y - a.y;
        const float len = sqrtf(dx * dx + dy * dy);
        if (len < 1e-6f) continue;

        const float invLen = 1.0f / len;
        /* Perpendicular in world space: (-uy, ux) = left side of forward direction. */
        const float ux = dx * invLen;
        const float uy = dy * invLen;

        const float edgeHalfWidthM =
            0.5f * (track->nodes[i].halfWidthM + track->nodes[j].halfWidthM);
        const float barrierHalfWidthM =
            0.5f * (track_node_barrier_half_width(&track->nodes[i]) +
                    track_node_barrier_half_width(&track->nodes[j]));

        const struct {
            float offsetM;
            float thicknessPx;
            Color color;
        } edges[2] = {
            { edgeHalfWidthM, 2.5f, (Color){ 220, 220, 225, 255 } },  /* track limit */
            { barrierHalfWidthM, 3.0f, (Color){ 190, 70, 70, 255 } }, /* barrier */
        };

        for (int e = 0; e < 2; e++) {
            const float perpX = -uy * edges[e].offsetM;
            const float perpY = ux * edges[e].offsetM;
            const Vector2 leftA =
                units_world_to_render_px((Vector2){ a.x + perpX, a.y + perpY }, ppm);
            const Vector2 leftB =
                units_world_to_render_px((Vector2){ b.x + perpX, b.y + perpY }, ppm);
            const Vector2 rightA =
                units_world_to_render_px((Vector2){ a.x - perpX, a.y - perpY }, ppm);
            const Vector2 rightB =
                units_world_to_render_px((Vector2){ b.x - perpX, b.y - perpY }, ppm);
            DrawLineEx(leftA, leftB, edges[e].thicknessPx, edges[e].color);
            DrawLineEx(rightA, rightB, edges[e].thicknessPx, edges[e].color);
        }
    }

    /* --- checkpoint gates --- * The next required gate is highlighted, the rest are dim, so a
     * recording shows at a glance where the car was supposed to go next. */
    for (int i = 0; i < track->checkpointCount; i++) {
        const Checkpoint *c = &track->checkpoints[i];
        const Vector2 perp = { -c->forwardUnit.y * c->halfWidthM,
                               c->forwardUnit.x * c->halfWidthM };
        const Vector2 aPx = units_world_to_render_px(
            (Vector2){ c->centerM.x + perp.x, c->centerM.y + perp.y }, ppm);
        const Vector2 bPx = units_world_to_render_px(
            (Vector2){ c->centerM.x - perp.x, c->centerM.y - perp.y }, ppm);
        const bool isNext = (i == track->nextCheckpoint);
        const bool isStartFinish = (i == 0);
        Color color = isNext ? (Color){ 250, 210, 70, 220 } : (Color){ 120, 160, 200, 110 };
        if (isStartFinish && !isNext) color = (Color){ 230, 230, 235, 160 };
        DrawLineEx(aPx, bPx, isNext ? 3.0f : 2.0f, color);
    }

    /* --- centreline hint (faint guide) --- */
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        const Vector2 aPx = units_world_to_render_px(track->nodes[i].centerM, ppm);
        const Vector2 bPx = units_world_to_render_px(track->nodes[j].centerM, ppm);
        DrawLineEx(aPx, bPx, 1.5f, (Color){ 180, 180, 190, 80 });
    }
}

void render_world_draw_debug(const Game *game, const VehicleDrawState *draw)
{
    const float ppm = game->renderPixelsPerMeter;
    const float heading = draw->headingRad;
    draw_world_vector(draw->positionM, body_vector_to_world((Vector2){ 2.0f, 0.0f }, heading),
                      ppm, YELLOW, "");
    draw_world_vector(draw->positionM, body_vector_to_world((Vector2){ 0.0f, 1.5f }, heading),
                      ppm, SKYBLUE, "");
    const Vector2 bodyVelocity = { game->vehicle.velocityLongitudinalMps * 0.30f,
                                   game->vehicle.velocityLateralMps * 0.30f };
    draw_world_vector(draw->positionM, body_vector_to_world(bodyVelocity, heading), ppm, GREEN,
                      "velocity");

    const Vector2 frontM =
        body_point_to_world((Vector2){ game->spec.cgToFrontM, 0.0f }, draw->positionM, heading);
    const Vector2 rearM =
        body_point_to_world((Vector2){ -game->spec.cgToRearM, 0.0f }, draw->positionM, heading);
    draw_world_vector(frontM,
                      body_vector_to_world(
                          (Vector2){ game->derived.frontAxleContactVelocityBodyMps.x * 0.20f,
                                     game->derived.frontAxleContactVelocityBodyMps.y * 0.20f },
                          heading),
                      ppm, ORANGE, "front v");
    draw_world_vector(rearM,
                      body_vector_to_world(
                          (Vector2){ game->derived.rearAxleContactVelocityBodyMps.x * 0.20f,
                                     game->derived.rearAxleContactVelocityBodyMps.y * 0.20f },
                          heading),
                      ppm, PURPLE, "rear v");
    draw_world_vector(
        frontM,
        body_vector_to_world((Vector2){ cosf(draw->wheelAngleRad[WHEEL_FRONT_LEFT]) * 1.4f,
                                        sinf(draw->wheelAngleRad[WHEEL_FRONT_LEFT]) * 1.4f },
                             heading),
        ppm, GOLD, "front heading");
    draw_world_vector(
        frontM,
        body_vector_to_world((Vector2){ game->derived.frontBodyForceN.x * 0.00008f,
                                        game->derived.frontBodyForceN.y * 0.00008f },
                             heading),
        ppm, RED, "front force");
    draw_world_vector(
        rearM,
        body_vector_to_world((Vector2){ game->derived.rearBodyForceN.x * 0.00008f,
                                        game->derived.rearBodyForceN.y * 0.00008f },
                             heading),
        ppm, MAROON, "rear force");
}

/* ---- tire smoke ------------------------------------------------------------------
 * Each active particle renders as a soft puff: three overlapping translucent circles
 * whose offsets come from a deterministic per-slot wobble (no per-frame randomness).
 * Colour drifts from a dark rubber-grey when fresh at the tire to near-white as it
 * disperses, and alpha falls off quadratically with age so the trail dissolves softly.
 * Cost: 3 DrawCircleV per active particle, pool cap 512 — comfortably cheap.
 */
void render_world_draw_particles(const Game *game)
{
    const float ppm = game->renderPixelsPerMeter;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &game->particles.particles[i];
        if (!p->active) continue;

        const float maxLifeS = (p->maxLifeS > 0.0f) ? p->maxLifeS : 1.0f;
        const float t = clampf(1.0f - p->lifeS / maxLifeS, 0.0f, 1.0f); /* 0 fresh -> 1 gone */

        const unsigned char cr = (unsigned char)lerpf(104.0f, 236.0f, t);
        const unsigned char cg = (unsigned char)lerpf(96.0f, 236.0f, t);
        const unsigned char cb = (unsigned char)lerpf(88.0f, 240.0f, t);
        const float fade = (1.0f - t) * (1.0f - t);
        const float baseA = (float)p->color.a * fade;

        const Vector2 px = units_world_to_render_px(p->positionM, ppm);
        const float radiusPx = p->sizeM * ppm * 0.5f * (1.0f + t * 1.1f);

        /* Two fixed wobble directions per pool slot, slowly swirling as the puff ages. */
        const float wobA =
            (float)(((unsigned)i * 2654435761u) >> 16 & 0xFF) / 255.0f * 6.2831853f + t * 1.7f;
        const float wobB = wobA + 2.4f;
        const Vector2 offA = { cosf(wobA) * radiusPx * 0.45f, sinf(wobA) * radiusPx * 0.45f };
        const Vector2 offB = { cosf(wobB) * radiusPx * 0.50f, sinf(wobB) * radiusPx * 0.50f };

        /* Outer puffs first (larger, fainter), core last. */
        DrawCircleV((Vector2){ px.x + offA.x, px.y + offA.y }, radiusPx * 1.35f,
                    (Color){ cr, cg, cb, (unsigned char)(baseA * 0.36f) });
        DrawCircleV((Vector2){ px.x + offB.x, px.y + offB.y }, radiusPx * 1.05f,
                    (Color){ cr, cg, cb, (unsigned char)(baseA * 0.28f) });
        DrawCircleV(px, radiusPx, (Color){ cr, cg, cb, (unsigned char)(baseA * 0.85f) });
    }
}

#endif /* !DRIFTY_HEADLESS */
