/*
 * collision.c — vehicle-track capsule collision with swept substeps and impulse response.
 *
 * Physics conventions:
 *   - SI units throughout.
 *   - Body X forward, body Y left.
 *   - Heading counterclockwise from world +X.
 *   - Yaw rate counterclockwise positive.
 *   - Impulses are computed in world frame, then rotated back to body frame.
 *
 * Sign convention for each equation is documented inline.
 *
 * This translation unit calls no raylib function.
 */

#include "world/collision.h"

#include <math.h>
#include <string.h>

/* ====================================================================================== */
/*  Boundary polyline helpers                                                              */
/* ====================================================================================== */

/* Closest point on segment a→b to point p, and the squared distance to it.
 * Robust for zero-length segments. */
static Vector2 closest_point_and_dist_sq(Vector2 p, Vector2 a, Vector2 b, float *distSqOut)
{
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float lenSq = dx * dx + dy * dy;
    float t;
    if (lenSq < 1e-12f) {
        t = 0.0f;
    } else {
        t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    const Vector2 closest = { a.x + t * dx, a.y + t * dy };
    const float ex = p.x - closest.x;
    const float ey = p.y - closest.y;
    *distSqOut = ex * ex + ey * ey;
    return closest;
}

/* ====================================================================================== */
/*  Capsule circle positions                                                               */
/* ====================================================================================== */

/* Body-frame position of the front capsule circle. */
static Vector2 body_front_position(float cgToFrontM)
{
    return (Vector2){ cgToFrontM, 0.0f };
}

/* Body-frame position of the rear capsule circle. */
static Vector2 body_rear_position(float cgToRearM)
{
    return (Vector2){ -cgToRearM, 0.0f };
}

/* Rotate a body-frame point into world frame given the CG position and heading. */
static Vector2 world_from_body(Vector2 bodyPoint, Vector2 cgWorld, float headingRad)
{
    const float cosH = cosf(headingRad);
    const float sinH = sinf(headingRad);
    return (Vector2){ cgWorld.x + bodyPoint.x * cosH - bodyPoint.y * sinH,
                      cgWorld.y + bodyPoint.x * sinH + bodyPoint.y * cosH };
}

/* ====================================================================================== */
/*  Velocity helpers                                                                       */
/* ====================================================================================== */

/* World-frame CG velocity from body-frame components. */
static Vector2 world_velocity(float vxBody, float vyBody, float headingRad)
{
    const float cosH = cosf(headingRad);
    const float sinH = sinf(headingRad);
    return (Vector2){ vxBody * cosH - vyBody * sinH, vxBody * sinH + vyBody * cosH };
}

/* World-frame contact-point velocity: v_CG + ω × r.  In 2D, ω × r = ω * (-r.y, r.x). */
static Vector2 contact_velocity_world(Vector2 vCgWorld, float yawRateRadS, Vector2 r)
{
    return (Vector2){ vCgWorld.x - yawRateRadS * r.y, vCgWorld.y + yawRateRadS * r.x };
}

/* World → body rotation of a velocity vector. */
/* Body X = dot(v_world, (cosH, sinH))   (forward)
 * Body Y = dot(v_world, (-sinH, cosH))  (left)
 */
static void world_vel_to_body(Vector2 vWorld, float headingRad, float *vxBodyOut,
                              float *vyBodyOut)
{
    *vxBodyOut = vWorld.x * cosf(headingRad) + vWorld.y * sinf(headingRad);
    *vyBodyOut = -vWorld.x * sinf(headingRad) + vWorld.y * cosf(headingRad);
}

/* ====================================================================================== */
/*  Interpolation helpers for swept test                                                   */
/* ====================================================================================== */

static float lerp_angle_simple(float a, float b, float t)
{
    /* Shortest-path interpolation. */
    float delta = b - a;
    /* Wrap delta to [-PI, PI). */
    if (delta > 3.14159265358979323846f) delta -= 2.0f * 3.14159265358979323846f;
    if (delta < -3.14159265358979323846f) delta += 2.0f * 3.14159265358979323846f;
    return a + delta * t;
}

static Vector2 lerp_vec(Vector2 a, Vector2 b, float t)
{
    return (Vector2){ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
}
/* ====================================================================================== */
/*  Contact resolution helper                                                              */
/* ====================================================================================== */

/* Resolves one circle-vs-barrier contact: penetration correction + impulse response.
 *
 * Penetration push moves the CG so the circle just touches the barrier. If the contact is
 * approaching (vn < 0), a normal impulse reflects velocity with restitution and a tangential
 * Coulomb-clamped friction impulse is applied; both update CG linear velocity and yaw rate.
 * *vCgWorld is updated to the post-impulse world-frame CG velocity so a subsequent contact
 * in the same substep sees the corrected velocity (no stale-data trap).
 *
 * Returns true if a contact was resolved (penetration corrected; impulse applied when
 * approaching). Returns false if there is no penetration or the contact is degenerate. */
static bool resolve_circle_barrier(const VehicleSpec *spec, VehicleState *state,
                                   VehicleRenderState *renderState, Vector2 pos, float hdg,
                                   Vector2 contactPt, float distSq, Vector2 pushN,
                                   float radiusM, float rHalf, float muC, Vector2 *vCgWorld,
                                   float *crashLockoutTimerS)
{
    const float dist = sqrtf(distSq);
    const float pen = radiusM - dist;
    if (!(pen > 0.0f) || !(dist > 1e-9f)) return false;

    /* Penetration correction: push the CG along the push normal. */
    state->positionM.x = pos.x + pushN.x * pen;
    state->positionM.y = pos.y + pushN.y * pen;
    state->headingRad = hdg;
    renderState->currPositionM = state->positionM;
    renderState->currHeadingRad = state->headingRad;

    /* Contact-point velocity: v_CG + ω × r. */
    const Vector2 rContact = { contactPt.x - pos.x, contactPt.y - pos.y };
    const Vector2 vContact = contact_velocity_world(*vCgWorld, state->yawRateRadS, rContact);

    /* Normal velocity (positive = separating, negative = approaching). */
    const float vn = vContact.x * pushN.x + vContact.y * pushN.y;
    if (vn >= 0.0f) return true; /* separating: push only, no impulse needed */

    /* Tangential direction: rotate normal +90°. */
    const Vector2 tang = { -pushN.y, pushN.x };
    const float vt = vContact.x * tang.x + vContact.y * tang.y;

    /* Effective mass for normal and tangential impulse. */
    const float rXn = rContact.x * pushN.y - rContact.y * pushN.x;
    const float rXt = rContact.x * tang.y - rContact.y * tang.x;
    const float invMass = 1.0f / spec->massKg;
    const float invInertia = 1.0f / spec->yawInertiaKgM2;
    const float effMassN = 1.0f / (invMass + rXn * rXn * invInertia);
    const float effMassT = 1.0f / (invMass + rXt * rXt * invInertia);

    /* Normal impulse: reflect with restitution (Δv_n = -(1+e)·v_n). */
    const float Jn = effMassN * (-(1.0f + rHalf) * vn);

    /* Friction impulse, Coulomb-clamped to |Jt| ≤ μ·Jn. */
    float Jt = -effMassT * vt;
    const float JtMax = muC * Jn;
    if (Jt > JtMax) Jt = JtMax;
    if (Jt < -JtMax) Jt = -JtMax;

    /* Total impulse in world frame, applied to CG linear velocity. */
    const Vector2 J = { Jn * pushN.x + Jt * tang.x, Jn * pushN.y + Jt * tang.y };
    const Vector2 vCgNew = { vCgWorld->x + J.x * invMass, vCgWorld->y + J.y * invMass };
    world_vel_to_body(vCgNew, state->headingRad, &state->velocityLongitudinalMps,
                      &state->velocityLateralMps);

    /* Angular impulse: r × J. */
    const float rXJ = rContact.x * J.y - rContact.y * J.x;
    state->yawRateRadS += rXJ * invInertia;

    /* Update the CG velocity for any subsequent contact in this substep. */
    *vCgWorld = vCgNew;

    /* Crash lockout on significant impact. */
    if (crashLockoutTimerS != NULL && fabsf(vn) > COLLISION_LOCKOUT_THRESHOLD_MPS) {
        *crashLockoutTimerS = CRASH_LOCKOUT_S;
    }
    return true;
}

/* ====================================================================================== */
/*  Collision resolution — the main function                                               */
/* ====================================================================================== */

int collision_resolve_track(const VehicleSpec *spec, VehicleState *state,
                            VehicleRenderState *renderState, const Track *track,
                            float *crashLockoutTimerS)
{
    if (spec == NULL || state == NULL || renderState == NULL || track == NULL ||
        track->nodes == NULL || track->count < 2) {
        return 0;
    }

    const float radiusM = spec->bodyHalfWidthM;
    const float radiusSq = radiusM * radiusM;
    if (radiusM <= 0.0f) return 0;

    const float rHalf = spec->collisionRestitution;
    const float muC = spec->collisionFriction;

    /* Capsule circle body-frame positions. */
    const Vector2 bFront = body_front_position(spec->cgToFrontM);
    const Vector2 bRear = body_rear_position(spec->cgToRearM);

    /* Start-of-tick and end-of-tick transforms. */
    const Vector2 prevPos = renderState->prevPositionM;
    const Vector2 currPos = renderState->currPositionM;
    const float prevHdg = renderState->prevHeadingRad;
    const float currHdg = renderState->currHeadingRad;

    /* World-frame CG velocity at start of tick. Mutable: each resolved contact updates it
     * so a subsequent contact in the same substep uses the corrected velocity. */
    Vector2 vCgWorld =
        world_velocity(state->velocityLongitudinalMps, state->velocityLateralMps, currHdg);

    /* Substep sweep: small increments from prev→curr catch the earliest contact.
     * At the FIRST substep that finds penetration, ALL currently-penetrating circle/barrier
     * pairs are resolved in deterministic order (front-then-rear, left-then-right, per
     * segment), then the function returns the total contact count. We do NOT continue to
     * later substeps: once state is mutated the prev→curr interpolation is stale. */
    for (int sub = 0; sub < COLLISION_SUBSTEPS; sub++) {
        const float t = (float)sub / (float)COLLISION_SUBSTEPS;
        Vector2 pos = lerp_vec(prevPos, currPos, t);
        const float hdg = lerp_angle_simple(prevHdg, currHdg, t);
        int contacts = 0;

        const int n = track->count;
        for (int i = 0; i < n; i++) {
            const int j = (i + 1) % n;
            const TrackNode *ni = &track->nodes[i];
            const TrackNode *nj = &track->nodes[j];

            /* Segment direction and left perpendicular. */
            const float segDx = nj->centerM.x - ni->centerM.x;
            const float segDy = nj->centerM.y - ni->centerM.y;
            const float segLen = sqrtf(segDx * segDx + segDy * segDy);
            if (segLen < 1e-12f) continue;
            const float invLen = 1.0f / segLen;
            const Vector2 dir = { segDx * invLen, segDy * invLen };
            const Vector2 perp = { -dir.y, dir.x };

            /* Barriers stand at the RUNOFF edge, not the racing-surface edge, so a car can
             * run wide onto the runoff and lose grip without instantly striking a wall. A
             * node with no runoff band reports its racing half-width here, which is exactly
             * where its barrier used to be. */
            const float hwI = track_node_barrier_half_width(ni);
            const float hwJ = track_node_barrier_half_width(nj);
            const Vector2 barriers[2][2] = {
                { /* left */ { ni->centerM.x + perp.x * hwI, ni->centerM.y + perp.y * hwI },
                  { nj->centerM.x + perp.x * hwJ, nj->centerM.y + perp.y * hwJ } },
                { /* right */ { ni->centerM.x - perp.x * hwI, ni->centerM.y - perp.y * hwI },
                  { nj->centerM.x - perp.x * hwJ, nj->centerM.y - perp.y * hwJ } },
            };
            /* Push normal: for ribbon tracks, left barrier pushes right ({dir.y, -dir.x}),
             * right barrier pushes left ({-dir.y, dir.x}).
             * For parking lot perimeter, BOTH barriers push INWARD (perp). */
            const Vector2 pushNs[2] = { track->isParkingLot ? perp : (Vector2){ dir.y, -dir.x },
                                        track->isParkingLot ? perp
                                                            : (Vector2){ -dir.y, dir.x } };

            for (int barrier = 0; barrier < 2; barrier++) {
                const Vector2 bA = barriers[barrier][0];
                const Vector2 bB = barriers[barrier][1];
                const Vector2 pushN = pushNs[barrier];

                for (int circle = 0; circle < 2; circle++) {
                    const Vector2 bodyPt = (circle == 0) ? bFront : bRear;
                    const Vector2 circleWorld = world_from_body(bodyPt, pos, hdg);

                    float distSq;
                    const Vector2 contactPt =
                        closest_point_and_dist_sq(circleWorld, bA, bB, &distSq);

                    if (distSq < radiusSq) {
                        if (resolve_circle_barrier(spec, state, renderState, pos, hdg,
                                                   contactPt, distSq, pushN, radiusM, rHalf,
                                                   muC, &vCgWorld, crashLockoutTimerS)) {
                            contacts++;
                            pos = state->positionM;
                        }
                    }
                }
            }
        }

        if (contacts > 0) return contacts;
    }

    return 0;
}
