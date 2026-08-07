/*
 * ai_driver.c — pure pursuit steering, curvature-limited speed target.
 *
 * Two independent controllers share one geometric query of the centreline:
 *
 *   STEERING follows the classic pure-pursuit law. A point is picked on the centreline a
 *   speed-dependent distance ahead, and the steer angle is the one that would put the car on
 *   a circular arc through it. A derivative term on cross-track error damps the weave that
 *   pure pursuit alone develops when the lookahead is short relative to the wheelbase.
 *
 *   SPEED comes from the tightest curvature within braking range. Each upcoming point on the
 *   centreline implies a cornering speed from the tyre's own lateral grip; back-propagating
 *   each of those through a constant-deceleration braking distance gives the fastest speed
 *   that is still stoppable in time for it. The minimum over the horizon is the target.
 *
 * Both read grip from the spec and the surface table, which is data the car publishes about
 * itself. Nothing here consults a force, a slip angle, or any other quantity a driver could
 * not feel, and nothing here writes anywhere but the Input.
 */
#include "game/ai_driver.h"

#include <math.h>
#include <stddef.h>

#include "core/config.h"
#include "core/math_utils.h"
#include "physics/surface.h"

/* Curvature below this is a straight; the reciprocal would otherwise overflow the target. */
#define AI_MIN_CURVATURE_1PM 1.0e-4f

/* How far past the pure braking distance the speed scan looks. Braking distance alone would
 * make the driver notice a corner exactly when it is already too late to slow for it, because
 * the scan and the braking both start at the same point. */
#define AI_SPEED_SCAN_MARGIN_M 25.0f

void ai_driver_config_default(AiDriverConfig *cfg)
{
    if (cfg == NULL) return;
    cfg->lookaheadBaseM = 6.0f;
    cfg->lookaheadSpeedS = 0.45f;
    cfg->corneringGripFraction = 0.80f;
    cfg->brakeGripFraction = 0.80f;
    cfg->steerGainP = 1.0f;
    cfg->steerGainD = 0.08f;
    cfg->speedGainP = 0.35f;
    cfg->speedDeadbandMps = 0.30f;
    cfg->maxSpeedMps = 80.0f;
}

/* ------------------------------------------------------------------------------------- */
/* Centreline geometry                                                                     */
/* ------------------------------------------------------------------------------------- */

static Vector2 vec_sub(Vector2 a, Vector2 b)
{
    return (Vector2){ a.x - b.x, a.y - b.y };
}

static float vec_len(Vector2 v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}

/*
 * Closest point on segment [a,b] to p. Writes the clamped parameter to *tOut and returns the
 * distance. A degenerate segment reports t = 0, which is the correct answer for a zero-length
 * segment and keeps the caller free of a special case.
 */
static float closest_on_segment(Vector2 a, Vector2 b, Vector2 p, float *tOut, Vector2 *pointOut)
{
    const Vector2 ab = vec_sub(b, a);
    const float lenSq = ab.x * ab.x + ab.y * ab.y;
    float t = 0.0f;
    if (lenSq > 1.0e-9f) {
        t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / lenSq;
        t = clampf(t, 0.0f, 1.0f);
    }
    const Vector2 point = { a.x + ab.x * t, a.y + ab.y * t };
    if (tOut != NULL) *tOut = t;
    if (pointOut != NULL) *pointOut = point;
    return vec_len(vec_sub(p, point));
}

/*
 * Menger curvature at node i, from its immediate neighbours: kappa = 2|AB x AC| / (|AB||BC||CA|).
 *
 * Exact for points sampled off a circle at any spacing, which is what the constant-radius
 * curves are, and it degrades gracefully to zero on a straight rather than dividing by a
 * near-zero radius. The whole track is a closed loop, so the neighbours always exist.
 */
static float node_curvature(const TrackNode *nodes, int count, int i)
{
    if (count < 3) return 0.0f;
    const Vector2 a = nodes[(i - 1 + count) % count].centerM;
    const Vector2 b = nodes[i].centerM;
    const Vector2 c = nodes[(i + 1) % count].centerM;

    const Vector2 ab = vec_sub(b, a);
    const Vector2 bc = vec_sub(c, b);
    const Vector2 ca = vec_sub(a, c);
    const float lab = vec_len(ab), lbc = vec_len(bc), lca = vec_len(ca);
    const float denom = lab * lbc * lca;
    if (denom < 1.0e-6f) return 0.0f;

    const float cross = ab.x * bc.y - ab.y * bc.x;
    return 2.0f * fabsf(cross) / denom;
}

/*
 * Walk `distanceM` forward along the centreline from (segment, t) and return where that lands.
 *
 * The walk is capped at one lap so a lookahead longer than the circuit cannot spin forever.
 */
static Vector2 point_ahead(const TrackNode *nodes, int count, int segment, float t,
                           float distanceM)
{
    Vector2 from = {
        nodes[segment].centerM.x +
            (nodes[(segment + 1) % count].centerM.x - nodes[segment].centerM.x) * t,
        nodes[segment].centerM.y +
            (nodes[(segment + 1) % count].centerM.y - nodes[segment].centerM.y) * t
    };
    float remainingM = distanceM;

    for (int step = 0; step < count; step++) {
        const int next = (segment + 1 + step) % count;
        const Vector2 target = nodes[next].centerM;
        const Vector2 leg = vec_sub(target, from);
        const float legM = vec_len(leg);
        if (legM >= remainingM) {
            if (legM < 1.0e-6f) return target;
            const float f = remainingM / legM;
            return (Vector2){ from.x + leg.x * f, from.y + leg.y * f };
        }
        remainingM -= legM;
        from = target;
    }
    return from;
}

/* ------------------------------------------------------------------------------------- */
/* Grip the car publishes about itself                                                     */
/* ------------------------------------------------------------------------------------- */

/*
 * The lateral and longitudinal friction coefficients a driver could reasonably expect, given
 * the tyres fitted and the surface under the car. Composed exactly the way physics.c composes
 * them, so the driver's expectation and the car's behaviour cannot silently diverge.
 */
static void available_grip(const VehicleSpec *spec, const Track *track, Vector2 positionM,
                           float *muLatOut, float *muLongOut)
{
    const SurfaceId id =
        (track != NULL) ? Track_SurfaceAt(track, positionM) : (SurfaceId)SURFACE_ASPHALT;
    const SurfaceSpec *surface = Surface_Get(id);

    const float tyreMuLat = minf(spec->tireMuLatFront, spec->tireMuLatRear);
    *muLatOut = tyreMuLat * (surface->muLateral / SURFACE_REFERENCE_MU_LAT);
    *muLongOut = spec->tireMuLongScale * surface->muLongitudinal;
}

/* ------------------------------------------------------------------------------------- */

void ai_driver_update(const AiDriverConfig *cfg, AiDriverState *state, const Track *track,
                      const VehicleState *vehicle, const VehicleDerived *derived,
                      const VehicleSpec *spec, Input *out, float dt)
{
    if (cfg == NULL || state == NULL || out == NULL) return;
    if (track == NULL || track->nodes == NULL || track->count < 3) return;
    if (vehicle == NULL || derived == NULL || spec == NULL) return;

    const TrackNode *nodes = track->nodes;
    const int count = track->count;
    const Vector2 posM = vehicle->positionM;
    const float speedMps = derived->speedMps;

    /* --- Where the car is on the centreline ---
     * A full scan rather than a search around last tick's answer: 170 segments is nothing at
     * 120 Hz, and it cannot lose the line after a spin or a barrier shove, which a local
     * search can. */
    int bestSegment = 0;
    float bestT = 0.0f;
    Vector2 bestPoint = nodes[0].centerM;
    float bestDistM = INFINITY;
    for (int i = 0; i < count; i++) {
        float t = 0.0f;
        Vector2 point = { 0.0f, 0.0f };
        const float d = closest_on_segment(nodes[i].centerM, nodes[(i + 1) % count].centerM,
                                           posM, &t, &point);
        if (d < bestDistM) {
            bestDistM = d;
            bestSegment = i;
            bestT = t;
            bestPoint = point;
        }
    }

    /* Signed cross-track error, positive when the car is LEFT of the centreline. Body Y is
     * left, so the left normal of a forward direction (fx,fy) is (-fy,fx). */
    const Vector2 segDir =
        vec_sub(nodes[(bestSegment + 1) % count].centerM, nodes[bestSegment].centerM);
    const float segLen = vec_len(segDir);
    float crossTrackM = 0.0f;
    if (segLen > 1.0e-6f) {
        const Vector2 leftUnit = { -segDir.y / segLen, segDir.x / segLen };
        const Vector2 toCar = vec_sub(posM, bestPoint);
        crossTrackM = toCar.x * leftUnit.x + toCar.y * leftUnit.y;
    }

    /* --- Steering --- */
    const float lookaheadM =
        maxf(cfg->lookaheadBaseM + cfg->lookaheadSpeedS * maxf(speedMps, 0.0f), 1.0f);
    const Vector2 targetM = point_ahead(nodes, count, bestSegment, bestT, lookaheadM);

    const Vector2 toTarget = vec_sub(targetM, posM);
    const float cosH = cosf(vehicle->headingRad), sinH = sinf(vehicle->headingRad);
    const float bodyX = cosH * toTarget.x + sinH * toTarget.y;  /* forward */
    const float bodyY = -sinH * toTarget.x + cosH * toTarget.y; /* left */
    const float ldM = maxf(sqrtf(bodyX * bodyX + bodyY * bodyY), 0.5f);
    const float alphaRad = atan2f(bodyY, bodyX);

    /* Pure pursuit: the arc through the lookahead point has curvature 2 sin(alpha) / L, and a
     * bicycle model tracks it at a road-wheel angle of atan(kappa * wheelbase). */
    const float arcCurvature = 2.0f * sinf(alphaRad) / ldM;
    float steerAngleRad = atanf(arcCurvature * spec->wheelbaseM);

    /* Derivative term. A growing error to the left (positive, rising) needs right steer. */
    if (state->hasPrevError && dt > 0.0f) {
        const float errorRateMps = (crossTrackM - state->prevCrossTrackErrorM) / dt;
        steerAngleRad -= cfg->steerGainD * errorRateMps;
    }

    const float maxAngleRad = maxf(spec->maxRoadWheelAngleRad, 1.0e-3f);
    out->steer = clampf(cfg->steerGainP * steerAngleRad / maxAngleRad, -1.0f, 1.0f);

    /* --- Speed target --- */
    float muLat = 1.0f, muLong = 1.0f;
    available_grip(spec, track, posM, &muLat, &muLong);
    const float latAccelLimit = maxf(cfg->corneringGripFraction * muLat * GRAVITY_MPS2, 0.1f);
    const float brakeAccelLimit = maxf(cfg->brakeGripFraction * muLong * GRAVITY_MPS2, 0.1f);

    const float horizonM =
        speedMps * speedMps / (2.0f * brakeAccelLimit) + AI_SPEED_SCAN_MARGIN_M;

    float targetSpeedMps = cfg->maxSpeedMps;
    float scannedM = 0.0f;
    Vector2 walk = bestPoint;
    for (int step = 0; step < count && scannedM <= horizonM; step++) {
        const int node = (bestSegment + 1 + step) % count;
        const Vector2 here = nodes[node].centerM;
        scannedM += vec_len(vec_sub(here, walk));
        walk = here;

        const float kappa = node_curvature(nodes, count, node);
        if (kappa < AI_MIN_CURVATURE_1PM) continue;

        /* Fastest this corner can be taken, then the fastest we may be going NOW and still
         * shed the difference before reaching it. */
        const float cornerSpeedMps = sqrtf(latAccelLimit / kappa);
        const float allowedMps =
            sqrtf(cornerSpeedMps * cornerSpeedMps + 2.0f * brakeAccelLimit * scannedM);
        if (allowedMps < targetSpeedMps) targetSpeedMps = allowedMps;
    }
    targetSpeedMps = clampf(targetSpeedMps, 0.0f, cfg->maxSpeedMps);

    const float speedErrorMps = targetSpeedMps - speedMps;
    if (speedErrorMps > cfg->speedDeadbandMps) {
        out->throttle = clampf(cfg->speedGainP * speedErrorMps, 0.0f, 1.0f);
        out->brake = 0.0f;
    } else if (speedErrorMps < -cfg->speedDeadbandMps) {
        out->throttle = 0.0f;
        out->brake = clampf(-cfg->speedGainP * speedErrorMps, 0.0f, 1.0f);
    } else {
        out->throttle = 0.0f;
        out->brake = 0.0f;
    }

    /* This driver has no handbrake. Stated as an assignment rather than an omission so that a
     * caller reusing an Input from a previous tick cannot inherit one. */
    out->handbrake = 0.0f;

    state->prevCrossTrackErrorM = crossTrackM;
    state->hasPrevError = true;
    state->nearestSegment = bestSegment;
    state->crossTrackErrorM = crossTrackM;
    state->targetSpeedMps = targetSpeedMps;
    state->lookaheadAngleRad = alphaRad;
}
