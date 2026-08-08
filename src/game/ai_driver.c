/*
 * ai_driver.c — an online path search through the road, then pure pursuit along what it found.
 *
 * Three layers, and only the first decides where to go:
 *
 *   PLANNING searches for the line. The road is a corridor: the authored centreline with a
 *   half-width either side. A lattice is laid over the next AI_PLAN_LAYERS nodes with
 *   AI_PLAN_OFFSETS candidate lateral offsets at each, and a dynamic program finds the chain
 *   through it that minimises integrated squared curvature. Curvature is the cost rather than
 *   time because at the friction limit a_lat = kappa * v^2, so it is curvature that sets how
 *   fast a path can be driven — and, unlike time, it does not depend on the speed the car
 *   arrives at, which is what keeps the cost separable and the search a plain shortest path.
 *   Minimising it is the classical minimum-curvature racing line, discovered here from geometry
 *   rather than read from a table.
 *
 *   STEERING follows the classic pure-pursuit law along the found line. A point is picked a
 *   speed-dependent distance ahead, and the steer angle is the one that would put the car on a
 *   circular arc through it. A derivative term on cross-track error damps the weave that pure
 *   pursuit alone develops when the lookahead is short relative to the wheelbase.
 *
 *   SPEED comes from the tightest curvature within braking range OF THE PLANNED LINE. Each
 *   upcoming point implies a cornering speed from the tyre's own lateral grip; back-propagating
 *   each of those through a constant-deceleration braking distance gives the fastest speed that
 *   is still stoppable in time for it. The minimum over the horizon is the target.
 *
 * The lattice stays inside the racing surface by construction — every candidate offset is
 * clamped to the half-width less planEdgeMarginM — so the found path cannot leave the road, and
 * because the checkpoint gates are wider than the surface, it cannot miss a gate either.
 *
 * The search runs every planReplanTicks, not every tick. A driver commits to a line through a
 * corner rather than reconsidering it 120 times a second, and a plan that flickered between two
 * near-equal lines would show up in the steering; the hysteresis term prices that directly.
 *
 * Everything read is either geometry the track publishes or grip the car publishes about itself.
 * Nothing here consults a force, a slip angle, or any other quantity a driver could not feel,
 * and nothing here writes anywhere but the Input.
 */
#include "game/ai_driver.h"

#include <math.h>
#include <stddef.h>

#include "core/config.h"
#include "core/math_utils.h"
#include "physics/surface.h"

/* Curvature below this is a straight; the reciprocal would otherwise overflow the target. */
#define AI_MIN_CURVATURE_1PM 1.0e-4f

/* Keep the scan close to the physical braking point: the speed limit already back-propagates
 * required deceleration, so a large extra margin would make the driver brake unnecessarily
 * early and sacrifice the exit line. */
#define AI_SPEED_SCAN_MARGIN_M 8.0f

/*
 * First plan layer whose curvature the speed controller is allowed to believe.
 *
 * Layer 0 is pinned to the car's own lateral position, so the triple centred on layer 1 measures
 * how the car rejoins its line, not what the road does. Half a metre of tracking error across a
 * 4 m node reads as a 20 m radius corner, and the speed controller brakes for it: the braking
 * upsets the car, the tracking error grows, the phantom tightens. rwd_power went from 27 m/s to
 * spun in two seconds that way, beached on the runoff, and — automatic box forward-only, no
 * handbrake — sat there for the remaining 90 s of the run. Binding radii measured mid-spiral
 * were 19 to 35 m, every one of them 1 to 2 m ahead, on a straight.
 *
 * Excluding it by PLAN INDEX rather than by distance matters. A distance-based skip was tried
 * first and was far worse: it also discards the curvature of the corner the car is currently
 * inside, so the driver stops limiting speed mid-corner and simply drives off the outside. Only
 * the anchor triple is an artefact; everything from layer 2 on is road.
 */
#define AI_PLAN_SPEED_SCAN_FIRST_LAYER 2

void ai_driver_config_default(AiDriverConfig *cfg)
{
    if (cfg == NULL) return;
    cfg->lookaheadBaseM = 6.5f;
    cfg->lookaheadSpeedS = 0.50f;
    cfg->corneringGripFraction = 0.75f;
    cfg->brakeGripFraction = 0.75f;
    cfg->steerGainP = 1.0f;
    cfg->steerGainD = 0.10f;
    cfg->speedGainP = 1.0f;
    cfg->speedDeadbandMps = 0.15f;
    cfg->maxSpeedMps = 80.0f;
    /* 200 ms from rest to full travel, 100 ms back off it. A deliberate press on a trigger
     * takes roughly the former; the spring returns it in roughly the latter. */
    cfg->pedalPressRatePerS = 5.0f;
    cfg->pedalReleaseRatePerS = 10.0f;
    /* 125 ms lock to lock, 83 ms back to centre. Faster than the road wheel's own
     * STEER_RATE_RAD_S so the stick bounds the jerk without becoming the limiting factor. */
    cfg->steerPressRatePerS = 8.0f;
    cfg->steerReleaseRatePerS = 12.0f;
    cfg->pedalDeadband = 0.02f;
    /* Ease off over ~0.6 s of sustained limit, feed it back over ~0.75 s. Cutting is quicker
     * than recovering because losing grip must be answered sooner than it is forgiven, but
     * recovery still has to finish inside the length of a short straight or the driver never
     * reaches full throttle at all. */
    cfg->gripCutThreshold = 0.97f;
    cfg->gripCutRatePerS = 1.4f;
    cfg->gripRecoverRatePerS = 1.15f;
    cfg->gripCutMax = 0.85f;
    /* Half a car plus the tracking error a pure-pursuit driver shows on a curved line. Measured
     * across the roster rather than inherited: 2.4 m is what the old offline optimiser used, and
     * at that value the low-grip awd_rally understeers off the outside of the chicane's curves
     * and spends a quarter of its run on the grass. 3.2 m puts every car on the roster at zero
     * off-track ticks and zero barrier contacts. 4.0 m is too far the other way — it leaves so
     * little road to plan in that rwd_power loses three seconds a lap. */
    cfg->planEdgeMarginM = 3.2f;
    /* Only the ratio between these two matters. The curvature weight carries the units the cost
     * is stated in; the hysteresis weight sits far enough below it that holding the previous
     * line breaks a tie without ever outvoting a genuinely straighter one. Raising it was tried
     * — 10, 50 and 200 each cost time on every circuit, because a plan held past its usefulness
     * is just a stale plan. */
    cfg->planCurvatureWeight = 7500.0f;
    cfg->planHysteresisWeight = 0.5f;
    cfg->planReplanTicks = 12; /* 10 Hz at 120 Hz fixed step */
}

/* Move an axis toward its demand no faster than the control can physically travel.
 * Moving away from zero is a press, moving toward zero is a release. */
static float slew_axis(float current, float target, float pressRatePerS, float releaseRatePerS,
                       float deadband, float dt)
{
    /* Hold rather than chase: a control held at a steady pressure does not dither, and
     * without this the limiter tracks every small wobble in the demand at full rate.
     *
     * The stops are exempt. A trigger can be pushed fully against its travel limit and
     * released fully to rest, so a demand of exactly 0 or of full travel is always followed;
     * applying the deadband there would leave the control permanently short of the rail and the
     * driver would never actually reach full throttle. */
    const bool targetAtStop = (target == 0.0f) || (fabsf(target) >= 1.0f);
    if (!targetAtStop && fabsf(target - current) <= deadband) return current;

    const float rate = (fabsf(target) >= fabsf(current)) ? pressRatePerS : releaseRatePerS;
    const float maxStep = maxf(rate, 0.0f) * dt;
    const float delta = target - current;
    if (delta > maxStep) return current + maxStep;
    if (delta < -maxStep) return current - maxStep;
    return target;
}

/* ------------------------------------------------------------------------------------- */
/* Geometry                                                                                */
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
 * Menger curvature of the triangle abc: kappa = 2|AB x BC| / (|AB||BC||CA|).
 *
 * Exact for points sampled off a circle at any spacing, which is what the constant-radius
 * curves are, and it degrades gracefully to zero on a straight rather than dividing by a
 * near-zero radius.
 */
static float menger_curvature(Vector2 a, Vector2 b, Vector2 c)
{
    const Vector2 ab = vec_sub(b, a);
    const Vector2 bc = vec_sub(c, b);
    const Vector2 ca = vec_sub(a, c);
    const float denom = vec_len(ab) * vec_len(bc) * vec_len(ca);
    if (denom < 1.0e-6f) return 0.0f;

    const float cross = ab.x * bc.y - ab.y * bc.x;
    return 2.0f * fabsf(cross) / denom;
}

/* Unit normal to the centreline at node i, pointing left of travel. Taken from the chord
 * between the node's neighbours so it is the tangent of the curve rather than of one segment. */
static Vector2 node_normal(const Track *track, int i)
{
    const int count = track->count;
    const Vector2 prev = track->nodes[(i - 1 + count) % count].centerM;
    const Vector2 next = track->nodes[(i + 1) % count].centerM;
    const Vector2 tangent = vec_sub(next, prev);
    const float length = vec_len(tangent);
    if (length <= 1.0e-6f) return (Vector2){ 0.0f, 0.0f };
    return (Vector2){ -tangent.y / length, tangent.x / length };
}

/* How far either side of the centreline a planned point may sit at node i. */
static float usable_half_width(const Track *track, int i, float marginM)
{
    const float usable = track->nodes[i].halfWidthM - marginM;
    return (usable > 0.0f) ? usable : 0.0f;
}

/* The planned lateral offset at a centreline node, or 0 (the centreline) outside the window. */
static float plan_offset_at(const AiDriverState *state, const Track *track, int nodeIndex)
{
    if (state->planLayerCount <= 0) return 0.0f;
    const int count = track->count;
    const int rel = ((nodeIndex - state->planBaseNode) % count + count) % count;
    return (rel < state->planLayerCount) ? state->planOffsetM[rel] : 0.0f;
}

Vector2 ai_driver_plan_point(const AiDriverState *state, const Track *track, int nodeIndex)
{
    if (state == NULL || track == NULL || track->nodes == NULL || track->count < 3)
        return (Vector2){ 0.0f, 0.0f };

    const int count = track->count;
    const int node = ((nodeIndex % count) + count) % count;
    const Vector2 centre = track->nodes[node].centerM;
    const float offsetM = plan_offset_at(state, track, node);
    if (offsetM == 0.0f) return centre;

    const Vector2 normal = node_normal(track, node);
    return (Vector2){ centre.x + normal.x * offsetM, centre.y + normal.y * offsetM };
}

/*
 * Walk `distanceM` forward along the planned line from (segment, t) and return where that lands.
 *
 * The walk is capped at one lap so a lookahead longer than the circuit cannot spin forever.
 */
static Vector2 plan_point_ahead(const AiDriverState *state, const Track *track, int segment,
                                float t, float distanceM)
{
    const int count = track->count;
    const Vector2 a = ai_driver_plan_point(state, track, segment);
    const Vector2 b = ai_driver_plan_point(state, track, segment + 1);
    Vector2 from = { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
    float remainingM = distanceM;

    for (int step = 0; step < count; step++) {
        const Vector2 target = ai_driver_plan_point(state, track, segment + 1 + step);
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
/* The path search                                                                         */
/* ------------------------------------------------------------------------------------- */

/*
 * Find the minimum-curvature chain through the corridor and store it as the plan.
 *
 * `baseSegment` is the centreline segment the car is currently on, so layer 0 is behind the car
 * and layer 1 ahead of it; `carOffsetM` is the car's own lateral offset there, which anchors
 * layer 0 so the line the search returns starts from where the car already is rather than
 * stepping sideways onto a fresh one.
 *
 * Only the position is pinned, not the direction. Pinning layer 1 to a heading extrapolation as
 * well was tried and cost 4 s a lap on every circuit: a straight probe off the body heading
 * lands outside the arc a cornering car is already describing, and the search then spends the
 * whole horizon recovering from an opening kink that was never real. The car's own position is
 * data; its instantaneous heading, with sideslip in it, is not the direction its path is going.
 *
 * THE ANCHOR IS FOR THE TRACKER, NOT THE SPEED CONTROLLER. Layer 0 carries the car's tracking
 * error, so the first triple of the plan encodes how the car rejoins its line rather than what
 * the road does — and 0.5 m of error over a 4 m node reads as a 20 m radius corner. The speed
 * scan therefore ignores it; see AI_PLAN_SPEED_SCAN_FIRST_LAYER.
 *
 * The state is the PAIR of offsets at the two most recent layers, because curvature is a
 * property of three consecutive points and a per-node state could not see it. That makes the
 * table AI_PLAN_OFFSETS^2 wide and the transition cost O(AI_PLAN_OFFSETS) — the reason the
 * offset count is the parameter worth keeping small.
 */
static void plan_path(const AiDriverConfig *cfg, AiDriverState *state, const Track *track,
                      int baseSegment, float carOffsetM)
{
    const int count = track->count;
    const int layers = (count < AI_PLAN_LAYERS) ? count : AI_PLAN_LAYERS;

    /* The previous plan, sampled per layer of the NEW window, before it is overwritten. */
    float prevOffsetM[AI_PLAN_LAYERS];
    const bool hadPlan = (state->planLayerCount > 0);
    for (int L = 0; L < layers; L++) {
        prevOffsetM[L] = hadPlan ? plan_offset_at(state, track, baseSegment + L) : 0.0f;
    }

    /* Lattice vertices. Recomputed per search rather than stored: they are a pure function of
     * geometry the track already publishes, and materialising them would cost more memory than
     * the arithmetic saves. */
    Vector2 point[AI_PLAN_LAYERS][AI_PLAN_OFFSETS];
    float offset[AI_PLAN_LAYERS][AI_PLAN_OFFSETS];

    for (int L = 0; L < layers; L++) {
        const int node = (baseSegment + L) % count;
        const Vector2 centre = track->nodes[node].centerM;
        const Vector2 normal = node_normal(track, node);
        const float usable = usable_half_width(track, node, cfg->planEdgeMarginM);

        for (int j = 0; j < AI_PLAN_OFFSETS; j++) {
            float o;
            if (L == 0) {
                /* One candidate, and it is where the car is. */
                o = clampf(carOffsetM, -usable, usable);
            } else {
                const float t = (float)j / (float)(AI_PLAN_OFFSETS - 1);
                o = -usable + 2.0f * usable * t;
            }
            offset[L][j] = o;
            point[L][j] = (Vector2){ centre.x + normal.x * o, centre.y + normal.y * o };
        }
    }

    if (layers < 3) {
        for (int L = 0; L < layers; L++) state->planOffsetM[L] = offset[L][0];
        state->planLayerCount = layers;
        state->planBaseNode = baseSegment;
        return;
    }

    /* cost[i][j]: best cost of a chain whose last two layers sit at offsets i and j.
     * Layer 0 has a single candidate, so only i == 0 is reachable when the chain is at layer 1. */
    float cost[AI_PLAN_OFFSETS][AI_PLAN_OFFSETS];
    unsigned char back[AI_PLAN_LAYERS][AI_PLAN_OFFSETS][AI_PLAN_OFFSETS];

    for (int i = 0; i < AI_PLAN_OFFSETS; i++) {
        for (int j = 0; j < AI_PLAN_OFFSETS; j++) {
            cost[i][j] = (i == 0)
                             ? cfg->planHysteresisWeight * fabsf(offset[1][j] - prevOffsetM[1])
                             : INFINITY;
        }
    }

    for (int L = 2; L < layers; L++) {
        float next[AI_PLAN_OFFSETS][AI_PLAN_OFFSETS];

        for (int j = 0; j < AI_PLAN_OFFSETS; j++) {
            for (int k = 0; k < AI_PLAN_OFFSETS; k++) {
                /* Everything about the transition that does not depend on where the chain came
                 * from: the length it adds and how far it departs from the previous plan. */
                const float legM = vec_len(vec_sub(point[L][k], point[L - 1][j]));
                const float hysteresis =
                    cfg->planHysteresisWeight * fabsf(offset[L][k] - prevOffsetM[L]);

                float best = INFINITY;
                int bestFrom = 0;
                for (int i = 0; i < AI_PLAN_OFFSETS; i++) {
                    if (cost[i][j] == INFINITY) continue;
                    const float kappa =
                        menger_curvature(point[L - 2][i], point[L - 1][j], point[L][k]);
                    const float total =
                        cost[i][j] + legM * cfg->planCurvatureWeight * kappa * kappa;
                    if (total < best) {
                        best = total;
                        bestFrom = i;
                    }
                }
                next[j][k] = (best == INFINITY) ? INFINITY : best + hysteresis;
                back[L][j][k] = (unsigned char)bestFrom;
            }
        }

        for (int j = 0; j < AI_PLAN_OFFSETS; j++)
            for (int k = 0; k < AI_PLAN_OFFSETS; k++) cost[j][k] = next[j][k];
    }

    int bestJ = 0, bestK = 0;
    float bestCost = INFINITY;
    for (int j = 0; j < AI_PLAN_OFFSETS; j++) {
        for (int k = 0; k < AI_PLAN_OFFSETS; k++) {
            if (cost[j][k] < bestCost) {
                bestCost = cost[j][k];
                bestJ = j;
                bestK = k;
            }
        }
    }

    state->planOffsetM[layers - 1] = offset[layers - 1][bestK];
    state->planOffsetM[layers - 2] = offset[layers - 2][bestJ];
    int curJ = bestJ, curK = bestK;
    for (int L = layers - 1; L >= 2; L--) {
        const int from = back[L][curJ][curK];
        state->planOffsetM[L - 2] = offset[L - 2][from];
        curK = curJ;
        curJ = from;
    }

    state->planLayerCount = layers;
    state->planBaseNode = baseSegment;
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

    const float tyreMu = minf(spec->tireMuLatFront, spec->tireMuLatRear);
    *muLatOut = tyreMu * (surface->muLateral / SURFACE_REFERENCE_MU_LAT);
    *muLongOut =
        tyreMu * spec->tireMuLongScale * (surface->muLongitudinal / SURFACE_REFERENCE_MU_LONG);
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

    /* --- Replan ---
     * Anchored to the CENTRELINE segment the car is on, because that is the frame the corridor
     * and the lattice are defined in. A full scan rather than a search around last tick's
     * answer: 170 segments is nothing at 10 Hz, and it cannot lose the road after a spin or a
     * barrier shove, which a local search can. */
    const bool needsPlan =
        (state->planLayerCount <= 0) || (state->ticksSinceReplan >= cfg->planReplanTicks);
    if (needsPlan) {
        int centreSegment = 0;
        float centreDistM = INFINITY;
        Vector2 centrePoint = nodes[0].centerM;
        for (int i = 0; i < count; i++) {
            float t = 0.0f;
            Vector2 p = { 0.0f, 0.0f };
            const float d = closest_on_segment(nodes[i].centerM, nodes[(i + 1) % count].centerM,
                                               posM, &t, &p);
            if (d < centreDistM) {
                centreDistM = d;
                centreSegment = i;
                centrePoint = p;
            }
        }

        /* Signed lateral offset of the car in the centreline frame, positive to the left. */
        const Vector2 normal = node_normal(track, centreSegment);
        const Vector2 toCar = vec_sub(posM, centrePoint);
        const float carOffsetM = toCar.x * normal.x + toCar.y * normal.y;

        plan_path(cfg, state, track, centreSegment, carOffsetM);
        state->ticksSinceReplan = 0;
    } else {
        state->ticksSinceReplan++;
    }

    /* --- Where the car is on the planned line --- */
    int bestSegment = 0;
    float bestT = 0.0f;
    Vector2 bestPoint = ai_driver_plan_point(state, track, 0);
    float bestDistM = INFINITY;
    for (int i = 0; i < count; i++) {
        float t = 0.0f;
        Vector2 p = { 0.0f, 0.0f };
        const float d =
            closest_on_segment(ai_driver_plan_point(state, track, i),
                               ai_driver_plan_point(state, track, i + 1), posM, &t, &p);
        if (d < bestDistM) {
            bestDistM = d;
            bestSegment = i;
            bestT = t;
            bestPoint = p;
        }
    }

    /* Signed cross-track error, positive when the car is LEFT of the planned line. Body Y is
     * left, so the left normal of a forward direction (fx,fy) is (-fy,fx). */
    const Vector2 segDir = vec_sub(ai_driver_plan_point(state, track, bestSegment + 1),
                                   ai_driver_plan_point(state, track, bestSegment));
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
    const Vector2 targetM = plan_point_ahead(state, track, bestSegment, bestT, lookaheadM);

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
    const float steerDemand =
        clampf(cfg->steerGainP * steerAngleRad / maxAngleRad, -1.0f, 1.0f);

    /* Rate-limit the stick for the same reason the pedals are limited: a thumbstick cannot
     * teleport, so an instantaneous step is a signal no player could have produced. */
    state->steerAxis = slew_axis(state->steerAxis, steerDemand, cfg->steerPressRatePerS,
                                 cfg->steerReleaseRatePerS, cfg->pedalDeadband, dt);
    out->steer = state->steerAxis;

    /* --- Speed target --- */
    float muLat = 1.0f, muLong = 1.0f;
    available_grip(spec, track, posM, &muLat, &muLong);
    const float latAccelLimit = maxf(cfg->corneringGripFraction * muLat * GRAVITY_MPS2, 0.1f);
    const float brakeAccelLimit = maxf(cfg->brakeGripFraction * muLong * GRAVITY_MPS2, 0.1f);

    const float horizonM =
        speedMps * speedMps / (2.0f * brakeAccelLimit) + AI_SPEED_SCAN_MARGIN_M;

    float targetSpeedMps = cfg->maxSpeedMps;
    float bindingCurvature1pm = 0.0f;
    float bindingDistanceM = 0.0f;
    float scannedM = 0.0f;
    Vector2 walk = bestPoint;
    for (int step = 0; step < count && scannedM <= horizonM; step++) {
        const int node = bestSegment + 1 + step;
        const Vector2 here = ai_driver_plan_point(state, track, node);
        scannedM += vec_len(vec_sub(here, walk));
        walk = here;

        /* Skip the anchor triple: see AI_PLAN_SPEED_SCAN_FIRST_LAYER. */
        const int planLayer = ((node - state->planBaseNode) % count + count) % count;
        if (planLayer < AI_PLAN_SPEED_SCAN_FIRST_LAYER) continue;

        const float kappa = menger_curvature(ai_driver_plan_point(state, track, node - 1), here,
                                             ai_driver_plan_point(state, track, node + 1));
        if (kappa < AI_MIN_CURVATURE_1PM) continue;

        /* Fastest this corner can be taken, then the fastest we may be going NOW and still
         * shed the difference before reaching it. */
        const float cornerSpeedMps = sqrtf(latAccelLimit / kappa);
        const float allowedMps =
            sqrtf(cornerSpeedMps * cornerSpeedMps + 2.0f * brakeAccelLimit * scannedM);
        if (allowedMps < targetSpeedMps) {
            targetSpeedMps = allowedMps;
            bindingCurvature1pm = kappa;
            bindingDistanceM = scannedM;
        }
    }
    targetSpeedMps = clampf(targetSpeedMps, 0.0f, cfg->maxSpeedMps);

    /* Traction management: ease off while the tyres are at the limit, feed the power back in
     * once they hook up again. Both directions are rate-bounded, so the throttle this produces
     * is smooth however jagged the underlying friction reading is. */
    if (derived->maxFrictionUsage >= cfg->gripCutThreshold) {
        state->gripCut += cfg->gripCutRatePerS * dt;
    } else {
        state->gripCut -= cfg->gripRecoverRatePerS * dt;
    }
    state->gripCut = clampf(state->gripCut, 0.0f, cfg->gripCutMax);

    /* One signed longitudinal demand: +1 full throttle, -1 full brake. Expressing it as a
     * single axis is what makes "never both pedals" structural rather than a rule to obey. */
    const float speedErrorMps = targetSpeedMps - speedMps;
    float pedalDemand;
    if (speedErrorMps >= -cfg->speedDeadbandMps) {
        /* Full throttle is the default on the planned line: the curvature scan has already
         * reduced targetSpeedMps before a corner, so being under the speed target means the
         * power is wanted. Traction management is the only thing that takes any of it away, and
         * only for as long as the tyres are actually saturated. */
        pedalDemand = clampf(1.0f - state->gripCut, 0.0f, 1.0f);
    } else {
        /* Once late braking is required, use the shortest available deceleration rather than
         * coasting into the corner. The target was back-propagated from its tightest point. */
        pedalDemand = -clampf(-cfg->speedGainP * speedErrorMps, 0.0f, 1.0f);
    }

    /* Rate-limit the axis, then split it. The driver must therefore lift off before it can
     * brake, exactly as a player moving between two triggers must. */
    state->pedalAxis =
        slew_axis(state->pedalAxis, clampf(pedalDemand, -1.0f, 1.0f), cfg->pedalPressRatePerS,
                  cfg->pedalReleaseRatePerS, cfg->pedalDeadband, dt);
    out->throttle = maxf(state->pedalAxis, 0.0f);
    out->brake = maxf(-state->pedalAxis, 0.0f);

    /* This driver has no handbrake. Stated as an assignment rather than an omission so that a
     * caller reusing an Input from a previous tick cannot inherit one. */
    out->handbrake = 0.0f;

    state->prevCrossTrackErrorM = crossTrackM;
    state->hasPrevError = true;
    state->nearestSegment = bestSegment;
    state->crossTrackErrorM = crossTrackM;
    state->targetSpeedMps = targetSpeedMps;
    state->lookaheadAngleRad = alphaRad;
    state->bindingCurvature1pm = bindingCurvature1pm;
    state->bindingDistanceM = bindingDistanceM;
}
