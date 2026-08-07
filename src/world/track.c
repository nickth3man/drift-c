/*
 * track.c — track geometry, surface bands, and ordered lap checkpoints.
 *
 * Two layouts live here: the open parking lot used for free driving, and the chicane circuit
 * every car is validated against. Both arrays are calloc'd here and freed by track_free();
 * they survive hot reloads because they are heap memory, not module static data.
 */
#include "world/track.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FNV1A_OFFSET_BASIS 2166136261u
#define FNV1A_PRIME 16777619u

/* --------------- straight-line helpers ---------------------------------------------------- */

/* Squared distance from point p to the finite line segment a→b. */
static float point_to_segment_sq(Vector2 p, Vector2 a, Vector2 b)
{
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float lenSq = dx * dx + dy * dy;
    if (lenSq < 1e-12f) {
        /* Degenerate segment: distance to the single point. */
        const float ex = p.x - a.x;
        const float ey = p.y - a.y;
        return ex * ex + ey * ey;
    }
    /* Projection parameter t clamped to [0, 1]. */
    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    const float closestX = a.x + t * dx;
    const float closestY = a.y + t * dy;
    const float ex = p.x - closestX;
    const float ey = p.y - closestY;
    return ex * ex + ey * ey;
}

/* Closest centreline segment: returns the squared perpendicular distance and sets *closestIdx
 * to the segment index (the i in nodes[i]→nodes[(i+1)%count]). *closestIdx is untouched
 * when closestIdx is NULL. */
static float nearest_centerline_distance_sq(const TrackNode *nodes, int count, Vector2 point,
                                            int *closestIdx)
{
    float best = 1e30f;
    int bestIdx = 0;
    for (int i = 0; i < count; i++) {
        const int j = (i + 1) % count;
        const float dSq = point_to_segment_sq(point, nodes[i].centerM, nodes[j].centerM);
        if (dSq < best) {
            best = dSq;
            bestIdx = i;
        }
    }
    if (closestIdx != NULL) *closestIdx = bestIdx;
    return best;
}

/* --------------- public API -------------------------------------------------------------- */

void track_init(Track *track)
{
    if (track == NULL) return;
    /* Defensive: free any previously-allocated state first. */
    track_free(track);

    /* Parking lot: 400m x 300m open rectangle centred at origin. */
    track->isParkingLot = true;
    track->lotMinXM = -200.0f;
    track->lotMaxXM = 200.0f;
    track->lotMinYM = -150.0f;
    track->lotMaxYM = 150.0f;

    /* Perimeter centreline for collision barriers. 4 sides in clockwise order:
     * bottom (L->R), right (B->T), top (R->L), left (T->B), plus a closing node. */
#define LOT_NODES 5
    track->count = LOT_NODES;
    track->nodes = (TrackNode *)calloc((size_t)track->count, sizeof(TrackNode));
    if (!track->nodes) return;

    /* No runoff band: the lot's perimeter barrier stands on the edge of the drivable area,
     * which is what it has always done. Stated explicitly rather than left to zero-fill so
     * the intent is visible and the compiler does not warn about a partial initialiser. */
    const float hw = 4.0f; /* wide enough so inner/outer barriers don't sandwich the car */
    const float noRunoff = 0.0f;
    int i = 0;
    /* The lot has no optimised line: a target point equal to the centreline is the correct
     * answer for an open area, and it keeps every consumer free of a null-line special case. */
    const Vector2 corners[LOT_NODES] = {
        { track->lotMinXM, track->lotMinYM }, { track->lotMaxXM, track->lotMinYM },
        { track->lotMaxXM, track->lotMaxYM }, { track->lotMinXM, track->lotMaxYM },
        { track->lotMinXM, track->lotMinYM },
    };
    for (i = 0; i < LOT_NODES; i++) {
        track->nodes[i] = (TrackNode){ corners[i], hw, SURFACE_ASPHALT, noRunoff, corners[i] };
    }

    track->offTrackSurfaceId = SURFACE_GRASS;
    track->runoffSurfaceId = SURFACE_GRASS;

    /* The lot is an open area, not a circuit, but the perimeter still carries gates so that
     * lap bookkeeping behaves identically to before gates became explicit data. Progress
     * starts at gate 0 rather than at track_reset_progress()'s post-start gate: nothing here
     * is a standing start on a finish line. */
    track_build_checkpoints_from_nodes(track);
    track->nextCheckpoint = 0;
    track->lap = 0;
    track->lapStartCheckpoint = 0;
    track->lapTimerS = 0.0f;
    track->lastLapTimeS = 0.0f;
    snprintf(track->id, sizeof(track->id), "%s", "parking_lot");
    snprintf(track->version, sizeof(track->version), "%s", "v1");
}

void track_free(Track *track)
{
    if (track == NULL) return;
    free(track->nodes);
    track->nodes = NULL;
    track->count = 0;
    free(track->checkpoints);
    track->checkpoints = NULL;
    track->checkpointCount = 0;
    track->offTrackSurfaceId = SURFACE_ASPHALT;
    track->runoffSurfaceId = SURFACE_ASPHALT;
    track->nextCheckpoint = 0;
    track->lap = 0;
    track->lapStartCheckpoint = 0;
    track->lapTimerS = 0.0f;
    track->lastLapTimeS = 0.0f;
    track->id[0] = '\0';
    track->version[0] = '\0';
}

SurfaceId Track_SurfaceAt(const Track *track, Vector2 pointM)
{
    /* Headless tests and scenarios that don't initialise a track hit this safe default, so
     * existing scenarios remain on asphalt and their CSVs are unchanged. */
    if (track == NULL || track->nodes == NULL || track->count <= 0) {
        return SURFACE_ASPHALT;
    }

    /* Parking lot: simple AABB test. */
    if (track->isParkingLot) {
        if (pointM.x >= track->lotMinXM && pointM.x <= track->lotMaxXM &&
            pointM.y >= track->lotMinYM && pointM.y <= track->lotMaxYM)
            return SURFACE_ASPHALT;
        return track->offTrackSurfaceId; /* grass */
    }

    int closestIdx = 0;
    const float dSq =
        nearest_centerline_distance_sq(track->nodes, track->count, pointM, &closestIdx);
    const TrackNode *seg = &track->nodes[closestIdx];

    /* Three bands: racing surface, then runoff, then off-track. The runoff band is what makes
     * an excursion measurable — without it the racing surface ends exactly where the barrier
     * begins and a car can never be off-track without also being in a wall. */
    if (dSq <= seg->halfWidthM * seg->halfWidthM) {
        return seg->surfaceId;
    }
    const float barrierHalfWidthM = track_node_barrier_half_width(seg);
    if (dSq <= barrierHalfWidthM * barrierHalfWidthM) {
        return track->runoffSurfaceId;
    }
    return track->offTrackSurfaceId;
}

float track_distance_to_centerline_m(const Track *track, Vector2 pointM, float *halfWidthM)
{
    if (track == NULL || track->nodes == NULL || track->count <= 0) {
        if (halfWidthM != NULL) *halfWidthM = 0.0f;
        return 0.0f;
    }
    int closestIdx = 0;
    const float dSq =
        nearest_centerline_distance_sq(track->nodes, track->count, pointM, &closestIdx);
    if (halfWidthM != NULL) {
        *halfWidthM = track->nodes[closestIdx].halfWidthM;
    }
    return sqrtf(dSq);
}

/* --------------- checkpoint crossing ------------------------------------------------------
 *
 * A gate is a line segment at each TrackNode, perpendicular to the centreline, spanning the
 * track width. The car crosses a gate when its prev→curr position segment intersects it, and
 * only when the car is moving in the track's forward direction.
 */

/* 2D cross-product z-component: a.x * b.y - a.y * b.x */
static float cross_z(Vector2 a, Vector2 b)
{
    return a.x * b.y - a.y * b.x;
}

/* Orientation test: > 0 if points a,b,c are counterclockwise. */
static float orient(Vector2 a, Vector2 b, Vector2 c)
{
    return cross_z((Vector2){ b.x - a.x, b.y - a.y }, (Vector2){ c.x - a.x, c.y - a.y });
}

/* Standard segment-segment intersection test (including endpoints). */
static bool segments_intersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4)
{
    const float d1 = orient(p3, p4, p1);
    const float d2 = orient(p3, p4, p2);
    const float d3 = orient(p1, p2, p3);
    const float d4 = orient(p1, p2, p4);

    /* Use a tolerance rather than strict sign checks so collinear grazing counts. */
    const float eps = 1e-9f;

    /* General case: the endpoints of each segment are on opposite sides of the other. */
    if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
        ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))) {
        return true;
    }

    /* Degenerate / collinear: any endpoint lies on the other segment. */
    if (fabsf(d1) <= eps && fabsf(d2) <= eps && fabsf(d3) <= eps && fabsf(d4) <= eps) {
        /* Overlap check: bounding-box test. */
        const float mnx1 = fminf(p1.x, p2.x) - eps;
        const float mxx1 = fmaxf(p1.x, p2.x) + eps;
        const float mny1 = fminf(p1.y, p2.y) - eps;
        const float mxy1 = fmaxf(p1.y, p2.y) + eps;
        const float mnx2 = fminf(p3.x, p4.x) - eps;
        const float mxx2 = fmaxf(p3.x, p4.x) + eps;
        const float mny2 = fminf(p3.y, p4.y) - eps;
        const float mxy2 = fmaxf(p3.y, p4.y) + eps;

        /* Check if any endpoint of one segment falls inside the bounding box of the other. */
        if ((p3.x >= mnx1 && p3.x <= mxx1 && p3.y >= mny1 && p3.y <= mxy1) ||
            (p4.x >= mnx1 && p4.x <= mxx1 && p4.y >= mny1 && p4.y <= mxy1) ||
            (p1.x >= mnx2 && p1.x <= mxx2 && p1.y >= mny2 && p1.y <= mxy2) ||
            (p2.x >= mnx2 && p2.x <= mxx2 && p2.y >= mny2 && p2.y <= mxy2)) {
            return true;
        }
        return false;
    }

    return false;
}

/* --------------- the chicane validation circuit --------------------------------------------
 *
 * A closed stadium — two 200 m straights joined by 45 m-radius 180-degree curves — with a
 * left-right chicane set into the far straight. About 690 m, so a competent lap is roughly
 * half a minute and an out-lap plus a timed lap fits comfortably inside the replay buffer.
 *
 * The shape is chosen for what it measures, not for interest: the main straight is long
 * enough to reach a speed where braking matters, the curves are constant-radius so a car's
 * steady-state balance is observable, and the chicane forces a genuine direction change with
 * no time to settle between the two apexes. That last part is the one a car with lazy
 * turn-in, too much rearward brake bias, or a snappy rear axle will fail.
 *
 * The chicane's lateral displacement follows A*sin^2(pi*u), which starts and ends with zero
 * slope. That matters structurally, not just aesthetically: barriers are built per segment,
 * so a kink in the centreline becomes a concave joint that a swept capsule can catch on.
 */

#define CHICANE_STRAIGHT_HALF_X_M 100.0f /* straights run x = -100 .. +100 */
#define CHICANE_CURVE_RADIUS_M 45.0f
#define CHICANE_FAR_STRAIGHT_Y_M 90.0f /* = 2 * curve radius, so the curves close the loop */
#define CHICANE_NODE_SPACING_M 4.0f
#define CHICANE_OFFSET_M 16.0f    /* chicane lateral displacement */
#define CHICANE_SPAN_M 80.0f      /* distance over which it displaces and returns */
#define CHICANE_ENTRY_X_M 55.0f   /* chicane begins here on the far straight */
#define TRACK_HALF_WIDTH_M 8.0f   /* racing surface, straights and curves */
#define TRACK_RUNOFF_HALF_M 12.0f /* barrier stands here; 4 m of grass in between */
#define CHICANE_HALF_WIDTH_M 6.0f /* the chicane is deliberately tighter */
#define CHICANE_RUNOFF_HALF_M 8.0f
#define GATE_HALF_WIDTH_M \
    10.0f /* gates span past the racing surface so a wide but legal
                                 * line still scores; they validate route, not precision */

typedef struct {
    TrackNode *nodes;
    int count;
    int capacity;
} NodeBuilder;

static void builder_push(NodeBuilder *b, float x, float y, float halfWidthM, float runoffM)
{
    if (b->nodes == NULL || b->count >= b->capacity) return;
    b->nodes[b->count].centerM = (Vector2){ x, y };
    b->nodes[b->count].halfWidthM = halfWidthM;
    b->nodes[b->count].surfaceId = SURFACE_ASPHALT;
    b->nodes[b->count].runoffHalfWidthM = runoffM;
    b->nodes[b->count].racingLineM = b->nodes[b->count].centerM;
    b->count++;
}

/* The chicane's lateral offset at arc fraction u in [0,1]. Zero slope at both ends. */
static float chicane_offset_at(float u)
{
    const float s = sinf(3.14159265358979323846f * u);
    return CHICANE_OFFSET_M * s * s;
}
/*
 * Learned target racing lines.
 *
 * Each array is the output of tools/validation/learn_racing_line.py: a small MLP policy whose
 * weights are searched by deterministic cross-entropy neuroevolution against a friction-limited
 * lap-time model, constrained to stay inside the racing surface and to pass every ordered gate.
 * The offsets are stored as data rather than recomputed at load time so the simulation keeps
 * bit-identical determinism and no runtime file dependency. Regenerate with:
 *
 *   python tools/validation/learn_racing_line.py --track <chicane|sprint|technical>
 */
#define RACING_LINE_CONTROL_COUNT 48

/* Asphalt the target line always leaves between itself and the edge: half a car plus the
 * tracking error a pure-pursuit driver shows on a curved line. Mirrors LINE_EDGE_MARGIN_M in
 * tools/validation/learn_racing_line.py, which is where the offsets below come from. */
#define RACING_LINE_EDGE_MARGIN_M 2.4f

static const float chicaneRacingLineOffsetsM[RACING_LINE_CONTROL_COUNT] = {
    4.203433f, 3.501952f, 3.094172f, 2.924932f, 2.942308f, 2.954400f, 2.946107f, 2.883061f,
    2.832930f, 2.806470f, 2.841968f, 2.905167f, 3.040182f, 3.457055f, 4.080215f, 4.743702f,
    4.928899f, 5.023841f, 5.006110f, 4.977602f, 4.970184f, 4.922951f, 4.734815f, 4.409962f,
    3.608801f, 2.659580f, 1.990668f, 1.793507f, 1.711879f, 2.264491f, 2.608949f, 2.183696f,
    1.630055f, 1.581713f, 1.864908f, 2.207226f, 2.439383f, 2.977775f, 3.770693f, 4.581289f,
    4.849478f, 5.006745f, 5.025177f, 5.026560f, 5.038845f, 5.049263f, 4.991628f, 4.821791f,
};

static const float sprintRacingLineOffsetsM[RACING_LINE_CONTROL_COUNT] = {
    4.128194f, 3.563185f, 3.468396f, 3.456193f, 3.536170f, 3.600452f, 3.592642f, 3.509487f,
    3.444480f, 3.413158f, 3.413106f, 3.418004f, 3.493897f, 3.793149f, 4.223717f, 4.671504f,
    4.680960f, 4.654544f, 4.565728f, 4.484815f, 4.471361f, 4.418684f, 4.244075f, 3.922720f,
    3.051397f, 2.119795f, 1.477665f, 1.295357f, 1.330572f, 2.083652f, 2.362090f, 1.757652f,
    0.991898f, 1.090016f, 1.475309f, 1.911820f, 2.446432f, 3.011854f, 3.710677f, 4.447579f,
    4.580745f, 4.657152f, 4.658769f, 4.663110f, 4.697817f, 4.745900f, 4.754998f, 4.652003f,
};

static const float technicalRacingLineOffsetsM[RACING_LINE_CONTROL_COUNT] = {
    2.977061f, 2.685542f, 2.607393f, 2.577832f, 2.632068f, 2.672957f, 2.629338f, 2.439148f,
    2.317213f, 2.266803f, 2.193323f, 2.100755f, 2.080360f, 2.198166f, 2.605405f, 3.051784f,
    3.074054f, 3.130259f, 3.157256f, 3.154750f, 3.164451f, 3.112223f, 2.976078f, 2.828533f,
    2.279205f, 1.703046f, 1.197010f, 0.931442f, 0.845105f, 1.544803f, 1.802673f, 1.400460f,
    0.798539f, 0.916805f, 1.315934f, 1.523162f, 1.881476f, 2.143848f, 2.610810f, 3.106198f,
    3.157555f, 3.216382f, 3.246782f, 3.253034f, 3.252252f, 3.246610f, 3.241533f, 3.221304f,
};

/*
 * Displace every node's target line laterally from the authored centreline by the interpolated
 * control offset, positive to the left of travel.
 *
 * The centreline itself is untouched: surface bands, barriers, gates, and the off-track metric
 * all continue to measure against the authored geometry. Only the path a driver aims at moves.
 */
static void apply_racing_line_controls(Track *track, const float *controls, int controlCount)
{
    if (track == NULL || track->nodes == NULL || track->count < 3) return;
    if (controls == NULL || controlCount < 2) return;

    for (int i = 0; i < track->count; i++) {
        const Vector2 prev = track->nodes[(i - 1 + track->count) % track->count].centerM;
        const Vector2 next = track->nodes[(i + 1) % track->count].centerM;
        const Vector2 tangent = { next.x - prev.x, next.y - prev.y };
        const float tangentLength = sqrtf(tangent.x * tangent.x + tangent.y * tangent.y);
        if (tangentLength <= 1.0e-6f) {
            track->nodes[i].racingLineM = track->nodes[i].centerM;
            continue;
        }

        const float phase = (float)i * (float)controlCount / (float)track->count;
        const int lower = ((int)phase) % controlCount;
        const int upper = (lower + 1) % controlCount;
        const float fraction = phase - floorf(phase);
        float offsetM = controls[lower] + (controls[upper] - controls[lower]) * fraction;

        /* Whatever the optimiser produced, the road's own bound is the track's to enforce. The
         * margin is half a car plus the tracking error a pure-pursuit driver shows on a curved
         * line: put the target any closer to the edge and a slightly understeering car runs out
         * of asphalt, which costs it more time than the shorter path ever saved. */
        const float widthLimitM = track->nodes[i].halfWidthM - RACING_LINE_EDGE_MARGIN_M;
        const float limitM = (widthLimitM > 0.0f) ? widthLimitM : 0.0f;
        if (offsetM > limitM) offsetM = limitM;
        if (offsetM < -limitM) offsetM = -limitM;

        const Vector2 normal = { -tangent.y / tangentLength, tangent.x / tangentLength };
        track->nodes[i].racingLineM =
            (Vector2){ track->nodes[i].centerM.x + normal.x * offsetM,
                       track->nodes[i].centerM.y + normal.y * offsetM };
    }
}

void track_load_chicane(Track *track)
{
    if (track == NULL) return;
    track_free(track);

    track->isParkingLot = false;
    track->offTrackSurfaceId = SURFACE_GRASS;
    track->runoffSurfaceId = SURFACE_GRASS;
    snprintf(track->id, sizeof(track->id), "%s", "chicane");
    snprintf(track->version, sizeof(track->version), "%s", "chicane_v1");

    /* Generous upper bound; the builder stops at capacity and count is what is used. */
    const int capacity = 512;
    track->nodes = (TrackNode *)calloc((size_t)capacity, sizeof(TrackNode));
    if (track->nodes == NULL) return;
    NodeBuilder b = { track->nodes, 0, capacity };

    const float pi = 3.14159265358979323846f;
    const float halfX = CHICANE_STRAIGHT_HALF_X_M;
    const float radius = CHICANE_CURVE_RADIUS_M;
    const float farY = CHICANE_FAR_STRAIGHT_Y_M;

    /* 1. Near straight, travelling +X along y = 0. */
    for (float x = -halfX; x < halfX - 0.5f * CHICANE_NODE_SPACING_M;
         x += CHICANE_NODE_SPACING_M) {
        builder_push(&b, x, 0.0f, TRACK_HALF_WIDTH_M, TRACK_RUNOFF_HALF_M);
    }

    /* 2. Right curve: centre (halfX, radius), sweeping -90 to +90 degrees, so the car turns
     * left through 180 degrees and comes back along the far straight. */
    {
        const int steps = (int)((pi * radius) / CHICANE_NODE_SPACING_M);
        for (int i = 0; i < steps; i++) {
            const float theta = -0.5f * pi + pi * ((float)i / (float)steps);
            builder_push(&b, halfX + radius * cosf(theta), radius + radius * sinf(theta),
                         TRACK_HALF_WIDTH_M, TRACK_RUNOFF_HALF_M);
        }
    }

    /* 3. Far straight, travelling -X along y = farY, with the chicane set into it. */
    for (float x = halfX; x > -halfX + 0.5f * CHICANE_NODE_SPACING_M;
         x -= CHICANE_NODE_SPACING_M) {
        const float intoChicane = CHICANE_ENTRY_X_M - x; /* grows as the car travels -X */
        const bool inChicane = (intoChicane >= 0.0f && intoChicane <= CHICANE_SPAN_M);
        if (inChicane) {
            const float u = intoChicane / CHICANE_SPAN_M;
            builder_push(&b, x, farY + chicane_offset_at(u), CHICANE_HALF_WIDTH_M,
                         CHICANE_RUNOFF_HALF_M);
        } else {
            builder_push(&b, x, farY, TRACK_HALF_WIDTH_M, TRACK_RUNOFF_HALF_M);
        }
    }

    /* 4. Left curve: centre (-halfX, radius), sweeping +90 to +270 degrees, closing the loop
     * back onto the start of the near straight. */
    {
        const int steps = (int)((pi * radius) / CHICANE_NODE_SPACING_M);
        for (int i = 0; i < steps; i++) {
            const float theta = 0.5f * pi + pi * ((float)i / (float)steps);
            builder_push(&b, -halfX + radius * cosf(theta), radius + radius * sinf(theta),
                         TRACK_HALF_WIDTH_M, TRACK_RUNOFF_HALF_M);
        }
    }

    track->count = b.count;

    /*
     * Eight required gates. Deliberately far fewer than there are nodes: gates exist to prove
     * the car went the right way round, and one per node would make a lap fail for a
     * momentary wide line rather than for cutting the course.
     */
    const int gateCount = 8;
    track->checkpoints = (Checkpoint *)calloc((size_t)gateCount, sizeof(Checkpoint));
    if (track->checkpoints == NULL) return;
    track->checkpointCount = gateCount;

    const float chicaneApexX = CHICANE_ENTRY_X_M - 0.5f * CHICANE_SPAN_M;
    const Checkpoint gates[8] = {
        /* 0: start/finish, on the near straight, facing +X */
        { { -60.0f, 0.0f }, { 1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },
        /* 1: end of the near straight, before turn-in */
        { { halfX, 0.0f }, { 1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },
        /* 2: right curve apex, facing +Y */
        { { halfX + radius, radius }, { 0.0f, 1.0f }, GATE_HALF_WIDTH_M, true },
        /* 3: far straight entry, facing -X */
        { { halfX, farY }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },
        /* 4: chicane entry */
        { { CHICANE_ENTRY_X_M, farY }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },
        /* 5: chicane apex, at the peak of the displacement */
        { { chicaneApexX, farY + CHICANE_OFFSET_M }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },
        /* 6: chicane exit */
        { { CHICANE_ENTRY_X_M - CHICANE_SPAN_M, farY },
          { -1.0f, 0.0f },
          GATE_HALF_WIDTH_M,
          true },
        /* 7: left curve apex, facing -Y */
        { { -halfX - radius, radius }, { 0.0f, -1.0f }, GATE_HALF_WIDTH_M, true },
    };
    memcpy(track->checkpoints, gates, sizeof(gates));

    apply_racing_line_controls(track, chicaneRacingLineOffsetsM, RACING_LINE_CONTROL_COUNT);
    track_reset_progress(track);
}
void track_load_sprint(Track *track)
{
    if (track == NULL) return;

    /* Start from the authored chicane, then apply a fixed affine layout change to both the
     * centreline and its gates. This keeps the route contract identical while exercising AI
     * geometry following on a genuinely different footprint. */
    track_load_chicane(track);
    const float scaleX = 0.82f;
    const float scaleY = 0.88f;
    for (int i = 0; i < track->count; i++) {
        track->nodes[i].centerM.x *= scaleX;
        track->nodes[i].centerM.y *= scaleY;
    }
    for (int i = 0; i < track->checkpointCount; i++) {
        track->checkpoints[i].centerM.x *= scaleX;
        track->checkpoints[i].centerM.y *= scaleY;
        const Vector2 forward = track->checkpoints[i].forwardUnit;
        const float length = sqrtf((forward.x * scaleX) * (forward.x * scaleX) +
                                   (forward.y * scaleY) * (forward.y * scaleY));
        if (length > 1e-12f) {
            track->checkpoints[i].forwardUnit =
                (Vector2){ forward.x * scaleX / length, forward.y * scaleY / length };
        }
    }
    snprintf(track->id, sizeof(track->id), "%s", "sprint");
    snprintf(track->version, sizeof(track->version), "%s", "sprint_v1");
    apply_racing_line_controls(track, sprintRacingLineOffsetsM, RACING_LINE_CONTROL_COUNT);
    track_reset_progress(track);
}
void track_load_technical(Track *track)
{
    if (track == NULL) return;

    /* The technical circuit keeps the authored route contract but compresses both axes, adds a
     * small affine skew, and narrows every ribbon. The result has materially tighter curves and
     * less recovery room than sprint_v1 while remaining a closed, collision-testable circuit. */
    track_load_chicane(track);
    const float scaleX = 0.62f;
    const float scaleY = 0.58f;
    const float skewX = 0.08f;
    const float skewY = 0.05f;

    for (int i = 0; i < track->count; i++) {
        TrackNode *node = &track->nodes[i];
        const Vector2 source = node->centerM;
        node->centerM = (Vector2){ scaleX * source.x + skewX * source.y,
                                   skewY * source.x + scaleY * source.y };
        const bool wasChicane = node->halfWidthM <= CHICANE_HALF_WIDTH_M;
        node->halfWidthM = wasChicane ? 4.5f : 5.8f;
        node->runoffHalfWidthM = node->halfWidthM + (wasChicane ? 2.0f : 2.5f);
    }
    for (int i = 0; i < track->checkpointCount; i++) {
        Checkpoint *checkpoint = &track->checkpoints[i];
        const Vector2 source = checkpoint->centerM;
        checkpoint->centerM = (Vector2){ scaleX * source.x + skewX * source.y,
                                         skewY * source.x + scaleY * source.y };
        const Vector2 forward = checkpoint->forwardUnit;
        const Vector2 transformed = { scaleX * forward.x + skewX * forward.y,
                                      skewY * forward.x + scaleY * forward.y };
        const float length =
            sqrtf(transformed.x * transformed.x + transformed.y * transformed.y);
        if (length > 1.0e-12f) {
            checkpoint->forwardUnit =
                (Vector2){ transformed.x / length, transformed.y / length };
        }
        checkpoint->halfWidthM = 7.0f;
    }
    snprintf(track->id, sizeof(track->id), "%s", "technical");
    snprintf(track->version, sizeof(track->version), "%s", "technical_v1");
    apply_racing_line_controls(track, technicalRacingLineOffsetsM, RACING_LINE_CONTROL_COUNT);
    track_reset_progress(track);
}

bool track_build_checkpoints_from_nodes(Track *track)
{
    if (track == NULL || track->nodes == NULL || track->count <= 0) return false;

    free(track->checkpoints);
    track->checkpoints = (Checkpoint *)calloc((size_t)track->count, sizeof(Checkpoint));
    if (track->checkpoints == NULL) {
        track->checkpointCount = 0;
        return false;
    }
    track->checkpointCount = track->count;

    for (int i = 0; i < track->count; i++) {
        const TrackNode *node = &track->nodes[i];
        const TrackNode *next = &track->nodes[(i + 1) % track->count];
        const float dx = next->centerM.x - node->centerM.x;
        const float dy = next->centerM.y - node->centerM.y;
        const float len = sqrtf(dx * dx + dy * dy);

        track->checkpoints[i].centerM = node->centerM;
        track->checkpoints[i].halfWidthM = node->halfWidthM;
        /* A hand-built ribbon has no learned line; its target path is the centreline it
         * authored, which is what every consumer of racingLineM must be able to assume. */
        track->nodes[i].racingLineM = node->centerM;
        track->checkpoints[i].required = true;
        /* A duplicated closing node leaves no direction to face; such a gate can never be
         * crossed, which is the same as the pre-existing behaviour for that degenerate case. */
        track->checkpoints[i].forwardUnit =
            (len < 1e-12f) ? (Vector2){ 0.0f, 0.0f } : (Vector2){ dx / len, dy / len };
    }
    return true;
}

void track_reset_progress_at(Track *track, int startCheckpointIndex)
{
    if (track == NULL) return;
    track->lap = 0;
    track->lapTimerS = 0.0f;
    track->lastLapTimeS = 0.0f;
    if (track->checkpointCount <= 0) {
        track->nextCheckpoint = 0;
        track->lapStartCheckpoint = 0;
        return;
    }
    if (startCheckpointIndex < 0 || startCheckpointIndex >= track->checkpointCount)
        startCheckpointIndex = 0;
    track->lapStartCheckpoint = startCheckpointIndex;
    track->nextCheckpoint = (startCheckpointIndex + 1) % track->checkpointCount;
}

void track_reset_progress(Track *track)
{
    track_reset_progress_at(track, 0);
}

bool track_start_pose_at(const Track *track, int checkpointIndex, Vector2 *positionM,
                         float *headingRad)
{
    if (track == NULL || track->checkpoints == NULL || track->checkpointCount <= 0)
        return false;
    if (checkpointIndex < 0 || checkpointIndex >= track->checkpointCount) return false;
    const Checkpoint *start = &track->checkpoints[checkpointIndex];
    if (positionM != NULL) *positionM = start->centerM;
    if (headingRad != NULL) *headingRad = atan2f(start->forwardUnit.y, start->forwardUnit.x);
    return true;
}

bool track_start_pose(const Track *track, Vector2 *positionM, float *headingRad)
{
    return track_start_pose_at(track, 0, positionM, headingRad);
}

static uint32_t hash_f32(uint32_t h, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    const unsigned char *bytes = (const unsigned char *)&bits;
    for (size_t i = 0; i < sizeof(bits); i++) {
        h ^= (uint32_t)bytes[i];
        h *= FNV1A_PRIME;
    }
    return h;
}

uint32_t track_geometry_hash(const Track *track)
{
    if (track == NULL) return 0u;
    uint32_t h = FNV1A_OFFSET_BASIS;
    for (int i = 0; i < track->count; i++) {
        const TrackNode *n = &track->nodes[i];
        h = hash_f32(h, n->centerM.x);
        h = hash_f32(h, n->centerM.y);
        h = hash_f32(h, n->racingLineM.x);
        h = hash_f32(h, n->racingLineM.y);
        h = hash_f32(h, n->halfWidthM);
        h = hash_f32(h, n->runoffHalfWidthM);
        h = hash_f32(h, (float)n->surfaceId);
    }
    for (int i = 0; i < track->checkpointCount; i++) {
        const Checkpoint *c = &track->checkpoints[i];
        h = hash_f32(h, c->centerM.x);
        h = hash_f32(h, c->centerM.y);
        h = hash_f32(h, c->forwardUnit.x);
        h = hash_f32(h, c->forwardUnit.y);
        h = hash_f32(h, c->halfWidthM);
        h = hash_f32(h, c->required ? 1.0f : 0.0f);
    }
    return h;
}

float track_length_m(const Track *track)
{
    if (track == NULL || track->nodes == NULL || track->count <= 1) return 0.0f;
    float total = 0.0f;
    for (int i = 0; i < track->count; i++) {
        const Vector2 a = track->nodes[i].centerM;
        const Vector2 b = track->nodes[(i + 1) % track->count].centerM;
        total += sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
    }
    return total;
}

/* Did the car's prev->curr motion pass through this gate, travelling the right way? */
static bool gate_crossed(const Checkpoint *gate, Vector2 prevPosM, Vector2 currPosM)
{
    const Vector2 perp = { -gate->forwardUnit.y, gate->forwardUnit.x };
    const Vector2 gateA = { gate->centerM.x + perp.x * gate->halfWidthM,
                            gate->centerM.y + perp.y * gate->halfWidthM };
    const Vector2 gateB = { gate->centerM.x - perp.x * gate->halfWidthM,
                            gate->centerM.y - perp.y * gate->halfWidthM };

    /* Forward-only: the motion must have a positive component along the gate's forward
     * direction, so reversing back over a line cannot score it. */
    const Vector2 motion = { currPosM.x - prevPosM.x, currPosM.y - prevPosM.y };
    if (motion.x * motion.x + motion.y * motion.y < 1e-24f) return false;
    if (motion.x * gate->forwardUnit.x + motion.y * gate->forwardUnit.y <= 0.0f) return false;

    return segments_intersect(prevPosM, currPosM, gateA, gateB);
}

TrackCheckpointEvent track_update_checkpoints(Track *track, Vector2 prevPosM, Vector2 currPosM)
{
    TrackCheckpointEvent event;
    memset(&event, 0, sizeof(event));
    event.index = -1;

    if (track == NULL || track->checkpoints == NULL || track->checkpointCount <= 0)
        return event;

    const int expected = track->nextCheckpoint;

    /*
     * Test the expected gate first, then every other one. Order matters: a car doing the
     * right thing must be recorded as in-order even in the rare tick where its motion segment
     * also clips a neighbouring gate, and checking the expected gate first guarantees that.
     * Testing the others at all is what makes a cut course visible — the old scheme only ever
     * looked at the expected gate, so shortcutting simply failed to advance and was
     * indistinguishable from not having reached the gate yet.
     */
    if (expected >= 0 && expected < track->checkpointCount &&
        gate_crossed(&track->checkpoints[expected], prevPosM, currPosM)) {
        event.crossed = true;
        event.index = expected;
    } else {
        for (int i = 0; i < track->checkpointCount; i++) {
            if (i == expected) continue;
            if (gate_crossed(&track->checkpoints[i], prevPosM, currPosM)) {
                event.crossed = true;
                event.index = i;
                event.outOfOrder = true;
                break;
            }
        }
    }

    if (!event.crossed || event.outOfOrder) return event;

    /* Only the expected gate advances progress. */
    track->nextCheckpoint++;
    if (track->nextCheckpoint >= track->checkpointCount) {
        track->nextCheckpoint = 0;
    }

    /* A lap closes when the ordered route returns to the gate where this run started. */
    const int lapCloseNext = (track->lapStartCheckpoint + 1) % track->checkpointCount;
    if (track->nextCheckpoint == lapCloseNext || track->checkpointCount == 1) {
        track->lap++;
        event.lapCompleted = true;
        event.lapTimeS = track->lapTimerS;
        track->lastLapTimeS = track->lapTimerS;
        track->lapTimerS = 0.0f;
    }

    return event;
}
